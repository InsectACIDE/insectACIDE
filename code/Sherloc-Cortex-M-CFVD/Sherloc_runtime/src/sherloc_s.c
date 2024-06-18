#include "../inc/sherloc_s.h"

static char msg[MAXMSG];
volatile uint32_t mtb_buff[1024] __attribute__((section(".ARM.__at_0x24000000")));
static ST STACK[MAXTASK];

static TASK_ST TASK_LIST;
uint32_t finish_time = 0;

static uint32_t in = 0, out = 0;
volatile uint32_t t_ibt_query = 0, t_ibt_query_start = 0, t_ibt_query_end = 0;
static int IRQ_count = 0, NESTED_IRQ_count = 0, PendSV_count = 0, test_count = 0;
uint32_t task_flag = 0; // a flag to show if it is an idirect call, 0: direct, 1: bx/blx, 2: pop, 3: pc changes
                        // if it is an PendSV entry, push special value 0xDEADBEEF

static uint32_t trace_data_buff[TRACE_BUFF_SIZE * 2]; // InsectACIDE: ring buffer for trace data
static uint32_t trace_data_buff_begin = 0;            // InsectACIDE: buffer begin
static uint32_t trace_data_buff_end = 0;              // InsectACIDE: buffer end

static void task_stack_init()
{
    for (size_t i = 0; i < MAXTASK; i++)
    {
        memset(&STACK[i], 0, sizeof(ST));
        STACK[task_flag].top_pos = STACK_FULL;
        STACK[task_flag].top = STACK_EMPTY;
    }
}

static void task_list_init()
{
    memset(&TASK_LIST, 0, sizeof(TASK_ST));
    TASK_LIST.num = 0;
}

#if defined(TRIGGER) || defined(SS_IRQ) || defined(INS_IDENTIFY_IRQ) || defined(READ_IRQ) || defined(USE_SYSTICK_NS)
void printf_int(uint32_t num) __attribute__((cmse_nonsecure_entry));
void printf_int(uint32_t num)
{
    printf("print uint32: %p\r\n", num);
}
#endif

int32_t time_measurement_s(uint32_t time) __attribute__((cmse_nonsecure_entry));
int32_t time_measurement_s(uint32_t time)
{
    uint32_t time_tmp = 0;
    finish_time++;

    // disable MTB
    MTB->MASTER &= ~(1 << 31);
    MTB->FLOW = 0;
    // clear haltreq bit
    MTB->MASTER &= ~(1 << 9);

    __asm volatile("dsb\n"
                   "isb\n");

    time_tmp = time;
    // sherloc_detection is the detection time for running SHERLOC, depends on the macro, it can be ss, buffer_read, or ins identification

    // enable MTB  0xFC3

    MTB->FLOW = WATERMARK_VALUE;


    // if defined IRQ, enable non-secure systick interrupt
    enable_SysTick_ns();


    MTB->MASTER |= (1 << 31);
    DWT->CYCCNT = 0;
    __asm volatile("dsb\n"
                   "isb\n");

    return 0;
}

volatile void *TB __attribute__((section(".ARM.__at_0x30000100")));
volatile BtHeader *pstBt = (BtHeader *)&TB;

int LoadBranchTable()
{
    printf("branch table size: %d, IRQ size: %d, Task size: %d, LOOP size: %d, tag_size: %d\r\n", pstBt->IBT_size, pstBt->IRQ_size, pstBt->TASK_size, pstBt->LOOP_size, pstBt->INS_tag_size);
    pstBt->IBT_entry = (uint32_t *)&TB + IBT_OFFSET;
    pstBt->IRQ_entry = (uint32_t *)&TB + IBT_OFFSET + pstBt->IBT_size;
    pstBt->TASK_entry = (uint32_t *)&TB + IBT_OFFSET + pstBt->IBT_size + pstBt->IRQ_size;
    pstBt->LOOP_entry = (uint32_t *)&TB + IBT_OFFSET + pstBt->IBT_size + pstBt->IRQ_size + pstBt->TASK_size * 2;

    printf("\r\n----Load branch table done.---\r\n");
    return 0;
}


ALWAYS_INLINE int IRQ_QUERY(uint32_t record)
{
    return binary_search(record, pstBt->IRQ_entry, pstBt->IRQ_size - 1);
}

/*----------------------------------- TASK_ID ----------------------------------*
 * take an address as input, check whether it is in the task list
 *--------------------------------------------------------------------------------*/

ALWAYS_INLINE int TASK_QUERY(uint32_t record)
{
    int mid = 0;
    int l = 0;
    int r = pstBt->TASK_size * 2;

    while (l <= r)
    {
        mid = l + (r - l) / 2;
        if (record == pstBt->TASK_entry[mid])
        {
            return mid;
        }
        else if (record < pstBt->TASK_entry[mid])
            r = mid - 1;
        else
            l = mid + 1;
    }
    return -1;
}

/*----------------------------------- CS_TASK_ID ----------------------------------*
 * take an address as input, return which task it belongs to using the knowlegde of fixed-priority scheduling
 TODO, please modify this according to the selected scheduling policy.
 *--------------------------------------------------------------------------------*/

ALWAYS_INLINE int CS_TASK_ID(uint32_t record, uint32_t t_flag)
{
    

    task_flag = 0;
    for (uint32_t i = 0; i < pstBt->TASK_size * 2; i += 2)
    {
        if ((record >= pstBt->TASK_entry[i]) && (record <= pstBt->TASK_entry[i + 1]) && (i % 2 == 0))
        {
            // 0 is the main stack for kernel
            task_flag = i / 2 + 1;

        }
    }
    if (task_flag != 0)
    {
        // printf("task_flag error\r\n");
        task_flag = t_flag;
    }

    return task_flag;
}

/*----------------------------------- TASK_ID ----------------------------------*
 * take an address as input, return which task it belongs to
 *--------------------------------------------------------------------------------*/

ALWAYS_INLINE int TASK_ID(uint32_t record)
{
    task_flag = 0;
    for (uint32_t i = 0; i < pstBt->TASK_size * 2; i += 2)
    {
        if ((record >= pstBt->TASK_entry[i]) && (record <= pstBt->TASK_entry[i + 1]) && (i % 2 == 0))
        {
            // 0 is the main stack for kernel
            task_flag = i / 2 + 1;
        }
    }
    return task_flag;
}

/*----------------------------------- IBT_query ----------------------------------*
 * take the address pair as input, binary search the branch table,
 * if find the branch, return true, else return false
 *--------------------------------------------------------------------------------*/

ALWAYS_INLINE int IBT_QUERY(uint32_t src, uint32_t dst)
{
    // if the binary serach find the branch, return true, else return false
    int r = binary_search(CONCACT_UINT32(src, dst), pstBt->IBT_entry, pstBt->IBT_size - 1);
    if (r > -1)
    {
    }
    else
    {
        // printf("IBT error\r\n");
    }
    return 1;
}

// context-sensitive IBQ query
ALWAYS_INLINE int CS_IBT_QUERY(uint32_t src, uint32_t dst, ST *stack)
{
    uint32_t encode_src = src;
    uint32_t encode_dst = dst;
    for (int i = 0; i < CONTEXT_LEVEL; i++)
    {
        if (stack->top_pos > i)
        {
            uint32_t src_tmp = stack->stk[stack->top_pos - i];
            encode_src = encode_src ^ src_tmp;
        }
    }

    // if the binary serach find the branch, return true, else return false
    int r = binary_search(CONCACT_UINT32(encode_src, encode_dst), pstBt->IBT_entry, pstBt->IBT_size - 1);
    if (r > -1)
    {
    }
    else
    {
        // printf("IBT--error\r\n");
        // TODO: angr tools can be improcised to produce sound analysis result
    }
    return 1;
}

/* IRQ_in hase two cases: normal IRQ and PendSV
    Normal IRQ: store interrupted position
    PendSV: push TASK_FLAG (0xaaaabbbb)
*/

ALWAYS_INLINE int check_IRQ_in(uint32_t src_addr, uint32_t dst_addr)
{
    // if it is an IRQ in, push the interrupted address into the STACK
    if (IRQ_QUERY(dst_addr) > -1)
    {
        if (STACK_push(&STACK[task_flag], src_addr & LAST_BIT_CLEAR) < 0)
        {
            printf("[IRQ] stack is full!! top: %d\r\n", STACK[task_flag].top_pos);
            STACK_check_all(&STACK[task_flag]);
            dummy_handler();
            return -1;
        }

        IRQ_count++;
        return IRQ_IN;
    }

    else if (dst_addr == pstBt->PendSV)
    {
        // store the interrupted task by pendsv
        if (TASK_insert(&TASK_LIST, src_addr & LAST_BIT_CLEAR) < 0) // InsectACIDE: should not insert a new task
        {
            printf("[PendSV] task list is full! num: %d.\r\n", TASK_LIST.num);
            return -1;
        }

        // store the previous tasks to task list and clear it from current stack
        // we need to check if current stack has a value, if it has, store to the task list
        if (TASK_insert(&TASK_LIST, STACK_pop(&STACK[task_flag])) < 0)
        {
            printf("[PendSV] task list is full!\r\n");
            return -1;
        }


        IRQ_count++;
        PendSV_count++;
        return PendSV_IN;
    }


    return -1;
}



/*
IRQ_out has three cases:
Normal IRQ_OUT: check if the returned position is the interrupted one
PendSV: assume new task, just pop TASK_FLAG
Nested IRQ_IN: check if the the returned position is another IRQ entry
*/

ALWAYS_INLINE int check_IRQ_out(uint32_t src_addr, uint32_t dst_addr, int idx)
{
    // we only check the exit that target to the non-secure state
    uint32_t top = STACK[task_flag].top;
    // if it is a normal IRQ out after existing Debug monitor handler


    if (dst_addr == top)
    {
        STACK_pop(&STACK[task_flag]);

        return IRQ_OUT;
    }

    // nested non-secure IRQ


    if (IRQ_QUERY(dst_addr) > -1)
    {

        IRQ_count++;
        NESTED_IRQ_count++;
        return NESTED_IRQ_IN;
    }

    // task scheduling exit

    // task schedule in
    if (dst_addr == pstBt->PendSV)
    {
        // store the interrupted pendsv tasks
        if (TASK_insert(&TASK_LIST, STACK_pop(&STACK[task_flag])) < 0)
        {
            printf("[PendSV] task list is full!\r\n");
            return -1;
        }
        // if the current stack is not empty, pop it and store to the task list
        if (TASK_insert(&TASK_LIST, STACK_pop(&STACK[task_flag])) < 0)
        {
            printf("[PendSV] task list is full!\r\n");
            return -1;
        }



        IRQ_count++;
        // NESTED_PendSV_count++;
        PendSV_count++;
        return PendSV_IN;
    }
    // if the target address is another task
    else if (TASK_check(&TASK_LIST, dst_addr) > -1)
    {
        TASK_ID(dst_addr);



        return PendSV_OUT;
    }
    // PendSV may return to an address that does not saved in the stack, which can be a new scheduled task
    else if (TASK_QUERY(dst_addr) > -1)
    {
        // check if it is an idle task and it is already inside the task list. Then we ignore it.
        if (binary_search(dst_addr, pstBt->LOOP_entry, pstBt->LOOP_size - 1))
        {
        }
        // Or we store the interrupted pendsv tasks
        else if (TASK_insert(&TASK_LIST, STACK_pop(&STACK[task_flag])) < 0)
        {
            printf("[PendSV] task list is full!\r\n");
            return -1;
        }
        // we need to check if current stack has a value, if it has, store to the task list
        if (TASK_insert(&TASK_LIST, STACK_pop(&STACK[task_flag])) < 0)
        {
            printf("[PendSV] task list is full!\r\n");
            return -1;
        }




        TASK_ID(dst_addr);
        STACK[task_flag].top_pos = STACK_FULL;




        return PendSV_OUT;
    }

    // printf("IRQ_out error\r\n");
    return 0; // TODO, fix exception handling
    return -1;
}


/*----------------------------------- branch filtering -------------------------------------
Instruction	Decode					Hex
B			0b1110 0xxx xxxx xxxx 	0xexxx
BL			0b1111 0xxx xxxx xxxx	0xfxxx
BLX/BLXNS	0b0100 0111 1xxx xxxx	0x47xx
BX/BXNS		0b0100 0111 0xxx xxxx	0x47xx
*--------------------------------------------------------------------------------
[+] conditional branch:
         |4   |4		   | 8	     |4		  |	12		|
T1 	B: 0b1101 cond != 111x imm8								2-byte:	0b11010000~0b11011111 0xd0~0xdf
T2	B: 0b1110 0 		imm11								2-byte: 0b11100000~0b11100111 0xe0~0xe7
T3	B: 0b1111 0 S cond != 111x imm6  1 0 J1 0 J2 imm11
4-byte:	0b11110011 10000000 10000000 00000000~0b11110111 11111111 10101111 11111111 0xf380 8000~0xf7ff afff
T4	B: 0b1111 0 S 		imm10	     1 0 J1 1 J2 imm11
4-byte: 0b11110000 00000000 10010000 00000000~0b11110111 11111111 10111111 11111111 0xf000 9000~0xf7ff bfff
    conditions:
        0000 EQ
        0001 NE
        0010 CS
        0011 CC
        0100 MI
        0101 PL
        0110 VS
        0111 VC
        1000 HI
        1001 LS
        1010 GE
        1011 LT
        1100 GT
        1101 LE
        1110 None (AL)
CBNZ: 	0b1011 10i1		0xb9~0xbb
CBZ:	0b1011 00i1		0xb1~0xb3
B
*--------------------------------------------------------------------------------
[+] Branches and miscellaneous control
BL: 	0b1111 0
        0b1111 0 S imm10 11 J1 1 J2 imm11	0b11110000 00000000 11010000 00000000~0b11110111 11111111 11111111 11111111 0xf000 d000~0xf7ff ffff
f7ff 1111 0111 1111 1111 bl 0xxx
BLX: 	0b0100 01 11 1
BX:		0b0100 01 11 0
4770: 0100 0111 0111 0000 bx lr
4788: 0100 0111 1000 1000 blx r1

*--------------------------------------------------------------------------------
[+] Return
    BX LR: 0x4770
    POP {..., pc}: 0xbd
*--------------------------------------------------------------------------------
[+] Interruption in:
    <any, any>
*--------------------------------------------------------------------------------
[+] Interruption out
    BX EXC_RETURN (0xFFFFFF00~0xFFFFFFFF)
EXC_RETURN Value	Mode to Return To	Stack to use
0xFFFFFFF1	Handler Mode	MSP
0xFFFFFFF9	Thread Mode	MSP
0xFFFFFFFD	Thread Mode	PSP
0xFFFFFFE1	Handler Mode (FPU Extended Frame)	MSP
0xFFFFFFE9	Thread Mode (FPU Extended Frame)	MSP
0xFFFFFFED	Thread Mode (FPU Extended Frame)	PSP
*--------------------------------------------------------------------------------
TBB:	0b1110 1000 1101 xxxx	0xe8d0~0xe8df
e8df f001 	tbb	[pc, r1]
*/
uint32_t process_preempted = 0;
void processing_trace_buff(uint32_t is_idle)
{
    uint32_t record_count = MTB->POSITION;
    uint32_t src_addr, dst_addr;
    MTB->POSITION = 0;

    for (int idx = 0; idx < record_count / 4; idx += 2)
    {
        if (is_idle)
        {
        }

        // check non-secure branches, code region: [0x00200000~0x00400000]
        src_addr = mtb_buff[idx];
        dst_addr = mtb_buff[idx + 1] & LAST_BIT_CLEAR;
        trace_data_buff[trace_data_buff_end % TRACE_BUFF_SIZE] = src_addr;
        trace_data_buff_end++;
        trace_data_buff[trace_data_buff_end % TRACE_BUFF_SIZE] = dst_addr;
        trace_data_buff_end++;
    }
    process_preempted = 0;
    if (is_idle)
    {
        enable_SysTick_ns();
    }
    uint32_t idf = 0;
    // printf("before process trace_data : %u - %u = %u.\r\n",  trace_data_buff_end,trace_data_buff_begin, trace_data_buff_end-trace_data_buff_begin);
    for (; trace_data_buff_begin < trace_data_buff_end; trace_data_buff_begin += 2)
    {
        idf++;
        if (is_idle && idf % 0x50 == 0)
        { // secure idle task.

            {

                uint32_t ctr_tick = SysTick_NS->CTRL;
                if (ctr_tick & SysTick_CTRL_COUNTFLAG_Msk)
                {
                    printf("tick %p - %u\r\n", ctr_tick, idf);
                    MTB->MASTER |= (1 << 31);
                    process_preempted = 1;
                    break;
                }
            }
        }

        int i = trace_data_buff_begin % TRACE_BUFF_SIZE;
        int idx = 0;

        src_addr = trace_data_buff[i];
        dst_addr = trace_data_buff[i + 1] & LAST_BIT_CLEAR;

        int bs = (src_addr < CODE_NS);
        int bd = (dst_addr < CODE_NS);

        if (bs != bd)
        {
            // printf("[%ld] Secure in: 0x%08lx, 0x%08lx\r\n", task_flag, src_addr, dst_addr);
        }

        if (src_addr < CODE_NS)
        {

            if (dst_addr < CODE_NS)
            {

                // if defined IRQ_ENABLE, filter return, those are other the dst should equal to the top of the stack
                if (dst_addr == STACK[task_flag].top)
                {
                    STACK_pop(&STACK[task_flag]);


                    continue;
                }

                if (src_addr & 0x1)
                {
                    if (check_IRQ_in(src_addr, dst_addr) > -1)
                    {
                        continue;
                    }
                }

                int t_flag = CS_TASK_check(&TASK_LIST, dst_addr, 1);

                // read bytes one by one to get the mc of instruction
                uint8_t *src_ins = (uint32_t *)src_addr;
                // filter direct branch (B: {d0000~e7ff}):
                if (src_ins[1] >= 0xd0 && src_ins[1] <= 0xe7)
                {
                    continue;
                }
                // filter CBNZ, CBZ {b100~bbff}
                else if (src_ins[1] >= 0xb1 && src_ins[1] <= 0xbb)
                {
                    continue;
                }
                // filter direct call (BL) and special direct branch with size of 4 bytes (conditional B: {f000 8000~f7ff bfff})
                else if ((src_ins[1] >= 0xf0) && (src_ins[1] <= 0xf7))
                {
                    if (src_ins[3] < 0xd0)
                    {
                        continue;
                    }



                    // bl
                    if (STACK_push(&STACK[task_flag], src_addr + 4) < 0)
                    {
                        printf("[%u]stack is full! top: %d\r\n", task_flag, STACK[task_flag].top_pos);
                        // STACK_check_all(&STACK[task_flag]);
                        dummy_handler();
                    }
                    continue;
                }
                // filter bx/blx
                else if (src_ins[1] == 0x47)
                {
                    // filter out bx lr at the very beginning
                    // the record is an indirect call/branch, check the IBT
                    // bx/blx Rx
                    if ((src_ins[0] != 0x70) && (CS_IBT_QUERY(src_addr, dst_addr, &STACK[task_flag]) < 0))
                    {
                        printf("[%ld]!!! illegal indirect call: 0x%08lx\r\n", task_flag, CONCACT_UINT32(src_addr, dst_addr));
                        STACK_check_all(&STACK[task_flag]);
                        TASK_check_all(&TASK_LIST);
                        dummy_handler();
                    }

                    // blx Rx, store the address of next instruction
                    if (src_ins[0] > 0x7f)
                    {



                        if (STACK_push(&STACK[task_flag], src_addr + 2) < 0)
                        {
                            printf("[%u]stack is full! top: %d\r\n", task_flag, STACK[task_flag].top_pos);
                            STACK_check_all(&STACK[task_flag]);
                            dummy_handler();
                        }
                        continue;
                    }
                    continue;
                }

                // it maybe a return from kernel functions, check the sorted task list
                // if the target address is another task
                // else if (TASK_check(&TASK_LIST,dst_addr) > -1)
                else if (CS_TASK_check(&TASK_LIST, dst_addr, 0) > -1)
                {
                    // int t_flag=CS_TASK_check(&TASK_LIST,0);
                    CS_TASK_ID(dst_addr, t_flag);
                    // TASK_ID(dst_addr);




                    continue;
                }

       // other records, check the IBT
                if (CS_IBT_QUERY(src_addr, dst_addr, &STACK[task_flag]) < 0)
                {

                    printf("[%ld]!!! illegal transfer.\r\n", task_flag);


                    dummy_handler();
                }
                continue;
            }

            // secure debugmon_handler in
            else if ((dst_addr > CODE_S) && (dst_addr < 0x10004000))
            {
                if (STACK_push(&STACK[task_flag], src_addr & LAST_BIT_CLEAR) < 0)
                {
                    printf("[%u]Secure_IRQ stack is full! top: %d\r\n", task_flag, STACK[task_flag].top_pos);
                    STACK_check_all(&STACK[task_flag]);
                    dummy_handler();
                }
                // printf("[%ld] Secure in: 0x%08lx, 0x%08lx\r\n", task_flag, src_addr, dst_addr);



                continue;
            }
            continue;
        }
        // Check IRQ out
        else if ((src_addr > IRQ_EXIT) && (dst_addr < CODE_NS))
        {

            if (check_IRQ_out(src_addr, dst_addr, idx) == -1)
            {
                for (int j = i - 8; j <= i; j += 2)
                {
                    printf("[%u] Wrong IRQ-exit: 0x%08x, 0x%08x, \r\n", task_flag, trace_data_buff[j], trace_data_buff[j + 1] & LAST_BIT_CLEAR);
                }

                printf("[%u] Wrong IRQ exit: 0x%08x, 0x%08x, \r\n", task_flag, src_addr, dst_addr);


                dummy_handler();
            }

            continue;
        }
    }
    if (is_idle)
    {
        disable_SysTick_ns();
    }
    printf("after processed buffer size %u - %u =%u \r\n", trace_data_buff_end, trace_data_buff_end, trace_data_buff_begin - trace_data_buff_begin);
}
void copy_to_trace_buff()
{
    uint32_t record_count = MTB->POSITION;
    uint32_t src_addr, dst_addr;
    MTB->POSITION = 0;
    if (record_count + trace_data_buff_end - trace_data_buff_begin > TRACE_BUFF_SIZE)
    {
        printf("The trace buffer is full, please increase the size of the buffer.\r\n");
        processing_trace_buff(0);
        return;
    }
    // printf("current buffer size %u\r\n",record_count+trace_data_buff_end-trace_data_buff_begin);
    // printf("display: %u.\r\n",  record_count);
    for (int idx = 0; idx < record_count / 4; idx += 2)
    {
        // check non-secure branches, code region: [0x00200000~0x00400000]
        src_addr = mtb_buff[idx];
        dst_addr = mtb_buff[idx + 1] & LAST_BIT_CLEAR;

        if (src_addr < CODE_NS)
        {
            if (dst_addr < CODE_NS)
            {

                if (src_addr & 0x1)
                {
                    trace_data_buff[trace_data_buff_end % TRACE_BUFF_SIZE] = src_addr;
                    trace_data_buff_end++;
                    trace_data_buff[trace_data_buff_end % TRACE_BUFF_SIZE] = dst_addr;
                    trace_data_buff_end++;

                    continue;
                }

                // read bytes one by one to get the mc of instruction
                uint8_t *src_ins = (uint32_t *)src_addr;
                // filter direct branch (B: {d0000~e7ff}):
                if (src_ins[1] >= 0xd0 && src_ins[1] <= 0xe7)
                {
                    continue;
                }
                // filter CBNZ, CBZ {b100~bbff}
                else if (src_ins[1] >= 0xb1 && src_ins[1] <= 0xbb)
                {
                    continue;
                }

                trace_data_buff[trace_data_buff_end % TRACE_BUFF_SIZE] = src_addr;
                trace_data_buff_end++;
                trace_data_buff[trace_data_buff_end % TRACE_BUFF_SIZE] = dst_addr;
                trace_data_buff_end++;
                continue;
            }

            // secure debugmon_handler in
            else if ((dst_addr > CODE_S) && (dst_addr < 0x10004000))
            {
                trace_data_buff[trace_data_buff_end % TRACE_BUFF_SIZE] = src_addr;
                trace_data_buff_end++;
                trace_data_buff[trace_data_buff_end % TRACE_BUFF_SIZE] = dst_addr;
                trace_data_buff_end++;
                continue;
            }
            continue;
        }
        // Check IRQ out
        else if ((src_addr > IRQ_EXIT) && (dst_addr < CODE_NS))
        {
            trace_data_buff[trace_data_buff_end % TRACE_BUFF_SIZE] = src_addr;
            trace_data_buff_end++;
            trace_data_buff[trace_data_buff_end % TRACE_BUFF_SIZE] = dst_addr;
            trace_data_buff_end++;
            continue;
        }

        // src_addr > S, the record is from the Secure state, we consider it is trusted
    }
}



void mtb_reconfig()
{
    MTB->FLOW = WATERMARK_VALUE;
    //  const int mask = __builtin_ctz(MTB_ALIAS_MAX) - 4;
    //   MTB->MASTER = (1 << 31) | (mask << 0);
    MTB->MASTER |= (1 << 31);
}

int isDWTmatched()
{
    // uint32_t matched_dwt0=DWT->FUNCTION0;
    if (DWT->FUNCTION0 & DWT_FUNCTION_MATCHED_Msk)
    {
        // printf("DWT000 %p: %p.\r\n",matched_dwt0,DWT->FUNCTION0);
        return 0;
    }
    else if (DWT->FUNCTION1 & DWT_FUNCTION_MATCHED_Msk)
    {
        return 1;
    }
    else if (DWT->FUNCTION2 & DWT_FUNCTION_MATCHED_Msk)
    {
        return 2;
    }
    else if (DWT->FUNCTION3 & DWT_FUNCTION_MATCHED_Msk)
    {
        return 3;
    }
    else
    {
        return -1;
    }
}

#define MAX_METADATA 0x100

#define IdleTASK_IDX 0
#define Output_IDX (IdleTASK_IDX + 1)

#define MemManage_Handler_IDX (Output_IDX + 1)
#define MemManage_Handler_2_IDX (MemManage_Handler_IDX + 1)

#define CODE_START_ADDR_IDX (MemManage_Handler_2_IDX + 1)
#define CODE_END_ADDR_IDX (CODE_START_ADDR_IDX + 1)

#define SANDBOX_START_ADDR_IDX (CODE_END_ADDR_IDX + 1)
#define SANDBOX_PAIR_SIZE 0x10
#define SANDBOX_END_ADDR_IDX (SANDBOX_START_ADDR_IDX + SANDBOX_PAIR_SIZE * 2)
/*----------------From portmacro.h--------------------------*/

#define portPRIVILEGED_FLASH_REGION (0UL)
#define portUNPRIVILEGED_FLASH_REGION (1UL)
#define portUNPRIVILEGED_SYSCALLS_REGION (2UL)
#define portPRIVILEGED_RAM_REGION (3UL)
#define portSTACK_REGION (4UL)
#define portFIRST_CONFIGURABLE_REGION (5UL)
// #define portLAST_CONFIGURABLE_REGION                  ( 7UL )
#define portLAST_CONFIGURABLE_REGION (6UL)
#define portNUM_CONFIGURABLE_REGIONS ((portLAST_CONFIGURABLE_REGION - portFIRST_CONFIGURABLE_REGION) + 1)
#define portTOTAL_NUM_REGIONS

/* Normal memory attributes used in MPU_MAIR registers. */
#define portMPU_NORMAL_MEMORY_NON_CACHEABLE (0x44)        /* Non-cacheable. */
#define portMPU_NORMAL_MEMORY_BUFFERABLE_CACHEABLE (0xFF) /* Non-Transient, Write-back, Read-Allocate and Write-Allocate. */

/* Attributes used in MPU_RBAR registers. */
#define portMPU_REGION_NON_SHAREABLE (0UL << 3UL)
#define portMPU_REGION_INNER_SHAREABLE (1UL << 3UL)
#define portMPU_REGION_OUTER_SHAREABLE (2UL << 3UL)

#define portMPU_REGION_PRIVILEGED_READ_WRITE (0UL << 1UL)
#define portMPU_REGION_READ_WRITE (1UL << 1UL)
#define portMPU_REGION_PRIVILEGED_READ_ONLY (2UL << 1UL)
#define portMPU_REGION_READ_ONLY (3UL << 1UL)

#define portMPU_REGION_EXECUTE_NEVER (1UL)

/**
 * @brief Constants required to manipulate the MPU.
 */
#define portMPU_TYPE_REG (*((volatile uint32_t *)0xe000ed90))
#define portMPU_CTRL_REG (*((volatile uint32_t *)0xe000ed94))
#define portMPU_RNR_REG (*((volatile uint32_t *)0xe000ed98))

#define portMPU_RBAR_REG (*((volatile uint32_t *)0xe000ed9c))
#define portMPU_RLAR_REG (*((volatile uint32_t *)0xe000eda0))

#define portMPU_RBAR_A1_REG (*((volatile uint32_t *)0xe000eda4))
#define portMPU_RLAR_A1_REG (*((volatile uint32_t *)0xe000eda8))

#define portMPU_RBAR_A2_REG (*((volatile uint32_t *)0xe000edac))
#define portMPU_RLAR_A2_REG (*((volatile uint32_t *)0xe000edb0))

#define portMPU_RBAR_A3_REG (*((volatile uint32_t *)0xe000edb4))
#define portMPU_RLAR_A3_REG (*((volatile uint32_t *)0xe000edb8))

#define portMPU_MAIR0_REG (*((volatile uint32_t *)0xe000edc0))
#define portMPU_MAIR1_REG (*((volatile uint32_t *)0xe000edc4))

#define portMPU_RBAR_ADDRESS_MASK (0xffffffe0) /* Must be 32-byte aligned. */
#define portMPU_RLAR_ADDRESS_MASK (0xffffffe0) /* Must be 32-byte aligned. */

#define portMPU_MAIR_ATTR0_POS (0UL)
#define portMPU_MAIR_ATTR0_MASK (0x000000ff)

#define portMPU_MAIR_ATTR1_POS (8UL)
#define portMPU_MAIR_ATTR1_MASK (0x0000ff00)

#define portMPU_MAIR_ATTR2_POS (16UL)
#define portMPU_MAIR_ATTR2_MASK (0x00ff0000)

#define portMPU_MAIR_ATTR3_POS (24UL)
#define portMPU_MAIR_ATTR3_MASK (0xff000000)

#define portMPU_MAIR_ATTR4_POS (0UL)
#define portMPU_MAIR_ATTR4_MASK (0x000000ff)

#define portMPU_MAIR_ATTR5_POS (8UL)
#define portMPU_MAIR_ATTR5_MASK (0x0000ff00)

#define portMPU_MAIR_ATTR6_POS (16UL)
#define portMPU_MAIR_ATTR6_MASK (0x00ff0000)

#define portMPU_MAIR_ATTR7_POS (24UL)
#define portMPU_MAIR_ATTR7_MASK (0xff000000)

#define portMPU_RLAR_ATTR_INDEX0 (0UL << 1UL)
#define portMPU_RLAR_ATTR_INDEX1 (1UL << 1UL)
#define portMPU_RLAR_ATTR_INDEX2 (2UL << 1UL)
#define portMPU_RLAR_ATTR_INDEX3 (3UL << 1UL)
#define portMPU_RLAR_ATTR_INDEX4 (4UL << 1UL)
#define portMPU_RLAR_ATTR_INDEX5 (5UL << 1UL)
#define portMPU_RLAR_ATTR_INDEX6 (6UL << 1UL)
#define portMPU_RLAR_ATTR_INDEX7 (7UL << 1UL)

#define portMPU_RLAR_REGION_ENABLE (1UL)
#define portSCB_MEM_FAULT_ENABLE_BIT (1UL << 16UL)
/* Enable privileged access to unmapped region. */
#define portMPU_PRIV_BACKGROUND_ENABLE_BIT (1UL << 2UL)

/* Enable MPU. */
#define portMPU_ENABLE_BIT (1UL << 0UL)

/* Expected value of the portMPU_TYPE register. */
#define portEXPECTED_MPU_TYPE_VALUE (8UL << 8UL) /* 8 regions, unified. */
/*-----------------------------------------------------------*/

/*-----------------------------------------------------------*/

#define NS_portMPU_RNR_REG (*((volatile uint32_t *)0xe002ed98))
#define NS_portMPU_RBAR_REG (*((volatile uint32_t *)0xe002ed9c))
#define NS_portMPU_RLAR_REG (*((volatile uint32_t *)0xe002eda0))
#define NS_portMPU_CTRL_REG (*((volatile uint32_t *)0xe002ed94))

#define NS_portSCB_SYS_HANDLER_CTRL_STATE_REG (*(volatile uint32_t *)0xe002ed24)
static void ns_configureMPU(uint32_t baseaddr, uint32_t endaddr, uint32_t permission, uint32_t region_num) __attribute__((noinline))
{

    if (region_num < portFIRST_CONFIGURABLE_REGION || region_num > 7)
    {
        // return;
    }
    NS_portMPU_CTRL_REG &= (~1);
    // uint32_t MAIR_attr_pos=(region_num-4)*8;

    /* MAIR1 - Index 5. */
    // portMPU_MAIR1_REG |= ( ( portMPU_NORMAL_MEMORY_BUFFERABLE_CACHEABLE << MAIR_attr_pos) & (0xff<<MAIR_attr_pos) );

    /* Setup privileged flash as Read Only so that privileged tasks can
     * read it but not modify. */

    NS_portMPU_RNR_REG = region_num; // region_num;
    NS_portMPU_RBAR_REG = (((uint32_t)baseaddr) & portMPU_RBAR_ADDRESS_MASK) |
                          (portMPU_REGION_NON_SHAREABLE) |
                          (permission);
    NS_portMPU_RLAR_REG = (((uint32_t)(endaddr)) & portMPU_RLAR_ADDRESS_MASK) |
                          (portMPU_RLAR_ATTR_INDEX0) |
                          (portMPU_RLAR_REGION_ENABLE);

    /* Setup unprivileged flash as Read Only by both privileged and
     * unprivileged tasks. All tasks can read it but no-one can modify. */
    // printf("configured MPU %d : %p - %p\r\n",region_num,NS_portMPU_RBAR_REG,NS_portMPU_RLAR_REG);

    /* Enable mem fault. */
    NS_portSCB_SYS_HANDLER_CTRL_STATE_REG |= portSCB_MEM_FAULT_ENABLE_BIT;

    /* Enable MPU with privileged background access i.e. unmapped
     * regions have privileged access. */
    NS_portMPU_CTRL_REG |= (portMPU_PRIV_BACKGROUND_ENABLE_BIT | portMPU_ENABLE_BIT);
    // portMPU_CTRL_REG |= (  portMPU_ENABLE_BIT );
}

uint32_t ns_metadata[MAX_METADATA];
uint32_t ns_metadata_idx = 0;

uint32_t sandbox_size = 0;
uint32_t sandbox_count_idx = 0;
uint32_t ns_mpu_configure[8 * 2];

uint32_t mpu_is_configured = 0;

#define MPU_SIZE 64
#define MPU2_SIZE 256
void sandbox_exit_detect(uint32_t range_begin, uint32_t range_end) __attribute__((noinline))
{

    NS_portMPU_CTRL_REG &= (~1);

    // for(int i=0;i<8;i++){
    //      NS_portMPU_RNR_REG = i ;//region_num;
    //      ns_mpu_configure[i*2]=NS_portMPU_RBAR_REG ;
    //      ns_mpu_configure[i*2+1]=NS_portMPU_RLAR_REG;
    //     }

    for (int i = 0; i < 3; i++)
    {
        NS_portMPU_RNR_REG = i; // region_num;
        //  printf("%d : %p - %p\r\n",i,NS_portMPU_RBAR_REG,NS_portMPU_RLAR_REG);
        NS_portMPU_RBAR_REG = 0;
        NS_portMPU_RLAR_REG = 0;
    }
   

    
    ns_configureMPU((ns_metadata[MemManage_Handler_IDX]), (ns_metadata[MemManage_Handler_IDX] + MPU_SIZE), portMPU_REGION_READ_ONLY, 0);
    ns_configureMPU((ns_metadata[MemManage_Handler_2_IDX]), (ns_metadata[MemManage_Handler_2_IDX] + MPU2_SIZE), portMPU_REGION_READ_ONLY, 1);

    ns_configureMPU(((uint32_t)(range_begin)), range_end + 32, portMPU_REGION_READ_ONLY, 2);
    // ns_configureMPU(((uint32_t)(range_end)+32), ns_metadata[CODE_END_ADDR_IDX], portMPU_REGION_READ_ONLY|portMPU_REGION_EXECUTE_NEVER,2); //|portMPU_REGION_EXECUTE_NEVER

  
    
    printf("----after MPU configuration\r\n");
    for (int i = 0; i < 8; i++)
    {
        NS_portMPU_RNR_REG = i; // region_num;
        printf("%d-th MPU : %p - %p\r\n", i, NS_portMPU_RBAR_REG, NS_portMPU_RLAR_REG);
    }

    NS_portMPU_CTRL_REG |= (portMPU_PRIV_BACKGROUND_ENABLE_BIT | portMPU_ENABLE_BIT);
}

void sandbox_recover() __attribute__((noinline))
{

    NS_portMPU_CTRL_REG &= (~1);

    if (mpu_is_configured)
    {

        for (int i = 0; i < 3; i++)
        {
            NS_portMPU_RNR_REG = i; // region_num;
            NS_portMPU_RBAR_REG = ns_mpu_configure[i * 2];
            NS_portMPU_RLAR_REG = ns_mpu_configure[i * 2 + 1];
        }
        for (int i = 0; i < 8; i++)
        {
            NS_portMPU_RNR_REG = i; // region_num;
            printf("recovered %d : %p - %p\r\n", i, NS_portMPU_RBAR_REG, NS_portMPU_RLAR_REG);
        }
    }
    else
    {
        printf("MPU recover fail.\r\n");
    }

    NS_portMPU_CTRL_REG |= (portMPU_PRIV_BACKGROUND_ENABLE_BIT | portMPU_ENABLE_BIT);
}



void sandbox_entry_detect_dwt(uint32_t range_begin, uint32_t range_end) __attribute__((noinline))
{

    printf(" sandbox_entry_detect_dwt: %p .\r\n", (range_begin));
    // printf(" sandbox_entry_detect_dwt: %p .\r\n",(range_begin+32)& portMPU_RBAR_ADDRESS_MASK);
    //    dwt_install_watchpoint(3, (range_begin+32)& portMPU_RBAR_ADDRESS_MASK, DWT_FUNCTION_INST);
    dwt_install_watchpoint(3, (range_begin), DWT_FUNCTION_INST);

    // ns_configureMPU(range_begin,range_end-range_begin, portMPU_REGION_READ_ONLY|portMPU_REGION_EXECUTE_NEVER,6);
    // ns_configureMPU(range_end,ns_metadata[CODE_END_ADDR_IDX]-range_end, portMPU_REGION_READ_ONLY|portMPU_REGION_EXECUTE_NEVER,7);
}

void sandbox_entry_detect_mpu(uint32_t range_begin, uint32_t range_end) __attribute__((noinline))
{

    for (int i = 0; i < 3; i++)
    {
        NS_portMPU_RNR_REG = i; // region_num;
        NS_portMPU_RBAR_REG = 0;
        NS_portMPU_RLAR_REG = 0;
    }

    ns_configureMPU(ns_metadata[CODE_START_ADDR_IDX], ((uint32_t)(range_begin + 32)), portMPU_REGION_READ_ONLY, 0); // TODO, extract only necessary part
    // ns_configureMPU((uint32_t*) range_begin,(uint32_t*) range_end, portMPU_REGION_READ_ONLY,1);
    ns_configureMPU(((uint32_t)(range_end)), ns_metadata[CODE_END_ADDR_IDX], portMPU_REGION_READ_ONLY, 2); //|portMPU_REGION_EXECUTE_NEVER

    printf("-------after MPU configuration\r\n");
    for (int i = 0; i < 8; i++)
    {
        NS_portMPU_RNR_REG = i; // region_num;
        printf("%d-th : %p - %p\r\n", i, NS_portMPU_RBAR_REG, NS_portMPU_RLAR_REG);
    }
}

void ns_metadata_pass_and_config(uint32_t *metadata, uint32_t n) __attribute__((cmse_nonsecure_entry));
void ns_metadata_pass_and_config(uint32_t *metadata, uint32_t n)
{
    for (int i = 0; i < n; i++)
    {
        ns_metadata[i + ns_metadata_idx] = metadata[i];
    }
    ns_metadata_idx += n;

    sandbox_size = (ns_metadata_idx - SANDBOX_START_ADDR_IDX) / 2;
    sandbox_size = sandbox_size;
    // printf("sandbox_size: %u (%u) - %u = %u(%u).\r\n",n,ns_metadata_idx,SANDBOX_START_ADDR_IDX,sandbox_size,(ns_metadata_idx-SANDBOX_START_ADDR_IDX));

    // for(;;){}
    { // init MPU
        for (int i = 6; i < 8; i++)
        {
            NS_portMPU_RNR_REG = i; // region_num;
            NS_portMPU_RBAR_REG = 0;
            NS_portMPU_RLAR_REG = 0;
        }
    }
    dwt_install_watchpoint(0, ns_metadata[IdleTASK_IDX], DWT_FUNCTION_INST);
    // printf("output_IDX: %p.\r\n", ns_metadata[Output_IDX]);
    dwt_install_watchpoint(1, ns_metadata[Output_IDX], DWT_FUNCTION_W);
    dwt_install_watchpoint(2, ns_metadata[MemManage_Handler_IDX], DWT_FUNCTION_INST);
    // printf("MemHan addr : %p \r\n",ns_metadata[MemManage_Handler_IDX]);
    mtb_enable(MTB_ALIAS_MAX);
    ARM_CM_DWT_CYCCNT = ARM_CM_DWT_EXCCNT = 0;
    // printf("sandbox-range %p - %p\r\n",ns_metadata[SANDBOX_START_ADDR_IDX+2],ns_metadata[SANDBOX_START_ADDR_IDX+2]+ns_metadata[SANDBOX_START_ADDR_IDX+2+1]);
    // printf("ns_metadata_pass_and_config: %p.\r\n", ns_metadata[0]);
    // ns_configureMPU(ns_metadata[SANDBOX_START_ADDR_IDX],ns_metadata[SANDBOX_START_ADDR_IDX+1] , portMPU_REGION_READ_ONLY,7);

    // sandbox_entry_detect_mpu(ns_metadata[SANDBOX_START_ADDR_IDX+(((sandbox_count_idx%sandbox_size)*2))],ns_metadata[SANDBOX_START_ADDR_IDX+((sandbox_count_idx%sandbox_size)*2+1)]);
    sandbox_count_idx--;
    sandbox_entry_detect_dwt(ns_metadata[SANDBOX_START_ADDR_IDX + (((0) % sandbox_size) * 2)], ns_metadata[SANDBOX_START_ADDR_IDX + (((0) % sandbox_size) * 2 + 1)]);
}

static uint32_t is_in_sandbox = 0;

uint32_t dwt3_trigger_time = 0;
#define MMFAR_NS_Const 0xE002ED34
void DebugMon_Handler_Main(sContextStateFrame *stack)
{
   

    // printf("start_debumon.\r\n");

    MTB->MASTER &= ~(1 << 31);

    disable_SysTick_ns(); // temporally

 
    // if it is a dwt event
    // if ((SCB->DFSR & SCB_DFSR_DWTTRAP_Msk))
    int dwt_id = isDWTmatched();
    if (dwt_id > -1)
    // if ((SCB->DFSR & SCB_DFSR_DWTTRAP_Msk)&& !(SCB->DFSR & SCB_DFSR_EXTERNAL_Msk))
    {

        // We have served this event so clear mask
        uint32_t before_clear = SCB->DFSR;
        SCB->DFSR = CLEARED_MASK_VALUE;
        // SCB->DFSR  &= (~(CLEARED_DWT_MASK_VALUE));//InsectACIDE
        // SCB->DFSR=0;
        // printf("SCB->DFSR %p : %p: %p.\r\n", before_clear,CLEARED_DWT_MASK_VALUE, SCB->DFSR);//InsectACIDE
        // printf("DWT0 %p: %p.\r\n", DWT->FUNCTION0,DWT_FUNCTION_INST );//InsectACIDE
        // clear the function match
        if (dwt_id == 0)
        {
            printf("DWT0: idle task intercept.\r\n"); // InsectACIDE
            // uint32_t current_dwt_function=DWT->FUNCTION0;
            DWT->FUNCTION0 &= ~DWT_FUNCTION_MATCHED_Msk;
            int record_count = MTB->POSITION;

            processing_trace_buff(1);

            if (!process_preempted)
            { // secure idle task.
                uint32_t idf = 0;
                enable_SysTick_ns();
                while (1)
                {
                    idf++;

                    uint32_t ctr_tick = SysTick_NS->CTRL;
                    if (ctr_tick & SysTick_CTRL_COUNTFLAG_Msk)
                    {
                        // printf("tick %p - %u\r\n",ctr_tick,idf);
                        printf("\r\n");
                        MTB->MASTER |= (1 << 31);

                        return;
                    }
                }
                disable_SysTick_ns();
            }
            else
            {
                process_preempted = 0;
            }

            printf("done-idle.\r\n");

            // printf("configured DWT0:%p ; %p \r\n",DWT->FUNCTION0,DWT->COMP0);//InsectACIDE
        }
        else if (dwt_id == 1)
        {
            printf("DWT1: physical output event intercept.\r\n");
            DWT->FUNCTION1 &= ~DWT_FUNCTION_MATCHED_Msk;
            int record_count = MTB->POSITION;
        }
        else if (dwt_id == 2)
        {
            DWT->FUNCTION2 &= ~DWT_FUNCTION_MATCHED_Msk;
            uint32_t fault_addr = *((uint32_t *)MMFAR_NS_Const);
            printf("DWT2: MPU fault intercept, fault address: %p.\r\n", fault_addr);

            uint32_t *ns_msp = 0;
            uint32_t *ns_psp = 0;
            uint32_t *s_msp = 0;
            uint32_t *s_psp = 0;
            __asm volatile(
                "mrs %0, MSP_NS\n"
                "mrs %1, PSP_NS\n"
                "mrs %2, MSP\n"
                "mrs %3, PSP\n"
                : "=r"(ns_msp), "=r"(ns_psp), "=r"(s_msp), "=r"(s_psp));
            uint32_t ns_lr = ns_msp[5];
            uint32_t *ns_sp;
            if (ns_lr & 4)
            {
                ns_sp = ns_psp;
            }
            else
            {
                ns_sp = ns_msp;
            }
            uint32_t ns_ulPC = ns_sp[6];

            if ((ns_ulPC >= ((ns_metadata[SANDBOX_START_ADDR_IDX + (((sandbox_count_idx % sandbox_size) * 2))] + 32) & portMPU_RBAR_ADDRESS_MASK)) && (ns_ulPC <= (ns_metadata[SANDBOX_START_ADDR_IDX + ((sandbox_count_idx % sandbox_size) * 2 + 1)] & portMPU_RLAR_ADDRESS_MASK)))
            {
                printf("entering  (%d)-th sandbox at %p, whose range is [%p,%p]\r\n", (sandbox_count_idx % sandbox_size), ns_ulPC, ((ns_metadata[SANDBOX_START_ADDR_IDX + (((sandbox_count_idx % sandbox_size) * 2))] + 32) & portMPU_RBAR_ADDRESS_MASK), (ns_metadata[SANDBOX_START_ADDR_IDX + ((sandbox_count_idx % sandbox_size) * 2 + 1)] & portMPU_RLAR_ADDRESS_MASK));

                // printf("see amsks %p & %p = %p.\r\n",ns_metadata[SANDBOX_START_ADDR_IDX+((sandbox_count_idx%sandbox_size)*2+1)],  portMPU_RBAR_ADDRESS_MASK,ns_metadata[SANDBOX_START_ADDR_IDX+((sandbox_count_idx%sandbox_size)*2+1)] & portMPU_RLAR_ADDRESS_MASK);
                // for(;;){}

                printf("-------before MPU configuration.\r\n");
                for (int i = 0; i < 8; i++)
                {
                    NS_portMPU_RNR_REG = i; // region_num;
                    printf("%d-th : %p - %p\r\n", i, NS_portMPU_RBAR_REG, NS_portMPU_RLAR_REG);
                }
                // printf("to configure %p - %p .\r\n", ns_metadata[SANDBOX_START_ADDR_IDX+(((sandbox_count_idx%sandbox_size)*2))], ns_metadata[SANDBOX_START_ADDR_IDX+((sandbox_count_idx%sandbox_size)*2+1)]);
                sandbox_exit_detect(ns_metadata[SANDBOX_START_ADDR_IDX + (((sandbox_count_idx % sandbox_size) * 2))], ns_metadata[SANDBOX_START_ADDR_IDX + ((sandbox_count_idx % sandbox_size) * 2 + 1)]);
            
                {
                    uint32_t record_count = MTB->POSITION;
                    uint32_t src_addr, dst_addr;
                    // MTB->POSITION = 0;
                    // printf("record_count: %d.\r\n", record_count);
                    for (int idx = record_count / 4 - 1; idx >= 0; idx -= 2)
                    {

                        // check non-secure branches, code region: [0x00200000~0x00400000]
                        src_addr = mtb_buff[idx];
                        dst_addr = mtb_buff[idx + 1] & LAST_BIT_CLEAR;
                        // printf("entering src: %p, dst: %p.\r\n", src_addr, dst_addr);
                    }
                }

                // MTB->MASTER |= (1 << 31);
                MTB->MASTER &= ~(1 << 31);
                enable_SysTick_ns(); // temporally

                return;

                //    is_in_sandbox=1;
            }
            else
            {
                printf(" leaving (%d)-th sandbox at %p, whose range is [%p,%p]\r\n", (sandbox_count_idx % sandbox_size), ns_ulPC, ((ns_metadata[SANDBOX_START_ADDR_IDX + (((sandbox_count_idx % sandbox_size) * 2))]) & portMPU_RBAR_ADDRESS_MASK), (ns_metadata[SANDBOX_START_ADDR_IDX + ((sandbox_count_idx % sandbox_size) * 2 + 1)] + 32) & portMPU_RLAR_ADDRESS_MASK);

                printf("-------before MPU configuration.\r\n");
                for (int i = 0; i < 8; i++)
                {
                    NS_portMPU_RNR_REG = i; // region_num;
                    printf("%d-th MPU : %p - %p\r\n", i, NS_portMPU_RBAR_REG, NS_portMPU_RLAR_REG);
                }

                sandbox_entry_detect_mpu(ns_metadata[SANDBOX_START_ADDR_IDX + (((sandbox_count_idx % sandbox_size) * 2))], ns_metadata[SANDBOX_START_ADDR_IDX + ((sandbox_count_idx % sandbox_size) * 2 + 1)]);

                {
                    uint32_t record_count = MTB->POSITION;
                    uint32_t src_addr, dst_addr;
                    // MTB->POSITION = 0;
                    // printf("record_count: %d.\r\n", record_count);
                    for (int idx = record_count / 4 - 16; idx < record_count / 4; idx += 2)
                    {

                        // check non-secure branches, code region: [0x00200000~0x00400000]
                        src_addr = mtb_buff[idx];
                        dst_addr = mtb_buff[idx + 1] & LAST_BIT_CLEAR;
                        // printf("leaving src: %p, dst: %p.\r\n", src_addr, dst_addr);
                    }
                }

                MTB->MASTER |= (1 << 31); // turn back on MTB
                enable_SysTick_ns();

                return;
            }
            /***
            printf("MSP_NS: %p, PSP_NS: %p ; MSP: %p, PSP: %p.  RET: %p\r\n",ns_msp,ns_psp,s_msp,s_psp,return_addr);
            for(int k=0;k<20;k++){
                // printf("\r (%d) MSP_NS: %p, PSP_NS: %p ; MSP: %p, PSP: %p.\r\n",k,ns_msp[k],ns_psp[k],s_msp[k],s_psp[k]);
            }
            printf("after MPU.\r\n");
            for(int i=0;i<8;i++){
                NS_portMPU_RNR_REG = i ;//region_num;
                printf("%d : %p - %p\r\n",i,NS_portMPU_RBAR_REG,NS_portMPU_RLAR_REG);
            }
            */

            // for(;;){}
        }
        else if (dwt_id == 3)
        {
            dwt3_trigger_time++;
            DWT->FUNCTION3 &= ~DWT_FUNCTION_MATCHED_Msk;
            printf("DWT3: detect new selective tracing region, configure sandbox.\r\n");

            sandbox_count_idx++;
            sandbox_count_idx = sandbox_count_idx % sandbox_size;

            printf("----before MPU configuration\r\n");
            for (int i = 0; i < 8; i++)
            {
                NS_portMPU_RNR_REG = i; // region_num;
                printf("%d-th MPU : %p - %p\r\n", i, NS_portMPU_RBAR_REG, NS_portMPU_RLAR_REG);
            }

            sandbox_entry_detect_mpu(ns_metadata[SANDBOX_START_ADDR_IDX + (((sandbox_count_idx % sandbox_size) * 2))], ns_metadata[SANDBOX_START_ADDR_IDX + ((sandbox_count_idx % sandbox_size) * 2 + 1)]);
            sandbox_entry_detect_dwt(ns_metadata[SANDBOX_START_ADDR_IDX + (((sandbox_count_idx + 1) % sandbox_size) * 2)], ns_metadata[SANDBOX_START_ADDR_IDX + (((sandbox_count_idx + 1) % sandbox_size) * 2 + 1)]);
            printf("(%d)-th sandbox configured: %p - %p; next sandbox entry is: %p.\r\n", (sandbox_count_idx % sandbox_size),
                   ns_metadata[SANDBOX_START_ADDR_IDX + (((sandbox_count_idx % sandbox_size) * 2))], ns_metadata[SANDBOX_START_ADDR_IDX + ((sandbox_count_idx % sandbox_size) * 2 + 1)],
                   ns_metadata[SANDBOX_START_ADDR_IDX + (((sandbox_count_idx + 1) % sandbox_size) * 2)]);

           

            // printf("-----after\r\n");
            // for (int i = 0; i < 8; i++)
            // {
            //     NS_portMPU_RNR_REG = i; // region_num;
            //     printf("%d : %p - %p\r\n", i, NS_portMPU_RBAR_REG, NS_portMPU_RLAR_REG);
            // }

           
        }
        else
        {
            printf("Wrong DWT event.\r\n");
        }

       
        MTB->MASTER |= (1 << 31);

  
        enable_SysTick_ns(); 

        return;
    }
    

    if (SCB->DFSR & SCB_DFSR_EXTERNAL_Msk)
    // if (MTB->MASTER & (1 << 9))
    {
        // printf("enter validation here \r\n");
        // We have served this event so clear mask
        // SCB->DFSR &= ~SCB_DFSR_EXTERNAL_Msk;
        // uint32_t before_clear=SCB->DFSR;//InsectACIDE
        // SCB->DFSR &= (~CLEARED_MASK_VALUE);
        SCB->DFSR = CLEARED_MASK_VALUE;
        // printf("MTB SCB->DFSR %p - %p -  %p.\r\n", before_clear,CLEARED_MASK_VALUE,SCB->DFSR);//InsectACIDE
        // SCB->DFSR = CLEARED_MASK_VALUE;

        // disable debugmonitor
        // DCB->DEMCR &= ~DCB_DEMCR_MON_EN_Msk;
        DCB->DEMCR = DEBUGMON_DISABLE;


        // disable non-secure systick interrupt
        disable_SysTick_ns(); // temporally



        // disable MTB, replaced by a fixed value to reduce instructions
        MTB->MASTER = MTB_CLEAR_VALUE;
        MTB->FLOW = 0;

        __asm volatile("dsb\n"
                       "isb\n");


        //  printf("MTB interrupts.\r\n");
       
        // processing_trace_buff();
        copy_to_trace_buff();


        // enable debug monitor
        // DCB->DEMCR |= DCB_DEMCR_MON_EN_Msk;
        DCB->DEMCR = DEBUGMON_ENABLE;

        // enable non-secure systick interrupt and set the frequency to 100ms

        enable_SysTick_ns(); // temporally
                     // IRQ

        // enable MTB  0xFC3
        MTB->FLOW = WATERMARK_VALUE;
        // MTB->MASTER |= (1 << 31);  // replace it by a fix value to reduce the instructions
        MTB->MASTER = MTB_RESET_VALUE;
        MTB->MASTER |= (1 << 31);
        // mtb_reconfig();
        // MTB->MASTER |= 1 << 6;//re-enable DWT InsectACIDE
        __asm volatile("dsb\n"
                       "isb\n");
    }
    // printf("end_debumon.\r\n");
    MTB->MASTER |= (1 << 31);
}

void config_dwt()
{


    // configure MTB_MASTER register to enable TSTOPEN
    MTB->MASTER |= 1 << 6;
}

void cfi_test_s()
{
    task_stack_init();
    task_list_init();

    if (debug_monitor_enable())
    {
        printf("\r\nMonitor Mode Debug Enabled!\r\n");
    }


    LoadBranchTable();

    printf("----Binary search enabled.---\r\n");

    measurement_init();

    config_dwt();
    MTB->POSITION = 0;

}