/*----------------------------------------------------------------------------
 * Name:    main_ns.c
 * Purpose: Main function non-secure mode
 *----------------------------------------------------------------------------*/

#include <stdint.h>
#include "IOTKit_CM33_FP.h" /* Device header */
#include "Board_LED.h"      /* ::Board Support:LED */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "Secure_Functions.h"


/* Externs needed by the MPU setup code. These are defined in Scatter-Loading
 * description file (FreeRTOSDemo_ns.sct). */
extern uint32_t Image$$ER_IROM_NS_PRIVILEGED$$Base;
extern uint32_t Image$$ER_IROM_NS_PRIVILEGED_ALIGN$$Limit;
extern uint32_t Image$$ER_IROM_NS_FREERTOS_SYSCALL$$Base;
extern uint32_t Image$$ER_IROM_NS_FREERTOS_SYSCALL_ALIGN$$Limit;
extern uint32_t Image$$ER_IROM_NS_UNPRIVILEGED$$Base;
extern uint32_t Image$$ER_IROM_NS_UNPRIVILEGED_ALIGN$$Limit;

extern uint32_t Image$$RW_IRAM_NS_PRIVILEGED$$Base;
extern uint32_t Image$$RW_IRAM_NS_PRIVILEGED_ALIGN$$Limit;
extern uint32_t Image$$RW_IRAM_NS_UNPRIVILEGED$$Base;
extern uint32_t Image$$RW_IRAM_NS_UNPRIVILEGED_ALIGN$$Limit;

/* Privileged flash. */
const uint32_t *__privileged_functions_start__ = (uint32_t *)&(Image$$ER_IROM_NS_PRIVILEGED$$Base);
const uint32_t *__privileged_functions_end__ = (uint32_t *)((uint32_t) & (Image$$ER_IROM_NS_PRIVILEGED_ALIGN$$Limit)-0x1); /* Last address in privileged Flash region. */

/* Flash containing system calls. */
const uint32_t *__syscalls_flash_start__ = (uint32_t *)&(Image$$ER_IROM_NS_FREERTOS_SYSCALL$$Base);
const uint32_t *__syscalls_flash_end__ = (uint32_t *)((uint32_t) & (Image$$ER_IROM_NS_FREERTOS_SYSCALL_ALIGN$$Limit)-0x1); /* Last address in Flash region containing system calls. */

/* Unprivileged flash. Note that the section containing system calls is
 * unprivileged so that unprivileged tasks can make system calls. */
const uint32_t *__unprivileged_flash_start__ = (uint32_t *)&(Image$$ER_IROM_NS_UNPRIVILEGED$$Base);
const uint32_t *__unprivileged_flash_end__ = (uint32_t *)((uint32_t) & (Image$$ER_IROM_NS_UNPRIVILEGED_ALIGN$$Limit)-0x1); /* Last address in un-privileged Flash region. */

/* RAM with priviledged access only. This contains kernel data. */
const uint32_t *__privileged_sram_start__ = (uint32_t *)&(Image$$RW_IRAM_NS_PRIVILEGED$$Base);
const uint32_t *__privileged_sram_end__ = (uint32_t *)((uint32_t) & (Image$$RW_IRAM_NS_PRIVILEGED_ALIGN$$Limit)-0x1); /* Last address in privileged RAM. */

/* Unprivileged RAM. */
const uint32_t *__unprivileged_sram_start__ = (uint32_t *)&(Image$$RW_IRAM_NS_UNPRIVILEGED$$Base);
const uint32_t *__unprivileged_sram_end__ = (uint32_t *)((uint32_t) & (Image$$RW_IRAM_NS_UNPRIVILEGED_ALIGN$$Limit)-0x1); /* Last address in un-privileged RAM. */
                                                                                                                          /*-----------------------------------------------------------*/
                                                                                                                          /*----------------From portmacro.h--------------------------*/

#define portPRIVILEGED_FLASH_REGION (0UL)
#define portUNPRIVILEGED_FLASH_REGION (1UL)
#define portUNPRIVILEGED_SYSCALLS_REGION (2UL)
#define portPRIVILEGED_RAM_REGION (3UL)
#define portSTACK_REGION (4UL)
#define portFIRST_CONFIGURABLE_REGION (5UL)
// #define portLAST_CONFIGURABLE_REGION                  ( 7UL )
#define portLAST_CONFIGURABLE_REGION (5UL)
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
/**
 * @brief Control evaluation.
 */

/**
 * @brief Application task startup hook.
 */
void vApplicationDaemonTaskStartupHook(void);

/**
 * @brief Size of the shared memory region.
 */
#define SHARED_MEMORY_SIZE 32

/**
 * @brief Memory region shared between two tasks.
 */
static uint8_t ucSharedMemory[SHARED_MEMORY_SIZE] __attribute__((aligned(32)));

/**
 * @brief Memory region used to track Memory Fault intentionally caused by the
 * RO Access task.
 *
 * RO Access task sets ucROTaskFaultTracker[ 0 ] to 1 before accessing illegal
 * memory. Illegal memory access causes Memory Fault and the fault handler
 * checks ucROTaskFaultTracker[ 0 ] to see if this is an expected fault. We
 * recover gracefully from an expected fault by jumping to the next instruction.
 *
 * @note We are declaring a region of 32 bytes even though we need only one. The
 * reason is that the size of an MPU region must be a multiple of 32 bytes.
 */
static uint8_t ucROTaskFaultTracker[SHARED_MEMORY_SIZE] __attribute__((aligned(32))) = {0};
/*-----------------------------------------------------------*/

/**
 * @brief Implements the task which has Read Only access to the memory region
 * ucSharedMemory.
 *
 * @param pvParameters[in] Parameters as passed during task creation.
 */


/**
 * @brief Implements the task which has Read Write access to the memory region
 * ucSharedMemory.
 *
 * @param pvParameters[in] Parameters as passed during task creation.
 */
static void sherlocBeebsTask(void *pvParameters);

/*-----------------------------------------------------------*/

/**
 * @brief The hard fault handler.
 *
 * It calls a function called vHandleMemoryFault.
 */
void MemManage_Handler(void) __attribute__((naked));
/*-----------------------------------------------------------*/

/**
 * @brief Create all demo tasks.
 */


static void rtosEmptyTask(void *pvParameters);

#define BufferSize configMINIMAL_STACK_SIZE * 2
static StackType_t microStackBuffer1[BufferSize];
static StackType_t microStackBuffer2[BufferSize];

static StackType_t idleStackBuffer[BufferSize];

#define TASK1_STACK_BUFF BufferSize * 2
static StackType_t task1StackBuffer[TASK1_STACK_BUFF] __attribute__((aligned(32))) = {0};
;

// static uint8_t task1_stack_Memory[TASK1_STACK_BUFF] __attribute__((aligned(32)));

void vApplicationDaemonTaskStartupHook(void);
static void SystemClock_Config(void);

/* system call functions */
void bench_start() __attribute__((noinline, section("freertos_system_calls")));
void bench_result() __attribute__((noinline, section("freertos_system_calls")));
/*----------------------------------------------------------*/

void tested_trigger() __attribute__((noinline));
void tested_trigger()
{
    int a = 1;
    int y = 0;
    Secure_printf("this is tested\r\n");
    for (int i = 0; i < 1000; i++)
    {
        y = y + i;
    }
    printf_int(y);
}

// #include <stdio.h>

typedef void (*FunctionPtr)();

__attribute__((optimize("O0"))) void functionA()
{
    Secure_printf("This is function A\n");
}

void functionB() __attribute__((noinline))
{
    Secure_printf("This is function B\n");
}

void indirectFunctionCall(FunctionPtr funcPtr) __attribute__((noinline))
{

    funcPtr();
    Secure_printf("This is indrect A\n");
}

int cs_cfi_test() __attribute__((noinline))
{
    FunctionPtr ptr = &functionA;
    indirectFunctionCall(ptr); // Call functionA

    ptr = &functionB;
    indirectFunctionCall(ptr); // Call functionB

    return 0;
}

// #include "task.h"
// /* MPU wrappers includes. */
// #include "mpu_wrappers.h"

// /* Portasm includes. */
#include "portasm.h"
/* copy from port.c */

#define MTB_BUFFER_TEST_MAX 0x100
uint8_t MPU_buffer_test[MTB_BUFFER_TEST_MAX] __attribute__((aligned(32)));
uint32_t MPU_single_test __attribute__((aligned(32)));

#define portSCB_SYS_HANDLER_CTRL_STATE_REG (*(volatile uint32_t *)0xe000ed24)
#define portSCB_MEM_FAULT_ENABLE_BIT (1UL << 16UL)

#define portMPU_RBAR_A_base_REG 0xE000EDA4
#define portMPU_RLAR_A_base_REG 0xE000EDA8

static void prvSetupMPU(void) __attribute__((noinline)) /* PRIVILEGED_FUNCTION */
{

    if (1 || portMPU_TYPE_REG == portEXPECTED_MPU_TYPE_VALUE)
    {
        portMPU_CTRL_REG &= (~1);
        /* MAIR0 - Index 0. */
        portMPU_MAIR0_REG |= ((portMPU_NORMAL_MEMORY_BUFFERABLE_CACHEABLE << portMPU_MAIR_ATTR0_POS) & portMPU_MAIR_ATTR0_MASK);
        /* MAIR0 - Index 1. */
        portMPU_MAIR0_REG |= ((portMPU_DEVICE_MEMORY_nGnRE << portMPU_MAIR_ATTR1_POS) & portMPU_MAIR_ATTR1_MASK);

        for (uint32_t i = 0; i < 8; i++)
        {
            portMPU_RNR_REG = i;
            portMPU_RBAR_REG = (((uint32_t)0) & portMPU_RBAR_ADDRESS_MASK) |
                               (portMPU_REGION_NON_SHAREABLE) |
                               (portMPU_REGION_READ_WRITE);
            portMPU_RLAR_REG = (((uint32_t)0) & portMPU_RLAR_ADDRESS_MASK) |
                               (portMPU_RLAR_ATTR_INDEX0) |
                               (portMPU_RLAR_REGION_ENABLE);
        }
        /* Setup privileged flash as Read Only so that privileged tasks can
         * read it but not modify. */
        portMPU_RNR_REG = portPRIVILEGED_FLASH_REGION;
        portMPU_RBAR_REG = (((uint32_t)__privileged_functions_start__) & portMPU_RBAR_ADDRESS_MASK) |
                           (portMPU_REGION_NON_SHAREABLE) |
                           (portMPU_REGION_PRIVILEGED_READ_ONLY);
        portMPU_RLAR_REG = (((uint32_t)__privileged_functions_end__) & portMPU_RLAR_ADDRESS_MASK) |
                           (portMPU_RLAR_ATTR_INDEX0) |
                           (portMPU_RLAR_REGION_ENABLE);

        /* Setup unprivileged flash as Read Only by both privileged and
         * unprivileged tasks. All tasks can read it but no-one can modify. */
        portMPU_RNR_REG = portUNPRIVILEGED_FLASH_REGION;
        portMPU_RBAR_REG = (((uint32_t)__unprivileged_flash_start__) & portMPU_RBAR_ADDRESS_MASK) |
                           (portMPU_REGION_NON_SHAREABLE) |
                           (portMPU_REGION_READ_ONLY);
        portMPU_RLAR_REG = (((uint32_t)__unprivileged_flash_end__) & portMPU_RLAR_ADDRESS_MASK) |
                           (portMPU_RLAR_ATTR_INDEX0) |
                           (portMPU_RLAR_REGION_ENABLE);

        /* Setup unprivileged syscalls flash as Read Only by both privileged
         * and unprivileged tasks. All tasks can read it but no-one can modify. */
        portMPU_RNR_REG = portUNPRIVILEGED_SYSCALLS_REGION;
        portMPU_RBAR_REG = (((uint32_t)__syscalls_flash_start__) & portMPU_RBAR_ADDRESS_MASK) |
                           (portMPU_REGION_NON_SHAREABLE) |
                           (portMPU_REGION_READ_ONLY);
        portMPU_RLAR_REG = (((uint32_t)__syscalls_flash_end__) & portMPU_RLAR_ADDRESS_MASK) |
                           (portMPU_RLAR_ATTR_INDEX0) |
                           (portMPU_RLAR_REGION_ENABLE);

        /* Setup RAM containing kernel data for privileged access only. */
        portMPU_RNR_REG = portPRIVILEGED_RAM_REGION;
        portMPU_RBAR_REG = (((uint32_t)__privileged_sram_start__) & portMPU_RBAR_ADDRESS_MASK) |
                           (portMPU_REGION_NON_SHAREABLE) |
                           (portMPU_REGION_PRIVILEGED_READ_WRITE) |
                           (portMPU_REGION_EXECUTE_NEVER);
        portMPU_RLAR_REG = (((uint32_t)__privileged_sram_end__) & portMPU_RLAR_ADDRESS_MASK) |
                           (portMPU_RLAR_ATTR_INDEX0) |
                           (portMPU_RLAR_REGION_ENABLE);

        /* Enable mem fault. */
        portSCB_SYS_HANDLER_CTRL_STATE_REG |= portSCB_MEM_FAULT_ENABLE_BIT;

        /* Enable MPU with privileged background access i.e. unmapped
         * regions have privileged access. */
        portMPU_CTRL_REG |= (portMPU_PRIV_BACKGROUND_ENABLE_BIT | portMPU_ENABLE_BIT);
    }
}

static void configureMPU(uint32_t *baseaddr, uint32_t size, uint32_t permission, uint32_t region_num) __attribute__((noinline))
{

    if (region_num < portFIRST_CONFIGURABLE_REGION || region_num > portLAST_CONFIGURABLE_REGION)
    {
        return;
    }
    portMPU_CTRL_REG &= (~1);
    // uint32_t MAIR_attr_pos=(region_num-4)*8;

    /* MAIR1 - Index 5. */
    // portMPU_MAIR1_REG |= ( ( portMPU_NORMAL_MEMORY_BUFFERABLE_CACHEABLE << MAIR_attr_pos) & (0xff<<MAIR_attr_pos) );

    /* Setup privileged flash as Read Only so that privileged tasks can
     * read it but not modify. */

    portMPU_RNR_REG = region_num; // region_num;
    portMPU_RBAR_REG = (((uint32_t)baseaddr) & portMPU_RBAR_ADDRESS_MASK) |
                       (portMPU_REGION_NON_SHAREABLE) |
                       (permission);
    portMPU_RLAR_REG = (((uint32_t)(baseaddr + size)) & portMPU_RLAR_ADDRESS_MASK) |
                       (portMPU_RLAR_ATTR_INDEX0) |
                       (portMPU_RLAR_REGION_ENABLE);

    /* Setup unprivileged flash as Read Only by both privileged and
     * unprivileged tasks. All tasks can read it but no-one can modify. */

    /* Enable mem fault. */
    portSCB_SYS_HANDLER_CTRL_STATE_REG |= portSCB_MEM_FAULT_ENABLE_BIT;

    /* Enable MPU with privileged background access i.e. unmapped
     * regions have privileged access. */
    portMPU_CTRL_REG |= (portMPU_PRIV_BACKGROUND_ENABLE_BIT | portMPU_ENABLE_BIT);
    // portMPU_CTRL_REG |= (  portMPU_ENABLE_BIT );
}
static void setMPU2(void) __attribute__((noinline)) /* PRIVILEGED_FUNCTION */
{

    portMPU_CTRL_REG &= (~1);
    /* MAIR1 - Index 5. */
    for (uint32_t i = 0; i < 8; i++)
    {
        portMPU_RNR_REG = 1 << i;
        portMPU_RBAR_REG = (((uint32_t)0) & portMPU_RBAR_ADDRESS_MASK) |
                           (portMPU_REGION_NON_SHAREABLE) |
                           (portMPU_REGION_READ_WRITE);
        portMPU_RLAR_REG = (((uint32_t)0) & portMPU_RLAR_ADDRESS_MASK) |
                           (portMPU_RLAR_ATTR_INDEX0) |
                           (portMPU_RLAR_REGION_ENABLE);
    }

    portMPU_MAIR1_REG |= ((portMPU_NORMAL_MEMORY_BUFFERABLE_CACHEABLE << portMPU_MAIR_ATTR5_POS) & portMPU_MAIR_ATTR5_MASK);

    /* Setup privileged flash as Read Only so that privileged tasks can
     * read it but not modify. */
    portMPU_RNR_REG = 1 << portFIRST_CONFIGURABLE_REGION;
    portMPU_RBAR_REG = (((uint32_t)(MPU_buffer_test)) & portMPU_RBAR_ADDRESS_MASK) |
                       (portMPU_REGION_NON_SHAREABLE) |
                       (portMPU_REGION_READ_ONLY);
    portMPU_RLAR_REG = (((uint32_t)(MPU_buffer_test + MTB_BUFFER_TEST_MAX)) & portMPU_RLAR_ADDRESS_MASK) |
                       (portMPU_RLAR_ATTR_INDEX0) |
                       (portMPU_RLAR_REGION_ENABLE);

    /* Setup unprivileged flash as Read Only by both privileged and
     * unprivileged tasks. All tasks can read it but no-one can modify. */

    /* Enable mem fault. */
    portSCB_SYS_HANDLER_CTRL_STATE_REG |= portSCB_MEM_FAULT_ENABLE_BIT;

    /* Enable MPU with privileged background access i.e. unmapped
     * regions have privileged access. */
    portMPU_CTRL_REG |= (portMPU_PRIV_BACKGROUND_ENABLE_BIT | portMPU_ENABLE_BIT);
}

void try_MPU()
{
    portMPU_CTRL_REG &= (~1);
    portMPU_MAIR1_REG |= ((portMPU_NORMAL_MEMORY_BUFFERABLE_CACHEABLE << portMPU_MAIR_ATTR5_POS) & portMPU_MAIR_ATTR5_MASK);

    portMPU_RNR_REG = portFIRST_CONFIGURABLE_REGION;
    portMPU_RBAR_REG
        // *((volatile uint32_t*)(portMPU_RBAR_A_base_REG+portFIRST_CONFIGURABLE_REGION*8))
        = (((uint32_t)(&MPU_single_test)) & portMPU_RBAR_ADDRESS_MASK) |
          (portMPU_REGION_NON_SHAREABLE) |
          (portMPU_REGION_READ_ONLY);
    portMPU_RLAR_REG
        // *((volatile uint32_t*)(portMPU_RLAR_A_base_REG+portFIRST_CONFIGURABLE_REGION*8))
        = (((uint32_t)(&MPU_single_test)) & portMPU_RLAR_ADDRESS_MASK) |
          (portMPU_RLAR_ATTR_INDEX0) |
          (portMPU_RLAR_REGION_ENABLE);

    __asm volatile("dsb" ::: "memory");
    __asm volatile("isb");
    printf_int(*((volatile uint32_t *)(portMPU_RBAR_A_base_REG + (portFIRST_CONFIGURABLE_REGION) * 8)));
    printf_int(*((volatile uint32_t *)(portMPU_RLAR_A_base_REG + (portFIRST_CONFIGURABLE_REGION) * 8)));
    Secure_printf("-----\r\n");
    /* Enable mem fault. */

    printf_int(portMPU_RBAR_REG);
    printf_int(portMPU_RLAR_REG);
    Secure_printf("--!!!!!---\r\n");
    portMPU_RNR_REG = 4;
    printf_int(portMPU_RBAR_REG);
    printf_int(portMPU_RLAR_REG);

    portSCB_SYS_HANDLER_CTRL_STATE_REG |= portSCB_MEM_FAULT_ENABLE_BIT;

    /* Enable MPU with privileged background access i.e. unmapped
     * regions have privileged access. */
    portMPU_CTRL_REG |= (portMPU_PRIV_BACKGROUND_ENABLE_BIT | portMPU_ENABLE_BIT);
    __asm volatile("dsb" ::: "memory");
    __asm volatile("isb");
}

volatile void test_MPU() __attribute__((noinline, section("freertos_system_calls")))
{
    //    portRAISE_PRIVILEGE();
    Secure_printf("Test-MPU\r\n");
    // prvSetupMPU();
    // try_MPU();
    //     printf_int((uint32_t)__privileged_functions_start__); 0x0020 0000 #0
    //     printf_int((uint32_t)__privileged_functions_end__); 0x0020 1fff

    //    printf_int((uint32_t)__syscalls_flash_start__);0x0020 2000 #2
    //    printf_int((uint32_t)__syscalls_flash_end__); 0x0020 21bf

    //     printf_int((uint32_t)__unprivileged_flash_start__); 0x0020 21c0 #1
    //     printf_int((uint32_t)__unprivileged_flash_end__);0x0020 2a9f
    /* Flash containing system calls. */

    // configureMPU(&MPU_buffer_test[0],MTB_BUFFER_TEST_MAX , portMPU_REGION_READ_ONLY,7);

    // uint32_t first=MPU_buffer_test[0];
    // printf_int(&MPU_single_test);
    // Secure_printf("MPU-----Base\r\n");

    for (uint32_t i = 0; i < 8; i++)
    {
        portMPU_RNR_REG = i;
        // printf_int(portMPU_RBAR_REG);
        // printf_int(portMPU_RLAR_REG);
        // Secure_printf("-----\r\n");
    }

    //  Secure_printf("go-!-MPU\r\n");
    volatile uint32_t *tptr = MPU_buffer_test;
    //   __privileged_functions_start__+0x100;
    *tptr = 0x1234;
    *(tptr + 10) = 0xabcd;

    //  Secure_printf("ki-!!!!MPU\r\n");

    // setMPU2();
    // portMPU_REGION_READ_WRITE
    // portMPU_REGION_PRIVILEGED_READ_ONLY

    // printf_int((uint32_t)&MPU_buffer_test[0]);
    //  return;
    // tptr=&MPU_buffer_test[0]+MTB_BUFFER_TEST_MAX/2;
    // *tptr=0x1234;
    Secure_printf("bbfg-MPU\r\n");
    return;
}
// uint32_t mmio_registers;
void write_physical_output() PRIVILEGED_FUNCTION
{
    portRAISE_PRIVILEGE();

    uint32_t *mmio_registers = &task1StackBuffer[0];
    *mmio_registers = 0x1234;
    Secure_printf("write_physical_output\r\n");
    portRESET_PRIVILEGE();
}

void test_system_config_setup()
{

    Secure_dwt_install_watchpoint(0, (uint32_t)&rtosEmptyTask - 1, DWT_FUNCTION_INST); // hijack empty task
    Secure_dwt_install_watchpoint(1, (uint32_t)&tested_trigger - 1, DWT_FUNCTION_INST);
    // Secure_dwt_install_watchpoint(1, (uint32_t)&mmio_registers, DWT_FUNCTION_W);

    Secure_dwt_install_watchpoint(2, (uint32_t)&MemManage_Handler - 1, DWT_FUNCTION_INST);

    printf_int((uint32_t)&tested_trigger - 1);
    mtb_enable(MTB_ALIAS_MAX);
    ARM_CM_DWT_CYCCNT = ARM_CM_DWT_EXCCNT = 0;
}
//  extern char region0_end;
extern char task1_region0_end;
//   extern char region0_start;
extern char task1_region0_start;

//   extern char region1_end;
//   extern char region1_start;
extern char task1_region1_end;
extern char task1_region1_start;

//   extern char memHandle_end;
extern uint32_t memHandle_end_tmp;
portDONT_DISCARD void vHandleMemoryFault(uint32_t *) PRIVILEGED_FUNCTION;
void system_config_setup();

// ns_metadata_pass_and_config
/* Non-secure main. */
int main(void)
{
    Secure_printf("fs-dfds");

    system_config_setup();
    test_MPU();

    tested_trigger();
    tested_trigger();
    cs_cfi_test();
    tested_trigger();
    /* start the schedule */
    vTaskStartScheduler();

    /* Should not reach here as the scheduler is already started. */
    for (;;)
    {
    }
}
/*-----------------------------------------------------------*/


__attribute__((always_inline)) int sandbox_0(uint32_t n)
{
    uint32_t ret = 0;
    for (int i = 0; i < 0x1000; i++)
    {
        if (i * ret > 4)
        {
            ret += (i + n);
        }
        else
        {
            ret += i * 2;
        }
    }
    //  Secure_printf("sandbox_0\r\n");
    printf_int(ret);
    return ret;
}

__attribute__((always_inline)) int sandbox_1(uint32_t n)
{
    uint32_t ret = 0;
    for (int i = 0; i < 0x1000; i++)
    {
        if (i * ret > 4)
        {
            ret += (i + n);
        }
        else
        {
            ret += i * 2;
        }
    }
    printf_int(ret);
    return ret;
    //  Secure_printf("sandbox_1\r\n");
    //  printf_int(ret);
}

// static void sherlocBeebsTask __attribute__((noinline, section("freertos_system_calls")));


static void rtosEmptyTask(void *pvParameters)
{
   
    int y = 0;
    while (1)
    {
        __asm volatile(
            ".global idle_task_loop\n\t"
            "idle_task_loop:\n\t" // Define a label
        );
        y++;
        if (y % 1000 == 0)
        {
            Secure_printf("Idle task\r\n");
            printf_int(y);
        }
    }
}

#include "task1.cpp"

void start_tasks()
{
    BaseType_t xReturn = pdFAIL;

    // create an TaskParameters_t structure that defines the task to be created.
    

    TaskParameters_t task1TaskParameters = {
        .pvTaskCode = task1,                     // pvTaskCode - the function that implements the task.
        .pcName = "Task1",                       // pcName - just a text name for the task to assist debugging.
        .usStackDepth = TASK1_STACK_BUFF - 0x10, // configMINIMAL_STACK_SIZE, // usStackDepth	- the stack size DEFINED IN WORDS.
        .pvParameters = NULL,                    // pvParameters - passed into the task function as the function parameters.
        .uxPriority = configMAX_PRIORITIES - 1,  // uxPriority - task priority, set the portPRIVILEGE_BIT if the task should run in a privileged state.

        .puxStackBuffer = &task1StackBuffer[0x10], // puxStackBuffer - the buffer to be used as the task stack.

        // xRegions - Allocate up to three separate memory regions for access by
        // the task, with appropriate access permissions.
        .xRegions = {
            // {0,0,0},
            {&task1StackBuffer[0], TASK1_STACK_BUFF, portMPU_REGION_READ_WRITE},
            // {&task1StackBuffer[0], TASK1_STACK_BUFF, portMPU_REGION_READ_WRITE}, // the other two region left unused.
            {0,0,0},
            {0,0,0}
            // {(void *)0x28204980, 0x1a0, portMPU_REGION_READ_WRITE},
            // {&ucSharedMemory[0], SHARED_MEMORY_SIZE, portMPU_REGION_READ_WRITE}
            }};

    if (xTaskCreateRestricted(&task1TaskParameters, NULL) == pdPASS)
   
    {
        xReturn = pdPASS;
    }
    else
    {
    }

    // Create an TaskParameters_t structure that defines the task to be created.
    TaskParameters_t emptyTaskParameters =
        {
            .pvTaskCode = rtosEmptyTask,                               // pvTaskCode - the function that implements the task.
            .pcName = "EmptyTask",                                   // pcName - just a text name for the task to assist debugging.
            .usStackDepth = configMINIMAL_STACK_SIZE,                     // usStackDepth	- the stack size DEFINED IN WORDS.
            .pvParameters = NULL,                                         // pvParameters - passed into the task function as the function parameters.
            .uxPriority = (configMAX_PRIORITIES - 3 | portPRIVILEGE_BIT), // uxPriority - task priority, set the portPRIVILEGE_BIT if the task should run in a privileged state.
            .puxStackBuffer = idleStackBuffer,                          // puxStackBuffer - the buffer to be used as the task stack.

            // xRegions - Allocate up to three separate memory regions for access by
            // the task, with appropriate access permissions.
            .xRegions = {
                {&idleStackBuffer[0], BufferSize, portMPU_REGION_READ_WRITE}, // the other two region left unused.
                {0, 0, 0},
                {0, 0, 0}}};
    if (xTaskCreateRestricted(&emptyTaskParameters, NULL) == pdPASS)
    {

        xReturn = pdPASS;
    }
    else
    {
    }
}

/**
 * @brief Warn user if pvPortMalloc fails.
 *
 * Called if a call to pvPortMalloc() fails because there is insufficient
 * free memory available in the FreeRTOS heap.  pvPortMalloc() is called
 * internally by FreeRTOS API functions that create tasks, queues, software
 * timers, and semaphores.  The size of the FreeRTOS heap is set by the
 * configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h.
 *
 */
void vApplicationMallocFailedHook() PRIVILEGED_FUNCTION
{
    /* The TCP tests will test behavior when the entire heap is allocated. In
     * order to avoid interfering with those tests, this function does nothing. */
}
/*-----------------------------------------------------------*/

/**
 * @brief Loop forever if stack overflow is detected.
 *
 * If configCHECK_FOR_STACK_OVERFLOW is set to 1,
 * this hook provides a location for applications to
 * define a response to a stack overflow.
 *
 * Use this hook to help identify that a stack overflow
 * has occurred.
 *
 */
void vApplicationStackOverflowHook(TaskHandle_t xTask,
                                   char *pcTaskName) PRIVILEGED_FUNCTION
{
    portDISABLE_INTERRUPTS();

    /* Loop forever */
    for (;;)
    {
    }
}

/*-----------------------------------------------------------*/

void vApplicationDaemonTaskStartupHook(void) PRIVILEGED_FUNCTION
{

    start_tasks();
}

#define MMFSR_Const 0xE000ED28
#define MMFAR_Const 0xE000ED34

portDONT_DISCARD void vHandleMemoryFault(uint32_t *pulFaultStackAddress) PRIVILEGED_FUNCTION
{

    uint32_t return_addr;
    __asm volatile(
        "mov %0, lr"
        : "=r"(return_addr));
    printf_int(return_addr);

    // Secure_printf("Memory Fault\r\n");
    uint8_t fault_status = *((uint8_t *)MMFSR_Const);
    printf_int(fault_status);
    uint32_t fault_addr = *((uint32_t *)MMFAR_Const);

    // Secure_printf("MPU Base\r\n");
    // for(uint32_t i=0;i<8;i++){
    //     printf_int(*((uint32_t*)(portMPU_RBAR_A_base_REG+(i-1)*8)));
    //     printf_int(*((uint32_t*)(portMPU_RLAR_A_base_REG+(i-1)*8)));
    //     Secure_printf("-----\r\n");
    // }
    // printf_int((uint32_t)&MPU_buffer_test[0]);

    uint32_t ulPC;
    uint16_t usOffendingInstruction;

    /* Is this an expected fault? */

    // if (ucROTaskFaultTracker[0] == 1)
    // {
    //     Secure_printf("expected-address \r\n");
    // }else{
    //     Secure_printf("anomaly-address \r\n");
    // }
  
        /* Read program counter. */
        ulPC = pulFaultStackAddress[6];
        
        Secure_printf("Faulty address: ");
        printf_int(ulPC);
        printf_int(fault_addr);
        
        /* Read the offending instruction. */
        usOffendingInstruction = *(uint16_t *)ulPC;

        /* From ARM docs:
         * If the value of bits[15:11] of the halfword being decoded is one of
         * the following, the halfword is the first halfword of a 32-bit
         * instruction:
         * - 0b11101.
         * - 0b11110.
         * - 0b11111.
         * Otherwise, the halfword is a 16-bit instruction.
         */

        /* Extract bits[15:11] of the offending instruction. */
        usOffendingInstruction = usOffendingInstruction & 0xF800;
        usOffendingInstruction = (usOffendingInstruction >> 11);

        /* Determine if the offending instruction is a 32-bit instruction or
         * a 16-bit instruction. */
        if (fault_addr == 0)
        {
            // Secure_printf("Instruction fault \r\n");
        }
        else if ((usOffendingInstruction == 0x001F) ||
                 (usOffendingInstruction == 0x001E) ||
                 (usOffendingInstruction == 0x001D))
        {
            /* Since the offending instruction is a 32-bit instruction,
             * increment the program counter by 4 to move to the next
             * instruction. */
            ulPC += 4;
        }
        else
        {
            /* Since the offending instruction is a 16-bit instruction,
             * increment the program counter by 2 to move to the next
             * instruction. */
            ulPC += 2;
        }

        /* Save the new program counter on the stack. */
        pulFaultStackAddress[6] = ulPC;

        /* Mark the fault as handled. */
        ucROTaskFaultTracker[0] = 0;
  
}
/*-----------------------------------------------------------*/
//  uint32_t memHandle_end_tmp=(uint32_t)&memHandle_end - 1;
void MemManage_Handler(void) PRIVILEGED_FUNCTION
{
    __asm volatile(
        " tst lr, #4										\n"
        " ite eq											\n"
        " mrseq r0, msp										\n"
        " mrsne r0, psp										\n"
        " ldr r1, handler_address_const						\n"
        " bx r1												\n"
        "													\n"
        " handler_address_const: .word vHandleMemoryFault	\n");
}
/*-----------------------------------------------------------*/

extern char idle_task_loop;
void system_config_setup()
{
    uint32_t buff[0x20];
    int i = 0;
    
    buff[i] = (uint32_t)&idle_task_loop;
    i++;
    buff[i] = (uint32_t)&task1StackBuffer[0];
    i++;
    buff[i] = (uint32_t)&MemManage_Handler - 1; // must use this address,
    //    buff[i]=(uint32_t)&vHandleMemoryFault-1; //this address can larger than region
    i++;



    buff[i] = (uint32_t)&vHandleMemoryFault - 1; // temporally

    i++;

    buff[i] = (uint32_t)__privileged_functions_start__;
    i++;
    buff[i] = (uint32_t)__unprivileged_flash_end__;
    i++;
 
    //  i++;
    //  buff[i]=(uint32_t)&region0_start;
    buff[i] = (uint32_t)&task1_region0_start;
    i++;
    buff[i] = ((uint32_t)&task1_region0_end);
    i++;

    buff[i] = (uint32_t)&task1_region1_start;
    i++;
    buff[i] = ((uint32_t)&task1_region1_end);
    i++;

    ns_metadata_pass_and_config(buff, i);
}