/*----------------------------------------------------------------------------
 * Name:    Secure_Functions.h
 * Purpose: Function and Typedef Declarations to include into NonSecure Application.
 *----------------------------------------------------------------------------*/

/* Define DWT and MTB buffer size */
#define ARM_CM_DWT_CYCCNT (*(uint32_t *)0xE0001004) // DWT->CYCCNT
#define ARM_CM_DWT_EXCCNT (*(uint32_t *)0xE000100C) // DWT->EXCCNT
#define MTB_ALIAS_MAX 0x10000                       // 0x10000: 16kb

#define DWT_FUNCTION_INST 0x50000812  // 0b01010000000000000000100000010010, trigger a debugmon event when the comparator matches

/* Define typedef for NonSecure callback function */ 
typedef int32_t (*NonSecure_funcptr)(uint32_t);

/* Function declarations for Secure functions called from NonSecure application */

extern void    Secure_printf (char*);
extern void    printf_int(uint32_t);

extern int32_t time_measurement_s(uint32_t time);
extern int mtb_enable(size_t mtb_size);


void Secure_dwt_install_watchpoint(uint32_t comp_id, uint32_t address, uint32_t function);

void ns_metadata_pass_and_config(uint32_t *metadata, uint32_t n);