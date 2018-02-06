#include <stdint.h>
#include <stdbool.h>

#include "mcu.h"
#include "toboot-api.h"

/**
 * @brief   Type of an IRQ vector.
 */
typedef void  (*irq_vector_t)(void);

/**
 * @brief   Type of a structure representing the whole vectors table.
 */
typedef struct {
  uint32_t      *init_stack;
  irq_vector_t  reset_handler;
  irq_vector_t  nmi_handler;
  irq_vector_t  hardfault_handler;
  irq_vector_t  memmanage_handler;
  irq_vector_t  busfault_handler;
  irq_vector_t  usagefault_handler;
  irq_vector_t  vector1c;
  irq_vector_t  vector20;
  irq_vector_t  vector24;
  irq_vector_t  vector28;
  irq_vector_t  svc_handler;
  irq_vector_t  debugmonitor_handler;
  irq_vector_t  vector34;
  irq_vector_t  pendsv_handler;
  irq_vector_t  systick_handler;
  irq_vector_t  vectors[CORTEX_NUM_VECTORS];
} vectors_t;

void _unhandled_exception(void) {
  asm("bkpt #0");
  while (true) {
  }
}

extern uint32_t __main_stack_end__;
void Reset_Handler(void) __attribute__((weak, alias("_unhandled_exception")));
void NMI_Handler(void) __attribute__((weak, alias("_unhandled_exception")));
void HardFault_Handler(void) __attribute__((weak, alias("_unhandled_exception")));
void MemManage_Handler(void) __attribute__((weak, alias("_unhandled_exception")));
void BusFault_Handler(void) __attribute__((weak, alias("_unhandled_exception")));
void UsageFault_Handler(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector1C(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector20(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector24(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector28(void) __attribute__((weak, alias("_unhandled_exception")));
void SVC_Handler(void) __attribute__((weak, alias("_unhandled_exception")));
void DebugMon_Handler(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector34(void) __attribute__((weak, alias("_unhandled_exception")));
void PendSV_Handler(void) __attribute__((weak, alias("_unhandled_exception")));
void SysTick_Handler(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector40(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector44(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector48(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector4C(void) __attribute__((weak, alias("_unhandled_exception")));
#if CORTEX_NUM_VECTORS > 4
void Vector50(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector54(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector58(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector5C(void) __attribute__((weak, alias("_unhandled_exception")));
#endif
#if CORTEX_NUM_VECTORS > 8
void Vector60(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector64(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector68(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector6C(void) __attribute__((weak, alias("_unhandled_exception")));
#endif
#if CORTEX_NUM_VECTORS > 12
void Vector70(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector74(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector78(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector7C(void) __attribute__((weak, alias("_unhandled_exception")));
#endif
#if CORTEX_NUM_VECTORS > 16
void Vector80(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector84(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector88(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector8C(void) __attribute__((weak, alias("_unhandled_exception")));
#endif
#if CORTEX_NUM_VECTORS > 20
void Vector90(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector94(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector98(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector9C(void) __attribute__((weak, alias("_unhandled_exception")));
#endif
#if CORTEX_NUM_VECTORS > 24
void VectorA0(void) __attribute__((weak, alias("_unhandled_exception")));
void VectorA4(void) __attribute__((weak, alias("_unhandled_exception")));
void VectorA8(void) __attribute__((weak, alias("_unhandled_exception")));
void VectorAC(void) __attribute__((weak, alias("_unhandled_exception")));
#endif
#if CORTEX_NUM_VECTORS > 28
void VectorB0(void) __attribute__((weak, alias("_unhandled_exception")));
void VectorB4(void) __attribute__((weak, alias("_unhandled_exception")));
void VectorB8(void) __attribute__((weak, alias("_unhandled_exception")));
void VectorBC(void) __attribute__((weak, alias("_unhandled_exception")));
#endif

__attribute__ ((used, section(".vectors")))
vectors_t _vectors = {
  &__main_stack_end__,Reset_Handler,      NMI_Handler,        HardFault_Handler,
  MemManage_Handler,  BusFault_Handler,   UsageFault_Handler, Vector1C,
  Vector20,           Vector24,           Vector28,           SVC_Handler,
  DebugMon_Handler,   Vector34,           PendSV_Handler,     SysTick_Handler,
  {
    Vector40,           Vector44,           Vector48,           Vector4C,
#if CORTEX_NUM_VECTORS > 4
    Vector50,           Vector54,           Vector58,           Vector5C,
#endif
#if CORTEX_NUM_VECTORS > 8
    Vector60,           Vector64,           Vector68,           Vector6C,
#endif
#if CORTEX_NUM_VECTORS > 12
    Vector70,           Vector74,           Vector78,           Vector7C,
#endif
#if CORTEX_NUM_VECTORS > 16
    Vector80,           Vector84,           Vector88,           Vector8C,
#endif
#if CORTEX_NUM_VECTORS > 20
    Vector90,
#endif
  }
};
