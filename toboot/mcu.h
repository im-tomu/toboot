#ifndef MCU_H
#define MCU_H

#include <stdint.h>

#define __I volatile
#define __IO volatile
#define __O volatile

/** Interrupt Number Definition */
typedef enum IRQn
{
/******  Cortex-M0+ Processor Exceptions Numbers *****************************************/
  NonMaskableInt_IRQn = -14,                /*!< -14 Cortex-M0+ Non Maskable Interrupt   */
  HardFault_IRQn      = -13,                /*!< -13 Cortex-M0+ Hard Fault Interrupt     */
  SVCall_IRQn         = -5,                 /*!< -5  Cortex-M0+ SV Call Interrupt        */
  PendSV_IRQn         = -2,                 /*!< -2  Cortex-M0+ Pend SV Interrupt        */
  SysTick_IRQn        = -1,                 /*!< -1  Cortex-M0+ System Tick Interrupt    */

/******  EFM32HG Peripheral Interrupt Numbers ********************************************/
  DMA_IRQn            = 0,  /*!< 0 EFM32 DMA Interrupt */
  GPIO_EVEN_IRQn      = 1,  /*!< 1 EFM32 GPIO_EVEN Interrupt */
  TIMER0_IRQn         = 2,  /*!< 2 EFM32 TIMER0 Interrupt */
  ACMP0_IRQn          = 3,  /*!< 3 EFM32 ACMP0 Interrupt */
  ADC0_IRQn           = 4,  /*!< 4 EFM32 ADC0 Interrupt */
  I2C0_IRQn           = 5,  /*!< 5 EFM32 I2C0 Interrupt */
  GPIO_ODD_IRQn       = 6,  /*!< 6 EFM32 GPIO_ODD Interrupt */
  TIMER1_IRQn         = 7,  /*!< 7 EFM32 TIMER1 Interrupt */
  USART1_RX_IRQn      = 8,  /*!< 8 EFM32 USART1_RX Interrupt */
  USART1_TX_IRQn      = 9,  /*!< 9 EFM32 USART1_TX Interrupt */
  LEUART0_IRQn        = 10, /*!< 10 EFM32 LEUART0 Interrupt */
  PCNT0_IRQn          = 11, /*!< 11 EFM32 PCNT0 Interrupt */
  RTC_IRQn            = 12, /*!< 12 EFM32 RTC Interrupt */
  CMU_IRQn            = 13, /*!< 13 EFM32 CMU Interrupt */
  VCMP_IRQn           = 14, /*!< 14 EFM32 VCMP Interrupt */
  MSC_IRQn            = 15, /*!< 15 EFM32 MSC Interrupt */
  USART0_RX_IRQn      = 17, /*!< 17 EFM32 USART0_RX Interrupt */
  USART0_TX_IRQn      = 18, /*!< 18 EFM32 USART0_TX Interrupt */
  USB_IRQn            = 19, /*!< 19 EFM32 USB Interrupt */
  TIMER2_IRQn         = 20, /*!< 20 EFM32 TIMER2 Interrupt */
} IRQn_Type;

/** \brief  Structure type to access the System Control Block (SCB).
 */
typedef struct
{
  __I  uint32_t CPUID;                   /*!< Offset: 0x000 (R/ )  CPUID Base Register                                   */
  __IO uint32_t ICSR;                    /*!< Offset: 0x004 (R/W)  Interrupt Control and State Register                  */
  __IO uint32_t VTOR;                    /*!< Offset: 0x008 (R/W)  Vector Table Offset Register                          */
  __IO uint32_t AIRCR;                   /*!< Offset: 0x00C (R/W)  Application Interrupt and Reset Control Register      */
  __IO uint32_t SCR;                     /*!< Offset: 0x010 (R/W)  System Control Register                               */
  __IO uint32_t CCR;                     /*!< Offset: 0x014 (R/W)  Configuration Control Register                        */
       uint32_t RESERVED1;
  __IO uint32_t SHP[2];                  /*!< Offset: 0x01C (R/W)  System Handlers Priority Registers. [0] is RESERVED   */
  __IO uint32_t SHCSR;                   /*!< Offset: 0x024 (R/W)  System Handler Control and State Register             */
} SCB_Type;

/** \brief  Structure type to access the System Timer (SysTick).
 */
typedef struct
{
  __IO uint32_t CTRL;                    /*!< Offset: 0x000 (R/W)  SysTick Control and Status Register */
  __IO uint32_t LOAD;                    /*!< Offset: 0x004 (R/W)  SysTick Reload Value Register       */
  __IO uint32_t VAL;                     /*!< Offset: 0x008 (R/W)  SysTick Current Value Register      */
  __I  uint32_t CALIB;                   /*!< Offset: 0x00C (R/ )  SysTick Calibration Register        */
} SysTick_Type;

/** \brief  Structure type to access the Nested Vectored Interrupt Controller (NVIC).
 */
typedef struct
{
  __IO uint32_t ISER[1];                 /*!< Offset: 0x000 (R/W)  Interrupt Set Enable Register           */
       uint32_t RESERVED0[31];
  __IO uint32_t ICER[1];                 /*!< Offset: 0x080 (R/W)  Interrupt Clear Enable Register          */
       uint32_t RSERVED1[31];
  __IO uint32_t ISPR[1];                 /*!< Offset: 0x100 (R/W)  Interrupt Set Pending Register           */
       uint32_t RESERVED2[31];
  __IO uint32_t ICPR[1];                 /*!< Offset: 0x180 (R/W)  Interrupt Clear Pending Register         */
       uint32_t RESERVED3[31];
       uint32_t RESERVED4[64];
  __IO uint32_t IP[8];                   /*!< Offset: 0x300 (R/W)  Interrupt Priority Register              */
}  NVIC_Type;

/** \brief  EFM32HG_CMU Register Declaration
 */
typedef struct
{
  __IO uint32_t CTRL;          /**< CMU Control Register  */
  __IO uint32_t HFCORECLKDIV;  /**< High Frequency Core Clock Division Register  */
  __IO uint32_t HFPERCLKDIV;   /**< High Frequency Peripheral Clock Division Register  */
  __IO uint32_t HFRCOCTRL;     /**< HFRCO Control Register  */
  __IO uint32_t LFRCOCTRL;     /**< LFRCO Control Register  */
  __IO uint32_t AUXHFRCOCTRL;  /**< AUXHFRCO Control Register  */
  __IO uint32_t CALCTRL;       /**< Calibration Control Register  */
  __IO uint32_t CALCNT;        /**< Calibration Counter Register  */
  __IO uint32_t OSCENCMD;      /**< Oscillator Enable/Disable Command Register  */
  __IO uint32_t CMD;           /**< Command Register  */
  __IO uint32_t LFCLKSEL;      /**< Low Frequency Clock Select Register  */
  __I uint32_t  STATUS;        /**< Status Register  */
  __I uint32_t  IF;            /**< Interrupt Flag Register  */
  __IO uint32_t IFS;           /**< Interrupt Flag Set Register  */
  __IO uint32_t IFC;           /**< Interrupt Flag Clear Register  */
  __IO uint32_t IEN;           /**< Interrupt Enable Register  */
  __IO uint32_t HFCORECLKEN0;  /**< High Frequency Core Clock Enable Register 0  */
  __IO uint32_t HFPERCLKEN0;   /**< High Frequency Peripheral Clock Enable Register 0  */
  uint32_t       RESERVED0[2];  /**< Reserved for future use **/
  __I uint32_t  SYNCBUSY;      /**< Synchronization Busy Register  */
  __IO uint32_t FREEZE;        /**< Freeze Register  */
  __IO uint32_t LFACLKEN0;     /**< Low Frequency A Clock Enable Register 0  (Async Reg)  */
  uint32_t       RESERVED1[1];  /**< Reserved for future use **/
  __IO uint32_t LFBCLKEN0;     /**< Low Frequency B Clock Enable Register 0 (Async Reg)  */
  __IO uint32_t LFCCLKEN0;     /**< Low Frequency C Clock Enable Register 0 (Async Reg)  */
  __IO uint32_t LFAPRESC0;     /**< Low Frequency A Prescaler Register 0 (Async Reg)  */
  uint32_t       RESERVED2[1];  /**< Reserved for future use **/
  __IO uint32_t LFBPRESC0;     /**< Low Frequency B Prescaler Register 0  (Async Reg)  */
  uint32_t       RESERVED3[1];  /**< Reserved for future use **/
  __IO uint32_t PCNTCTRL;      /**< PCNT Control Register  */

  uint32_t       RESERVED4[1];  /**< Reserved for future use **/
  __IO uint32_t ROUTE;         /**< I/O Routing Register  */
  __IO uint32_t LOCK;          /**< Configuration Lock Register  */

  uint32_t       RESERVED5[18]; /**< Reserved for future use **/
  __IO uint32_t USBCRCTRL;     /**< USB Clock Recovery Control  */
  __IO uint32_t USHFRCOCTRL;   /**< USHFRCO Control  */
  __IO uint32_t USHFRCOTUNE;   /**< USHFRCO Frequency Tune  */
  __IO uint32_t USHFRCOCONF;   /**< USHFRCO Configuration  */
} CMU_TypeDef;

/** \brief  EFM32HG_RMU Register Declaration
 */
typedef struct
{
  __IO uint32_t CTRL;     /**< Control Register  */
  __I uint32_t  RSTCAUSE; /**< Reset Cause Register  */
  __O uint32_t  CMD;      /**< Command Register  */
} RMU_TypeDef;

/** \brief  EFM32HG_EMU Register Declaration
 */
typedef struct
{
  __IO uint32_t CTRL;         /**< Control Register  */

  uint32_t       RESERVED0[1]; /**< Reserved for future use **/
  __IO uint32_t LOCK;         /**< Configuration Lock Register  */

  uint32_t       RESERVED1[6]; /**< Reserved for future use **/
  __IO uint32_t AUXCTRL;      /**< Auxiliary Control Register  */
} EMU_TypeDef;

/** \brief   EFM32HG_RTC Register Declaration
 */
typedef struct
{
  __IO uint32_t CTRL;     /**< Control Register  */
  __IO uint32_t CNT;      /**< Counter Value Register  */
  __IO uint32_t COMP0;    /**< Compare Value Register 0  */
  __IO uint32_t COMP1;    /**< Compare Value Register 1  */
  __I uint32_t  IF;       /**< Interrupt Flag Register  */
  __IO uint32_t IFS;      /**< Interrupt Flag Set Register  */
  __IO uint32_t IFC;      /**< Interrupt Flag Clear Register  */
  __IO uint32_t IEN;      /**< Interrupt Enable Register  */

  __IO uint32_t FREEZE;   /**< Freeze Register  */
  __I uint32_t  SYNCBUSY; /**< Synchronization Busy Register  */
} RTC_TypeDef;

 /** \brief  EFM32HG_WDOG Register Declaration
  */
typedef struct
{
  __IO uint32_t CTRL;     /**< Control Register  */
  __IO uint32_t CMD;      /**< Command Register  */

  __I uint32_t  SYNCBUSY; /**< Synchronization Busy Register  */
} WDOG_TypeDef;

/* Memory mapping of Cortex-M0+ Hardware */
#define SCS_BASE            (0xE000E000UL)                            /*!< System Control Space Base Address */
#define SysTick_BASE        (SCS_BASE +  0x0010UL)                    /*!< SysTick Base Address              */
#define NVIC_BASE           (SCS_BASE +  0x0100UL)                    /*!< NVIC Base Address                 */
#define SCB_BASE            (SCS_BASE +  0x0D00UL)                    /*!< System Control Block Base Address */
#define RTC_BASE            (0x40080000UL)                            /*!< RTC Base Address                  */
#define WDOG_BASE           (0x40088000UL)                            /*!< RTC Base Address                  */
#define EMU_BASE            (0x400C6000UL)                            /*!< EMU Base Address                  */
#define CMU_BASE            (0x400C8000UL)                            /*!< CMU Base Address                  */
#define RMU_BASE            (0x400CA000UL)                            /*!< RMU Base Address                  */

#define SCB                 ((SCB_Type       *)     SCB_BASE      )   /*!< SCB configuration struct           */
#define SysTick             ((SysTick_Type   *)     SysTick_BASE  )   /*!< SysTick configuration struct       */
#define NVIC                ((NVIC_Type      *)     NVIC_BASE     )   /*!< NVIC configuration struct          */
#define RTC                 ((RTC_TypeDef    *)     RTC_BASE      )   /*!< RTC configuration struct           */
#define WDOG                ((WDOG_TypeDef   *)     WDOG_BASE     )   /*!< Watchdog configuration struct      */
#define EMU                 ((EMU_TypeDef    *)     EMU_BASE      )   /*!< Energy Management Unit configuration struct */
#define CMU                 ((CMU_TypeDef    *)     CMU_BASE      )   /*!< CMU configuration struct           */
#define RMU                 ((RMU_TypeDef    *)     RMU_BASE      )   /*!< Reset Management Unit configuration struct */

/* Bit fields for CMU HFPERCLKEN0 */
#define _CMU_HFPERCLKEN0_RESETVALUE                 0x00000000UL                           /**< Default value for CMU_HFPERCLKEN0 */
#define _CMU_HFPERCLKEN0_MASK                       0x00000FFFUL                           /**< Mask for CMU_HFPERCLKEN0 */
#define CMU_HFPERCLKEN0_TIMER0                      (0x1UL << 0)                           /**< Timer 0 Clock Enable */
#define _CMU_HFPERCLKEN0_TIMER0_SHIFT               0                                      /**< Shift value for CMU_TIMER0 */
#define _CMU_HFPERCLKEN0_TIMER0_MASK                0x1UL                                  /**< Bit mask for CMU_TIMER0 */
#define _CMU_HFPERCLKEN0_TIMER0_DEFAULT             0x00000000UL                           /**< Mode DEFAULT for CMU_HFPERCLKEN0 */
#define CMU_HFPERCLKEN0_TIMER0_DEFAULT              (_CMU_HFPERCLKEN0_TIMER0_DEFAULT << 0) /**< Shifted mode DEFAULT for CMU_HFPERCLKEN0 */
#define CMU_HFPERCLKEN0_TIMER1                      (0x1UL << 1)                           /**< Timer 1 Clock Enable */
#define _CMU_HFPERCLKEN0_TIMER1_SHIFT               1                                      /**< Shift value for CMU_TIMER1 */
#define _CMU_HFPERCLKEN0_TIMER1_MASK                0x2UL                                  /**< Bit mask for CMU_TIMER1 */
#define _CMU_HFPERCLKEN0_TIMER1_DEFAULT             0x00000000UL                           /**< Mode DEFAULT for CMU_HFPERCLKEN0 */
#define CMU_HFPERCLKEN0_TIMER1_DEFAULT              (_CMU_HFPERCLKEN0_TIMER1_DEFAULT << 1) /**< Shifted mode DEFAULT for CMU_HFPERCLKEN0 */
#define CMU_HFPERCLKEN0_TIMER2                      (0x1UL << 2)                           /**< Timer 2 Clock Enable */
#define _CMU_HFPERCLKEN0_TIMER2_SHIFT               2                                      /**< Shift value for CMU_TIMER2 */
#define _CMU_HFPERCLKEN0_TIMER2_MASK                0x4UL                                  /**< Bit mask for CMU_TIMER2 */
#define _CMU_HFPERCLKEN0_TIMER2_DEFAULT             0x00000000UL                           /**< Mode DEFAULT for CMU_HFPERCLKEN0 */
#define CMU_HFPERCLKEN0_TIMER2_DEFAULT              (_CMU_HFPERCLKEN0_TIMER2_DEFAULT << 2) /**< Shifted mode DEFAULT for CMU_HFPERCLKEN0 */
#define CMU_HFPERCLKEN0_USART0                      (0x1UL << 3)                           /**< Universal Synchronous/Asynchronous Receiver/Transmitter 0 Clock Enable */
#define _CMU_HFPERCLKEN0_USART0_SHIFT               3                                      /**< Shift value for CMU_USART0 */
#define _CMU_HFPERCLKEN0_USART0_MASK                0x8UL                                  /**< Bit mask for CMU_USART0 */
#define _CMU_HFPERCLKEN0_USART0_DEFAULT             0x00000000UL                           /**< Mode DEFAULT for CMU_HFPERCLKEN0 */
#define CMU_HFPERCLKEN0_USART0_DEFAULT              (_CMU_HFPERCLKEN0_USART0_DEFAULT << 3) /**< Shifted mode DEFAULT for CMU_HFPERCLKEN0 */
#define CMU_HFPERCLKEN0_USART1                      (0x1UL << 4)                           /**< Universal Synchronous/Asynchronous Receiver/Transmitter 1 Clock Enable */
#define _CMU_HFPERCLKEN0_USART1_SHIFT               4                                      /**< Shift value for CMU_USART1 */
#define _CMU_HFPERCLKEN0_USART1_MASK                0x10UL                                 /**< Bit mask for CMU_USART1 */
#define _CMU_HFPERCLKEN0_USART1_DEFAULT             0x00000000UL                           /**< Mode DEFAULT for CMU_HFPERCLKEN0 */
#define CMU_HFPERCLKEN0_USART1_DEFAULT              (_CMU_HFPERCLKEN0_USART1_DEFAULT << 4) /**< Shifted mode DEFAULT for CMU_HFPERCLKEN0 */
#define CMU_HFPERCLKEN0_ACMP0                       (0x1UL << 5)                           /**< Analog Comparator 0 Clock Enable */
#define _CMU_HFPERCLKEN0_ACMP0_SHIFT                5                                      /**< Shift value for CMU_ACMP0 */
#define _CMU_HFPERCLKEN0_ACMP0_MASK                 0x20UL                                 /**< Bit mask for CMU_ACMP0 */
#define _CMU_HFPERCLKEN0_ACMP0_DEFAULT              0x00000000UL                           /**< Mode DEFAULT for CMU_HFPERCLKEN0 */
#define CMU_HFPERCLKEN0_ACMP0_DEFAULT               (_CMU_HFPERCLKEN0_ACMP0_DEFAULT << 5)  /**< Shifted mode DEFAULT for CMU_HFPERCLKEN0 */
#define CMU_HFPERCLKEN0_PRS                         (0x1UL << 6)                           /**< Peripheral Reflex System Clock Enable */
#define _CMU_HFPERCLKEN0_PRS_SHIFT                  6                                      /**< Shift value for CMU_PRS */
#define _CMU_HFPERCLKEN0_PRS_MASK                   0x40UL                                 /**< Bit mask for CMU_PRS */
#define _CMU_HFPERCLKEN0_PRS_DEFAULT                0x00000000UL                           /**< Mode DEFAULT for CMU_HFPERCLKEN0 */
#define CMU_HFPERCLKEN0_PRS_DEFAULT                 (_CMU_HFPERCLKEN0_PRS_DEFAULT << 6)    /**< Shifted mode DEFAULT for CMU_HFPERCLKEN0 */
#define CMU_HFPERCLKEN0_IDAC0                       (0x1UL << 7)                           /**< Current Digital to Analog Converter 0 Clock Enable */
#define _CMU_HFPERCLKEN0_IDAC0_SHIFT                7                                      /**< Shift value for CMU_IDAC0 */
#define _CMU_HFPERCLKEN0_IDAC0_MASK                 0x80UL                                 /**< Bit mask for CMU_IDAC0 */
#define _CMU_HFPERCLKEN0_IDAC0_DEFAULT              0x00000000UL                           /**< Mode DEFAULT for CMU_HFPERCLKEN0 */
#define CMU_HFPERCLKEN0_IDAC0_DEFAULT               (_CMU_HFPERCLKEN0_IDAC0_DEFAULT << 7)  /**< Shifted mode DEFAULT for CMU_HFPERCLKEN0 */
#define CMU_HFPERCLKEN0_GPIO                        (0x1UL << 8)                           /**< General purpose Input/Output Clock Enable */
#define _CMU_HFPERCLKEN0_GPIO_SHIFT                 8                                      /**< Shift value for CMU_GPIO */
#define _CMU_HFPERCLKEN0_GPIO_MASK                  0x100UL                                /**< Bit mask for CMU_GPIO */
#define _CMU_HFPERCLKEN0_GPIO_DEFAULT               0x00000000UL                           /**< Mode DEFAULT for CMU_HFPERCLKEN0 */
#define CMU_HFPERCLKEN0_GPIO_DEFAULT                (_CMU_HFPERCLKEN0_GPIO_DEFAULT << 8)   /**< Shifted mode DEFAULT for CMU_HFPERCLKEN0 */
#define CMU_HFPERCLKEN0_VCMP                        (0x1UL << 9)                           /**< Voltage Comparator Clock Enable */
#define _CMU_HFPERCLKEN0_VCMP_SHIFT                 9                                      /**< Shift value for CMU_VCMP */
#define _CMU_HFPERCLKEN0_VCMP_MASK                  0x200UL                                /**< Bit mask for CMU_VCMP */
#define _CMU_HFPERCLKEN0_VCMP_DEFAULT               0x00000000UL                           /**< Mode DEFAULT for CMU_HFPERCLKEN0 */
#define CMU_HFPERCLKEN0_VCMP_DEFAULT                (_CMU_HFPERCLKEN0_VCMP_DEFAULT << 9)   /**< Shifted mode DEFAULT for CMU_HFPERCLKEN0 */
#define CMU_HFPERCLKEN0_ADC0                        (0x1UL << 10)                          /**< Analog to Digital Converter 0 Clock Enable */
#define _CMU_HFPERCLKEN0_ADC0_SHIFT                 10                                     /**< Shift value for CMU_ADC0 */
#define _CMU_HFPERCLKEN0_ADC0_MASK                  0x400UL                                /**< Bit mask for CMU_ADC0 */
#define _CMU_HFPERCLKEN0_ADC0_DEFAULT               0x00000000UL                           /**< Mode DEFAULT for CMU_HFPERCLKEN0 */
#define CMU_HFPERCLKEN0_ADC0_DEFAULT                (_CMU_HFPERCLKEN0_ADC0_DEFAULT << 10)  /**< Shifted mode DEFAULT for CMU_HFPERCLKEN0 */
#define CMU_HFPERCLKEN0_I2C0                        (0x1UL << 11)                          /**< I2C 0 Clock Enable */
#define _CMU_HFPERCLKEN0_I2C0_SHIFT                 11                                     /**< Shift value for CMU_I2C0 */
#define _CMU_HFPERCLKEN0_I2C0_MASK                  0x800UL                                /**< Bit mask for CMU_I2C0 */
#define _CMU_HFPERCLKEN0_I2C0_DEFAULT               0x00000000UL                           /**< Mode DEFAULT for CMU_HFPERCLKEN0 */
#define CMU_HFPERCLKEN0_I2C0_DEFAULT                (_CMU_HFPERCLKEN0_I2C0_DEFAULT << 11)  /**< Shifted mode DEFAULT for CMU_HFPERCLKEN0 */

/* Bit fields for CMU LFACLKEN0 */
#define _CMU_LFACLKEN0_RESETVALUE                   0x00000000UL                      /**< Default value for CMU_LFACLKEN0 */
#define _CMU_LFACLKEN0_MASK                         0x00000001UL                      /**< Mask for CMU_LFACLKEN0 */
#define CMU_LFACLKEN0_RTC                           (0x1UL << 0)                      /**< Real-Time Counter Clock Enable */
#define _CMU_LFACLKEN0_RTC_SHIFT                    0                                 /**< Shift value for CMU_RTC */
#define _CMU_LFACLKEN0_RTC_MASK                     0x1UL                             /**< Bit mask for CMU_RTC */
#define _CMU_LFACLKEN0_RTC_DEFAULT                  0x00000000UL                      /**< Mode DEFAULT for CMU_LFACLKEN0 */
#define CMU_LFACLKEN0_RTC_DEFAULT                   (_CMU_LFACLKEN0_RTC_DEFAULT << 0) /**< Shifted mode DEFAULT for CMU_LFACLKEN0 */

/* Bit fields for CMU LFBCLKEN0 */
#define _CMU_LFBCLKEN0_RESETVALUE                   0x00000000UL                          /**< Default value for CMU_LFBCLKEN0 */
#define _CMU_LFBCLKEN0_MASK                         0x00000001UL                          /**< Mask for CMU_LFBCLKEN0 */
#define CMU_LFBCLKEN0_LEUART0                       (0x1UL << 0)                          /**< Low Energy UART 0 Clock Enable */
#define _CMU_LFBCLKEN0_LEUART0_SHIFT                0                                     /**< Shift value for CMU_LEUART0 */
#define _CMU_LFBCLKEN0_LEUART0_MASK                 0x1UL                                 /**< Bit mask for CMU_LEUART0 */
#define _CMU_LFBCLKEN0_LEUART0_DEFAULT              0x00000000UL                          /**< Mode DEFAULT for CMU_LFBCLKEN0 */
#define CMU_LFBCLKEN0_LEUART0_DEFAULT               (_CMU_LFBCLKEN0_LEUART0_DEFAULT << 0) /**< Shifted mode DEFAULT for CMU_LFBCLKEN0 */

/* Bit fields for CMU LFCCLKEN0 */
#define _CMU_LFCCLKEN0_RESETVALUE                   0x00000000UL                        /**< Default value for CMU_LFCCLKEN0 */
#define _CMU_LFCCLKEN0_MASK                         0x00000001UL                        /**< Mask for CMU_LFCCLKEN0 */
#define CMU_LFCCLKEN0_USBLE                         (0x1UL << 0)                        /**< Universal Serial Bus Low Energy Clock Clock Enable */
#define _CMU_LFCCLKEN0_USBLE_SHIFT                  0                                   /**< Shift value for CMU_USBLE */
#define _CMU_LFCCLKEN0_USBLE_MASK                   0x1UL                               /**< Bit mask for CMU_USBLE */
#define _CMU_LFCCLKEN0_USBLE_DEFAULT                0x00000000UL                        /**< Mode DEFAULT for CMU_LFCCLKEN0 */
#define CMU_LFCCLKEN0_USBLE_DEFAULT                 (_CMU_LFCCLKEN0_USBLE_DEFAULT << 0) /**< Shifted mode DEFAULT for CMU_LFCCLKEN0 */

/* Bit fields for CMU LFAPRESC0 */
#define _CMU_LFAPRESC0_RESETVALUE                   0x00000000UL                       /**< Default value for CMU_LFAPRESC0 */
#define _CMU_LFAPRESC0_MASK                         0x0000000FUL                       /**< Mask for CMU_LFAPRESC0 */
#define _CMU_LFAPRESC0_RTC_SHIFT                    0                                  /**< Shift value for CMU_RTC */
#define _CMU_LFAPRESC0_RTC_MASK                     0xFUL                              /**< Bit mask for CMU_RTC */
#define _CMU_LFAPRESC0_RTC_DIV1                     0x00000000UL                       /**< Mode DIV1 for CMU_LFAPRESC0 */
#define _CMU_LFAPRESC0_RTC_DIV2                     0x00000001UL                       /**< Mode DIV2 for CMU_LFAPRESC0 */
#define _CMU_LFAPRESC0_RTC_DIV4                     0x00000002UL                       /**< Mode DIV4 for CMU_LFAPRESC0 */
#define _CMU_LFAPRESC0_RTC_DIV8                     0x00000003UL                       /**< Mode DIV8 for CMU_LFAPRESC0 */
#define _CMU_LFAPRESC0_RTC_DIV16                    0x00000004UL                       /**< Mode DIV16 for CMU_LFAPRESC0 */
#define _CMU_LFAPRESC0_RTC_DIV32                    0x00000005UL                       /**< Mode DIV32 for CMU_LFAPRESC0 */
#define _CMU_LFAPRESC0_RTC_DIV64                    0x00000006UL                       /**< Mode DIV64 for CMU_LFAPRESC0 */
#define _CMU_LFAPRESC0_RTC_DIV128                   0x00000007UL                       /**< Mode DIV128 for CMU_LFAPRESC0 */
#define _CMU_LFAPRESC0_RTC_DIV256                   0x00000008UL                       /**< Mode DIV256 for CMU_LFAPRESC0 */
#define _CMU_LFAPRESC0_RTC_DIV512                   0x00000009UL                       /**< Mode DIV512 for CMU_LFAPRESC0 */
#define _CMU_LFAPRESC0_RTC_DIV1024                  0x0000000AUL                       /**< Mode DIV1024 for CMU_LFAPRESC0 */
#define _CMU_LFAPRESC0_RTC_DIV2048                  0x0000000BUL                       /**< Mode DIV2048 for CMU_LFAPRESC0 */
#define _CMU_LFAPRESC0_RTC_DIV4096                  0x0000000CUL                       /**< Mode DIV4096 for CMU_LFAPRESC0 */
#define _CMU_LFAPRESC0_RTC_DIV8192                  0x0000000DUL                       /**< Mode DIV8192 for CMU_LFAPRESC0 */
#define _CMU_LFAPRESC0_RTC_DIV16384                 0x0000000EUL                       /**< Mode DIV16384 for CMU_LFAPRESC0 */
#define _CMU_LFAPRESC0_RTC_DIV32768                 0x0000000FUL                       /**< Mode DIV32768 for CMU_LFAPRESC0 */
#define CMU_LFAPRESC0_RTC_DIV1                      (_CMU_LFAPRESC0_RTC_DIV1 << 0)     /**< Shifted mode DIV1 for CMU_LFAPRESC0 */
#define CMU_LFAPRESC0_RTC_DIV2                      (_CMU_LFAPRESC0_RTC_DIV2 << 0)     /**< Shifted mode DIV2 for CMU_LFAPRESC0 */
#define CMU_LFAPRESC0_RTC_DIV4                      (_CMU_LFAPRESC0_RTC_DIV4 << 0)     /**< Shifted mode DIV4 for CMU_LFAPRESC0 */
#define CMU_LFAPRESC0_RTC_DIV8                      (_CMU_LFAPRESC0_RTC_DIV8 << 0)     /**< Shifted mode DIV8 for CMU_LFAPRESC0 */
#define CMU_LFAPRESC0_RTC_DIV16                     (_CMU_LFAPRESC0_RTC_DIV16 << 0)    /**< Shifted mode DIV16 for CMU_LFAPRESC0 */
#define CMU_LFAPRESC0_RTC_DIV32                     (_CMU_LFAPRESC0_RTC_DIV32 << 0)    /**< Shifted mode DIV32 for CMU_LFAPRESC0 */
#define CMU_LFAPRESC0_RTC_DIV64                     (_CMU_LFAPRESC0_RTC_DIV64 << 0)    /**< Shifted mode DIV64 for CMU_LFAPRESC0 */
#define CMU_LFAPRESC0_RTC_DIV128                    (_CMU_LFAPRESC0_RTC_DIV128 << 0)   /**< Shifted mode DIV128 for CMU_LFAPRESC0 */
#define CMU_LFAPRESC0_RTC_DIV256                    (_CMU_LFAPRESC0_RTC_DIV256 << 0)   /**< Shifted mode DIV256 for CMU_LFAPRESC0 */
#define CMU_LFAPRESC0_RTC_DIV512                    (_CMU_LFAPRESC0_RTC_DIV512 << 0)   /**< Shifted mode DIV512 for CMU_LFAPRESC0 */
#define CMU_LFAPRESC0_RTC_DIV1024                   (_CMU_LFAPRESC0_RTC_DIV1024 << 0)  /**< Shifted mode DIV1024 for CMU_LFAPRESC0 */
#define CMU_LFAPRESC0_RTC_DIV2048                   (_CMU_LFAPRESC0_RTC_DIV2048 << 0)  /**< Shifted mode DIV2048 for CMU_LFAPRESC0 */
#define CMU_LFAPRESC0_RTC_DIV4096                   (_CMU_LFAPRESC0_RTC_DIV4096 << 0)  /**< Shifted mode DIV4096 for CMU_LFAPRESC0 */
#define CMU_LFAPRESC0_RTC_DIV8192                   (_CMU_LFAPRESC0_RTC_DIV8192 << 0)  /**< Shifted mode DIV8192 for CMU_LFAPRESC0 */
#define CMU_LFAPRESC0_RTC_DIV16384                  (_CMU_LFAPRESC0_RTC_DIV16384 << 0) /**< Shifted mode DIV16384 for CMU_LFAPRESC0 */
#define CMU_LFAPRESC0_RTC_DIV32768                  (_CMU_LFAPRESC0_RTC_DIV32768 << 0) /**< Shifted mode DIV32768 for CMU_LFAPRESC0 */

/* Bit fields for CMU OSCENCMD */
#define _CMU_OSCENCMD_RESETVALUE                    0x00000000UL                             /**< Default value for CMU_OSCENCMD */
#define _CMU_OSCENCMD_MASK                          0x00000FFFUL                             /**< Mask for CMU_OSCENCMD */
#define CMU_OSCENCMD_HFRCOEN                        (0x1UL << 0)                             /**< HFRCO Enable */
#define _CMU_OSCENCMD_HFRCOEN_SHIFT                 0                                        /**< Shift value for CMU_HFRCOEN */
#define _CMU_OSCENCMD_HFRCOEN_MASK                  0x1UL                                    /**< Bit mask for CMU_HFRCOEN */
#define _CMU_OSCENCMD_HFRCOEN_DEFAULT               0x00000000UL                             /**< Mode DEFAULT for CMU_OSCENCMD */
#define CMU_OSCENCMD_HFRCOEN_DEFAULT                (_CMU_OSCENCMD_HFRCOEN_DEFAULT << 0)     /**< Shifted mode DEFAULT for CMU_OSCENCMD */
#define CMU_OSCENCMD_HFRCODIS                       (0x1UL << 1)                             /**< HFRCO Disable */
#define _CMU_OSCENCMD_HFRCODIS_SHIFT                1                                        /**< Shift value for CMU_HFRCODIS */
#define _CMU_OSCENCMD_HFRCODIS_MASK                 0x2UL                                    /**< Bit mask for CMU_HFRCODIS */
#define _CMU_OSCENCMD_HFRCODIS_DEFAULT              0x00000000UL                             /**< Mode DEFAULT for CMU_OSCENCMD */
#define CMU_OSCENCMD_HFRCODIS_DEFAULT               (_CMU_OSCENCMD_HFRCODIS_DEFAULT << 1)    /**< Shifted mode DEFAULT for CMU_OSCENCMD */
#define CMU_OSCENCMD_HFXOEN                         (0x1UL << 2)                             /**< HFXO Enable */
#define _CMU_OSCENCMD_HFXOEN_SHIFT                  2                                        /**< Shift value for CMU_HFXOEN */
#define _CMU_OSCENCMD_HFXOEN_MASK                   0x4UL                                    /**< Bit mask for CMU_HFXOEN */
#define _CMU_OSCENCMD_HFXOEN_DEFAULT                0x00000000UL                             /**< Mode DEFAULT for CMU_OSCENCMD */
#define CMU_OSCENCMD_HFXOEN_DEFAULT                 (_CMU_OSCENCMD_HFXOEN_DEFAULT << 2)      /**< Shifted mode DEFAULT for CMU_OSCENCMD */
#define CMU_OSCENCMD_HFXODIS                        (0x1UL << 3)                             /**< HFXO Disable */
#define _CMU_OSCENCMD_HFXODIS_SHIFT                 3                                        /**< Shift value for CMU_HFXODIS */
#define _CMU_OSCENCMD_HFXODIS_MASK                  0x8UL                                    /**< Bit mask for CMU_HFXODIS */
#define _CMU_OSCENCMD_HFXODIS_DEFAULT               0x00000000UL                             /**< Mode DEFAULT for CMU_OSCENCMD */
#define CMU_OSCENCMD_HFXODIS_DEFAULT                (_CMU_OSCENCMD_HFXODIS_DEFAULT << 3)     /**< Shifted mode DEFAULT for CMU_OSCENCMD */
#define CMU_OSCENCMD_AUXHFRCOEN                     (0x1UL << 4)                             /**< AUXHFRCO Enable */
#define _CMU_OSCENCMD_AUXHFRCOEN_SHIFT              4                                        /**< Shift value for CMU_AUXHFRCOEN */
#define _CMU_OSCENCMD_AUXHFRCOEN_MASK               0x10UL                                   /**< Bit mask for CMU_AUXHFRCOEN */
#define _CMU_OSCENCMD_AUXHFRCOEN_DEFAULT            0x00000000UL                             /**< Mode DEFAULT for CMU_OSCENCMD */
#define CMU_OSCENCMD_AUXHFRCOEN_DEFAULT             (_CMU_OSCENCMD_AUXHFRCOEN_DEFAULT << 4)  /**< Shifted mode DEFAULT for CMU_OSCENCMD */
#define CMU_OSCENCMD_AUXHFRCODIS                    (0x1UL << 5)                             /**< AUXHFRCO Disable */
#define _CMU_OSCENCMD_AUXHFRCODIS_SHIFT             5                                        /**< Shift value for CMU_AUXHFRCODIS */
#define _CMU_OSCENCMD_AUXHFRCODIS_MASK              0x20UL                                   /**< Bit mask for CMU_AUXHFRCODIS */
#define _CMU_OSCENCMD_AUXHFRCODIS_DEFAULT           0x00000000UL                             /**< Mode DEFAULT for CMU_OSCENCMD */
#define CMU_OSCENCMD_AUXHFRCODIS_DEFAULT            (_CMU_OSCENCMD_AUXHFRCODIS_DEFAULT << 5) /**< Shifted mode DEFAULT for CMU_OSCENCMD */
#define CMU_OSCENCMD_LFRCOEN                        (0x1UL << 6)                             /**< LFRCO Enable */
#define _CMU_OSCENCMD_LFRCOEN_SHIFT                 6                                        /**< Shift value for CMU_LFRCOEN */
#define _CMU_OSCENCMD_LFRCOEN_MASK                  0x40UL                                   /**< Bit mask for CMU_LFRCOEN */
#define _CMU_OSCENCMD_LFRCOEN_DEFAULT               0x00000000UL                             /**< Mode DEFAULT for CMU_OSCENCMD */
#define CMU_OSCENCMD_LFRCOEN_DEFAULT                (_CMU_OSCENCMD_LFRCOEN_DEFAULT << 6)     /**< Shifted mode DEFAULT for CMU_OSCENCMD */
#define CMU_OSCENCMD_LFRCODIS                       (0x1UL << 7)                             /**< LFRCO Disable */
#define _CMU_OSCENCMD_LFRCODIS_SHIFT                7                                        /**< Shift value for CMU_LFRCODIS */
#define _CMU_OSCENCMD_LFRCODIS_MASK                 0x80UL                                   /**< Bit mask for CMU_LFRCODIS */
#define _CMU_OSCENCMD_LFRCODIS_DEFAULT              0x00000000UL                             /**< Mode DEFAULT for CMU_OSCENCMD */
#define CMU_OSCENCMD_LFRCODIS_DEFAULT               (_CMU_OSCENCMD_LFRCODIS_DEFAULT << 7)    /**< Shifted mode DEFAULT for CMU_OSCENCMD */
#define CMU_OSCENCMD_LFXOEN                         (0x1UL << 8)                             /**< LFXO Enable */
#define _CMU_OSCENCMD_LFXOEN_SHIFT                  8                                        /**< Shift value for CMU_LFXOEN */
#define _CMU_OSCENCMD_LFXOEN_MASK                   0x100UL                                  /**< Bit mask for CMU_LFXOEN */
#define _CMU_OSCENCMD_LFXOEN_DEFAULT                0x00000000UL                             /**< Mode DEFAULT for CMU_OSCENCMD */
#define CMU_OSCENCMD_LFXOEN_DEFAULT                 (_CMU_OSCENCMD_LFXOEN_DEFAULT << 8)      /**< Shifted mode DEFAULT for CMU_OSCENCMD */
#define CMU_OSCENCMD_LFXODIS                        (0x1UL << 9)                             /**< LFXO Disable */
#define _CMU_OSCENCMD_LFXODIS_SHIFT                 9                                        /**< Shift value for CMU_LFXODIS */
#define _CMU_OSCENCMD_LFXODIS_MASK                  0x200UL                                  /**< Bit mask for CMU_LFXODIS */
#define _CMU_OSCENCMD_LFXODIS_DEFAULT               0x00000000UL                             /**< Mode DEFAULT for CMU_OSCENCMD */
#define CMU_OSCENCMD_LFXODIS_DEFAULT                (_CMU_OSCENCMD_LFXODIS_DEFAULT << 9)     /**< Shifted mode DEFAULT for CMU_OSCENCMD */
#define CMU_OSCENCMD_USHFRCOEN                      (0x1UL << 10)                            /**< USHFRCO Enable */
#define _CMU_OSCENCMD_USHFRCOEN_SHIFT               10                                       /**< Shift value for CMU_USHFRCOEN */
#define _CMU_OSCENCMD_USHFRCOEN_MASK                0x400UL                                  /**< Bit mask for CMU_USHFRCOEN */
#define _CMU_OSCENCMD_USHFRCOEN_DEFAULT             0x00000000UL                             /**< Mode DEFAULT for CMU_OSCENCMD */
#define CMU_OSCENCMD_USHFRCOEN_DEFAULT              (_CMU_OSCENCMD_USHFRCOEN_DEFAULT << 10)  /**< Shifted mode DEFAULT for CMU_OSCENCMD */
#define CMU_OSCENCMD_USHFRCODIS                     (0x1UL << 11)                            /**< USHFRCO Disable */
#define _CMU_OSCENCMD_USHFRCODIS_SHIFT              11                                       /**< Shift value for CMU_USHFRCODIS */
#define _CMU_OSCENCMD_USHFRCODIS_MASK               0x800UL                                  /**< Bit mask for CMU_USHFRCODIS */
#define _CMU_OSCENCMD_USHFRCODIS_DEFAULT            0x00000000UL                             /**< Mode DEFAULT for CMU_OSCENCMD */
#define CMU_OSCENCMD_USHFRCODIS_DEFAULT             (_CMU_OSCENCMD_USHFRCODIS_DEFAULT << 11) /**< Shifted mode DEFAULT for CMU_OSCENCMD */

/* Bit fields for CMU HFPERCLKDIV */
#define _CMU_HFPERCLKDIV_RESETVALUE                 0x00000100UL                                 /**< Default value for CMU_HFPERCLKDIV */
#define _CMU_HFPERCLKDIV_MASK                       0x0000010FUL                                 /**< Mask for CMU_HFPERCLKDIV */
#define _CMU_HFPERCLKDIV_HFPERCLKDIV_SHIFT          0                                            /**< Shift value for CMU_HFPERCLKDIV */
#define _CMU_HFPERCLKDIV_HFPERCLKDIV_MASK           0xFUL                                        /**< Bit mask for CMU_HFPERCLKDIV */
#define _CMU_HFPERCLKDIV_HFPERCLKDIV_DEFAULT        0x00000000UL                                 /**< Mode DEFAULT for CMU_HFPERCLKDIV */
#define _CMU_HFPERCLKDIV_HFPERCLKDIV_HFCLK          0x00000000UL                                 /**< Mode HFCLK for CMU_HFPERCLKDIV */
#define _CMU_HFPERCLKDIV_HFPERCLKDIV_HFCLK2         0x00000001UL                                 /**< Mode HFCLK2 for CMU_HFPERCLKDIV */
#define _CMU_HFPERCLKDIV_HFPERCLKDIV_HFCLK4         0x00000002UL                                 /**< Mode HFCLK4 for CMU_HFPERCLKDIV */
#define _CMU_HFPERCLKDIV_HFPERCLKDIV_HFCLK8         0x00000003UL                                 /**< Mode HFCLK8 for CMU_HFPERCLKDIV */
#define _CMU_HFPERCLKDIV_HFPERCLKDIV_HFCLK16        0x00000004UL                                 /**< Mode HFCLK16 for CMU_HFPERCLKDIV */
#define _CMU_HFPERCLKDIV_HFPERCLKDIV_HFCLK32        0x00000005UL                                 /**< Mode HFCLK32 for CMU_HFPERCLKDIV */
#define _CMU_HFPERCLKDIV_HFPERCLKDIV_HFCLK64        0x00000006UL                                 /**< Mode HFCLK64 for CMU_HFPERCLKDIV */
#define _CMU_HFPERCLKDIV_HFPERCLKDIV_HFCLK128       0x00000007UL                                 /**< Mode HFCLK128 for CMU_HFPERCLKDIV */
#define _CMU_HFPERCLKDIV_HFPERCLKDIV_HFCLK256       0x00000008UL                                 /**< Mode HFCLK256 for CMU_HFPERCLKDIV */
#define _CMU_HFPERCLKDIV_HFPERCLKDIV_HFCLK512       0x00000009UL                                 /**< Mode HFCLK512 for CMU_HFPERCLKDIV */
#define CMU_HFPERCLKDIV_HFPERCLKDIV_DEFAULT         (_CMU_HFPERCLKDIV_HFPERCLKDIV_DEFAULT << 0)  /**< Shifted mode DEFAULT for CMU_HFPERCLKDIV */
#define CMU_HFPERCLKDIV_HFPERCLKDIV_HFCLK           (_CMU_HFPERCLKDIV_HFPERCLKDIV_HFCLK << 0)    /**< Shifted mode HFCLK for CMU_HFPERCLKDIV */
#define CMU_HFPERCLKDIV_HFPERCLKDIV_HFCLK2          (_CMU_HFPERCLKDIV_HFPERCLKDIV_HFCLK2 << 0)   /**< Shifted mode HFCLK2 for CMU_HFPERCLKDIV */
#define CMU_HFPERCLKDIV_HFPERCLKDIV_HFCLK4          (_CMU_HFPERCLKDIV_HFPERCLKDIV_HFCLK4 << 0)   /**< Shifted mode HFCLK4 for CMU_HFPERCLKDIV */
#define CMU_HFPERCLKDIV_HFPERCLKDIV_HFCLK8          (_CMU_HFPERCLKDIV_HFPERCLKDIV_HFCLK8 << 0)   /**< Shifted mode HFCLK8 for CMU_HFPERCLKDIV */
#define CMU_HFPERCLKDIV_HFPERCLKDIV_HFCLK16         (_CMU_HFPERCLKDIV_HFPERCLKDIV_HFCLK16 << 0)  /**< Shifted mode HFCLK16 for CMU_HFPERCLKDIV */
#define CMU_HFPERCLKDIV_HFPERCLKDIV_HFCLK32         (_CMU_HFPERCLKDIV_HFPERCLKDIV_HFCLK32 << 0)  /**< Shifted mode HFCLK32 for CMU_HFPERCLKDIV */
#define CMU_HFPERCLKDIV_HFPERCLKDIV_HFCLK64         (_CMU_HFPERCLKDIV_HFPERCLKDIV_HFCLK64 << 0)  /**< Shifted mode HFCLK64 for CMU_HFPERCLKDIV */
#define CMU_HFPERCLKDIV_HFPERCLKDIV_HFCLK128        (_CMU_HFPERCLKDIV_HFPERCLKDIV_HFCLK128 << 0) /**< Shifted mode HFCLK128 for CMU_HFPERCLKDIV */
#define CMU_HFPERCLKDIV_HFPERCLKDIV_HFCLK256        (_CMU_HFPERCLKDIV_HFPERCLKDIV_HFCLK256 << 0) /**< Shifted mode HFCLK256 for CMU_HFPERCLKDIV */
#define CMU_HFPERCLKDIV_HFPERCLKDIV_HFCLK512        (_CMU_HFPERCLKDIV_HFPERCLKDIV_HFCLK512 << 0) /**< Shifted mode HFCLK512 for CMU_HFPERCLKDIV */
#define CMU_HFPERCLKDIV_HFPERCLKEN                  (0x1UL << 8)                                 /**< HFPERCLK Enable */
#define _CMU_HFPERCLKDIV_HFPERCLKEN_SHIFT           8                                            /**< Shift value for CMU_HFPERCLKEN */
#define _CMU_HFPERCLKDIV_HFPERCLKEN_MASK            0x100UL                                      /**< Bit mask for CMU_HFPERCLKEN */
#define _CMU_HFPERCLKDIV_HFPERCLKEN_DEFAULT         0x00000001UL                                 /**< Mode DEFAULT for CMU_HFPERCLKDIV */
#define CMU_HFPERCLKDIV_HFPERCLKEN_DEFAULT          (_CMU_HFPERCLKDIV_HFPERCLKEN_DEFAULT << 8)   /**< Shifted mode DEFAULT for CMU_HFPERCLKDIV */

/* Bit fields for CMU HFCORECLKEN0 */
#define _CMU_HFCORECLKEN0_RESETVALUE                0x00000000UL                          /**< Default value for CMU_HFCORECLKEN0 */
#define _CMU_HFCORECLKEN0_MASK                      0x0000001FUL                          /**< Mask for CMU_HFCORECLKEN0 */
#define CMU_HFCORECLKEN0_AES                        (0x1UL << 0)                          /**< Advanced Encryption Standard Accelerator Clock Enable */
#define _CMU_HFCORECLKEN0_AES_SHIFT                 0                                     /**< Shift value for CMU_AES */
#define _CMU_HFCORECLKEN0_AES_MASK                  0x1UL                                 /**< Bit mask for CMU_AES */
#define _CMU_HFCORECLKEN0_AES_DEFAULT               0x00000000UL                          /**< Mode DEFAULT for CMU_HFCORECLKEN0 */
#define CMU_HFCORECLKEN0_AES_DEFAULT                (_CMU_HFCORECLKEN0_AES_DEFAULT << 0)  /**< Shifted mode DEFAULT for CMU_HFCORECLKEN0 */
#define CMU_HFCORECLKEN0_DMA                        (0x1UL << 1)                          /**< Direct Memory Access Controller Clock Enable */
#define _CMU_HFCORECLKEN0_DMA_SHIFT                 1                                     /**< Shift value for CMU_DMA */
#define _CMU_HFCORECLKEN0_DMA_MASK                  0x2UL                                 /**< Bit mask for CMU_DMA */
#define _CMU_HFCORECLKEN0_DMA_DEFAULT               0x00000000UL                          /**< Mode DEFAULT for CMU_HFCORECLKEN0 */
#define CMU_HFCORECLKEN0_DMA_DEFAULT                (_CMU_HFCORECLKEN0_DMA_DEFAULT << 1)  /**< Shifted mode DEFAULT for CMU_HFCORECLKEN0 */
#define CMU_HFCORECLKEN0_LE                         (0x1UL << 2)                          /**< Low Energy Peripheral Interface Clock Enable */
#define _CMU_HFCORECLKEN0_LE_SHIFT                  2                                     /**< Shift value for CMU_LE */
#define _CMU_HFCORECLKEN0_LE_MASK                   0x4UL                                 /**< Bit mask for CMU_LE */
#define _CMU_HFCORECLKEN0_LE_DEFAULT                0x00000000UL                          /**< Mode DEFAULT for CMU_HFCORECLKEN0 */
#define CMU_HFCORECLKEN0_LE_DEFAULT                 (_CMU_HFCORECLKEN0_LE_DEFAULT << 2)   /**< Shifted mode DEFAULT for CMU_HFCORECLKEN0 */
#define CMU_HFCORECLKEN0_USBC                       (0x1UL << 3)                          /**< Universal Serial Bus Interface Core Clock Enable */
#define _CMU_HFCORECLKEN0_USBC_SHIFT                3                                     /**< Shift value for CMU_USBC */
#define _CMU_HFCORECLKEN0_USBC_MASK                 0x8UL                                 /**< Bit mask for CMU_USBC */
#define _CMU_HFCORECLKEN0_USBC_DEFAULT              0x00000000UL                          /**< Mode DEFAULT for CMU_HFCORECLKEN0 */
#define CMU_HFCORECLKEN0_USBC_DEFAULT               (_CMU_HFCORECLKEN0_USBC_DEFAULT << 3) /**< Shifted mode DEFAULT for CMU_HFCORECLKEN0 */
#define CMU_HFCORECLKEN0_USB                        (0x1UL << 4)                          /**< Universal Serial Bus Interface Clock Enable */
#define _CMU_HFCORECLKEN0_USB_SHIFT                 4                                     /**< Shift value for CMU_USB */
#define _CMU_HFCORECLKEN0_USB_MASK                  0x10UL                                /**< Bit mask for CMU_USB */
#define _CMU_HFCORECLKEN0_USB_DEFAULT               0x00000000UL                          /**< Mode DEFAULT for CMU_HFCORECLKEN0 */
#define CMU_HFCORECLKEN0_USB_DEFAULT (_CMU_HFCORECLKEN0_USB_DEFAULT << 4) /**< Shifted mode DEFAULT for CMU_HFCORECLKEN0 */

/* Bit fields for CMU LFCLKSEL */
#define _CMU_LFCLKSEL_RESETVALUE                    0x00000015UL                             /**< Default value for CMU_LFCLKSEL */
#define _CMU_LFCLKSEL_MASK                          0x0011003FUL                             /**< Mask for CMU_LFCLKSEL */
#define _CMU_LFCLKSEL_LFA_SHIFT                     0                                        /**< Shift value for CMU_LFA */
#define _CMU_LFCLKSEL_LFA_MASK                      0x3UL                                    /**< Bit mask for CMU_LFA */
#define _CMU_LFCLKSEL_LFA_DISABLED                  0x00000000UL                             /**< Mode DISABLED for CMU_LFCLKSEL */
#define _CMU_LFCLKSEL_LFA_DEFAULT                   0x00000001UL                             /**< Mode DEFAULT for CMU_LFCLKSEL */
#define _CMU_LFCLKSEL_LFA_LFRCO                     0x00000001UL                             /**< Mode LFRCO for CMU_LFCLKSEL */
#define _CMU_LFCLKSEL_LFA_LFXO                      0x00000002UL                             /**< Mode LFXO for CMU_LFCLKSEL */
#define _CMU_LFCLKSEL_LFA_HFCORECLKLEDIV2           0x00000003UL                             /**< Mode HFCORECLKLEDIV2 for CMU_LFCLKSEL */
#define CMU_LFCLKSEL_LFA_DISABLED                   (_CMU_LFCLKSEL_LFA_DISABLED << 0)        /**< Shifted mode DISABLED for CMU_LFCLKSEL */
#define CMU_LFCLKSEL_LFA_DEFAULT                    (_CMU_LFCLKSEL_LFA_DEFAULT << 0)         /**< Shifted mode DEFAULT for CMU_LFCLKSEL */
#define CMU_LFCLKSEL_LFA_LFRCO                      (_CMU_LFCLKSEL_LFA_LFRCO << 0)           /**< Shifted mode LFRCO for CMU_LFCLKSEL */
#define CMU_LFCLKSEL_LFA_LFXO                       (_CMU_LFCLKSEL_LFA_LFXO << 0)            /**< Shifted mode LFXO for CMU_LFCLKSEL */
#define CMU_LFCLKSEL_LFA_HFCORECLKLEDIV2            (_CMU_LFCLKSEL_LFA_HFCORECLKLEDIV2 << 0) /**< Shifted mode HFCORECLKLEDIV2 for CMU_LFCLKSEL */
#define _CMU_LFCLKSEL_LFB_SHIFT                     2                                        /**< Shift value for CMU_LFB */
#define _CMU_LFCLKSEL_LFB_MASK                      0xCUL                                    /**< Bit mask for CMU_LFB */
#define _CMU_LFCLKSEL_LFB_DISABLED                  0x00000000UL                             /**< Mode DISABLED for CMU_LFCLKSEL */
#define _CMU_LFCLKSEL_LFB_DEFAULT                   0x00000001UL                             /**< Mode DEFAULT for CMU_LFCLKSEL */
#define _CMU_LFCLKSEL_LFB_LFRCO                     0x00000001UL                             /**< Mode LFRCO for CMU_LFCLKSEL */
#define _CMU_LFCLKSEL_LFB_LFXO                      0x00000002UL                             /**< Mode LFXO for CMU_LFCLKSEL */
#define _CMU_LFCLKSEL_LFB_HFCORECLKLEDIV2           0x00000003UL                             /**< Mode HFCORECLKLEDIV2 for CMU_LFCLKSEL */
#define CMU_LFCLKSEL_LFB_DISABLED                   (_CMU_LFCLKSEL_LFB_DISABLED << 2)        /**< Shifted mode DISABLED for CMU_LFCLKSEL */
#define CMU_LFCLKSEL_LFB_DEFAULT                    (_CMU_LFCLKSEL_LFB_DEFAULT << 2)         /**< Shifted mode DEFAULT for CMU_LFCLKSEL */
#define CMU_LFCLKSEL_LFB_LFRCO                      (_CMU_LFCLKSEL_LFB_LFRCO << 2)           /**< Shifted mode LFRCO for CMU_LFCLKSEL */
#define CMU_LFCLKSEL_LFB_LFXO                       (_CMU_LFCLKSEL_LFB_LFXO << 2)            /**< Shifted mode LFXO for CMU_LFCLKSEL */
#define CMU_LFCLKSEL_LFB_HFCORECLKLEDIV2            (_CMU_LFCLKSEL_LFB_HFCORECLKLEDIV2 << 2) /**< Shifted mode HFCORECLKLEDIV2 for CMU_LFCLKSEL */
#define _CMU_LFCLKSEL_LFC_SHIFT                     4                                        /**< Shift value for CMU_LFC */
#define _CMU_LFCLKSEL_LFC_MASK                      0x30UL                                   /**< Bit mask for CMU_LFC */
#define _CMU_LFCLKSEL_LFC_DISABLED                  0x00000000UL                             /**< Mode DISABLED for CMU_LFCLKSEL */
#define _CMU_LFCLKSEL_LFC_DEFAULT                   0x00000001UL                             /**< Mode DEFAULT for CMU_LFCLKSEL */
#define _CMU_LFCLKSEL_LFC_LFRCO                     0x00000001UL                             /**< Mode LFRCO for CMU_LFCLKSEL */
#define _CMU_LFCLKSEL_LFC_LFXO                      0x00000002UL                             /**< Mode LFXO for CMU_LFCLKSEL */
#define CMU_LFCLKSEL_LFC_DISABLED                   (_CMU_LFCLKSEL_LFC_DISABLED << 4)        /**< Shifted mode DISABLED for CMU_LFCLKSEL */
#define CMU_LFCLKSEL_LFC_DEFAULT                    (_CMU_LFCLKSEL_LFC_DEFAULT << 4)         /**< Shifted mode DEFAULT for CMU_LFCLKSEL */
#define CMU_LFCLKSEL_LFC_LFRCO                      (_CMU_LFCLKSEL_LFC_LFRCO << 4)           /**< Shifted mode LFRCO for CMU_LFCLKSEL */
#define CMU_LFCLKSEL_LFC_LFXO                       (_CMU_LFCLKSEL_LFC_LFXO << 4)            /**< Shifted mode LFXO for CMU_LFCLKSEL */
#define CMU_LFCLKSEL_LFAE                           (0x1UL << 16)                            /**< Clock Select for LFA Extended */
#define _CMU_LFCLKSEL_LFAE_SHIFT                    16                                       /**< Shift value for CMU_LFAE */
#define _CMU_LFCLKSEL_LFAE_MASK                     0x10000UL                                /**< Bit mask for CMU_LFAE */
#define _CMU_LFCLKSEL_LFAE_DEFAULT                  0x00000000UL                             /**< Mode DEFAULT for CMU_LFCLKSEL */
#define _CMU_LFCLKSEL_LFAE_DISABLED                 0x00000000UL                             /**< Mode DISABLED for CMU_LFCLKSEL */
#define _CMU_LFCLKSEL_LFAE_ULFRCO                   0x00000001UL                             /**< Mode ULFRCO for CMU_LFCLKSEL */
#define CMU_LFCLKSEL_LFAE_DEFAULT                   (_CMU_LFCLKSEL_LFAE_DEFAULT << 16)       /**< Shifted mode DEFAULT for CMU_LFCLKSEL */
#define CMU_LFCLKSEL_LFAE_DISABLED                  (_CMU_LFCLKSEL_LFAE_DISABLED << 16)      /**< Shifted mode DISABLED for CMU_LFCLKSEL */
#define CMU_LFCLKSEL_LFAE_ULFRCO                    (_CMU_LFCLKSEL_LFAE_ULFRCO << 16)        /**< Shifted mode ULFRCO for CMU_LFCLKSEL */
#define CMU_LFCLKSEL_LFBE                           (0x1UL << 20)                            /**< Clock Select for LFB Extended */
#define _CMU_LFCLKSEL_LFBE_SHIFT                    20                                       /**< Shift value for CMU_LFBE */
#define _CMU_LFCLKSEL_LFBE_MASK                     0x100000UL                               /**< Bit mask for CMU_LFBE */
#define _CMU_LFCLKSEL_LFBE_DEFAULT                  0x00000000UL                             /**< Mode DEFAULT for CMU_LFCLKSEL */
#define _CMU_LFCLKSEL_LFBE_DISABLED                 0x00000000UL                             /**< Mode DISABLED for CMU_LFCLKSEL */
#define _CMU_LFCLKSEL_LFBE_ULFRCO                   0x00000001UL                             /**< Mode ULFRCO for CMU_LFCLKSEL */
#define CMU_LFCLKSEL_LFBE_DEFAULT                   (_CMU_LFCLKSEL_LFBE_DEFAULT << 20)       /**< Shifted mode DEFAULT for CMU_LFCLKSEL */
#define CMU_LFCLKSEL_LFBE_DISABLED                  (_CMU_LFCLKSEL_LFBE_DISABLED << 20)      /**< Shifted mode DISABLED for CMU_LFCLKSEL */
#define CMU_LFCLKSEL_LFBE_ULFRCO                    (_CMU_LFCLKSEL_LFBE_ULFRCO << 20)        /**< Shifted mode ULFRCO for CMU_LFCLKSEL */

/* Bit fields for CMU SYNCBUSY */
#define _CMU_SYNCBUSY_RESETVALUE                    0x00000000UL                           /**< Default value for CMU_SYNCBUSY */
#define _CMU_SYNCBUSY_MASK                          0x00000155UL                           /**< Mask for CMU_SYNCBUSY */
#define CMU_SYNCBUSY_LFACLKEN0                      (0x1UL << 0)                           /**< Low Frequency A Clock Enable 0 Busy */
#define _CMU_SYNCBUSY_LFACLKEN0_SHIFT               0                                      /**< Shift value for CMU_LFACLKEN0 */
#define _CMU_SYNCBUSY_LFACLKEN0_MASK                0x1UL                                  /**< Bit mask for CMU_LFACLKEN0 */
#define _CMU_SYNCBUSY_LFACLKEN0_DEFAULT             0x00000000UL                           /**< Mode DEFAULT for CMU_SYNCBUSY */
#define CMU_SYNCBUSY_LFACLKEN0_DEFAULT              (_CMU_SYNCBUSY_LFACLKEN0_DEFAULT << 0) /**< Shifted mode DEFAULT for CMU_SYNCBUSY */
#define CMU_SYNCBUSY_LFAPRESC0                      (0x1UL << 2)                           /**< Low Frequency A Prescaler 0 Busy */
#define _CMU_SYNCBUSY_LFAPRESC0_SHIFT               2                                      /**< Shift value for CMU_LFAPRESC0 */
#define _CMU_SYNCBUSY_LFAPRESC0_MASK                0x4UL                                  /**< Bit mask for CMU_LFAPRESC0 */
#define _CMU_SYNCBUSY_LFAPRESC0_DEFAULT             0x00000000UL                           /**< Mode DEFAULT for CMU_SYNCBUSY */
#define CMU_SYNCBUSY_LFAPRESC0_DEFAULT              (_CMU_SYNCBUSY_LFAPRESC0_DEFAULT << 2) /**< Shifted mode DEFAULT for CMU_SYNCBUSY */
#define CMU_SYNCBUSY_LFBCLKEN0                      (0x1UL << 4)                           /**< Low Frequency B Clock Enable 0 Busy */
#define _CMU_SYNCBUSY_LFBCLKEN0_SHIFT               4                                      /**< Shift value for CMU_LFBCLKEN0 */
#define _CMU_SYNCBUSY_LFBCLKEN0_MASK                0x10UL                                 /**< Bit mask for CMU_LFBCLKEN0 */
#define _CMU_SYNCBUSY_LFBCLKEN0_DEFAULT             0x00000000UL                           /**< Mode DEFAULT for CMU_SYNCBUSY */
#define CMU_SYNCBUSY_LFBCLKEN0_DEFAULT              (_CMU_SYNCBUSY_LFBCLKEN0_DEFAULT << 4) /**< Shifted mode DEFAULT for CMU_SYNCBUSY */
#define CMU_SYNCBUSY_LFBPRESC0                      (0x1UL << 6)                           /**< Low Frequency B Prescaler 0 Busy */
#define _CMU_SYNCBUSY_LFBPRESC0_SHIFT               6                                      /**< Shift value for CMU_LFBPRESC0 */
#define _CMU_SYNCBUSY_LFBPRESC0_MASK                0x40UL                                 /**< Bit mask for CMU_LFBPRESC0 */
#define _CMU_SYNCBUSY_LFBPRESC0_DEFAULT             0x00000000UL                           /**< Mode DEFAULT for CMU_SYNCBUSY */
#define CMU_SYNCBUSY_LFBPRESC0_DEFAULT              (_CMU_SYNCBUSY_LFBPRESC0_DEFAULT << 6) /**< Shifted mode DEFAULT for CMU_SYNCBUSY */
#define CMU_SYNCBUSY_LFCCLKEN0                      (0x1UL << 8)                           /**< Low Frequency C Clock Enable 0 Busy */
#define _CMU_SYNCBUSY_LFCCLKEN0_SHIFT               8                                      /**< Shift value for CMU_LFCCLKEN0 */
#define _CMU_SYNCBUSY_LFCCLKEN0_MASK                0x100UL                                /**< Bit mask for CMU_LFCCLKEN0 */
#define _CMU_SYNCBUSY_LFCCLKEN0_DEFAULT             0x00000000UL                           /**< Mode DEFAULT for CMU_SYNCBUSY */
#define CMU_SYNCBUSY_LFCCLKEN0_DEFAULT              (_CMU_SYNCBUSY_LFCCLKEN0_DEFAULT << 8) /**< Shifted mode DEFAULT for CMU_SYNCBUSY */

/* Bit fields for CMU CMD */
#define _CMU_CMD_RESETVALUE                         0x00000000UL                         /**< Default value for CMU_CMD */
#define _CMU_CMD_MASK                               0x000000FFUL                         /**< Mask for CMU_CMD */
#define _CMU_CMD_HFCLKSEL_SHIFT                     0                                    /**< Shift value for CMU_HFCLKSEL */
#define _CMU_CMD_HFCLKSEL_MASK                      0x7UL                                /**< Bit mask for CMU_HFCLKSEL */
#define _CMU_CMD_HFCLKSEL_DEFAULT                   0x00000000UL                         /**< Mode DEFAULT for CMU_CMD */
#define _CMU_CMD_HFCLKSEL_HFRCO                     0x00000001UL                         /**< Mode HFRCO for CMU_CMD */
#define _CMU_CMD_HFCLKSEL_HFXO                      0x00000002UL                         /**< Mode HFXO for CMU_CMD */
#define _CMU_CMD_HFCLKSEL_LFRCO                     0x00000003UL                         /**< Mode LFRCO for CMU_CMD */
#define _CMU_CMD_HFCLKSEL_LFXO                      0x00000004UL                         /**< Mode LFXO for CMU_CMD */
#define _CMU_CMD_HFCLKSEL_USHFRCODIV2               0x00000005UL                         /**< Mode USHFRCODIV2 for CMU_CMD */
#define CMU_CMD_HFCLKSEL_DEFAULT                    (_CMU_CMD_HFCLKSEL_DEFAULT << 0)     /**< Shifted mode DEFAULT for CMU_CMD */
#define CMU_CMD_HFCLKSEL_HFRCO                      (_CMU_CMD_HFCLKSEL_HFRCO << 0)       /**< Shifted mode HFRCO for CMU_CMD */
#define CMU_CMD_HFCLKSEL_HFXO                       (_CMU_CMD_HFCLKSEL_HFXO << 0)        /**< Shifted mode HFXO for CMU_CMD */
#define CMU_CMD_HFCLKSEL_LFRCO                      (_CMU_CMD_HFCLKSEL_LFRCO << 0)       /**< Shifted mode LFRCO for CMU_CMD */
#define CMU_CMD_HFCLKSEL_LFXO                       (_CMU_CMD_HFCLKSEL_LFXO << 0)        /**< Shifted mode LFXO for CMU_CMD */
#define CMU_CMD_HFCLKSEL_USHFRCODIV2                (_CMU_CMD_HFCLKSEL_USHFRCODIV2 << 0) /**< Shifted mode USHFRCODIV2 for CMU_CMD */
#define CMU_CMD_CALSTART                            (0x1UL << 3)                         /**< Calibration Start */
#define _CMU_CMD_CALSTART_SHIFT                     3                                    /**< Shift value for CMU_CALSTART */
#define _CMU_CMD_CALSTART_MASK                      0x8UL                                /**< Bit mask for CMU_CALSTART */
#define _CMU_CMD_CALSTART_DEFAULT                   0x00000000UL                         /**< Mode DEFAULT for CMU_CMD */
#define CMU_CMD_CALSTART_DEFAULT                    (_CMU_CMD_CALSTART_DEFAULT << 3)     /**< Shifted mode DEFAULT for CMU_CMD */
#define CMU_CMD_CALSTOP                             (0x1UL << 4)                         /**< Calibration Stop */
#define _CMU_CMD_CALSTOP_SHIFT                      4                                    /**< Shift value for CMU_CALSTOP */
#define _CMU_CMD_CALSTOP_MASK                       0x10UL                               /**< Bit mask for CMU_CALSTOP */
#define _CMU_CMD_CALSTOP_DEFAULT                    0x00000000UL                         /**< Mode DEFAULT for CMU_CMD */
#define CMU_CMD_CALSTOP_DEFAULT                     (_CMU_CMD_CALSTOP_DEFAULT << 4)      /**< Shifted mode DEFAULT for CMU_CMD */
#define _CMU_CMD_USBCCLKSEL_SHIFT                   5                                    /**< Shift value for CMU_USBCCLKSEL */
#define _CMU_CMD_USBCCLKSEL_MASK                    0xE0UL                               /**< Bit mask for CMU_USBCCLKSEL */
#define _CMU_CMD_USBCCLKSEL_DEFAULT                 0x00000000UL                         /**< Mode DEFAULT for CMU_CMD */
#define _CMU_CMD_USBCCLKSEL_LFXO                    0x00000002UL                         /**< Mode LFXO for CMU_CMD */
#define _CMU_CMD_USBCCLKSEL_LFRCO                   0x00000003UL                         /**< Mode LFRCO for CMU_CMD */
#define _CMU_CMD_USBCCLKSEL_USHFRCO                 0x00000004UL                         /**< Mode USHFRCO for CMU_CMD */
#define CMU_CMD_USBCCLKSEL_DEFAULT                  (_CMU_CMD_USBCCLKSEL_DEFAULT << 5)   /**< Shifted mode DEFAULT for CMU_CMD */
#define CMU_CMD_USBCCLKSEL_LFXO                     (_CMU_CMD_USBCCLKSEL_LFXO << 5)      /**< Shifted mode LFXO for CMU_CMD */
#define CMU_CMD_USBCCLKSEL_LFRCO                    (_CMU_CMD_USBCCLKSEL_LFRCO << 5)     /**< Shifted mode LFRCO for CMU_CMD */
#define CMU_CMD_USBCCLKSEL_USHFRCO                  (_CMU_CMD_USBCCLKSEL_USHFRCO << 5)   /**< Shifted mode USHFRCO for CMU_CMD */

/* Bit fields for RMU CTRL */
#define _RMU_CTRL_RESETVALUE                 0x00000000UL                        /**< Default value for RMU_CTRL */
#define _RMU_CTRL_MASK                       0x00000001UL                        /**< Mask for RMU_CTRL */
#define RMU_CTRL_LOCKUPRDIS                  (0x1UL << 0)                        /**< Lockup Reset Disable */
#define _RMU_CTRL_LOCKUPRDIS_SHIFT           0                                   /**< Shift value for RMU_LOCKUPRDIS */
#define _RMU_CTRL_LOCKUPRDIS_MASK            0x1UL                               /**< Bit mask for RMU_LOCKUPRDIS */
#define _RMU_CTRL_LOCKUPRDIS_DEFAULT         0x00000000UL                        /**< Mode DEFAULT for RMU_CTRL */
#define RMU_CTRL_LOCKUPRDIS_DEFAULT          (_RMU_CTRL_LOCKUPRDIS_DEFAULT << 0) /**< Shifted mode DEFAULT for RMU_CTRL */

/* Bit fields for RMU RSTCAUSE */
#define _RMU_RSTCAUSE_RESETVALUE             0x00000000UL                             /**< Default value for RMU_RSTCAUSE */
#define _RMU_RSTCAUSE_MASK                   0x000007FFUL                             /**< Mask for RMU_RSTCAUSE */
#define RMU_RSTCAUSE_PORST                   (0x1UL << 0)                             /**< Power On Reset */
#define _RMU_RSTCAUSE_PORST_SHIFT            0                                        /**< Shift value for RMU_PORST */
#define _RMU_RSTCAUSE_PORST_MASK             0x1UL                                    /**< Bit mask for RMU_PORST */
#define _RMU_RSTCAUSE_PORST_DEFAULT          0x00000000UL                             /**< Mode DEFAULT for RMU_RSTCAUSE */
#define RMU_RSTCAUSE_PORST_DEFAULT           (_RMU_RSTCAUSE_PORST_DEFAULT << 0)       /**< Shifted mode DEFAULT for RMU_RSTCAUSE */
#define RMU_RSTCAUSE_BODUNREGRST             (0x1UL << 1)                             /**< Brown Out Detector Unregulated Domain Reset */
#define _RMU_RSTCAUSE_BODUNREGRST_SHIFT      1                                        /**< Shift value for RMU_BODUNREGRST */
#define _RMU_RSTCAUSE_BODUNREGRST_MASK       0x2UL                                    /**< Bit mask for RMU_BODUNREGRST */
#define _RMU_RSTCAUSE_BODUNREGRST_DEFAULT    0x00000000UL                             /**< Mode DEFAULT for RMU_RSTCAUSE */
#define RMU_RSTCAUSE_BODUNREGRST_DEFAULT     (_RMU_RSTCAUSE_BODUNREGRST_DEFAULT << 1) /**< Shifted mode DEFAULT for RMU_RSTCAUSE */
#define RMU_RSTCAUSE_BODREGRST               (0x1UL << 2)                             /**< Brown Out Detector Regulated Domain Reset */
#define _RMU_RSTCAUSE_BODREGRST_SHIFT        2                                        /**< Shift value for RMU_BODREGRST */
#define _RMU_RSTCAUSE_BODREGRST_MASK         0x4UL                                    /**< Bit mask for RMU_BODREGRST */
#define _RMU_RSTCAUSE_BODREGRST_DEFAULT      0x00000000UL                             /**< Mode DEFAULT for RMU_RSTCAUSE */
#define RMU_RSTCAUSE_BODREGRST_DEFAULT       (_RMU_RSTCAUSE_BODREGRST_DEFAULT << 2)   /**< Shifted mode DEFAULT for RMU_RSTCAUSE */
#define RMU_RSTCAUSE_EXTRST                  (0x1UL << 3)                             /**< External Pin Reset */
#define _RMU_RSTCAUSE_EXTRST_SHIFT           3                                        /**< Shift value for RMU_EXTRST */
#define _RMU_RSTCAUSE_EXTRST_MASK            0x8UL                                    /**< Bit mask for RMU_EXTRST */
#define _RMU_RSTCAUSE_EXTRST_DEFAULT         0x00000000UL                             /**< Mode DEFAULT for RMU_RSTCAUSE */
#define RMU_RSTCAUSE_EXTRST_DEFAULT          (_RMU_RSTCAUSE_EXTRST_DEFAULT << 3)      /**< Shifted mode DEFAULT for RMU_RSTCAUSE */
#define RMU_RSTCAUSE_WDOGRST                 (0x1UL << 4)                             /**< Watchdog Reset */
#define _RMU_RSTCAUSE_WDOGRST_SHIFT          4                                        /**< Shift value for RMU_WDOGRST */
#define _RMU_RSTCAUSE_WDOGRST_MASK           0x10UL                                   /**< Bit mask for RMU_WDOGRST */
#define _RMU_RSTCAUSE_WDOGRST_DEFAULT        0x00000000UL                             /**< Mode DEFAULT for RMU_RSTCAUSE */
#define RMU_RSTCAUSE_WDOGRST_DEFAULT         (_RMU_RSTCAUSE_WDOGRST_DEFAULT << 4)     /**< Shifted mode DEFAULT for RMU_RSTCAUSE */
#define RMU_RSTCAUSE_LOCKUPRST               (0x1UL << 5)                             /**< LOCKUP Reset */
#define _RMU_RSTCAUSE_LOCKUPRST_SHIFT        5                                        /**< Shift value for RMU_LOCKUPRST */
#define _RMU_RSTCAUSE_LOCKUPRST_MASK         0x20UL                                   /**< Bit mask for RMU_LOCKUPRST */
#define _RMU_RSTCAUSE_LOCKUPRST_DEFAULT      0x00000000UL                             /**< Mode DEFAULT for RMU_RSTCAUSE */
#define RMU_RSTCAUSE_LOCKUPRST_DEFAULT       (_RMU_RSTCAUSE_LOCKUPRST_DEFAULT << 5)   /**< Shifted mode DEFAULT for RMU_RSTCAUSE */
#define RMU_RSTCAUSE_SYSREQRST               (0x1UL << 6)                             /**< System Request Reset */
#define _RMU_RSTCAUSE_SYSREQRST_SHIFT        6                                        /**< Shift value for RMU_SYSREQRST */
#define _RMU_RSTCAUSE_SYSREQRST_MASK         0x40UL                                   /**< Bit mask for RMU_SYSREQRST */
#define _RMU_RSTCAUSE_SYSREQRST_DEFAULT      0x00000000UL                             /**< Mode DEFAULT for RMU_RSTCAUSE */
#define RMU_RSTCAUSE_SYSREQRST_DEFAULT       (_RMU_RSTCAUSE_SYSREQRST_DEFAULT << 6)   /**< Shifted mode DEFAULT for RMU_RSTCAUSE */
#define RMU_RSTCAUSE_EM4RST                  (0x1UL << 7)                             /**< EM4 Reset */
#define _RMU_RSTCAUSE_EM4RST_SHIFT           7                                        /**< Shift value for RMU_EM4RST */
#define _RMU_RSTCAUSE_EM4RST_MASK            0x80UL                                   /**< Bit mask for RMU_EM4RST */
#define _RMU_RSTCAUSE_EM4RST_DEFAULT         0x00000000UL                             /**< Mode DEFAULT for RMU_RSTCAUSE */
#define RMU_RSTCAUSE_EM4RST_DEFAULT          (_RMU_RSTCAUSE_EM4RST_DEFAULT << 7)      /**< Shifted mode DEFAULT for RMU_RSTCAUSE */
#define RMU_RSTCAUSE_EM4WURST                (0x1UL << 8)                             /**< EM4 Wake-up Reset */
#define _RMU_RSTCAUSE_EM4WURST_SHIFT         8                                        /**< Shift value for RMU_EM4WURST */
#define _RMU_RSTCAUSE_EM4WURST_MASK          0x100UL                                  /**< Bit mask for RMU_EM4WURST */
#define _RMU_RSTCAUSE_EM4WURST_DEFAULT       0x00000000UL                             /**< Mode DEFAULT for RMU_RSTCAUSE */
#define RMU_RSTCAUSE_EM4WURST_DEFAULT        (_RMU_RSTCAUSE_EM4WURST_DEFAULT << 8)    /**< Shifted mode DEFAULT for RMU_RSTCAUSE */
#define RMU_RSTCAUSE_BODAVDD0                (0x1UL << 9)                             /**< AVDD0 Bod Reset */
#define _RMU_RSTCAUSE_BODAVDD0_SHIFT         9                                        /**< Shift value for RMU_BODAVDD0 */
#define _RMU_RSTCAUSE_BODAVDD0_MASK          0x200UL                                  /**< Bit mask for RMU_BODAVDD0 */
#define _RMU_RSTCAUSE_BODAVDD0_DEFAULT       0x00000000UL                             /**< Mode DEFAULT for RMU_RSTCAUSE */
#define RMU_RSTCAUSE_BODAVDD0_DEFAULT        (_RMU_RSTCAUSE_BODAVDD0_DEFAULT << 9)    /**< Shifted mode DEFAULT for RMU_RSTCAUSE */
#define RMU_RSTCAUSE_BODAVDD1                (0x1UL << 10)                            /**< AVDD1 Bod Reset */
#define _RMU_RSTCAUSE_BODAVDD1_SHIFT         10                                       /**< Shift value for RMU_BODAVDD1 */
#define _RMU_RSTCAUSE_BODAVDD1_MASK          0x400UL                                  /**< Bit mask for RMU_BODAVDD1 */
#define _RMU_RSTCAUSE_BODAVDD1_DEFAULT       0x00000000UL                             /**< Mode DEFAULT for RMU_RSTCAUSE */
#define RMU_RSTCAUSE_BODAVDD1_DEFAULT        (_RMU_RSTCAUSE_BODAVDD1_DEFAULT << 10)   /**< Shifted mode DEFAULT for RMU_RSTCAUSE */

/* Bit fields for RMU CMD */
#define _RMU_CMD_RESETVALUE                  0x00000000UL                  /**< Default value for RMU_CMD */
#define _RMU_CMD_MASK                        0x00000001UL                  /**< Mask for RMU_CMD */
#define RMU_CMD_RCCLR                        (0x1UL << 0)                  /**< Reset Cause Clear */
#define _RMU_CMD_RCCLR_SHIFT                 0                             /**< Shift value for RMU_RCCLR */
#define _RMU_CMD_RCCLR_MASK                  0x1UL                         /**< Bit mask for RMU_RCCLR */
#define _RMU_CMD_RCCLR_DEFAULT               0x00000000UL                  /**< Mode DEFAULT for RMU_CMD */
#define RMU_CMD_RCCLR_DEFAULT                (_RMU_CMD_RCCLR_DEFAULT << 0) /**< Shifted mode DEFAULT for RMU_CMD */

/* Bit fields for EMU CTRL */
#define _EMU_CTRL_RESETVALUE           0x00000000UL                      /**< Default value for EMU_CTRL */
#define _EMU_CTRL_MASK                 0x0000000FUL                      /**< Mask for EMU_CTRL */
#define EMU_CTRL_EMVREG                (0x1UL << 0)                      /**< Energy Mode Voltage Regulator Control */
#define _EMU_CTRL_EMVREG_SHIFT         0                                 /**< Shift value for EMU_EMVREG */
#define _EMU_CTRL_EMVREG_MASK          0x1UL                             /**< Bit mask for EMU_EMVREG */
#define _EMU_CTRL_EMVREG_DEFAULT       0x00000000UL                      /**< Mode DEFAULT for EMU_CTRL */
#define _EMU_CTRL_EMVREG_REDUCED       0x00000000UL                      /**< Mode REDUCED for EMU_CTRL */
#define _EMU_CTRL_EMVREG_FULL          0x00000001UL                      /**< Mode FULL for EMU_CTRL */
#define EMU_CTRL_EMVREG_DEFAULT        (_EMU_CTRL_EMVREG_DEFAULT << 0)   /**< Shifted mode DEFAULT for EMU_CTRL */
#define EMU_CTRL_EMVREG_REDUCED        (_EMU_CTRL_EMVREG_REDUCED << 0)   /**< Shifted mode REDUCED for EMU_CTRL */
#define EMU_CTRL_EMVREG_FULL           (_EMU_CTRL_EMVREG_FULL << 0)      /**< Shifted mode FULL for EMU_CTRL */
#define EMU_CTRL_EM2BLOCK              (0x1UL << 1)                      /**< Energy Mode 2 Block */
#define _EMU_CTRL_EM2BLOCK_SHIFT       1                                 /**< Shift value for EMU_EM2BLOCK */
#define _EMU_CTRL_EM2BLOCK_MASK        0x2UL                             /**< Bit mask for EMU_EM2BLOCK */
#define _EMU_CTRL_EM2BLOCK_DEFAULT     0x00000000UL                      /**< Mode DEFAULT for EMU_CTRL */
#define EMU_CTRL_EM2BLOCK_DEFAULT      (_EMU_CTRL_EM2BLOCK_DEFAULT << 1) /**< Shifted mode DEFAULT for EMU_CTRL */
#define _EMU_CTRL_EM4CTRL_SHIFT        2                                 /**< Shift value for EMU_EM4CTRL */
#define _EMU_CTRL_EM4CTRL_MASK         0xCUL                             /**< Bit mask for EMU_EM4CTRL */
#define _EMU_CTRL_EM4CTRL_DEFAULT      0x00000000UL                      /**< Mode DEFAULT for EMU_CTRL */
#define EMU_CTRL_EM4CTRL_DEFAULT       (_EMU_CTRL_EM4CTRL_DEFAULT << 2)  /**< Shifted mode DEFAULT for EMU_CTRL */

/* Bit fields for EMU LOCK */
#define _EMU_LOCK_RESETVALUE           0x00000000UL                      /**< Default value for EMU_LOCK */
#define _EMU_LOCK_MASK                 0x0000FFFFUL                      /**< Mask for EMU_LOCK */
#define _EMU_LOCK_LOCKKEY_SHIFT        0                                 /**< Shift value for EMU_LOCKKEY */
#define _EMU_LOCK_LOCKKEY_MASK         0xFFFFUL                          /**< Bit mask for EMU_LOCKKEY */
#define _EMU_LOCK_LOCKKEY_DEFAULT      0x00000000UL                      /**< Mode DEFAULT for EMU_LOCK */
#define _EMU_LOCK_LOCKKEY_LOCK         0x00000000UL                      /**< Mode LOCK for EMU_LOCK */
#define _EMU_LOCK_LOCKKEY_UNLOCKED     0x00000000UL                      /**< Mode UNLOCKED for EMU_LOCK */
#define _EMU_LOCK_LOCKKEY_LOCKED       0x00000001UL                      /**< Mode LOCKED for EMU_LOCK */
#define _EMU_LOCK_LOCKKEY_UNLOCK       0x0000ADE8UL                      /**< Mode UNLOCK for EMU_LOCK */
#define EMU_LOCK_LOCKKEY_DEFAULT       (_EMU_LOCK_LOCKKEY_DEFAULT << 0)  /**< Shifted mode DEFAULT for EMU_LOCK */
#define EMU_LOCK_LOCKKEY_LOCK          (_EMU_LOCK_LOCKKEY_LOCK << 0)     /**< Shifted mode LOCK for EMU_LOCK */
#define EMU_LOCK_LOCKKEY_UNLOCKED      (_EMU_LOCK_LOCKKEY_UNLOCKED << 0) /**< Shifted mode UNLOCKED for EMU_LOCK */
#define EMU_LOCK_LOCKKEY_LOCKED        (_EMU_LOCK_LOCKKEY_LOCKED << 0)   /**< Shifted mode LOCKED for EMU_LOCK */
#define EMU_LOCK_LOCKKEY_UNLOCK        (_EMU_LOCK_LOCKKEY_UNLOCK << 0)   /**< Shifted mode UNLOCK for EMU_LOCK */

/* Bit fields for EMU AUXCTRL */
#define _EMU_AUXCTRL_RESETVALUE        0x00000000UL                       /**< Default value for EMU_AUXCTRL */
#define _EMU_AUXCTRL_MASK              0x00000001UL                       /**< Mask for EMU_AUXCTRL */
#define EMU_AUXCTRL_HRCCLR             (0x1UL << 0)                       /**< Hard Reset Cause Clear */
#define _EMU_AUXCTRL_HRCCLR_SHIFT      0                                  /**< Shift value for EMU_HRCCLR */
#define _EMU_AUXCTRL_HRCCLR_MASK       0x1UL                              /**< Bit mask for EMU_HRCCLR */
#define _EMU_AUXCTRL_HRCCLR_DEFAULT    0x00000000UL                       /**< Mode DEFAULT for EMU_AUXCTRL */
#define EMU_AUXCTRL_HRCCLR_DEFAULT     (_EMU_AUXCTRL_HRCCLR_DEFAULT << 0) /**< Shifted mode DEFAULT for EMU_AUXCTRL */

/* Bit fields for RTC CTRL */
#define _RTC_CTRL_RESETVALUE             0x00000000UL                      /**< Default value for RTC_CTRL */
#define _RTC_CTRL_MASK                   0x00000007UL                      /**< Mask for RTC_CTRL */
#define RTC_CTRL_EN                      (0x1UL << 0)                      /**< RTC Enable */
#define _RTC_CTRL_EN_SHIFT               0                                 /**< Shift value for RTC_EN */
#define _RTC_CTRL_EN_MASK                0x1UL                             /**< Bit mask for RTC_EN */
#define _RTC_CTRL_EN_DEFAULT             0x00000000UL                      /**< Mode DEFAULT for RTC_CTRL */
#define RTC_CTRL_EN_DEFAULT              (_RTC_CTRL_EN_DEFAULT << 0)       /**< Shifted mode DEFAULT for RTC_CTRL */
#define RTC_CTRL_DEBUGRUN                (0x1UL << 1)                      /**< Debug Mode Run Enable */
#define _RTC_CTRL_DEBUGRUN_SHIFT         1                                 /**< Shift value for RTC_DEBUGRUN */
#define _RTC_CTRL_DEBUGRUN_MASK          0x2UL                             /**< Bit mask for RTC_DEBUGRUN */
#define _RTC_CTRL_DEBUGRUN_DEFAULT       0x00000000UL                      /**< Mode DEFAULT for RTC_CTRL */
#define RTC_CTRL_DEBUGRUN_DEFAULT        (_RTC_CTRL_DEBUGRUN_DEFAULT << 1) /**< Shifted mode DEFAULT for RTC_CTRL */
#define RTC_CTRL_COMP0TOP                (0x1UL << 2)                      /**< Compare Channel 0 is Top Value */
#define _RTC_CTRL_COMP0TOP_SHIFT         2                                 /**< Shift value for RTC_COMP0TOP */
#define _RTC_CTRL_COMP0TOP_MASK          0x4UL                             /**< Bit mask for RTC_COMP0TOP */
#define _RTC_CTRL_COMP0TOP_DEFAULT       0x00000000UL                      /**< Mode DEFAULT for RTC_CTRL */
#define _RTC_CTRL_COMP0TOP_DISABLE       0x00000000UL                      /**< Mode DISABLE for RTC_CTRL */
#define _RTC_CTRL_COMP0TOP_ENABLE        0x00000001UL                      /**< Mode ENABLE for RTC_CTRL */
#define RTC_CTRL_COMP0TOP_DEFAULT        (_RTC_CTRL_COMP0TOP_DEFAULT << 2) /**< Shifted mode DEFAULT for RTC_CTRL */
#define RTC_CTRL_COMP0TOP_DISABLE        (_RTC_CTRL_COMP0TOP_DISABLE << 2) /**< Shifted mode DISABLE for RTC_CTRL */
#define RTC_CTRL_COMP0TOP_ENABLE         (_RTC_CTRL_COMP0TOP_ENABLE << 2)  /**< Shifted mode ENABLE for RTC_CTRL */

/* Bit fields for RTC CNT */
#define _RTC_CNT_RESETVALUE              0x00000000UL                /**< Default value for RTC_CNT */
#define _RTC_CNT_MASK                    0x00FFFFFFUL                /**< Mask for RTC_CNT */
#define _RTC_CNT_CNT_SHIFT               0                           /**< Shift value for RTC_CNT */
#define _RTC_CNT_CNT_MASK                0xFFFFFFUL                  /**< Bit mask for RTC_CNT */
#define _RTC_CNT_CNT_DEFAULT             0x00000000UL                /**< Mode DEFAULT for RTC_CNT */
#define RTC_CNT_CNT_DEFAULT              (_RTC_CNT_CNT_DEFAULT << 0) /**< Shifted mode DEFAULT for RTC_CNT */

/* Bit fields for RTC COMP0 */
#define _RTC_COMP0_RESETVALUE            0x00000000UL                    /**< Default value for RTC_COMP0 */
#define _RTC_COMP0_MASK                  0x00FFFFFFUL                    /**< Mask for RTC_COMP0 */
#define _RTC_COMP0_COMP0_SHIFT           0                               /**< Shift value for RTC_COMP0 */
#define _RTC_COMP0_COMP0_MASK            0xFFFFFFUL                      /**< Bit mask for RTC_COMP0 */
#define _RTC_COMP0_COMP0_DEFAULT         0x00000000UL                    /**< Mode DEFAULT for RTC_COMP0 */
#define RTC_COMP0_COMP0_DEFAULT          (_RTC_COMP0_COMP0_DEFAULT << 0) /**< Shifted mode DEFAULT for RTC_COMP0 */

/* Bit fields for RTC COMP1 */
#define _RTC_COMP1_RESETVALUE            0x00000000UL                    /**< Default value for RTC_COMP1 */
#define _RTC_COMP1_MASK                  0x00FFFFFFUL                    /**< Mask for RTC_COMP1 */
#define _RTC_COMP1_COMP1_SHIFT           0                               /**< Shift value for RTC_COMP1 */
#define _RTC_COMP1_COMP1_MASK            0xFFFFFFUL                      /**< Bit mask for RTC_COMP1 */
#define _RTC_COMP1_COMP1_DEFAULT         0x00000000UL                    /**< Mode DEFAULT for RTC_COMP1 */
#define RTC_COMP1_COMP1_DEFAULT          (_RTC_COMP1_COMP1_DEFAULT << 0) /**< Shifted mode DEFAULT for RTC_COMP1 */

/* Bit fields for RTC IF */
#define _RTC_IF_RESETVALUE               0x00000000UL                 /**< Default value for RTC_IF */
#define _RTC_IF_MASK                     0x00000007UL                 /**< Mask for RTC_IF */
#define RTC_IF_OF                        (0x1UL << 0)                 /**< Overflow Interrupt Flag */
#define _RTC_IF_OF_SHIFT                 0                            /**< Shift value for RTC_OF */
#define _RTC_IF_OF_MASK                  0x1UL                        /**< Bit mask for RTC_OF */
#define _RTC_IF_OF_DEFAULT               0x00000000UL                 /**< Mode DEFAULT for RTC_IF */
#define RTC_IF_OF_DEFAULT                (_RTC_IF_OF_DEFAULT << 0)    /**< Shifted mode DEFAULT for RTC_IF */
#define RTC_IF_COMP0                     (0x1UL << 1)                 /**< Compare Match 0 Interrupt Flag */
#define _RTC_IF_COMP0_SHIFT              1                            /**< Shift value for RTC_COMP0 */
#define _RTC_IF_COMP0_MASK               0x2UL                        /**< Bit mask for RTC_COMP0 */
#define _RTC_IF_COMP0_DEFAULT            0x00000000UL                 /**< Mode DEFAULT for RTC_IF */
#define RTC_IF_COMP0_DEFAULT             (_RTC_IF_COMP0_DEFAULT << 1) /**< Shifted mode DEFAULT for RTC_IF */
#define RTC_IF_COMP1                     (0x1UL << 2)                 /**< Compare Match 1 Interrupt Flag */
#define _RTC_IF_COMP1_SHIFT              2                            /**< Shift value for RTC_COMP1 */
#define _RTC_IF_COMP1_MASK               0x4UL                        /**< Bit mask for RTC_COMP1 */
#define _RTC_IF_COMP1_DEFAULT            0x00000000UL                 /**< Mode DEFAULT for RTC_IF */
#define RTC_IF_COMP1_DEFAULT             (_RTC_IF_COMP1_DEFAULT << 2) /**< Shifted mode DEFAULT for RTC_IF */

/* Bit fields for RTC IFS */
#define _RTC_IFS_RESETVALUE              0x00000000UL                  /**< Default value for RTC_IFS */
#define _RTC_IFS_MASK                    0x00000007UL                  /**< Mask for RTC_IFS */
#define RTC_IFS_OF                       (0x1UL << 0)                  /**< Set Overflow Interrupt Flag */
#define _RTC_IFS_OF_SHIFT                0                             /**< Shift value for RTC_OF */
#define _RTC_IFS_OF_MASK                 0x1UL                         /**< Bit mask for RTC_OF */
#define _RTC_IFS_OF_DEFAULT              0x00000000UL                  /**< Mode DEFAULT for RTC_IFS */
#define RTC_IFS_OF_DEFAULT               (_RTC_IFS_OF_DEFAULT << 0)    /**< Shifted mode DEFAULT for RTC_IFS */
#define RTC_IFS_COMP0                    (0x1UL << 1)                  /**< Set Compare match 0 Interrupt Flag */
#define _RTC_IFS_COMP0_SHIFT             1                             /**< Shift value for RTC_COMP0 */
#define _RTC_IFS_COMP0_MASK              0x2UL                         /**< Bit mask for RTC_COMP0 */
#define _RTC_IFS_COMP0_DEFAULT           0x00000000UL                  /**< Mode DEFAULT for RTC_IFS */
#define RTC_IFS_COMP0_DEFAULT            (_RTC_IFS_COMP0_DEFAULT << 1) /**< Shifted mode DEFAULT for RTC_IFS */
#define RTC_IFS_COMP1                    (0x1UL << 2)                  /**< Set Compare match 1 Interrupt Flag */
#define _RTC_IFS_COMP1_SHIFT             2                             /**< Shift value for RTC_COMP1 */
#define _RTC_IFS_COMP1_MASK              0x4UL                         /**< Bit mask for RTC_COMP1 */
#define _RTC_IFS_COMP1_DEFAULT           0x00000000UL                  /**< Mode DEFAULT for RTC_IFS */
#define RTC_IFS_COMP1_DEFAULT            (_RTC_IFS_COMP1_DEFAULT << 2) /**< Shifted mode DEFAULT for RTC_IFS */

/* Bit fields for RTC IFC */
#define _RTC_IFC_RESETVALUE              0x00000000UL                  /**< Default value for RTC_IFC */
#define _RTC_IFC_MASK                    0x00000007UL                  /**< Mask for RTC_IFC */
#define RTC_IFC_OF                       (0x1UL << 0)                  /**< Clear Overflow Interrupt Flag */
#define _RTC_IFC_OF_SHIFT                0                             /**< Shift value for RTC_OF */
#define _RTC_IFC_OF_MASK                 0x1UL                         /**< Bit mask for RTC_OF */
#define _RTC_IFC_OF_DEFAULT              0x00000000UL                  /**< Mode DEFAULT for RTC_IFC */
#define RTC_IFC_OF_DEFAULT               (_RTC_IFC_OF_DEFAULT << 0)    /**< Shifted mode DEFAULT for RTC_IFC */
#define RTC_IFC_COMP0                    (0x1UL << 1)                  /**< Clear Compare match 0 Interrupt Flag */
#define _RTC_IFC_COMP0_SHIFT             1                             /**< Shift value for RTC_COMP0 */
#define _RTC_IFC_COMP0_MASK              0x2UL                         /**< Bit mask for RTC_COMP0 */
#define _RTC_IFC_COMP0_DEFAULT           0x00000000UL                  /**< Mode DEFAULT for RTC_IFC */
#define RTC_IFC_COMP0_DEFAULT            (_RTC_IFC_COMP0_DEFAULT << 1) /**< Shifted mode DEFAULT for RTC_IFC */
#define RTC_IFC_COMP1                    (0x1UL << 2)                  /**< Clear Compare match 1 Interrupt Flag */
#define _RTC_IFC_COMP1_SHIFT             2                             /**< Shift value for RTC_COMP1 */
#define _RTC_IFC_COMP1_MASK              0x4UL                         /**< Bit mask for RTC_COMP1 */
#define _RTC_IFC_COMP1_DEFAULT           0x00000000UL                  /**< Mode DEFAULT for RTC_IFC */
#define RTC_IFC_COMP1_DEFAULT            (_RTC_IFC_COMP1_DEFAULT << 2) /**< Shifted mode DEFAULT for RTC_IFC */

/* Bit fields for RTC IEN */
#define _RTC_IEN_RESETVALUE              0x00000000UL                  /**< Default value for RTC_IEN */
#define _RTC_IEN_MASK                    0x00000007UL                  /**< Mask for RTC_IEN */
#define RTC_IEN_OF                       (0x1UL << 0)                  /**< Overflow Interrupt Enable */
#define _RTC_IEN_OF_SHIFT                0                             /**< Shift value for RTC_OF */
#define _RTC_IEN_OF_MASK                 0x1UL                         /**< Bit mask for RTC_OF */
#define _RTC_IEN_OF_DEFAULT              0x00000000UL                  /**< Mode DEFAULT for RTC_IEN */
#define RTC_IEN_OF_DEFAULT               (_RTC_IEN_OF_DEFAULT << 0)    /**< Shifted mode DEFAULT for RTC_IEN */
#define RTC_IEN_COMP0                    (0x1UL << 1)                  /**< Compare Match 0 Interrupt Enable */
#define _RTC_IEN_COMP0_SHIFT             1                             /**< Shift value for RTC_COMP0 */
#define _RTC_IEN_COMP0_MASK              0x2UL                         /**< Bit mask for RTC_COMP0 */
#define _RTC_IEN_COMP0_DEFAULT           0x00000000UL                  /**< Mode DEFAULT for RTC_IEN */
#define RTC_IEN_COMP0_DEFAULT            (_RTC_IEN_COMP0_DEFAULT << 1) /**< Shifted mode DEFAULT for RTC_IEN */
#define RTC_IEN_COMP1                    (0x1UL << 2)                  /**< Compare Match 1 Interrupt Enable */
#define _RTC_IEN_COMP1_SHIFT             2                             /**< Shift value for RTC_COMP1 */
#define _RTC_IEN_COMP1_MASK              0x4UL                         /**< Bit mask for RTC_COMP1 */
#define _RTC_IEN_COMP1_DEFAULT           0x00000000UL                  /**< Mode DEFAULT for RTC_IEN */
#define RTC_IEN_COMP1_DEFAULT            (_RTC_IEN_COMP1_DEFAULT << 2) /**< Shifted mode DEFAULT for RTC_IEN */

/* Bit fields for RTC FREEZE */
#define _RTC_FREEZE_RESETVALUE           0x00000000UL                         /**< Default value for RTC_FREEZE */
#define _RTC_FREEZE_MASK                 0x00000001UL                         /**< Mask for RTC_FREEZE */
#define RTC_FREEZE_REGFREEZE             (0x1UL << 0)                         /**< Register Update Freeze */
#define _RTC_FREEZE_REGFREEZE_SHIFT      0                                    /**< Shift value for RTC_REGFREEZE */
#define _RTC_FREEZE_REGFREEZE_MASK       0x1UL                                /**< Bit mask for RTC_REGFREEZE */
#define _RTC_FREEZE_REGFREEZE_DEFAULT    0x00000000UL                         /**< Mode DEFAULT for RTC_FREEZE */
#define _RTC_FREEZE_REGFREEZE_UPDATE     0x00000000UL                         /**< Mode UPDATE for RTC_FREEZE */
#define _RTC_FREEZE_REGFREEZE_FREEZE     0x00000001UL                         /**< Mode FREEZE for RTC_FREEZE */
#define RTC_FREEZE_REGFREEZE_DEFAULT     (_RTC_FREEZE_REGFREEZE_DEFAULT << 0) /**< Shifted mode DEFAULT for RTC_FREEZE */
#define RTC_FREEZE_REGFREEZE_UPDATE      (_RTC_FREEZE_REGFREEZE_UPDATE << 0)  /**< Shifted mode UPDATE for RTC_FREEZE */
#define RTC_FREEZE_REGFREEZE_FREEZE      (_RTC_FREEZE_REGFREEZE_FREEZE << 0)  /**< Shifted mode FREEZE for RTC_FREEZE */

/* Bit fields for RTC SYNCBUSY */
#define _RTC_SYNCBUSY_RESETVALUE         0x00000000UL                       /**< Default value for RTC_SYNCBUSY */
#define _RTC_SYNCBUSY_MASK               0x00000007UL                       /**< Mask for RTC_SYNCBUSY */
#define RTC_SYNCBUSY_CTRL                (0x1UL << 0)                       /**< CTRL Register Busy */
#define _RTC_SYNCBUSY_CTRL_SHIFT         0                                  /**< Shift value for RTC_CTRL */
#define _RTC_SYNCBUSY_CTRL_MASK          0x1UL                              /**< Bit mask for RTC_CTRL */
#define _RTC_SYNCBUSY_CTRL_DEFAULT       0x00000000UL                       /**< Mode DEFAULT for RTC_SYNCBUSY */
#define RTC_SYNCBUSY_CTRL_DEFAULT        (_RTC_SYNCBUSY_CTRL_DEFAULT << 0)  /**< Shifted mode DEFAULT for RTC_SYNCBUSY */
#define RTC_SYNCBUSY_COMP0               (0x1UL << 1)                       /**< COMP0 Register Busy */
#define _RTC_SYNCBUSY_COMP0_SHIFT        1                                  /**< Shift value for RTC_COMP0 */
#define _RTC_SYNCBUSY_COMP0_MASK         0x2UL                              /**< Bit mask for RTC_COMP0 */
#define _RTC_SYNCBUSY_COMP0_DEFAULT      0x00000000UL                       /**< Mode DEFAULT for RTC_SYNCBUSY */
#define RTC_SYNCBUSY_COMP0_DEFAULT       (_RTC_SYNCBUSY_COMP0_DEFAULT << 1) /**< Shifted mode DEFAULT for RTC_SYNCBUSY */
#define RTC_SYNCBUSY_COMP1               (0x1UL << 2)                       /**< COMP1 Register Busy */
#define _RTC_SYNCBUSY_COMP1_SHIFT        2                                  /**< Shift value for RTC_COMP1 */
#define _RTC_SYNCBUSY_COMP1_MASK         0x4UL                              /**< Bit mask for RTC_COMP1 */
#define _RTC_SYNCBUSY_COMP1_DEFAULT      0x00000000UL                       /**< Mode DEFAULT for RTC_SYNCBUSY */
#define RTC_SYNCBUSY_COMP1_DEFAULT       (_RTC_SYNCBUSY_COMP1_DEFAULT << 2) /**< Shifted mode DEFAULT for RTC_SYNCBUSY */

/* Bit fields for WDOG CTRL */
#define _WDOG_CTRL_RESETVALUE            0x00000F00UL                         /**< Default value for WDOG_CTRL */
#define _WDOG_CTRL_MASK                  0x00003F7FUL                         /**< Mask for WDOG_CTRL */
#define WDOG_CTRL_EN                     (0x1UL << 0)                         /**< Watchdog Timer Enable */
#define _WDOG_CTRL_EN_SHIFT              0                                    /**< Shift value for WDOG_EN */
#define _WDOG_CTRL_EN_MASK               0x1UL                                /**< Bit mask for WDOG_EN */
#define _WDOG_CTRL_EN_DEFAULT            0x00000000UL                         /**< Mode DEFAULT for WDOG_CTRL */
#define WDOG_CTRL_EN_DEFAULT             (_WDOG_CTRL_EN_DEFAULT << 0)         /**< Shifted mode DEFAULT for WDOG_CTRL */
#define WDOG_CTRL_DEBUGRUN               (0x1UL << 1)                         /**< Debug Mode Run Enable */
#define _WDOG_CTRL_DEBUGRUN_SHIFT        1                                    /**< Shift value for WDOG_DEBUGRUN */
#define _WDOG_CTRL_DEBUGRUN_MASK         0x2UL                                /**< Bit mask for WDOG_DEBUGRUN */
#define _WDOG_CTRL_DEBUGRUN_DEFAULT      0x00000000UL                         /**< Mode DEFAULT for WDOG_CTRL */
#define WDOG_CTRL_DEBUGRUN_DEFAULT       (_WDOG_CTRL_DEBUGRUN_DEFAULT << 1)   /**< Shifted mode DEFAULT for WDOG_CTRL */
#define WDOG_CTRL_EM2RUN                 (0x1UL << 2)                         /**< Energy Mode 2 Run Enable */
#define _WDOG_CTRL_EM2RUN_SHIFT          2                                    /**< Shift value for WDOG_EM2RUN */
#define _WDOG_CTRL_EM2RUN_MASK           0x4UL                                /**< Bit mask for WDOG_EM2RUN */
#define _WDOG_CTRL_EM2RUN_DEFAULT        0x00000000UL                         /**< Mode DEFAULT for WDOG_CTRL */
#define WDOG_CTRL_EM2RUN_DEFAULT         (_WDOG_CTRL_EM2RUN_DEFAULT << 2)     /**< Shifted mode DEFAULT for WDOG_CTRL */
#define WDOG_CTRL_EM3RUN                 (0x1UL << 3)                         /**< Energy Mode 3 Run Enable */
#define _WDOG_CTRL_EM3RUN_SHIFT          3                                    /**< Shift value for WDOG_EM3RUN */
#define _WDOG_CTRL_EM3RUN_MASK           0x8UL                                /**< Bit mask for WDOG_EM3RUN */
#define _WDOG_CTRL_EM3RUN_DEFAULT        0x00000000UL                         /**< Mode DEFAULT for WDOG_CTRL */
#define WDOG_CTRL_EM3RUN_DEFAULT         (_WDOG_CTRL_EM3RUN_DEFAULT << 3)     /**< Shifted mode DEFAULT for WDOG_CTRL */
#define WDOG_CTRL_LOCK                   (0x1UL << 4)                         /**< Configuration lock */
#define _WDOG_CTRL_LOCK_SHIFT            4                                    /**< Shift value for WDOG_LOCK */
#define _WDOG_CTRL_LOCK_MASK             0x10UL                               /**< Bit mask for WDOG_LOCK */
#define _WDOG_CTRL_LOCK_DEFAULT          0x00000000UL                         /**< Mode DEFAULT for WDOG_CTRL */
#define WDOG_CTRL_LOCK_DEFAULT           (_WDOG_CTRL_LOCK_DEFAULT << 4)       /**< Shifted mode DEFAULT for WDOG_CTRL */
#define WDOG_CTRL_EM4BLOCK               (0x1UL << 5)                         /**< Energy Mode 4 Block */
#define _WDOG_CTRL_EM4BLOCK_SHIFT        5                                    /**< Shift value for WDOG_EM4BLOCK */
#define _WDOG_CTRL_EM4BLOCK_MASK         0x20UL                               /**< Bit mask for WDOG_EM4BLOCK */
#define _WDOG_CTRL_EM4BLOCK_DEFAULT      0x00000000UL                         /**< Mode DEFAULT for WDOG_CTRL */
#define WDOG_CTRL_EM4BLOCK_DEFAULT       (_WDOG_CTRL_EM4BLOCK_DEFAULT << 5)   /**< Shifted mode DEFAULT for WDOG_CTRL */
#define WDOG_CTRL_SWOSCBLOCK             (0x1UL << 6)                         /**< Software Oscillator Disable Block */
#define _WDOG_CTRL_SWOSCBLOCK_SHIFT      6                                    /**< Shift value for WDOG_SWOSCBLOCK */
#define _WDOG_CTRL_SWOSCBLOCK_MASK       0x40UL                               /**< Bit mask for WDOG_SWOSCBLOCK */
#define _WDOG_CTRL_SWOSCBLOCK_DEFAULT    0x00000000UL                         /**< Mode DEFAULT for WDOG_CTRL */
#define WDOG_CTRL_SWOSCBLOCK_DEFAULT     (_WDOG_CTRL_SWOSCBLOCK_DEFAULT << 6) /**< Shifted mode DEFAULT for WDOG_CTRL */
#define _WDOG_CTRL_PERSEL_SHIFT          8                                    /**< Shift value for WDOG_PERSEL */
#define _WDOG_CTRL_PERSEL_MASK           0xF00UL                              /**< Bit mask for WDOG_PERSEL */
#define _WDOG_CTRL_PERSEL_DEFAULT        0x0000000FUL                         /**< Mode DEFAULT for WDOG_CTRL */
#define WDOG_CTRL_PERSEL_DEFAULT         (_WDOG_CTRL_PERSEL_DEFAULT << 8)     /**< Shifted mode DEFAULT for WDOG_CTRL */
#define _WDOG_CTRL_CLKSEL_SHIFT          12                                   /**< Shift value for WDOG_CLKSEL */
#define _WDOG_CTRL_CLKSEL_MASK           0x3000UL                             /**< Bit mask for WDOG_CLKSEL */
#define _WDOG_CTRL_CLKSEL_DEFAULT        0x00000000UL                         /**< Mode DEFAULT for WDOG_CTRL */
#define _WDOG_CTRL_CLKSEL_ULFRCO         0x00000000UL                         /**< Mode ULFRCO for WDOG_CTRL */
#define _WDOG_CTRL_CLKSEL_LFRCO          0x00000001UL                         /**< Mode LFRCO for WDOG_CTRL */
#define _WDOG_CTRL_CLKSEL_LFXO           0x00000002UL                         /**< Mode LFXO for WDOG_CTRL */
#define WDOG_CTRL_CLKSEL_DEFAULT         (_WDOG_CTRL_CLKSEL_DEFAULT << 12)    /**< Shifted mode DEFAULT for WDOG_CTRL */
#define WDOG_CTRL_CLKSEL_ULFRCO          (_WDOG_CTRL_CLKSEL_ULFRCO << 12)     /**< Shifted mode ULFRCO for WDOG_CTRL */
#define WDOG_CTRL_CLKSEL_LFRCO           (_WDOG_CTRL_CLKSEL_LFRCO << 12)      /**< Shifted mode LFRCO for WDOG_CTRL */
#define WDOG_CTRL_CLKSEL_LFXO            (_WDOG_CTRL_CLKSEL_LFXO << 12)       /**< Shifted mode LFXO for WDOG_CTRL */

/* Bit fields for WDOG CMD */
#define _WDOG_CMD_RESETVALUE             0x00000000UL                     /**< Default value for WDOG_CMD */
#define _WDOG_CMD_MASK                   0x00000001UL                     /**< Mask for WDOG_CMD */
#define WDOG_CMD_CLEAR                   (0x1UL << 0)                     /**< Watchdog Timer Clear */
#define _WDOG_CMD_CLEAR_SHIFT            0                                /**< Shift value for WDOG_CLEAR */
#define _WDOG_CMD_CLEAR_MASK             0x1UL                            /**< Bit mask for WDOG_CLEAR */
#define _WDOG_CMD_CLEAR_DEFAULT          0x00000000UL                     /**< Mode DEFAULT for WDOG_CMD */
#define _WDOG_CMD_CLEAR_UNCHANGED        0x00000000UL                     /**< Mode UNCHANGED for WDOG_CMD */
#define _WDOG_CMD_CLEAR_CLEARED          0x00000001UL                     /**< Mode CLEARED for WDOG_CMD */
#define WDOG_CMD_CLEAR_DEFAULT           (_WDOG_CMD_CLEAR_DEFAULT << 0)   /**< Shifted mode DEFAULT for WDOG_CMD */
#define WDOG_CMD_CLEAR_UNCHANGED         (_WDOG_CMD_CLEAR_UNCHANGED << 0) /**< Shifted mode UNCHANGED for WDOG_CMD */
#define WDOG_CMD_CLEAR_CLEARED           (_WDOG_CMD_CLEAR_CLEARED << 0)   /**< Shifted mode CLEARED for WDOG_CMD */

/* Bit fields for WDOG SYNCBUSY */
#define _WDOG_SYNCBUSY_RESETVALUE        0x00000000UL                       /**< Default value for WDOG_SYNCBUSY */
#define _WDOG_SYNCBUSY_MASK              0x00000003UL                       /**< Mask for WDOG_SYNCBUSY */
#define WDOG_SYNCBUSY_CTRL               (0x1UL << 0)                       /**< CTRL Register Busy */
#define _WDOG_SYNCBUSY_CTRL_SHIFT        0                                  /**< Shift value for WDOG_CTRL */
#define _WDOG_SYNCBUSY_CTRL_MASK         0x1UL                              /**< Bit mask for WDOG_CTRL */
#define _WDOG_SYNCBUSY_CTRL_DEFAULT      0x00000000UL                       /**< Mode DEFAULT for WDOG_SYNCBUSY */
#define WDOG_SYNCBUSY_CTRL_DEFAULT       (_WDOG_SYNCBUSY_CTRL_DEFAULT << 0) /**< Shifted mode DEFAULT for WDOG_SYNCBUSY */
#define WDOG_SYNCBUSY_CMD                (0x1UL << 1)                       /**< CMD Register Busy */
#define _WDOG_SYNCBUSY_CMD_SHIFT         1                                  /**< Shift value for WDOG_CMD */
#define _WDOG_SYNCBUSY_CMD_MASK          0x2UL                              /**< Bit mask for WDOG_CMD */
#define _WDOG_SYNCBUSY_CMD_DEFAULT       0x00000000UL                       /**< Mode DEFAULT for WDOG_SYNCBUSY */
#define WDOG_SYNCBUSY_CMD_DEFAULT        (_WDOG_SYNCBUSY_CMD_DEFAULT << 1)  /**< Shifted mode DEFAULT for WDOG_SYNCBUSY */

/** LFRCO frequency, tuned to below frequency during manufacturing. */
#define EFM32_LFRCO_FREQ  (32768UL)
#define EFM32_ULFRCO_FREQ (1000UL)

/** \brief  Enable IRQ Interrupts
  This function enables IRQ interrupts by clearing the I-bit in the CPSR.
  Can only be executed in Privileged modes.
 */
__attribute__((always_inline)) static inline void __enable_irq(void)
{
    asm volatile ("cpsie i" : : : "memory");
}

/** \brief  Disable IRQ Interrupts
  This function disables IRQ interrupts by setting the I-bit in the CPSR.
  Can only be executed in Privileged modes.
 */
__attribute__((always_inline)) static inline void __disable_irq(void)
{
    asm volatile ("cpsid i" : : : "memory");
}

/** \brief  Enable External Interrupt
    The function enables a device-specific interrupt in the NVIC interrupt controller.
    \param [in]      IRQn  External interrupt number. Value cannot be negative.
 */
static inline void NVIC_EnableIRQ(IRQn_Type IRQn)
{
    NVIC->ISER[0] = (1 << ((uint32_t)(IRQn) & 0x1F));
}

/** \brief  Disable External Interrupt
    The function disables a device-specific interrupt in the NVIC interrupt controller.
    \param [in]      IRQn  External interrupt number. Value cannot be negative.
 */
static inline void NVIC_DisableIRQ(IRQn_Type IRQn)
{
    NVIC->ICER[0] = (1 << ((uint32_t)(IRQn) & 0x1F));
}

#endif /* MCU_H */