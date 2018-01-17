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

/**
 * \brief  Device info
 */
typedef struct
{
  __I uint32_t CAL;          /**< Calibration temperature and checksum */
  __I uint32_t ADC0CAL0;     /**< ADC0 Calibration register 0 */
  __I uint32_t ADC0CAL1;     /**< ADC0 Calibration register 1 */
  __I uint32_t ADC0CAL2;     /**< ADC0 Calibration register 2 */
  uint32_t     RESERVED0[2]; /**< Reserved */
  __I uint32_t IDAC0CAL0;    /**< IDAC0 calibration register */
  __I uint32_t USHFRCOCAL0;  /**< USHFRCO calibration register */
  uint32_t     RESERVED1[1]; /**< Reserved */
  __I uint32_t AUXHFRCOCAL0; /**< AUXHFRCO calibration register 0 */
  __I uint32_t AUXHFRCOCAL1; /**< AUXHFRCO calibration register 1 */
  __I uint32_t HFRCOCAL0;    /**< HFRCO calibration register 0 */
  __I uint32_t HFRCOCAL1;    /**< HFRCO calibration register 1 */
  __I uint32_t MEMINFO;      /**< Memory information */
  uint32_t     RESERVED2[2]; /**< Reserved */
  __I uint32_t UNIQUEL;      /**< Low 32 bits of device unique number */
  __I uint32_t UNIQUEH;      /**< High 32 bits of device unique number */
  __I uint32_t MSIZE;        /**< Flash and SRAM Memory size in KiloBytes */
  __I uint32_t PART;         /**< Part description */
} DEVINFO_TypeDef;

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

/** \brief  EFM32_USB_DIEP Register Declaration
 */
typedef struct
{
  __IO uint32_t CTL;          /**< Device IN Endpoint x+1 Control Register  */
  uint32_t       RESERVED0[1]; /**< Reserved for future use **/
  __IO uint32_t INT;          /**< Device IN Endpoint x+1 Interrupt Register  */
  uint32_t       RESERVED1[1]; /**< Reserved for future use **/
  __IO uint32_t TSIZ;         /**< Device IN Endpoint x+1 Transfer Size Register  */
  __IO uint32_t DMAADDR;      /**< Device IN Endpoint x+1 DMA Address Register  */
  __I uint32_t  TXFSTS;       /**< Device IN Endpoint x+1 Transmit FIFO Status Register  */
  uint32_t       RESERVED2[1]; /**< Reserved future */
} USB_DIEP_TypeDef;

/** \brief  EMF32_USB_DOEP Register Declaration
 */
typedef struct
{
  __IO uint32_t CTL;          /**< Device OUT Endpoint x+1 Control Register  */
  uint32_t       RESERVED0[1]; /**< Reserved for future use **/
  __IO uint32_t INT;          /**< Device OUT Endpoint x+1 Interrupt Register  */
  uint32_t       RESERVED1[1]; /**< Reserved for future use **/
  __IO uint32_t TSIZ;         /**< Device OUT Endpoint x+1 Transfer Size Register  */
  __IO uint32_t DMAADDR;      /**< Device OUT Endpoint x+1 DMA Address Register  */
  uint32_t       RESERVED2[2]; /**< Reserved future */
} USB_DOEP_TypeDef;

/** \brief  EFM32HG_USB Register Declaration
 */
typedef struct
{
  __IO uint32_t   CTRL;              /**< System Control Register  */
  __I uint32_t    STATUS;            /**< System Status Register  */
  __I uint32_t    IF;                /**< Interrupt Flag Register  */
  __IO uint32_t   IFS;               /**< Interrupt Flag Set Register  */
  __IO uint32_t   IFC;               /**< Interrupt Flag Clear Register  */
  __IO uint32_t   IEN;               /**< Interrupt Enable Register  */
  __IO uint32_t   ROUTE;             /**< I/O Routing Register  */

  uint32_t         RESERVED0[61435];  /**< Reserved for future use **/
  __IO uint32_t   GAHBCFG;           /**< AHB Configuration Register  */
  __IO uint32_t   GUSBCFG;           /**< USB Configuration Register  */
  __IO uint32_t   GRSTCTL;           /**< Reset Register  */
  __IO uint32_t   GINTSTS;           /**< Interrupt Register  */
  __IO uint32_t   GINTMSK;           /**< Interrupt Mask Register  */
  __I uint32_t    GRXSTSR;           /**< Receive Status Debug Read Register  */
  __I uint32_t    GRXSTSP;           /**< Receive Status Read and Pop Register  */
  __IO uint32_t   GRXFSIZ;           /**< Receive FIFO Size Register  */
  __IO uint32_t   GNPTXFSIZ;         /**< Non-periodic Transmit FIFO Size Register  */

  uint32_t         RESERVED1[12];     /**< Reserved for future use **/
  __IO uint32_t   GDFIFOCFG;         /**< Global DFIFO Configuration Register  */

  uint32_t         RESERVED2[41];     /**< Reserved for future use **/
  __IO uint32_t   DIEPTXF1;          /**< Device IN Endpoint Transmit FIFO 1 Size Register  */
  __IO uint32_t   DIEPTXF2;          /**< Device IN Endpoint Transmit FIFO 2 Size Register  */
  __IO uint32_t   DIEPTXF3;          /**< Device IN Endpoint Transmit FIFO 3 Size Register  */

  uint32_t         RESERVED3[444];    /**< Reserved for future use **/
  __IO uint32_t   DCFG;              /**< Device Configuration Register  */
  __IO uint32_t   DCTL;              /**< Device Control Register  */
  __I uint32_t    DSTS;              /**< Device Status Register  */
  uint32_t         RESERVED4[1];      /**< Reserved for future use **/
  __IO uint32_t   DIEPMSK;           /**< Device IN Endpoint Common Interrupt Mask Register  */
  __IO uint32_t   DOEPMSK;           /**< Device OUT Endpoint Common Interrupt Mask Register  */
  __I uint32_t    DAINT;             /**< Device All Endpoints Interrupt Register  */
  __IO uint32_t   DAINTMSK;          /**< Device All Endpoints Interrupt Mask Register  */

  uint32_t         RESERVED5[5];      /**< Reserved for future use **/
  __IO uint32_t   DIEPEMPMSK;        /**< Device IN Endpoint FIFO Empty Interrupt Mask Register  */

  uint32_t         RESERVED6[50];     /**< Reserved for future use **/
  __IO uint32_t   DIEP0CTL;          /**< Device IN Endpoint 0 Control Register  */
  uint32_t         RESERVED7[1];      /**< Reserved for future use **/
  __IO uint32_t   DIEP0INT;          /**< Device IN Endpoint 0 Interrupt Register  */
  uint32_t         RESERVED8[1];      /**< Reserved for future use **/
  __IO uint32_t   DIEP0TSIZ;         /**< Device IN Endpoint 0 Transfer Size Register  */
  __IO uint32_t   DIEP0DMAADDR;      /**< Device IN Endpoint 0 DMA Address Register  */
  __I uint32_t    DIEP0TXFSTS;       /**< Device IN Endpoint 0 Transmit FIFO Status Register  */

  uint32_t         RESERVED9[1];      /**< Reserved registers */
  USB_DIEP_TypeDef DIEP[3];           /**< Device IN Endpoint x+1 Registers */

  uint32_t         RESERVED10[96];    /**< Reserved for future use **/
  __IO uint32_t   DOEP0CTL;          /**< Device OUT Endpoint 0 Control Register  */
  uint32_t         RESERVED11[1];     /**< Reserved for future use **/
  __IO uint32_t   DOEP0INT;          /**< Device OUT Endpoint 0 Interrupt Register  */
  uint32_t         RESERVED12[1];     /**< Reserved for future use **/
  __IO uint32_t   DOEP0TSIZ;         /**< Device OUT Endpoint 0 Transfer Size Register  */
  __IO uint32_t   DOEP0DMAADDR;      /**< Device OUT Endpoint 0 DMA Address Register  */

  uint32_t         RESERVED13[2];     /**< Reserved registers */
  USB_DOEP_TypeDef DOEP[3];           /**< Device OUT Endpoint x+1 Registers */

  uint32_t         RESERVED14[160];   /**< Reserved for future use **/
  __IO uint32_t   PCGCCTL;           /**< Power and Clock Gating Control Register  */

  uint32_t         RESERVED15[127];   /**< Reserved registers */
  __IO uint32_t   FIFO0D[384];       /**< Device EP 0 FIFO  */

  uint32_t         RESERVED16[640];   /**< Reserved registers */
  __IO uint32_t   FIFO1D[384];       /**< Device EP 1 FIFO  */

  uint32_t         RESERVED17[640];   /**< Reserved registers */
  __IO uint32_t   FIFO2D[384];       /**< Device EP 2 FIFO  */

  uint32_t         RESERVED18[640];   /**< Reserved registers */
  __IO uint32_t   FIFO3D[384];       /**< Device EP 3 FIFO  */

  uint32_t         RESERVED19[28288]; /**< Reserved registers */
  __IO uint32_t   FIFORAM[512];      /**< Direct Access to Data FIFO RAM for Debugging (2 KB)  */
} USB_TypeDef;

/**
 * \brief GPIO_P EFM32HG GPIO P
 */
typedef struct
{
  __IO uint32_t CTRL;     /**< Port Control Register  */
  __IO uint32_t MODEL;    /**< Port Pin Mode Low Register  */
  __IO uint32_t MODEH;    /**< Port Pin Mode High Register  */
  __IO uint32_t DOUT;     /**< Port Data Out Register  */
  __O uint32_t  DOUTSET;  /**< Port Data Out Set Register  */
  __O uint32_t  DOUTCLR;  /**< Port Data Out Clear Register  */
  __O uint32_t  DOUTTGL;  /**< Port Data Out Toggle Register  */
  __I uint32_t  DIN;      /**< Port Data In Register  */
  __IO uint32_t PINLOCKN; /**< Port Unlocked Pins Register  */
} GPIO_P_TypeDef;

/** \brief  EFM32HG_GPIO Register Declaration
 */
typedef struct
{
  GPIO_P_TypeDef P[6];          /**< Port configuration bits */

  uint32_t       RESERVED0[10]; /**< Reserved for future use **/
  __IO uint32_t  EXTIPSELL;     /**< External Interrupt Port Select Low Register  */
  __IO uint32_t  EXTIPSELH;     /**< External Interrupt Port Select High Register  */
  __IO uint32_t  EXTIRISE;      /**< External Interrupt Rising Edge Trigger Register  */
  __IO uint32_t  EXTIFALL;      /**< External Interrupt Falling Edge Trigger Register  */
  __IO uint32_t  IEN;           /**< Interrupt Enable Register  */
  __I uint32_t   IF;            /**< Interrupt Flag Register  */
  __IO uint32_t  IFS;           /**< Interrupt Flag Set Register  */
  __IO uint32_t  IFC;           /**< Interrupt Flag Clear Register  */

  __IO uint32_t  ROUTE;         /**< I/O Routing Register  */
  __IO uint32_t  INSENSE;       /**< Input Sense Register  */
  __IO uint32_t  LOCK;          /**< Configuration Lock Register  */
  __IO uint32_t  CTRL;          /**< GPIO Control Register  */
  __IO uint32_t  CMD;           /**< GPIO Command Register  */
  __IO uint32_t  EM4WUEN;       /**< EM4 Wake-up Enable Register  */
  __IO uint32_t  EM4WUPOL;      /**< EM4 Wake-up Polarity Register  */
  __I uint32_t   EM4WUCAUSE;    /**< EM4 Wake-up Cause Register  */
} GPIO_TypeDef;

typedef struct
{
  volatile const uint32_t PID4;        /* JEP_106_BANK */
  volatile const uint32_t PID5;        /* Unused */
  volatile const uint32_t PID6;        /* Unused */
  volatile const uint32_t PID7;        /* Unused */
  volatile const uint32_t PID0;        /* Chip family LSB, chip major revision */
  volatile const uint32_t PID1;        /* JEP_106_NO, Chip family MSB */
  volatile const uint32_t PID2;        /* Chip minor rev MSB, JEP_106_PRESENT, JEP_106_NO */
  volatile const uint32_t PID3;        /* Chip minor rev LSB */
  volatile const uint32_t CID0;        /* Unused */
} ROMTABLE_TypeDef;

/* Memory mapping of Cortex-M0+ Hardware */
#define SCS_BASE            (0xE000E000UL)                            /*!< System Control Space Base Address */
#define SysTick_BASE        (SCS_BASE +  0x0010UL)                    /*!< SysTick Base Address              */
#define NVIC_BASE           (SCS_BASE +  0x0100UL)                    /*!< NVIC Base Address                 */
#define SCB_BASE            (SCS_BASE +  0x0D00UL)                    /*!< System Control Block Base Address */
#define GPIO_BASE           (0x40006000UL)                            /*!< GPIO Base Address                 */
#define RTC_BASE            (0x40080000UL)                            /*!< RTC Base Address                  */
#define WDOG_BASE           (0x40088000UL)                            /*!< RTC Base Address                  */
#define USB_BASE            (0x400C4000UL)                            /*!< USB Base Address                  */
#define EMU_BASE            (0x400C6000UL)                            /*!< EMU Base Address                  */
#define CMU_BASE            (0x400C8000UL)                            /*!< CMU Base Address                  */
#define RMU_BASE            (0x400CA000UL)                            /*!< RMU Base Address                  */
#define ROMTABLE_BASE       (0xF00FFFD0UL)                            /*!< ROMTABLE Base Address             */
#define DEVINFO_BASE        (0x0FE081B0UL)                            /*!< DEVINFO base address              */

#define SysTick             ((SysTick_Type   *)     SysTick_BASE  )   /*!< SysTick configuration struct       */
#define NVIC                ((NVIC_Type      *)     NVIC_BASE     )   /*!< NVIC configuration struct          */
#define SCB                 ((SCB_Type       *)     SCB_BASE      )   /*!< SCB configuration struct           */
#define GPIO                ((GPIO_TypeDef   *)     GPIO_BASE     )   /*!< GPIO configuration struct */
#define RTC                 ((RTC_TypeDef    *)     RTC_BASE      )   /*!< RTC configuration struct           */
#define WDOG                ((WDOG_TypeDef   *)     WDOG_BASE     )   /*!< Watchdog configuration struct      */
#define USB                 ((USB_TypeDef    *)     USB_BASE      )   /*!< USB configuration struct           */
#define EMU                 ((EMU_TypeDef    *)     EMU_BASE      )   /*!< Energy Management Unit configuration struct */
#define CMU                 ((CMU_TypeDef    *)     CMU_BASE      )   /*!< CMU configuration struct           */
#define RMU                 ((RMU_TypeDef    *)     RMU_BASE      )   /*!< Reset Management Unit configuration struct */
#define ROMTABLE            ((ROMTABLE_TypeDef *)   ROMTABLE_BASE )   /*!< ROMTABLE struct                    */
#define DEVINFO             ((DEVINFO_TypeDef *)    DEVINFO_BASE  )   /*!< Device information struct          */

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

/* Bit fields for CMU STATUS */
#define _CMU_STATUS_RESETVALUE                      0x00000403UL                               /**< Default value for CMU_STATUS */
#define _CMU_STATUS_MASK                            0x04F77FFFUL                               /**< Mask for CMU_STATUS */
#define CMU_STATUS_HFRCOENS                         (0x1UL << 0)                               /**< HFRCO Enable Status */
#define _CMU_STATUS_HFRCOENS_SHIFT                  0                                          /**< Shift value for CMU_HFRCOENS */
#define _CMU_STATUS_HFRCOENS_MASK                   0x1UL                                      /**< Bit mask for CMU_HFRCOENS */
#define _CMU_STATUS_HFRCOENS_DEFAULT                0x00000001UL                               /**< Mode DEFAULT for CMU_STATUS */
#define CMU_STATUS_HFRCOENS_DEFAULT                 (_CMU_STATUS_HFRCOENS_DEFAULT << 0)        /**< Shifted mode DEFAULT for CMU_STATUS */
#define CMU_STATUS_HFRCORDY                         (0x1UL << 1)                               /**< HFRCO Ready */
#define _CMU_STATUS_HFRCORDY_SHIFT                  1                                          /**< Shift value for CMU_HFRCORDY */
#define _CMU_STATUS_HFRCORDY_MASK                   0x2UL                                      /**< Bit mask for CMU_HFRCORDY */
#define _CMU_STATUS_HFRCORDY_DEFAULT                0x00000001UL                               /**< Mode DEFAULT for CMU_STATUS */
#define CMU_STATUS_HFRCORDY_DEFAULT                 (_CMU_STATUS_HFRCORDY_DEFAULT << 1)        /**< Shifted mode DEFAULT for CMU_STATUS */
#define CMU_STATUS_HFXOENS                          (0x1UL << 2)                               /**< HFXO Enable Status */
#define _CMU_STATUS_HFXOENS_SHIFT                   2                                          /**< Shift value for CMU_HFXOENS */
#define _CMU_STATUS_HFXOENS_MASK                    0x4UL                                      /**< Bit mask for CMU_HFXOENS */
#define _CMU_STATUS_HFXOENS_DEFAULT                 0x00000000UL                               /**< Mode DEFAULT for CMU_STATUS */
#define CMU_STATUS_HFXOENS_DEFAULT                  (_CMU_STATUS_HFXOENS_DEFAULT << 2)         /**< Shifted mode DEFAULT for CMU_STATUS */
#define CMU_STATUS_HFXORDY                          (0x1UL << 3)                               /**< HFXO Ready */
#define _CMU_STATUS_HFXORDY_SHIFT                   3                                          /**< Shift value for CMU_HFXORDY */
#define _CMU_STATUS_HFXORDY_MASK                    0x8UL                                      /**< Bit mask for CMU_HFXORDY */
#define _CMU_STATUS_HFXORDY_DEFAULT                 0x00000000UL                               /**< Mode DEFAULT for CMU_STATUS */
#define CMU_STATUS_HFXORDY_DEFAULT                  (_CMU_STATUS_HFXORDY_DEFAULT << 3)         /**< Shifted mode DEFAULT for CMU_STATUS */
#define CMU_STATUS_AUXHFRCOENS                      (0x1UL << 4)                               /**< AUXHFRCO Enable Status */
#define _CMU_STATUS_AUXHFRCOENS_SHIFT               4                                          /**< Shift value for CMU_AUXHFRCOENS */
#define _CMU_STATUS_AUXHFRCOENS_MASK                0x10UL                                     /**< Bit mask for CMU_AUXHFRCOENS */
#define _CMU_STATUS_AUXHFRCOENS_DEFAULT             0x00000000UL                               /**< Mode DEFAULT for CMU_STATUS */
#define CMU_STATUS_AUXHFRCOENS_DEFAULT              (_CMU_STATUS_AUXHFRCOENS_DEFAULT << 4)     /**< Shifted mode DEFAULT for CMU_STATUS */
#define CMU_STATUS_AUXHFRCORDY                      (0x1UL << 5)                               /**< AUXHFRCO Ready */
#define _CMU_STATUS_AUXHFRCORDY_SHIFT               5                                          /**< Shift value for CMU_AUXHFRCORDY */
#define _CMU_STATUS_AUXHFRCORDY_MASK                0x20UL                                     /**< Bit mask for CMU_AUXHFRCORDY */
#define _CMU_STATUS_AUXHFRCORDY_DEFAULT             0x00000000UL                               /**< Mode DEFAULT for CMU_STATUS */
#define CMU_STATUS_AUXHFRCORDY_DEFAULT              (_CMU_STATUS_AUXHFRCORDY_DEFAULT << 5)     /**< Shifted mode DEFAULT for CMU_STATUS */
#define CMU_STATUS_LFRCOENS                         (0x1UL << 6)                               /**< LFRCO Enable Status */
#define _CMU_STATUS_LFRCOENS_SHIFT                  6                                          /**< Shift value for CMU_LFRCOENS */
#define _CMU_STATUS_LFRCOENS_MASK                   0x40UL                                     /**< Bit mask for CMU_LFRCOENS */
#define _CMU_STATUS_LFRCOENS_DEFAULT                0x00000000UL                               /**< Mode DEFAULT for CMU_STATUS */
#define CMU_STATUS_LFRCOENS_DEFAULT                 (_CMU_STATUS_LFRCOENS_DEFAULT << 6)        /**< Shifted mode DEFAULT for CMU_STATUS */
#define CMU_STATUS_LFRCORDY                         (0x1UL << 7)                               /**< LFRCO Ready */
#define _CMU_STATUS_LFRCORDY_SHIFT                  7                                          /**< Shift value for CMU_LFRCORDY */
#define _CMU_STATUS_LFRCORDY_MASK                   0x80UL                                     /**< Bit mask for CMU_LFRCORDY */
#define _CMU_STATUS_LFRCORDY_DEFAULT                0x00000000UL                               /**< Mode DEFAULT for CMU_STATUS */
#define CMU_STATUS_LFRCORDY_DEFAULT                 (_CMU_STATUS_LFRCORDY_DEFAULT << 7)        /**< Shifted mode DEFAULT for CMU_STATUS */
#define CMU_STATUS_LFXOENS                          (0x1UL << 8)                               /**< LFXO Enable Status */
#define _CMU_STATUS_LFXOENS_SHIFT                   8                                          /**< Shift value for CMU_LFXOENS */
#define _CMU_STATUS_LFXOENS_MASK                    0x100UL                                    /**< Bit mask for CMU_LFXOENS */
#define _CMU_STATUS_LFXOENS_DEFAULT                 0x00000000UL                               /**< Mode DEFAULT for CMU_STATUS */
#define CMU_STATUS_LFXOENS_DEFAULT                  (_CMU_STATUS_LFXOENS_DEFAULT << 8)         /**< Shifted mode DEFAULT for CMU_STATUS */
#define CMU_STATUS_LFXORDY                          (0x1UL << 9)                               /**< LFXO Ready */
#define _CMU_STATUS_LFXORDY_SHIFT                   9                                          /**< Shift value for CMU_LFXORDY */
#define _CMU_STATUS_LFXORDY_MASK                    0x200UL                                    /**< Bit mask for CMU_LFXORDY */
#define _CMU_STATUS_LFXORDY_DEFAULT                 0x00000000UL                               /**< Mode DEFAULT for CMU_STATUS */
#define CMU_STATUS_LFXORDY_DEFAULT                  (_CMU_STATUS_LFXORDY_DEFAULT << 9)         /**< Shifted mode DEFAULT for CMU_STATUS */
#define CMU_STATUS_HFRCOSEL                         (0x1UL << 10)                              /**< HFRCO Selected */
#define _CMU_STATUS_HFRCOSEL_SHIFT                  10                                         /**< Shift value for CMU_HFRCOSEL */
#define _CMU_STATUS_HFRCOSEL_MASK                   0x400UL                                    /**< Bit mask for CMU_HFRCOSEL */
#define _CMU_STATUS_HFRCOSEL_DEFAULT                0x00000001UL                               /**< Mode DEFAULT for CMU_STATUS */
#define CMU_STATUS_HFRCOSEL_DEFAULT                 (_CMU_STATUS_HFRCOSEL_DEFAULT << 10)       /**< Shifted mode DEFAULT for CMU_STATUS */
#define CMU_STATUS_HFXOSEL                          (0x1UL << 11)                              /**< HFXO Selected */
#define _CMU_STATUS_HFXOSEL_SHIFT                   11                                         /**< Shift value for CMU_HFXOSEL */
#define _CMU_STATUS_HFXOSEL_MASK                    0x800UL                                    /**< Bit mask for CMU_HFXOSEL */
#define _CMU_STATUS_HFXOSEL_DEFAULT                 0x00000000UL                               /**< Mode DEFAULT for CMU_STATUS */
#define CMU_STATUS_HFXOSEL_DEFAULT                  (_CMU_STATUS_HFXOSEL_DEFAULT << 11)        /**< Shifted mode DEFAULT for CMU_STATUS */
#define CMU_STATUS_LFRCOSEL                         (0x1UL << 12)                              /**< LFRCO Selected */
#define _CMU_STATUS_LFRCOSEL_SHIFT                  12                                         /**< Shift value for CMU_LFRCOSEL */
#define _CMU_STATUS_LFRCOSEL_MASK                   0x1000UL                                   /**< Bit mask for CMU_LFRCOSEL */
#define _CMU_STATUS_LFRCOSEL_DEFAULT                0x00000000UL                               /**< Mode DEFAULT for CMU_STATUS */
#define CMU_STATUS_LFRCOSEL_DEFAULT                 (_CMU_STATUS_LFRCOSEL_DEFAULT << 12)       /**< Shifted mode DEFAULT for CMU_STATUS */
#define CMU_STATUS_LFXOSEL                          (0x1UL << 13)                              /**< LFXO Selected */
#define _CMU_STATUS_LFXOSEL_SHIFT                   13                                         /**< Shift value for CMU_LFXOSEL */
#define _CMU_STATUS_LFXOSEL_MASK                    0x2000UL                                   /**< Bit mask for CMU_LFXOSEL */
#define _CMU_STATUS_LFXOSEL_DEFAULT                 0x00000000UL                               /**< Mode DEFAULT for CMU_STATUS */
#define CMU_STATUS_LFXOSEL_DEFAULT                  (_CMU_STATUS_LFXOSEL_DEFAULT << 13)        /**< Shifted mode DEFAULT for CMU_STATUS */
#define CMU_STATUS_CALBSY                           (0x1UL << 14)                              /**< Calibration Busy */
#define _CMU_STATUS_CALBSY_SHIFT                    14                                         /**< Shift value for CMU_CALBSY */
#define _CMU_STATUS_CALBSY_MASK                     0x4000UL                                   /**< Bit mask for CMU_CALBSY */
#define _CMU_STATUS_CALBSY_DEFAULT                  0x00000000UL                               /**< Mode DEFAULT for CMU_STATUS */
#define CMU_STATUS_CALBSY_DEFAULT                   (_CMU_STATUS_CALBSY_DEFAULT << 14)         /**< Shifted mode DEFAULT for CMU_STATUS */
#define CMU_STATUS_USBCLFXOSEL                      (0x1UL << 16)                              /**< USBC LFXO Selected */
#define _CMU_STATUS_USBCLFXOSEL_SHIFT               16                                         /**< Shift value for CMU_USBCLFXOSEL */
#define _CMU_STATUS_USBCLFXOSEL_MASK                0x10000UL                                  /**< Bit mask for CMU_USBCLFXOSEL */
#define _CMU_STATUS_USBCLFXOSEL_DEFAULT             0x00000000UL                               /**< Mode DEFAULT for CMU_STATUS */
#define CMU_STATUS_USBCLFXOSEL_DEFAULT              (_CMU_STATUS_USBCLFXOSEL_DEFAULT << 16)    /**< Shifted mode DEFAULT for CMU_STATUS */
#define CMU_STATUS_USBCLFRCOSEL                     (0x1UL << 17)                              /**< USBC LFRCO Selected */
#define _CMU_STATUS_USBCLFRCOSEL_SHIFT              17                                         /**< Shift value for CMU_USBCLFRCOSEL */
#define _CMU_STATUS_USBCLFRCOSEL_MASK               0x20000UL                                  /**< Bit mask for CMU_USBCLFRCOSEL */
#define _CMU_STATUS_USBCLFRCOSEL_DEFAULT            0x00000000UL                               /**< Mode DEFAULT for CMU_STATUS */
#define CMU_STATUS_USBCLFRCOSEL_DEFAULT             (_CMU_STATUS_USBCLFRCOSEL_DEFAULT << 17)   /**< Shifted mode DEFAULT for CMU_STATUS */
#define CMU_STATUS_USBCUSHFRCOSEL                   (0x1UL << 18)                              /**< USBC USHFRCO Selected */
#define _CMU_STATUS_USBCUSHFRCOSEL_SHIFT            18                                         /**< Shift value for CMU_USBCUSHFRCOSEL */
#define _CMU_STATUS_USBCUSHFRCOSEL_MASK             0x40000UL                                  /**< Bit mask for CMU_USBCUSHFRCOSEL */
#define _CMU_STATUS_USBCUSHFRCOSEL_DEFAULT          0x00000000UL                               /**< Mode DEFAULT for CMU_STATUS */
#define CMU_STATUS_USBCUSHFRCOSEL_DEFAULT           (_CMU_STATUS_USBCUSHFRCOSEL_DEFAULT << 18) /**< Shifted mode DEFAULT for CMU_STATUS */
#define CMU_STATUS_USBCHFCLKSYNC                    (0x1UL << 20)                              /**< USBC is synchronous to HFCLK */
#define _CMU_STATUS_USBCHFCLKSYNC_SHIFT             20                                         /**< Shift value for CMU_USBCHFCLKSYNC */
#define _CMU_STATUS_USBCHFCLKSYNC_MASK              0x100000UL                                 /**< Bit mask for CMU_USBCHFCLKSYNC */
#define _CMU_STATUS_USBCHFCLKSYNC_DEFAULT           0x00000000UL                               /**< Mode DEFAULT for CMU_STATUS */
#define CMU_STATUS_USBCHFCLKSYNC_DEFAULT            (_CMU_STATUS_USBCHFCLKSYNC_DEFAULT << 20)  /**< Shifted mode DEFAULT for CMU_STATUS */
#define CMU_STATUS_USHFRCOENS                       (0x1UL << 21)                              /**< USHFRCO Enable Status */
#define _CMU_STATUS_USHFRCOENS_SHIFT                21                                         /**< Shift value for CMU_USHFRCOENS */
#define _CMU_STATUS_USHFRCOENS_MASK                 0x200000UL                                 /**< Bit mask for CMU_USHFRCOENS */
#define _CMU_STATUS_USHFRCOENS_DEFAULT              0x00000000UL                               /**< Mode DEFAULT for CMU_STATUS */
#define CMU_STATUS_USHFRCOENS_DEFAULT               (_CMU_STATUS_USHFRCOENS_DEFAULT << 21)     /**< Shifted mode DEFAULT for CMU_STATUS */
#define CMU_STATUS_USHFRCORDY                       (0x1UL << 22)                              /**< USHFRCO Ready */
#define _CMU_STATUS_USHFRCORDY_SHIFT                22                                         /**< Shift value for CMU_USHFRCORDY */
#define _CMU_STATUS_USHFRCORDY_MASK                 0x400000UL                                 /**< Bit mask for CMU_USHFRCORDY */
#define _CMU_STATUS_USHFRCORDY_DEFAULT              0x00000000UL                               /**< Mode DEFAULT for CMU_STATUS */
#define CMU_STATUS_USHFRCORDY_DEFAULT               (_CMU_STATUS_USHFRCORDY_DEFAULT << 22)     /**< Shifted mode DEFAULT for CMU_STATUS */
#define CMU_STATUS_USHFRCOSUSPEND                   (0x1UL << 23)                              /**< USHFRCO is suspended */
#define _CMU_STATUS_USHFRCOSUSPEND_SHIFT            23                                         /**< Shift value for CMU_USHFRCOSUSPEND */
#define _CMU_STATUS_USHFRCOSUSPEND_MASK             0x800000UL                                 /**< Bit mask for CMU_USHFRCOSUSPEND */
#define _CMU_STATUS_USHFRCOSUSPEND_DEFAULT          0x00000000UL                               /**< Mode DEFAULT for CMU_STATUS */
#define CMU_STATUS_USHFRCOSUSPEND_DEFAULT           (_CMU_STATUS_USHFRCOSUSPEND_DEFAULT << 23) /**< Shifted mode DEFAULT for CMU_STATUS */
#define CMU_STATUS_USHFRCODIV2SEL                   (0x1UL << 26)                              /**< USHFRCODIV2 Selected */
#define _CMU_STATUS_USHFRCODIV2SEL_SHIFT            26                                         /**< Shift value for CMU_USHFRCODIV2SEL */
#define _CMU_STATUS_USHFRCODIV2SEL_MASK             0x4000000UL                                /**< Bit mask for CMU_USHFRCODIV2SEL */
#define _CMU_STATUS_USHFRCODIV2SEL_DEFAULT          0x00000000UL                               /**< Mode DEFAULT for CMU_STATUS */
#define CMU_STATUS_USHFRCODIV2SEL_DEFAULT           (_CMU_STATUS_USHFRCODIV2SEL_DEFAULT << 26) /**< Shifted mode DEFAULT for CMU_STATUS */

/* Bit fields for CMU USBCRCTRL */
#define _CMU_USBCRCTRL_RESETVALUE                   0x00000000UL                         /**< Default value for CMU_USBCRCTRL */
#define _CMU_USBCRCTRL_MASK                         0x00000003UL                         /**< Mask for CMU_USBCRCTRL */
#define CMU_USBCRCTRL_EN                            (0x1UL << 0)                         /**< Clock Recovery Enable */
#define _CMU_USBCRCTRL_EN_SHIFT                     0                                    /**< Shift value for CMU_EN */
#define _CMU_USBCRCTRL_EN_MASK                      0x1UL                                /**< Bit mask for CMU_EN */
#define _CMU_USBCRCTRL_EN_DEFAULT                   0x00000000UL                         /**< Mode DEFAULT for CMU_USBCRCTRL */
#define CMU_USBCRCTRL_EN_DEFAULT                    (_CMU_USBCRCTRL_EN_DEFAULT << 0)     /**< Shifted mode DEFAULT for CMU_USBCRCTRL */
#define CMU_USBCRCTRL_LSMODE                        (0x1UL << 1)                         /**< Low Speed Clock Recovery Mode */
#define _CMU_USBCRCTRL_LSMODE_SHIFT                 1                                    /**< Shift value for CMU_LSMODE */
#define _CMU_USBCRCTRL_LSMODE_MASK                  0x2UL                                /**< Bit mask for CMU_LSMODE */
#define _CMU_USBCRCTRL_LSMODE_DEFAULT               0x00000000UL                         /**< Mode DEFAULT for CMU_USBCRCTRL */
#define CMU_USBCRCTRL_LSMODE_DEFAULT                (_CMU_USBCRCTRL_LSMODE_DEFAULT << 1) /**< Shifted mode DEFAULT for CMU_USBCRCTRL */

/* Bit fields for CMU USHFRCOCTRL */
#define _CMU_USHFRCOCTRL_RESETVALUE                 0x000FF040UL                             /**< Default value for CMU_USHFRCOCTRL */
#define _CMU_USHFRCOCTRL_MASK                       0x000FF37FUL                             /**< Mask for CMU_USHFRCOCTRL */
#define _CMU_USHFRCOCTRL_TUNING_SHIFT               0                                        /**< Shift value for CMU_TUNING */
#define _CMU_USHFRCOCTRL_TUNING_MASK                0x7FUL                                   /**< Bit mask for CMU_TUNING */
#define _CMU_USHFRCOCTRL_TUNING_DEFAULT             0x00000040UL                             /**< Mode DEFAULT for CMU_USHFRCOCTRL */
#define CMU_USHFRCOCTRL_TUNING_DEFAULT              (_CMU_USHFRCOCTRL_TUNING_DEFAULT << 0)   /**< Shifted mode DEFAULT for CMU_USHFRCOCTRL */
#define CMU_USHFRCOCTRL_DITHEN                      (0x1UL << 8)                             /**< USHFRCO dither enable */
#define _CMU_USHFRCOCTRL_DITHEN_SHIFT               8                                        /**< Shift value for CMU_DITHEN */
#define _CMU_USHFRCOCTRL_DITHEN_MASK                0x100UL                                  /**< Bit mask for CMU_DITHEN */
#define _CMU_USHFRCOCTRL_DITHEN_DEFAULT             0x00000000UL                             /**< Mode DEFAULT for CMU_USHFRCOCTRL */
#define CMU_USHFRCOCTRL_DITHEN_DEFAULT              (_CMU_USHFRCOCTRL_DITHEN_DEFAULT << 8)   /**< Shifted mode DEFAULT for CMU_USHFRCOCTRL */
#define CMU_USHFRCOCTRL_SUSPEND                     (0x1UL << 9)                             /**< USHFRCO suspend */
#define _CMU_USHFRCOCTRL_SUSPEND_SHIFT              9                                        /**< Shift value for CMU_SUSPEND */
#define _CMU_USHFRCOCTRL_SUSPEND_MASK               0x200UL                                  /**< Bit mask for CMU_SUSPEND */
#define _CMU_USHFRCOCTRL_SUSPEND_DEFAULT            0x00000000UL                             /**< Mode DEFAULT for CMU_USHFRCOCTRL */
#define CMU_USHFRCOCTRL_SUSPEND_DEFAULT             (_CMU_USHFRCOCTRL_SUSPEND_DEFAULT << 9)  /**< Shifted mode DEFAULT for CMU_USHFRCOCTRL */
#define _CMU_USHFRCOCTRL_TIMEOUT_SHIFT              12                                       /**< Shift value for CMU_TIMEOUT */
#define _CMU_USHFRCOCTRL_TIMEOUT_MASK               0xFF000UL                                /**< Bit mask for CMU_TIMEOUT */
#define _CMU_USHFRCOCTRL_TIMEOUT_DEFAULT            0x000000FFUL                             /**< Mode DEFAULT for CMU_USHFRCOCTRL */
#define CMU_USHFRCOCTRL_TIMEOUT_DEFAULT             (_CMU_USHFRCOCTRL_TIMEOUT_DEFAULT << 12) /**< Shifted mode DEFAULT for CMU_USHFRCOCTRL */

/* Bit fields for CMU USHFRCOTUNE */
#define _CMU_USHFRCOTUNE_RESETVALUE                 0x00000020UL                               /**< Default value for CMU_USHFRCOTUNE */
#define _CMU_USHFRCOTUNE_MASK                       0x0000003FUL                               /**< Mask for CMU_USHFRCOTUNE */
#define _CMU_USHFRCOTUNE_FINETUNING_SHIFT           0                                          /**< Shift value for CMU_FINETUNING */
#define _CMU_USHFRCOTUNE_FINETUNING_MASK            0x3FUL                                     /**< Bit mask for CMU_FINETUNING */
#define _CMU_USHFRCOTUNE_FINETUNING_DEFAULT         0x00000020UL                               /**< Mode DEFAULT for CMU_USHFRCOTUNE */
#define CMU_USHFRCOTUNE_FINETUNING_DEFAULT          (_CMU_USHFRCOTUNE_FINETUNING_DEFAULT << 0) /**< Shifted mode DEFAULT for CMU_USHFRCOTUNE */

/* Bit fields for CMU USHFRCOCONF */
#define _CMU_USHFRCOCONF_RESETVALUE                 0x00000001UL                                   /**< Default value for CMU_USHFRCOCONF */
#define _CMU_USHFRCOCONF_MASK                       0x00000017UL                                   /**< Mask for CMU_USHFRCOCONF */
#define _CMU_USHFRCOCONF_BAND_SHIFT                 0                                              /**< Shift value for CMU_BAND */
#define _CMU_USHFRCOCONF_BAND_MASK                  0x7UL                                          /**< Bit mask for CMU_BAND */
#define _CMU_USHFRCOCONF_BAND_DEFAULT               0x00000001UL                                   /**< Mode DEFAULT for CMU_USHFRCOCONF */
#define _CMU_USHFRCOCONF_BAND_48MHZ                 0x00000001UL                                   /**< Mode 48MHZ for CMU_USHFRCOCONF */
#define _CMU_USHFRCOCONF_BAND_24MHZ                 0x00000003UL                                   /**< Mode 24MHZ for CMU_USHFRCOCONF */
#define CMU_USHFRCOCONF_BAND_DEFAULT                (_CMU_USHFRCOCONF_BAND_DEFAULT << 0)           /**< Shifted mode DEFAULT for CMU_USHFRCOCONF */
#define CMU_USHFRCOCONF_BAND_48MHZ                  (_CMU_USHFRCOCONF_BAND_48MHZ << 0)             /**< Shifted mode 48MHZ for CMU_USHFRCOCONF */
#define CMU_USHFRCOCONF_BAND_24MHZ                  (_CMU_USHFRCOCONF_BAND_24MHZ << 0)             /**< Shifted mode 24MHZ for CMU_USHFRCOCONF */
#define CMU_USHFRCOCONF_USHFRCODIV2DIS              (0x1UL << 4)                                   /**< USHFRCO divider for HFCLK disable */
#define _CMU_USHFRCOCONF_USHFRCODIV2DIS_SHIFT       4                                              /**< Shift value for CMU_USHFRCODIV2DIS */
#define _CMU_USHFRCOCONF_USHFRCODIV2DIS_MASK        0x10UL                                         /**< Bit mask for CMU_USHFRCODIV2DIS */
#define _CMU_USHFRCOCONF_USHFRCODIV2DIS_DEFAULT     0x00000000UL                                   /**< Mode DEFAULT for CMU_USHFRCOCONF */
#define CMU_USHFRCOCONF_USHFRCODIV2DIS_DEFAULT (_CMU_USHFRCOCONF_USHFRCODIV2DIS_DEFAULT << 4) /**< Shifted mode DEFAULT for CMU_USHFRCOCONF */

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

/* Bit fields for USB CTRL */
#define _USB_CTRL_RESETVALUE                       0x00000020UL                           /**< Default value for USB_CTRL */
#define _USB_CTRL_MASK                             0x03330EB2UL                           /**< Mask for USB_CTRL */
#define USB_CTRL_DMPUAP                            (0x1UL << 1)                           /**< DMPU Active Polarity */
#define _USB_CTRL_DMPUAP_SHIFT                     1                                      /**< Shift value for USB_DMPUAP */
#define _USB_CTRL_DMPUAP_MASK                      0x2UL                                  /**< Bit mask for USB_DMPUAP */
#define _USB_CTRL_DMPUAP_DEFAULT                   0x00000000UL                           /**< Mode DEFAULT for USB_CTRL */
#define _USB_CTRL_DMPUAP_LOW                       0x00000000UL                           /**< Mode LOW for USB_CTRL */
#define _USB_CTRL_DMPUAP_HIGH                      0x00000001UL                           /**< Mode HIGH for USB_CTRL */
#define USB_CTRL_DMPUAP_DEFAULT                    (_USB_CTRL_DMPUAP_DEFAULT << 1)        /**< Shifted mode DEFAULT for USB_CTRL */
#define USB_CTRL_DMPUAP_LOW                        (_USB_CTRL_DMPUAP_LOW << 1)            /**< Shifted mode LOW for USB_CTRL */
#define USB_CTRL_DMPUAP_HIGH                       (_USB_CTRL_DMPUAP_HIGH << 1)           /**< Shifted mode HIGH for USB_CTRL */
#define _USB_CTRL_LEMOSCCTRL_SHIFT                 4                                      /**< Shift value for USB_LEMOSCCTRL */
#define _USB_CTRL_LEMOSCCTRL_MASK                  0x30UL                                 /**< Bit mask for USB_LEMOSCCTRL */
#define _USB_CTRL_LEMOSCCTRL_NONE                  0x00000000UL                           /**< Mode NONE for USB_CTRL */
#define _USB_CTRL_LEMOSCCTRL_GATE                  0x00000001UL                           /**< Mode GATE for USB_CTRL */
#define _USB_CTRL_LEMOSCCTRL_DEFAULT               0x00000002UL                           /**< Mode DEFAULT for USB_CTRL */
#define _USB_CTRL_LEMOSCCTRL_SUSPEND               0x00000002UL                           /**< Mode SUSPEND for USB_CTRL */
#define USB_CTRL_LEMOSCCTRL_NONE                   (_USB_CTRL_LEMOSCCTRL_NONE << 4)       /**< Shifted mode NONE for USB_CTRL */
#define USB_CTRL_LEMOSCCTRL_GATE                   (_USB_CTRL_LEMOSCCTRL_GATE << 4)       /**< Shifted mode GATE for USB_CTRL */
#define USB_CTRL_LEMOSCCTRL_DEFAULT                (_USB_CTRL_LEMOSCCTRL_DEFAULT << 4)    /**< Shifted mode DEFAULT for USB_CTRL */
#define USB_CTRL_LEMOSCCTRL_SUSPEND                (_USB_CTRL_LEMOSCCTRL_SUSPEND << 4)    /**< Shifted mode SUSPEND for USB_CTRL */
#define USB_CTRL_LEMPHYCTRL                        (0x1UL << 7)                           /**< Low Energy Mode USB PHY Control */
#define _USB_CTRL_LEMPHYCTRL_SHIFT                 7                                      /**< Shift value for USB_LEMPHYCTRL */
#define _USB_CTRL_LEMPHYCTRL_MASK                  0x80UL                                 /**< Bit mask for USB_LEMPHYCTRL */
#define _USB_CTRL_LEMPHYCTRL_DEFAULT               0x00000000UL                           /**< Mode DEFAULT for USB_CTRL */
#define _USB_CTRL_LEMPHYCTRL_NONE                  0x00000000UL                           /**< Mode NONE for USB_CTRL */
#define _USB_CTRL_LEMPHYCTRL_LEM                   0x00000001UL                           /**< Mode LEM for USB_CTRL */
#define USB_CTRL_LEMPHYCTRL_DEFAULT                (_USB_CTRL_LEMPHYCTRL_DEFAULT << 7)    /**< Shifted mode DEFAULT for USB_CTRL */
#define USB_CTRL_LEMPHYCTRL_NONE                   (_USB_CTRL_LEMPHYCTRL_NONE << 7)       /**< Shifted mode NONE for USB_CTRL */
#define USB_CTRL_LEMPHYCTRL_LEM                    (_USB_CTRL_LEMPHYCTRL_LEM << 7)        /**< Shifted mode LEM for USB_CTRL */
#define USB_CTRL_LEMIDLEEN                         (0x1UL << 9)                           /**< Low Energy Mode on  Bus Idle Enable */
#define _USB_CTRL_LEMIDLEEN_SHIFT                  9                                      /**< Shift value for USB_LEMIDLEEN */
#define _USB_CTRL_LEMIDLEEN_MASK                   0x200UL                                /**< Bit mask for USB_LEMIDLEEN */
#define _USB_CTRL_LEMIDLEEN_DEFAULT                0x00000000UL                           /**< Mode DEFAULT for USB_CTRL */
#define USB_CTRL_LEMIDLEEN_DEFAULT                 (_USB_CTRL_LEMIDLEEN_DEFAULT << 9)     /**< Shifted mode DEFAULT for USB_CTRL */
#define USB_CTRL_LEMNAKEN                          (0x1UL << 10)                          /**< Low Energy Mode on OUT NAK Enable */
#define _USB_CTRL_LEMNAKEN_SHIFT                   10                                     /**< Shift value for USB_LEMNAKEN */
#define _USB_CTRL_LEMNAKEN_MASK                    0x400UL                                /**< Bit mask for USB_LEMNAKEN */
#define _USB_CTRL_LEMNAKEN_DEFAULT                 0x00000000UL                           /**< Mode DEFAULT for USB_CTRL */
#define USB_CTRL_LEMNAKEN_DEFAULT                  (_USB_CTRL_LEMNAKEN_DEFAULT << 10)     /**< Shifted mode DEFAULT for USB_CTRL */
#define USB_CTRL_LEMADDRMEN                        (0x1UL << 11)                          /**< Low Energy Mode on Device Address Mismatch Enable */
#define _USB_CTRL_LEMADDRMEN_SHIFT                 11                                     /**< Shift value for USB_LEMADDRMEN */
#define _USB_CTRL_LEMADDRMEN_MASK                  0x800UL                                /**< Bit mask for USB_LEMADDRMEN */
#define _USB_CTRL_LEMADDRMEN_DEFAULT               0x00000000UL                           /**< Mode DEFAULT for USB_CTRL */
#define USB_CTRL_LEMADDRMEN_DEFAULT                (_USB_CTRL_LEMADDRMEN_DEFAULT << 11)   /**< Shifted mode DEFAULT for USB_CTRL */
#define USB_CTRL_VREGDIS                           (0x1UL << 16)                          /**< Voltage Regulator Disable */
#define _USB_CTRL_VREGDIS_SHIFT                    16                                     /**< Shift value for USB_VREGDIS */
#define _USB_CTRL_VREGDIS_MASK                     0x10000UL                              /**< Bit mask for USB_VREGDIS */
#define _USB_CTRL_VREGDIS_DEFAULT                  0x00000000UL                           /**< Mode DEFAULT for USB_CTRL */
#define USB_CTRL_VREGDIS_DEFAULT                   (_USB_CTRL_VREGDIS_DEFAULT << 16)      /**< Shifted mode DEFAULT for USB_CTRL */
#define USB_CTRL_VREGOSEN                          (0x1UL << 17)                          /**< VREGO Sense Enable */
#define _USB_CTRL_VREGOSEN_SHIFT                   17                                     /**< Shift value for USB_VREGOSEN */
#define _USB_CTRL_VREGOSEN_MASK                    0x20000UL                              /**< Bit mask for USB_VREGOSEN */
#define _USB_CTRL_VREGOSEN_DEFAULT                 0x00000000UL                           /**< Mode DEFAULT for USB_CTRL */
#define USB_CTRL_VREGOSEN_DEFAULT                  (_USB_CTRL_VREGOSEN_DEFAULT << 17)     /**< Shifted mode DEFAULT for USB_CTRL */
#define _USB_CTRL_BIASPROGEM01_SHIFT               20                                     /**< Shift value for USB_BIASPROGEM01 */
#define _USB_CTRL_BIASPROGEM01_MASK                0x300000UL                             /**< Bit mask for USB_BIASPROGEM01 */
#define _USB_CTRL_BIASPROGEM01_DEFAULT             0x00000000UL                           /**< Mode DEFAULT for USB_CTRL */
#define USB_CTRL_BIASPROGEM01_DEFAULT              (_USB_CTRL_BIASPROGEM01_DEFAULT << 20) /**< Shifted mode DEFAULT for USB_CTRL */
#define _USB_CTRL_BIASPROGEM23_SHIFT               24                                     /**< Shift value for USB_BIASPROGEM23 */
#define _USB_CTRL_BIASPROGEM23_MASK                0x3000000UL                            /**< Bit mask for USB_BIASPROGEM23 */
#define _USB_CTRL_BIASPROGEM23_DEFAULT             0x00000000UL                           /**< Mode DEFAULT for USB_CTRL */
#define USB_CTRL_BIASPROGEM23_DEFAULT              (_USB_CTRL_BIASPROGEM23_DEFAULT << 24) /**< Shifted mode DEFAULT for USB_CTRL */

/* Bit fields for USB STATUS */
#define _USB_STATUS_RESETVALUE                     0x00000000UL                         /**< Default value for USB_STATUS */
#define _USB_STATUS_MASK                           0x00000005UL                         /**< Mask for USB_STATUS */
#define USB_STATUS_VREGOS                          (0x1UL << 0)                         /**< VREGO Sense Output */
#define _USB_STATUS_VREGOS_SHIFT                   0                                    /**< Shift value for USB_VREGOS */
#define _USB_STATUS_VREGOS_MASK                    0x1UL                                /**< Bit mask for USB_VREGOS */
#define _USB_STATUS_VREGOS_DEFAULT                 0x00000000UL                         /**< Mode DEFAULT for USB_STATUS */
#define USB_STATUS_VREGOS_DEFAULT                  (_USB_STATUS_VREGOS_DEFAULT << 0)    /**< Shifted mode DEFAULT for USB_STATUS */
#define USB_STATUS_LEMACTIVE                       (0x1UL << 2)                         /**< Low Energy Mode Active */
#define _USB_STATUS_LEMACTIVE_SHIFT                2                                    /**< Shift value for USB_LEMACTIVE */
#define _USB_STATUS_LEMACTIVE_MASK                 0x4UL                                /**< Bit mask for USB_LEMACTIVE */
#define _USB_STATUS_LEMACTIVE_DEFAULT              0x00000000UL                         /**< Mode DEFAULT for USB_STATUS */
#define USB_STATUS_LEMACTIVE_DEFAULT               (_USB_STATUS_LEMACTIVE_DEFAULT << 2) /**< Shifted mode DEFAULT for USB_STATUS */

/* Bit fields for USB IF */
#define _USB_IF_RESETVALUE                         0x00000003UL                   /**< Default value for USB_IF */
#define _USB_IF_MASK                               0x00000003UL                   /**< Mask for USB_IF */
#define USB_IF_VREGOSH                             (0x1UL << 0)                   /**< VREGO Sense High Interrupt Flag */
#define _USB_IF_VREGOSH_SHIFT                      0                              /**< Shift value for USB_VREGOSH */
#define _USB_IF_VREGOSH_MASK                       0x1UL                          /**< Bit mask for USB_VREGOSH */
#define _USB_IF_VREGOSH_DEFAULT                    0x00000001UL                   /**< Mode DEFAULT for USB_IF */
#define USB_IF_VREGOSH_DEFAULT                     (_USB_IF_VREGOSH_DEFAULT << 0) /**< Shifted mode DEFAULT for USB_IF */
#define USB_IF_VREGOSL                             (0x1UL << 1)                   /**< VREGO Sense Low Interrupt Flag */
#define _USB_IF_VREGOSL_SHIFT                      1                              /**< Shift value for USB_VREGOSL */
#define _USB_IF_VREGOSL_MASK                       0x2UL                          /**< Bit mask for USB_VREGOSL */
#define _USB_IF_VREGOSL_DEFAULT                    0x00000001UL                   /**< Mode DEFAULT for USB_IF */
#define USB_IF_VREGOSL_DEFAULT                     (_USB_IF_VREGOSL_DEFAULT << 1) /**< Shifted mode DEFAULT for USB_IF */

/* Bit fields for USB IFS */
#define _USB_IFS_RESETVALUE                        0x00000000UL                    /**< Default value for USB_IFS */
#define _USB_IFS_MASK                              0x00000003UL                    /**< Mask for USB_IFS */
#define USB_IFS_VREGOSH                            (0x1UL << 0)                    /**< Set VREGO Sense High Interrupt Flag */
#define _USB_IFS_VREGOSH_SHIFT                     0                               /**< Shift value for USB_VREGOSH */
#define _USB_IFS_VREGOSH_MASK                      0x1UL                           /**< Bit mask for USB_VREGOSH */
#define _USB_IFS_VREGOSH_DEFAULT                   0x00000000UL                    /**< Mode DEFAULT for USB_IFS */
#define USB_IFS_VREGOSH_DEFAULT                    (_USB_IFS_VREGOSH_DEFAULT << 0) /**< Shifted mode DEFAULT for USB_IFS */
#define USB_IFS_VREGOSL                            (0x1UL << 1)                    /**< Set VREGO Sense Low Interrupt Flag */
#define _USB_IFS_VREGOSL_SHIFT                     1                               /**< Shift value for USB_VREGOSL */
#define _USB_IFS_VREGOSL_MASK                      0x2UL                           /**< Bit mask for USB_VREGOSL */
#define _USB_IFS_VREGOSL_DEFAULT                   0x00000000UL                    /**< Mode DEFAULT for USB_IFS */
#define USB_IFS_VREGOSL_DEFAULT                    (_USB_IFS_VREGOSL_DEFAULT << 1) /**< Shifted mode DEFAULT for USB_IFS */

/* Bit fields for USB IFC */
#define _USB_IFC_RESETVALUE                        0x00000000UL                    /**< Default value for USB_IFC */
#define _USB_IFC_MASK                              0x00000003UL                    /**< Mask for USB_IFC */
#define USB_IFC_VREGOSH                            (0x1UL << 0)                    /**< Clear VREGO Sense High Interrupt Flag */
#define _USB_IFC_VREGOSH_SHIFT                     0                               /**< Shift value for USB_VREGOSH */
#define _USB_IFC_VREGOSH_MASK                      0x1UL                           /**< Bit mask for USB_VREGOSH */
#define _USB_IFC_VREGOSH_DEFAULT                   0x00000000UL                    /**< Mode DEFAULT for USB_IFC */
#define USB_IFC_VREGOSH_DEFAULT                    (_USB_IFC_VREGOSH_DEFAULT << 0) /**< Shifted mode DEFAULT for USB_IFC */
#define USB_IFC_VREGOSL                            (0x1UL << 1)                    /**< Clear VREGO Sense Low Interrupt Flag */
#define _USB_IFC_VREGOSL_SHIFT                     1                               /**< Shift value for USB_VREGOSL */
#define _USB_IFC_VREGOSL_MASK                      0x2UL                           /**< Bit mask for USB_VREGOSL */
#define _USB_IFC_VREGOSL_DEFAULT                   0x00000000UL                    /**< Mode DEFAULT for USB_IFC */
#define USB_IFC_VREGOSL_DEFAULT                    (_USB_IFC_VREGOSL_DEFAULT << 1) /**< Shifted mode DEFAULT for USB_IFC */

/* Bit fields for USB IEN */
#define _USB_IEN_RESETVALUE                        0x00000000UL                    /**< Default value for USB_IEN */
#define _USB_IEN_MASK                              0x00000003UL                    /**< Mask for USB_IEN */
#define USB_IEN_VREGOSH                            (0x1UL << 0)                    /**< VREGO Sense High Interrupt Enable */
#define _USB_IEN_VREGOSH_SHIFT                     0                               /**< Shift value for USB_VREGOSH */
#define _USB_IEN_VREGOSH_MASK                      0x1UL                           /**< Bit mask for USB_VREGOSH */
#define _USB_IEN_VREGOSH_DEFAULT                   0x00000000UL                    /**< Mode DEFAULT for USB_IEN */
#define USB_IEN_VREGOSH_DEFAULT                    (_USB_IEN_VREGOSH_DEFAULT << 0) /**< Shifted mode DEFAULT for USB_IEN */
#define USB_IEN_VREGOSL                            (0x1UL << 1)                    /**< VREGO Sense Low Interrupt Enable */
#define _USB_IEN_VREGOSL_SHIFT                     1                               /**< Shift value for USB_VREGOSL */
#define _USB_IEN_VREGOSL_MASK                      0x2UL                           /**< Bit mask for USB_VREGOSL */
#define _USB_IEN_VREGOSL_DEFAULT                   0x00000000UL                    /**< Mode DEFAULT for USB_IEN */
#define USB_IEN_VREGOSL_DEFAULT                    (_USB_IEN_VREGOSL_DEFAULT << 1) /**< Shifted mode DEFAULT for USB_IEN */

/* Bit fields for USB ROUTE */
#define _USB_ROUTE_RESETVALUE                      0x00000000UL                      /**< Default value for USB_ROUTE */
#define _USB_ROUTE_MASK                            0x00000005UL                      /**< Mask for USB_ROUTE */
#define USB_ROUTE_PHYPEN                           (0x1UL << 0)                      /**< USB PHY Pin Enable */
#define _USB_ROUTE_PHYPEN_SHIFT                    0                                 /**< Shift value for USB_PHYPEN */
#define _USB_ROUTE_PHYPEN_MASK                     0x1UL                             /**< Bit mask for USB_PHYPEN */
#define _USB_ROUTE_PHYPEN_DEFAULT                  0x00000000UL                      /**< Mode DEFAULT for USB_ROUTE */
#define USB_ROUTE_PHYPEN_DEFAULT                   (_USB_ROUTE_PHYPEN_DEFAULT << 0)  /**< Shifted mode DEFAULT for USB_ROUTE */
#define USB_ROUTE_DMPUPEN                          (0x1UL << 2)                      /**< DMPU Pin Enable */
#define _USB_ROUTE_DMPUPEN_SHIFT                   2                                 /**< Shift value for USB_DMPUPEN */
#define _USB_ROUTE_DMPUPEN_MASK                    0x4UL                             /**< Bit mask for USB_DMPUPEN */
#define _USB_ROUTE_DMPUPEN_DEFAULT                 0x00000000UL                      /**< Mode DEFAULT for USB_ROUTE */
#define USB_ROUTE_DMPUPEN_DEFAULT (_USB_ROUTE_DMPUPEN_DEFAULT << 2) /**< Shifted mode DEFAULT for USB_ROUTE */

/* Bit fields for USB GAHBCFG */
#define _USB_GAHBCFG_RESETVALUE                    0x00000000UL                                /**< Default value for USB_GAHBCFG */
#define _USB_GAHBCFG_MASK                          0x00E000BFUL                                /**< Mask for USB_GAHBCFG */
#define USB_GAHBCFG_GLBLINTRMSK                    (0x1UL << 0)                                /**< Global Interrupt Mask */
#define _USB_GAHBCFG_GLBLINTRMSK_SHIFT             0                                           /**< Shift value for USB_GLBLINTRMSK */
#define _USB_GAHBCFG_GLBLINTRMSK_MASK              0x1UL                                       /**< Bit mask for USB_GLBLINTRMSK */
#define _USB_GAHBCFG_GLBLINTRMSK_DEFAULT           0x00000000UL                                /**< Mode DEFAULT for USB_GAHBCFG */
#define USB_GAHBCFG_GLBLINTRMSK_DEFAULT            (_USB_GAHBCFG_GLBLINTRMSK_DEFAULT << 0)     /**< Shifted mode DEFAULT for USB_GAHBCFG */
#define _USB_GAHBCFG_HBSTLEN_SHIFT                 1                                           /**< Shift value for USB_HBSTLEN */
#define _USB_GAHBCFG_HBSTLEN_MASK                  0x1EUL                                      /**< Bit mask for USB_HBSTLEN */
#define _USB_GAHBCFG_HBSTLEN_DEFAULT               0x00000000UL                                /**< Mode DEFAULT for USB_GAHBCFG */
#define _USB_GAHBCFG_HBSTLEN_SINGLE                0x00000000UL                                /**< Mode SINGLE for USB_GAHBCFG */
#define _USB_GAHBCFG_HBSTLEN_INCR                  0x00000001UL                                /**< Mode INCR for USB_GAHBCFG */
#define _USB_GAHBCFG_HBSTLEN_INCR4                 0x00000003UL                                /**< Mode INCR4 for USB_GAHBCFG */
#define _USB_GAHBCFG_HBSTLEN_INCR8                 0x00000005UL                                /**< Mode INCR8 for USB_GAHBCFG */
#define _USB_GAHBCFG_HBSTLEN_INCR16                0x00000007UL                                /**< Mode INCR16 for USB_GAHBCFG */
#define USB_GAHBCFG_HBSTLEN_DEFAULT                (_USB_GAHBCFG_HBSTLEN_DEFAULT << 1)         /**< Shifted mode DEFAULT for USB_GAHBCFG */
#define USB_GAHBCFG_HBSTLEN_SINGLE                 (_USB_GAHBCFG_HBSTLEN_SINGLE << 1)          /**< Shifted mode SINGLE for USB_GAHBCFG */
#define USB_GAHBCFG_HBSTLEN_INCR                   (_USB_GAHBCFG_HBSTLEN_INCR << 1)            /**< Shifted mode INCR for USB_GAHBCFG */
#define USB_GAHBCFG_HBSTLEN_INCR4                  (_USB_GAHBCFG_HBSTLEN_INCR4 << 1)           /**< Shifted mode INCR4 for USB_GAHBCFG */
#define USB_GAHBCFG_HBSTLEN_INCR8                  (_USB_GAHBCFG_HBSTLEN_INCR8 << 1)           /**< Shifted mode INCR8 for USB_GAHBCFG */
#define USB_GAHBCFG_HBSTLEN_INCR16                 (_USB_GAHBCFG_HBSTLEN_INCR16 << 1)          /**< Shifted mode INCR16 for USB_GAHBCFG */
#define USB_GAHBCFG_DMAEN                          (0x1UL << 5)                                /**< DMA Enable */
#define _USB_GAHBCFG_DMAEN_SHIFT                   5                                           /**< Shift value for USB_DMAEN */
#define _USB_GAHBCFG_DMAEN_MASK                    0x20UL                                      /**< Bit mask for USB_DMAEN */
#define _USB_GAHBCFG_DMAEN_DEFAULT                 0x00000000UL                                /**< Mode DEFAULT for USB_GAHBCFG */
#define USB_GAHBCFG_DMAEN_DEFAULT                  (_USB_GAHBCFG_DMAEN_DEFAULT << 5)           /**< Shifted mode DEFAULT for USB_GAHBCFG */
#define USB_GAHBCFG_NPTXFEMPLVL                    (0x1UL << 7)                                /**< Non-Periodic TxFIFO Empty Level */
#define _USB_GAHBCFG_NPTXFEMPLVL_SHIFT             7                                           /**< Shift value for USB_NPTXFEMPLVL */
#define _USB_GAHBCFG_NPTXFEMPLVL_MASK              0x80UL                                      /**< Bit mask for USB_NPTXFEMPLVL */
#define _USB_GAHBCFG_NPTXFEMPLVL_DEFAULT           0x00000000UL                                /**< Mode DEFAULT for USB_GAHBCFG */
#define _USB_GAHBCFG_NPTXFEMPLVL_HALFEMPTY         0x00000000UL                                /**< Mode HALFEMPTY for USB_GAHBCFG */
#define _USB_GAHBCFG_NPTXFEMPLVL_EMPTY             0x00000001UL                                /**< Mode EMPTY for USB_GAHBCFG */
#define USB_GAHBCFG_NPTXFEMPLVL_DEFAULT            (_USB_GAHBCFG_NPTXFEMPLVL_DEFAULT << 7)     /**< Shifted mode DEFAULT for USB_GAHBCFG */
#define USB_GAHBCFG_NPTXFEMPLVL_HALFEMPTY          (_USB_GAHBCFG_NPTXFEMPLVL_HALFEMPTY << 7)   /**< Shifted mode HALFEMPTY for USB_GAHBCFG */
#define USB_GAHBCFG_NPTXFEMPLVL_EMPTY              (_USB_GAHBCFG_NPTXFEMPLVL_EMPTY << 7)       /**< Shifted mode EMPTY for USB_GAHBCFG */
#define USB_GAHBCFG_REMMEMSUPP                     (0x1UL << 21)                               /**< Remote Memory Support */
#define _USB_GAHBCFG_REMMEMSUPP_SHIFT              21                                          /**< Shift value for USB_REMMEMSUPP */
#define _USB_GAHBCFG_REMMEMSUPP_MASK               0x200000UL                                  /**< Bit mask for USB_REMMEMSUPP */
#define _USB_GAHBCFG_REMMEMSUPP_DEFAULT            0x00000000UL                                /**< Mode DEFAULT for USB_GAHBCFG */
#define USB_GAHBCFG_REMMEMSUPP_DEFAULT             (_USB_GAHBCFG_REMMEMSUPP_DEFAULT << 21)     /**< Shifted mode DEFAULT for USB_GAHBCFG */
#define USB_GAHBCFG_NOTIALLDMAWRIT                 (0x1UL << 22)                               /**< Notify All DMA Writes */
#define _USB_GAHBCFG_NOTIALLDMAWRIT_SHIFT          22                                          /**< Shift value for USB_NOTIALLDMAWRIT */
#define _USB_GAHBCFG_NOTIALLDMAWRIT_MASK           0x400000UL                                  /**< Bit mask for USB_NOTIALLDMAWRIT */
#define _USB_GAHBCFG_NOTIALLDMAWRIT_DEFAULT        0x00000000UL                                /**< Mode DEFAULT for USB_GAHBCFG */
#define USB_GAHBCFG_NOTIALLDMAWRIT_DEFAULT         (_USB_GAHBCFG_NOTIALLDMAWRIT_DEFAULT << 22) /**< Shifted mode DEFAULT for USB_GAHBCFG */
#define USB_GAHBCFG_AHBSINGLE                      (0x1UL << 23)                               /**< AHB Single Support */
#define _USB_GAHBCFG_AHBSINGLE_SHIFT               23                                          /**< Shift value for USB_AHBSINGLE */
#define _USB_GAHBCFG_AHBSINGLE_MASK                0x800000UL                                  /**< Bit mask for USB_AHBSINGLE */
#define _USB_GAHBCFG_AHBSINGLE_DEFAULT             0x00000000UL                                /**< Mode DEFAULT for USB_GAHBCFG */
#define USB_GAHBCFG_AHBSINGLE_DEFAULT              (_USB_GAHBCFG_AHBSINGLE_DEFAULT << 23)      /**< Shifted mode DEFAULT for USB_GAHBCFG */

/* Bit fields for USB GUSBCFG */
#define _USB_GUSBCFG_RESETVALUE                    0x00001440UL                                /**< Default value for USB_GUSBCFG */
#define _USB_GUSBCFG_MASK                          0x90403C27UL                                /**< Mask for USB_GUSBCFG */
#define _USB_GUSBCFG_TOUTCAL_SHIFT                 0                                           /**< Shift value for USB_TOUTCAL */
#define _USB_GUSBCFG_TOUTCAL_MASK                  0x7UL                                       /**< Bit mask for USB_TOUTCAL */
#define _USB_GUSBCFG_TOUTCAL_DEFAULT               0x00000000UL                                /**< Mode DEFAULT for USB_GUSBCFG */
#define USB_GUSBCFG_TOUTCAL_DEFAULT                (_USB_GUSBCFG_TOUTCAL_DEFAULT << 0)         /**< Shifted mode DEFAULT for USB_GUSBCFG */
#define USB_GUSBCFG_FSINTF                         (0x1UL << 5)                                /**< Full-Speed Serial Interface Select */
#define _USB_GUSBCFG_FSINTF_SHIFT                  5                                           /**< Shift value for USB_FSINTF */
#define _USB_GUSBCFG_FSINTF_MASK                   0x20UL                                      /**< Bit mask for USB_FSINTF */
#define _USB_GUSBCFG_FSINTF_DEFAULT                0x00000000UL                                /**< Mode DEFAULT for USB_GUSBCFG */
#define USB_GUSBCFG_FSINTF_DEFAULT                 (_USB_GUSBCFG_FSINTF_DEFAULT << 5)          /**< Shifted mode DEFAULT for USB_GUSBCFG */
#define _USB_GUSBCFG_USBTRDTIM_SHIFT               10                                          /**< Shift value for USB_USBTRDTIM */
#define _USB_GUSBCFG_USBTRDTIM_MASK                0x3C00UL                                    /**< Bit mask for USB_USBTRDTIM */
#define _USB_GUSBCFG_USBTRDTIM_DEFAULT             0x00000005UL                                /**< Mode DEFAULT for USB_GUSBCFG */
#define USB_GUSBCFG_USBTRDTIM_DEFAULT              (_USB_GUSBCFG_USBTRDTIM_DEFAULT << 10)      /**< Shifted mode DEFAULT for USB_GUSBCFG */
#define USB_GUSBCFG_TERMSELDLPULSE                 (0x1UL << 22)                               /**< TermSel DLine Pulsing Selection */
#define _USB_GUSBCFG_TERMSELDLPULSE_SHIFT          22                                          /**< Shift value for USB_TERMSELDLPULSE */
#define _USB_GUSBCFG_TERMSELDLPULSE_MASK           0x400000UL                                  /**< Bit mask for USB_TERMSELDLPULSE */
#define _USB_GUSBCFG_TERMSELDLPULSE_DEFAULT        0x00000000UL                                /**< Mode DEFAULT for USB_GUSBCFG */
#define _USB_GUSBCFG_TERMSELDLPULSE_TXVALID        0x00000000UL                                /**< Mode TXVALID for USB_GUSBCFG */
#define _USB_GUSBCFG_TERMSELDLPULSE_TERMSEL        0x00000001UL                                /**< Mode TERMSEL for USB_GUSBCFG */
#define USB_GUSBCFG_TERMSELDLPULSE_DEFAULT         (_USB_GUSBCFG_TERMSELDLPULSE_DEFAULT << 22) /**< Shifted mode DEFAULT for USB_GUSBCFG */
#define USB_GUSBCFG_TERMSELDLPULSE_TXVALID         (_USB_GUSBCFG_TERMSELDLPULSE_TXVALID << 22) /**< Shifted mode TXVALID for USB_GUSBCFG */
#define USB_GUSBCFG_TERMSELDLPULSE_TERMSEL         (_USB_GUSBCFG_TERMSELDLPULSE_TERMSEL << 22) /**< Shifted mode TERMSEL for USB_GUSBCFG */
#define USB_GUSBCFG_TXENDDELAY                     (0x1UL << 28)                               /**< Tx End Delay */
#define _USB_GUSBCFG_TXENDDELAY_SHIFT              28                                          /**< Shift value for USB_TXENDDELAY */
#define _USB_GUSBCFG_TXENDDELAY_MASK               0x10000000UL                                /**< Bit mask for USB_TXENDDELAY */
#define _USB_GUSBCFG_TXENDDELAY_DEFAULT            0x00000000UL                                /**< Mode DEFAULT for USB_GUSBCFG */
#define USB_GUSBCFG_TXENDDELAY_DEFAULT             (_USB_GUSBCFG_TXENDDELAY_DEFAULT << 28)     /**< Shifted mode DEFAULT for USB_GUSBCFG */
#define USB_GUSBCFG_CORRUPTTXPKT                   (0x1UL << 31)                               /**< Corrupt Tx packet */
#define _USB_GUSBCFG_CORRUPTTXPKT_SHIFT            31                                          /**< Shift value for USB_CORRUPTTXPKT */
#define _USB_GUSBCFG_CORRUPTTXPKT_MASK             0x80000000UL                                /**< Bit mask for USB_CORRUPTTXPKT */
#define _USB_GUSBCFG_CORRUPTTXPKT_DEFAULT          0x00000000UL                                /**< Mode DEFAULT for USB_GUSBCFG */
#define USB_GUSBCFG_CORRUPTTXPKT_DEFAULT           (_USB_GUSBCFG_CORRUPTTXPKT_DEFAULT << 31)   /**< Shifted mode DEFAULT for USB_GUSBCFG */

/* Bit fields for USB GRSTCTL */
#define _USB_GRSTCTL_RESETVALUE                    0x80000000UL                            /**< Default value for USB_GRSTCTL */
#define _USB_GRSTCTL_MASK                          0xC00007F3UL                            /**< Mask for USB_GRSTCTL */
#define USB_GRSTCTL_CSFTRST                        (0x1UL << 0)                            /**< Core Soft Reset */
#define _USB_GRSTCTL_CSFTRST_SHIFT                 0                                       /**< Shift value for USB_CSFTRST */
#define _USB_GRSTCTL_CSFTRST_MASK                  0x1UL                                   /**< Bit mask for USB_CSFTRST */
#define _USB_GRSTCTL_CSFTRST_DEFAULT               0x00000000UL                            /**< Mode DEFAULT for USB_GRSTCTL */
#define USB_GRSTCTL_CSFTRST_DEFAULT                (_USB_GRSTCTL_CSFTRST_DEFAULT << 0)     /**< Shifted mode DEFAULT for USB_GRSTCTL */
#define USB_GRSTCTL_PIUFSSFTRST                    (0x1UL << 1)                            /**< PIU FS Dedicated Controller Soft Reset */
#define _USB_GRSTCTL_PIUFSSFTRST_SHIFT             1                                       /**< Shift value for USB_PIUFSSFTRST */
#define _USB_GRSTCTL_PIUFSSFTRST_MASK              0x2UL                                   /**< Bit mask for USB_PIUFSSFTRST */
#define _USB_GRSTCTL_PIUFSSFTRST_DEFAULT           0x00000000UL                            /**< Mode DEFAULT for USB_GRSTCTL */
#define USB_GRSTCTL_PIUFSSFTRST_DEFAULT            (_USB_GRSTCTL_PIUFSSFTRST_DEFAULT << 1) /**< Shifted mode DEFAULT for USB_GRSTCTL */
#define USB_GRSTCTL_RXFFLSH                        (0x1UL << 4)                            /**< RxFIFO Flush */
#define _USB_GRSTCTL_RXFFLSH_SHIFT                 4                                       /**< Shift value for USB_RXFFLSH */
#define _USB_GRSTCTL_RXFFLSH_MASK                  0x10UL                                  /**< Bit mask for USB_RXFFLSH */
#define _USB_GRSTCTL_RXFFLSH_DEFAULT               0x00000000UL                            /**< Mode DEFAULT for USB_GRSTCTL */
#define USB_GRSTCTL_RXFFLSH_DEFAULT                (_USB_GRSTCTL_RXFFLSH_DEFAULT << 4)     /**< Shifted mode DEFAULT for USB_GRSTCTL */
#define USB_GRSTCTL_TXFFLSH                        (0x1UL << 5)                            /**< TxFIFO Flush */
#define _USB_GRSTCTL_TXFFLSH_SHIFT                 5                                       /**< Shift value for USB_TXFFLSH */
#define _USB_GRSTCTL_TXFFLSH_MASK                  0x20UL                                  /**< Bit mask for USB_TXFFLSH */
#define _USB_GRSTCTL_TXFFLSH_DEFAULT               0x00000000UL                            /**< Mode DEFAULT for USB_GRSTCTL */
#define USB_GRSTCTL_TXFFLSH_DEFAULT                (_USB_GRSTCTL_TXFFLSH_DEFAULT << 5)     /**< Shifted mode DEFAULT for USB_GRSTCTL */
#define _USB_GRSTCTL_TXFNUM_SHIFT                  6                                       /**< Shift value for USB_TXFNUM */
#define _USB_GRSTCTL_TXFNUM_MASK                   0x7C0UL                                 /**< Bit mask for USB_TXFNUM */
#define _USB_GRSTCTL_TXFNUM_DEFAULT                0x00000000UL                            /**< Mode DEFAULT for USB_GRSTCTL */
#define _USB_GRSTCTL_TXFNUM_F0                     0x00000000UL                            /**< Mode F0 for USB_GRSTCTL */
#define _USB_GRSTCTL_TXFNUM_F1                     0x00000001UL                            /**< Mode F1 for USB_GRSTCTL */
#define _USB_GRSTCTL_TXFNUM_F2                     0x00000002UL                            /**< Mode F2 for USB_GRSTCTL */
#define _USB_GRSTCTL_TXFNUM_F3                     0x00000003UL                            /**< Mode F3 for USB_GRSTCTL */
#define _USB_GRSTCTL_TXFNUM_F4                     0x00000004UL                            /**< Mode F4 for USB_GRSTCTL */
#define _USB_GRSTCTL_TXFNUM_F5                     0x00000005UL                            /**< Mode F5 for USB_GRSTCTL */
#define _USB_GRSTCTL_TXFNUM_F6                     0x00000006UL                            /**< Mode F6 for USB_GRSTCTL */
#define _USB_GRSTCTL_TXFNUM_FALL                   0x00000010UL                            /**< Mode FALL for USB_GRSTCTL */
#define USB_GRSTCTL_TXFNUM_DEFAULT                 (_USB_GRSTCTL_TXFNUM_DEFAULT << 6)      /**< Shifted mode DEFAULT for USB_GRSTCTL */
#define USB_GRSTCTL_TXFNUM_F0                      (_USB_GRSTCTL_TXFNUM_F0 << 6)           /**< Shifted mode F0 for USB_GRSTCTL */
#define USB_GRSTCTL_TXFNUM_F1                      (_USB_GRSTCTL_TXFNUM_F1 << 6)           /**< Shifted mode F1 for USB_GRSTCTL */
#define USB_GRSTCTL_TXFNUM_F2                      (_USB_GRSTCTL_TXFNUM_F2 << 6)           /**< Shifted mode F2 for USB_GRSTCTL */
#define USB_GRSTCTL_TXFNUM_F3                      (_USB_GRSTCTL_TXFNUM_F3 << 6)           /**< Shifted mode F3 for USB_GRSTCTL */
#define USB_GRSTCTL_TXFNUM_F4                      (_USB_GRSTCTL_TXFNUM_F4 << 6)           /**< Shifted mode F4 for USB_GRSTCTL */
#define USB_GRSTCTL_TXFNUM_F5                      (_USB_GRSTCTL_TXFNUM_F5 << 6)           /**< Shifted mode F5 for USB_GRSTCTL */
#define USB_GRSTCTL_TXFNUM_F6                      (_USB_GRSTCTL_TXFNUM_F6 << 6)           /**< Shifted mode F6 for USB_GRSTCTL */
#define USB_GRSTCTL_TXFNUM_FALL                    (_USB_GRSTCTL_TXFNUM_FALL << 6)         /**< Shifted mode FALL for USB_GRSTCTL */
#define USB_GRSTCTL_DMAREQ                         (0x1UL << 30)                           /**< DMA Request Signal */
#define _USB_GRSTCTL_DMAREQ_SHIFT                  30                                      /**< Shift value for USB_DMAREQ */
#define _USB_GRSTCTL_DMAREQ_MASK                   0x40000000UL                            /**< Bit mask for USB_DMAREQ */
#define _USB_GRSTCTL_DMAREQ_DEFAULT                0x00000000UL                            /**< Mode DEFAULT for USB_GRSTCTL */
#define USB_GRSTCTL_DMAREQ_DEFAULT                 (_USB_GRSTCTL_DMAREQ_DEFAULT << 30)     /**< Shifted mode DEFAULT for USB_GRSTCTL */
#define USB_GRSTCTL_AHBIDLE                        (0x1UL << 31)                           /**< AHB Master Idle */
#define _USB_GRSTCTL_AHBIDLE_SHIFT                 31                                      /**< Shift value for USB_AHBIDLE */
#define _USB_GRSTCTL_AHBIDLE_MASK                  0x80000000UL                            /**< Bit mask for USB_AHBIDLE */
#define _USB_GRSTCTL_AHBIDLE_DEFAULT               0x00000001UL                            /**< Mode DEFAULT for USB_GRSTCTL */
#define USB_GRSTCTL_AHBIDLE_DEFAULT                (_USB_GRSTCTL_AHBIDLE_DEFAULT << 31)    /**< Shifted mode DEFAULT for USB_GRSTCTL */

/* Bit fields for USB GINTSTS */
#define _USB_GINTSTS_RESETVALUE                    0x00000000UL                             /**< Default value for USB_GINTSTS */
#define _USB_GINTSTS_MASK                          0x80FCFCD9UL                             /**< Mask for USB_GINTSTS */
#define USB_GINTSTS_CURMOD                         (0x1UL << 0)                             /**< Current Mode of Operation */
#define _USB_GINTSTS_CURMOD_SHIFT                  0                                        /**< Shift value for USB_CURMOD */
#define _USB_GINTSTS_CURMOD_MASK                   0x1UL                                    /**< Bit mask for USB_CURMOD */
#define _USB_GINTSTS_CURMOD_DEFAULT                0x00000000UL                             /**< Mode DEFAULT for USB_GINTSTS */
#define _USB_GINTSTS_CURMOD_DEVICE                 0x00000000UL                             /**< Mode DEVICE for USB_GINTSTS */
#define USB_GINTSTS_CURMOD_DEFAULT                 (_USB_GINTSTS_CURMOD_DEFAULT << 0)       /**< Shifted mode DEFAULT for USB_GINTSTS */
#define USB_GINTSTS_CURMOD_DEVICE                  (_USB_GINTSTS_CURMOD_DEVICE << 0)        /**< Shifted mode DEVICE for USB_GINTSTS */
#define USB_GINTSTS_SOF                            (0x1UL << 3)                             /**< Start of Frame */
#define _USB_GINTSTS_SOF_SHIFT                     3                                        /**< Shift value for USB_SOF */
#define _USB_GINTSTS_SOF_MASK                      0x8UL                                    /**< Bit mask for USB_SOF */
#define _USB_GINTSTS_SOF_DEFAULT                   0x00000000UL                             /**< Mode DEFAULT for USB_GINTSTS */
#define USB_GINTSTS_SOF_DEFAULT                    (_USB_GINTSTS_SOF_DEFAULT << 3)          /**< Shifted mode DEFAULT for USB_GINTSTS */
#define USB_GINTSTS_RXFLVL                         (0x1UL << 4)                             /**< RxFIFO Non-Empty */
#define _USB_GINTSTS_RXFLVL_SHIFT                  4                                        /**< Shift value for USB_RXFLVL */
#define _USB_GINTSTS_RXFLVL_MASK                   0x10UL                                   /**< Bit mask for USB_RXFLVL */
#define _USB_GINTSTS_RXFLVL_DEFAULT                0x00000000UL                             /**< Mode DEFAULT for USB_GINTSTS */
#define USB_GINTSTS_RXFLVL_DEFAULT                 (_USB_GINTSTS_RXFLVL_DEFAULT << 4)       /**< Shifted mode DEFAULT for USB_GINTSTS */
#define USB_GINTSTS_GINNAKEFF                      (0x1UL << 6)                             /**< Global IN Non-periodic NAK Effective */
#define _USB_GINTSTS_GINNAKEFF_SHIFT               6                                        /**< Shift value for USB_GINNAKEFF */
#define _USB_GINTSTS_GINNAKEFF_MASK                0x40UL                                   /**< Bit mask for USB_GINNAKEFF */
#define _USB_GINTSTS_GINNAKEFF_DEFAULT             0x00000000UL                             /**< Mode DEFAULT for USB_GINTSTS */
#define USB_GINTSTS_GINNAKEFF_DEFAULT              (_USB_GINTSTS_GINNAKEFF_DEFAULT << 6)    /**< Shifted mode DEFAULT for USB_GINTSTS */
#define USB_GINTSTS_GOUTNAKEFF                     (0x1UL << 7)                             /**< Global OUT NAK Effective */
#define _USB_GINTSTS_GOUTNAKEFF_SHIFT              7                                        /**< Shift value for USB_GOUTNAKEFF */
#define _USB_GINTSTS_GOUTNAKEFF_MASK               0x80UL                                   /**< Bit mask for USB_GOUTNAKEFF */
#define _USB_GINTSTS_GOUTNAKEFF_DEFAULT            0x00000000UL                             /**< Mode DEFAULT for USB_GINTSTS */
#define USB_GINTSTS_GOUTNAKEFF_DEFAULT             (_USB_GINTSTS_GOUTNAKEFF_DEFAULT << 7)   /**< Shifted mode DEFAULT for USB_GINTSTS */
#define USB_GINTSTS_ERLYSUSP                       (0x1UL << 10)                            /**< Early Suspend */
#define _USB_GINTSTS_ERLYSUSP_SHIFT                10                                       /**< Shift value for USB_ERLYSUSP */
#define _USB_GINTSTS_ERLYSUSP_MASK                 0x400UL                                  /**< Bit mask for USB_ERLYSUSP */
#define _USB_GINTSTS_ERLYSUSP_DEFAULT              0x00000000UL                             /**< Mode DEFAULT for USB_GINTSTS */
#define USB_GINTSTS_ERLYSUSP_DEFAULT               (_USB_GINTSTS_ERLYSUSP_DEFAULT << 10)    /**< Shifted mode DEFAULT for USB_GINTSTS */
#define USB_GINTSTS_USBSUSP                        (0x1UL << 11)                            /**< USB Suspend */
#define _USB_GINTSTS_USBSUSP_SHIFT                 11                                       /**< Shift value for USB_USBSUSP */
#define _USB_GINTSTS_USBSUSP_MASK                  0x800UL                                  /**< Bit mask for USB_USBSUSP */
#define _USB_GINTSTS_USBSUSP_DEFAULT               0x00000000UL                             /**< Mode DEFAULT for USB_GINTSTS */
#define USB_GINTSTS_USBSUSP_DEFAULT                (_USB_GINTSTS_USBSUSP_DEFAULT << 11)     /**< Shifted mode DEFAULT for USB_GINTSTS */
#define USB_GINTSTS_USBRST                         (0x1UL << 12)                            /**< USB Reset */
#define _USB_GINTSTS_USBRST_SHIFT                  12                                       /**< Shift value for USB_USBRST */
#define _USB_GINTSTS_USBRST_MASK                   0x1000UL                                 /**< Bit mask for USB_USBRST */
#define _USB_GINTSTS_USBRST_DEFAULT                0x00000000UL                             /**< Mode DEFAULT for USB_GINTSTS */
#define USB_GINTSTS_USBRST_DEFAULT                 (_USB_GINTSTS_USBRST_DEFAULT << 12)      /**< Shifted mode DEFAULT for USB_GINTSTS */
#define USB_GINTSTS_ENUMDONE                       (0x1UL << 13)                            /**< Enumeration Done */
#define _USB_GINTSTS_ENUMDONE_SHIFT                13                                       /**< Shift value for USB_ENUMDONE */
#define _USB_GINTSTS_ENUMDONE_MASK                 0x2000UL                                 /**< Bit mask for USB_ENUMDONE */
#define _USB_GINTSTS_ENUMDONE_DEFAULT              0x00000000UL                             /**< Mode DEFAULT for USB_GINTSTS */
#define USB_GINTSTS_ENUMDONE_DEFAULT               (_USB_GINTSTS_ENUMDONE_DEFAULT << 13)    /**< Shifted mode DEFAULT for USB_GINTSTS */
#define USB_GINTSTS_ISOOUTDROP                     (0x1UL << 14)                            /**< Isochronous OUT Packet Dropped Interrupt */
#define _USB_GINTSTS_ISOOUTDROP_SHIFT              14                                       /**< Shift value for USB_ISOOUTDROP */
#define _USB_GINTSTS_ISOOUTDROP_MASK               0x4000UL                                 /**< Bit mask for USB_ISOOUTDROP */
#define _USB_GINTSTS_ISOOUTDROP_DEFAULT            0x00000000UL                             /**< Mode DEFAULT for USB_GINTSTS */
#define USB_GINTSTS_ISOOUTDROP_DEFAULT             (_USB_GINTSTS_ISOOUTDROP_DEFAULT << 14)  /**< Shifted mode DEFAULT for USB_GINTSTS */
#define USB_GINTSTS_EOPF                           (0x1UL << 15)                            /**< End of Periodic Frame Interrupt */
#define _USB_GINTSTS_EOPF_SHIFT                    15                                       /**< Shift value for USB_EOPF */
#define _USB_GINTSTS_EOPF_MASK                     0x8000UL                                 /**< Bit mask for USB_EOPF */
#define _USB_GINTSTS_EOPF_DEFAULT                  0x00000000UL                             /**< Mode DEFAULT for USB_GINTSTS */
#define USB_GINTSTS_EOPF_DEFAULT                   (_USB_GINTSTS_EOPF_DEFAULT << 15)        /**< Shifted mode DEFAULT for USB_GINTSTS */
#define USB_GINTSTS_IEPINT                         (0x1UL << 18)                            /**< IN Endpoints Interrupt */
#define _USB_GINTSTS_IEPINT_SHIFT                  18                                       /**< Shift value for USB_IEPINT */
#define _USB_GINTSTS_IEPINT_MASK                   0x40000UL                                /**< Bit mask for USB_IEPINT */
#define _USB_GINTSTS_IEPINT_DEFAULT                0x00000000UL                             /**< Mode DEFAULT for USB_GINTSTS */
#define USB_GINTSTS_IEPINT_DEFAULT                 (_USB_GINTSTS_IEPINT_DEFAULT << 18)      /**< Shifted mode DEFAULT for USB_GINTSTS */
#define USB_GINTSTS_OEPINT                         (0x1UL << 19)                            /**< OUT Endpoints Interrupt */
#define _USB_GINTSTS_OEPINT_SHIFT                  19                                       /**< Shift value for USB_OEPINT */
#define _USB_GINTSTS_OEPINT_MASK                   0x80000UL                                /**< Bit mask for USB_OEPINT */
#define _USB_GINTSTS_OEPINT_DEFAULT                0x00000000UL                             /**< Mode DEFAULT for USB_GINTSTS */
#define USB_GINTSTS_OEPINT_DEFAULT                 (_USB_GINTSTS_OEPINT_DEFAULT << 19)      /**< Shifted mode DEFAULT for USB_GINTSTS */
#define USB_GINTSTS_INCOMPISOIN                    (0x1UL << 20)                            /**< Incomplete Isochronous IN Transfer */
#define _USB_GINTSTS_INCOMPISOIN_SHIFT             20                                       /**< Shift value for USB_INCOMPISOIN */
#define _USB_GINTSTS_INCOMPISOIN_MASK              0x100000UL                               /**< Bit mask for USB_INCOMPISOIN */
#define _USB_GINTSTS_INCOMPISOIN_DEFAULT           0x00000000UL                             /**< Mode DEFAULT for USB_GINTSTS */
#define USB_GINTSTS_INCOMPISOIN_DEFAULT            (_USB_GINTSTS_INCOMPISOIN_DEFAULT << 20) /**< Shifted mode DEFAULT for USB_GINTSTS */
#define USB_GINTSTS_INCOMPLP                       (0x1UL << 21)                            /**< Incomplete Periodic Transfer */
#define _USB_GINTSTS_INCOMPLP_SHIFT                21                                       /**< Shift value for USB_INCOMPLP */
#define _USB_GINTSTS_INCOMPLP_MASK                 0x200000UL                               /**< Bit mask for USB_INCOMPLP */
#define _USB_GINTSTS_INCOMPLP_DEFAULT              0x00000000UL                             /**< Mode DEFAULT for USB_GINTSTS */
#define USB_GINTSTS_INCOMPLP_DEFAULT               (_USB_GINTSTS_INCOMPLP_DEFAULT << 21)    /**< Shifted mode DEFAULT for USB_GINTSTS */
#define USB_GINTSTS_FETSUSP                        (0x1UL << 22)                            /**< Data Fetch Suspended */
#define _USB_GINTSTS_FETSUSP_SHIFT                 22                                       /**< Shift value for USB_FETSUSP */
#define _USB_GINTSTS_FETSUSP_MASK                  0x400000UL                               /**< Bit mask for USB_FETSUSP */
#define _USB_GINTSTS_FETSUSP_DEFAULT               0x00000000UL                             /**< Mode DEFAULT for USB_GINTSTS */
#define USB_GINTSTS_FETSUSP_DEFAULT                (_USB_GINTSTS_FETSUSP_DEFAULT << 22)     /**< Shifted mode DEFAULT for USB_GINTSTS */
#define USB_GINTSTS_RESETDET                       (0x1UL << 23)                            /**< Reset detected Interrupt */
#define _USB_GINTSTS_RESETDET_SHIFT                23                                       /**< Shift value for USB_RESETDET */
#define _USB_GINTSTS_RESETDET_MASK                 0x800000UL                               /**< Bit mask for USB_RESETDET */
#define _USB_GINTSTS_RESETDET_DEFAULT              0x00000000UL                             /**< Mode DEFAULT for USB_GINTSTS */
#define USB_GINTSTS_RESETDET_DEFAULT               (_USB_GINTSTS_RESETDET_DEFAULT << 23)    /**< Shifted mode DEFAULT for USB_GINTSTS */
#define USB_GINTSTS_WKUPINT                        (0x1UL << 31)                            /**< Resume/Remote Wakeup Detected Interrupt */
#define _USB_GINTSTS_WKUPINT_SHIFT                 31                                       /**< Shift value for USB_WKUPINT */
#define _USB_GINTSTS_WKUPINT_MASK                  0x80000000UL                             /**< Bit mask for USB_WKUPINT */
#define _USB_GINTSTS_WKUPINT_DEFAULT               0x00000000UL                             /**< Mode DEFAULT for USB_GINTSTS */
#define USB_GINTSTS_WKUPINT_DEFAULT                (_USB_GINTSTS_WKUPINT_DEFAULT << 31)     /**< Shifted mode DEFAULT for USB_GINTSTS */

/* Bit fields for USB GINTMSK */
#define _USB_GINTMSK_RESETVALUE                    0x00000000UL                                /**< Default value for USB_GINTMSK */
#define _USB_GINTMSK_MASK                          0x80FCFCDAUL                                /**< Mask for USB_GINTMSK */
#define USB_GINTMSK_MODEMISMSK                     (0x1UL << 1)                                /**< Mode Mismatch Interrupt Mask */
#define _USB_GINTMSK_MODEMISMSK_SHIFT              1                                           /**< Shift value for USB_MODEMISMSK */
#define _USB_GINTMSK_MODEMISMSK_MASK               0x2UL                                       /**< Bit mask for USB_MODEMISMSK */
#define _USB_GINTMSK_MODEMISMSK_DEFAULT            0x00000000UL                                /**< Mode DEFAULT for USB_GINTMSK */
#define USB_GINTMSK_MODEMISMSK_DEFAULT             (_USB_GINTMSK_MODEMISMSK_DEFAULT << 1)      /**< Shifted mode DEFAULT for USB_GINTMSK */
#define USB_GINTMSK_SOFMSK                         (0x1UL << 3)                                /**< Start of Frame Mask */
#define _USB_GINTMSK_SOFMSK_SHIFT                  3                                           /**< Shift value for USB_SOFMSK */
#define _USB_GINTMSK_SOFMSK_MASK                   0x8UL                                       /**< Bit mask for USB_SOFMSK */
#define _USB_GINTMSK_SOFMSK_DEFAULT                0x00000000UL                                /**< Mode DEFAULT for USB_GINTMSK */
#define USB_GINTMSK_SOFMSK_DEFAULT                 (_USB_GINTMSK_SOFMSK_DEFAULT << 3)          /**< Shifted mode DEFAULT for USB_GINTMSK */
#define USB_GINTMSK_RXFLVLMSK                      (0x1UL << 4)                                /**< Receive FIFO Non-Empty Mask */
#define _USB_GINTMSK_RXFLVLMSK_SHIFT               4                                           /**< Shift value for USB_RXFLVLMSK */
#define _USB_GINTMSK_RXFLVLMSK_MASK                0x10UL                                      /**< Bit mask for USB_RXFLVLMSK */
#define _USB_GINTMSK_RXFLVLMSK_DEFAULT             0x00000000UL                                /**< Mode DEFAULT for USB_GINTMSK */
#define USB_GINTMSK_RXFLVLMSK_DEFAULT              (_USB_GINTMSK_RXFLVLMSK_DEFAULT << 4)       /**< Shifted mode DEFAULT for USB_GINTMSK */
#define USB_GINTMSK_GINNAKEFFMSK                   (0x1UL << 6)                                /**< Global Non-periodic IN NAK Effective Mask */
#define _USB_GINTMSK_GINNAKEFFMSK_SHIFT            6                                           /**< Shift value for USB_GINNAKEFFMSK */
#define _USB_GINTMSK_GINNAKEFFMSK_MASK             0x40UL                                      /**< Bit mask for USB_GINNAKEFFMSK */
#define _USB_GINTMSK_GINNAKEFFMSK_DEFAULT          0x00000000UL                                /**< Mode DEFAULT for USB_GINTMSK */
#define USB_GINTMSK_GINNAKEFFMSK_DEFAULT           (_USB_GINTMSK_GINNAKEFFMSK_DEFAULT << 6)    /**< Shifted mode DEFAULT for USB_GINTMSK */
#define USB_GINTMSK_GOUTNAKEFFMSK                  (0x1UL << 7)                                /**< Global OUT NAK Effective Mask */
#define _USB_GINTMSK_GOUTNAKEFFMSK_SHIFT           7                                           /**< Shift value for USB_GOUTNAKEFFMSK */
#define _USB_GINTMSK_GOUTNAKEFFMSK_MASK            0x80UL                                      /**< Bit mask for USB_GOUTNAKEFFMSK */
#define _USB_GINTMSK_GOUTNAKEFFMSK_DEFAULT         0x00000000UL                                /**< Mode DEFAULT for USB_GINTMSK */
#define USB_GINTMSK_GOUTNAKEFFMSK_DEFAULT          (_USB_GINTMSK_GOUTNAKEFFMSK_DEFAULT << 7)   /**< Shifted mode DEFAULT for USB_GINTMSK */
#define USB_GINTMSK_ERLYSUSPMSK                    (0x1UL << 10)                               /**< Early Suspend Mask */
#define _USB_GINTMSK_ERLYSUSPMSK_SHIFT             10                                          /**< Shift value for USB_ERLYSUSPMSK */
#define _USB_GINTMSK_ERLYSUSPMSK_MASK              0x400UL                                     /**< Bit mask for USB_ERLYSUSPMSK */
#define _USB_GINTMSK_ERLYSUSPMSK_DEFAULT           0x00000000UL                                /**< Mode DEFAULT for USB_GINTMSK */
#define USB_GINTMSK_ERLYSUSPMSK_DEFAULT            (_USB_GINTMSK_ERLYSUSPMSK_DEFAULT << 10)    /**< Shifted mode DEFAULT for USB_GINTMSK */
#define USB_GINTMSK_USBSUSPMSK                     (0x1UL << 11)                               /**< USB Suspend Mask */
#define _USB_GINTMSK_USBSUSPMSK_SHIFT              11                                          /**< Shift value for USB_USBSUSPMSK */
#define _USB_GINTMSK_USBSUSPMSK_MASK               0x800UL                                     /**< Bit mask for USB_USBSUSPMSK */
#define _USB_GINTMSK_USBSUSPMSK_DEFAULT            0x00000000UL                                /**< Mode DEFAULT for USB_GINTMSK */
#define USB_GINTMSK_USBSUSPMSK_DEFAULT             (_USB_GINTMSK_USBSUSPMSK_DEFAULT << 11)     /**< Shifted mode DEFAULT for USB_GINTMSK */
#define USB_GINTMSK_USBRSTMSK                      (0x1UL << 12)                               /**< USB Reset Mask */
#define _USB_GINTMSK_USBRSTMSK_SHIFT               12                                          /**< Shift value for USB_USBRSTMSK */
#define _USB_GINTMSK_USBRSTMSK_MASK                0x1000UL                                    /**< Bit mask for USB_USBRSTMSK */
#define _USB_GINTMSK_USBRSTMSK_DEFAULT             0x00000000UL                                /**< Mode DEFAULT for USB_GINTMSK */
#define USB_GINTMSK_USBRSTMSK_DEFAULT              (_USB_GINTMSK_USBRSTMSK_DEFAULT << 12)      /**< Shifted mode DEFAULT for USB_GINTMSK */
#define USB_GINTMSK_ENUMDONEMSK                    (0x1UL << 13)                               /**< Enumeration Done Mask */
#define _USB_GINTMSK_ENUMDONEMSK_SHIFT             13                                          /**< Shift value for USB_ENUMDONEMSK */
#define _USB_GINTMSK_ENUMDONEMSK_MASK              0x2000UL                                    /**< Bit mask for USB_ENUMDONEMSK */
#define _USB_GINTMSK_ENUMDONEMSK_DEFAULT           0x00000000UL                                /**< Mode DEFAULT for USB_GINTMSK */
#define USB_GINTMSK_ENUMDONEMSK_DEFAULT            (_USB_GINTMSK_ENUMDONEMSK_DEFAULT << 13)    /**< Shifted mode DEFAULT for USB_GINTMSK */
#define USB_GINTMSK_ISOOUTDROPMSK                  (0x1UL << 14)                               /**< Isochronous OUT Packet Dropped Interrupt Mask */
#define _USB_GINTMSK_ISOOUTDROPMSK_SHIFT           14                                          /**< Shift value for USB_ISOOUTDROPMSK */
#define _USB_GINTMSK_ISOOUTDROPMSK_MASK            0x4000UL                                    /**< Bit mask for USB_ISOOUTDROPMSK */
#define _USB_GINTMSK_ISOOUTDROPMSK_DEFAULT         0x00000000UL                                /**< Mode DEFAULT for USB_GINTMSK */
#define USB_GINTMSK_ISOOUTDROPMSK_DEFAULT          (_USB_GINTMSK_ISOOUTDROPMSK_DEFAULT << 14)  /**< Shifted mode DEFAULT for USB_GINTMSK */
#define USB_GINTMSK_EOPFMSK                        (0x1UL << 15)                               /**< End of Periodic Frame Interrupt Mask */
#define _USB_GINTMSK_EOPFMSK_SHIFT                 15                                          /**< Shift value for USB_EOPFMSK */
#define _USB_GINTMSK_EOPFMSK_MASK                  0x8000UL                                    /**< Bit mask for USB_EOPFMSK */
#define _USB_GINTMSK_EOPFMSK_DEFAULT               0x00000000UL                                /**< Mode DEFAULT for USB_GINTMSK */
#define USB_GINTMSK_EOPFMSK_DEFAULT                (_USB_GINTMSK_EOPFMSK_DEFAULT << 15)        /**< Shifted mode DEFAULT for USB_GINTMSK */
#define USB_GINTMSK_IEPINTMSK                      (0x1UL << 18)                               /**< IN Endpoints Interrupt Mask */
#define _USB_GINTMSK_IEPINTMSK_SHIFT               18                                          /**< Shift value for USB_IEPINTMSK */
#define _USB_GINTMSK_IEPINTMSK_MASK                0x40000UL                                   /**< Bit mask for USB_IEPINTMSK */
#define _USB_GINTMSK_IEPINTMSK_DEFAULT             0x00000000UL                                /**< Mode DEFAULT for USB_GINTMSK */
#define USB_GINTMSK_IEPINTMSK_DEFAULT              (_USB_GINTMSK_IEPINTMSK_DEFAULT << 18)      /**< Shifted mode DEFAULT for USB_GINTMSK */
#define USB_GINTMSK_OEPINTMSK                      (0x1UL << 19)                               /**< OUT Endpoints Interrupt Mask */
#define _USB_GINTMSK_OEPINTMSK_SHIFT               19                                          /**< Shift value for USB_OEPINTMSK */
#define _USB_GINTMSK_OEPINTMSK_MASK                0x80000UL                                   /**< Bit mask for USB_OEPINTMSK */
#define _USB_GINTMSK_OEPINTMSK_DEFAULT             0x00000000UL                                /**< Mode DEFAULT for USB_GINTMSK */
#define USB_GINTMSK_OEPINTMSK_DEFAULT              (_USB_GINTMSK_OEPINTMSK_DEFAULT << 19)      /**< Shifted mode DEFAULT for USB_GINTMSK */
#define USB_GINTMSK_INCOMPISOINMSK                 (0x1UL << 20)                               /**< Incomplete Isochronous IN Transfer Mask */
#define _USB_GINTMSK_INCOMPISOINMSK_SHIFT          20                                          /**< Shift value for USB_INCOMPISOINMSK */
#define _USB_GINTMSK_INCOMPISOINMSK_MASK           0x100000UL                                  /**< Bit mask for USB_INCOMPISOINMSK */
#define _USB_GINTMSK_INCOMPISOINMSK_DEFAULT        0x00000000UL                                /**< Mode DEFAULT for USB_GINTMSK */
#define USB_GINTMSK_INCOMPISOINMSK_DEFAULT         (_USB_GINTMSK_INCOMPISOINMSK_DEFAULT << 20) /**< Shifted mode DEFAULT for USB_GINTMSK */
#define USB_GINTMSK_INCOMPLPMSK                    (0x1UL << 21)                               /**< Incomplete Periodic Transfer Mask */
#define _USB_GINTMSK_INCOMPLPMSK_SHIFT             21                                          /**< Shift value for USB_INCOMPLPMSK */
#define _USB_GINTMSK_INCOMPLPMSK_MASK              0x200000UL                                  /**< Bit mask for USB_INCOMPLPMSK */
#define _USB_GINTMSK_INCOMPLPMSK_DEFAULT           0x00000000UL                                /**< Mode DEFAULT for USB_GINTMSK */
#define USB_GINTMSK_INCOMPLPMSK_DEFAULT            (_USB_GINTMSK_INCOMPLPMSK_DEFAULT << 21)    /**< Shifted mode DEFAULT for USB_GINTMSK */
#define USB_GINTMSK_FETSUSPMSK                     (0x1UL << 22)                               /**< Data Fetch Suspended Mask */
#define _USB_GINTMSK_FETSUSPMSK_SHIFT              22                                          /**< Shift value for USB_FETSUSPMSK */
#define _USB_GINTMSK_FETSUSPMSK_MASK               0x400000UL                                  /**< Bit mask for USB_FETSUSPMSK */
#define _USB_GINTMSK_FETSUSPMSK_DEFAULT            0x00000000UL                                /**< Mode DEFAULT for USB_GINTMSK */
#define USB_GINTMSK_FETSUSPMSK_DEFAULT             (_USB_GINTMSK_FETSUSPMSK_DEFAULT << 22)     /**< Shifted mode DEFAULT for USB_GINTMSK */
#define USB_GINTMSK_RESETDETMSK                    (0x1UL << 23)                               /**< Reset detected Interrupt Mask */
#define _USB_GINTMSK_RESETDETMSK_SHIFT             23                                          /**< Shift value for USB_RESETDETMSK */
#define _USB_GINTMSK_RESETDETMSK_MASK              0x800000UL                                  /**< Bit mask for USB_RESETDETMSK */
#define _USB_GINTMSK_RESETDETMSK_DEFAULT           0x00000000UL                                /**< Mode DEFAULT for USB_GINTMSK */
#define USB_GINTMSK_RESETDETMSK_DEFAULT            (_USB_GINTMSK_RESETDETMSK_DEFAULT << 23)    /**< Shifted mode DEFAULT for USB_GINTMSK */
#define USB_GINTMSK_WKUPINTMSK                     (0x1UL << 31)                               /**< Resume/Remote Wakeup Detected Interrupt Mask */
#define _USB_GINTMSK_WKUPINTMSK_SHIFT              31                                          /**< Shift value for USB_WKUPINTMSK */
#define _USB_GINTMSK_WKUPINTMSK_MASK               0x80000000UL                                /**< Bit mask for USB_WKUPINTMSK */
#define _USB_GINTMSK_WKUPINTMSK_DEFAULT            0x00000000UL                                /**< Mode DEFAULT for USB_GINTMSK */
#define USB_GINTMSK_WKUPINTMSK_DEFAULT             (_USB_GINTMSK_WKUPINTMSK_DEFAULT << 31)     /**< Shifted mode DEFAULT for USB_GINTMSK */

/* Bit fields for USB GRXSTSR */
#define _USB_GRXSTSR_RESETVALUE                    0x00000000UL                           /**< Default value for USB_GRXSTSR */
#define _USB_GRXSTSR_MASK                          0x01FFFFFFUL                           /**< Mask for USB_GRXSTSR */
#define _USB_GRXSTSR_CHEPNUM_SHIFT                 0                                      /**< Shift value for USB_CHEPNUM */
#define _USB_GRXSTSR_CHEPNUM_MASK                  0xFUL                                  /**< Bit mask for USB_CHEPNUM */
#define _USB_GRXSTSR_CHEPNUM_DEFAULT               0x00000000UL                           /**< Mode DEFAULT for USB_GRXSTSR */
#define USB_GRXSTSR_CHEPNUM_DEFAULT                (_USB_GRXSTSR_CHEPNUM_DEFAULT << 0)    /**< Shifted mode DEFAULT for USB_GRXSTSR */
#define _USB_GRXSTSR_BCNT_SHIFT                    4                                      /**< Shift value for USB_BCNT */
#define _USB_GRXSTSR_BCNT_MASK                     0x7FF0UL                               /**< Bit mask for USB_BCNT */
#define _USB_GRXSTSR_BCNT_DEFAULT                  0x00000000UL                           /**< Mode DEFAULT for USB_GRXSTSR */
#define USB_GRXSTSR_BCNT_DEFAULT                   (_USB_GRXSTSR_BCNT_DEFAULT << 4)       /**< Shifted mode DEFAULT for USB_GRXSTSR */
#define _USB_GRXSTSR_DPID_SHIFT                    15                                     /**< Shift value for USB_DPID */
#define _USB_GRXSTSR_DPID_MASK                     0x18000UL                              /**< Bit mask for USB_DPID */
#define _USB_GRXSTSR_DPID_DEFAULT                  0x00000000UL                           /**< Mode DEFAULT for USB_GRXSTSR */
#define _USB_GRXSTSR_DPID_DATA0                    0x00000000UL                           /**< Mode DATA0 for USB_GRXSTSR */
#define _USB_GRXSTSR_DPID_DATA1                    0x00000001UL                           /**< Mode DATA1 for USB_GRXSTSR */
#define _USB_GRXSTSR_DPID_DATA2                    0x00000002UL                           /**< Mode DATA2 for USB_GRXSTSR */
#define _USB_GRXSTSR_DPID_MDATA                    0x00000003UL                           /**< Mode MDATA for USB_GRXSTSR */
#define USB_GRXSTSR_DPID_DEFAULT                   (_USB_GRXSTSR_DPID_DEFAULT << 15)      /**< Shifted mode DEFAULT for USB_GRXSTSR */
#define USB_GRXSTSR_DPID_DATA0                     (_USB_GRXSTSR_DPID_DATA0 << 15)        /**< Shifted mode DATA0 for USB_GRXSTSR */
#define USB_GRXSTSR_DPID_DATA1                     (_USB_GRXSTSR_DPID_DATA1 << 15)        /**< Shifted mode DATA1 for USB_GRXSTSR */
#define USB_GRXSTSR_DPID_DATA2                     (_USB_GRXSTSR_DPID_DATA2 << 15)        /**< Shifted mode DATA2 for USB_GRXSTSR */
#define USB_GRXSTSR_DPID_MDATA                     (_USB_GRXSTSR_DPID_MDATA << 15)        /**< Shifted mode MDATA for USB_GRXSTSR */
#define _USB_GRXSTSR_PKTSTS_SHIFT                  17                                     /**< Shift value for USB_PKTSTS */
#define _USB_GRXSTSR_PKTSTS_MASK                   0x1E0000UL                             /**< Bit mask for USB_PKTSTS */
#define _USB_GRXSTSR_PKTSTS_DEFAULT                0x00000000UL                           /**< Mode DEFAULT for USB_GRXSTSR */
#define _USB_GRXSTSR_PKTSTS_GOUTNAK                0x00000001UL                           /**< Mode GOUTNAK for USB_GRXSTSR */
#define _USB_GRXSTSR_PKTSTS_PKTRCV                 0x00000002UL                           /**< Mode PKTRCV for USB_GRXSTSR */
#define _USB_GRXSTSR_PKTSTS_XFERCOMPL              0x00000003UL                           /**< Mode XFERCOMPL for USB_GRXSTSR */
#define _USB_GRXSTSR_PKTSTS_SETUPCOMPL             0x00000004UL                           /**< Mode SETUPCOMPL for USB_GRXSTSR */
#define _USB_GRXSTSR_PKTSTS_TGLERR                 0x00000005UL                           /**< Mode TGLERR for USB_GRXSTSR */
#define _USB_GRXSTSR_PKTSTS_SETUPRCV               0x00000006UL                           /**< Mode SETUPRCV for USB_GRXSTSR */
#define _USB_GRXSTSR_PKTSTS_CHLT                   0x00000007UL                           /**< Mode CHLT for USB_GRXSTSR */
#define USB_GRXSTSR_PKTSTS_DEFAULT                 (_USB_GRXSTSR_PKTSTS_DEFAULT << 17)    /**< Shifted mode DEFAULT for USB_GRXSTSR */
#define USB_GRXSTSR_PKTSTS_GOUTNAK                 (_USB_GRXSTSR_PKTSTS_GOUTNAK << 17)    /**< Shifted mode GOUTNAK for USB_GRXSTSR */
#define USB_GRXSTSR_PKTSTS_PKTRCV                  (_USB_GRXSTSR_PKTSTS_PKTRCV << 17)     /**< Shifted mode PKTRCV for USB_GRXSTSR */
#define USB_GRXSTSR_PKTSTS_XFERCOMPL               (_USB_GRXSTSR_PKTSTS_XFERCOMPL << 17)  /**< Shifted mode XFERCOMPL for USB_GRXSTSR */
#define USB_GRXSTSR_PKTSTS_SETUPCOMPL              (_USB_GRXSTSR_PKTSTS_SETUPCOMPL << 17) /**< Shifted mode SETUPCOMPL for USB_GRXSTSR */
#define USB_GRXSTSR_PKTSTS_TGLERR                  (_USB_GRXSTSR_PKTSTS_TGLERR << 17)     /**< Shifted mode TGLERR for USB_GRXSTSR */
#define USB_GRXSTSR_PKTSTS_SETUPRCV                (_USB_GRXSTSR_PKTSTS_SETUPRCV << 17)   /**< Shifted mode SETUPRCV for USB_GRXSTSR */
#define USB_GRXSTSR_PKTSTS_CHLT                    (_USB_GRXSTSR_PKTSTS_CHLT << 17)       /**< Shifted mode CHLT for USB_GRXSTSR */
#define _USB_GRXSTSR_FN_SHIFT                      21                                     /**< Shift value for USB_FN */
#define _USB_GRXSTSR_FN_MASK                       0x1E00000UL                            /**< Bit mask for USB_FN */
#define _USB_GRXSTSR_FN_DEFAULT                    0x00000000UL                           /**< Mode DEFAULT for USB_GRXSTSR */
#define USB_GRXSTSR_FN_DEFAULT                     (_USB_GRXSTSR_FN_DEFAULT << 21)        /**< Shifted mode DEFAULT for USB_GRXSTSR */

/* Bit fields for USB GRXSTSP */
#define _USB_GRXSTSP_RESETVALUE                    0x00000000UL                           /**< Default value for USB_GRXSTSP */
#define _USB_GRXSTSP_MASK                          0x01FFFFFFUL                           /**< Mask for USB_GRXSTSP */
#define _USB_GRXSTSP_CHEPNUM_SHIFT                 0                                      /**< Shift value for USB_CHEPNUM */
#define _USB_GRXSTSP_CHEPNUM_MASK                  0xFUL                                  /**< Bit mask for USB_CHEPNUM */
#define _USB_GRXSTSP_CHEPNUM_DEFAULT               0x00000000UL                           /**< Mode DEFAULT for USB_GRXSTSP */
#define USB_GRXSTSP_CHEPNUM_DEFAULT                (_USB_GRXSTSP_CHEPNUM_DEFAULT << 0)    /**< Shifted mode DEFAULT for USB_GRXSTSP */
#define _USB_GRXSTSP_BCNT_SHIFT                    4                                      /**< Shift value for USB_BCNT */
#define _USB_GRXSTSP_BCNT_MASK                     0x7FF0UL                               /**< Bit mask for USB_BCNT */
#define _USB_GRXSTSP_BCNT_DEFAULT                  0x00000000UL                           /**< Mode DEFAULT for USB_GRXSTSP */
#define USB_GRXSTSP_BCNT_DEFAULT                   (_USB_GRXSTSP_BCNT_DEFAULT << 4)       /**< Shifted mode DEFAULT for USB_GRXSTSP */
#define _USB_GRXSTSP_DPID_SHIFT                    15                                     /**< Shift value for USB_DPID */
#define _USB_GRXSTSP_DPID_MASK                     0x18000UL                              /**< Bit mask for USB_DPID */
#define _USB_GRXSTSP_DPID_DEFAULT                  0x00000000UL                           /**< Mode DEFAULT for USB_GRXSTSP */
#define _USB_GRXSTSP_DPID_DATA0                    0x00000000UL                           /**< Mode DATA0 for USB_GRXSTSP */
#define _USB_GRXSTSP_DPID_DATA1                    0x00000001UL                           /**< Mode DATA1 for USB_GRXSTSP */
#define _USB_GRXSTSP_DPID_DATA2                    0x00000002UL                           /**< Mode DATA2 for USB_GRXSTSP */
#define _USB_GRXSTSP_DPID_MDATA                    0x00000003UL                           /**< Mode MDATA for USB_GRXSTSP */
#define USB_GRXSTSP_DPID_DEFAULT                   (_USB_GRXSTSP_DPID_DEFAULT << 15)      /**< Shifted mode DEFAULT for USB_GRXSTSP */
#define USB_GRXSTSP_DPID_DATA0                     (_USB_GRXSTSP_DPID_DATA0 << 15)        /**< Shifted mode DATA0 for USB_GRXSTSP */
#define USB_GRXSTSP_DPID_DATA1                     (_USB_GRXSTSP_DPID_DATA1 << 15)        /**< Shifted mode DATA1 for USB_GRXSTSP */
#define USB_GRXSTSP_DPID_DATA2                     (_USB_GRXSTSP_DPID_DATA2 << 15)        /**< Shifted mode DATA2 for USB_GRXSTSP */
#define USB_GRXSTSP_DPID_MDATA                     (_USB_GRXSTSP_DPID_MDATA << 15)        /**< Shifted mode MDATA for USB_GRXSTSP */
#define _USB_GRXSTSP_PKTSTS_SHIFT                  17                                     /**< Shift value for USB_PKTSTS */
#define _USB_GRXSTSP_PKTSTS_MASK                   0x1E0000UL                             /**< Bit mask for USB_PKTSTS */
#define _USB_GRXSTSP_PKTSTS_DEFAULT                0x00000000UL                           /**< Mode DEFAULT for USB_GRXSTSP */
#define _USB_GRXSTSP_PKTSTS_GOUTNAK                0x00000001UL                           /**< Mode GOUTNAK for USB_GRXSTSP */
#define _USB_GRXSTSP_PKTSTS_PKTRCV                 0x00000002UL                           /**< Mode PKTRCV for USB_GRXSTSP */
#define _USB_GRXSTSP_PKTSTS_XFERCOMPL              0x00000003UL                           /**< Mode XFERCOMPL for USB_GRXSTSP */
#define _USB_GRXSTSP_PKTSTS_SETUPCOMPL             0x00000004UL                           /**< Mode SETUPCOMPL for USB_GRXSTSP */
#define _USB_GRXSTSP_PKTSTS_TGLERR                 0x00000005UL                           /**< Mode TGLERR for USB_GRXSTSP */
#define _USB_GRXSTSP_PKTSTS_SETUPRCV               0x00000006UL                           /**< Mode SETUPRCV for USB_GRXSTSP */
#define _USB_GRXSTSP_PKTSTS_CHLT                   0x00000007UL                           /**< Mode CHLT for USB_GRXSTSP */
#define USB_GRXSTSP_PKTSTS_DEFAULT                 (_USB_GRXSTSP_PKTSTS_DEFAULT << 17)    /**< Shifted mode DEFAULT for USB_GRXSTSP */
#define USB_GRXSTSP_PKTSTS_GOUTNAK                 (_USB_GRXSTSP_PKTSTS_GOUTNAK << 17)    /**< Shifted mode GOUTNAK for USB_GRXSTSP */
#define USB_GRXSTSP_PKTSTS_PKTRCV                  (_USB_GRXSTSP_PKTSTS_PKTRCV << 17)     /**< Shifted mode PKTRCV for USB_GRXSTSP */
#define USB_GRXSTSP_PKTSTS_XFERCOMPL               (_USB_GRXSTSP_PKTSTS_XFERCOMPL << 17)  /**< Shifted mode XFERCOMPL for USB_GRXSTSP */
#define USB_GRXSTSP_PKTSTS_SETUPCOMPL              (_USB_GRXSTSP_PKTSTS_SETUPCOMPL << 17) /**< Shifted mode SETUPCOMPL for USB_GRXSTSP */
#define USB_GRXSTSP_PKTSTS_TGLERR                  (_USB_GRXSTSP_PKTSTS_TGLERR << 17)     /**< Shifted mode TGLERR for USB_GRXSTSP */
#define USB_GRXSTSP_PKTSTS_SETUPRCV                (_USB_GRXSTSP_PKTSTS_SETUPRCV << 17)   /**< Shifted mode SETUPRCV for USB_GRXSTSP */
#define USB_GRXSTSP_PKTSTS_CHLT                    (_USB_GRXSTSP_PKTSTS_CHLT << 17)       /**< Shifted mode CHLT for USB_GRXSTSP */
#define _USB_GRXSTSP_FN_SHIFT                      21                                     /**< Shift value for USB_FN */
#define _USB_GRXSTSP_FN_MASK                       0x1E00000UL                            /**< Bit mask for USB_FN */
#define _USB_GRXSTSP_FN_DEFAULT                    0x00000000UL                           /**< Mode DEFAULT for USB_GRXSTSP */
#define USB_GRXSTSP_FN_DEFAULT                     (_USB_GRXSTSP_FN_DEFAULT << 21)        /**< Shifted mode DEFAULT for USB_GRXSTSP */

/* Bit fields for USB GRXFSIZ */
#define _USB_GRXFSIZ_RESETVALUE                    0x00000200UL                       /**< Default value for USB_GRXFSIZ */
#define _USB_GRXFSIZ_MASK                          0x000003FFUL                       /**< Mask for USB_GRXFSIZ */
#define _USB_GRXFSIZ_RXFDEP_SHIFT                  0                                  /**< Shift value for USB_RXFDEP */
#define _USB_GRXFSIZ_RXFDEP_MASK                   0x3FFUL                            /**< Bit mask for USB_RXFDEP */
#define _USB_GRXFSIZ_RXFDEP_DEFAULT                0x00000200UL                       /**< Mode DEFAULT for USB_GRXFSIZ */
#define USB_GRXFSIZ_RXFDEP_DEFAULT                 (_USB_GRXFSIZ_RXFDEP_DEFAULT << 0) /**< Shifted mode DEFAULT for USB_GRXFSIZ */

/* Bit fields for USB GNPTXFSIZ */
#define _USB_GNPTXFSIZ_RESETVALUE                  0x02000200UL                                    /**< Default value for USB_GNPTXFSIZ */
#define _USB_GNPTXFSIZ_MASK                        0xFFFF03FFUL                                    /**< Mask for USB_GNPTXFSIZ */
#define _USB_GNPTXFSIZ_NPTXFSTADDR_SHIFT           0                                               /**< Shift value for USB_NPTXFSTADDR */
#define _USB_GNPTXFSIZ_NPTXFSTADDR_MASK            0x3FFUL                                         /**< Bit mask for USB_NPTXFSTADDR */
#define _USB_GNPTXFSIZ_NPTXFSTADDR_DEFAULT         0x00000200UL                                    /**< Mode DEFAULT for USB_GNPTXFSIZ */
#define USB_GNPTXFSIZ_NPTXFSTADDR_DEFAULT          (_USB_GNPTXFSIZ_NPTXFSTADDR_DEFAULT << 0)       /**< Shifted mode DEFAULT for USB_GNPTXFSIZ */
#define _USB_GNPTXFSIZ_NPTXFINEPTXF0DEP_SHIFT      16                                              /**< Shift value for USB_NPTXFINEPTXF0DEP */
#define _USB_GNPTXFSIZ_NPTXFINEPTXF0DEP_MASK       0xFFFF0000UL                                    /**< Bit mask for USB_NPTXFINEPTXF0DEP */
#define _USB_GNPTXFSIZ_NPTXFINEPTXF0DEP_DEFAULT    0x00000200UL                                    /**< Mode DEFAULT for USB_GNPTXFSIZ */
#define USB_GNPTXFSIZ_NPTXFINEPTXF0DEP_DEFAULT     (_USB_GNPTXFSIZ_NPTXFINEPTXF0DEP_DEFAULT << 16) /**< Shifted mode DEFAULT for USB_GNPTXFSIZ */

/* Bit fields for USB GDFIFOCFG */
#define _USB_GDFIFOCFG_RESETVALUE                  0x05F80600UL                                  /**< Default value for USB_GDFIFOCFG */
#define _USB_GDFIFOCFG_MASK                        0xFFFFFFFFUL                                  /**< Mask for USB_GDFIFOCFG */
#define _USB_GDFIFOCFG_GDFIFOCFG_SHIFT             0                                             /**< Shift value for USB_GDFIFOCFG */
#define _USB_GDFIFOCFG_GDFIFOCFG_MASK              0xFFFFUL                                      /**< Bit mask for USB_GDFIFOCFG */
#define _USB_GDFIFOCFG_GDFIFOCFG_DEFAULT           0x00000600UL                                  /**< Mode DEFAULT for USB_GDFIFOCFG */
#define USB_GDFIFOCFG_GDFIFOCFG_DEFAULT            (_USB_GDFIFOCFG_GDFIFOCFG_DEFAULT << 0)       /**< Shifted mode DEFAULT for USB_GDFIFOCFG */
#define _USB_GDFIFOCFG_EPINFOBASEADDR_SHIFT        16                                            /**< Shift value for USB_EPINFOBASEADDR */
#define _USB_GDFIFOCFG_EPINFOBASEADDR_MASK         0xFFFF0000UL                                  /**< Bit mask for USB_EPINFOBASEADDR */
#define _USB_GDFIFOCFG_EPINFOBASEADDR_DEFAULT      0x000005F8UL                                  /**< Mode DEFAULT for USB_GDFIFOCFG */
#define USB_GDFIFOCFG_EPINFOBASEADDR_DEFAULT       (_USB_GDFIFOCFG_EPINFOBASEADDR_DEFAULT << 16) /**< Shifted mode DEFAULT for USB_GDFIFOCFG */

/* Bit fields for USB DIEPTXF1 */
#define _USB_DIEPTXF1_RESETVALUE                   0x02000400UL                                /**< Default value for USB_DIEPTXF1 */
#define _USB_DIEPTXF1_MASK                         0x03FF07FFUL                                /**< Mask for USB_DIEPTXF1 */
#define _USB_DIEPTXF1_INEPNTXFSTADDR_SHIFT         0                                           /**< Shift value for USB_INEPNTXFSTADDR */
#define _USB_DIEPTXF1_INEPNTXFSTADDR_MASK          0x7FFUL                                     /**< Bit mask for USB_INEPNTXFSTADDR */
#define _USB_DIEPTXF1_INEPNTXFSTADDR_DEFAULT       0x00000400UL                                /**< Mode DEFAULT for USB_DIEPTXF1 */
#define USB_DIEPTXF1_INEPNTXFSTADDR_DEFAULT        (_USB_DIEPTXF1_INEPNTXFSTADDR_DEFAULT << 0) /**< Shifted mode DEFAULT for USB_DIEPTXF1 */
#define _USB_DIEPTXF1_INEPNTXFDEP_SHIFT            16                                          /**< Shift value for USB_INEPNTXFDEP */
#define _USB_DIEPTXF1_INEPNTXFDEP_MASK             0x3FF0000UL                                 /**< Bit mask for USB_INEPNTXFDEP */
#define _USB_DIEPTXF1_INEPNTXFDEP_DEFAULT          0x00000200UL                                /**< Mode DEFAULT for USB_DIEPTXF1 */
#define USB_DIEPTXF1_INEPNTXFDEP_DEFAULT           (_USB_DIEPTXF1_INEPNTXFDEP_DEFAULT << 16)   /**< Shifted mode DEFAULT for USB_DIEPTXF1 */

/* Bit fields for USB DIEPTXF2 */
#define _USB_DIEPTXF2_RESETVALUE                   0x02000600UL                                /**< Default value for USB_DIEPTXF2 */
#define _USB_DIEPTXF2_MASK                         0x03FF07FFUL                                /**< Mask for USB_DIEPTXF2 */
#define _USB_DIEPTXF2_INEPNTXFSTADDR_SHIFT         0                                           /**< Shift value for USB_INEPNTXFSTADDR */
#define _USB_DIEPTXF2_INEPNTXFSTADDR_MASK          0x7FFUL                                     /**< Bit mask for USB_INEPNTXFSTADDR */
#define _USB_DIEPTXF2_INEPNTXFSTADDR_DEFAULT       0x00000600UL                                /**< Mode DEFAULT for USB_DIEPTXF2 */
#define USB_DIEPTXF2_INEPNTXFSTADDR_DEFAULT        (_USB_DIEPTXF2_INEPNTXFSTADDR_DEFAULT << 0) /**< Shifted mode DEFAULT for USB_DIEPTXF2 */
#define _USB_DIEPTXF2_INEPNTXFDEP_SHIFT            16                                          /**< Shift value for USB_INEPNTXFDEP */
#define _USB_DIEPTXF2_INEPNTXFDEP_MASK             0x3FF0000UL                                 /**< Bit mask for USB_INEPNTXFDEP */
#define _USB_DIEPTXF2_INEPNTXFDEP_DEFAULT          0x00000200UL                                /**< Mode DEFAULT for USB_DIEPTXF2 */
#define USB_DIEPTXF2_INEPNTXFDEP_DEFAULT           (_USB_DIEPTXF2_INEPNTXFDEP_DEFAULT << 16)   /**< Shifted mode DEFAULT for USB_DIEPTXF2 */

/* Bit fields for USB DIEPTXF3 */
#define _USB_DIEPTXF3_RESETVALUE                   0x02000800UL                                /**< Default value for USB_DIEPTXF3 */
#define _USB_DIEPTXF3_MASK                         0x03FF0FFFUL                                /**< Mask for USB_DIEPTXF3 */
#define _USB_DIEPTXF3_INEPNTXFSTADDR_SHIFT         0                                           /**< Shift value for USB_INEPNTXFSTADDR */
#define _USB_DIEPTXF3_INEPNTXFSTADDR_MASK          0xFFFUL                                     /**< Bit mask for USB_INEPNTXFSTADDR */
#define _USB_DIEPTXF3_INEPNTXFSTADDR_DEFAULT       0x00000800UL                                /**< Mode DEFAULT for USB_DIEPTXF3 */
#define USB_DIEPTXF3_INEPNTXFSTADDR_DEFAULT        (_USB_DIEPTXF3_INEPNTXFSTADDR_DEFAULT << 0) /**< Shifted mode DEFAULT for USB_DIEPTXF3 */
#define _USB_DIEPTXF3_INEPNTXFDEP_SHIFT            16                                          /**< Shift value for USB_INEPNTXFDEP */
#define _USB_DIEPTXF3_INEPNTXFDEP_MASK             0x3FF0000UL                                 /**< Bit mask for USB_INEPNTXFDEP */
#define _USB_DIEPTXF3_INEPNTXFDEP_DEFAULT          0x00000200UL                                /**< Mode DEFAULT for USB_DIEPTXF3 */
#define USB_DIEPTXF3_INEPNTXFDEP_DEFAULT           (_USB_DIEPTXF3_INEPNTXFDEP_DEFAULT << 16)   /**< Shifted mode DEFAULT for USB_DIEPTXF3 */

/* Bit fields for USB DCFG */
#define _USB_DCFG_RESETVALUE                       0x08000000UL                            /**< Default value for USB_DCFG */
#define _USB_DCFG_MASK                             0xFC009FFFUL                            /**< Mask for USB_DCFG */
#define _USB_DCFG_DEVSPD_SHIFT                     0                                       /**< Shift value for USB_DEVSPD */
#define _USB_DCFG_DEVSPD_MASK                      0x3UL                                   /**< Bit mask for USB_DEVSPD */
#define _USB_DCFG_DEVSPD_DEFAULT                   0x00000000UL                            /**< Mode DEFAULT for USB_DCFG */
#define _USB_DCFG_DEVSPD_LS                        0x00000002UL                            /**< Mode LS for USB_DCFG */
#define _USB_DCFG_DEVSPD_FS                        0x00000003UL                            /**< Mode FS for USB_DCFG */
#define USB_DCFG_DEVSPD_DEFAULT                    (_USB_DCFG_DEVSPD_DEFAULT << 0)         /**< Shifted mode DEFAULT for USB_DCFG */
#define USB_DCFG_DEVSPD_LS                         (_USB_DCFG_DEVSPD_LS << 0)              /**< Shifted mode LS for USB_DCFG */
#define USB_DCFG_DEVSPD_FS                         (_USB_DCFG_DEVSPD_FS << 0)              /**< Shifted mode FS for USB_DCFG */
#define USB_DCFG_NZSTSOUTHSHK                      (0x1UL << 2)                            /**< Non-Zero-Length Status OUT Handshake */
#define _USB_DCFG_NZSTSOUTHSHK_SHIFT               2                                       /**< Shift value for USB_NZSTSOUTHSHK */
#define _USB_DCFG_NZSTSOUTHSHK_MASK                0x4UL                                   /**< Bit mask for USB_NZSTSOUTHSHK */
#define _USB_DCFG_NZSTSOUTHSHK_DEFAULT             0x00000000UL                            /**< Mode DEFAULT for USB_DCFG */
#define USB_DCFG_NZSTSOUTHSHK_DEFAULT              (_USB_DCFG_NZSTSOUTHSHK_DEFAULT << 2)   /**< Shifted mode DEFAULT for USB_DCFG */
#define USB_DCFG_ENA32KHZSUSP                      (0x1UL << 3)                            /**< Enable 32 KHz Suspend mode */
#define _USB_DCFG_ENA32KHZSUSP_SHIFT               3                                       /**< Shift value for USB_ENA32KHZSUSP */
#define _USB_DCFG_ENA32KHZSUSP_MASK                0x8UL                                   /**< Bit mask for USB_ENA32KHZSUSP */
#define _USB_DCFG_ENA32KHZSUSP_DEFAULT             0x00000000UL                            /**< Mode DEFAULT for USB_DCFG */
#define USB_DCFG_ENA32KHZSUSP_DEFAULT              (_USB_DCFG_ENA32KHZSUSP_DEFAULT << 3)   /**< Shifted mode DEFAULT for USB_DCFG */
#define _USB_DCFG_DEVADDR_SHIFT                    4                                       /**< Shift value for USB_DEVADDR */
#define _USB_DCFG_DEVADDR_MASK                     0x7F0UL                                 /**< Bit mask for USB_DEVADDR */
#define _USB_DCFG_DEVADDR_DEFAULT                  0x00000000UL                            /**< Mode DEFAULT for USB_DCFG */
#define USB_DCFG_DEVADDR_DEFAULT                   (_USB_DCFG_DEVADDR_DEFAULT << 4)        /**< Shifted mode DEFAULT for USB_DCFG */
#define _USB_DCFG_PERFRINT_SHIFT                   11                                      /**< Shift value for USB_PERFRINT */
#define _USB_DCFG_PERFRINT_MASK                    0x1800UL                                /**< Bit mask for USB_PERFRINT */
#define _USB_DCFG_PERFRINT_DEFAULT                 0x00000000UL                            /**< Mode DEFAULT for USB_DCFG */
#define _USB_DCFG_PERFRINT_80PCNT                  0x00000000UL                            /**< Mode 80PCNT for USB_DCFG */
#define _USB_DCFG_PERFRINT_85PCNT                  0x00000001UL                            /**< Mode 85PCNT for USB_DCFG */
#define _USB_DCFG_PERFRINT_90PCNT                  0x00000002UL                            /**< Mode 90PCNT for USB_DCFG */
#define _USB_DCFG_PERFRINT_95PCNT                  0x00000003UL                            /**< Mode 95PCNT for USB_DCFG */
#define USB_DCFG_PERFRINT_DEFAULT                  (_USB_DCFG_PERFRINT_DEFAULT << 11)      /**< Shifted mode DEFAULT for USB_DCFG */
#define USB_DCFG_PERFRINT_80PCNT                   (_USB_DCFG_PERFRINT_80PCNT << 11)       /**< Shifted mode 80PCNT for USB_DCFG */
#define USB_DCFG_PERFRINT_85PCNT                   (_USB_DCFG_PERFRINT_85PCNT << 11)       /**< Shifted mode 85PCNT for USB_DCFG */
#define USB_DCFG_PERFRINT_90PCNT                   (_USB_DCFG_PERFRINT_90PCNT << 11)       /**< Shifted mode 90PCNT for USB_DCFG */
#define USB_DCFG_PERFRINT_95PCNT                   (_USB_DCFG_PERFRINT_95PCNT << 11)       /**< Shifted mode 95PCNT for USB_DCFG */
#define USB_DCFG_ERRATICINTMSK                     (0x1UL << 15)                           /**<  */
#define _USB_DCFG_ERRATICINTMSK_SHIFT              15                                      /**< Shift value for USB_ERRATICINTMSK */
#define _USB_DCFG_ERRATICINTMSK_MASK               0x8000UL                                /**< Bit mask for USB_ERRATICINTMSK */
#define _USB_DCFG_ERRATICINTMSK_DEFAULT            0x00000000UL                            /**< Mode DEFAULT for USB_DCFG */
#define USB_DCFG_ERRATICINTMSK_DEFAULT             (_USB_DCFG_ERRATICINTMSK_DEFAULT << 15) /**< Shifted mode DEFAULT for USB_DCFG */
#define _USB_DCFG_RESVALID_SHIFT                   26                                      /**< Shift value for USB_RESVALID */
#define _USB_DCFG_RESVALID_MASK                    0xFC000000UL                            /**< Bit mask for USB_RESVALID */
#define _USB_DCFG_RESVALID_DEFAULT                 0x00000002UL                            /**< Mode DEFAULT for USB_DCFG */
#define USB_DCFG_RESVALID_DEFAULT                  (_USB_DCFG_RESVALID_DEFAULT << 26)      /**< Shifted mode DEFAULT for USB_DCFG */

/* Bit fields for USB DCTL */
#define _USB_DCTL_RESETVALUE                       0x00000002UL                           /**< Default value for USB_DCTL */
#define _USB_DCTL_MASK                             0x00018FFFUL                           /**< Mask for USB_DCTL */
#define USB_DCTL_RMTWKUPSIG                        (0x1UL << 0)                           /**< Remote Wakeup Signaling */
#define _USB_DCTL_RMTWKUPSIG_SHIFT                 0                                      /**< Shift value for USB_RMTWKUPSIG */
#define _USB_DCTL_RMTWKUPSIG_MASK                  0x1UL                                  /**< Bit mask for USB_RMTWKUPSIG */
#define _USB_DCTL_RMTWKUPSIG_DEFAULT               0x00000000UL                           /**< Mode DEFAULT for USB_DCTL */
#define USB_DCTL_RMTWKUPSIG_DEFAULT                (_USB_DCTL_RMTWKUPSIG_DEFAULT << 0)    /**< Shifted mode DEFAULT for USB_DCTL */
#define USB_DCTL_SFTDISCON                         (0x1UL << 1)                           /**< Soft Disconnect */
#define _USB_DCTL_SFTDISCON_SHIFT                  1                                      /**< Shift value for USB_SFTDISCON */
#define _USB_DCTL_SFTDISCON_MASK                   0x2UL                                  /**< Bit mask for USB_SFTDISCON */
#define _USB_DCTL_SFTDISCON_DEFAULT                0x00000001UL                           /**< Mode DEFAULT for USB_DCTL */
#define USB_DCTL_SFTDISCON_DEFAULT                 (_USB_DCTL_SFTDISCON_DEFAULT << 1)     /**< Shifted mode DEFAULT for USB_DCTL */
#define USB_DCTL_GNPINNAKSTS                       (0x1UL << 2)                           /**< Global Non-periodic IN NAK Status */
#define _USB_DCTL_GNPINNAKSTS_SHIFT                2                                      /**< Shift value for USB_GNPINNAKSTS */
#define _USB_DCTL_GNPINNAKSTS_MASK                 0x4UL                                  /**< Bit mask for USB_GNPINNAKSTS */
#define _USB_DCTL_GNPINNAKSTS_DEFAULT              0x00000000UL                           /**< Mode DEFAULT for USB_DCTL */
#define USB_DCTL_GNPINNAKSTS_DEFAULT               (_USB_DCTL_GNPINNAKSTS_DEFAULT << 2)   /**< Shifted mode DEFAULT for USB_DCTL */
#define USB_DCTL_GOUTNAKSTS                        (0x1UL << 3)                           /**< Global OUT NAK Status */
#define _USB_DCTL_GOUTNAKSTS_SHIFT                 3                                      /**< Shift value for USB_GOUTNAKSTS */
#define _USB_DCTL_GOUTNAKSTS_MASK                  0x8UL                                  /**< Bit mask for USB_GOUTNAKSTS */
#define _USB_DCTL_GOUTNAKSTS_DEFAULT               0x00000000UL                           /**< Mode DEFAULT for USB_DCTL */
#define USB_DCTL_GOUTNAKSTS_DEFAULT                (_USB_DCTL_GOUTNAKSTS_DEFAULT << 3)    /**< Shifted mode DEFAULT for USB_DCTL */
#define _USB_DCTL_TSTCTL_SHIFT                     4                                      /**< Shift value for USB_TSTCTL */
#define _USB_DCTL_TSTCTL_MASK                      0x70UL                                 /**< Bit mask for USB_TSTCTL */
#define _USB_DCTL_TSTCTL_DEFAULT                   0x00000000UL                           /**< Mode DEFAULT for USB_DCTL */
#define _USB_DCTL_TSTCTL_DISABLE                   0x00000000UL                           /**< Mode DISABLE for USB_DCTL */
#define _USB_DCTL_TSTCTL_J                         0x00000001UL                           /**< Mode J for USB_DCTL */
#define _USB_DCTL_TSTCTL_K                         0x00000002UL                           /**< Mode K for USB_DCTL */
#define _USB_DCTL_TSTCTL_SE0NAK                    0x00000003UL                           /**< Mode SE0NAK for USB_DCTL */
#define _USB_DCTL_TSTCTL_PACKET                    0x00000004UL                           /**< Mode PACKET for USB_DCTL */
#define _USB_DCTL_TSTCTL_FORCE                     0x00000005UL                           /**< Mode FORCE for USB_DCTL */
#define USB_DCTL_TSTCTL_DEFAULT                    (_USB_DCTL_TSTCTL_DEFAULT << 4)        /**< Shifted mode DEFAULT for USB_DCTL */
#define USB_DCTL_TSTCTL_DISABLE                    (_USB_DCTL_TSTCTL_DISABLE << 4)        /**< Shifted mode DISABLE for USB_DCTL */
#define USB_DCTL_TSTCTL_J                          (_USB_DCTL_TSTCTL_J << 4)              /**< Shifted mode J for USB_DCTL */
#define USB_DCTL_TSTCTL_K                          (_USB_DCTL_TSTCTL_K << 4)              /**< Shifted mode K for USB_DCTL */
#define USB_DCTL_TSTCTL_SE0NAK                     (_USB_DCTL_TSTCTL_SE0NAK << 4)         /**< Shifted mode SE0NAK for USB_DCTL */
#define USB_DCTL_TSTCTL_PACKET                     (_USB_DCTL_TSTCTL_PACKET << 4)         /**< Shifted mode PACKET for USB_DCTL */
#define USB_DCTL_TSTCTL_FORCE                      (_USB_DCTL_TSTCTL_FORCE << 4)          /**< Shifted mode FORCE for USB_DCTL */
#define USB_DCTL_SGNPINNAK                         (0x1UL << 7)                           /**< Set Global Non-periodic IN NAK */
#define _USB_DCTL_SGNPINNAK_SHIFT                  7                                      /**< Shift value for USB_SGNPINNAK */
#define _USB_DCTL_SGNPINNAK_MASK                   0x80UL                                 /**< Bit mask for USB_SGNPINNAK */
#define _USB_DCTL_SGNPINNAK_DEFAULT                0x00000000UL                           /**< Mode DEFAULT for USB_DCTL */
#define USB_DCTL_SGNPINNAK_DEFAULT                 (_USB_DCTL_SGNPINNAK_DEFAULT << 7)     /**< Shifted mode DEFAULT for USB_DCTL */
#define USB_DCTL_CGNPINNAK                         (0x1UL << 8)                           /**< Clear Global Non-periodic IN NAK */
#define _USB_DCTL_CGNPINNAK_SHIFT                  8                                      /**< Shift value for USB_CGNPINNAK */
#define _USB_DCTL_CGNPINNAK_MASK                   0x100UL                                /**< Bit mask for USB_CGNPINNAK */
#define _USB_DCTL_CGNPINNAK_DEFAULT                0x00000000UL                           /**< Mode DEFAULT for USB_DCTL */
#define USB_DCTL_CGNPINNAK_DEFAULT                 (_USB_DCTL_CGNPINNAK_DEFAULT << 8)     /**< Shifted mode DEFAULT for USB_DCTL */
#define USB_DCTL_SGOUTNAK                          (0x1UL << 9)                           /**< Set Global OUT NAK */
#define _USB_DCTL_SGOUTNAK_SHIFT                   9                                      /**< Shift value for USB_SGOUTNAK */
#define _USB_DCTL_SGOUTNAK_MASK                    0x200UL                                /**< Bit mask for USB_SGOUTNAK */
#define _USB_DCTL_SGOUTNAK_DEFAULT                 0x00000000UL                           /**< Mode DEFAULT for USB_DCTL */
#define USB_DCTL_SGOUTNAK_DEFAULT                  (_USB_DCTL_SGOUTNAK_DEFAULT << 9)      /**< Shifted mode DEFAULT for USB_DCTL */
#define USB_DCTL_CGOUTNAK                          (0x1UL << 10)                          /**< Clear Global OUT NAK */
#define _USB_DCTL_CGOUTNAK_SHIFT                   10                                     /**< Shift value for USB_CGOUTNAK */
#define _USB_DCTL_CGOUTNAK_MASK                    0x400UL                                /**< Bit mask for USB_CGOUTNAK */
#define _USB_DCTL_CGOUTNAK_DEFAULT                 0x00000000UL                           /**< Mode DEFAULT for USB_DCTL */
#define USB_DCTL_CGOUTNAK_DEFAULT                  (_USB_DCTL_CGOUTNAK_DEFAULT << 10)     /**< Shifted mode DEFAULT for USB_DCTL */
#define USB_DCTL_PWRONPRGDONE                      (0x1UL << 11)                          /**< Power-On Programming Done */
#define _USB_DCTL_PWRONPRGDONE_SHIFT               11                                     /**< Shift value for USB_PWRONPRGDONE */
#define _USB_DCTL_PWRONPRGDONE_MASK                0x800UL                                /**< Bit mask for USB_PWRONPRGDONE */
#define _USB_DCTL_PWRONPRGDONE_DEFAULT             0x00000000UL                           /**< Mode DEFAULT for USB_DCTL */
#define USB_DCTL_PWRONPRGDONE_DEFAULT              (_USB_DCTL_PWRONPRGDONE_DEFAULT << 11) /**< Shifted mode DEFAULT for USB_DCTL */
#define USB_DCTL_IGNRFRMNUM                        (0x1UL << 15)                          /**< Ignore Frame number For Isochronous End points */
#define _USB_DCTL_IGNRFRMNUM_SHIFT                 15                                     /**< Shift value for USB_IGNRFRMNUM */
#define _USB_DCTL_IGNRFRMNUM_MASK                  0x8000UL                               /**< Bit mask for USB_IGNRFRMNUM */
#define _USB_DCTL_IGNRFRMNUM_DEFAULT               0x00000000UL                           /**< Mode DEFAULT for USB_DCTL */
#define USB_DCTL_IGNRFRMNUM_DEFAULT                (_USB_DCTL_IGNRFRMNUM_DEFAULT << 15)   /**< Shifted mode DEFAULT for USB_DCTL */
#define USB_DCTL_NAKONBBLE                         (0x1UL << 16)                          /**< NAK on Babble Error */
#define _USB_DCTL_NAKONBBLE_SHIFT                  16                                     /**< Shift value for USB_NAKONBBLE */
#define _USB_DCTL_NAKONBBLE_MASK                   0x10000UL                              /**< Bit mask for USB_NAKONBBLE */
#define _USB_DCTL_NAKONBBLE_DEFAULT                0x00000000UL                           /**< Mode DEFAULT for USB_DCTL */
#define USB_DCTL_NAKONBBLE_DEFAULT                 (_USB_DCTL_NAKONBBLE_DEFAULT << 16)    /**< Shifted mode DEFAULT for USB_DCTL */

/* Bit fields for USB DSTS */
#define _USB_DSTS_RESETVALUE                       0x00000002UL                       /**< Default value for USB_DSTS */
#define _USB_DSTS_MASK                             0x00FFFF0FUL                       /**< Mask for USB_DSTS */
#define USB_DSTS_SUSPSTS                           (0x1UL << 0)                       /**< Suspend Status */
#define _USB_DSTS_SUSPSTS_SHIFT                    0                                  /**< Shift value for USB_SUSPSTS */
#define _USB_DSTS_SUSPSTS_MASK                     0x1UL                              /**< Bit mask for USB_SUSPSTS */
#define _USB_DSTS_SUSPSTS_DEFAULT                  0x00000000UL                       /**< Mode DEFAULT for USB_DSTS */
#define USB_DSTS_SUSPSTS_DEFAULT                   (_USB_DSTS_SUSPSTS_DEFAULT << 0)   /**< Shifted mode DEFAULT for USB_DSTS */
#define _USB_DSTS_ENUMSPD_SHIFT                    1                                  /**< Shift value for USB_ENUMSPD */
#define _USB_DSTS_ENUMSPD_MASK                     0x6UL                              /**< Bit mask for USB_ENUMSPD */
#define _USB_DSTS_ENUMSPD_DEFAULT                  0x00000001UL                       /**< Mode DEFAULT for USB_DSTS */
#define _USB_DSTS_ENUMSPD_LS                       0x00000002UL                       /**< Mode LS for USB_DSTS */
#define _USB_DSTS_ENUMSPD_FS                       0x00000003UL                       /**< Mode FS for USB_DSTS */
#define USB_DSTS_ENUMSPD_DEFAULT                   (_USB_DSTS_ENUMSPD_DEFAULT << 1)   /**< Shifted mode DEFAULT for USB_DSTS */
#define USB_DSTS_ENUMSPD_LS                        (_USB_DSTS_ENUMSPD_LS << 1)        /**< Shifted mode LS for USB_DSTS */
#define USB_DSTS_ENUMSPD_FS                        (_USB_DSTS_ENUMSPD_FS << 1)        /**< Shifted mode FS for USB_DSTS */
#define USB_DSTS_ERRTICERR                         (0x1UL << 3)                       /**< Erratic Error */
#define _USB_DSTS_ERRTICERR_SHIFT                  3                                  /**< Shift value for USB_ERRTICERR */
#define _USB_DSTS_ERRTICERR_MASK                   0x8UL                              /**< Bit mask for USB_ERRTICERR */
#define _USB_DSTS_ERRTICERR_DEFAULT                0x00000000UL                       /**< Mode DEFAULT for USB_DSTS */
#define USB_DSTS_ERRTICERR_DEFAULT                 (_USB_DSTS_ERRTICERR_DEFAULT << 3) /**< Shifted mode DEFAULT for USB_DSTS */
#define _USB_DSTS_SOFFN_SHIFT                      8                                  /**< Shift value for USB_SOFFN */
#define _USB_DSTS_SOFFN_MASK                       0x3FFF00UL                         /**< Bit mask for USB_SOFFN */
#define _USB_DSTS_SOFFN_DEFAULT                    0x00000000UL                       /**< Mode DEFAULT for USB_DSTS */
#define USB_DSTS_SOFFN_DEFAULT                     (_USB_DSTS_SOFFN_DEFAULT << 8)     /**< Shifted mode DEFAULT for USB_DSTS */
#define _USB_DSTS_DEVLNSTS_SHIFT                   22                                 /**< Shift value for USB_DEVLNSTS */
#define _USB_DSTS_DEVLNSTS_MASK                    0xC00000UL                         /**< Bit mask for USB_DEVLNSTS */
#define _USB_DSTS_DEVLNSTS_DEFAULT                 0x00000000UL                       /**< Mode DEFAULT for USB_DSTS */
#define USB_DSTS_DEVLNSTS_DEFAULT                  (_USB_DSTS_DEVLNSTS_DEFAULT << 22) /**< Shifted mode DEFAULT for USB_DSTS */

/* Bit fields for USB DIEPMSK */
#define _USB_DIEPMSK_RESETVALUE                    0x00000000UL                               /**< Default value for USB_DIEPMSK */
#define _USB_DIEPMSK_MASK                          0x0000215FUL                               /**< Mask for USB_DIEPMSK */
#define USB_DIEPMSK_XFERCOMPLMSK                   (0x1UL << 0)                               /**< Transfer Completed Interrupt Mask */
#define _USB_DIEPMSK_XFERCOMPLMSK_SHIFT            0                                          /**< Shift value for USB_XFERCOMPLMSK */
#define _USB_DIEPMSK_XFERCOMPLMSK_MASK             0x1UL                                      /**< Bit mask for USB_XFERCOMPLMSK */
#define _USB_DIEPMSK_XFERCOMPLMSK_DEFAULT          0x00000000UL                               /**< Mode DEFAULT for USB_DIEPMSK */
#define USB_DIEPMSK_XFERCOMPLMSK_DEFAULT           (_USB_DIEPMSK_XFERCOMPLMSK_DEFAULT << 0)   /**< Shifted mode DEFAULT for USB_DIEPMSK */
#define USB_DIEPMSK_EPDISBLDMSK                    (0x1UL << 1)                               /**< Endpoint Disabled Interrupt Mask */
#define _USB_DIEPMSK_EPDISBLDMSK_SHIFT             1                                          /**< Shift value for USB_EPDISBLDMSK */
#define _USB_DIEPMSK_EPDISBLDMSK_MASK              0x2UL                                      /**< Bit mask for USB_EPDISBLDMSK */
#define _USB_DIEPMSK_EPDISBLDMSK_DEFAULT           0x00000000UL                               /**< Mode DEFAULT for USB_DIEPMSK */
#define USB_DIEPMSK_EPDISBLDMSK_DEFAULT            (_USB_DIEPMSK_EPDISBLDMSK_DEFAULT << 1)    /**< Shifted mode DEFAULT for USB_DIEPMSK */
#define USB_DIEPMSK_AHBERRMSK                      (0x1UL << 2)                               /**< AHB Error Mask */
#define _USB_DIEPMSK_AHBERRMSK_SHIFT               2                                          /**< Shift value for USB_AHBERRMSK */
#define _USB_DIEPMSK_AHBERRMSK_MASK                0x4UL                                      /**< Bit mask for USB_AHBERRMSK */
#define _USB_DIEPMSK_AHBERRMSK_DEFAULT             0x00000000UL                               /**< Mode DEFAULT for USB_DIEPMSK */
#define USB_DIEPMSK_AHBERRMSK_DEFAULT              (_USB_DIEPMSK_AHBERRMSK_DEFAULT << 2)      /**< Shifted mode DEFAULT for USB_DIEPMSK */
#define USB_DIEPMSK_TIMEOUTMSK                     (0x1UL << 3)                               /**< Timeout Condition Mask */
#define _USB_DIEPMSK_TIMEOUTMSK_SHIFT              3                                          /**< Shift value for USB_TIMEOUTMSK */
#define _USB_DIEPMSK_TIMEOUTMSK_MASK               0x8UL                                      /**< Bit mask for USB_TIMEOUTMSK */
#define _USB_DIEPMSK_TIMEOUTMSK_DEFAULT            0x00000000UL                               /**< Mode DEFAULT for USB_DIEPMSK */
#define USB_DIEPMSK_TIMEOUTMSK_DEFAULT             (_USB_DIEPMSK_TIMEOUTMSK_DEFAULT << 3)     /**< Shifted mode DEFAULT for USB_DIEPMSK */
#define USB_DIEPMSK_INTKNTXFEMPMSK                 (0x1UL << 4)                               /**< IN Token Received When TxFIFO Empty Mask */
#define _USB_DIEPMSK_INTKNTXFEMPMSK_SHIFT          4                                          /**< Shift value for USB_INTKNTXFEMPMSK */
#define _USB_DIEPMSK_INTKNTXFEMPMSK_MASK           0x10UL                                     /**< Bit mask for USB_INTKNTXFEMPMSK */
#define _USB_DIEPMSK_INTKNTXFEMPMSK_DEFAULT        0x00000000UL                               /**< Mode DEFAULT for USB_DIEPMSK */
#define USB_DIEPMSK_INTKNTXFEMPMSK_DEFAULT         (_USB_DIEPMSK_INTKNTXFEMPMSK_DEFAULT << 4) /**< Shifted mode DEFAULT for USB_DIEPMSK */
#define USB_DIEPMSK_INEPNAKEFFMSK                  (0x1UL << 6)                               /**< IN Endpoint NAK Effective Mask */
#define _USB_DIEPMSK_INEPNAKEFFMSK_SHIFT           6                                          /**< Shift value for USB_INEPNAKEFFMSK */
#define _USB_DIEPMSK_INEPNAKEFFMSK_MASK            0x40UL                                     /**< Bit mask for USB_INEPNAKEFFMSK */
#define _USB_DIEPMSK_INEPNAKEFFMSK_DEFAULT         0x00000000UL                               /**< Mode DEFAULT for USB_DIEPMSK */
#define USB_DIEPMSK_INEPNAKEFFMSK_DEFAULT          (_USB_DIEPMSK_INEPNAKEFFMSK_DEFAULT << 6)  /**< Shifted mode DEFAULT for USB_DIEPMSK */
#define USB_DIEPMSK_TXFIFOUNDRNMSK                 (0x1UL << 8)                               /**< Fifo Underrun Mask */
#define _USB_DIEPMSK_TXFIFOUNDRNMSK_SHIFT          8                                          /**< Shift value for USB_TXFIFOUNDRNMSK */
#define _USB_DIEPMSK_TXFIFOUNDRNMSK_MASK           0x100UL                                    /**< Bit mask for USB_TXFIFOUNDRNMSK */
#define _USB_DIEPMSK_TXFIFOUNDRNMSK_DEFAULT        0x00000000UL                               /**< Mode DEFAULT for USB_DIEPMSK */
#define USB_DIEPMSK_TXFIFOUNDRNMSK_DEFAULT         (_USB_DIEPMSK_TXFIFOUNDRNMSK_DEFAULT << 8) /**< Shifted mode DEFAULT for USB_DIEPMSK */
#define USB_DIEPMSK_NAKMSK                         (0x1UL << 13)                              /**< NAK interrupt Mask */
#define _USB_DIEPMSK_NAKMSK_SHIFT                  13                                         /**< Shift value for USB_NAKMSK */
#define _USB_DIEPMSK_NAKMSK_MASK                   0x2000UL                                   /**< Bit mask for USB_NAKMSK */
#define _USB_DIEPMSK_NAKMSK_DEFAULT                0x00000000UL                               /**< Mode DEFAULT for USB_DIEPMSK */
#define USB_DIEPMSK_NAKMSK_DEFAULT                 (_USB_DIEPMSK_NAKMSK_DEFAULT << 13)        /**< Shifted mode DEFAULT for USB_DIEPMSK */

/* Bit fields for USB DOEPMSK */
#define _USB_DOEPMSK_RESETVALUE                    0x00000000UL                               /**< Default value for USB_DOEPMSK */
#define _USB_DOEPMSK_MASK                          0x0000317FUL                               /**< Mask for USB_DOEPMSK */
#define USB_DOEPMSK_XFERCOMPLMSK                   (0x1UL << 0)                               /**< Transfer Completed Interrupt Mask */
#define _USB_DOEPMSK_XFERCOMPLMSK_SHIFT            0                                          /**< Shift value for USB_XFERCOMPLMSK */
#define _USB_DOEPMSK_XFERCOMPLMSK_MASK             0x1UL                                      /**< Bit mask for USB_XFERCOMPLMSK */
#define _USB_DOEPMSK_XFERCOMPLMSK_DEFAULT          0x00000000UL                               /**< Mode DEFAULT for USB_DOEPMSK */
#define USB_DOEPMSK_XFERCOMPLMSK_DEFAULT           (_USB_DOEPMSK_XFERCOMPLMSK_DEFAULT << 0)   /**< Shifted mode DEFAULT for USB_DOEPMSK */
#define USB_DOEPMSK_EPDISBLDMSK                    (0x1UL << 1)                               /**< Endpoint Disabled Interrupt Mask */
#define _USB_DOEPMSK_EPDISBLDMSK_SHIFT             1                                          /**< Shift value for USB_EPDISBLDMSK */
#define _USB_DOEPMSK_EPDISBLDMSK_MASK              0x2UL                                      /**< Bit mask for USB_EPDISBLDMSK */
#define _USB_DOEPMSK_EPDISBLDMSK_DEFAULT           0x00000000UL                               /**< Mode DEFAULT for USB_DOEPMSK */
#define USB_DOEPMSK_EPDISBLDMSK_DEFAULT            (_USB_DOEPMSK_EPDISBLDMSK_DEFAULT << 1)    /**< Shifted mode DEFAULT for USB_DOEPMSK */
#define USB_DOEPMSK_AHBERRMSK                      (0x1UL << 2)                               /**< AHB Error */
#define _USB_DOEPMSK_AHBERRMSK_SHIFT               2                                          /**< Shift value for USB_AHBERRMSK */
#define _USB_DOEPMSK_AHBERRMSK_MASK                0x4UL                                      /**< Bit mask for USB_AHBERRMSK */
#define _USB_DOEPMSK_AHBERRMSK_DEFAULT             0x00000000UL                               /**< Mode DEFAULT for USB_DOEPMSK */
#define USB_DOEPMSK_AHBERRMSK_DEFAULT              (_USB_DOEPMSK_AHBERRMSK_DEFAULT << 2)      /**< Shifted mode DEFAULT for USB_DOEPMSK */
#define USB_DOEPMSK_SETUPMSK                       (0x1UL << 3)                               /**< SETUP Phase Done Mask */
#define _USB_DOEPMSK_SETUPMSK_SHIFT                3                                          /**< Shift value for USB_SETUPMSK */
#define _USB_DOEPMSK_SETUPMSK_MASK                 0x8UL                                      /**< Bit mask for USB_SETUPMSK */
#define _USB_DOEPMSK_SETUPMSK_DEFAULT              0x00000000UL                               /**< Mode DEFAULT for USB_DOEPMSK */
#define USB_DOEPMSK_SETUPMSK_DEFAULT               (_USB_DOEPMSK_SETUPMSK_DEFAULT << 3)       /**< Shifted mode DEFAULT for USB_DOEPMSK */
#define USB_DOEPMSK_OUTTKNEPDISMSK                 (0x1UL << 4)                               /**< OUT Token Received when Endpoint Disabled Mask */
#define _USB_DOEPMSK_OUTTKNEPDISMSK_SHIFT          4                                          /**< Shift value for USB_OUTTKNEPDISMSK */
#define _USB_DOEPMSK_OUTTKNEPDISMSK_MASK           0x10UL                                     /**< Bit mask for USB_OUTTKNEPDISMSK */
#define _USB_DOEPMSK_OUTTKNEPDISMSK_DEFAULT        0x00000000UL                               /**< Mode DEFAULT for USB_DOEPMSK */
#define USB_DOEPMSK_OUTTKNEPDISMSK_DEFAULT         (_USB_DOEPMSK_OUTTKNEPDISMSK_DEFAULT << 4) /**< Shifted mode DEFAULT for USB_DOEPMSK */
#define USB_DOEPMSK_STSPHSERCVDMSK                 (0x1UL << 5)                               /**< Status Phase Received Mask */
#define _USB_DOEPMSK_STSPHSERCVDMSK_SHIFT          5                                          /**< Shift value for USB_STSPHSERCVDMSK */
#define _USB_DOEPMSK_STSPHSERCVDMSK_MASK           0x20UL                                     /**< Bit mask for USB_STSPHSERCVDMSK */
#define _USB_DOEPMSK_STSPHSERCVDMSK_DEFAULT        0x00000000UL                               /**< Mode DEFAULT for USB_DOEPMSK */
#define USB_DOEPMSK_STSPHSERCVDMSK_DEFAULT         (_USB_DOEPMSK_STSPHSERCVDMSK_DEFAULT << 5) /**< Shifted mode DEFAULT for USB_DOEPMSK */
#define USB_DOEPMSK_BACK2BACKSETUP                 (0x1UL << 6)                               /**< Back-to-Back SETUP Packets Received Mask */
#define _USB_DOEPMSK_BACK2BACKSETUP_SHIFT          6                                          /**< Shift value for USB_BACK2BACKSETUP */
#define _USB_DOEPMSK_BACK2BACKSETUP_MASK           0x40UL                                     /**< Bit mask for USB_BACK2BACKSETUP */
#define _USB_DOEPMSK_BACK2BACKSETUP_DEFAULT        0x00000000UL                               /**< Mode DEFAULT for USB_DOEPMSK */
#define USB_DOEPMSK_BACK2BACKSETUP_DEFAULT         (_USB_DOEPMSK_BACK2BACKSETUP_DEFAULT << 6) /**< Shifted mode DEFAULT for USB_DOEPMSK */
#define USB_DOEPMSK_OUTPKTERRMSK                   (0x1UL << 8)                               /**< OUT Packet Error Mask */
#define _USB_DOEPMSK_OUTPKTERRMSK_SHIFT            8                                          /**< Shift value for USB_OUTPKTERRMSK */
#define _USB_DOEPMSK_OUTPKTERRMSK_MASK             0x100UL                                    /**< Bit mask for USB_OUTPKTERRMSK */
#define _USB_DOEPMSK_OUTPKTERRMSK_DEFAULT          0x00000000UL                               /**< Mode DEFAULT for USB_DOEPMSK */
#define USB_DOEPMSK_OUTPKTERRMSK_DEFAULT           (_USB_DOEPMSK_OUTPKTERRMSK_DEFAULT << 8)   /**< Shifted mode DEFAULT for USB_DOEPMSK */
#define USB_DOEPMSK_BBLEERRMSK                     (0x1UL << 12)                              /**< Babble Error interrupt Mask */
#define _USB_DOEPMSK_BBLEERRMSK_SHIFT              12                                         /**< Shift value for USB_BBLEERRMSK */
#define _USB_DOEPMSK_BBLEERRMSK_MASK               0x1000UL                                   /**< Bit mask for USB_BBLEERRMSK */
#define _USB_DOEPMSK_BBLEERRMSK_DEFAULT            0x00000000UL                               /**< Mode DEFAULT for USB_DOEPMSK */
#define USB_DOEPMSK_BBLEERRMSK_DEFAULT             (_USB_DOEPMSK_BBLEERRMSK_DEFAULT << 12)    /**< Shifted mode DEFAULT for USB_DOEPMSK */
#define USB_DOEPMSK_NAKMSK                         (0x1UL << 13)                              /**< NAK interrupt Mask */
#define _USB_DOEPMSK_NAKMSK_SHIFT                  13                                         /**< Shift value for USB_NAKMSK */
#define _USB_DOEPMSK_NAKMSK_MASK                   0x2000UL                                   /**< Bit mask for USB_NAKMSK */
#define _USB_DOEPMSK_NAKMSK_DEFAULT                0x00000000UL                               /**< Mode DEFAULT for USB_DOEPMSK */
#define USB_DOEPMSK_NAKMSK_DEFAULT                 (_USB_DOEPMSK_NAKMSK_DEFAULT << 13)        /**< Shifted mode DEFAULT for USB_DOEPMSK */

/* Bit fields for USB DAINT */
#define _USB_DAINT_RESETVALUE                      0x00000000UL                         /**< Default value for USB_DAINT */
#define _USB_DAINT_MASK                            0x000F000FUL                         /**< Mask for USB_DAINT */
#define USB_DAINT_INEPINT0                         (0x1UL << 0)                         /**< IN Endpoint 0 Interrupt Bit */
#define _USB_DAINT_INEPINT0_SHIFT                  0                                    /**< Shift value for USB_INEPINT0 */
#define _USB_DAINT_INEPINT0_MASK                   0x1UL                                /**< Bit mask for USB_INEPINT0 */
#define _USB_DAINT_INEPINT0_DEFAULT                0x00000000UL                         /**< Mode DEFAULT for USB_DAINT */
#define USB_DAINT_INEPINT0_DEFAULT                 (_USB_DAINT_INEPINT0_DEFAULT << 0)   /**< Shifted mode DEFAULT for USB_DAINT */
#define USB_DAINT_INEPINT1                         (0x1UL << 1)                         /**< IN Endpoint 1 Interrupt Bit */
#define _USB_DAINT_INEPINT1_SHIFT                  1                                    /**< Shift value for USB_INEPINT1 */
#define _USB_DAINT_INEPINT1_MASK                   0x2UL                                /**< Bit mask for USB_INEPINT1 */
#define _USB_DAINT_INEPINT1_DEFAULT                0x00000000UL                         /**< Mode DEFAULT for USB_DAINT */
#define USB_DAINT_INEPINT1_DEFAULT                 (_USB_DAINT_INEPINT1_DEFAULT << 1)   /**< Shifted mode DEFAULT for USB_DAINT */
#define USB_DAINT_INEPINT2                         (0x1UL << 2)                         /**< IN Endpoint 2 Interrupt Bit */
#define _USB_DAINT_INEPINT2_SHIFT                  2                                    /**< Shift value for USB_INEPINT2 */
#define _USB_DAINT_INEPINT2_MASK                   0x4UL                                /**< Bit mask for USB_INEPINT2 */
#define _USB_DAINT_INEPINT2_DEFAULT                0x00000000UL                         /**< Mode DEFAULT for USB_DAINT */
#define USB_DAINT_INEPINT2_DEFAULT                 (_USB_DAINT_INEPINT2_DEFAULT << 2)   /**< Shifted mode DEFAULT for USB_DAINT */
#define USB_DAINT_INEPINT3                         (0x1UL << 3)                         /**< IN Endpoint 3 Interrupt Bit */
#define _USB_DAINT_INEPINT3_SHIFT                  3                                    /**< Shift value for USB_INEPINT3 */
#define _USB_DAINT_INEPINT3_MASK                   0x8UL                                /**< Bit mask for USB_INEPINT3 */
#define _USB_DAINT_INEPINT3_DEFAULT                0x00000000UL                         /**< Mode DEFAULT for USB_DAINT */
#define USB_DAINT_INEPINT3_DEFAULT                 (_USB_DAINT_INEPINT3_DEFAULT << 3)   /**< Shifted mode DEFAULT for USB_DAINT */
#define USB_DAINT_OUTEPINT0                        (0x1UL << 16)                        /**< OUT Endpoint 0 Interrupt Bit */
#define _USB_DAINT_OUTEPINT0_SHIFT                 16                                   /**< Shift value for USB_OUTEPINT0 */
#define _USB_DAINT_OUTEPINT0_MASK                  0x10000UL                            /**< Bit mask for USB_OUTEPINT0 */
#define _USB_DAINT_OUTEPINT0_DEFAULT               0x00000000UL                         /**< Mode DEFAULT for USB_DAINT */
#define USB_DAINT_OUTEPINT0_DEFAULT                (_USB_DAINT_OUTEPINT0_DEFAULT << 16) /**< Shifted mode DEFAULT for USB_DAINT */
#define USB_DAINT_OUTEPINT1                        (0x1UL << 17)                        /**< OUT Endpoint 1 Interrupt Bit */
#define _USB_DAINT_OUTEPINT1_SHIFT                 17                                   /**< Shift value for USB_OUTEPINT1 */
#define _USB_DAINT_OUTEPINT1_MASK                  0x20000UL                            /**< Bit mask for USB_OUTEPINT1 */
#define _USB_DAINT_OUTEPINT1_DEFAULT               0x00000000UL                         /**< Mode DEFAULT for USB_DAINT */
#define USB_DAINT_OUTEPINT1_DEFAULT                (_USB_DAINT_OUTEPINT1_DEFAULT << 17) /**< Shifted mode DEFAULT for USB_DAINT */
#define USB_DAINT_OUTEPINT2                        (0x1UL << 18)                        /**< OUT Endpoint 2 Interrupt Bit */
#define _USB_DAINT_OUTEPINT2_SHIFT                 18                                   /**< Shift value for USB_OUTEPINT2 */
#define _USB_DAINT_OUTEPINT2_MASK                  0x40000UL                            /**< Bit mask for USB_OUTEPINT2 */
#define _USB_DAINT_OUTEPINT2_DEFAULT               0x00000000UL                         /**< Mode DEFAULT for USB_DAINT */
#define USB_DAINT_OUTEPINT2_DEFAULT                (_USB_DAINT_OUTEPINT2_DEFAULT << 18) /**< Shifted mode DEFAULT for USB_DAINT */
#define USB_DAINT_OUTEPINT3                        (0x1UL << 19)                        /**< OUT Endpoint 3 Interrupt Bit */
#define _USB_DAINT_OUTEPINT3_SHIFT                 19                                   /**< Shift value for USB_OUTEPINT3 */
#define _USB_DAINT_OUTEPINT3_MASK                  0x80000UL                            /**< Bit mask for USB_OUTEPINT3 */
#define _USB_DAINT_OUTEPINT3_DEFAULT               0x00000000UL                         /**< Mode DEFAULT for USB_DAINT */
#define USB_DAINT_OUTEPINT3_DEFAULT                (_USB_DAINT_OUTEPINT3_DEFAULT << 19) /**< Shifted mode DEFAULT for USB_DAINT */

/* Bit fields for USB DAINTMSK */
#define _USB_DAINTMSK_RESETVALUE                   0x00000000UL                            /**< Default value for USB_DAINTMSK */
#define _USB_DAINTMSK_MASK                         0x000F000FUL                            /**< Mask for USB_DAINTMSK */
#define USB_DAINTMSK_INEPMSK0                      (0x1UL << 0)                            /**< IN Endpoint 0 Interrupt mask Bit */
#define _USB_DAINTMSK_INEPMSK0_SHIFT               0                                       /**< Shift value for USB_INEPMSK0 */
#define _USB_DAINTMSK_INEPMSK0_MASK                0x1UL                                   /**< Bit mask for USB_INEPMSK0 */
#define _USB_DAINTMSK_INEPMSK0_DEFAULT             0x00000000UL                            /**< Mode DEFAULT for USB_DAINTMSK */
#define USB_DAINTMSK_INEPMSK0_DEFAULT              (_USB_DAINTMSK_INEPMSK0_DEFAULT << 0)   /**< Shifted mode DEFAULT for USB_DAINTMSK */
#define USB_DAINTMSK_INEPMSK1                      (0x1UL << 1)                            /**< IN Endpoint 1 Interrupt mask Bit */
#define _USB_DAINTMSK_INEPMSK1_SHIFT               1                                       /**< Shift value for USB_INEPMSK1 */
#define _USB_DAINTMSK_INEPMSK1_MASK                0x2UL                                   /**< Bit mask for USB_INEPMSK1 */
#define _USB_DAINTMSK_INEPMSK1_DEFAULT             0x00000000UL                            /**< Mode DEFAULT for USB_DAINTMSK */
#define USB_DAINTMSK_INEPMSK1_DEFAULT              (_USB_DAINTMSK_INEPMSK1_DEFAULT << 1)   /**< Shifted mode DEFAULT for USB_DAINTMSK */
#define USB_DAINTMSK_INEPMSK2                      (0x1UL << 2)                            /**< IN Endpoint 2 Interrupt mask Bit */
#define _USB_DAINTMSK_INEPMSK2_SHIFT               2                                       /**< Shift value for USB_INEPMSK2 */
#define _USB_DAINTMSK_INEPMSK2_MASK                0x4UL                                   /**< Bit mask for USB_INEPMSK2 */
#define _USB_DAINTMSK_INEPMSK2_DEFAULT             0x00000000UL                            /**< Mode DEFAULT for USB_DAINTMSK */
#define USB_DAINTMSK_INEPMSK2_DEFAULT              (_USB_DAINTMSK_INEPMSK2_DEFAULT << 2)   /**< Shifted mode DEFAULT for USB_DAINTMSK */
#define USB_DAINTMSK_INEPMSK3                      (0x1UL << 3)                            /**< IN Endpoint 3 Interrupt mask Bit */
#define _USB_DAINTMSK_INEPMSK3_SHIFT               3                                       /**< Shift value for USB_INEPMSK3 */
#define _USB_DAINTMSK_INEPMSK3_MASK                0x8UL                                   /**< Bit mask for USB_INEPMSK3 */
#define _USB_DAINTMSK_INEPMSK3_DEFAULT             0x00000000UL                            /**< Mode DEFAULT for USB_DAINTMSK */
#define USB_DAINTMSK_INEPMSK3_DEFAULT              (_USB_DAINTMSK_INEPMSK3_DEFAULT << 3)   /**< Shifted mode DEFAULT for USB_DAINTMSK */
#define USB_DAINTMSK_OUTEPMSK0                     (0x1UL << 16)                           /**< OUT Endpoint 0 Interrupt mask Bit */
#define _USB_DAINTMSK_OUTEPMSK0_SHIFT              16                                      /**< Shift value for USB_OUTEPMSK0 */
#define _USB_DAINTMSK_OUTEPMSK0_MASK               0x10000UL                               /**< Bit mask for USB_OUTEPMSK0 */
#define _USB_DAINTMSK_OUTEPMSK0_DEFAULT            0x00000000UL                            /**< Mode DEFAULT for USB_DAINTMSK */
#define USB_DAINTMSK_OUTEPMSK0_DEFAULT             (_USB_DAINTMSK_OUTEPMSK0_DEFAULT << 16) /**< Shifted mode DEFAULT for USB_DAINTMSK */
#define USB_DAINTMSK_OUTEPMSK1                     (0x1UL << 17)                           /**< OUT Endpoint 1 Interrupt mask Bit */
#define _USB_DAINTMSK_OUTEPMSK1_SHIFT              17                                      /**< Shift value for USB_OUTEPMSK1 */
#define _USB_DAINTMSK_OUTEPMSK1_MASK               0x20000UL                               /**< Bit mask for USB_OUTEPMSK1 */
#define _USB_DAINTMSK_OUTEPMSK1_DEFAULT            0x00000000UL                            /**< Mode DEFAULT for USB_DAINTMSK */
#define USB_DAINTMSK_OUTEPMSK1_DEFAULT             (_USB_DAINTMSK_OUTEPMSK1_DEFAULT << 17) /**< Shifted mode DEFAULT for USB_DAINTMSK */
#define USB_DAINTMSK_OUTEPMSK2                     (0x1UL << 18)                           /**< OUT Endpoint 2 Interrupt mask Bit */
#define _USB_DAINTMSK_OUTEPMSK2_SHIFT              18                                      /**< Shift value for USB_OUTEPMSK2 */
#define _USB_DAINTMSK_OUTEPMSK2_MASK               0x40000UL                               /**< Bit mask for USB_OUTEPMSK2 */
#define _USB_DAINTMSK_OUTEPMSK2_DEFAULT            0x00000000UL                            /**< Mode DEFAULT for USB_DAINTMSK */
#define USB_DAINTMSK_OUTEPMSK2_DEFAULT             (_USB_DAINTMSK_OUTEPMSK2_DEFAULT << 18) /**< Shifted mode DEFAULT for USB_DAINTMSK */
#define USB_DAINTMSK_OUTEPMSK3                     (0x1UL << 19)                           /**< OUT Endpoint 3 Interrupt mask Bit */
#define _USB_DAINTMSK_OUTEPMSK3_SHIFT              19                                      /**< Shift value for USB_OUTEPMSK3 */
#define _USB_DAINTMSK_OUTEPMSK3_MASK               0x80000UL                               /**< Bit mask for USB_OUTEPMSK3 */
#define _USB_DAINTMSK_OUTEPMSK3_DEFAULT            0x00000000UL                            /**< Mode DEFAULT for USB_DAINTMSK */
#define USB_DAINTMSK_OUTEPMSK3_DEFAULT             (_USB_DAINTMSK_OUTEPMSK3_DEFAULT << 19) /**< Shifted mode DEFAULT for USB_DAINTMSK */

/* Bit fields for USB DIEPEMPMSK */
#define _USB_DIEPEMPMSK_RESETVALUE                 0x00000000UL                              /**< Default value for USB_DIEPEMPMSK */
#define _USB_DIEPEMPMSK_MASK                       0x0000FFFFUL                              /**< Mask for USB_DIEPEMPMSK */
#define _USB_DIEPEMPMSK_DIEPEMPMSK_SHIFT           0                                         /**< Shift value for USB_DIEPEMPMSK */
#define _USB_DIEPEMPMSK_DIEPEMPMSK_MASK            0xFFFFUL                                  /**< Bit mask for USB_DIEPEMPMSK */
#define _USB_DIEPEMPMSK_DIEPEMPMSK_DEFAULT         0x00000000UL                              /**< Mode DEFAULT for USB_DIEPEMPMSK */
#define USB_DIEPEMPMSK_DIEPEMPMSK_DEFAULT          (_USB_DIEPEMPMSK_DIEPEMPMSK_DEFAULT << 0) /**< Shifted mode DEFAULT for USB_DIEPEMPMSK */

/* Bit fields for USB DIEP0CTL */
#define _USB_DIEP0CTL_RESETVALUE                   0x00008000UL                           /**< Default value for USB_DIEP0CTL */
#define _USB_DIEP0CTL_MASK                         0xCFEE8003UL                           /**< Mask for USB_DIEP0CTL */
#define _USB_DIEP0CTL_MPS_SHIFT                    0                                      /**< Shift value for USB_MPS */
#define _USB_DIEP0CTL_MPS_MASK                     0x3UL                                  /**< Bit mask for USB_MPS */
#define _USB_DIEP0CTL_MPS_DEFAULT                  0x00000000UL                           /**< Mode DEFAULT for USB_DIEP0CTL */
#define _USB_DIEP0CTL_MPS_64B                      0x00000000UL                           /**< Mode 64B for USB_DIEP0CTL */
#define _USB_DIEP0CTL_MPS_32B                      0x00000001UL                           /**< Mode 32B for USB_DIEP0CTL */
#define _USB_DIEP0CTL_MPS_16B                      0x00000002UL                           /**< Mode 16B for USB_DIEP0CTL */
#define _USB_DIEP0CTL_MPS_8B                       0x00000003UL                           /**< Mode 8B for USB_DIEP0CTL */
#define USB_DIEP0CTL_MPS_DEFAULT                   (_USB_DIEP0CTL_MPS_DEFAULT << 0)       /**< Shifted mode DEFAULT for USB_DIEP0CTL */
#define USB_DIEP0CTL_MPS_64B                       (_USB_DIEP0CTL_MPS_64B << 0)           /**< Shifted mode 64B for USB_DIEP0CTL */
#define USB_DIEP0CTL_MPS_32B                       (_USB_DIEP0CTL_MPS_32B << 0)           /**< Shifted mode 32B for USB_DIEP0CTL */
#define USB_DIEP0CTL_MPS_16B                       (_USB_DIEP0CTL_MPS_16B << 0)           /**< Shifted mode 16B for USB_DIEP0CTL */
#define USB_DIEP0CTL_MPS_8B                        (_USB_DIEP0CTL_MPS_8B << 0)            /**< Shifted mode 8B for USB_DIEP0CTL */
#define USB_DIEP0CTL_USBACTEP                      (0x1UL << 15)                          /**< USB Active Endpoint */
#define _USB_DIEP0CTL_USBACTEP_SHIFT               15                                     /**< Shift value for USB_USBACTEP */
#define _USB_DIEP0CTL_USBACTEP_MASK                0x8000UL                               /**< Bit mask for USB_USBACTEP */
#define _USB_DIEP0CTL_USBACTEP_DEFAULT             0x00000001UL                           /**< Mode DEFAULT for USB_DIEP0CTL */
#define USB_DIEP0CTL_USBACTEP_DEFAULT              (_USB_DIEP0CTL_USBACTEP_DEFAULT << 15) /**< Shifted mode DEFAULT for USB_DIEP0CTL */
#define USB_DIEP0CTL_NAKSTS                        (0x1UL << 17)                          /**< NAK Status */
#define _USB_DIEP0CTL_NAKSTS_SHIFT                 17                                     /**< Shift value for USB_NAKSTS */
#define _USB_DIEP0CTL_NAKSTS_MASK                  0x20000UL                              /**< Bit mask for USB_NAKSTS */
#define _USB_DIEP0CTL_NAKSTS_DEFAULT               0x00000000UL                           /**< Mode DEFAULT for USB_DIEP0CTL */
#define USB_DIEP0CTL_NAKSTS_DEFAULT                (_USB_DIEP0CTL_NAKSTS_DEFAULT << 17)   /**< Shifted mode DEFAULT for USB_DIEP0CTL */
#define _USB_DIEP0CTL_EPTYPE_SHIFT                 18                                     /**< Shift value for USB_EPTYPE */
#define _USB_DIEP0CTL_EPTYPE_MASK                  0xC0000UL                              /**< Bit mask for USB_EPTYPE */
#define _USB_DIEP0CTL_EPTYPE_DEFAULT               0x00000000UL                           /**< Mode DEFAULT for USB_DIEP0CTL */
#define USB_DIEP0CTL_EPTYPE_DEFAULT                (_USB_DIEP0CTL_EPTYPE_DEFAULT << 18)   /**< Shifted mode DEFAULT for USB_DIEP0CTL */
#define USB_DIEP0CTL_STALL                         (0x1UL << 21)                          /**< Handshake */
#define _USB_DIEP0CTL_STALL_SHIFT                  21                                     /**< Shift value for USB_STALL */
#define _USB_DIEP0CTL_STALL_MASK                   0x200000UL                             /**< Bit mask for USB_STALL */
#define _USB_DIEP0CTL_STALL_DEFAULT                0x00000000UL                           /**< Mode DEFAULT for USB_DIEP0CTL */
#define USB_DIEP0CTL_STALL_DEFAULT                 (_USB_DIEP0CTL_STALL_DEFAULT << 21)    /**< Shifted mode DEFAULT for USB_DIEP0CTL */
#define _USB_DIEP0CTL_TXFNUM_SHIFT                 22                                     /**< Shift value for USB_TXFNUM */
#define _USB_DIEP0CTL_TXFNUM_MASK                  0x3C00000UL                            /**< Bit mask for USB_TXFNUM */
#define _USB_DIEP0CTL_TXFNUM_DEFAULT               0x00000000UL                           /**< Mode DEFAULT for USB_DIEP0CTL */
#define USB_DIEP0CTL_TXFNUM_DEFAULT                (_USB_DIEP0CTL_TXFNUM_DEFAULT << 22)   /**< Shifted mode DEFAULT for USB_DIEP0CTL */
#define USB_DIEP0CTL_CNAK                          (0x1UL << 26)                          /**< Clear NAK */
#define _USB_DIEP0CTL_CNAK_SHIFT                   26                                     /**< Shift value for USB_CNAK */
#define _USB_DIEP0CTL_CNAK_MASK                    0x4000000UL                            /**< Bit mask for USB_CNAK */
#define _USB_DIEP0CTL_CNAK_DEFAULT                 0x00000000UL                           /**< Mode DEFAULT for USB_DIEP0CTL */
#define USB_DIEP0CTL_CNAK_DEFAULT                  (_USB_DIEP0CTL_CNAK_DEFAULT << 26)     /**< Shifted mode DEFAULT for USB_DIEP0CTL */
#define USB_DIEP0CTL_SNAK                          (0x1UL << 27)                          /**< Set NAK */
#define _USB_DIEP0CTL_SNAK_SHIFT                   27                                     /**< Shift value for USB_SNAK */
#define _USB_DIEP0CTL_SNAK_MASK                    0x8000000UL                            /**< Bit mask for USB_SNAK */
#define _USB_DIEP0CTL_SNAK_DEFAULT                 0x00000000UL                           /**< Mode DEFAULT for USB_DIEP0CTL */
#define USB_DIEP0CTL_SNAK_DEFAULT                  (_USB_DIEP0CTL_SNAK_DEFAULT << 27)     /**< Shifted mode DEFAULT for USB_DIEP0CTL */
#define USB_DIEP0CTL_EPDIS                         (0x1UL << 30)                          /**< Endpoint Disable */
#define _USB_DIEP0CTL_EPDIS_SHIFT                  30                                     /**< Shift value for USB_EPDIS */
#define _USB_DIEP0CTL_EPDIS_MASK                   0x40000000UL                           /**< Bit mask for USB_EPDIS */
#define _USB_DIEP0CTL_EPDIS_DEFAULT                0x00000000UL                           /**< Mode DEFAULT for USB_DIEP0CTL */
#define USB_DIEP0CTL_EPDIS_DEFAULT                 (_USB_DIEP0CTL_EPDIS_DEFAULT << 30)    /**< Shifted mode DEFAULT for USB_DIEP0CTL */
#define USB_DIEP0CTL_EPENA                         (0x1UL << 31)                          /**< Endpoint Enable */
#define _USB_DIEP0CTL_EPENA_SHIFT                  31                                     /**< Shift value for USB_EPENA */
#define _USB_DIEP0CTL_EPENA_MASK                   0x80000000UL                           /**< Bit mask for USB_EPENA */
#define _USB_DIEP0CTL_EPENA_DEFAULT                0x00000000UL                           /**< Mode DEFAULT for USB_DIEP0CTL */
#define USB_DIEP0CTL_EPENA_DEFAULT                 (_USB_DIEP0CTL_EPENA_DEFAULT << 31)    /**< Shifted mode DEFAULT for USB_DIEP0CTL */

/* Bit fields for USB DIEP0INT */
#define _USB_DIEP0INT_RESETVALUE                   0x00000080UL                             /**< Default value for USB_DIEP0INT */
#define _USB_DIEP0INT_MASK                         0x000038DFUL                             /**< Mask for USB_DIEP0INT */
#define USB_DIEP0INT_XFERCOMPL                     (0x1UL << 0)                             /**< Transfer Completed Interrupt */
#define _USB_DIEP0INT_XFERCOMPL_SHIFT              0                                        /**< Shift value for USB_XFERCOMPL */
#define _USB_DIEP0INT_XFERCOMPL_MASK               0x1UL                                    /**< Bit mask for USB_XFERCOMPL */
#define _USB_DIEP0INT_XFERCOMPL_DEFAULT            0x00000000UL                             /**< Mode DEFAULT for USB_DIEP0INT */
#define USB_DIEP0INT_XFERCOMPL_DEFAULT             (_USB_DIEP0INT_XFERCOMPL_DEFAULT << 0)   /**< Shifted mode DEFAULT for USB_DIEP0INT */
#define USB_DIEP0INT_EPDISBLD                      (0x1UL << 1)                             /**< Endpoint Disabled Interrupt */
#define _USB_DIEP0INT_EPDISBLD_SHIFT               1                                        /**< Shift value for USB_EPDISBLD */
#define _USB_DIEP0INT_EPDISBLD_MASK                0x2UL                                    /**< Bit mask for USB_EPDISBLD */
#define _USB_DIEP0INT_EPDISBLD_DEFAULT             0x00000000UL                             /**< Mode DEFAULT for USB_DIEP0INT */
#define USB_DIEP0INT_EPDISBLD_DEFAULT              (_USB_DIEP0INT_EPDISBLD_DEFAULT << 1)    /**< Shifted mode DEFAULT for USB_DIEP0INT */
#define USB_DIEP0INT_AHBERR                        (0x1UL << 2)                             /**< AHB Error */
#define _USB_DIEP0INT_AHBERR_SHIFT                 2                                        /**< Shift value for USB_AHBERR */
#define _USB_DIEP0INT_AHBERR_MASK                  0x4UL                                    /**< Bit mask for USB_AHBERR */
#define _USB_DIEP0INT_AHBERR_DEFAULT               0x00000000UL                             /**< Mode DEFAULT for USB_DIEP0INT */
#define USB_DIEP0INT_AHBERR_DEFAULT                (_USB_DIEP0INT_AHBERR_DEFAULT << 2)      /**< Shifted mode DEFAULT for USB_DIEP0INT */
#define USB_DIEP0INT_TIMEOUT                       (0x1UL << 3)                             /**< Timeout Condition */
#define _USB_DIEP0INT_TIMEOUT_SHIFT                3                                        /**< Shift value for USB_TIMEOUT */
#define _USB_DIEP0INT_TIMEOUT_MASK                 0x8UL                                    /**< Bit mask for USB_TIMEOUT */
#define _USB_DIEP0INT_TIMEOUT_DEFAULT              0x00000000UL                             /**< Mode DEFAULT for USB_DIEP0INT */
#define USB_DIEP0INT_TIMEOUT_DEFAULT               (_USB_DIEP0INT_TIMEOUT_DEFAULT << 3)     /**< Shifted mode DEFAULT for USB_DIEP0INT */
#define USB_DIEP0INT_INTKNTXFEMP                   (0x1UL << 4)                             /**< IN Token Received When TxFIFO is Empty */
#define _USB_DIEP0INT_INTKNTXFEMP_SHIFT            4                                        /**< Shift value for USB_INTKNTXFEMP */
#define _USB_DIEP0INT_INTKNTXFEMP_MASK             0x10UL                                   /**< Bit mask for USB_INTKNTXFEMP */
#define _USB_DIEP0INT_INTKNTXFEMP_DEFAULT          0x00000000UL                             /**< Mode DEFAULT for USB_DIEP0INT */
#define USB_DIEP0INT_INTKNTXFEMP_DEFAULT           (_USB_DIEP0INT_INTKNTXFEMP_DEFAULT << 4) /**< Shifted mode DEFAULT for USB_DIEP0INT */
#define USB_DIEP0INT_INEPNAKEFF                    (0x1UL << 6)                             /**< IN Endpoint NAK Effective */
#define _USB_DIEP0INT_INEPNAKEFF_SHIFT             6                                        /**< Shift value for USB_INEPNAKEFF */
#define _USB_DIEP0INT_INEPNAKEFF_MASK              0x40UL                                   /**< Bit mask for USB_INEPNAKEFF */
#define _USB_DIEP0INT_INEPNAKEFF_DEFAULT           0x00000000UL                             /**< Mode DEFAULT for USB_DIEP0INT */
#define USB_DIEP0INT_INEPNAKEFF_DEFAULT            (_USB_DIEP0INT_INEPNAKEFF_DEFAULT << 6)  /**< Shifted mode DEFAULT for USB_DIEP0INT */
#define USB_DIEP0INT_TXFEMP                        (0x1UL << 7)                             /**< Transmit FIFO Empty */
#define _USB_DIEP0INT_TXFEMP_SHIFT                 7                                        /**< Shift value for USB_TXFEMP */
#define _USB_DIEP0INT_TXFEMP_MASK                  0x80UL                                   /**< Bit mask for USB_TXFEMP */
#define _USB_DIEP0INT_TXFEMP_DEFAULT               0x00000001UL                             /**< Mode DEFAULT for USB_DIEP0INT */
#define USB_DIEP0INT_TXFEMP_DEFAULT                (_USB_DIEP0INT_TXFEMP_DEFAULT << 7)      /**< Shifted mode DEFAULT for USB_DIEP0INT */
#define USB_DIEP0INT_PKTDRPSTS                     (0x1UL << 11)                            /**< Packet Drop Status */
#define _USB_DIEP0INT_PKTDRPSTS_SHIFT              11                                       /**< Shift value for USB_PKTDRPSTS */
#define _USB_DIEP0INT_PKTDRPSTS_MASK               0x800UL                                  /**< Bit mask for USB_PKTDRPSTS */
#define _USB_DIEP0INT_PKTDRPSTS_DEFAULT            0x00000000UL                             /**< Mode DEFAULT for USB_DIEP0INT */
#define USB_DIEP0INT_PKTDRPSTS_DEFAULT             (_USB_DIEP0INT_PKTDRPSTS_DEFAULT << 11)  /**< Shifted mode DEFAULT for USB_DIEP0INT */
#define USB_DIEP0INT_BBLEERR                       (0x1UL << 12)                            /**< NAK Interrupt */
#define _USB_DIEP0INT_BBLEERR_SHIFT                12                                       /**< Shift value for USB_BBLEERR */
#define _USB_DIEP0INT_BBLEERR_MASK                 0x1000UL                                 /**< Bit mask for USB_BBLEERR */
#define _USB_DIEP0INT_BBLEERR_DEFAULT              0x00000000UL                             /**< Mode DEFAULT for USB_DIEP0INT */
#define USB_DIEP0INT_BBLEERR_DEFAULT               (_USB_DIEP0INT_BBLEERR_DEFAULT << 12)    /**< Shifted mode DEFAULT for USB_DIEP0INT */
#define USB_DIEP0INT_NAKINTRPT                     (0x1UL << 13)                            /**< NAK Interrupt */
#define _USB_DIEP0INT_NAKINTRPT_SHIFT              13                                       /**< Shift value for USB_NAKINTRPT */
#define _USB_DIEP0INT_NAKINTRPT_MASK               0x2000UL                                 /**< Bit mask for USB_NAKINTRPT */
#define _USB_DIEP0INT_NAKINTRPT_DEFAULT            0x00000000UL                             /**< Mode DEFAULT for USB_DIEP0INT */
#define USB_DIEP0INT_NAKINTRPT_DEFAULT             (_USB_DIEP0INT_NAKINTRPT_DEFAULT << 13)  /**< Shifted mode DEFAULT for USB_DIEP0INT */

/* Bit fields for USB DIEP0TSIZ */
#define _USB_DIEP0TSIZ_RESETVALUE                  0x00000000UL                           /**< Default value for USB_DIEP0TSIZ */
#define _USB_DIEP0TSIZ_MASK                        0x0018007FUL                           /**< Mask for USB_DIEP0TSIZ */
#define _USB_DIEP0TSIZ_XFERSIZE_SHIFT              0                                      /**< Shift value for USB_XFERSIZE */
#define _USB_DIEP0TSIZ_XFERSIZE_MASK               0x7FUL                                 /**< Bit mask for USB_XFERSIZE */
#define _USB_DIEP0TSIZ_XFERSIZE_DEFAULT            0x00000000UL                           /**< Mode DEFAULT for USB_DIEP0TSIZ */
#define USB_DIEP0TSIZ_XFERSIZE_DEFAULT             (_USB_DIEP0TSIZ_XFERSIZE_DEFAULT << 0) /**< Shifted mode DEFAULT for USB_DIEP0TSIZ */
#define _USB_DIEP0TSIZ_PKTCNT_SHIFT                19                                     /**< Shift value for USB_PKTCNT */
#define _USB_DIEP0TSIZ_PKTCNT_MASK                 0x180000UL                             /**< Bit mask for USB_PKTCNT */
#define _USB_DIEP0TSIZ_PKTCNT_DEFAULT              0x00000000UL                           /**< Mode DEFAULT for USB_DIEP0TSIZ */
#define USB_DIEP0TSIZ_PKTCNT_DEFAULT               (_USB_DIEP0TSIZ_PKTCNT_DEFAULT << 19)  /**< Shifted mode DEFAULT for USB_DIEP0TSIZ */

/* Bit fields for USB DIEP0DMAADDR */
#define _USB_DIEP0DMAADDR_RESETVALUE               0x00000000UL                                  /**< Default value for USB_DIEP0DMAADDR */
#define _USB_DIEP0DMAADDR_MASK                     0xFFFFFFFFUL                                  /**< Mask for USB_DIEP0DMAADDR */
#define _USB_DIEP0DMAADDR_DIEP0DMAADDR_SHIFT       0                                             /**< Shift value for USB_DIEP0DMAADDR */
#define _USB_DIEP0DMAADDR_DIEP0DMAADDR_MASK        0xFFFFFFFFUL                                  /**< Bit mask for USB_DIEP0DMAADDR */
#define _USB_DIEP0DMAADDR_DIEP0DMAADDR_DEFAULT     0x00000000UL                                  /**< Mode DEFAULT for USB_DIEP0DMAADDR */
#define USB_DIEP0DMAADDR_DIEP0DMAADDR_DEFAULT      (_USB_DIEP0DMAADDR_DIEP0DMAADDR_DEFAULT << 0) /**< Shifted mode DEFAULT for USB_DIEP0DMAADDR */

/* Bit fields for USB DIEP0TXFSTS */
#define _USB_DIEP0TXFSTS_RESETVALUE                0x00000200UL                             /**< Default value for USB_DIEP0TXFSTS */
#define _USB_DIEP0TXFSTS_MASK                      0x0000FFFFUL                             /**< Mask for USB_DIEP0TXFSTS */
#define _USB_DIEP0TXFSTS_SPCAVAIL_SHIFT            0                                        /**< Shift value for USB_SPCAVAIL */
#define _USB_DIEP0TXFSTS_SPCAVAIL_MASK             0xFFFFUL                                 /**< Bit mask for USB_SPCAVAIL */
#define _USB_DIEP0TXFSTS_SPCAVAIL_DEFAULT          0x00000200UL                             /**< Mode DEFAULT for USB_DIEP0TXFSTS */
#define USB_DIEP0TXFSTS_SPCAVAIL_DEFAULT           (_USB_DIEP0TXFSTS_SPCAVAIL_DEFAULT << 0) /**< Shifted mode DEFAULT for USB_DIEP0TXFSTS */

/* Bit fields for USB DIEP_CTL */
#define _USB_DIEP_CTL_RESETVALUE                   0x00000000UL                             /**< Default value for USB_DIEP_CTL */
#define _USB_DIEP_CTL_MASK                         0xFFEF87FFUL                             /**< Mask for USB_DIEP_CTL */
#define _USB_DIEP_CTL_MPS_SHIFT                    0                                        /**< Shift value for USB_MPS */
#define _USB_DIEP_CTL_MPS_MASK                     0x7FFUL                                  /**< Bit mask for USB_MPS */
#define _USB_DIEP_CTL_MPS_DEFAULT                  0x00000000UL                             /**< Mode DEFAULT for USB_DIEP_CTL */
#define USB_DIEP_CTL_MPS_DEFAULT                   (_USB_DIEP_CTL_MPS_DEFAULT << 0)         /**< Shifted mode DEFAULT for USB_DIEP_CTL */
#define USB_DIEP_CTL_USBACTEP                      (0x1UL << 15)                            /**< USB Active Endpoint */
#define _USB_DIEP_CTL_USBACTEP_SHIFT               15                                       /**< Shift value for USB_USBACTEP */
#define _USB_DIEP_CTL_USBACTEP_MASK                0x8000UL                                 /**< Bit mask for USB_USBACTEP */
#define _USB_DIEP_CTL_USBACTEP_DEFAULT             0x00000000UL                             /**< Mode DEFAULT for USB_DIEP_CTL */
#define USB_DIEP_CTL_USBACTEP_DEFAULT              (_USB_DIEP_CTL_USBACTEP_DEFAULT << 15)   /**< Shifted mode DEFAULT for USB_DIEP_CTL */
#define USB_DIEP_CTL_DPIDEOF                       (0x1UL << 16)                            /**< Endpoint Data PID / Even or Odd Frame */
#define _USB_DIEP_CTL_DPIDEOF_SHIFT                16                                       /**< Shift value for USB_DPIDEOF */
#define _USB_DIEP_CTL_DPIDEOF_MASK                 0x10000UL                                /**< Bit mask for USB_DPIDEOF */
#define _USB_DIEP_CTL_DPIDEOF_DEFAULT              0x00000000UL                             /**< Mode DEFAULT for USB_DIEP_CTL */
#define _USB_DIEP_CTL_DPIDEOF_DATA0EVEN            0x00000000UL                             /**< Mode DATA0EVEN for USB_DIEP_CTL */
#define _USB_DIEP_CTL_DPIDEOF_DATA1ODD             0x00000001UL                             /**< Mode DATA1ODD for USB_DIEP_CTL */
#define USB_DIEP_CTL_DPIDEOF_DEFAULT               (_USB_DIEP_CTL_DPIDEOF_DEFAULT << 16)    /**< Shifted mode DEFAULT for USB_DIEP_CTL */
#define USB_DIEP_CTL_DPIDEOF_DATA0EVEN             (_USB_DIEP_CTL_DPIDEOF_DATA0EVEN << 16)  /**< Shifted mode DATA0EVEN for USB_DIEP_CTL */
#define USB_DIEP_CTL_DPIDEOF_DATA1ODD              (_USB_DIEP_CTL_DPIDEOF_DATA1ODD << 16)   /**< Shifted mode DATA1ODD for USB_DIEP_CTL */
#define USB_DIEP_CTL_NAKSTS                        (0x1UL << 17)                            /**< NAK Status */
#define _USB_DIEP_CTL_NAKSTS_SHIFT                 17                                       /**< Shift value for USB_NAKSTS */
#define _USB_DIEP_CTL_NAKSTS_MASK                  0x20000UL                                /**< Bit mask for USB_NAKSTS */
#define _USB_DIEP_CTL_NAKSTS_DEFAULT               0x00000000UL                             /**< Mode DEFAULT for USB_DIEP_CTL */
#define USB_DIEP_CTL_NAKSTS_DEFAULT                (_USB_DIEP_CTL_NAKSTS_DEFAULT << 17)     /**< Shifted mode DEFAULT for USB_DIEP_CTL */
#define _USB_DIEP_CTL_EPTYPE_SHIFT                 18                                       /**< Shift value for USB_EPTYPE */
#define _USB_DIEP_CTL_EPTYPE_MASK                  0xC0000UL                                /**< Bit mask for USB_EPTYPE */
#define _USB_DIEP_CTL_EPTYPE_DEFAULT               0x00000000UL                             /**< Mode DEFAULT for USB_DIEP_CTL */
#define _USB_DIEP_CTL_EPTYPE_CONTROL               0x00000000UL                             /**< Mode CONTROL for USB_DIEP_CTL */
#define _USB_DIEP_CTL_EPTYPE_ISO                   0x00000001UL                             /**< Mode ISO for USB_DIEP_CTL */
#define _USB_DIEP_CTL_EPTYPE_BULK                  0x00000002UL                             /**< Mode BULK for USB_DIEP_CTL */
#define _USB_DIEP_CTL_EPTYPE_INT                   0x00000003UL                             /**< Mode INT for USB_DIEP_CTL */
#define USB_DIEP_CTL_EPTYPE_DEFAULT                (_USB_DIEP_CTL_EPTYPE_DEFAULT << 18)     /**< Shifted mode DEFAULT for USB_DIEP_CTL */
#define USB_DIEP_CTL_EPTYPE_CONTROL                (_USB_DIEP_CTL_EPTYPE_CONTROL << 18)     /**< Shifted mode CONTROL for USB_DIEP_CTL */
#define USB_DIEP_CTL_EPTYPE_ISO                    (_USB_DIEP_CTL_EPTYPE_ISO << 18)         /**< Shifted mode ISO for USB_DIEP_CTL */
#define USB_DIEP_CTL_EPTYPE_BULK                   (_USB_DIEP_CTL_EPTYPE_BULK << 18)        /**< Shifted mode BULK for USB_DIEP_CTL */
#define USB_DIEP_CTL_EPTYPE_INT                    (_USB_DIEP_CTL_EPTYPE_INT << 18)         /**< Shifted mode INT for USB_DIEP_CTL */
#define USB_DIEP_CTL_STALL                         (0x1UL << 21)                            /**< Handshake */
#define _USB_DIEP_CTL_STALL_SHIFT                  21                                       /**< Shift value for USB_STALL */
#define _USB_DIEP_CTL_STALL_MASK                   0x200000UL                               /**< Bit mask for USB_STALL */
#define _USB_DIEP_CTL_STALL_DEFAULT                0x00000000UL                             /**< Mode DEFAULT for USB_DIEP_CTL */
#define USB_DIEP_CTL_STALL_DEFAULT                 (_USB_DIEP_CTL_STALL_DEFAULT << 21)      /**< Shifted mode DEFAULT for USB_DIEP_CTL */
#define _USB_DIEP_CTL_TXFNUM_SHIFT                 22                                       /**< Shift value for USB_TXFNUM */
#define _USB_DIEP_CTL_TXFNUM_MASK                  0x3C00000UL                              /**< Bit mask for USB_TXFNUM */
#define _USB_DIEP_CTL_TXFNUM_DEFAULT               0x00000000UL                             /**< Mode DEFAULT for USB_DIEP_CTL */
#define USB_DIEP_CTL_TXFNUM_DEFAULT                (_USB_DIEP_CTL_TXFNUM_DEFAULT << 22)     /**< Shifted mode DEFAULT for USB_DIEP_CTL */
#define USB_DIEP_CTL_CNAK                          (0x1UL << 26)                            /**< Clear NAK */
#define _USB_DIEP_CTL_CNAK_SHIFT                   26                                       /**< Shift value for USB_CNAK */
#define _USB_DIEP_CTL_CNAK_MASK                    0x4000000UL                              /**< Bit mask for USB_CNAK */
#define _USB_DIEP_CTL_CNAK_DEFAULT                 0x00000000UL                             /**< Mode DEFAULT for USB_DIEP_CTL */
#define USB_DIEP_CTL_CNAK_DEFAULT                  (_USB_DIEP_CTL_CNAK_DEFAULT << 26)       /**< Shifted mode DEFAULT for USB_DIEP_CTL */
#define USB_DIEP_CTL_SNAK                          (0x1UL << 27)                            /**< Set NAK */
#define _USB_DIEP_CTL_SNAK_SHIFT                   27                                       /**< Shift value for USB_SNAK */
#define _USB_DIEP_CTL_SNAK_MASK                    0x8000000UL                              /**< Bit mask for USB_SNAK */
#define _USB_DIEP_CTL_SNAK_DEFAULT                 0x00000000UL                             /**< Mode DEFAULT for USB_DIEP_CTL */
#define USB_DIEP_CTL_SNAK_DEFAULT                  (_USB_DIEP_CTL_SNAK_DEFAULT << 27)       /**< Shifted mode DEFAULT for USB_DIEP_CTL */
#define USB_DIEP_CTL_SETD0PIDEF                    (0x1UL << 28)                            /**< Set DATA0 PID / Even Frame */
#define _USB_DIEP_CTL_SETD0PIDEF_SHIFT             28                                       /**< Shift value for USB_SETD0PIDEF */
#define _USB_DIEP_CTL_SETD0PIDEF_MASK              0x10000000UL                             /**< Bit mask for USB_SETD0PIDEF */
#define _USB_DIEP_CTL_SETD0PIDEF_DEFAULT           0x00000000UL                             /**< Mode DEFAULT for USB_DIEP_CTL */
#define USB_DIEP_CTL_SETD0PIDEF_DEFAULT            (_USB_DIEP_CTL_SETD0PIDEF_DEFAULT << 28) /**< Shifted mode DEFAULT for USB_DIEP_CTL */
#define USB_DIEP_CTL_SETD1PIDOF                    (0x1UL << 29)                            /**< Set DATA1 PID / Odd Frame */
#define _USB_DIEP_CTL_SETD1PIDOF_SHIFT             29                                       /**< Shift value for USB_SETD1PIDOF */
#define _USB_DIEP_CTL_SETD1PIDOF_MASK              0x20000000UL                             /**< Bit mask for USB_SETD1PIDOF */
#define _USB_DIEP_CTL_SETD1PIDOF_DEFAULT           0x00000000UL                             /**< Mode DEFAULT for USB_DIEP_CTL */
#define USB_DIEP_CTL_SETD1PIDOF_DEFAULT            (_USB_DIEP_CTL_SETD1PIDOF_DEFAULT << 29) /**< Shifted mode DEFAULT for USB_DIEP_CTL */
#define USB_DIEP_CTL_EPDIS                         (0x1UL << 30)                            /**< Endpoint Disable */
#define _USB_DIEP_CTL_EPDIS_SHIFT                  30                                       /**< Shift value for USB_EPDIS */
#define _USB_DIEP_CTL_EPDIS_MASK                   0x40000000UL                             /**< Bit mask for USB_EPDIS */
#define _USB_DIEP_CTL_EPDIS_DEFAULT                0x00000000UL                             /**< Mode DEFAULT for USB_DIEP_CTL */
#define USB_DIEP_CTL_EPDIS_DEFAULT                 (_USB_DIEP_CTL_EPDIS_DEFAULT << 30)      /**< Shifted mode DEFAULT for USB_DIEP_CTL */
#define USB_DIEP_CTL_EPENA                         (0x1UL << 31)                            /**< Endpoint Enable */
#define _USB_DIEP_CTL_EPENA_SHIFT                  31                                       /**< Shift value for USB_EPENA */
#define _USB_DIEP_CTL_EPENA_MASK                   0x80000000UL                             /**< Bit mask for USB_EPENA */
#define _USB_DIEP_CTL_EPENA_DEFAULT                0x00000000UL                             /**< Mode DEFAULT for USB_DIEP_CTL */
#define USB_DIEP_CTL_EPENA_DEFAULT                 (_USB_DIEP_CTL_EPENA_DEFAULT << 31)      /**< Shifted mode DEFAULT for USB_DIEP_CTL */

/* Bit fields for USB DIEP_INT */
#define _USB_DIEP_INT_RESETVALUE                   0x00000080UL                             /**< Default value for USB_DIEP_INT */
#define _USB_DIEP_INT_MASK                         0x000038DFUL                             /**< Mask for USB_DIEP_INT */
#define USB_DIEP_INT_XFERCOMPL                     (0x1UL << 0)                             /**< Transfer Completed Interrupt */
#define _USB_DIEP_INT_XFERCOMPL_SHIFT              0                                        /**< Shift value for USB_XFERCOMPL */
#define _USB_DIEP_INT_XFERCOMPL_MASK               0x1UL                                    /**< Bit mask for USB_XFERCOMPL */
#define _USB_DIEP_INT_XFERCOMPL_DEFAULT            0x00000000UL                             /**< Mode DEFAULT for USB_DIEP_INT */
#define USB_DIEP_INT_XFERCOMPL_DEFAULT             (_USB_DIEP_INT_XFERCOMPL_DEFAULT << 0)   /**< Shifted mode DEFAULT for USB_DIEP_INT */
#define USB_DIEP_INT_EPDISBLD                      (0x1UL << 1)                             /**< Endpoint Disabled Interrupt */
#define _USB_DIEP_INT_EPDISBLD_SHIFT               1                                        /**< Shift value for USB_EPDISBLD */
#define _USB_DIEP_INT_EPDISBLD_MASK                0x2UL                                    /**< Bit mask for USB_EPDISBLD */
#define _USB_DIEP_INT_EPDISBLD_DEFAULT             0x00000000UL                             /**< Mode DEFAULT for USB_DIEP_INT */
#define USB_DIEP_INT_EPDISBLD_DEFAULT              (_USB_DIEP_INT_EPDISBLD_DEFAULT << 1)    /**< Shifted mode DEFAULT for USB_DIEP_INT */
#define USB_DIEP_INT_AHBERR                        (0x1UL << 2)                             /**< AHB Error */
#define _USB_DIEP_INT_AHBERR_SHIFT                 2                                        /**< Shift value for USB_AHBERR */
#define _USB_DIEP_INT_AHBERR_MASK                  0x4UL                                    /**< Bit mask for USB_AHBERR */
#define _USB_DIEP_INT_AHBERR_DEFAULT               0x00000000UL                             /**< Mode DEFAULT for USB_DIEP_INT */
#define USB_DIEP_INT_AHBERR_DEFAULT                (_USB_DIEP_INT_AHBERR_DEFAULT << 2)      /**< Shifted mode DEFAULT for USB_DIEP_INT */
#define USB_DIEP_INT_TIMEOUT                       (0x1UL << 3)                             /**< Timeout Condition */
#define _USB_DIEP_INT_TIMEOUT_SHIFT                3                                        /**< Shift value for USB_TIMEOUT */
#define _USB_DIEP_INT_TIMEOUT_MASK                 0x8UL                                    /**< Bit mask for USB_TIMEOUT */
#define _USB_DIEP_INT_TIMEOUT_DEFAULT              0x00000000UL                             /**< Mode DEFAULT for USB_DIEP_INT */
#define USB_DIEP_INT_TIMEOUT_DEFAULT               (_USB_DIEP_INT_TIMEOUT_DEFAULT << 3)     /**< Shifted mode DEFAULT for USB_DIEP_INT */
#define USB_DIEP_INT_INTKNTXFEMP                   (0x1UL << 4)                             /**< IN Token Received When TxFIFO is Empty */
#define _USB_DIEP_INT_INTKNTXFEMP_SHIFT            4                                        /**< Shift value for USB_INTKNTXFEMP */
#define _USB_DIEP_INT_INTKNTXFEMP_MASK             0x10UL                                   /**< Bit mask for USB_INTKNTXFEMP */
#define _USB_DIEP_INT_INTKNTXFEMP_DEFAULT          0x00000000UL                             /**< Mode DEFAULT for USB_DIEP_INT */
#define USB_DIEP_INT_INTKNTXFEMP_DEFAULT           (_USB_DIEP_INT_INTKNTXFEMP_DEFAULT << 4) /**< Shifted mode DEFAULT for USB_DIEP_INT */
#define USB_DIEP_INT_INEPNAKEFF                    (0x1UL << 6)                             /**< IN Endpoint NAK Effective */
#define _USB_DIEP_INT_INEPNAKEFF_SHIFT             6                                        /**< Shift value for USB_INEPNAKEFF */
#define _USB_DIEP_INT_INEPNAKEFF_MASK              0x40UL                                   /**< Bit mask for USB_INEPNAKEFF */
#define _USB_DIEP_INT_INEPNAKEFF_DEFAULT           0x00000000UL                             /**< Mode DEFAULT for USB_DIEP_INT */
#define USB_DIEP_INT_INEPNAKEFF_DEFAULT            (_USB_DIEP_INT_INEPNAKEFF_DEFAULT << 6)  /**< Shifted mode DEFAULT for USB_DIEP_INT */
#define USB_DIEP_INT_TXFEMP                        (0x1UL << 7)                             /**< Transmit FIFO Empty */
#define _USB_DIEP_INT_TXFEMP_SHIFT                 7                                        /**< Shift value for USB_TXFEMP */
#define _USB_DIEP_INT_TXFEMP_MASK                  0x80UL                                   /**< Bit mask for USB_TXFEMP */
#define _USB_DIEP_INT_TXFEMP_DEFAULT               0x00000001UL                             /**< Mode DEFAULT for USB_DIEP_INT */
#define USB_DIEP_INT_TXFEMP_DEFAULT                (_USB_DIEP_INT_TXFEMP_DEFAULT << 7)      /**< Shifted mode DEFAULT for USB_DIEP_INT */
#define USB_DIEP_INT_PKTDRPSTS                     (0x1UL << 11)                            /**< Packet Drop Status */
#define _USB_DIEP_INT_PKTDRPSTS_SHIFT              11                                       /**< Shift value for USB_PKTDRPSTS */
#define _USB_DIEP_INT_PKTDRPSTS_MASK               0x800UL                                  /**< Bit mask for USB_PKTDRPSTS */
#define _USB_DIEP_INT_PKTDRPSTS_DEFAULT            0x00000000UL                             /**< Mode DEFAULT for USB_DIEP_INT */
#define USB_DIEP_INT_PKTDRPSTS_DEFAULT             (_USB_DIEP_INT_PKTDRPSTS_DEFAULT << 11)  /**< Shifted mode DEFAULT for USB_DIEP_INT */
#define USB_DIEP_INT_BBLEERR                       (0x1UL << 12)                            /**< NAK Interrupt */
#define _USB_DIEP_INT_BBLEERR_SHIFT                12                                       /**< Shift value for USB_BBLEERR */
#define _USB_DIEP_INT_BBLEERR_MASK                 0x1000UL                                 /**< Bit mask for USB_BBLEERR */
#define _USB_DIEP_INT_BBLEERR_DEFAULT              0x00000000UL                             /**< Mode DEFAULT for USB_DIEP_INT */
#define USB_DIEP_INT_BBLEERR_DEFAULT               (_USB_DIEP_INT_BBLEERR_DEFAULT << 12)    /**< Shifted mode DEFAULT for USB_DIEP_INT */
#define USB_DIEP_INT_NAKINTRPT                     (0x1UL << 13)                            /**< NAK Interrupt */
#define _USB_DIEP_INT_NAKINTRPT_SHIFT              13                                       /**< Shift value for USB_NAKINTRPT */
#define _USB_DIEP_INT_NAKINTRPT_MASK               0x2000UL                                 /**< Bit mask for USB_NAKINTRPT */
#define _USB_DIEP_INT_NAKINTRPT_DEFAULT            0x00000000UL                             /**< Mode DEFAULT for USB_DIEP_INT */
#define USB_DIEP_INT_NAKINTRPT_DEFAULT             (_USB_DIEP_INT_NAKINTRPT_DEFAULT << 13)  /**< Shifted mode DEFAULT for USB_DIEP_INT */

/* Bit fields for USB DIEP_TSIZ */
#define _USB_DIEP_TSIZ_RESETVALUE                  0x00000000UL                           /**< Default value for USB_DIEP_TSIZ */
#define _USB_DIEP_TSIZ_MASK                        0x7FFFFFFFUL                           /**< Mask for USB_DIEP_TSIZ */
#define _USB_DIEP_TSIZ_XFERSIZE_SHIFT              0                                      /**< Shift value for USB_XFERSIZE */
#define _USB_DIEP_TSIZ_XFERSIZE_MASK               0x7FFFFUL                              /**< Bit mask for USB_XFERSIZE */
#define _USB_DIEP_TSIZ_XFERSIZE_DEFAULT            0x00000000UL                           /**< Mode DEFAULT for USB_DIEP_TSIZ */
#define USB_DIEP_TSIZ_XFERSIZE_DEFAULT             (_USB_DIEP_TSIZ_XFERSIZE_DEFAULT << 0) /**< Shifted mode DEFAULT for USB_DIEP_TSIZ */
#define _USB_DIEP_TSIZ_PKTCNT_SHIFT                19                                     /**< Shift value for USB_PKTCNT */
#define _USB_DIEP_TSIZ_PKTCNT_MASK                 0x1FF80000UL                           /**< Bit mask for USB_PKTCNT */
#define _USB_DIEP_TSIZ_PKTCNT_DEFAULT              0x00000000UL                           /**< Mode DEFAULT for USB_DIEP_TSIZ */
#define USB_DIEP_TSIZ_PKTCNT_DEFAULT               (_USB_DIEP_TSIZ_PKTCNT_DEFAULT << 19)  /**< Shifted mode DEFAULT for USB_DIEP_TSIZ */
#define _USB_DIEP_TSIZ_MC_SHIFT                    29                                     /**< Shift value for USB_MC */
#define _USB_DIEP_TSIZ_MC_MASK                     0x60000000UL                           /**< Bit mask for USB_MC */
#define _USB_DIEP_TSIZ_MC_DEFAULT                  0x00000000UL                           /**< Mode DEFAULT for USB_DIEP_TSIZ */
#define USB_DIEP_TSIZ_MC_DEFAULT                   (_USB_DIEP_TSIZ_MC_DEFAULT << 29)      /**< Shifted mode DEFAULT for USB_DIEP_TSIZ */

/* Bit fields for USB DIEP_DMAADDR */
#define _USB_DIEP_DMAADDR_RESETVALUE               0x00000000UL                             /**< Default value for USB_DIEP_DMAADDR */
#define _USB_DIEP_DMAADDR_MASK                     0xFFFFFFFFUL                             /**< Mask for USB_DIEP_DMAADDR */
#define _USB_DIEP_DMAADDR_DMAADDR_SHIFT            0                                        /**< Shift value for USB_DMAADDR */
#define _USB_DIEP_DMAADDR_DMAADDR_MASK             0xFFFFFFFFUL                             /**< Bit mask for USB_DMAADDR */
#define _USB_DIEP_DMAADDR_DMAADDR_DEFAULT          0x00000000UL                             /**< Mode DEFAULT for USB_DIEP_DMAADDR */
#define USB_DIEP_DMAADDR_DMAADDR_DEFAULT           (_USB_DIEP_DMAADDR_DMAADDR_DEFAULT << 0) /**< Shifted mode DEFAULT for USB_DIEP_DMAADDR */

/* Bit fields for USB DIEP_TXFSTS */
#define _USB_DIEP_TXFSTS_RESETVALUE                0x00000200UL                             /**< Default value for USB_DIEP_TXFSTS */
#define _USB_DIEP_TXFSTS_MASK                      0x0000FFFFUL                             /**< Mask for USB_DIEP_TXFSTS */
#define _USB_DIEP_TXFSTS_SPCAVAIL_SHIFT            0                                        /**< Shift value for USB_SPCAVAIL */
#define _USB_DIEP_TXFSTS_SPCAVAIL_MASK             0xFFFFUL                                 /**< Bit mask for USB_SPCAVAIL */
#define _USB_DIEP_TXFSTS_SPCAVAIL_DEFAULT          0x00000200UL                             /**< Mode DEFAULT for USB_DIEP_TXFSTS */
#define USB_DIEP_TXFSTS_SPCAVAIL_DEFAULT           (_USB_DIEP_TXFSTS_SPCAVAIL_DEFAULT << 0) /**< Shifted mode DEFAULT for USB_DIEP_TXFSTS */

/* Bit fields for USB DOEP0CTL */
#define _USB_DOEP0CTL_RESETVALUE                   0x00008000UL                           /**< Default value for USB_DOEP0CTL */
#define _USB_DOEP0CTL_MASK                         0xCC3E8003UL                           /**< Mask for USB_DOEP0CTL */
#define _USB_DOEP0CTL_MPS_SHIFT                    0                                      /**< Shift value for USB_MPS */
#define _USB_DOEP0CTL_MPS_MASK                     0x3UL                                  /**< Bit mask for USB_MPS */
#define _USB_DOEP0CTL_MPS_DEFAULT                  0x00000000UL                           /**< Mode DEFAULT for USB_DOEP0CTL */
#define _USB_DOEP0CTL_MPS_64B                      0x00000000UL                           /**< Mode 64B for USB_DOEP0CTL */
#define _USB_DOEP0CTL_MPS_32B                      0x00000001UL                           /**< Mode 32B for USB_DOEP0CTL */
#define _USB_DOEP0CTL_MPS_16B                      0x00000002UL                           /**< Mode 16B for USB_DOEP0CTL */
#define _USB_DOEP0CTL_MPS_8B                       0x00000003UL                           /**< Mode 8B for USB_DOEP0CTL */
#define USB_DOEP0CTL_MPS_DEFAULT                   (_USB_DOEP0CTL_MPS_DEFAULT << 0)       /**< Shifted mode DEFAULT for USB_DOEP0CTL */
#define USB_DOEP0CTL_MPS_64B                       (_USB_DOEP0CTL_MPS_64B << 0)           /**< Shifted mode 64B for USB_DOEP0CTL */
#define USB_DOEP0CTL_MPS_32B                       (_USB_DOEP0CTL_MPS_32B << 0)           /**< Shifted mode 32B for USB_DOEP0CTL */
#define USB_DOEP0CTL_MPS_16B                       (_USB_DOEP0CTL_MPS_16B << 0)           /**< Shifted mode 16B for USB_DOEP0CTL */
#define USB_DOEP0CTL_MPS_8B                        (_USB_DOEP0CTL_MPS_8B << 0)            /**< Shifted mode 8B for USB_DOEP0CTL */
#define USB_DOEP0CTL_USBACTEP                      (0x1UL << 15)                          /**< USB Active Endpoint */
#define _USB_DOEP0CTL_USBACTEP_SHIFT               15                                     /**< Shift value for USB_USBACTEP */
#define _USB_DOEP0CTL_USBACTEP_MASK                0x8000UL                               /**< Bit mask for USB_USBACTEP */
#define _USB_DOEP0CTL_USBACTEP_DEFAULT             0x00000001UL                           /**< Mode DEFAULT for USB_DOEP0CTL */
#define USB_DOEP0CTL_USBACTEP_DEFAULT              (_USB_DOEP0CTL_USBACTEP_DEFAULT << 15) /**< Shifted mode DEFAULT for USB_DOEP0CTL */
#define USB_DOEP0CTL_NAKSTS                        (0x1UL << 17)                          /**< NAK Status */
#define _USB_DOEP0CTL_NAKSTS_SHIFT                 17                                     /**< Shift value for USB_NAKSTS */
#define _USB_DOEP0CTL_NAKSTS_MASK                  0x20000UL                              /**< Bit mask for USB_NAKSTS */
#define _USB_DOEP0CTL_NAKSTS_DEFAULT               0x00000000UL                           /**< Mode DEFAULT for USB_DOEP0CTL */
#define USB_DOEP0CTL_NAKSTS_DEFAULT                (_USB_DOEP0CTL_NAKSTS_DEFAULT << 17)   /**< Shifted mode DEFAULT for USB_DOEP0CTL */
#define _USB_DOEP0CTL_EPTYPE_SHIFT                 18                                     /**< Shift value for USB_EPTYPE */
#define _USB_DOEP0CTL_EPTYPE_MASK                  0xC0000UL                              /**< Bit mask for USB_EPTYPE */
#define _USB_DOEP0CTL_EPTYPE_DEFAULT               0x00000000UL                           /**< Mode DEFAULT for USB_DOEP0CTL */
#define USB_DOEP0CTL_EPTYPE_DEFAULT                (_USB_DOEP0CTL_EPTYPE_DEFAULT << 18)   /**< Shifted mode DEFAULT for USB_DOEP0CTL */
#define USB_DOEP0CTL_SNP                           (0x1UL << 20)                          /**< Snoop Mode */
#define _USB_DOEP0CTL_SNP_SHIFT                    20                                     /**< Shift value for USB_SNP */
#define _USB_DOEP0CTL_SNP_MASK                     0x100000UL                             /**< Bit mask for USB_SNP */
#define _USB_DOEP0CTL_SNP_DEFAULT                  0x00000000UL                           /**< Mode DEFAULT for USB_DOEP0CTL */
#define USB_DOEP0CTL_SNP_DEFAULT                   (_USB_DOEP0CTL_SNP_DEFAULT << 20)      /**< Shifted mode DEFAULT for USB_DOEP0CTL */
#define USB_DOEP0CTL_STALL                         (0x1UL << 21)                          /**< Handshake */
#define _USB_DOEP0CTL_STALL_SHIFT                  21                                     /**< Shift value for USB_STALL */
#define _USB_DOEP0CTL_STALL_MASK                   0x200000UL                             /**< Bit mask for USB_STALL */
#define _USB_DOEP0CTL_STALL_DEFAULT                0x00000000UL                           /**< Mode DEFAULT for USB_DOEP0CTL */
#define USB_DOEP0CTL_STALL_DEFAULT                 (_USB_DOEP0CTL_STALL_DEFAULT << 21)    /**< Shifted mode DEFAULT for USB_DOEP0CTL */
#define USB_DOEP0CTL_CNAK                          (0x1UL << 26)                          /**< Clear NAK */
#define _USB_DOEP0CTL_CNAK_SHIFT                   26                                     /**< Shift value for USB_CNAK */
#define _USB_DOEP0CTL_CNAK_MASK                    0x4000000UL                            /**< Bit mask for USB_CNAK */
#define _USB_DOEP0CTL_CNAK_DEFAULT                 0x00000000UL                           /**< Mode DEFAULT for USB_DOEP0CTL */
#define USB_DOEP0CTL_CNAK_DEFAULT                  (_USB_DOEP0CTL_CNAK_DEFAULT << 26)     /**< Shifted mode DEFAULT for USB_DOEP0CTL */
#define USB_DOEP0CTL_SNAK                          (0x1UL << 27)                          /**< Set NAK */
#define _USB_DOEP0CTL_SNAK_SHIFT                   27                                     /**< Shift value for USB_SNAK */
#define _USB_DOEP0CTL_SNAK_MASK                    0x8000000UL                            /**< Bit mask for USB_SNAK */
#define _USB_DOEP0CTL_SNAK_DEFAULT                 0x00000000UL                           /**< Mode DEFAULT for USB_DOEP0CTL */
#define USB_DOEP0CTL_SNAK_DEFAULT                  (_USB_DOEP0CTL_SNAK_DEFAULT << 27)     /**< Shifted mode DEFAULT for USB_DOEP0CTL */
#define USB_DOEP0CTL_EPDIS                         (0x1UL << 30)                          /**< Endpoint Disable */
#define _USB_DOEP0CTL_EPDIS_SHIFT                  30                                     /**< Shift value for USB_EPDIS */
#define _USB_DOEP0CTL_EPDIS_MASK                   0x40000000UL                           /**< Bit mask for USB_EPDIS */
#define _USB_DOEP0CTL_EPDIS_DEFAULT                0x00000000UL                           /**< Mode DEFAULT for USB_DOEP0CTL */
#define USB_DOEP0CTL_EPDIS_DEFAULT                 (_USB_DOEP0CTL_EPDIS_DEFAULT << 30)    /**< Shifted mode DEFAULT for USB_DOEP0CTL */
#define USB_DOEP0CTL_EPENA                         (0x1UL << 31)                          /**< Endpoint Enable */
#define _USB_DOEP0CTL_EPENA_SHIFT                  31                                     /**< Shift value for USB_EPENA */
#define _USB_DOEP0CTL_EPENA_MASK                   0x80000000UL                           /**< Bit mask for USB_EPENA */
#define _USB_DOEP0CTL_EPENA_DEFAULT                0x00000000UL                           /**< Mode DEFAULT for USB_DOEP0CTL */
#define USB_DOEP0CTL_EPENA_DEFAULT                 (_USB_DOEP0CTL_EPENA_DEFAULT << 31)    /**< Shifted mode DEFAULT for USB_DOEP0CTL */

/* Bit fields for USB DOEP0INT */
#define _USB_DOEP0INT_RESETVALUE                   0x00000000UL                                /**< Default value for USB_DOEP0INT */
#define _USB_DOEP0INT_MASK                         0x0000B87FUL                                /**< Mask for USB_DOEP0INT */
#define USB_DOEP0INT_XFERCOMPL                     (0x1UL << 0)                                /**< Transfer Completed Interrupt */
#define _USB_DOEP0INT_XFERCOMPL_SHIFT              0                                           /**< Shift value for USB_XFERCOMPL */
#define _USB_DOEP0INT_XFERCOMPL_MASK               0x1UL                                       /**< Bit mask for USB_XFERCOMPL */
#define _USB_DOEP0INT_XFERCOMPL_DEFAULT            0x00000000UL                                /**< Mode DEFAULT for USB_DOEP0INT */
#define USB_DOEP0INT_XFERCOMPL_DEFAULT             (_USB_DOEP0INT_XFERCOMPL_DEFAULT << 0)      /**< Shifted mode DEFAULT for USB_DOEP0INT */
#define USB_DOEP0INT_EPDISBLD                      (0x1UL << 1)                                /**< Endpoint Disabled Interrupt */
#define _USB_DOEP0INT_EPDISBLD_SHIFT               1                                           /**< Shift value for USB_EPDISBLD */
#define _USB_DOEP0INT_EPDISBLD_MASK                0x2UL                                       /**< Bit mask for USB_EPDISBLD */
#define _USB_DOEP0INT_EPDISBLD_DEFAULT             0x00000000UL                                /**< Mode DEFAULT for USB_DOEP0INT */
#define USB_DOEP0INT_EPDISBLD_DEFAULT              (_USB_DOEP0INT_EPDISBLD_DEFAULT << 1)       /**< Shifted mode DEFAULT for USB_DOEP0INT */
#define USB_DOEP0INT_AHBERR                        (0x1UL << 2)                                /**< AHB Error */
#define _USB_DOEP0INT_AHBERR_SHIFT                 2                                           /**< Shift value for USB_AHBERR */
#define _USB_DOEP0INT_AHBERR_MASK                  0x4UL                                       /**< Bit mask for USB_AHBERR */
#define _USB_DOEP0INT_AHBERR_DEFAULT               0x00000000UL                                /**< Mode DEFAULT for USB_DOEP0INT */
#define USB_DOEP0INT_AHBERR_DEFAULT                (_USB_DOEP0INT_AHBERR_DEFAULT << 2)         /**< Shifted mode DEFAULT for USB_DOEP0INT */
#define USB_DOEP0INT_SETUP                         (0x1UL << 3)                                /**< Setup Phase Done */
#define _USB_DOEP0INT_SETUP_SHIFT                  3                                           /**< Shift value for USB_SETUP */
#define _USB_DOEP0INT_SETUP_MASK                   0x8UL                                       /**< Bit mask for USB_SETUP */
#define _USB_DOEP0INT_SETUP_DEFAULT                0x00000000UL                                /**< Mode DEFAULT for USB_DOEP0INT */
#define USB_DOEP0INT_SETUP_DEFAULT                 (_USB_DOEP0INT_SETUP_DEFAULT << 3)          /**< Shifted mode DEFAULT for USB_DOEP0INT */
#define USB_DOEP0INT_OUTTKNEPDIS                   (0x1UL << 4)                                /**< OUT Token Received When Endpoint Disabled */
#define _USB_DOEP0INT_OUTTKNEPDIS_SHIFT            4                                           /**< Shift value for USB_OUTTKNEPDIS */
#define _USB_DOEP0INT_OUTTKNEPDIS_MASK             0x10UL                                      /**< Bit mask for USB_OUTTKNEPDIS */
#define _USB_DOEP0INT_OUTTKNEPDIS_DEFAULT          0x00000000UL                                /**< Mode DEFAULT for USB_DOEP0INT */
#define USB_DOEP0INT_OUTTKNEPDIS_DEFAULT           (_USB_DOEP0INT_OUTTKNEPDIS_DEFAULT << 4)    /**< Shifted mode DEFAULT for USB_DOEP0INT */
#define USB_DOEP0INT_STSPHSERCVD                   (0x1UL << 5)                                /**< Status Phase Received For Control Write */
#define _USB_DOEP0INT_STSPHSERCVD_SHIFT            5                                           /**< Shift value for USB_STSPHSERCVD */
#define _USB_DOEP0INT_STSPHSERCVD_MASK             0x20UL                                      /**< Bit mask for USB_STSPHSERCVD */
#define _USB_DOEP0INT_STSPHSERCVD_DEFAULT          0x00000000UL                                /**< Mode DEFAULT for USB_DOEP0INT */
#define USB_DOEP0INT_STSPHSERCVD_DEFAULT           (_USB_DOEP0INT_STSPHSERCVD_DEFAULT << 5)    /**< Shifted mode DEFAULT for USB_DOEP0INT */
#define USB_DOEP0INT_BACK2BACKSETUP                (0x1UL << 6)                                /**< Back-to-Back SETUP Packets Received */
#define _USB_DOEP0INT_BACK2BACKSETUP_SHIFT         6                                           /**< Shift value for USB_BACK2BACKSETUP */
#define _USB_DOEP0INT_BACK2BACKSETUP_MASK          0x40UL                                      /**< Bit mask for USB_BACK2BACKSETUP */
#define _USB_DOEP0INT_BACK2BACKSETUP_DEFAULT       0x00000000UL                                /**< Mode DEFAULT for USB_DOEP0INT */
#define USB_DOEP0INT_BACK2BACKSETUP_DEFAULT        (_USB_DOEP0INT_BACK2BACKSETUP_DEFAULT << 6) /**< Shifted mode DEFAULT for USB_DOEP0INT */
#define USB_DOEP0INT_PKTDRPSTS                     (0x1UL << 11)                               /**< Packet Drop Status */
#define _USB_DOEP0INT_PKTDRPSTS_SHIFT              11                                          /**< Shift value for USB_PKTDRPSTS */
#define _USB_DOEP0INT_PKTDRPSTS_MASK               0x800UL                                     /**< Bit mask for USB_PKTDRPSTS */
#define _USB_DOEP0INT_PKTDRPSTS_DEFAULT            0x00000000UL                                /**< Mode DEFAULT for USB_DOEP0INT */
#define USB_DOEP0INT_PKTDRPSTS_DEFAULT             (_USB_DOEP0INT_PKTDRPSTS_DEFAULT << 11)     /**< Shifted mode DEFAULT for USB_DOEP0INT */
#define USB_DOEP0INT_BBLEERR                       (0x1UL << 12)                               /**< NAK Interrupt */
#define _USB_DOEP0INT_BBLEERR_SHIFT                12                                          /**< Shift value for USB_BBLEERR */
#define _USB_DOEP0INT_BBLEERR_MASK                 0x1000UL                                    /**< Bit mask for USB_BBLEERR */
#define _USB_DOEP0INT_BBLEERR_DEFAULT              0x00000000UL                                /**< Mode DEFAULT for USB_DOEP0INT */
#define USB_DOEP0INT_BBLEERR_DEFAULT               (_USB_DOEP0INT_BBLEERR_DEFAULT << 12)       /**< Shifted mode DEFAULT for USB_DOEP0INT */
#define USB_DOEP0INT_NAKINTRPT                     (0x1UL << 13)                               /**< NAK Interrupt */
#define _USB_DOEP0INT_NAKINTRPT_SHIFT              13                                          /**< Shift value for USB_NAKINTRPT */
#define _USB_DOEP0INT_NAKINTRPT_MASK               0x2000UL                                    /**< Bit mask for USB_NAKINTRPT */
#define _USB_DOEP0INT_NAKINTRPT_DEFAULT            0x00000000UL                                /**< Mode DEFAULT for USB_DOEP0INT */
#define USB_DOEP0INT_NAKINTRPT_DEFAULT             (_USB_DOEP0INT_NAKINTRPT_DEFAULT << 13)     /**< Shifted mode DEFAULT for USB_DOEP0INT */
#define USB_DOEP0INT_STUPPKTRCVD                   (0x1UL << 15)                               /**< Setup Packet Received */
#define _USB_DOEP0INT_STUPPKTRCVD_SHIFT            15                                          /**< Shift value for USB_STUPPKTRCVD */
#define _USB_DOEP0INT_STUPPKTRCVD_MASK             0x8000UL                                    /**< Bit mask for USB_STUPPKTRCVD */
#define _USB_DOEP0INT_STUPPKTRCVD_DEFAULT          0x00000000UL                                /**< Mode DEFAULT for USB_DOEP0INT */
#define USB_DOEP0INT_STUPPKTRCVD_DEFAULT           (_USB_DOEP0INT_STUPPKTRCVD_DEFAULT << 15)   /**< Shifted mode DEFAULT for USB_DOEP0INT */

/* Bit fields for USB DOEP0TSIZ */
#define _USB_DOEP0TSIZ_RESETVALUE                  0x00000000UL                           /**< Default value for USB_DOEP0TSIZ */
#define _USB_DOEP0TSIZ_MASK                        0x6008007FUL                           /**< Mask for USB_DOEP0TSIZ */
#define _USB_DOEP0TSIZ_XFERSIZE_SHIFT              0                                      /**< Shift value for USB_XFERSIZE */
#define _USB_DOEP0TSIZ_XFERSIZE_MASK               0x7FUL                                 /**< Bit mask for USB_XFERSIZE */
#define _USB_DOEP0TSIZ_XFERSIZE_DEFAULT            0x00000000UL                           /**< Mode DEFAULT for USB_DOEP0TSIZ */
#define USB_DOEP0TSIZ_XFERSIZE_DEFAULT             (_USB_DOEP0TSIZ_XFERSIZE_DEFAULT << 0) /**< Shifted mode DEFAULT for USB_DOEP0TSIZ */
#define USB_DOEP0TSIZ_PKTCNT                       (0x1UL << 19)                          /**< Packet Count */
#define _USB_DOEP0TSIZ_PKTCNT_SHIFT                19                                     /**< Shift value for USB_PKTCNT */
#define _USB_DOEP0TSIZ_PKTCNT_MASK                 0x80000UL                              /**< Bit mask for USB_PKTCNT */
#define _USB_DOEP0TSIZ_PKTCNT_DEFAULT              0x00000000UL                           /**< Mode DEFAULT for USB_DOEP0TSIZ */
#define USB_DOEP0TSIZ_PKTCNT_DEFAULT               (_USB_DOEP0TSIZ_PKTCNT_DEFAULT << 19)  /**< Shifted mode DEFAULT for USB_DOEP0TSIZ */
#define _USB_DOEP0TSIZ_SUPCNT_SHIFT                29                                     /**< Shift value for USB_SUPCNT */
#define _USB_DOEP0TSIZ_SUPCNT_MASK                 0x60000000UL                           /**< Bit mask for USB_SUPCNT */
#define _USB_DOEP0TSIZ_SUPCNT_DEFAULT              0x00000000UL                           /**< Mode DEFAULT for USB_DOEP0TSIZ */
#define USB_DOEP0TSIZ_SUPCNT_DEFAULT               (_USB_DOEP0TSIZ_SUPCNT_DEFAULT << 29)  /**< Shifted mode DEFAULT for USB_DOEP0TSIZ */

/* Bit fields for USB DOEP0DMAADDR */
#define _USB_DOEP0DMAADDR_RESETVALUE               0x00000000UL                                  /**< Default value for USB_DOEP0DMAADDR */
#define _USB_DOEP0DMAADDR_MASK                     0xFFFFFFFFUL                                  /**< Mask for USB_DOEP0DMAADDR */
#define _USB_DOEP0DMAADDR_DOEP0DMAADDR_SHIFT       0                                             /**< Shift value for USB_DOEP0DMAADDR */
#define _USB_DOEP0DMAADDR_DOEP0DMAADDR_MASK        0xFFFFFFFFUL                                  /**< Bit mask for USB_DOEP0DMAADDR */
#define _USB_DOEP0DMAADDR_DOEP0DMAADDR_DEFAULT     0x00000000UL                                  /**< Mode DEFAULT for USB_DOEP0DMAADDR */
#define USB_DOEP0DMAADDR_DOEP0DMAADDR_DEFAULT      (_USB_DOEP0DMAADDR_DOEP0DMAADDR_DEFAULT << 0) /**< Shifted mode DEFAULT for USB_DOEP0DMAADDR */

/* Bit fields for USB DOEP_CTL */
#define _USB_DOEP_CTL_RESETVALUE                   0x00000000UL                             /**< Default value for USB_DOEP_CTL */
#define _USB_DOEP_CTL_MASK                         0xFC3F87FFUL                             /**< Mask for USB_DOEP_CTL */
#define _USB_DOEP_CTL_MPS_SHIFT                    0                                        /**< Shift value for USB_MPS */
#define _USB_DOEP_CTL_MPS_MASK                     0x7FFUL                                  /**< Bit mask for USB_MPS */
#define _USB_DOEP_CTL_MPS_DEFAULT                  0x00000000UL                             /**< Mode DEFAULT for USB_DOEP_CTL */
#define USB_DOEP_CTL_MPS_DEFAULT                   (_USB_DOEP_CTL_MPS_DEFAULT << 0)         /**< Shifted mode DEFAULT for USB_DOEP_CTL */
#define USB_DOEP_CTL_USBACTEP                      (0x1UL << 15)                            /**< USB Active Endpoint */
#define _USB_DOEP_CTL_USBACTEP_SHIFT               15                                       /**< Shift value for USB_USBACTEP */
#define _USB_DOEP_CTL_USBACTEP_MASK                0x8000UL                                 /**< Bit mask for USB_USBACTEP */
#define _USB_DOEP_CTL_USBACTEP_DEFAULT             0x00000000UL                             /**< Mode DEFAULT for USB_DOEP_CTL */
#define USB_DOEP_CTL_USBACTEP_DEFAULT              (_USB_DOEP_CTL_USBACTEP_DEFAULT << 15)   /**< Shifted mode DEFAULT for USB_DOEP_CTL */
#define USB_DOEP_CTL_DPIDEOF                       (0x1UL << 16)                            /**< Endpoint Data PID / Even-odd Frame */
#define _USB_DOEP_CTL_DPIDEOF_SHIFT                16                                       /**< Shift value for USB_DPIDEOF */
#define _USB_DOEP_CTL_DPIDEOF_MASK                 0x10000UL                                /**< Bit mask for USB_DPIDEOF */
#define _USB_DOEP_CTL_DPIDEOF_DEFAULT              0x00000000UL                             /**< Mode DEFAULT for USB_DOEP_CTL */
#define _USB_DOEP_CTL_DPIDEOF_DATA0EVEN            0x00000000UL                             /**< Mode DATA0EVEN for USB_DOEP_CTL */
#define _USB_DOEP_CTL_DPIDEOF_DATA1ODD             0x00000001UL                             /**< Mode DATA1ODD for USB_DOEP_CTL */
#define USB_DOEP_CTL_DPIDEOF_DEFAULT               (_USB_DOEP_CTL_DPIDEOF_DEFAULT << 16)    /**< Shifted mode DEFAULT for USB_DOEP_CTL */
#define USB_DOEP_CTL_DPIDEOF_DATA0EVEN             (_USB_DOEP_CTL_DPIDEOF_DATA0EVEN << 16)  /**< Shifted mode DATA0EVEN for USB_DOEP_CTL */
#define USB_DOEP_CTL_DPIDEOF_DATA1ODD              (_USB_DOEP_CTL_DPIDEOF_DATA1ODD << 16)   /**< Shifted mode DATA1ODD for USB_DOEP_CTL */
#define USB_DOEP_CTL_NAKSTS                        (0x1UL << 17)                            /**< NAK Status */
#define _USB_DOEP_CTL_NAKSTS_SHIFT                 17                                       /**< Shift value for USB_NAKSTS */
#define _USB_DOEP_CTL_NAKSTS_MASK                  0x20000UL                                /**< Bit mask for USB_NAKSTS */
#define _USB_DOEP_CTL_NAKSTS_DEFAULT               0x00000000UL                             /**< Mode DEFAULT for USB_DOEP_CTL */
#define USB_DOEP_CTL_NAKSTS_DEFAULT                (_USB_DOEP_CTL_NAKSTS_DEFAULT << 17)     /**< Shifted mode DEFAULT for USB_DOEP_CTL */
#define _USB_DOEP_CTL_EPTYPE_SHIFT                 18                                       /**< Shift value for USB_EPTYPE */
#define _USB_DOEP_CTL_EPTYPE_MASK                  0xC0000UL                                /**< Bit mask for USB_EPTYPE */
#define _USB_DOEP_CTL_EPTYPE_DEFAULT               0x00000000UL                             /**< Mode DEFAULT for USB_DOEP_CTL */
#define _USB_DOEP_CTL_EPTYPE_CONTROL               0x00000000UL                             /**< Mode CONTROL for USB_DOEP_CTL */
#define _USB_DOEP_CTL_EPTYPE_ISO                   0x00000001UL                             /**< Mode ISO for USB_DOEP_CTL */
#define _USB_DOEP_CTL_EPTYPE_BULK                  0x00000002UL                             /**< Mode BULK for USB_DOEP_CTL */
#define _USB_DOEP_CTL_EPTYPE_INT                   0x00000003UL                             /**< Mode INT for USB_DOEP_CTL */
#define USB_DOEP_CTL_EPTYPE_DEFAULT                (_USB_DOEP_CTL_EPTYPE_DEFAULT << 18)     /**< Shifted mode DEFAULT for USB_DOEP_CTL */
#define USB_DOEP_CTL_EPTYPE_CONTROL                (_USB_DOEP_CTL_EPTYPE_CONTROL << 18)     /**< Shifted mode CONTROL for USB_DOEP_CTL */
#define USB_DOEP_CTL_EPTYPE_ISO                    (_USB_DOEP_CTL_EPTYPE_ISO << 18)         /**< Shifted mode ISO for USB_DOEP_CTL */
#define USB_DOEP_CTL_EPTYPE_BULK                   (_USB_DOEP_CTL_EPTYPE_BULK << 18)        /**< Shifted mode BULK for USB_DOEP_CTL */
#define USB_DOEP_CTL_EPTYPE_INT                    (_USB_DOEP_CTL_EPTYPE_INT << 18)         /**< Shifted mode INT for USB_DOEP_CTL */
#define USB_DOEP_CTL_SNP                           (0x1UL << 20)                            /**< Snoop Mode */
#define _USB_DOEP_CTL_SNP_SHIFT                    20                                       /**< Shift value for USB_SNP */
#define _USB_DOEP_CTL_SNP_MASK                     0x100000UL                               /**< Bit mask for USB_SNP */
#define _USB_DOEP_CTL_SNP_DEFAULT                  0x00000000UL                             /**< Mode DEFAULT for USB_DOEP_CTL */
#define USB_DOEP_CTL_SNP_DEFAULT                   (_USB_DOEP_CTL_SNP_DEFAULT << 20)        /**< Shifted mode DEFAULT for USB_DOEP_CTL */
#define USB_DOEP_CTL_STALL                         (0x1UL << 21)                            /**< STALL Handshake */
#define _USB_DOEP_CTL_STALL_SHIFT                  21                                       /**< Shift value for USB_STALL */
#define _USB_DOEP_CTL_STALL_MASK                   0x200000UL                               /**< Bit mask for USB_STALL */
#define _USB_DOEP_CTL_STALL_DEFAULT                0x00000000UL                             /**< Mode DEFAULT for USB_DOEP_CTL */
#define USB_DOEP_CTL_STALL_DEFAULT                 (_USB_DOEP_CTL_STALL_DEFAULT << 21)      /**< Shifted mode DEFAULT for USB_DOEP_CTL */
#define USB_DOEP_CTL_CNAK                          (0x1UL << 26)                            /**< Clear NAK */
#define _USB_DOEP_CTL_CNAK_SHIFT                   26                                       /**< Shift value for USB_CNAK */
#define _USB_DOEP_CTL_CNAK_MASK                    0x4000000UL                              /**< Bit mask for USB_CNAK */
#define _USB_DOEP_CTL_CNAK_DEFAULT                 0x00000000UL                             /**< Mode DEFAULT for USB_DOEP_CTL */
#define USB_DOEP_CTL_CNAK_DEFAULT                  (_USB_DOEP_CTL_CNAK_DEFAULT << 26)       /**< Shifted mode DEFAULT for USB_DOEP_CTL */
#define USB_DOEP_CTL_SNAK                          (0x1UL << 27)                            /**< Set NAK */
#define _USB_DOEP_CTL_SNAK_SHIFT                   27                                       /**< Shift value for USB_SNAK */
#define _USB_DOEP_CTL_SNAK_MASK                    0x8000000UL                              /**< Bit mask for USB_SNAK */
#define _USB_DOEP_CTL_SNAK_DEFAULT                 0x00000000UL                             /**< Mode DEFAULT for USB_DOEP_CTL */
#define USB_DOEP_CTL_SNAK_DEFAULT                  (_USB_DOEP_CTL_SNAK_DEFAULT << 27)       /**< Shifted mode DEFAULT for USB_DOEP_CTL */
#define USB_DOEP_CTL_SETD0PIDEF                    (0x1UL << 28)                            /**< Set DATA0 PID / Even Frame */
#define _USB_DOEP_CTL_SETD0PIDEF_SHIFT             28                                       /**< Shift value for USB_SETD0PIDEF */
#define _USB_DOEP_CTL_SETD0PIDEF_MASK              0x10000000UL                             /**< Bit mask for USB_SETD0PIDEF */
#define _USB_DOEP_CTL_SETD0PIDEF_DEFAULT           0x00000000UL                             /**< Mode DEFAULT for USB_DOEP_CTL */
#define USB_DOEP_CTL_SETD0PIDEF_DEFAULT            (_USB_DOEP_CTL_SETD0PIDEF_DEFAULT << 28) /**< Shifted mode DEFAULT for USB_DOEP_CTL */
#define USB_DOEP_CTL_SETD1PIDOF                    (0x1UL << 29)                            /**< Set DATA1 PID / Odd Frame */
#define _USB_DOEP_CTL_SETD1PIDOF_SHIFT             29                                       /**< Shift value for USB_SETD1PIDOF */
#define _USB_DOEP_CTL_SETD1PIDOF_MASK              0x20000000UL                             /**< Bit mask for USB_SETD1PIDOF */
#define _USB_DOEP_CTL_SETD1PIDOF_DEFAULT           0x00000000UL                             /**< Mode DEFAULT for USB_DOEP_CTL */
#define USB_DOEP_CTL_SETD1PIDOF_DEFAULT            (_USB_DOEP_CTL_SETD1PIDOF_DEFAULT << 29) /**< Shifted mode DEFAULT for USB_DOEP_CTL */
#define USB_DOEP_CTL_EPDIS                         (0x1UL << 30)                            /**< Endpoint Disable */
#define _USB_DOEP_CTL_EPDIS_SHIFT                  30                                       /**< Shift value for USB_EPDIS */
#define _USB_DOEP_CTL_EPDIS_MASK                   0x40000000UL                             /**< Bit mask for USB_EPDIS */
#define _USB_DOEP_CTL_EPDIS_DEFAULT                0x00000000UL                             /**< Mode DEFAULT for USB_DOEP_CTL */
#define USB_DOEP_CTL_EPDIS_DEFAULT                 (_USB_DOEP_CTL_EPDIS_DEFAULT << 30)      /**< Shifted mode DEFAULT for USB_DOEP_CTL */
#define USB_DOEP_CTL_EPENA                         (0x1UL << 31)                            /**< Endpoint Enable */
#define _USB_DOEP_CTL_EPENA_SHIFT                  31                                       /**< Shift value for USB_EPENA */
#define _USB_DOEP_CTL_EPENA_MASK                   0x80000000UL                             /**< Bit mask for USB_EPENA */
#define _USB_DOEP_CTL_EPENA_DEFAULT                0x00000000UL                             /**< Mode DEFAULT for USB_DOEP_CTL */
#define USB_DOEP_CTL_EPENA_DEFAULT                 (_USB_DOEP_CTL_EPENA_DEFAULT << 31)      /**< Shifted mode DEFAULT for USB_DOEP_CTL */

/* Bit fields for USB DOEP_INT */
#define _USB_DOEP_INT_RESETVALUE                   0x00000000UL                                /**< Default value for USB_DOEP_INT */
#define _USB_DOEP_INT_MASK                         0x0000B87FUL                                /**< Mask for USB_DOEP_INT */
#define USB_DOEP_INT_XFERCOMPL                     (0x1UL << 0)                                /**< Transfer Completed Interrupt */
#define _USB_DOEP_INT_XFERCOMPL_SHIFT              0                                           /**< Shift value for USB_XFERCOMPL */
#define _USB_DOEP_INT_XFERCOMPL_MASK               0x1UL                                       /**< Bit mask for USB_XFERCOMPL */
#define _USB_DOEP_INT_XFERCOMPL_DEFAULT            0x00000000UL                                /**< Mode DEFAULT for USB_DOEP_INT */
#define USB_DOEP_INT_XFERCOMPL_DEFAULT             (_USB_DOEP_INT_XFERCOMPL_DEFAULT << 0)      /**< Shifted mode DEFAULT for USB_DOEP_INT */
#define USB_DOEP_INT_EPDISBLD                      (0x1UL << 1)                                /**< Endpoint Disabled Interrupt */
#define _USB_DOEP_INT_EPDISBLD_SHIFT               1                                           /**< Shift value for USB_EPDISBLD */
#define _USB_DOEP_INT_EPDISBLD_MASK                0x2UL                                       /**< Bit mask for USB_EPDISBLD */
#define _USB_DOEP_INT_EPDISBLD_DEFAULT             0x00000000UL                                /**< Mode DEFAULT for USB_DOEP_INT */
#define USB_DOEP_INT_EPDISBLD_DEFAULT              (_USB_DOEP_INT_EPDISBLD_DEFAULT << 1)       /**< Shifted mode DEFAULT for USB_DOEP_INT */
#define USB_DOEP_INT_AHBERR                        (0x1UL << 2)                                /**< AHB Error */
#define _USB_DOEP_INT_AHBERR_SHIFT                 2                                           /**< Shift value for USB_AHBERR */
#define _USB_DOEP_INT_AHBERR_MASK                  0x4UL                                       /**< Bit mask for USB_AHBERR */
#define _USB_DOEP_INT_AHBERR_DEFAULT               0x00000000UL                                /**< Mode DEFAULT for USB_DOEP_INT */
#define USB_DOEP_INT_AHBERR_DEFAULT                (_USB_DOEP_INT_AHBERR_DEFAULT << 2)         /**< Shifted mode DEFAULT for USB_DOEP_INT */
#define USB_DOEP_INT_SETUP                         (0x1UL << 3)                                /**< Setup Phase Done */
#define _USB_DOEP_INT_SETUP_SHIFT                  3                                           /**< Shift value for USB_SETUP */
#define _USB_DOEP_INT_SETUP_MASK                   0x8UL                                       /**< Bit mask for USB_SETUP */
#define _USB_DOEP_INT_SETUP_DEFAULT                0x00000000UL                                /**< Mode DEFAULT for USB_DOEP_INT */
#define USB_DOEP_INT_SETUP_DEFAULT                 (_USB_DOEP_INT_SETUP_DEFAULT << 3)          /**< Shifted mode DEFAULT for USB_DOEP_INT */
#define USB_DOEP_INT_OUTTKNEPDIS                   (0x1UL << 4)                                /**< OUT Token Received When Endpoint Disabled */
#define _USB_DOEP_INT_OUTTKNEPDIS_SHIFT            4                                           /**< Shift value for USB_OUTTKNEPDIS */
#define _USB_DOEP_INT_OUTTKNEPDIS_MASK             0x10UL                                      /**< Bit mask for USB_OUTTKNEPDIS */
#define _USB_DOEP_INT_OUTTKNEPDIS_DEFAULT          0x00000000UL                                /**< Mode DEFAULT for USB_DOEP_INT */
#define USB_DOEP_INT_OUTTKNEPDIS_DEFAULT           (_USB_DOEP_INT_OUTTKNEPDIS_DEFAULT << 4)    /**< Shifted mode DEFAULT for USB_DOEP_INT */
#define USB_DOEP_INT_STSPHSERCVD                   (0x1UL << 5)                                /**< Status Phase Received For Control Write */
#define _USB_DOEP_INT_STSPHSERCVD_SHIFT            5                                           /**< Shift value for USB_STSPHSERCVD */
#define _USB_DOEP_INT_STSPHSERCVD_MASK             0x20UL                                      /**< Bit mask for USB_STSPHSERCVD */
#define _USB_DOEP_INT_STSPHSERCVD_DEFAULT          0x00000000UL                                /**< Mode DEFAULT for USB_DOEP_INT */
#define USB_DOEP_INT_STSPHSERCVD_DEFAULT           (_USB_DOEP_INT_STSPHSERCVD_DEFAULT << 5)    /**< Shifted mode DEFAULT for USB_DOEP_INT */
#define USB_DOEP_INT_BACK2BACKSETUP                (0x1UL << 6)                                /**< Back-to-Back SETUP Packets Received */
#define _USB_DOEP_INT_BACK2BACKSETUP_SHIFT         6                                           /**< Shift value for USB_BACK2BACKSETUP */
#define _USB_DOEP_INT_BACK2BACKSETUP_MASK          0x40UL                                      /**< Bit mask for USB_BACK2BACKSETUP */
#define _USB_DOEP_INT_BACK2BACKSETUP_DEFAULT       0x00000000UL                                /**< Mode DEFAULT for USB_DOEP_INT */
#define USB_DOEP_INT_BACK2BACKSETUP_DEFAULT        (_USB_DOEP_INT_BACK2BACKSETUP_DEFAULT << 6) /**< Shifted mode DEFAULT for USB_DOEP_INT */
#define USB_DOEP_INT_PKTDRPSTS                     (0x1UL << 11)                               /**< Packet Drop Status */
#define _USB_DOEP_INT_PKTDRPSTS_SHIFT              11                                          /**< Shift value for USB_PKTDRPSTS */
#define _USB_DOEP_INT_PKTDRPSTS_MASK               0x800UL                                     /**< Bit mask for USB_PKTDRPSTS */
#define _USB_DOEP_INT_PKTDRPSTS_DEFAULT            0x00000000UL                                /**< Mode DEFAULT for USB_DOEP_INT */
#define USB_DOEP_INT_PKTDRPSTS_DEFAULT             (_USB_DOEP_INT_PKTDRPSTS_DEFAULT << 11)     /**< Shifted mode DEFAULT for USB_DOEP_INT */
#define USB_DOEP_INT_BBLEERR                       (0x1UL << 12)                               /**< Babble Error */
#define _USB_DOEP_INT_BBLEERR_SHIFT                12                                          /**< Shift value for USB_BBLEERR */
#define _USB_DOEP_INT_BBLEERR_MASK                 0x1000UL                                    /**< Bit mask for USB_BBLEERR */
#define _USB_DOEP_INT_BBLEERR_DEFAULT              0x00000000UL                                /**< Mode DEFAULT for USB_DOEP_INT */
#define USB_DOEP_INT_BBLEERR_DEFAULT               (_USB_DOEP_INT_BBLEERR_DEFAULT << 12)       /**< Shifted mode DEFAULT for USB_DOEP_INT */
#define USB_DOEP_INT_NAKINTRPT                     (0x1UL << 13)                               /**< NAK Interrupt */
#define _USB_DOEP_INT_NAKINTRPT_SHIFT              13                                          /**< Shift value for USB_NAKINTRPT */
#define _USB_DOEP_INT_NAKINTRPT_MASK               0x2000UL                                    /**< Bit mask for USB_NAKINTRPT */
#define _USB_DOEP_INT_NAKINTRPT_DEFAULT            0x00000000UL                                /**< Mode DEFAULT for USB_DOEP_INT */
#define USB_DOEP_INT_NAKINTRPT_DEFAULT             (_USB_DOEP_INT_NAKINTRPT_DEFAULT << 13)     /**< Shifted mode DEFAULT for USB_DOEP_INT */
#define USB_DOEP_INT_STUPPKTRCVD                   (0x1UL << 15)                               /**< Setup Packet Received */
#define _USB_DOEP_INT_STUPPKTRCVD_SHIFT            15                                          /**< Shift value for USB_STUPPKTRCVD */
#define _USB_DOEP_INT_STUPPKTRCVD_MASK             0x8000UL                                    /**< Bit mask for USB_STUPPKTRCVD */
#define _USB_DOEP_INT_STUPPKTRCVD_DEFAULT          0x00000000UL                                /**< Mode DEFAULT for USB_DOEP_INT */
#define USB_DOEP_INT_STUPPKTRCVD_DEFAULT           (_USB_DOEP_INT_STUPPKTRCVD_DEFAULT << 15)   /**< Shifted mode DEFAULT for USB_DOEP_INT */

/* Bit fields for USB DOEP_TSIZ */
#define _USB_DOEP_TSIZ_RESETVALUE                  0x00000000UL                                /**< Default value for USB_DOEP_TSIZ */
#define _USB_DOEP_TSIZ_MASK                        0x7FFFFFFFUL                                /**< Mask for USB_DOEP_TSIZ */
#define _USB_DOEP_TSIZ_XFERSIZE_SHIFT              0                                           /**< Shift value for USB_XFERSIZE */
#define _USB_DOEP_TSIZ_XFERSIZE_MASK               0x7FFFFUL                                   /**< Bit mask for USB_XFERSIZE */
#define _USB_DOEP_TSIZ_XFERSIZE_DEFAULT            0x00000000UL                                /**< Mode DEFAULT for USB_DOEP_TSIZ */
#define USB_DOEP_TSIZ_XFERSIZE_DEFAULT             (_USB_DOEP_TSIZ_XFERSIZE_DEFAULT << 0)      /**< Shifted mode DEFAULT for USB_DOEP_TSIZ */
#define _USB_DOEP_TSIZ_PKTCNT_SHIFT                19                                          /**< Shift value for USB_PKTCNT */
#define _USB_DOEP_TSIZ_PKTCNT_MASK                 0x1FF80000UL                                /**< Bit mask for USB_PKTCNT */
#define _USB_DOEP_TSIZ_PKTCNT_DEFAULT              0x00000000UL                                /**< Mode DEFAULT for USB_DOEP_TSIZ */
#define USB_DOEP_TSIZ_PKTCNT_DEFAULT               (_USB_DOEP_TSIZ_PKTCNT_DEFAULT << 19)       /**< Shifted mode DEFAULT for USB_DOEP_TSIZ */
#define _USB_DOEP_TSIZ_RXDPIDSUPCNT_SHIFT          29                                          /**< Shift value for USB_RXDPIDSUPCNT */
#define _USB_DOEP_TSIZ_RXDPIDSUPCNT_MASK           0x60000000UL                                /**< Bit mask for USB_RXDPIDSUPCNT */
#define _USB_DOEP_TSIZ_RXDPIDSUPCNT_DEFAULT        0x00000000UL                                /**< Mode DEFAULT for USB_DOEP_TSIZ */
#define _USB_DOEP_TSIZ_RXDPIDSUPCNT_DATA0          0x00000000UL                                /**< Mode DATA0 for USB_DOEP_TSIZ */
#define _USB_DOEP_TSIZ_RXDPIDSUPCNT_DATA2          0x00000001UL                                /**< Mode DATA2 for USB_DOEP_TSIZ */
#define _USB_DOEP_TSIZ_RXDPIDSUPCNT_DATA1          0x00000002UL                                /**< Mode DATA1 for USB_DOEP_TSIZ */
#define _USB_DOEP_TSIZ_RXDPIDSUPCNT_MDATA          0x00000003UL                                /**< Mode MDATA for USB_DOEP_TSIZ */
#define USB_DOEP_TSIZ_RXDPIDSUPCNT_DEFAULT         (_USB_DOEP_TSIZ_RXDPIDSUPCNT_DEFAULT << 29) /**< Shifted mode DEFAULT for USB_DOEP_TSIZ */
#define USB_DOEP_TSIZ_RXDPIDSUPCNT_DATA0           (_USB_DOEP_TSIZ_RXDPIDSUPCNT_DATA0 << 29)   /**< Shifted mode DATA0 for USB_DOEP_TSIZ */
#define USB_DOEP_TSIZ_RXDPIDSUPCNT_DATA2           (_USB_DOEP_TSIZ_RXDPIDSUPCNT_DATA2 << 29)   /**< Shifted mode DATA2 for USB_DOEP_TSIZ */
#define USB_DOEP_TSIZ_RXDPIDSUPCNT_DATA1           (_USB_DOEP_TSIZ_RXDPIDSUPCNT_DATA1 << 29)   /**< Shifted mode DATA1 for USB_DOEP_TSIZ */
#define USB_DOEP_TSIZ_RXDPIDSUPCNT_MDATA           (_USB_DOEP_TSIZ_RXDPIDSUPCNT_MDATA << 29)   /**< Shifted mode MDATA for USB_DOEP_TSIZ */

/* Bit fields for USB DOEP_DMAADDR */
#define _USB_DOEP_DMAADDR_RESETVALUE               0x00000000UL                             /**< Default value for USB_DOEP_DMAADDR */
#define _USB_DOEP_DMAADDR_MASK                     0xFFFFFFFFUL                             /**< Mask for USB_DOEP_DMAADDR */
#define _USB_DOEP_DMAADDR_DMAADDR_SHIFT            0                                        /**< Shift value for USB_DMAADDR */
#define _USB_DOEP_DMAADDR_DMAADDR_MASK             0xFFFFFFFFUL                             /**< Bit mask for USB_DMAADDR */
#define _USB_DOEP_DMAADDR_DMAADDR_DEFAULT          0x00000000UL                             /**< Mode DEFAULT for USB_DOEP_DMAADDR */
#define USB_DOEP_DMAADDR_DMAADDR_DEFAULT           (_USB_DOEP_DMAADDR_DMAADDR_DEFAULT << 0) /**< Shifted mode DEFAULT for USB_DOEP_DMAADDR */

/* Bit fields for USB PCGCCTL */
#define _USB_PCGCCTL_RESETVALUE                    0x00000000UL                              /**< Default value for USB_PCGCCTL */
#define _USB_PCGCCTL_MASK                          0x0000004FUL                              /**< Mask for USB_PCGCCTL */
#define USB_PCGCCTL_STOPPCLK                       (0x1UL << 0)                              /**< Stop PHY clock */
#define _USB_PCGCCTL_STOPPCLK_SHIFT                0                                         /**< Shift value for USB_STOPPCLK */
#define _USB_PCGCCTL_STOPPCLK_MASK                 0x1UL                                     /**< Bit mask for USB_STOPPCLK */
#define _USB_PCGCCTL_STOPPCLK_DEFAULT              0x00000000UL                              /**< Mode DEFAULT for USB_PCGCCTL */
#define USB_PCGCCTL_STOPPCLK_DEFAULT               (_USB_PCGCCTL_STOPPCLK_DEFAULT << 0)      /**< Shifted mode DEFAULT for USB_PCGCCTL */
#define USB_PCGCCTL_GATEHCLK                       (0x1UL << 1)                              /**< Gate HCLK */
#define _USB_PCGCCTL_GATEHCLK_SHIFT                1                                         /**< Shift value for USB_GATEHCLK */
#define _USB_PCGCCTL_GATEHCLK_MASK                 0x2UL                                     /**< Bit mask for USB_GATEHCLK */
#define _USB_PCGCCTL_GATEHCLK_DEFAULT              0x00000000UL                              /**< Mode DEFAULT for USB_PCGCCTL */
#define USB_PCGCCTL_GATEHCLK_DEFAULT               (_USB_PCGCCTL_GATEHCLK_DEFAULT << 1)      /**< Shifted mode DEFAULT for USB_PCGCCTL */
#define USB_PCGCCTL_PWRCLMP                        (0x1UL << 2)                              /**< Power Clamp */
#define _USB_PCGCCTL_PWRCLMP_SHIFT                 2                                         /**< Shift value for USB_PWRCLMP */
#define _USB_PCGCCTL_PWRCLMP_MASK                  0x4UL                                     /**< Bit mask for USB_PWRCLMP */
#define _USB_PCGCCTL_PWRCLMP_DEFAULT               0x00000000UL                              /**< Mode DEFAULT for USB_PCGCCTL */
#define USB_PCGCCTL_PWRCLMP_DEFAULT                (_USB_PCGCCTL_PWRCLMP_DEFAULT << 2)       /**< Shifted mode DEFAULT for USB_PCGCCTL */
#define USB_PCGCCTL_RSTPDWNMODULE                  (0x1UL << 3)                              /**< Reset Power-Down Modules */
#define _USB_PCGCCTL_RSTPDWNMODULE_SHIFT           3                                         /**< Shift value for USB_RSTPDWNMODULE */
#define _USB_PCGCCTL_RSTPDWNMODULE_MASK            0x8UL                                     /**< Bit mask for USB_RSTPDWNMODULE */
#define _USB_PCGCCTL_RSTPDWNMODULE_DEFAULT         0x00000000UL                              /**< Mode DEFAULT for USB_PCGCCTL */
#define USB_PCGCCTL_RSTPDWNMODULE_DEFAULT          (_USB_PCGCCTL_RSTPDWNMODULE_DEFAULT << 3) /**< Shifted mode DEFAULT for USB_PCGCCTL */
#define USB_PCGCCTL_PHYSLEEP                       (0x1UL << 6)                              /**< PHY In Sleep */
#define _USB_PCGCCTL_PHYSLEEP_SHIFT                6                                         /**< Shift value for USB_PHYSLEEP */
#define _USB_PCGCCTL_PHYSLEEP_MASK                 0x40UL                                    /**< Bit mask for USB_PHYSLEEP */
#define _USB_PCGCCTL_PHYSLEEP_DEFAULT              0x00000000UL                              /**< Mode DEFAULT for USB_PCGCCTL */
#define USB_PCGCCTL_PHYSLEEP_DEFAULT               (_USB_PCGCCTL_PHYSLEEP_DEFAULT << 6)      /**< Shifted mode DEFAULT for USB_PCGCCTL */

/* Bit fields for USB FIFO0D */
#define _USB_FIFO0D_RESETVALUE                     0x00000000UL                      /**< Default value for USB_FIFO0D */
#define _USB_FIFO0D_MASK                           0xFFFFFFFFUL                      /**< Mask for USB_FIFO0D */
#define _USB_FIFO0D_FIFO0D_SHIFT                   0                                 /**< Shift value for USB_FIFO0D */
#define _USB_FIFO0D_FIFO0D_MASK                    0xFFFFFFFFUL                      /**< Bit mask for USB_FIFO0D */
#define _USB_FIFO0D_FIFO0D_DEFAULT                 0x00000000UL                      /**< Mode DEFAULT for USB_FIFO0D */
#define USB_FIFO0D_FIFO0D_DEFAULT                  (_USB_FIFO0D_FIFO0D_DEFAULT << 0) /**< Shifted mode DEFAULT for USB_FIFO0D */

/* Bit fields for USB FIFO1D */
#define _USB_FIFO1D_RESETVALUE                     0x00000000UL                      /**< Default value for USB_FIFO1D */
#define _USB_FIFO1D_MASK                           0xFFFFFFFFUL                      /**< Mask for USB_FIFO1D */
#define _USB_FIFO1D_FIFO1D_SHIFT                   0                                 /**< Shift value for USB_FIFO1D */
#define _USB_FIFO1D_FIFO1D_MASK                    0xFFFFFFFFUL                      /**< Bit mask for USB_FIFO1D */
#define _USB_FIFO1D_FIFO1D_DEFAULT                 0x00000000UL                      /**< Mode DEFAULT for USB_FIFO1D */
#define USB_FIFO1D_FIFO1D_DEFAULT                  (_USB_FIFO1D_FIFO1D_DEFAULT << 0) /**< Shifted mode DEFAULT for USB_FIFO1D */

/* Bit fields for USB FIFO2D */
#define _USB_FIFO2D_RESETVALUE                     0x00000000UL                      /**< Default value for USB_FIFO2D */
#define _USB_FIFO2D_MASK                           0xFFFFFFFFUL                      /**< Mask for USB_FIFO2D */
#define _USB_FIFO2D_FIFO2D_SHIFT                   0                                 /**< Shift value for USB_FIFO2D */
#define _USB_FIFO2D_FIFO2D_MASK                    0xFFFFFFFFUL                      /**< Bit mask for USB_FIFO2D */
#define _USB_FIFO2D_FIFO2D_DEFAULT                 0x00000000UL                      /**< Mode DEFAULT for USB_FIFO2D */
#define USB_FIFO2D_FIFO2D_DEFAULT                  (_USB_FIFO2D_FIFO2D_DEFAULT << 0) /**< Shifted mode DEFAULT for USB_FIFO2D */

/* Bit fields for USB FIFO3D */
#define _USB_FIFO3D_RESETVALUE                     0x00000000UL                      /**< Default value for USB_FIFO3D */
#define _USB_FIFO3D_MASK                           0xFFFFFFFFUL                      /**< Mask for USB_FIFO3D */
#define _USB_FIFO3D_FIFO3D_SHIFT                   0                                 /**< Shift value for USB_FIFO3D */
#define _USB_FIFO3D_FIFO3D_MASK                    0xFFFFFFFFUL                      /**< Bit mask for USB_FIFO3D */
#define _USB_FIFO3D_FIFO3D_DEFAULT                 0x00000000UL                      /**< Mode DEFAULT for USB_FIFO3D */
#define USB_FIFO3D_FIFO3D_DEFAULT                  (_USB_FIFO3D_FIFO3D_DEFAULT << 0) /**< Shifted mode DEFAULT for USB_FIFO3D */

/* Bit fields for USB FIFORAM */
#define _USB_FIFORAM_RESETVALUE                    0x00000000UL                        /**< Default value for USB_FIFORAM */
#define _USB_FIFORAM_MASK                          0xFFFFFFFFUL                        /**< Mask for USB_FIFORAM */
#define _USB_FIFORAM_FIFORAM_SHIFT                 0                                   /**< Shift value for USB_FIFORAM */
#define _USB_FIFORAM_FIFORAM_MASK                  0xFFFFFFFFUL                        /**< Bit mask for USB_FIFORAM */
#define _USB_FIFORAM_FIFORAM_DEFAULT               0x00000000UL                        /**< Mode DEFAULT for USB_FIFORAM */
#define USB_FIFORAM_FIFORAM_DEFAULT                (_USB_FIFORAM_FIFORAM_DEFAULT << 0) /**< Shifted mode DEFAULT for USB_FIFORAM */

/* Bit fields for GPIO P_CTRL */
#define _GPIO_P_CTRL_RESETVALUE                           0x00000000UL                           /**< Default value for GPIO_P_CTRL */
#define _GPIO_P_CTRL_MASK                                 0x00000003UL                           /**< Mask for GPIO_P_CTRL */
#define _GPIO_P_CTRL_DRIVEMODE_SHIFT                      0                                      /**< Shift value for GPIO_DRIVEMODE */
#define _GPIO_P_CTRL_DRIVEMODE_MASK                       0x3UL                                  /**< Bit mask for GPIO_DRIVEMODE */
#define _GPIO_P_CTRL_DRIVEMODE_DEFAULT                    0x00000000UL                           /**< Mode DEFAULT for GPIO_P_CTRL */
#define _GPIO_P_CTRL_DRIVEMODE_STANDARD                   0x00000000UL                           /**< Mode STANDARD for GPIO_P_CTRL */
#define _GPIO_P_CTRL_DRIVEMODE_LOWEST                     0x00000001UL                           /**< Mode LOWEST for GPIO_P_CTRL */
#define _GPIO_P_CTRL_DRIVEMODE_HIGH                       0x00000002UL                           /**< Mode HIGH for GPIO_P_CTRL */
#define _GPIO_P_CTRL_DRIVEMODE_LOW                        0x00000003UL                           /**< Mode LOW for GPIO_P_CTRL */
#define GPIO_P_CTRL_DRIVEMODE_DEFAULT                     (_GPIO_P_CTRL_DRIVEMODE_DEFAULT << 0)  /**< Shifted mode DEFAULT for GPIO_P_CTRL */
#define GPIO_P_CTRL_DRIVEMODE_STANDARD                    (_GPIO_P_CTRL_DRIVEMODE_STANDARD << 0) /**< Shifted mode STANDARD for GPIO_P_CTRL */
#define GPIO_P_CTRL_DRIVEMODE_LOWEST                      (_GPIO_P_CTRL_DRIVEMODE_LOWEST << 0)   /**< Shifted mode LOWEST for GPIO_P_CTRL */
#define GPIO_P_CTRL_DRIVEMODE_HIGH                        (_GPIO_P_CTRL_DRIVEMODE_HIGH << 0)     /**< Shifted mode HIGH for GPIO_P_CTRL */
#define GPIO_P_CTRL_DRIVEMODE_LOW                         (_GPIO_P_CTRL_DRIVEMODE_LOW << 0)      /**< Shifted mode LOW for GPIO_P_CTRL */

/* Bit fields for GPIO P_MODEL */
#define _GPIO_P_MODEL_RESETVALUE                          0x00000000UL                                          /**< Default value for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MASK                                0xFFFFFFFFUL                                          /**< Mask for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE0_SHIFT                         0                                                     /**< Shift value for GPIO_MODE0 */
#define _GPIO_P_MODEL_MODE0_MASK                          0xFUL                                                 /**< Bit mask for GPIO_MODE0 */
#define _GPIO_P_MODEL_MODE0_DEFAULT                       0x00000000UL                                          /**< Mode DEFAULT for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE0_DISABLED                      0x00000000UL                                          /**< Mode DISABLED for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE0_INPUT                         0x00000001UL                                          /**< Mode INPUT for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE0_INPUTPULL                     0x00000002UL                                          /**< Mode INPUTPULL for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE0_INPUTPULLFILTER               0x00000003UL                                          /**< Mode INPUTPULLFILTER for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE0_PUSHPULL                      0x00000004UL                                          /**< Mode PUSHPULL for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE0_PUSHPULLDRIVE                 0x00000005UL                                          /**< Mode PUSHPULLDRIVE for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE0_WIREDOR                       0x00000006UL                                          /**< Mode WIREDOR for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE0_WIREDORPULLDOWN               0x00000007UL                                          /**< Mode WIREDORPULLDOWN for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE0_WIREDAND                      0x00000008UL                                          /**< Mode WIREDAND for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE0_WIREDANDFILTER                0x00000009UL                                          /**< Mode WIREDANDFILTER for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE0_WIREDANDPULLUP                0x0000000AUL                                          /**< Mode WIREDANDPULLUP for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE0_WIREDANDPULLUPFILTER          0x0000000BUL                                          /**< Mode WIREDANDPULLUPFILTER for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE0_WIREDANDDRIVE                 0x0000000CUL                                          /**< Mode WIREDANDDRIVE for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE0_WIREDANDDRIVEFILTER           0x0000000DUL                                          /**< Mode WIREDANDDRIVEFILTER for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE0_WIREDANDDRIVEPULLUP           0x0000000EUL                                          /**< Mode WIREDANDDRIVEPULLUP for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE0_WIREDANDDRIVEPULLUPFILTER     0x0000000FUL                                          /**< Mode WIREDANDDRIVEPULLUPFILTER for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE0_DEFAULT                        (_GPIO_P_MODEL_MODE0_DEFAULT << 0)                    /**< Shifted mode DEFAULT for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE0_DISABLED                       (_GPIO_P_MODEL_MODE0_DISABLED << 0)                   /**< Shifted mode DISABLED for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE0_INPUT                          (_GPIO_P_MODEL_MODE0_INPUT << 0)                      /**< Shifted mode INPUT for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE0_INPUTPULL                      (_GPIO_P_MODEL_MODE0_INPUTPULL << 0)                  /**< Shifted mode INPUTPULL for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE0_INPUTPULLFILTER                (_GPIO_P_MODEL_MODE0_INPUTPULLFILTER << 0)            /**< Shifted mode INPUTPULLFILTER for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE0_PUSHPULL                       (_GPIO_P_MODEL_MODE0_PUSHPULL << 0)                   /**< Shifted mode PUSHPULL for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE0_PUSHPULLDRIVE                  (_GPIO_P_MODEL_MODE0_PUSHPULLDRIVE << 0)              /**< Shifted mode PUSHPULLDRIVE for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE0_WIREDOR                        (_GPIO_P_MODEL_MODE0_WIREDOR << 0)                    /**< Shifted mode WIREDOR for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE0_WIREDORPULLDOWN                (_GPIO_P_MODEL_MODE0_WIREDORPULLDOWN << 0)            /**< Shifted mode WIREDORPULLDOWN for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE0_WIREDAND                       (_GPIO_P_MODEL_MODE0_WIREDAND << 0)                   /**< Shifted mode WIREDAND for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE0_WIREDANDFILTER                 (_GPIO_P_MODEL_MODE0_WIREDANDFILTER << 0)             /**< Shifted mode WIREDANDFILTER for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE0_WIREDANDPULLUP                 (_GPIO_P_MODEL_MODE0_WIREDANDPULLUP << 0)             /**< Shifted mode WIREDANDPULLUP for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE0_WIREDANDPULLUPFILTER           (_GPIO_P_MODEL_MODE0_WIREDANDPULLUPFILTER << 0)       /**< Shifted mode WIREDANDPULLUPFILTER for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE0_WIREDANDDRIVE                  (_GPIO_P_MODEL_MODE0_WIREDANDDRIVE << 0)              /**< Shifted mode WIREDANDDRIVE for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE0_WIREDANDDRIVEFILTER            (_GPIO_P_MODEL_MODE0_WIREDANDDRIVEFILTER << 0)        /**< Shifted mode WIREDANDDRIVEFILTER for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE0_WIREDANDDRIVEPULLUP            (_GPIO_P_MODEL_MODE0_WIREDANDDRIVEPULLUP << 0)        /**< Shifted mode WIREDANDDRIVEPULLUP for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE0_WIREDANDDRIVEPULLUPFILTER      (_GPIO_P_MODEL_MODE0_WIREDANDDRIVEPULLUPFILTER << 0)  /**< Shifted mode WIREDANDDRIVEPULLUPFILTER for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE1_SHIFT                         4                                                     /**< Shift value for GPIO_MODE1 */
#define _GPIO_P_MODEL_MODE1_MASK                          0xF0UL                                                /**< Bit mask for GPIO_MODE1 */
#define _GPIO_P_MODEL_MODE1_DEFAULT                       0x00000000UL                                          /**< Mode DEFAULT for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE1_DISABLED                      0x00000000UL                                          /**< Mode DISABLED for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE1_INPUT                         0x00000001UL                                          /**< Mode INPUT for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE1_INPUTPULL                     0x00000002UL                                          /**< Mode INPUTPULL for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE1_INPUTPULLFILTER               0x00000003UL                                          /**< Mode INPUTPULLFILTER for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE1_PUSHPULL                      0x00000004UL                                          /**< Mode PUSHPULL for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE1_PUSHPULLDRIVE                 0x00000005UL                                          /**< Mode PUSHPULLDRIVE for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE1_WIREDOR                       0x00000006UL                                          /**< Mode WIREDOR for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE1_WIREDORPULLDOWN               0x00000007UL                                          /**< Mode WIREDORPULLDOWN for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE1_WIREDAND                      0x00000008UL                                          /**< Mode WIREDAND for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE1_WIREDANDFILTER                0x00000009UL                                          /**< Mode WIREDANDFILTER for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE1_WIREDANDPULLUP                0x0000000AUL                                          /**< Mode WIREDANDPULLUP for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE1_WIREDANDPULLUPFILTER          0x0000000BUL                                          /**< Mode WIREDANDPULLUPFILTER for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE1_WIREDANDDRIVE                 0x0000000CUL                                          /**< Mode WIREDANDDRIVE for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE1_WIREDANDDRIVEFILTER           0x0000000DUL                                          /**< Mode WIREDANDDRIVEFILTER for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE1_WIREDANDDRIVEPULLUP           0x0000000EUL                                          /**< Mode WIREDANDDRIVEPULLUP for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE1_WIREDANDDRIVEPULLUPFILTER     0x0000000FUL                                          /**< Mode WIREDANDDRIVEPULLUPFILTER for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE1_DEFAULT                        (_GPIO_P_MODEL_MODE1_DEFAULT << 4)                    /**< Shifted mode DEFAULT for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE1_DISABLED                       (_GPIO_P_MODEL_MODE1_DISABLED << 4)                   /**< Shifted mode DISABLED for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE1_INPUT                          (_GPIO_P_MODEL_MODE1_INPUT << 4)                      /**< Shifted mode INPUT for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE1_INPUTPULL                      (_GPIO_P_MODEL_MODE1_INPUTPULL << 4)                  /**< Shifted mode INPUTPULL for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE1_INPUTPULLFILTER                (_GPIO_P_MODEL_MODE1_INPUTPULLFILTER << 4)            /**< Shifted mode INPUTPULLFILTER for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE1_PUSHPULL                       (_GPIO_P_MODEL_MODE1_PUSHPULL << 4)                   /**< Shifted mode PUSHPULL for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE1_PUSHPULLDRIVE                  (_GPIO_P_MODEL_MODE1_PUSHPULLDRIVE << 4)              /**< Shifted mode PUSHPULLDRIVE for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE1_WIREDOR                        (_GPIO_P_MODEL_MODE1_WIREDOR << 4)                    /**< Shifted mode WIREDOR for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE1_WIREDORPULLDOWN                (_GPIO_P_MODEL_MODE1_WIREDORPULLDOWN << 4)            /**< Shifted mode WIREDORPULLDOWN for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE1_WIREDAND                       (_GPIO_P_MODEL_MODE1_WIREDAND << 4)                   /**< Shifted mode WIREDAND for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE1_WIREDANDFILTER                 (_GPIO_P_MODEL_MODE1_WIREDANDFILTER << 4)             /**< Shifted mode WIREDANDFILTER for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE1_WIREDANDPULLUP                 (_GPIO_P_MODEL_MODE1_WIREDANDPULLUP << 4)             /**< Shifted mode WIREDANDPULLUP for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE1_WIREDANDPULLUPFILTER           (_GPIO_P_MODEL_MODE1_WIREDANDPULLUPFILTER << 4)       /**< Shifted mode WIREDANDPULLUPFILTER for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE1_WIREDANDDRIVE                  (_GPIO_P_MODEL_MODE1_WIREDANDDRIVE << 4)              /**< Shifted mode WIREDANDDRIVE for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE1_WIREDANDDRIVEFILTER            (_GPIO_P_MODEL_MODE1_WIREDANDDRIVEFILTER << 4)        /**< Shifted mode WIREDANDDRIVEFILTER for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE1_WIREDANDDRIVEPULLUP            (_GPIO_P_MODEL_MODE1_WIREDANDDRIVEPULLUP << 4)        /**< Shifted mode WIREDANDDRIVEPULLUP for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE1_WIREDANDDRIVEPULLUPFILTER      (_GPIO_P_MODEL_MODE1_WIREDANDDRIVEPULLUPFILTER << 4)  /**< Shifted mode WIREDANDDRIVEPULLUPFILTER for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE2_SHIFT                         8                                                     /**< Shift value for GPIO_MODE2 */
#define _GPIO_P_MODEL_MODE2_MASK                          0xF00UL                                               /**< Bit mask for GPIO_MODE2 */
#define _GPIO_P_MODEL_MODE2_DEFAULT                       0x00000000UL                                          /**< Mode DEFAULT for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE2_DISABLED                      0x00000000UL                                          /**< Mode DISABLED for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE2_INPUT                         0x00000001UL                                          /**< Mode INPUT for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE2_INPUTPULL                     0x00000002UL                                          /**< Mode INPUTPULL for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE2_INPUTPULLFILTER               0x00000003UL                                          /**< Mode INPUTPULLFILTER for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE2_PUSHPULL                      0x00000004UL                                          /**< Mode PUSHPULL for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE2_PUSHPULLDRIVE                 0x00000005UL                                          /**< Mode PUSHPULLDRIVE for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE2_WIREDOR                       0x00000006UL                                          /**< Mode WIREDOR for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE2_WIREDORPULLDOWN               0x00000007UL                                          /**< Mode WIREDORPULLDOWN for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE2_WIREDAND                      0x00000008UL                                          /**< Mode WIREDAND for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE2_WIREDANDFILTER                0x00000009UL                                          /**< Mode WIREDANDFILTER for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE2_WIREDANDPULLUP                0x0000000AUL                                          /**< Mode WIREDANDPULLUP for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE2_WIREDANDPULLUPFILTER          0x0000000BUL                                          /**< Mode WIREDANDPULLUPFILTER for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE2_WIREDANDDRIVE                 0x0000000CUL                                          /**< Mode WIREDANDDRIVE for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE2_WIREDANDDRIVEFILTER           0x0000000DUL                                          /**< Mode WIREDANDDRIVEFILTER for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE2_WIREDANDDRIVEPULLUP           0x0000000EUL                                          /**< Mode WIREDANDDRIVEPULLUP for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE2_WIREDANDDRIVEPULLUPFILTER     0x0000000FUL                                          /**< Mode WIREDANDDRIVEPULLUPFILTER for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE2_DEFAULT                        (_GPIO_P_MODEL_MODE2_DEFAULT << 8)                    /**< Shifted mode DEFAULT for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE2_DISABLED                       (_GPIO_P_MODEL_MODE2_DISABLED << 8)                   /**< Shifted mode DISABLED for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE2_INPUT                          (_GPIO_P_MODEL_MODE2_INPUT << 8)                      /**< Shifted mode INPUT for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE2_INPUTPULL                      (_GPIO_P_MODEL_MODE2_INPUTPULL << 8)                  /**< Shifted mode INPUTPULL for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE2_INPUTPULLFILTER                (_GPIO_P_MODEL_MODE2_INPUTPULLFILTER << 8)            /**< Shifted mode INPUTPULLFILTER for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE2_PUSHPULL                       (_GPIO_P_MODEL_MODE2_PUSHPULL << 8)                   /**< Shifted mode PUSHPULL for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE2_PUSHPULLDRIVE                  (_GPIO_P_MODEL_MODE2_PUSHPULLDRIVE << 8)              /**< Shifted mode PUSHPULLDRIVE for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE2_WIREDOR                        (_GPIO_P_MODEL_MODE2_WIREDOR << 8)                    /**< Shifted mode WIREDOR for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE2_WIREDORPULLDOWN                (_GPIO_P_MODEL_MODE2_WIREDORPULLDOWN << 8)            /**< Shifted mode WIREDORPULLDOWN for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE2_WIREDAND                       (_GPIO_P_MODEL_MODE2_WIREDAND << 8)                   /**< Shifted mode WIREDAND for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE2_WIREDANDFILTER                 (_GPIO_P_MODEL_MODE2_WIREDANDFILTER << 8)             /**< Shifted mode WIREDANDFILTER for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE2_WIREDANDPULLUP                 (_GPIO_P_MODEL_MODE2_WIREDANDPULLUP << 8)             /**< Shifted mode WIREDANDPULLUP for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE2_WIREDANDPULLUPFILTER           (_GPIO_P_MODEL_MODE2_WIREDANDPULLUPFILTER << 8)       /**< Shifted mode WIREDANDPULLUPFILTER for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE2_WIREDANDDRIVE                  (_GPIO_P_MODEL_MODE2_WIREDANDDRIVE << 8)              /**< Shifted mode WIREDANDDRIVE for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE2_WIREDANDDRIVEFILTER            (_GPIO_P_MODEL_MODE2_WIREDANDDRIVEFILTER << 8)        /**< Shifted mode WIREDANDDRIVEFILTER for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE2_WIREDANDDRIVEPULLUP            (_GPIO_P_MODEL_MODE2_WIREDANDDRIVEPULLUP << 8)        /**< Shifted mode WIREDANDDRIVEPULLUP for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE2_WIREDANDDRIVEPULLUPFILTER      (_GPIO_P_MODEL_MODE2_WIREDANDDRIVEPULLUPFILTER << 8)  /**< Shifted mode WIREDANDDRIVEPULLUPFILTER for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE3_SHIFT                         12                                                    /**< Shift value for GPIO_MODE3 */
#define _GPIO_P_MODEL_MODE3_MASK                          0xF000UL                                              /**< Bit mask for GPIO_MODE3 */
#define _GPIO_P_MODEL_MODE3_DEFAULT                       0x00000000UL                                          /**< Mode DEFAULT for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE3_DISABLED                      0x00000000UL                                          /**< Mode DISABLED for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE3_INPUT                         0x00000001UL                                          /**< Mode INPUT for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE3_INPUTPULL                     0x00000002UL                                          /**< Mode INPUTPULL for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE3_INPUTPULLFILTER               0x00000003UL                                          /**< Mode INPUTPULLFILTER for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE3_PUSHPULL                      0x00000004UL                                          /**< Mode PUSHPULL for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE3_PUSHPULLDRIVE                 0x00000005UL                                          /**< Mode PUSHPULLDRIVE for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE3_WIREDOR                       0x00000006UL                                          /**< Mode WIREDOR for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE3_WIREDORPULLDOWN               0x00000007UL                                          /**< Mode WIREDORPULLDOWN for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE3_WIREDAND                      0x00000008UL                                          /**< Mode WIREDAND for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE3_WIREDANDFILTER                0x00000009UL                                          /**< Mode WIREDANDFILTER for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE3_WIREDANDPULLUP                0x0000000AUL                                          /**< Mode WIREDANDPULLUP for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE3_WIREDANDPULLUPFILTER          0x0000000BUL                                          /**< Mode WIREDANDPULLUPFILTER for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE3_WIREDANDDRIVE                 0x0000000CUL                                          /**< Mode WIREDANDDRIVE for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE3_WIREDANDDRIVEFILTER           0x0000000DUL                                          /**< Mode WIREDANDDRIVEFILTER for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE3_WIREDANDDRIVEPULLUP           0x0000000EUL                                          /**< Mode WIREDANDDRIVEPULLUP for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE3_WIREDANDDRIVEPULLUPFILTER     0x0000000FUL                                          /**< Mode WIREDANDDRIVEPULLUPFILTER for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE3_DEFAULT                        (_GPIO_P_MODEL_MODE3_DEFAULT << 12)                   /**< Shifted mode DEFAULT for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE3_DISABLED                       (_GPIO_P_MODEL_MODE3_DISABLED << 12)                  /**< Shifted mode DISABLED for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE3_INPUT                          (_GPIO_P_MODEL_MODE3_INPUT << 12)                     /**< Shifted mode INPUT for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE3_INPUTPULL                      (_GPIO_P_MODEL_MODE3_INPUTPULL << 12)                 /**< Shifted mode INPUTPULL for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE3_INPUTPULLFILTER                (_GPIO_P_MODEL_MODE3_INPUTPULLFILTER << 12)           /**< Shifted mode INPUTPULLFILTER for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE3_PUSHPULL                       (_GPIO_P_MODEL_MODE3_PUSHPULL << 12)                  /**< Shifted mode PUSHPULL for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE3_PUSHPULLDRIVE                  (_GPIO_P_MODEL_MODE3_PUSHPULLDRIVE << 12)             /**< Shifted mode PUSHPULLDRIVE for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE3_WIREDOR                        (_GPIO_P_MODEL_MODE3_WIREDOR << 12)                   /**< Shifted mode WIREDOR for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE3_WIREDORPULLDOWN                (_GPIO_P_MODEL_MODE3_WIREDORPULLDOWN << 12)           /**< Shifted mode WIREDORPULLDOWN for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE3_WIREDAND                       (_GPIO_P_MODEL_MODE3_WIREDAND << 12)                  /**< Shifted mode WIREDAND for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE3_WIREDANDFILTER                 (_GPIO_P_MODEL_MODE3_WIREDANDFILTER << 12)            /**< Shifted mode WIREDANDFILTER for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE3_WIREDANDPULLUP                 (_GPIO_P_MODEL_MODE3_WIREDANDPULLUP << 12)            /**< Shifted mode WIREDANDPULLUP for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE3_WIREDANDPULLUPFILTER           (_GPIO_P_MODEL_MODE3_WIREDANDPULLUPFILTER << 12)      /**< Shifted mode WIREDANDPULLUPFILTER for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE3_WIREDANDDRIVE                  (_GPIO_P_MODEL_MODE3_WIREDANDDRIVE << 12)             /**< Shifted mode WIREDANDDRIVE for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE3_WIREDANDDRIVEFILTER            (_GPIO_P_MODEL_MODE3_WIREDANDDRIVEFILTER << 12)       /**< Shifted mode WIREDANDDRIVEFILTER for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE3_WIREDANDDRIVEPULLUP            (_GPIO_P_MODEL_MODE3_WIREDANDDRIVEPULLUP << 12)       /**< Shifted mode WIREDANDDRIVEPULLUP for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE3_WIREDANDDRIVEPULLUPFILTER      (_GPIO_P_MODEL_MODE3_WIREDANDDRIVEPULLUPFILTER << 12) /**< Shifted mode WIREDANDDRIVEPULLUPFILTER for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE4_SHIFT                         16                                                    /**< Shift value for GPIO_MODE4 */
#define _GPIO_P_MODEL_MODE4_MASK                          0xF0000UL                                             /**< Bit mask for GPIO_MODE4 */
#define _GPIO_P_MODEL_MODE4_DEFAULT                       0x00000000UL                                          /**< Mode DEFAULT for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE4_DISABLED                      0x00000000UL                                          /**< Mode DISABLED for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE4_INPUT                         0x00000001UL                                          /**< Mode INPUT for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE4_INPUTPULL                     0x00000002UL                                          /**< Mode INPUTPULL for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE4_INPUTPULLFILTER               0x00000003UL                                          /**< Mode INPUTPULLFILTER for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE4_PUSHPULL                      0x00000004UL                                          /**< Mode PUSHPULL for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE4_PUSHPULLDRIVE                 0x00000005UL                                          /**< Mode PUSHPULLDRIVE for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE4_WIREDOR                       0x00000006UL                                          /**< Mode WIREDOR for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE4_WIREDORPULLDOWN               0x00000007UL                                          /**< Mode WIREDORPULLDOWN for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE4_WIREDAND                      0x00000008UL                                          /**< Mode WIREDAND for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE4_WIREDANDFILTER                0x00000009UL                                          /**< Mode WIREDANDFILTER for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE4_WIREDANDPULLUP                0x0000000AUL                                          /**< Mode WIREDANDPULLUP for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE4_WIREDANDPULLUPFILTER          0x0000000BUL                                          /**< Mode WIREDANDPULLUPFILTER for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE4_WIREDANDDRIVE                 0x0000000CUL                                          /**< Mode WIREDANDDRIVE for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE4_WIREDANDDRIVEFILTER           0x0000000DUL                                          /**< Mode WIREDANDDRIVEFILTER for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE4_WIREDANDDRIVEPULLUP           0x0000000EUL                                          /**< Mode WIREDANDDRIVEPULLUP for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE4_WIREDANDDRIVEPULLUPFILTER     0x0000000FUL                                          /**< Mode WIREDANDDRIVEPULLUPFILTER for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE4_DEFAULT                        (_GPIO_P_MODEL_MODE4_DEFAULT << 16)                   /**< Shifted mode DEFAULT for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE4_DISABLED                       (_GPIO_P_MODEL_MODE4_DISABLED << 16)                  /**< Shifted mode DISABLED for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE4_INPUT                          (_GPIO_P_MODEL_MODE4_INPUT << 16)                     /**< Shifted mode INPUT for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE4_INPUTPULL                      (_GPIO_P_MODEL_MODE4_INPUTPULL << 16)                 /**< Shifted mode INPUTPULL for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE4_INPUTPULLFILTER                (_GPIO_P_MODEL_MODE4_INPUTPULLFILTER << 16)           /**< Shifted mode INPUTPULLFILTER for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE4_PUSHPULL                       (_GPIO_P_MODEL_MODE4_PUSHPULL << 16)                  /**< Shifted mode PUSHPULL for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE4_PUSHPULLDRIVE                  (_GPIO_P_MODEL_MODE4_PUSHPULLDRIVE << 16)             /**< Shifted mode PUSHPULLDRIVE for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE4_WIREDOR                        (_GPIO_P_MODEL_MODE4_WIREDOR << 16)                   /**< Shifted mode WIREDOR for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE4_WIREDORPULLDOWN                (_GPIO_P_MODEL_MODE4_WIREDORPULLDOWN << 16)           /**< Shifted mode WIREDORPULLDOWN for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE4_WIREDAND                       (_GPIO_P_MODEL_MODE4_WIREDAND << 16)                  /**< Shifted mode WIREDAND for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE4_WIREDANDFILTER                 (_GPIO_P_MODEL_MODE4_WIREDANDFILTER << 16)            /**< Shifted mode WIREDANDFILTER for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE4_WIREDANDPULLUP                 (_GPIO_P_MODEL_MODE4_WIREDANDPULLUP << 16)            /**< Shifted mode WIREDANDPULLUP for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE4_WIREDANDPULLUPFILTER           (_GPIO_P_MODEL_MODE4_WIREDANDPULLUPFILTER << 16)      /**< Shifted mode WIREDANDPULLUPFILTER for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE4_WIREDANDDRIVE                  (_GPIO_P_MODEL_MODE4_WIREDANDDRIVE << 16)             /**< Shifted mode WIREDANDDRIVE for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE4_WIREDANDDRIVEFILTER            (_GPIO_P_MODEL_MODE4_WIREDANDDRIVEFILTER << 16)       /**< Shifted mode WIREDANDDRIVEFILTER for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE4_WIREDANDDRIVEPULLUP            (_GPIO_P_MODEL_MODE4_WIREDANDDRIVEPULLUP << 16)       /**< Shifted mode WIREDANDDRIVEPULLUP for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE4_WIREDANDDRIVEPULLUPFILTER      (_GPIO_P_MODEL_MODE4_WIREDANDDRIVEPULLUPFILTER << 16) /**< Shifted mode WIREDANDDRIVEPULLUPFILTER for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE5_SHIFT                         20                                                    /**< Shift value for GPIO_MODE5 */
#define _GPIO_P_MODEL_MODE5_MASK                          0xF00000UL                                            /**< Bit mask for GPIO_MODE5 */
#define _GPIO_P_MODEL_MODE5_DEFAULT                       0x00000000UL                                          /**< Mode DEFAULT for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE5_DISABLED                      0x00000000UL                                          /**< Mode DISABLED for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE5_INPUT                         0x00000001UL                                          /**< Mode INPUT for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE5_INPUTPULL                     0x00000002UL                                          /**< Mode INPUTPULL for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE5_INPUTPULLFILTER               0x00000003UL                                          /**< Mode INPUTPULLFILTER for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE5_PUSHPULL                      0x00000004UL                                          /**< Mode PUSHPULL for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE5_PUSHPULLDRIVE                 0x00000005UL                                          /**< Mode PUSHPULLDRIVE for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE5_WIREDOR                       0x00000006UL                                          /**< Mode WIREDOR for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE5_WIREDORPULLDOWN               0x00000007UL                                          /**< Mode WIREDORPULLDOWN for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE5_WIREDAND                      0x00000008UL                                          /**< Mode WIREDAND for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE5_WIREDANDFILTER                0x00000009UL                                          /**< Mode WIREDANDFILTER for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE5_WIREDANDPULLUP                0x0000000AUL                                          /**< Mode WIREDANDPULLUP for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE5_WIREDANDPULLUPFILTER          0x0000000BUL                                          /**< Mode WIREDANDPULLUPFILTER for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE5_WIREDANDDRIVE                 0x0000000CUL                                          /**< Mode WIREDANDDRIVE for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE5_WIREDANDDRIVEFILTER           0x0000000DUL                                          /**< Mode WIREDANDDRIVEFILTER for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE5_WIREDANDDRIVEPULLUP           0x0000000EUL                                          /**< Mode WIREDANDDRIVEPULLUP for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE5_WIREDANDDRIVEPULLUPFILTER     0x0000000FUL                                          /**< Mode WIREDANDDRIVEPULLUPFILTER for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE5_DEFAULT                        (_GPIO_P_MODEL_MODE5_DEFAULT << 20)                   /**< Shifted mode DEFAULT for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE5_DISABLED                       (_GPIO_P_MODEL_MODE5_DISABLED << 20)                  /**< Shifted mode DISABLED for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE5_INPUT                          (_GPIO_P_MODEL_MODE5_INPUT << 20)                     /**< Shifted mode INPUT for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE5_INPUTPULL                      (_GPIO_P_MODEL_MODE5_INPUTPULL << 20)                 /**< Shifted mode INPUTPULL for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE5_INPUTPULLFILTER                (_GPIO_P_MODEL_MODE5_INPUTPULLFILTER << 20)           /**< Shifted mode INPUTPULLFILTER for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE5_PUSHPULL                       (_GPIO_P_MODEL_MODE5_PUSHPULL << 20)                  /**< Shifted mode PUSHPULL for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE5_PUSHPULLDRIVE                  (_GPIO_P_MODEL_MODE5_PUSHPULLDRIVE << 20)             /**< Shifted mode PUSHPULLDRIVE for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE5_WIREDOR                        (_GPIO_P_MODEL_MODE5_WIREDOR << 20)                   /**< Shifted mode WIREDOR for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE5_WIREDORPULLDOWN                (_GPIO_P_MODEL_MODE5_WIREDORPULLDOWN << 20)           /**< Shifted mode WIREDORPULLDOWN for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE5_WIREDAND                       (_GPIO_P_MODEL_MODE5_WIREDAND << 20)                  /**< Shifted mode WIREDAND for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE5_WIREDANDFILTER                 (_GPIO_P_MODEL_MODE5_WIREDANDFILTER << 20)            /**< Shifted mode WIREDANDFILTER for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE5_WIREDANDPULLUP                 (_GPIO_P_MODEL_MODE5_WIREDANDPULLUP << 20)            /**< Shifted mode WIREDANDPULLUP for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE5_WIREDANDPULLUPFILTER           (_GPIO_P_MODEL_MODE5_WIREDANDPULLUPFILTER << 20)      /**< Shifted mode WIREDANDPULLUPFILTER for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE5_WIREDANDDRIVE                  (_GPIO_P_MODEL_MODE5_WIREDANDDRIVE << 20)             /**< Shifted mode WIREDANDDRIVE for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE5_WIREDANDDRIVEFILTER            (_GPIO_P_MODEL_MODE5_WIREDANDDRIVEFILTER << 20)       /**< Shifted mode WIREDANDDRIVEFILTER for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE5_WIREDANDDRIVEPULLUP            (_GPIO_P_MODEL_MODE5_WIREDANDDRIVEPULLUP << 20)       /**< Shifted mode WIREDANDDRIVEPULLUP for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE5_WIREDANDDRIVEPULLUPFILTER      (_GPIO_P_MODEL_MODE5_WIREDANDDRIVEPULLUPFILTER << 20) /**< Shifted mode WIREDANDDRIVEPULLUPFILTER for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE6_SHIFT                         24                                                    /**< Shift value for GPIO_MODE6 */
#define _GPIO_P_MODEL_MODE6_MASK                          0xF000000UL                                           /**< Bit mask for GPIO_MODE6 */
#define _GPIO_P_MODEL_MODE6_DEFAULT                       0x00000000UL                                          /**< Mode DEFAULT for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE6_DISABLED                      0x00000000UL                                          /**< Mode DISABLED for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE6_INPUT                         0x00000001UL                                          /**< Mode INPUT for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE6_INPUTPULL                     0x00000002UL                                          /**< Mode INPUTPULL for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE6_INPUTPULLFILTER               0x00000003UL                                          /**< Mode INPUTPULLFILTER for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE6_PUSHPULL                      0x00000004UL                                          /**< Mode PUSHPULL for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE6_PUSHPULLDRIVE                 0x00000005UL                                          /**< Mode PUSHPULLDRIVE for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE6_WIREDOR                       0x00000006UL                                          /**< Mode WIREDOR for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE6_WIREDORPULLDOWN               0x00000007UL                                          /**< Mode WIREDORPULLDOWN for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE6_WIREDAND                      0x00000008UL                                          /**< Mode WIREDAND for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE6_WIREDANDFILTER                0x00000009UL                                          /**< Mode WIREDANDFILTER for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE6_WIREDANDPULLUP                0x0000000AUL                                          /**< Mode WIREDANDPULLUP for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE6_WIREDANDPULLUPFILTER          0x0000000BUL                                          /**< Mode WIREDANDPULLUPFILTER for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE6_WIREDANDDRIVE                 0x0000000CUL                                          /**< Mode WIREDANDDRIVE for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE6_WIREDANDDRIVEFILTER           0x0000000DUL                                          /**< Mode WIREDANDDRIVEFILTER for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE6_WIREDANDDRIVEPULLUP           0x0000000EUL                                          /**< Mode WIREDANDDRIVEPULLUP for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE6_WIREDANDDRIVEPULLUPFILTER     0x0000000FUL                                          /**< Mode WIREDANDDRIVEPULLUPFILTER for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE6_DEFAULT                        (_GPIO_P_MODEL_MODE6_DEFAULT << 24)                   /**< Shifted mode DEFAULT for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE6_DISABLED                       (_GPIO_P_MODEL_MODE6_DISABLED << 24)                  /**< Shifted mode DISABLED for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE6_INPUT                          (_GPIO_P_MODEL_MODE6_INPUT << 24)                     /**< Shifted mode INPUT for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE6_INPUTPULL                      (_GPIO_P_MODEL_MODE6_INPUTPULL << 24)                 /**< Shifted mode INPUTPULL for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE6_INPUTPULLFILTER                (_GPIO_P_MODEL_MODE6_INPUTPULLFILTER << 24)           /**< Shifted mode INPUTPULLFILTER for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE6_PUSHPULL                       (_GPIO_P_MODEL_MODE6_PUSHPULL << 24)                  /**< Shifted mode PUSHPULL for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE6_PUSHPULLDRIVE                  (_GPIO_P_MODEL_MODE6_PUSHPULLDRIVE << 24)             /**< Shifted mode PUSHPULLDRIVE for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE6_WIREDOR                        (_GPIO_P_MODEL_MODE6_WIREDOR << 24)                   /**< Shifted mode WIREDOR for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE6_WIREDORPULLDOWN                (_GPIO_P_MODEL_MODE6_WIREDORPULLDOWN << 24)           /**< Shifted mode WIREDORPULLDOWN for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE6_WIREDAND                       (_GPIO_P_MODEL_MODE6_WIREDAND << 24)                  /**< Shifted mode WIREDAND for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE6_WIREDANDFILTER                 (_GPIO_P_MODEL_MODE6_WIREDANDFILTER << 24)            /**< Shifted mode WIREDANDFILTER for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE6_WIREDANDPULLUP                 (_GPIO_P_MODEL_MODE6_WIREDANDPULLUP << 24)            /**< Shifted mode WIREDANDPULLUP for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE6_WIREDANDPULLUPFILTER           (_GPIO_P_MODEL_MODE6_WIREDANDPULLUPFILTER << 24)      /**< Shifted mode WIREDANDPULLUPFILTER for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE6_WIREDANDDRIVE                  (_GPIO_P_MODEL_MODE6_WIREDANDDRIVE << 24)             /**< Shifted mode WIREDANDDRIVE for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE6_WIREDANDDRIVEFILTER            (_GPIO_P_MODEL_MODE6_WIREDANDDRIVEFILTER << 24)       /**< Shifted mode WIREDANDDRIVEFILTER for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE6_WIREDANDDRIVEPULLUP            (_GPIO_P_MODEL_MODE6_WIREDANDDRIVEPULLUP << 24)       /**< Shifted mode WIREDANDDRIVEPULLUP for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE6_WIREDANDDRIVEPULLUPFILTER      (_GPIO_P_MODEL_MODE6_WIREDANDDRIVEPULLUPFILTER << 24) /**< Shifted mode WIREDANDDRIVEPULLUPFILTER for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE7_SHIFT                         28                                                    /**< Shift value for GPIO_MODE7 */
#define _GPIO_P_MODEL_MODE7_MASK                          0xF0000000UL                                          /**< Bit mask for GPIO_MODE7 */
#define _GPIO_P_MODEL_MODE7_DEFAULT                       0x00000000UL                                          /**< Mode DEFAULT for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE7_DISABLED                      0x00000000UL                                          /**< Mode DISABLED for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE7_INPUT                         0x00000001UL                                          /**< Mode INPUT for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE7_INPUTPULL                     0x00000002UL                                          /**< Mode INPUTPULL for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE7_INPUTPULLFILTER               0x00000003UL                                          /**< Mode INPUTPULLFILTER for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE7_PUSHPULL                      0x00000004UL                                          /**< Mode PUSHPULL for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE7_PUSHPULLDRIVE                 0x00000005UL                                          /**< Mode PUSHPULLDRIVE for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE7_WIREDOR                       0x00000006UL                                          /**< Mode WIREDOR for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE7_WIREDORPULLDOWN               0x00000007UL                                          /**< Mode WIREDORPULLDOWN for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE7_WIREDAND                      0x00000008UL                                          /**< Mode WIREDAND for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE7_WIREDANDFILTER                0x00000009UL                                          /**< Mode WIREDANDFILTER for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE7_WIREDANDPULLUP                0x0000000AUL                                          /**< Mode WIREDANDPULLUP for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE7_WIREDANDPULLUPFILTER          0x0000000BUL                                          /**< Mode WIREDANDPULLUPFILTER for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE7_WIREDANDDRIVE                 0x0000000CUL                                          /**< Mode WIREDANDDRIVE for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE7_WIREDANDDRIVEFILTER           0x0000000DUL                                          /**< Mode WIREDANDDRIVEFILTER for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE7_WIREDANDDRIVEPULLUP           0x0000000EUL                                          /**< Mode WIREDANDDRIVEPULLUP for GPIO_P_MODEL */
#define _GPIO_P_MODEL_MODE7_WIREDANDDRIVEPULLUPFILTER     0x0000000FUL                                          /**< Mode WIREDANDDRIVEPULLUPFILTER for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE7_DEFAULT                        (_GPIO_P_MODEL_MODE7_DEFAULT << 28)                   /**< Shifted mode DEFAULT for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE7_DISABLED                       (_GPIO_P_MODEL_MODE7_DISABLED << 28)                  /**< Shifted mode DISABLED for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE7_INPUT                          (_GPIO_P_MODEL_MODE7_INPUT << 28)                     /**< Shifted mode INPUT for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE7_INPUTPULL                      (_GPIO_P_MODEL_MODE7_INPUTPULL << 28)                 /**< Shifted mode INPUTPULL for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE7_INPUTPULLFILTER                (_GPIO_P_MODEL_MODE7_INPUTPULLFILTER << 28)           /**< Shifted mode INPUTPULLFILTER for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE7_PUSHPULL                       (_GPIO_P_MODEL_MODE7_PUSHPULL << 28)                  /**< Shifted mode PUSHPULL for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE7_PUSHPULLDRIVE                  (_GPIO_P_MODEL_MODE7_PUSHPULLDRIVE << 28)             /**< Shifted mode PUSHPULLDRIVE for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE7_WIREDOR                        (_GPIO_P_MODEL_MODE7_WIREDOR << 28)                   /**< Shifted mode WIREDOR for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE7_WIREDORPULLDOWN                (_GPIO_P_MODEL_MODE7_WIREDORPULLDOWN << 28)           /**< Shifted mode WIREDORPULLDOWN for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE7_WIREDAND                       (_GPIO_P_MODEL_MODE7_WIREDAND << 28)                  /**< Shifted mode WIREDAND for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE7_WIREDANDFILTER                 (_GPIO_P_MODEL_MODE7_WIREDANDFILTER << 28)            /**< Shifted mode WIREDANDFILTER for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE7_WIREDANDPULLUP                 (_GPIO_P_MODEL_MODE7_WIREDANDPULLUP << 28)            /**< Shifted mode WIREDANDPULLUP for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE7_WIREDANDPULLUPFILTER           (_GPIO_P_MODEL_MODE7_WIREDANDPULLUPFILTER << 28)      /**< Shifted mode WIREDANDPULLUPFILTER for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE7_WIREDANDDRIVE                  (_GPIO_P_MODEL_MODE7_WIREDANDDRIVE << 28)             /**< Shifted mode WIREDANDDRIVE for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE7_WIREDANDDRIVEFILTER            (_GPIO_P_MODEL_MODE7_WIREDANDDRIVEFILTER << 28)       /**< Shifted mode WIREDANDDRIVEFILTER for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE7_WIREDANDDRIVEPULLUP            (_GPIO_P_MODEL_MODE7_WIREDANDDRIVEPULLUP << 28)       /**< Shifted mode WIREDANDDRIVEPULLUP for GPIO_P_MODEL */
#define GPIO_P_MODEL_MODE7_WIREDANDDRIVEPULLUPFILTER      (_GPIO_P_MODEL_MODE7_WIREDANDDRIVEPULLUPFILTER << 28) /**< Shifted mode WIREDANDDRIVEPULLUPFILTER for GPIO_P_MODEL */

/* Bit fields for GPIO P_MODEH */
#define _GPIO_P_MODEH_RESETVALUE                          0x00000000UL                                           /**< Default value for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MASK                                0xFFFFFFFFUL                                           /**< Mask for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE8_SHIFT                         0                                                      /**< Shift value for GPIO_MODE8 */
#define _GPIO_P_MODEH_MODE8_MASK                          0xFUL                                                  /**< Bit mask for GPIO_MODE8 */
#define _GPIO_P_MODEH_MODE8_DEFAULT                       0x00000000UL                                           /**< Mode DEFAULT for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE8_DISABLED                      0x00000000UL                                           /**< Mode DISABLED for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE8_INPUT                         0x00000001UL                                           /**< Mode INPUT for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE8_INPUTPULL                     0x00000002UL                                           /**< Mode INPUTPULL for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE8_INPUTPULLFILTER               0x00000003UL                                           /**< Mode INPUTPULLFILTER for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE8_PUSHPULL                      0x00000004UL                                           /**< Mode PUSHPULL for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE8_PUSHPULLDRIVE                 0x00000005UL                                           /**< Mode PUSHPULLDRIVE for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE8_WIREDOR                       0x00000006UL                                           /**< Mode WIREDOR for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE8_WIREDORPULLDOWN               0x00000007UL                                           /**< Mode WIREDORPULLDOWN for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE8_WIREDAND                      0x00000008UL                                           /**< Mode WIREDAND for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE8_WIREDANDFILTER                0x00000009UL                                           /**< Mode WIREDANDFILTER for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE8_WIREDANDPULLUP                0x0000000AUL                                           /**< Mode WIREDANDPULLUP for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE8_WIREDANDPULLUPFILTER          0x0000000BUL                                           /**< Mode WIREDANDPULLUPFILTER for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE8_WIREDANDDRIVE                 0x0000000CUL                                           /**< Mode WIREDANDDRIVE for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE8_WIREDANDDRIVEFILTER           0x0000000DUL                                           /**< Mode WIREDANDDRIVEFILTER for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE8_WIREDANDDRIVEPULLUP           0x0000000EUL                                           /**< Mode WIREDANDDRIVEPULLUP for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE8_WIREDANDDRIVEPULLUPFILTER     0x0000000FUL                                           /**< Mode WIREDANDDRIVEPULLUPFILTER for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE8_DEFAULT                        (_GPIO_P_MODEH_MODE8_DEFAULT << 0)                     /**< Shifted mode DEFAULT for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE8_DISABLED                       (_GPIO_P_MODEH_MODE8_DISABLED << 0)                    /**< Shifted mode DISABLED for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE8_INPUT                          (_GPIO_P_MODEH_MODE8_INPUT << 0)                       /**< Shifted mode INPUT for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE8_INPUTPULL                      (_GPIO_P_MODEH_MODE8_INPUTPULL << 0)                   /**< Shifted mode INPUTPULL for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE8_INPUTPULLFILTER                (_GPIO_P_MODEH_MODE8_INPUTPULLFILTER << 0)             /**< Shifted mode INPUTPULLFILTER for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE8_PUSHPULL                       (_GPIO_P_MODEH_MODE8_PUSHPULL << 0)                    /**< Shifted mode PUSHPULL for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE8_PUSHPULLDRIVE                  (_GPIO_P_MODEH_MODE8_PUSHPULLDRIVE << 0)               /**< Shifted mode PUSHPULLDRIVE for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE8_WIREDOR                        (_GPIO_P_MODEH_MODE8_WIREDOR << 0)                     /**< Shifted mode WIREDOR for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE8_WIREDORPULLDOWN                (_GPIO_P_MODEH_MODE8_WIREDORPULLDOWN << 0)             /**< Shifted mode WIREDORPULLDOWN for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE8_WIREDAND                       (_GPIO_P_MODEH_MODE8_WIREDAND << 0)                    /**< Shifted mode WIREDAND for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE8_WIREDANDFILTER                 (_GPIO_P_MODEH_MODE8_WIREDANDFILTER << 0)              /**< Shifted mode WIREDANDFILTER for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE8_WIREDANDPULLUP                 (_GPIO_P_MODEH_MODE8_WIREDANDPULLUP << 0)              /**< Shifted mode WIREDANDPULLUP for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE8_WIREDANDPULLUPFILTER           (_GPIO_P_MODEH_MODE8_WIREDANDPULLUPFILTER << 0)        /**< Shifted mode WIREDANDPULLUPFILTER for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE8_WIREDANDDRIVE                  (_GPIO_P_MODEH_MODE8_WIREDANDDRIVE << 0)               /**< Shifted mode WIREDANDDRIVE for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE8_WIREDANDDRIVEFILTER            (_GPIO_P_MODEH_MODE8_WIREDANDDRIVEFILTER << 0)         /**< Shifted mode WIREDANDDRIVEFILTER for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE8_WIREDANDDRIVEPULLUP            (_GPIO_P_MODEH_MODE8_WIREDANDDRIVEPULLUP << 0)         /**< Shifted mode WIREDANDDRIVEPULLUP for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE8_WIREDANDDRIVEPULLUPFILTER      (_GPIO_P_MODEH_MODE8_WIREDANDDRIVEPULLUPFILTER << 0)   /**< Shifted mode WIREDANDDRIVEPULLUPFILTER for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE9_SHIFT                         4                                                      /**< Shift value for GPIO_MODE9 */
#define _GPIO_P_MODEH_MODE9_MASK                          0xF0UL                                                 /**< Bit mask for GPIO_MODE9 */
#define _GPIO_P_MODEH_MODE9_DEFAULT                       0x00000000UL                                           /**< Mode DEFAULT for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE9_DISABLED                      0x00000000UL                                           /**< Mode DISABLED for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE9_INPUT                         0x00000001UL                                           /**< Mode INPUT for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE9_INPUTPULL                     0x00000002UL                                           /**< Mode INPUTPULL for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE9_INPUTPULLFILTER               0x00000003UL                                           /**< Mode INPUTPULLFILTER for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE9_PUSHPULL                      0x00000004UL                                           /**< Mode PUSHPULL for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE9_PUSHPULLDRIVE                 0x00000005UL                                           /**< Mode PUSHPULLDRIVE for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE9_WIREDOR                       0x00000006UL                                           /**< Mode WIREDOR for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE9_WIREDORPULLDOWN               0x00000007UL                                           /**< Mode WIREDORPULLDOWN for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE9_WIREDAND                      0x00000008UL                                           /**< Mode WIREDAND for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE9_WIREDANDFILTER                0x00000009UL                                           /**< Mode WIREDANDFILTER for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE9_WIREDANDPULLUP                0x0000000AUL                                           /**< Mode WIREDANDPULLUP for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE9_WIREDANDPULLUPFILTER          0x0000000BUL                                           /**< Mode WIREDANDPULLUPFILTER for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE9_WIREDANDDRIVE                 0x0000000CUL                                           /**< Mode WIREDANDDRIVE for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE9_WIREDANDDRIVEFILTER           0x0000000DUL                                           /**< Mode WIREDANDDRIVEFILTER for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE9_WIREDANDDRIVEPULLUP           0x0000000EUL                                           /**< Mode WIREDANDDRIVEPULLUP for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE9_WIREDANDDRIVEPULLUPFILTER     0x0000000FUL                                           /**< Mode WIREDANDDRIVEPULLUPFILTER for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE9_DEFAULT                        (_GPIO_P_MODEH_MODE9_DEFAULT << 4)                     /**< Shifted mode DEFAULT for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE9_DISABLED                       (_GPIO_P_MODEH_MODE9_DISABLED << 4)                    /**< Shifted mode DISABLED for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE9_INPUT                          (_GPIO_P_MODEH_MODE9_INPUT << 4)                       /**< Shifted mode INPUT for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE9_INPUTPULL                      (_GPIO_P_MODEH_MODE9_INPUTPULL << 4)                   /**< Shifted mode INPUTPULL for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE9_INPUTPULLFILTER                (_GPIO_P_MODEH_MODE9_INPUTPULLFILTER << 4)             /**< Shifted mode INPUTPULLFILTER for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE9_PUSHPULL                       (_GPIO_P_MODEH_MODE9_PUSHPULL << 4)                    /**< Shifted mode PUSHPULL for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE9_PUSHPULLDRIVE                  (_GPIO_P_MODEH_MODE9_PUSHPULLDRIVE << 4)               /**< Shifted mode PUSHPULLDRIVE for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE9_WIREDOR                        (_GPIO_P_MODEH_MODE9_WIREDOR << 4)                     /**< Shifted mode WIREDOR for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE9_WIREDORPULLDOWN                (_GPIO_P_MODEH_MODE9_WIREDORPULLDOWN << 4)             /**< Shifted mode WIREDORPULLDOWN for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE9_WIREDAND                       (_GPIO_P_MODEH_MODE9_WIREDAND << 4)                    /**< Shifted mode WIREDAND for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE9_WIREDANDFILTER                 (_GPIO_P_MODEH_MODE9_WIREDANDFILTER << 4)              /**< Shifted mode WIREDANDFILTER for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE9_WIREDANDPULLUP                 (_GPIO_P_MODEH_MODE9_WIREDANDPULLUP << 4)              /**< Shifted mode WIREDANDPULLUP for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE9_WIREDANDPULLUPFILTER           (_GPIO_P_MODEH_MODE9_WIREDANDPULLUPFILTER << 4)        /**< Shifted mode WIREDANDPULLUPFILTER for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE9_WIREDANDDRIVE                  (_GPIO_P_MODEH_MODE9_WIREDANDDRIVE << 4)               /**< Shifted mode WIREDANDDRIVE for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE9_WIREDANDDRIVEFILTER            (_GPIO_P_MODEH_MODE9_WIREDANDDRIVEFILTER << 4)         /**< Shifted mode WIREDANDDRIVEFILTER for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE9_WIREDANDDRIVEPULLUP            (_GPIO_P_MODEH_MODE9_WIREDANDDRIVEPULLUP << 4)         /**< Shifted mode WIREDANDDRIVEPULLUP for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE9_WIREDANDDRIVEPULLUPFILTER      (_GPIO_P_MODEH_MODE9_WIREDANDDRIVEPULLUPFILTER << 4)   /**< Shifted mode WIREDANDDRIVEPULLUPFILTER for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE10_SHIFT                        8                                                      /**< Shift value for GPIO_MODE10 */
#define _GPIO_P_MODEH_MODE10_MASK                         0xF00UL                                                /**< Bit mask for GPIO_MODE10 */
#define _GPIO_P_MODEH_MODE10_DEFAULT                      0x00000000UL                                           /**< Mode DEFAULT for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE10_DISABLED                     0x00000000UL                                           /**< Mode DISABLED for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE10_INPUT                        0x00000001UL                                           /**< Mode INPUT for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE10_INPUTPULL                    0x00000002UL                                           /**< Mode INPUTPULL for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE10_INPUTPULLFILTER              0x00000003UL                                           /**< Mode INPUTPULLFILTER for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE10_PUSHPULL                     0x00000004UL                                           /**< Mode PUSHPULL for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE10_PUSHPULLDRIVE                0x00000005UL                                           /**< Mode PUSHPULLDRIVE for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE10_WIREDOR                      0x00000006UL                                           /**< Mode WIREDOR for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE10_WIREDORPULLDOWN              0x00000007UL                                           /**< Mode WIREDORPULLDOWN for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE10_WIREDAND                     0x00000008UL                                           /**< Mode WIREDAND for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE10_WIREDANDFILTER               0x00000009UL                                           /**< Mode WIREDANDFILTER for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE10_WIREDANDPULLUP               0x0000000AUL                                           /**< Mode WIREDANDPULLUP for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE10_WIREDANDPULLUPFILTER         0x0000000BUL                                           /**< Mode WIREDANDPULLUPFILTER for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE10_WIREDANDDRIVE                0x0000000CUL                                           /**< Mode WIREDANDDRIVE for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE10_WIREDANDDRIVEFILTER          0x0000000DUL                                           /**< Mode WIREDANDDRIVEFILTER for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE10_WIREDANDDRIVEPULLUP          0x0000000EUL                                           /**< Mode WIREDANDDRIVEPULLUP for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE10_WIREDANDDRIVEPULLUPFILTER    0x0000000FUL                                           /**< Mode WIREDANDDRIVEPULLUPFILTER for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE10_DEFAULT                       (_GPIO_P_MODEH_MODE10_DEFAULT << 8)                    /**< Shifted mode DEFAULT for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE10_DISABLED                      (_GPIO_P_MODEH_MODE10_DISABLED << 8)                   /**< Shifted mode DISABLED for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE10_INPUT                         (_GPIO_P_MODEH_MODE10_INPUT << 8)                      /**< Shifted mode INPUT for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE10_INPUTPULL                     (_GPIO_P_MODEH_MODE10_INPUTPULL << 8)                  /**< Shifted mode INPUTPULL for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE10_INPUTPULLFILTER               (_GPIO_P_MODEH_MODE10_INPUTPULLFILTER << 8)            /**< Shifted mode INPUTPULLFILTER for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE10_PUSHPULL                      (_GPIO_P_MODEH_MODE10_PUSHPULL << 8)                   /**< Shifted mode PUSHPULL for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE10_PUSHPULLDRIVE                 (_GPIO_P_MODEH_MODE10_PUSHPULLDRIVE << 8)              /**< Shifted mode PUSHPULLDRIVE for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE10_WIREDOR                       (_GPIO_P_MODEH_MODE10_WIREDOR << 8)                    /**< Shifted mode WIREDOR for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE10_WIREDORPULLDOWN               (_GPIO_P_MODEH_MODE10_WIREDORPULLDOWN << 8)            /**< Shifted mode WIREDORPULLDOWN for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE10_WIREDAND                      (_GPIO_P_MODEH_MODE10_WIREDAND << 8)                   /**< Shifted mode WIREDAND for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE10_WIREDANDFILTER                (_GPIO_P_MODEH_MODE10_WIREDANDFILTER << 8)             /**< Shifted mode WIREDANDFILTER for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE10_WIREDANDPULLUP                (_GPIO_P_MODEH_MODE10_WIREDANDPULLUP << 8)             /**< Shifted mode WIREDANDPULLUP for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE10_WIREDANDPULLUPFILTER          (_GPIO_P_MODEH_MODE10_WIREDANDPULLUPFILTER << 8)       /**< Shifted mode WIREDANDPULLUPFILTER for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE10_WIREDANDDRIVE                 (_GPIO_P_MODEH_MODE10_WIREDANDDRIVE << 8)              /**< Shifted mode WIREDANDDRIVE for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE10_WIREDANDDRIVEFILTER           (_GPIO_P_MODEH_MODE10_WIREDANDDRIVEFILTER << 8)        /**< Shifted mode WIREDANDDRIVEFILTER for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE10_WIREDANDDRIVEPULLUP           (_GPIO_P_MODEH_MODE10_WIREDANDDRIVEPULLUP << 8)        /**< Shifted mode WIREDANDDRIVEPULLUP for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE10_WIREDANDDRIVEPULLUPFILTER     (_GPIO_P_MODEH_MODE10_WIREDANDDRIVEPULLUPFILTER << 8)  /**< Shifted mode WIREDANDDRIVEPULLUPFILTER for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE11_SHIFT                        12                                                     /**< Shift value for GPIO_MODE11 */
#define _GPIO_P_MODEH_MODE11_MASK                         0xF000UL                                               /**< Bit mask for GPIO_MODE11 */
#define _GPIO_P_MODEH_MODE11_DEFAULT                      0x00000000UL                                           /**< Mode DEFAULT for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE11_DISABLED                     0x00000000UL                                           /**< Mode DISABLED for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE11_INPUT                        0x00000001UL                                           /**< Mode INPUT for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE11_INPUTPULL                    0x00000002UL                                           /**< Mode INPUTPULL for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE11_INPUTPULLFILTER              0x00000003UL                                           /**< Mode INPUTPULLFILTER for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE11_PUSHPULL                     0x00000004UL                                           /**< Mode PUSHPULL for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE11_PUSHPULLDRIVE                0x00000005UL                                           /**< Mode PUSHPULLDRIVE for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE11_WIREDOR                      0x00000006UL                                           /**< Mode WIREDOR for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE11_WIREDORPULLDOWN              0x00000007UL                                           /**< Mode WIREDORPULLDOWN for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE11_WIREDAND                     0x00000008UL                                           /**< Mode WIREDAND for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE11_WIREDANDFILTER               0x00000009UL                                           /**< Mode WIREDANDFILTER for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE11_WIREDANDPULLUP               0x0000000AUL                                           /**< Mode WIREDANDPULLUP for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE11_WIREDANDPULLUPFILTER         0x0000000BUL                                           /**< Mode WIREDANDPULLUPFILTER for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE11_WIREDANDDRIVE                0x0000000CUL                                           /**< Mode WIREDANDDRIVE for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE11_WIREDANDDRIVEFILTER          0x0000000DUL                                           /**< Mode WIREDANDDRIVEFILTER for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE11_WIREDANDDRIVEPULLUP          0x0000000EUL                                           /**< Mode WIREDANDDRIVEPULLUP for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE11_WIREDANDDRIVEPULLUPFILTER    0x0000000FUL                                           /**< Mode WIREDANDDRIVEPULLUPFILTER for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE11_DEFAULT                       (_GPIO_P_MODEH_MODE11_DEFAULT << 12)                   /**< Shifted mode DEFAULT for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE11_DISABLED                      (_GPIO_P_MODEH_MODE11_DISABLED << 12)                  /**< Shifted mode DISABLED for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE11_INPUT                         (_GPIO_P_MODEH_MODE11_INPUT << 12)                     /**< Shifted mode INPUT for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE11_INPUTPULL                     (_GPIO_P_MODEH_MODE11_INPUTPULL << 12)                 /**< Shifted mode INPUTPULL for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE11_INPUTPULLFILTER               (_GPIO_P_MODEH_MODE11_INPUTPULLFILTER << 12)           /**< Shifted mode INPUTPULLFILTER for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE11_PUSHPULL                      (_GPIO_P_MODEH_MODE11_PUSHPULL << 12)                  /**< Shifted mode PUSHPULL for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE11_PUSHPULLDRIVE                 (_GPIO_P_MODEH_MODE11_PUSHPULLDRIVE << 12)             /**< Shifted mode PUSHPULLDRIVE for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE11_WIREDOR                       (_GPIO_P_MODEH_MODE11_WIREDOR << 12)                   /**< Shifted mode WIREDOR for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE11_WIREDORPULLDOWN               (_GPIO_P_MODEH_MODE11_WIREDORPULLDOWN << 12)           /**< Shifted mode WIREDORPULLDOWN for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE11_WIREDAND                      (_GPIO_P_MODEH_MODE11_WIREDAND << 12)                  /**< Shifted mode WIREDAND for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE11_WIREDANDFILTER                (_GPIO_P_MODEH_MODE11_WIREDANDFILTER << 12)            /**< Shifted mode WIREDANDFILTER for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE11_WIREDANDPULLUP                (_GPIO_P_MODEH_MODE11_WIREDANDPULLUP << 12)            /**< Shifted mode WIREDANDPULLUP for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE11_WIREDANDPULLUPFILTER          (_GPIO_P_MODEH_MODE11_WIREDANDPULLUPFILTER << 12)      /**< Shifted mode WIREDANDPULLUPFILTER for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE11_WIREDANDDRIVE                 (_GPIO_P_MODEH_MODE11_WIREDANDDRIVE << 12)             /**< Shifted mode WIREDANDDRIVE for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE11_WIREDANDDRIVEFILTER           (_GPIO_P_MODEH_MODE11_WIREDANDDRIVEFILTER << 12)       /**< Shifted mode WIREDANDDRIVEFILTER for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE11_WIREDANDDRIVEPULLUP           (_GPIO_P_MODEH_MODE11_WIREDANDDRIVEPULLUP << 12)       /**< Shifted mode WIREDANDDRIVEPULLUP for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE11_WIREDANDDRIVEPULLUPFILTER     (_GPIO_P_MODEH_MODE11_WIREDANDDRIVEPULLUPFILTER << 12) /**< Shifted mode WIREDANDDRIVEPULLUPFILTER for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE12_SHIFT                        16                                                     /**< Shift value for GPIO_MODE12 */
#define _GPIO_P_MODEH_MODE12_MASK                         0xF0000UL                                              /**< Bit mask for GPIO_MODE12 */
#define _GPIO_P_MODEH_MODE12_DEFAULT                      0x00000000UL                                           /**< Mode DEFAULT for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE12_DISABLED                     0x00000000UL                                           /**< Mode DISABLED for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE12_INPUT                        0x00000001UL                                           /**< Mode INPUT for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE12_INPUTPULL                    0x00000002UL                                           /**< Mode INPUTPULL for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE12_INPUTPULLFILTER              0x00000003UL                                           /**< Mode INPUTPULLFILTER for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE12_PUSHPULL                     0x00000004UL                                           /**< Mode PUSHPULL for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE12_PUSHPULLDRIVE                0x00000005UL                                           /**< Mode PUSHPULLDRIVE for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE12_WIREDOR                      0x00000006UL                                           /**< Mode WIREDOR for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE12_WIREDORPULLDOWN              0x00000007UL                                           /**< Mode WIREDORPULLDOWN for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE12_WIREDAND                     0x00000008UL                                           /**< Mode WIREDAND for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE12_WIREDANDFILTER               0x00000009UL                                           /**< Mode WIREDANDFILTER for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE12_WIREDANDPULLUP               0x0000000AUL                                           /**< Mode WIREDANDPULLUP for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE12_WIREDANDPULLUPFILTER         0x0000000BUL                                           /**< Mode WIREDANDPULLUPFILTER for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE12_WIREDANDDRIVE                0x0000000CUL                                           /**< Mode WIREDANDDRIVE for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE12_WIREDANDDRIVEFILTER          0x0000000DUL                                           /**< Mode WIREDANDDRIVEFILTER for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE12_WIREDANDDRIVEPULLUP          0x0000000EUL                                           /**< Mode WIREDANDDRIVEPULLUP for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE12_WIREDANDDRIVEPULLUPFILTER    0x0000000FUL                                           /**< Mode WIREDANDDRIVEPULLUPFILTER for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE12_DEFAULT                       (_GPIO_P_MODEH_MODE12_DEFAULT << 16)                   /**< Shifted mode DEFAULT for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE12_DISABLED                      (_GPIO_P_MODEH_MODE12_DISABLED << 16)                  /**< Shifted mode DISABLED for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE12_INPUT                         (_GPIO_P_MODEH_MODE12_INPUT << 16)                     /**< Shifted mode INPUT for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE12_INPUTPULL                     (_GPIO_P_MODEH_MODE12_INPUTPULL << 16)                 /**< Shifted mode INPUTPULL for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE12_INPUTPULLFILTER               (_GPIO_P_MODEH_MODE12_INPUTPULLFILTER << 16)           /**< Shifted mode INPUTPULLFILTER for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE12_PUSHPULL                      (_GPIO_P_MODEH_MODE12_PUSHPULL << 16)                  /**< Shifted mode PUSHPULL for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE12_PUSHPULLDRIVE                 (_GPIO_P_MODEH_MODE12_PUSHPULLDRIVE << 16)             /**< Shifted mode PUSHPULLDRIVE for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE12_WIREDOR                       (_GPIO_P_MODEH_MODE12_WIREDOR << 16)                   /**< Shifted mode WIREDOR for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE12_WIREDORPULLDOWN               (_GPIO_P_MODEH_MODE12_WIREDORPULLDOWN << 16)           /**< Shifted mode WIREDORPULLDOWN for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE12_WIREDAND                      (_GPIO_P_MODEH_MODE12_WIREDAND << 16)                  /**< Shifted mode WIREDAND for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE12_WIREDANDFILTER                (_GPIO_P_MODEH_MODE12_WIREDANDFILTER << 16)            /**< Shifted mode WIREDANDFILTER for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE12_WIREDANDPULLUP                (_GPIO_P_MODEH_MODE12_WIREDANDPULLUP << 16)            /**< Shifted mode WIREDANDPULLUP for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE12_WIREDANDPULLUPFILTER          (_GPIO_P_MODEH_MODE12_WIREDANDPULLUPFILTER << 16)      /**< Shifted mode WIREDANDPULLUPFILTER for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE12_WIREDANDDRIVE                 (_GPIO_P_MODEH_MODE12_WIREDANDDRIVE << 16)             /**< Shifted mode WIREDANDDRIVE for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE12_WIREDANDDRIVEFILTER           (_GPIO_P_MODEH_MODE12_WIREDANDDRIVEFILTER << 16)       /**< Shifted mode WIREDANDDRIVEFILTER for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE12_WIREDANDDRIVEPULLUP           (_GPIO_P_MODEH_MODE12_WIREDANDDRIVEPULLUP << 16)       /**< Shifted mode WIREDANDDRIVEPULLUP for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE12_WIREDANDDRIVEPULLUPFILTER     (_GPIO_P_MODEH_MODE12_WIREDANDDRIVEPULLUPFILTER << 16) /**< Shifted mode WIREDANDDRIVEPULLUPFILTER for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE13_SHIFT                        20                                                     /**< Shift value for GPIO_MODE13 */
#define _GPIO_P_MODEH_MODE13_MASK                         0xF00000UL                                             /**< Bit mask for GPIO_MODE13 */
#define _GPIO_P_MODEH_MODE13_DEFAULT                      0x00000000UL                                           /**< Mode DEFAULT for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE13_DISABLED                     0x00000000UL                                           /**< Mode DISABLED for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE13_INPUT                        0x00000001UL                                           /**< Mode INPUT for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE13_INPUTPULL                    0x00000002UL                                           /**< Mode INPUTPULL for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE13_INPUTPULLFILTER              0x00000003UL                                           /**< Mode INPUTPULLFILTER for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE13_PUSHPULL                     0x00000004UL                                           /**< Mode PUSHPULL for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE13_PUSHPULLDRIVE                0x00000005UL                                           /**< Mode PUSHPULLDRIVE for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE13_WIREDOR                      0x00000006UL                                           /**< Mode WIREDOR for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE13_WIREDORPULLDOWN              0x00000007UL                                           /**< Mode WIREDORPULLDOWN for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE13_WIREDAND                     0x00000008UL                                           /**< Mode WIREDAND for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE13_WIREDANDFILTER               0x00000009UL                                           /**< Mode WIREDANDFILTER for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE13_WIREDANDPULLUP               0x0000000AUL                                           /**< Mode WIREDANDPULLUP for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE13_WIREDANDPULLUPFILTER         0x0000000BUL                                           /**< Mode WIREDANDPULLUPFILTER for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE13_WIREDANDDRIVE                0x0000000CUL                                           /**< Mode WIREDANDDRIVE for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE13_WIREDANDDRIVEFILTER          0x0000000DUL                                           /**< Mode WIREDANDDRIVEFILTER for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE13_WIREDANDDRIVEPULLUP          0x0000000EUL                                           /**< Mode WIREDANDDRIVEPULLUP for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE13_WIREDANDDRIVEPULLUPFILTER    0x0000000FUL                                           /**< Mode WIREDANDDRIVEPULLUPFILTER for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE13_DEFAULT                       (_GPIO_P_MODEH_MODE13_DEFAULT << 20)                   /**< Shifted mode DEFAULT for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE13_DISABLED                      (_GPIO_P_MODEH_MODE13_DISABLED << 20)                  /**< Shifted mode DISABLED for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE13_INPUT                         (_GPIO_P_MODEH_MODE13_INPUT << 20)                     /**< Shifted mode INPUT for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE13_INPUTPULL                     (_GPIO_P_MODEH_MODE13_INPUTPULL << 20)                 /**< Shifted mode INPUTPULL for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE13_INPUTPULLFILTER               (_GPIO_P_MODEH_MODE13_INPUTPULLFILTER << 20)           /**< Shifted mode INPUTPULLFILTER for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE13_PUSHPULL                      (_GPIO_P_MODEH_MODE13_PUSHPULL << 20)                  /**< Shifted mode PUSHPULL for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE13_PUSHPULLDRIVE                 (_GPIO_P_MODEH_MODE13_PUSHPULLDRIVE << 20)             /**< Shifted mode PUSHPULLDRIVE for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE13_WIREDOR                       (_GPIO_P_MODEH_MODE13_WIREDOR << 20)                   /**< Shifted mode WIREDOR for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE13_WIREDORPULLDOWN               (_GPIO_P_MODEH_MODE13_WIREDORPULLDOWN << 20)           /**< Shifted mode WIREDORPULLDOWN for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE13_WIREDAND                      (_GPIO_P_MODEH_MODE13_WIREDAND << 20)                  /**< Shifted mode WIREDAND for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE13_WIREDANDFILTER                (_GPIO_P_MODEH_MODE13_WIREDANDFILTER << 20)            /**< Shifted mode WIREDANDFILTER for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE13_WIREDANDPULLUP                (_GPIO_P_MODEH_MODE13_WIREDANDPULLUP << 20)            /**< Shifted mode WIREDANDPULLUP for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE13_WIREDANDPULLUPFILTER          (_GPIO_P_MODEH_MODE13_WIREDANDPULLUPFILTER << 20)      /**< Shifted mode WIREDANDPULLUPFILTER for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE13_WIREDANDDRIVE                 (_GPIO_P_MODEH_MODE13_WIREDANDDRIVE << 20)             /**< Shifted mode WIREDANDDRIVE for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE13_WIREDANDDRIVEFILTER           (_GPIO_P_MODEH_MODE13_WIREDANDDRIVEFILTER << 20)       /**< Shifted mode WIREDANDDRIVEFILTER for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE13_WIREDANDDRIVEPULLUP           (_GPIO_P_MODEH_MODE13_WIREDANDDRIVEPULLUP << 20)       /**< Shifted mode WIREDANDDRIVEPULLUP for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE13_WIREDANDDRIVEPULLUPFILTER     (_GPIO_P_MODEH_MODE13_WIREDANDDRIVEPULLUPFILTER << 20) /**< Shifted mode WIREDANDDRIVEPULLUPFILTER for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE14_SHIFT                        24                                                     /**< Shift value for GPIO_MODE14 */
#define _GPIO_P_MODEH_MODE14_MASK                         0xF000000UL                                            /**< Bit mask for GPIO_MODE14 */
#define _GPIO_P_MODEH_MODE14_DEFAULT                      0x00000000UL                                           /**< Mode DEFAULT for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE14_DISABLED                     0x00000000UL                                           /**< Mode DISABLED for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE14_INPUT                        0x00000001UL                                           /**< Mode INPUT for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE14_INPUTPULL                    0x00000002UL                                           /**< Mode INPUTPULL for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE14_INPUTPULLFILTER              0x00000003UL                                           /**< Mode INPUTPULLFILTER for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE14_PUSHPULL                     0x00000004UL                                           /**< Mode PUSHPULL for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE14_PUSHPULLDRIVE                0x00000005UL                                           /**< Mode PUSHPULLDRIVE for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE14_WIREDOR                      0x00000006UL                                           /**< Mode WIREDOR for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE14_WIREDORPULLDOWN              0x00000007UL                                           /**< Mode WIREDORPULLDOWN for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE14_WIREDAND                     0x00000008UL                                           /**< Mode WIREDAND for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE14_WIREDANDFILTER               0x00000009UL                                           /**< Mode WIREDANDFILTER for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE14_WIREDANDPULLUP               0x0000000AUL                                           /**< Mode WIREDANDPULLUP for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE14_WIREDANDPULLUPFILTER         0x0000000BUL                                           /**< Mode WIREDANDPULLUPFILTER for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE14_WIREDANDDRIVE                0x0000000CUL                                           /**< Mode WIREDANDDRIVE for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE14_WIREDANDDRIVEFILTER          0x0000000DUL                                           /**< Mode WIREDANDDRIVEFILTER for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE14_WIREDANDDRIVEPULLUP          0x0000000EUL                                           /**< Mode WIREDANDDRIVEPULLUP for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE14_WIREDANDDRIVEPULLUPFILTER    0x0000000FUL                                           /**< Mode WIREDANDDRIVEPULLUPFILTER for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE14_DEFAULT                       (_GPIO_P_MODEH_MODE14_DEFAULT << 24)                   /**< Shifted mode DEFAULT for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE14_DISABLED                      (_GPIO_P_MODEH_MODE14_DISABLED << 24)                  /**< Shifted mode DISABLED for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE14_INPUT                         (_GPIO_P_MODEH_MODE14_INPUT << 24)                     /**< Shifted mode INPUT for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE14_INPUTPULL                     (_GPIO_P_MODEH_MODE14_INPUTPULL << 24)                 /**< Shifted mode INPUTPULL for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE14_INPUTPULLFILTER               (_GPIO_P_MODEH_MODE14_INPUTPULLFILTER << 24)           /**< Shifted mode INPUTPULLFILTER for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE14_PUSHPULL                      (_GPIO_P_MODEH_MODE14_PUSHPULL << 24)                  /**< Shifted mode PUSHPULL for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE14_PUSHPULLDRIVE                 (_GPIO_P_MODEH_MODE14_PUSHPULLDRIVE << 24)             /**< Shifted mode PUSHPULLDRIVE for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE14_WIREDOR                       (_GPIO_P_MODEH_MODE14_WIREDOR << 24)                   /**< Shifted mode WIREDOR for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE14_WIREDORPULLDOWN               (_GPIO_P_MODEH_MODE14_WIREDORPULLDOWN << 24)           /**< Shifted mode WIREDORPULLDOWN for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE14_WIREDAND                      (_GPIO_P_MODEH_MODE14_WIREDAND << 24)                  /**< Shifted mode WIREDAND for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE14_WIREDANDFILTER                (_GPIO_P_MODEH_MODE14_WIREDANDFILTER << 24)            /**< Shifted mode WIREDANDFILTER for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE14_WIREDANDPULLUP                (_GPIO_P_MODEH_MODE14_WIREDANDPULLUP << 24)            /**< Shifted mode WIREDANDPULLUP for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE14_WIREDANDPULLUPFILTER          (_GPIO_P_MODEH_MODE14_WIREDANDPULLUPFILTER << 24)      /**< Shifted mode WIREDANDPULLUPFILTER for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE14_WIREDANDDRIVE                 (_GPIO_P_MODEH_MODE14_WIREDANDDRIVE << 24)             /**< Shifted mode WIREDANDDRIVE for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE14_WIREDANDDRIVEFILTER           (_GPIO_P_MODEH_MODE14_WIREDANDDRIVEFILTER << 24)       /**< Shifted mode WIREDANDDRIVEFILTER for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE14_WIREDANDDRIVEPULLUP           (_GPIO_P_MODEH_MODE14_WIREDANDDRIVEPULLUP << 24)       /**< Shifted mode WIREDANDDRIVEPULLUP for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE14_WIREDANDDRIVEPULLUPFILTER     (_GPIO_P_MODEH_MODE14_WIREDANDDRIVEPULLUPFILTER << 24) /**< Shifted mode WIREDANDDRIVEPULLUPFILTER for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE15_SHIFT                        28                                                     /**< Shift value for GPIO_MODE15 */
#define _GPIO_P_MODEH_MODE15_MASK                         0xF0000000UL                                           /**< Bit mask for GPIO_MODE15 */
#define _GPIO_P_MODEH_MODE15_DEFAULT                      0x00000000UL                                           /**< Mode DEFAULT for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE15_DISABLED                     0x00000000UL                                           /**< Mode DISABLED for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE15_INPUT                        0x00000001UL                                           /**< Mode INPUT for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE15_INPUTPULL                    0x00000002UL                                           /**< Mode INPUTPULL for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE15_INPUTPULLFILTER              0x00000003UL                                           /**< Mode INPUTPULLFILTER for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE15_PUSHPULL                     0x00000004UL                                           /**< Mode PUSHPULL for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE15_PUSHPULLDRIVE                0x00000005UL                                           /**< Mode PUSHPULLDRIVE for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE15_WIREDOR                      0x00000006UL                                           /**< Mode WIREDOR for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE15_WIREDORPULLDOWN              0x00000007UL                                           /**< Mode WIREDORPULLDOWN for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE15_WIREDAND                     0x00000008UL                                           /**< Mode WIREDAND for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE15_WIREDANDFILTER               0x00000009UL                                           /**< Mode WIREDANDFILTER for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE15_WIREDANDPULLUP               0x0000000AUL                                           /**< Mode WIREDANDPULLUP for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE15_WIREDANDPULLUPFILTER         0x0000000BUL                                           /**< Mode WIREDANDPULLUPFILTER for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE15_WIREDANDDRIVE                0x0000000CUL                                           /**< Mode WIREDANDDRIVE for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE15_WIREDANDDRIVEFILTER          0x0000000DUL                                           /**< Mode WIREDANDDRIVEFILTER for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE15_WIREDANDDRIVEPULLUP          0x0000000EUL                                           /**< Mode WIREDANDDRIVEPULLUP for GPIO_P_MODEH */
#define _GPIO_P_MODEH_MODE15_WIREDANDDRIVEPULLUPFILTER    0x0000000FUL                                           /**< Mode WIREDANDDRIVEPULLUPFILTER for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE15_DEFAULT                       (_GPIO_P_MODEH_MODE15_DEFAULT << 28)                   /**< Shifted mode DEFAULT for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE15_DISABLED                      (_GPIO_P_MODEH_MODE15_DISABLED << 28)                  /**< Shifted mode DISABLED for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE15_INPUT                         (_GPIO_P_MODEH_MODE15_INPUT << 28)                     /**< Shifted mode INPUT for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE15_INPUTPULL                     (_GPIO_P_MODEH_MODE15_INPUTPULL << 28)                 /**< Shifted mode INPUTPULL for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE15_INPUTPULLFILTER               (_GPIO_P_MODEH_MODE15_INPUTPULLFILTER << 28)           /**< Shifted mode INPUTPULLFILTER for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE15_PUSHPULL                      (_GPIO_P_MODEH_MODE15_PUSHPULL << 28)                  /**< Shifted mode PUSHPULL for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE15_PUSHPULLDRIVE                 (_GPIO_P_MODEH_MODE15_PUSHPULLDRIVE << 28)             /**< Shifted mode PUSHPULLDRIVE for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE15_WIREDOR                       (_GPIO_P_MODEH_MODE15_WIREDOR << 28)                   /**< Shifted mode WIREDOR for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE15_WIREDORPULLDOWN               (_GPIO_P_MODEH_MODE15_WIREDORPULLDOWN << 28)           /**< Shifted mode WIREDORPULLDOWN for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE15_WIREDAND                      (_GPIO_P_MODEH_MODE15_WIREDAND << 28)                  /**< Shifted mode WIREDAND for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE15_WIREDANDFILTER                (_GPIO_P_MODEH_MODE15_WIREDANDFILTER << 28)            /**< Shifted mode WIREDANDFILTER for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE15_WIREDANDPULLUP                (_GPIO_P_MODEH_MODE15_WIREDANDPULLUP << 28)            /**< Shifted mode WIREDANDPULLUP for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE15_WIREDANDPULLUPFILTER          (_GPIO_P_MODEH_MODE15_WIREDANDPULLUPFILTER << 28)      /**< Shifted mode WIREDANDPULLUPFILTER for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE15_WIREDANDDRIVE                 (_GPIO_P_MODEH_MODE15_WIREDANDDRIVE << 28)             /**< Shifted mode WIREDANDDRIVE for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE15_WIREDANDDRIVEFILTER           (_GPIO_P_MODEH_MODE15_WIREDANDDRIVEFILTER << 28)       /**< Shifted mode WIREDANDDRIVEFILTER for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE15_WIREDANDDRIVEPULLUP           (_GPIO_P_MODEH_MODE15_WIREDANDDRIVEPULLUP << 28)       /**< Shifted mode WIREDANDDRIVEPULLUP for GPIO_P_MODEH */
#define GPIO_P_MODEH_MODE15_WIREDANDDRIVEPULLUPFILTER     (_GPIO_P_MODEH_MODE15_WIREDANDDRIVEPULLUPFILTER << 28) /**< Shifted mode WIREDANDDRIVEPULLUPFILTER for GPIO_P_MODEH */

/* Bit fields for GPIO P_DOUT */
#define _GPIO_P_DOUT_RESETVALUE                           0x00000000UL                     /**< Default value for GPIO_P_DOUT */
#define _GPIO_P_DOUT_MASK                                 0x0000FFFFUL                     /**< Mask for GPIO_P_DOUT */
#define _GPIO_P_DOUT_DOUT_SHIFT                           0                                /**< Shift value for GPIO_DOUT */
#define _GPIO_P_DOUT_DOUT_MASK                            0xFFFFUL                         /**< Bit mask for GPIO_DOUT */
#define _GPIO_P_DOUT_DOUT_DEFAULT                         0x00000000UL                     /**< Mode DEFAULT for GPIO_P_DOUT */
#define GPIO_P_DOUT_DOUT_DEFAULT                          (_GPIO_P_DOUT_DOUT_DEFAULT << 0) /**< Shifted mode DEFAULT for GPIO_P_DOUT */

/* Bit fields for GPIO P_DOUTSET */
#define _GPIO_P_DOUTSET_RESETVALUE                        0x00000000UL                           /**< Default value for GPIO_P_DOUTSET */
#define _GPIO_P_DOUTSET_MASK                              0x0000FFFFUL                           /**< Mask for GPIO_P_DOUTSET */
#define _GPIO_P_DOUTSET_DOUTSET_SHIFT                     0                                      /**< Shift value for GPIO_DOUTSET */
#define _GPIO_P_DOUTSET_DOUTSET_MASK                      0xFFFFUL                               /**< Bit mask for GPIO_DOUTSET */
#define _GPIO_P_DOUTSET_DOUTSET_DEFAULT                   0x00000000UL                           /**< Mode DEFAULT for GPIO_P_DOUTSET */
#define GPIO_P_DOUTSET_DOUTSET_DEFAULT                    (_GPIO_P_DOUTSET_DOUTSET_DEFAULT << 0) /**< Shifted mode DEFAULT for GPIO_P_DOUTSET */

/* Bit fields for GPIO P_DOUTCLR */
#define _GPIO_P_DOUTCLR_RESETVALUE                        0x00000000UL                           /**< Default value for GPIO_P_DOUTCLR */
#define _GPIO_P_DOUTCLR_MASK                              0x0000FFFFUL                           /**< Mask for GPIO_P_DOUTCLR */
#define _GPIO_P_DOUTCLR_DOUTCLR_SHIFT                     0                                      /**< Shift value for GPIO_DOUTCLR */
#define _GPIO_P_DOUTCLR_DOUTCLR_MASK                      0xFFFFUL                               /**< Bit mask for GPIO_DOUTCLR */
#define _GPIO_P_DOUTCLR_DOUTCLR_DEFAULT                   0x00000000UL                           /**< Mode DEFAULT for GPIO_P_DOUTCLR */
#define GPIO_P_DOUTCLR_DOUTCLR_DEFAULT                    (_GPIO_P_DOUTCLR_DOUTCLR_DEFAULT << 0) /**< Shifted mode DEFAULT for GPIO_P_DOUTCLR */

/* Bit fields for GPIO P_DOUTTGL */
#define _GPIO_P_DOUTTGL_RESETVALUE                        0x00000000UL                           /**< Default value for GPIO_P_DOUTTGL */
#define _GPIO_P_DOUTTGL_MASK                              0x0000FFFFUL                           /**< Mask for GPIO_P_DOUTTGL */
#define _GPIO_P_DOUTTGL_DOUTTGL_SHIFT                     0                                      /**< Shift value for GPIO_DOUTTGL */
#define _GPIO_P_DOUTTGL_DOUTTGL_MASK                      0xFFFFUL                               /**< Bit mask for GPIO_DOUTTGL */
#define _GPIO_P_DOUTTGL_DOUTTGL_DEFAULT                   0x00000000UL                           /**< Mode DEFAULT for GPIO_P_DOUTTGL */
#define GPIO_P_DOUTTGL_DOUTTGL_DEFAULT                    (_GPIO_P_DOUTTGL_DOUTTGL_DEFAULT << 0) /**< Shifted mode DEFAULT for GPIO_P_DOUTTGL */

/* Bit fields for GPIO P_DIN */
#define _GPIO_P_DIN_RESETVALUE                            0x00000000UL                   /**< Default value for GPIO_P_DIN */
#define _GPIO_P_DIN_MASK                                  0x0000FFFFUL                   /**< Mask for GPIO_P_DIN */
#define _GPIO_P_DIN_DIN_SHIFT                             0                              /**< Shift value for GPIO_DIN */
#define _GPIO_P_DIN_DIN_MASK                              0xFFFFUL                       /**< Bit mask for GPIO_DIN */
#define _GPIO_P_DIN_DIN_DEFAULT                           0x00000000UL                   /**< Mode DEFAULT for GPIO_P_DIN */
#define GPIO_P_DIN_DIN_DEFAULT                            (_GPIO_P_DIN_DIN_DEFAULT << 0) /**< Shifted mode DEFAULT for GPIO_P_DIN */

/* Bit fields for GPIO P_PINLOCKN */
#define _GPIO_P_PINLOCKN_RESETVALUE                       0x0000FFFFUL                             /**< Default value for GPIO_P_PINLOCKN */
#define _GPIO_P_PINLOCKN_MASK                             0x0000FFFFUL                             /**< Mask for GPIO_P_PINLOCKN */
#define _GPIO_P_PINLOCKN_PINLOCKN_SHIFT                   0                                        /**< Shift value for GPIO_PINLOCKN */
#define _GPIO_P_PINLOCKN_PINLOCKN_MASK                    0xFFFFUL                                 /**< Bit mask for GPIO_PINLOCKN */
#define _GPIO_P_PINLOCKN_PINLOCKN_DEFAULT                 0x0000FFFFUL                             /**< Mode DEFAULT for GPIO_P_PINLOCKN */
#define GPIO_P_PINLOCKN_PINLOCKN_DEFAULT                  (_GPIO_P_PINLOCKN_PINLOCKN_DEFAULT << 0) /**< Shifted mode DEFAULT for GPIO_P_PINLOCKN */

/* Bit fields for GPIO EXTIPSELL */
#define _GPIO_EXTIPSELL_RESETVALUE                        0x00000000UL                              /**< Default value for GPIO_EXTIPSELL */
#define _GPIO_EXTIPSELL_MASK                              0x77777777UL                              /**< Mask for GPIO_EXTIPSELL */
#define _GPIO_EXTIPSELL_EXTIPSEL0_SHIFT                   0                                         /**< Shift value for GPIO_EXTIPSEL0 */
#define _GPIO_EXTIPSELL_EXTIPSEL0_MASK                    0x7UL                                     /**< Bit mask for GPIO_EXTIPSEL0 */
#define _GPIO_EXTIPSELL_EXTIPSEL0_DEFAULT                 0x00000000UL                              /**< Mode DEFAULT for GPIO_EXTIPSELL */
#define _GPIO_EXTIPSELL_EXTIPSEL0_PORTA                   0x00000000UL                              /**< Mode PORTA for GPIO_EXTIPSELL */
#define _GPIO_EXTIPSELL_EXTIPSEL0_PORTB                   0x00000001UL                              /**< Mode PORTB for GPIO_EXTIPSELL */
#define _GPIO_EXTIPSELL_EXTIPSEL0_PORTC                   0x00000002UL                              /**< Mode PORTC for GPIO_EXTIPSELL */
#define _GPIO_EXTIPSELL_EXTIPSEL0_PORTD                   0x00000003UL                              /**< Mode PORTD for GPIO_EXTIPSELL */
#define _GPIO_EXTIPSELL_EXTIPSEL0_PORTE                   0x00000004UL                              /**< Mode PORTE for GPIO_EXTIPSELL */
#define _GPIO_EXTIPSELL_EXTIPSEL0_PORTF                   0x00000005UL                              /**< Mode PORTF for GPIO_EXTIPSELL */
#define GPIO_EXTIPSELL_EXTIPSEL0_DEFAULT                  (_GPIO_EXTIPSELL_EXTIPSEL0_DEFAULT << 0)  /**< Shifted mode DEFAULT for GPIO_EXTIPSELL */
#define GPIO_EXTIPSELL_EXTIPSEL0_PORTA                    (_GPIO_EXTIPSELL_EXTIPSEL0_PORTA << 0)    /**< Shifted mode PORTA for GPIO_EXTIPSELL */
#define GPIO_EXTIPSELL_EXTIPSEL0_PORTB                    (_GPIO_EXTIPSELL_EXTIPSEL0_PORTB << 0)    /**< Shifted mode PORTB for GPIO_EXTIPSELL */
#define GPIO_EXTIPSELL_EXTIPSEL0_PORTC                    (_GPIO_EXTIPSELL_EXTIPSEL0_PORTC << 0)    /**< Shifted mode PORTC for GPIO_EXTIPSELL */
#define GPIO_EXTIPSELL_EXTIPSEL0_PORTD                    (_GPIO_EXTIPSELL_EXTIPSEL0_PORTD << 0)    /**< Shifted mode PORTD for GPIO_EXTIPSELL */
#define GPIO_EXTIPSELL_EXTIPSEL0_PORTE                    (_GPIO_EXTIPSELL_EXTIPSEL0_PORTE << 0)    /**< Shifted mode PORTE for GPIO_EXTIPSELL */
#define GPIO_EXTIPSELL_EXTIPSEL0_PORTF                    (_GPIO_EXTIPSELL_EXTIPSEL0_PORTF << 0)    /**< Shifted mode PORTF for GPIO_EXTIPSELL */
#define _GPIO_EXTIPSELL_EXTIPSEL1_SHIFT                   4                                         /**< Shift value for GPIO_EXTIPSEL1 */
#define _GPIO_EXTIPSELL_EXTIPSEL1_MASK                    0x70UL                                    /**< Bit mask for GPIO_EXTIPSEL1 */
#define _GPIO_EXTIPSELL_EXTIPSEL1_DEFAULT                 0x00000000UL                              /**< Mode DEFAULT for GPIO_EXTIPSELL */
#define _GPIO_EXTIPSELL_EXTIPSEL1_PORTA                   0x00000000UL                              /**< Mode PORTA for GPIO_EXTIPSELL */
#define _GPIO_EXTIPSELL_EXTIPSEL1_PORTB                   0x00000001UL                              /**< Mode PORTB for GPIO_EXTIPSELL */
#define _GPIO_EXTIPSELL_EXTIPSEL1_PORTC                   0x00000002UL                              /**< Mode PORTC for GPIO_EXTIPSELL */
#define _GPIO_EXTIPSELL_EXTIPSEL1_PORTD                   0x00000003UL                              /**< Mode PORTD for GPIO_EXTIPSELL */
#define _GPIO_EXTIPSELL_EXTIPSEL1_PORTE                   0x00000004UL                              /**< Mode PORTE for GPIO_EXTIPSELL */
#define _GPIO_EXTIPSELL_EXTIPSEL1_PORTF                   0x00000005UL                              /**< Mode PORTF for GPIO_EXTIPSELL */
#define GPIO_EXTIPSELL_EXTIPSEL1_DEFAULT                  (_GPIO_EXTIPSELL_EXTIPSEL1_DEFAULT << 4)  /**< Shifted mode DEFAULT for GPIO_EXTIPSELL */
#define GPIO_EXTIPSELL_EXTIPSEL1_PORTA                    (_GPIO_EXTIPSELL_EXTIPSEL1_PORTA << 4)    /**< Shifted mode PORTA for GPIO_EXTIPSELL */
#define GPIO_EXTIPSELL_EXTIPSEL1_PORTB                    (_GPIO_EXTIPSELL_EXTIPSEL1_PORTB << 4)    /**< Shifted mode PORTB for GPIO_EXTIPSELL */
#define GPIO_EXTIPSELL_EXTIPSEL1_PORTC                    (_GPIO_EXTIPSELL_EXTIPSEL1_PORTC << 4)    /**< Shifted mode PORTC for GPIO_EXTIPSELL */
#define GPIO_EXTIPSELL_EXTIPSEL1_PORTD                    (_GPIO_EXTIPSELL_EXTIPSEL1_PORTD << 4)    /**< Shifted mode PORTD for GPIO_EXTIPSELL */
#define GPIO_EXTIPSELL_EXTIPSEL1_PORTE                    (_GPIO_EXTIPSELL_EXTIPSEL1_PORTE << 4)    /**< Shifted mode PORTE for GPIO_EXTIPSELL */
#define GPIO_EXTIPSELL_EXTIPSEL1_PORTF                    (_GPIO_EXTIPSELL_EXTIPSEL1_PORTF << 4)    /**< Shifted mode PORTF for GPIO_EXTIPSELL */
#define _GPIO_EXTIPSELL_EXTIPSEL2_SHIFT                   8                                         /**< Shift value for GPIO_EXTIPSEL2 */
#define _GPIO_EXTIPSELL_EXTIPSEL2_MASK                    0x700UL                                   /**< Bit mask for GPIO_EXTIPSEL2 */
#define _GPIO_EXTIPSELL_EXTIPSEL2_DEFAULT                 0x00000000UL                              /**< Mode DEFAULT for GPIO_EXTIPSELL */
#define _GPIO_EXTIPSELL_EXTIPSEL2_PORTA                   0x00000000UL                              /**< Mode PORTA for GPIO_EXTIPSELL */
#define _GPIO_EXTIPSELL_EXTIPSEL2_PORTB                   0x00000001UL                              /**< Mode PORTB for GPIO_EXTIPSELL */
#define _GPIO_EXTIPSELL_EXTIPSEL2_PORTC                   0x00000002UL                              /**< Mode PORTC for GPIO_EXTIPSELL */
#define _GPIO_EXTIPSELL_EXTIPSEL2_PORTD                   0x00000003UL                              /**< Mode PORTD for GPIO_EXTIPSELL */
#define _GPIO_EXTIPSELL_EXTIPSEL2_PORTE                   0x00000004UL                              /**< Mode PORTE for GPIO_EXTIPSELL */
#define _GPIO_EXTIPSELL_EXTIPSEL2_PORTF                   0x00000005UL                              /**< Mode PORTF for GPIO_EXTIPSELL */
#define GPIO_EXTIPSELL_EXTIPSEL2_DEFAULT                  (_GPIO_EXTIPSELL_EXTIPSEL2_DEFAULT << 8)  /**< Shifted mode DEFAULT for GPIO_EXTIPSELL */
#define GPIO_EXTIPSELL_EXTIPSEL2_PORTA                    (_GPIO_EXTIPSELL_EXTIPSEL2_PORTA << 8)    /**< Shifted mode PORTA for GPIO_EXTIPSELL */
#define GPIO_EXTIPSELL_EXTIPSEL2_PORTB                    (_GPIO_EXTIPSELL_EXTIPSEL2_PORTB << 8)    /**< Shifted mode PORTB for GPIO_EXTIPSELL */
#define GPIO_EXTIPSELL_EXTIPSEL2_PORTC                    (_GPIO_EXTIPSELL_EXTIPSEL2_PORTC << 8)    /**< Shifted mode PORTC for GPIO_EXTIPSELL */
#define GPIO_EXTIPSELL_EXTIPSEL2_PORTD                    (_GPIO_EXTIPSELL_EXTIPSEL2_PORTD << 8)    /**< Shifted mode PORTD for GPIO_EXTIPSELL */
#define GPIO_EXTIPSELL_EXTIPSEL2_PORTE                    (_GPIO_EXTIPSELL_EXTIPSEL2_PORTE << 8)    /**< Shifted mode PORTE for GPIO_EXTIPSELL */
#define GPIO_EXTIPSELL_EXTIPSEL2_PORTF                    (_GPIO_EXTIPSELL_EXTIPSEL2_PORTF << 8)    /**< Shifted mode PORTF for GPIO_EXTIPSELL */
#define _GPIO_EXTIPSELL_EXTIPSEL3_SHIFT                   12                                        /**< Shift value for GPIO_EXTIPSEL3 */
#define _GPIO_EXTIPSELL_EXTIPSEL3_MASK                    0x7000UL                                  /**< Bit mask for GPIO_EXTIPSEL3 */
#define _GPIO_EXTIPSELL_EXTIPSEL3_DEFAULT                 0x00000000UL                              /**< Mode DEFAULT for GPIO_EXTIPSELL */
#define _GPIO_EXTIPSELL_EXTIPSEL3_PORTA                   0x00000000UL                              /**< Mode PORTA for GPIO_EXTIPSELL */
#define _GPIO_EXTIPSELL_EXTIPSEL3_PORTB                   0x00000001UL                              /**< Mode PORTB for GPIO_EXTIPSELL */
#define _GPIO_EXTIPSELL_EXTIPSEL3_PORTC                   0x00000002UL                              /**< Mode PORTC for GPIO_EXTIPSELL */
#define _GPIO_EXTIPSELL_EXTIPSEL3_PORTD                   0x00000003UL                              /**< Mode PORTD for GPIO_EXTIPSELL */
#define _GPIO_EXTIPSELL_EXTIPSEL3_PORTE                   0x00000004UL                              /**< Mode PORTE for GPIO_EXTIPSELL */
#define _GPIO_EXTIPSELL_EXTIPSEL3_PORTF                   0x00000005UL                              /**< Mode PORTF for GPIO_EXTIPSELL */
#define GPIO_EXTIPSELL_EXTIPSEL3_DEFAULT                  (_GPIO_EXTIPSELL_EXTIPSEL3_DEFAULT << 12) /**< Shifted mode DEFAULT for GPIO_EXTIPSELL */
#define GPIO_EXTIPSELL_EXTIPSEL3_PORTA                    (_GPIO_EXTIPSELL_EXTIPSEL3_PORTA << 12)   /**< Shifted mode PORTA for GPIO_EXTIPSELL */
#define GPIO_EXTIPSELL_EXTIPSEL3_PORTB                    (_GPIO_EXTIPSELL_EXTIPSEL3_PORTB << 12)   /**< Shifted mode PORTB for GPIO_EXTIPSELL */
#define GPIO_EXTIPSELL_EXTIPSEL3_PORTC                    (_GPIO_EXTIPSELL_EXTIPSEL3_PORTC << 12)   /**< Shifted mode PORTC for GPIO_EXTIPSELL */
#define GPIO_EXTIPSELL_EXTIPSEL3_PORTD                    (_GPIO_EXTIPSELL_EXTIPSEL3_PORTD << 12)   /**< Shifted mode PORTD for GPIO_EXTIPSELL */
#define GPIO_EXTIPSELL_EXTIPSEL3_PORTE                    (_GPIO_EXTIPSELL_EXTIPSEL3_PORTE << 12)   /**< Shifted mode PORTE for GPIO_EXTIPSELL */
#define GPIO_EXTIPSELL_EXTIPSEL3_PORTF                    (_GPIO_EXTIPSELL_EXTIPSEL3_PORTF << 12)   /**< Shifted mode PORTF for GPIO_EXTIPSELL */
#define _GPIO_EXTIPSELL_EXTIPSEL4_SHIFT                   16                                        /**< Shift value for GPIO_EXTIPSEL4 */
#define _GPIO_EXTIPSELL_EXTIPSEL4_MASK                    0x70000UL                                 /**< Bit mask for GPIO_EXTIPSEL4 */
#define _GPIO_EXTIPSELL_EXTIPSEL4_DEFAULT                 0x00000000UL                              /**< Mode DEFAULT for GPIO_EXTIPSELL */
#define _GPIO_EXTIPSELL_EXTIPSEL4_PORTA                   0x00000000UL                              /**< Mode PORTA for GPIO_EXTIPSELL */
#define _GPIO_EXTIPSELL_EXTIPSEL4_PORTB                   0x00000001UL                              /**< Mode PORTB for GPIO_EXTIPSELL */
#define _GPIO_EXTIPSELL_EXTIPSEL4_PORTC                   0x00000002UL                              /**< Mode PORTC for GPIO_EXTIPSELL */
#define _GPIO_EXTIPSELL_EXTIPSEL4_PORTD                   0x00000003UL                              /**< Mode PORTD for GPIO_EXTIPSELL */
#define _GPIO_EXTIPSELL_EXTIPSEL4_PORTE                   0x00000004UL                              /**< Mode PORTE for GPIO_EXTIPSELL */
#define _GPIO_EXTIPSELL_EXTIPSEL4_PORTF                   0x00000005UL                              /**< Mode PORTF for GPIO_EXTIPSELL */
#define GPIO_EXTIPSELL_EXTIPSEL4_DEFAULT                  (_GPIO_EXTIPSELL_EXTIPSEL4_DEFAULT << 16) /**< Shifted mode DEFAULT for GPIO_EXTIPSELL */
#define GPIO_EXTIPSELL_EXTIPSEL4_PORTA                    (_GPIO_EXTIPSELL_EXTIPSEL4_PORTA << 16)   /**< Shifted mode PORTA for GPIO_EXTIPSELL */
#define GPIO_EXTIPSELL_EXTIPSEL4_PORTB                    (_GPIO_EXTIPSELL_EXTIPSEL4_PORTB << 16)   /**< Shifted mode PORTB for GPIO_EXTIPSELL */
#define GPIO_EXTIPSELL_EXTIPSEL4_PORTC                    (_GPIO_EXTIPSELL_EXTIPSEL4_PORTC << 16)   /**< Shifted mode PORTC for GPIO_EXTIPSELL */
#define GPIO_EXTIPSELL_EXTIPSEL4_PORTD                    (_GPIO_EXTIPSELL_EXTIPSEL4_PORTD << 16)   /**< Shifted mode PORTD for GPIO_EXTIPSELL */
#define GPIO_EXTIPSELL_EXTIPSEL4_PORTE                    (_GPIO_EXTIPSELL_EXTIPSEL4_PORTE << 16)   /**< Shifted mode PORTE for GPIO_EXTIPSELL */
#define GPIO_EXTIPSELL_EXTIPSEL4_PORTF                    (_GPIO_EXTIPSELL_EXTIPSEL4_PORTF << 16)   /**< Shifted mode PORTF for GPIO_EXTIPSELL */
#define _GPIO_EXTIPSELL_EXTIPSEL5_SHIFT                   20                                        /**< Shift value for GPIO_EXTIPSEL5 */
#define _GPIO_EXTIPSELL_EXTIPSEL5_MASK                    0x700000UL                                /**< Bit mask for GPIO_EXTIPSEL5 */
#define _GPIO_EXTIPSELL_EXTIPSEL5_DEFAULT                 0x00000000UL                              /**< Mode DEFAULT for GPIO_EXTIPSELL */
#define _GPIO_EXTIPSELL_EXTIPSEL5_PORTA                   0x00000000UL                              /**< Mode PORTA for GPIO_EXTIPSELL */
#define _GPIO_EXTIPSELL_EXTIPSEL5_PORTB                   0x00000001UL                              /**< Mode PORTB for GPIO_EXTIPSELL */
#define _GPIO_EXTIPSELL_EXTIPSEL5_PORTC                   0x00000002UL                              /**< Mode PORTC for GPIO_EXTIPSELL */
#define _GPIO_EXTIPSELL_EXTIPSEL5_PORTD                   0x00000003UL                              /**< Mode PORTD for GPIO_EXTIPSELL */
#define _GPIO_EXTIPSELL_EXTIPSEL5_PORTE                   0x00000004UL                              /**< Mode PORTE for GPIO_EXTIPSELL */
#define _GPIO_EXTIPSELL_EXTIPSEL5_PORTF                   0x00000005UL                              /**< Mode PORTF for GPIO_EXTIPSELL */
#define GPIO_EXTIPSELL_EXTIPSEL5_DEFAULT                  (_GPIO_EXTIPSELL_EXTIPSEL5_DEFAULT << 20) /**< Shifted mode DEFAULT for GPIO_EXTIPSELL */
#define GPIO_EXTIPSELL_EXTIPSEL5_PORTA                    (_GPIO_EXTIPSELL_EXTIPSEL5_PORTA << 20)   /**< Shifted mode PORTA for GPIO_EXTIPSELL */
#define GPIO_EXTIPSELL_EXTIPSEL5_PORTB                    (_GPIO_EXTIPSELL_EXTIPSEL5_PORTB << 20)   /**< Shifted mode PORTB for GPIO_EXTIPSELL */
#define GPIO_EXTIPSELL_EXTIPSEL5_PORTC                    (_GPIO_EXTIPSELL_EXTIPSEL5_PORTC << 20)   /**< Shifted mode PORTC for GPIO_EXTIPSELL */
#define GPIO_EXTIPSELL_EXTIPSEL5_PORTD                    (_GPIO_EXTIPSELL_EXTIPSEL5_PORTD << 20)   /**< Shifted mode PORTD for GPIO_EXTIPSELL */
#define GPIO_EXTIPSELL_EXTIPSEL5_PORTE                    (_GPIO_EXTIPSELL_EXTIPSEL5_PORTE << 20)   /**< Shifted mode PORTE for GPIO_EXTIPSELL */
#define GPIO_EXTIPSELL_EXTIPSEL5_PORTF                    (_GPIO_EXTIPSELL_EXTIPSEL5_PORTF << 20)   /**< Shifted mode PORTF for GPIO_EXTIPSELL */
#define _GPIO_EXTIPSELL_EXTIPSEL6_SHIFT                   24                                        /**< Shift value for GPIO_EXTIPSEL6 */
#define _GPIO_EXTIPSELL_EXTIPSEL6_MASK                    0x7000000UL                               /**< Bit mask for GPIO_EXTIPSEL6 */
#define _GPIO_EXTIPSELL_EXTIPSEL6_DEFAULT                 0x00000000UL                              /**< Mode DEFAULT for GPIO_EXTIPSELL */
#define _GPIO_EXTIPSELL_EXTIPSEL6_PORTA                   0x00000000UL                              /**< Mode PORTA for GPIO_EXTIPSELL */
#define _GPIO_EXTIPSELL_EXTIPSEL6_PORTB                   0x00000001UL                              /**< Mode PORTB for GPIO_EXTIPSELL */
#define _GPIO_EXTIPSELL_EXTIPSEL6_PORTC                   0x00000002UL                              /**< Mode PORTC for GPIO_EXTIPSELL */
#define _GPIO_EXTIPSELL_EXTIPSEL6_PORTD                   0x00000003UL                              /**< Mode PORTD for GPIO_EXTIPSELL */
#define _GPIO_EXTIPSELL_EXTIPSEL6_PORTE                   0x00000004UL                              /**< Mode PORTE for GPIO_EXTIPSELL */
#define _GPIO_EXTIPSELL_EXTIPSEL6_PORTF                   0x00000005UL                              /**< Mode PORTF for GPIO_EXTIPSELL */
#define GPIO_EXTIPSELL_EXTIPSEL6_DEFAULT                  (_GPIO_EXTIPSELL_EXTIPSEL6_DEFAULT << 24) /**< Shifted mode DEFAULT for GPIO_EXTIPSELL */
#define GPIO_EXTIPSELL_EXTIPSEL6_PORTA                    (_GPIO_EXTIPSELL_EXTIPSEL6_PORTA << 24)   /**< Shifted mode PORTA for GPIO_EXTIPSELL */
#define GPIO_EXTIPSELL_EXTIPSEL6_PORTB                    (_GPIO_EXTIPSELL_EXTIPSEL6_PORTB << 24)   /**< Shifted mode PORTB for GPIO_EXTIPSELL */
#define GPIO_EXTIPSELL_EXTIPSEL6_PORTC                    (_GPIO_EXTIPSELL_EXTIPSEL6_PORTC << 24)   /**< Shifted mode PORTC for GPIO_EXTIPSELL */
#define GPIO_EXTIPSELL_EXTIPSEL6_PORTD                    (_GPIO_EXTIPSELL_EXTIPSEL6_PORTD << 24)   /**< Shifted mode PORTD for GPIO_EXTIPSELL */
#define GPIO_EXTIPSELL_EXTIPSEL6_PORTE                    (_GPIO_EXTIPSELL_EXTIPSEL6_PORTE << 24)   /**< Shifted mode PORTE for GPIO_EXTIPSELL */
#define GPIO_EXTIPSELL_EXTIPSEL6_PORTF                    (_GPIO_EXTIPSELL_EXTIPSEL6_PORTF << 24)   /**< Shifted mode PORTF for GPIO_EXTIPSELL */
#define _GPIO_EXTIPSELL_EXTIPSEL7_SHIFT                   28                                        /**< Shift value for GPIO_EXTIPSEL7 */
#define _GPIO_EXTIPSELL_EXTIPSEL7_MASK                    0x70000000UL                              /**< Bit mask for GPIO_EXTIPSEL7 */
#define _GPIO_EXTIPSELL_EXTIPSEL7_DEFAULT                 0x00000000UL                              /**< Mode DEFAULT for GPIO_EXTIPSELL */
#define _GPIO_EXTIPSELL_EXTIPSEL7_PORTA                   0x00000000UL                              /**< Mode PORTA for GPIO_EXTIPSELL */
#define _GPIO_EXTIPSELL_EXTIPSEL7_PORTB                   0x00000001UL                              /**< Mode PORTB for GPIO_EXTIPSELL */
#define _GPIO_EXTIPSELL_EXTIPSEL7_PORTC                   0x00000002UL                              /**< Mode PORTC for GPIO_EXTIPSELL */
#define _GPIO_EXTIPSELL_EXTIPSEL7_PORTD                   0x00000003UL                              /**< Mode PORTD for GPIO_EXTIPSELL */
#define _GPIO_EXTIPSELL_EXTIPSEL7_PORTE                   0x00000004UL                              /**< Mode PORTE for GPIO_EXTIPSELL */
#define _GPIO_EXTIPSELL_EXTIPSEL7_PORTF                   0x00000005UL                              /**< Mode PORTF for GPIO_EXTIPSELL */
#define GPIO_EXTIPSELL_EXTIPSEL7_DEFAULT                  (_GPIO_EXTIPSELL_EXTIPSEL7_DEFAULT << 28) /**< Shifted mode DEFAULT for GPIO_EXTIPSELL */
#define GPIO_EXTIPSELL_EXTIPSEL7_PORTA                    (_GPIO_EXTIPSELL_EXTIPSEL7_PORTA << 28)   /**< Shifted mode PORTA for GPIO_EXTIPSELL */
#define GPIO_EXTIPSELL_EXTIPSEL7_PORTB                    (_GPIO_EXTIPSELL_EXTIPSEL7_PORTB << 28)   /**< Shifted mode PORTB for GPIO_EXTIPSELL */
#define GPIO_EXTIPSELL_EXTIPSEL7_PORTC                    (_GPIO_EXTIPSELL_EXTIPSEL7_PORTC << 28)   /**< Shifted mode PORTC for GPIO_EXTIPSELL */
#define GPIO_EXTIPSELL_EXTIPSEL7_PORTD                    (_GPIO_EXTIPSELL_EXTIPSEL7_PORTD << 28)   /**< Shifted mode PORTD for GPIO_EXTIPSELL */
#define GPIO_EXTIPSELL_EXTIPSEL7_PORTE                    (_GPIO_EXTIPSELL_EXTIPSEL7_PORTE << 28)   /**< Shifted mode PORTE for GPIO_EXTIPSELL */
#define GPIO_EXTIPSELL_EXTIPSEL7_PORTF                    (_GPIO_EXTIPSELL_EXTIPSEL7_PORTF << 28)   /**< Shifted mode PORTF for GPIO_EXTIPSELL */

/* Bit fields for GPIO EXTIPSELH */
#define _GPIO_EXTIPSELH_RESETVALUE                        0x00000000UL                               /**< Default value for GPIO_EXTIPSELH */
#define _GPIO_EXTIPSELH_MASK                              0x77777777UL                               /**< Mask for GPIO_EXTIPSELH */
#define _GPIO_EXTIPSELH_EXTIPSEL8_SHIFT                   0                                          /**< Shift value for GPIO_EXTIPSEL8 */
#define _GPIO_EXTIPSELH_EXTIPSEL8_MASK                    0x7UL                                      /**< Bit mask for GPIO_EXTIPSEL8 */
#define _GPIO_EXTIPSELH_EXTIPSEL8_DEFAULT                 0x00000000UL                               /**< Mode DEFAULT for GPIO_EXTIPSELH */
#define _GPIO_EXTIPSELH_EXTIPSEL8_PORTA                   0x00000000UL                               /**< Mode PORTA for GPIO_EXTIPSELH */
#define _GPIO_EXTIPSELH_EXTIPSEL8_PORTB                   0x00000001UL                               /**< Mode PORTB for GPIO_EXTIPSELH */
#define _GPIO_EXTIPSELH_EXTIPSEL8_PORTC                   0x00000002UL                               /**< Mode PORTC for GPIO_EXTIPSELH */
#define _GPIO_EXTIPSELH_EXTIPSEL8_PORTD                   0x00000003UL                               /**< Mode PORTD for GPIO_EXTIPSELH */
#define _GPIO_EXTIPSELH_EXTIPSEL8_PORTE                   0x00000004UL                               /**< Mode PORTE for GPIO_EXTIPSELH */
#define _GPIO_EXTIPSELH_EXTIPSEL8_PORTF                   0x00000005UL                               /**< Mode PORTF for GPIO_EXTIPSELH */
#define GPIO_EXTIPSELH_EXTIPSEL8_DEFAULT                  (_GPIO_EXTIPSELH_EXTIPSEL8_DEFAULT << 0)   /**< Shifted mode DEFAULT for GPIO_EXTIPSELH */
#define GPIO_EXTIPSELH_EXTIPSEL8_PORTA                    (_GPIO_EXTIPSELH_EXTIPSEL8_PORTA << 0)     /**< Shifted mode PORTA for GPIO_EXTIPSELH */
#define GPIO_EXTIPSELH_EXTIPSEL8_PORTB                    (_GPIO_EXTIPSELH_EXTIPSEL8_PORTB << 0)     /**< Shifted mode PORTB for GPIO_EXTIPSELH */
#define GPIO_EXTIPSELH_EXTIPSEL8_PORTC                    (_GPIO_EXTIPSELH_EXTIPSEL8_PORTC << 0)     /**< Shifted mode PORTC for GPIO_EXTIPSELH */
#define GPIO_EXTIPSELH_EXTIPSEL8_PORTD                    (_GPIO_EXTIPSELH_EXTIPSEL8_PORTD << 0)     /**< Shifted mode PORTD for GPIO_EXTIPSELH */
#define GPIO_EXTIPSELH_EXTIPSEL8_PORTE                    (_GPIO_EXTIPSELH_EXTIPSEL8_PORTE << 0)     /**< Shifted mode PORTE for GPIO_EXTIPSELH */
#define GPIO_EXTIPSELH_EXTIPSEL8_PORTF                    (_GPIO_EXTIPSELH_EXTIPSEL8_PORTF << 0)     /**< Shifted mode PORTF for GPIO_EXTIPSELH */
#define _GPIO_EXTIPSELH_EXTIPSEL9_SHIFT                   4                                          /**< Shift value for GPIO_EXTIPSEL9 */
#define _GPIO_EXTIPSELH_EXTIPSEL9_MASK                    0x70UL                                     /**< Bit mask for GPIO_EXTIPSEL9 */
#define _GPIO_EXTIPSELH_EXTIPSEL9_DEFAULT                 0x00000000UL                               /**< Mode DEFAULT for GPIO_EXTIPSELH */
#define _GPIO_EXTIPSELH_EXTIPSEL9_PORTA                   0x00000000UL                               /**< Mode PORTA for GPIO_EXTIPSELH */
#define _GPIO_EXTIPSELH_EXTIPSEL9_PORTB                   0x00000001UL                               /**< Mode PORTB for GPIO_EXTIPSELH */
#define _GPIO_EXTIPSELH_EXTIPSEL9_PORTC                   0x00000002UL                               /**< Mode PORTC for GPIO_EXTIPSELH */
#define _GPIO_EXTIPSELH_EXTIPSEL9_PORTD                   0x00000003UL                               /**< Mode PORTD for GPIO_EXTIPSELH */
#define _GPIO_EXTIPSELH_EXTIPSEL9_PORTE                   0x00000004UL                               /**< Mode PORTE for GPIO_EXTIPSELH */
#define _GPIO_EXTIPSELH_EXTIPSEL9_PORTF                   0x00000005UL                               /**< Mode PORTF for GPIO_EXTIPSELH */
#define GPIO_EXTIPSELH_EXTIPSEL9_DEFAULT                  (_GPIO_EXTIPSELH_EXTIPSEL9_DEFAULT << 4)   /**< Shifted mode DEFAULT for GPIO_EXTIPSELH */
#define GPIO_EXTIPSELH_EXTIPSEL9_PORTA                    (_GPIO_EXTIPSELH_EXTIPSEL9_PORTA << 4)     /**< Shifted mode PORTA for GPIO_EXTIPSELH */
#define GPIO_EXTIPSELH_EXTIPSEL9_PORTB                    (_GPIO_EXTIPSELH_EXTIPSEL9_PORTB << 4)     /**< Shifted mode PORTB for GPIO_EXTIPSELH */
#define GPIO_EXTIPSELH_EXTIPSEL9_PORTC                    (_GPIO_EXTIPSELH_EXTIPSEL9_PORTC << 4)     /**< Shifted mode PORTC for GPIO_EXTIPSELH */
#define GPIO_EXTIPSELH_EXTIPSEL9_PORTD                    (_GPIO_EXTIPSELH_EXTIPSEL9_PORTD << 4)     /**< Shifted mode PORTD for GPIO_EXTIPSELH */
#define GPIO_EXTIPSELH_EXTIPSEL9_PORTE                    (_GPIO_EXTIPSELH_EXTIPSEL9_PORTE << 4)     /**< Shifted mode PORTE for GPIO_EXTIPSELH */
#define GPIO_EXTIPSELH_EXTIPSEL9_PORTF                    (_GPIO_EXTIPSELH_EXTIPSEL9_PORTF << 4)     /**< Shifted mode PORTF for GPIO_EXTIPSELH */
#define _GPIO_EXTIPSELH_EXTIPSEL10_SHIFT                  8                                          /**< Shift value for GPIO_EXTIPSEL10 */
#define _GPIO_EXTIPSELH_EXTIPSEL10_MASK                   0x700UL                                    /**< Bit mask for GPIO_EXTIPSEL10 */
#define _GPIO_EXTIPSELH_EXTIPSEL10_DEFAULT                0x00000000UL                               /**< Mode DEFAULT for GPIO_EXTIPSELH */
#define _GPIO_EXTIPSELH_EXTIPSEL10_PORTA                  0x00000000UL                               /**< Mode PORTA for GPIO_EXTIPSELH */
#define _GPIO_EXTIPSELH_EXTIPSEL10_PORTB                  0x00000001UL                               /**< Mode PORTB for GPIO_EXTIPSELH */
#define _GPIO_EXTIPSELH_EXTIPSEL10_PORTC                  0x00000002UL                               /**< Mode PORTC for GPIO_EXTIPSELH */
#define _GPIO_EXTIPSELH_EXTIPSEL10_PORTD                  0x00000003UL                               /**< Mode PORTD for GPIO_EXTIPSELH */
#define _GPIO_EXTIPSELH_EXTIPSEL10_PORTE                  0x00000004UL                               /**< Mode PORTE for GPIO_EXTIPSELH */
#define _GPIO_EXTIPSELH_EXTIPSEL10_PORTF                  0x00000005UL                               /**< Mode PORTF for GPIO_EXTIPSELH */
#define GPIO_EXTIPSELH_EXTIPSEL10_DEFAULT                 (_GPIO_EXTIPSELH_EXTIPSEL10_DEFAULT << 8)  /**< Shifted mode DEFAULT for GPIO_EXTIPSELH */
#define GPIO_EXTIPSELH_EXTIPSEL10_PORTA                   (_GPIO_EXTIPSELH_EXTIPSEL10_PORTA << 8)    /**< Shifted mode PORTA for GPIO_EXTIPSELH */
#define GPIO_EXTIPSELH_EXTIPSEL10_PORTB                   (_GPIO_EXTIPSELH_EXTIPSEL10_PORTB << 8)    /**< Shifted mode PORTB for GPIO_EXTIPSELH */
#define GPIO_EXTIPSELH_EXTIPSEL10_PORTC                   (_GPIO_EXTIPSELH_EXTIPSEL10_PORTC << 8)    /**< Shifted mode PORTC for GPIO_EXTIPSELH */
#define GPIO_EXTIPSELH_EXTIPSEL10_PORTD                   (_GPIO_EXTIPSELH_EXTIPSEL10_PORTD << 8)    /**< Shifted mode PORTD for GPIO_EXTIPSELH */
#define GPIO_EXTIPSELH_EXTIPSEL10_PORTE                   (_GPIO_EXTIPSELH_EXTIPSEL10_PORTE << 8)    /**< Shifted mode PORTE for GPIO_EXTIPSELH */
#define GPIO_EXTIPSELH_EXTIPSEL10_PORTF                   (_GPIO_EXTIPSELH_EXTIPSEL10_PORTF << 8)    /**< Shifted mode PORTF for GPIO_EXTIPSELH */
#define _GPIO_EXTIPSELH_EXTIPSEL11_SHIFT                  12                                         /**< Shift value for GPIO_EXTIPSEL11 */
#define _GPIO_EXTIPSELH_EXTIPSEL11_MASK                   0x7000UL                                   /**< Bit mask for GPIO_EXTIPSEL11 */
#define _GPIO_EXTIPSELH_EXTIPSEL11_DEFAULT                0x00000000UL                               /**< Mode DEFAULT for GPIO_EXTIPSELH */
#define _GPIO_EXTIPSELH_EXTIPSEL11_PORTA                  0x00000000UL                               /**< Mode PORTA for GPIO_EXTIPSELH */
#define _GPIO_EXTIPSELH_EXTIPSEL11_PORTB                  0x00000001UL                               /**< Mode PORTB for GPIO_EXTIPSELH */
#define _GPIO_EXTIPSELH_EXTIPSEL11_PORTC                  0x00000002UL                               /**< Mode PORTC for GPIO_EXTIPSELH */
#define _GPIO_EXTIPSELH_EXTIPSEL11_PORTD                  0x00000003UL                               /**< Mode PORTD for GPIO_EXTIPSELH */
#define _GPIO_EXTIPSELH_EXTIPSEL11_PORTE                  0x00000004UL                               /**< Mode PORTE for GPIO_EXTIPSELH */
#define _GPIO_EXTIPSELH_EXTIPSEL11_PORTF                  0x00000005UL                               /**< Mode PORTF for GPIO_EXTIPSELH */
#define GPIO_EXTIPSELH_EXTIPSEL11_DEFAULT                 (_GPIO_EXTIPSELH_EXTIPSEL11_DEFAULT << 12) /**< Shifted mode DEFAULT for GPIO_EXTIPSELH */
#define GPIO_EXTIPSELH_EXTIPSEL11_PORTA                   (_GPIO_EXTIPSELH_EXTIPSEL11_PORTA << 12)   /**< Shifted mode PORTA for GPIO_EXTIPSELH */
#define GPIO_EXTIPSELH_EXTIPSEL11_PORTB                   (_GPIO_EXTIPSELH_EXTIPSEL11_PORTB << 12)   /**< Shifted mode PORTB for GPIO_EXTIPSELH */
#define GPIO_EXTIPSELH_EXTIPSEL11_PORTC                   (_GPIO_EXTIPSELH_EXTIPSEL11_PORTC << 12)   /**< Shifted mode PORTC for GPIO_EXTIPSELH */
#define GPIO_EXTIPSELH_EXTIPSEL11_PORTD                   (_GPIO_EXTIPSELH_EXTIPSEL11_PORTD << 12)   /**< Shifted mode PORTD for GPIO_EXTIPSELH */
#define GPIO_EXTIPSELH_EXTIPSEL11_PORTE                   (_GPIO_EXTIPSELH_EXTIPSEL11_PORTE << 12)   /**< Shifted mode PORTE for GPIO_EXTIPSELH */
#define GPIO_EXTIPSELH_EXTIPSEL11_PORTF                   (_GPIO_EXTIPSELH_EXTIPSEL11_PORTF << 12)   /**< Shifted mode PORTF for GPIO_EXTIPSELH */
#define _GPIO_EXTIPSELH_EXTIPSEL12_SHIFT                  16                                         /**< Shift value for GPIO_EXTIPSEL12 */
#define _GPIO_EXTIPSELH_EXTIPSEL12_MASK                   0x70000UL                                  /**< Bit mask for GPIO_EXTIPSEL12 */
#define _GPIO_EXTIPSELH_EXTIPSEL12_DEFAULT                0x00000000UL                               /**< Mode DEFAULT for GPIO_EXTIPSELH */
#define _GPIO_EXTIPSELH_EXTIPSEL12_PORTA                  0x00000000UL                               /**< Mode PORTA for GPIO_EXTIPSELH */
#define _GPIO_EXTIPSELH_EXTIPSEL12_PORTB                  0x00000001UL                               /**< Mode PORTB for GPIO_EXTIPSELH */
#define _GPIO_EXTIPSELH_EXTIPSEL12_PORTC                  0x00000002UL                               /**< Mode PORTC for GPIO_EXTIPSELH */
#define _GPIO_EXTIPSELH_EXTIPSEL12_PORTD                  0x00000003UL                               /**< Mode PORTD for GPIO_EXTIPSELH */
#define _GPIO_EXTIPSELH_EXTIPSEL12_PORTE                  0x00000004UL                               /**< Mode PORTE for GPIO_EXTIPSELH */
#define _GPIO_EXTIPSELH_EXTIPSEL12_PORTF                  0x00000005UL                               /**< Mode PORTF for GPIO_EXTIPSELH */
#define GPIO_EXTIPSELH_EXTIPSEL12_DEFAULT                 (_GPIO_EXTIPSELH_EXTIPSEL12_DEFAULT << 16) /**< Shifted mode DEFAULT for GPIO_EXTIPSELH */
#define GPIO_EXTIPSELH_EXTIPSEL12_PORTA                   (_GPIO_EXTIPSELH_EXTIPSEL12_PORTA << 16)   /**< Shifted mode PORTA for GPIO_EXTIPSELH */
#define GPIO_EXTIPSELH_EXTIPSEL12_PORTB                   (_GPIO_EXTIPSELH_EXTIPSEL12_PORTB << 16)   /**< Shifted mode PORTB for GPIO_EXTIPSELH */
#define GPIO_EXTIPSELH_EXTIPSEL12_PORTC                   (_GPIO_EXTIPSELH_EXTIPSEL12_PORTC << 16)   /**< Shifted mode PORTC for GPIO_EXTIPSELH */
#define GPIO_EXTIPSELH_EXTIPSEL12_PORTD                   (_GPIO_EXTIPSELH_EXTIPSEL12_PORTD << 16)   /**< Shifted mode PORTD for GPIO_EXTIPSELH */
#define GPIO_EXTIPSELH_EXTIPSEL12_PORTE                   (_GPIO_EXTIPSELH_EXTIPSEL12_PORTE << 16)   /**< Shifted mode PORTE for GPIO_EXTIPSELH */
#define GPIO_EXTIPSELH_EXTIPSEL12_PORTF                   (_GPIO_EXTIPSELH_EXTIPSEL12_PORTF << 16)   /**< Shifted mode PORTF for GPIO_EXTIPSELH */
#define _GPIO_EXTIPSELH_EXTIPSEL13_SHIFT                  20                                         /**< Shift value for GPIO_EXTIPSEL13 */
#define _GPIO_EXTIPSELH_EXTIPSEL13_MASK                   0x700000UL                                 /**< Bit mask for GPIO_EXTIPSEL13 */
#define _GPIO_EXTIPSELH_EXTIPSEL13_DEFAULT                0x00000000UL                               /**< Mode DEFAULT for GPIO_EXTIPSELH */
#define _GPIO_EXTIPSELH_EXTIPSEL13_PORTA                  0x00000000UL                               /**< Mode PORTA for GPIO_EXTIPSELH */
#define _GPIO_EXTIPSELH_EXTIPSEL13_PORTB                  0x00000001UL                               /**< Mode PORTB for GPIO_EXTIPSELH */
#define _GPIO_EXTIPSELH_EXTIPSEL13_PORTC                  0x00000002UL                               /**< Mode PORTC for GPIO_EXTIPSELH */
#define _GPIO_EXTIPSELH_EXTIPSEL13_PORTD                  0x00000003UL                               /**< Mode PORTD for GPIO_EXTIPSELH */
#define _GPIO_EXTIPSELH_EXTIPSEL13_PORTE                  0x00000004UL                               /**< Mode PORTE for GPIO_EXTIPSELH */
#define _GPIO_EXTIPSELH_EXTIPSEL13_PORTF                  0x00000005UL                               /**< Mode PORTF for GPIO_EXTIPSELH */
#define GPIO_EXTIPSELH_EXTIPSEL13_DEFAULT                 (_GPIO_EXTIPSELH_EXTIPSEL13_DEFAULT << 20) /**< Shifted mode DEFAULT for GPIO_EXTIPSELH */
#define GPIO_EXTIPSELH_EXTIPSEL13_PORTA                   (_GPIO_EXTIPSELH_EXTIPSEL13_PORTA << 20)   /**< Shifted mode PORTA for GPIO_EXTIPSELH */
#define GPIO_EXTIPSELH_EXTIPSEL13_PORTB                   (_GPIO_EXTIPSELH_EXTIPSEL13_PORTB << 20)   /**< Shifted mode PORTB for GPIO_EXTIPSELH */
#define GPIO_EXTIPSELH_EXTIPSEL13_PORTC                   (_GPIO_EXTIPSELH_EXTIPSEL13_PORTC << 20)   /**< Shifted mode PORTC for GPIO_EXTIPSELH */
#define GPIO_EXTIPSELH_EXTIPSEL13_PORTD                   (_GPIO_EXTIPSELH_EXTIPSEL13_PORTD << 20)   /**< Shifted mode PORTD for GPIO_EXTIPSELH */
#define GPIO_EXTIPSELH_EXTIPSEL13_PORTE                   (_GPIO_EXTIPSELH_EXTIPSEL13_PORTE << 20)   /**< Shifted mode PORTE for GPIO_EXTIPSELH */
#define GPIO_EXTIPSELH_EXTIPSEL13_PORTF                   (_GPIO_EXTIPSELH_EXTIPSEL13_PORTF << 20)   /**< Shifted mode PORTF for GPIO_EXTIPSELH */
#define _GPIO_EXTIPSELH_EXTIPSEL14_SHIFT                  24                                         /**< Shift value for GPIO_EXTIPSEL14 */
#define _GPIO_EXTIPSELH_EXTIPSEL14_MASK                   0x7000000UL                                /**< Bit mask for GPIO_EXTIPSEL14 */
#define _GPIO_EXTIPSELH_EXTIPSEL14_DEFAULT                0x00000000UL                               /**< Mode DEFAULT for GPIO_EXTIPSELH */
#define _GPIO_EXTIPSELH_EXTIPSEL14_PORTA                  0x00000000UL                               /**< Mode PORTA for GPIO_EXTIPSELH */
#define _GPIO_EXTIPSELH_EXTIPSEL14_PORTB                  0x00000001UL                               /**< Mode PORTB for GPIO_EXTIPSELH */
#define _GPIO_EXTIPSELH_EXTIPSEL14_PORTC                  0x00000002UL                               /**< Mode PORTC for GPIO_EXTIPSELH */
#define _GPIO_EXTIPSELH_EXTIPSEL14_PORTD                  0x00000003UL                               /**< Mode PORTD for GPIO_EXTIPSELH */
#define _GPIO_EXTIPSELH_EXTIPSEL14_PORTE                  0x00000004UL                               /**< Mode PORTE for GPIO_EXTIPSELH */
#define _GPIO_EXTIPSELH_EXTIPSEL14_PORTF                  0x00000005UL                               /**< Mode PORTF for GPIO_EXTIPSELH */
#define GPIO_EXTIPSELH_EXTIPSEL14_DEFAULT                 (_GPIO_EXTIPSELH_EXTIPSEL14_DEFAULT << 24) /**< Shifted mode DEFAULT for GPIO_EXTIPSELH */
#define GPIO_EXTIPSELH_EXTIPSEL14_PORTA                   (_GPIO_EXTIPSELH_EXTIPSEL14_PORTA << 24)   /**< Shifted mode PORTA for GPIO_EXTIPSELH */
#define GPIO_EXTIPSELH_EXTIPSEL14_PORTB                   (_GPIO_EXTIPSELH_EXTIPSEL14_PORTB << 24)   /**< Shifted mode PORTB for GPIO_EXTIPSELH */
#define GPIO_EXTIPSELH_EXTIPSEL14_PORTC                   (_GPIO_EXTIPSELH_EXTIPSEL14_PORTC << 24)   /**< Shifted mode PORTC for GPIO_EXTIPSELH */
#define GPIO_EXTIPSELH_EXTIPSEL14_PORTD                   (_GPIO_EXTIPSELH_EXTIPSEL14_PORTD << 24)   /**< Shifted mode PORTD for GPIO_EXTIPSELH */
#define GPIO_EXTIPSELH_EXTIPSEL14_PORTE                   (_GPIO_EXTIPSELH_EXTIPSEL14_PORTE << 24)   /**< Shifted mode PORTE for GPIO_EXTIPSELH */
#define GPIO_EXTIPSELH_EXTIPSEL14_PORTF                   (_GPIO_EXTIPSELH_EXTIPSEL14_PORTF << 24)   /**< Shifted mode PORTF for GPIO_EXTIPSELH */
#define _GPIO_EXTIPSELH_EXTIPSEL15_SHIFT                  28                                         /**< Shift value for GPIO_EXTIPSEL15 */
#define _GPIO_EXTIPSELH_EXTIPSEL15_MASK                   0x70000000UL                               /**< Bit mask for GPIO_EXTIPSEL15 */
#define _GPIO_EXTIPSELH_EXTIPSEL15_DEFAULT                0x00000000UL                               /**< Mode DEFAULT for GPIO_EXTIPSELH */
#define _GPIO_EXTIPSELH_EXTIPSEL15_PORTA                  0x00000000UL                               /**< Mode PORTA for GPIO_EXTIPSELH */
#define _GPIO_EXTIPSELH_EXTIPSEL15_PORTB                  0x00000001UL                               /**< Mode PORTB for GPIO_EXTIPSELH */
#define _GPIO_EXTIPSELH_EXTIPSEL15_PORTC                  0x00000002UL                               /**< Mode PORTC for GPIO_EXTIPSELH */
#define _GPIO_EXTIPSELH_EXTIPSEL15_PORTD                  0x00000003UL                               /**< Mode PORTD for GPIO_EXTIPSELH */
#define _GPIO_EXTIPSELH_EXTIPSEL15_PORTE                  0x00000004UL                               /**< Mode PORTE for GPIO_EXTIPSELH */
#define _GPIO_EXTIPSELH_EXTIPSEL15_PORTF                  0x00000005UL                               /**< Mode PORTF for GPIO_EXTIPSELH */
#define GPIO_EXTIPSELH_EXTIPSEL15_DEFAULT                 (_GPIO_EXTIPSELH_EXTIPSEL15_DEFAULT << 28) /**< Shifted mode DEFAULT for GPIO_EXTIPSELH */
#define GPIO_EXTIPSELH_EXTIPSEL15_PORTA                   (_GPIO_EXTIPSELH_EXTIPSEL15_PORTA << 28)   /**< Shifted mode PORTA for GPIO_EXTIPSELH */
#define GPIO_EXTIPSELH_EXTIPSEL15_PORTB                   (_GPIO_EXTIPSELH_EXTIPSEL15_PORTB << 28)   /**< Shifted mode PORTB for GPIO_EXTIPSELH */
#define GPIO_EXTIPSELH_EXTIPSEL15_PORTC                   (_GPIO_EXTIPSELH_EXTIPSEL15_PORTC << 28)   /**< Shifted mode PORTC for GPIO_EXTIPSELH */
#define GPIO_EXTIPSELH_EXTIPSEL15_PORTD                   (_GPIO_EXTIPSELH_EXTIPSEL15_PORTD << 28)   /**< Shifted mode PORTD for GPIO_EXTIPSELH */
#define GPIO_EXTIPSELH_EXTIPSEL15_PORTE                   (_GPIO_EXTIPSELH_EXTIPSEL15_PORTE << 28)   /**< Shifted mode PORTE for GPIO_EXTIPSELH */
#define GPIO_EXTIPSELH_EXTIPSEL15_PORTF                   (_GPIO_EXTIPSELH_EXTIPSEL15_PORTF << 28)   /**< Shifted mode PORTF for GPIO_EXTIPSELH */

/* Bit fields for GPIO EXTIRISE */
#define _GPIO_EXTIRISE_RESETVALUE                         0x00000000UL                           /**< Default value for GPIO_EXTIRISE */
#define _GPIO_EXTIRISE_MASK                               0x0000FFFFUL                           /**< Mask for GPIO_EXTIRISE */
#define _GPIO_EXTIRISE_EXTIRISE_SHIFT                     0                                      /**< Shift value for GPIO_EXTIRISE */
#define _GPIO_EXTIRISE_EXTIRISE_MASK                      0xFFFFUL                               /**< Bit mask for GPIO_EXTIRISE */
#define _GPIO_EXTIRISE_EXTIRISE_DEFAULT                   0x00000000UL                           /**< Mode DEFAULT for GPIO_EXTIRISE */
#define GPIO_EXTIRISE_EXTIRISE_DEFAULT                    (_GPIO_EXTIRISE_EXTIRISE_DEFAULT << 0) /**< Shifted mode DEFAULT for GPIO_EXTIRISE */

/* Bit fields for GPIO EXTIFALL */
#define _GPIO_EXTIFALL_RESETVALUE                         0x00000000UL                           /**< Default value for GPIO_EXTIFALL */
#define _GPIO_EXTIFALL_MASK                               0x0000FFFFUL                           /**< Mask for GPIO_EXTIFALL */
#define _GPIO_EXTIFALL_EXTIFALL_SHIFT                     0                                      /**< Shift value for GPIO_EXTIFALL */
#define _GPIO_EXTIFALL_EXTIFALL_MASK                      0xFFFFUL                               /**< Bit mask for GPIO_EXTIFALL */
#define _GPIO_EXTIFALL_EXTIFALL_DEFAULT                   0x00000000UL                           /**< Mode DEFAULT for GPIO_EXTIFALL */
#define GPIO_EXTIFALL_EXTIFALL_DEFAULT                    (_GPIO_EXTIFALL_EXTIFALL_DEFAULT << 0) /**< Shifted mode DEFAULT for GPIO_EXTIFALL */

/* Bit fields for GPIO IEN */
#define _GPIO_IEN_RESETVALUE                              0x00000000UL                 /**< Default value for GPIO_IEN */
#define _GPIO_IEN_MASK                                    0x0000FFFFUL                 /**< Mask for GPIO_IEN */
#define _GPIO_IEN_EXT_SHIFT                               0                            /**< Shift value for GPIO_EXT */
#define _GPIO_IEN_EXT_MASK                                0xFFFFUL                     /**< Bit mask for GPIO_EXT */
#define _GPIO_IEN_EXT_DEFAULT                             0x00000000UL                 /**< Mode DEFAULT for GPIO_IEN */
#define GPIO_IEN_EXT_DEFAULT                              (_GPIO_IEN_EXT_DEFAULT << 0) /**< Shifted mode DEFAULT for GPIO_IEN */

/* Bit fields for GPIO IF */
#define _GPIO_IF_RESETVALUE                               0x00000000UL                /**< Default value for GPIO_IF */
#define _GPIO_IF_MASK                                     0x0000FFFFUL                /**< Mask for GPIO_IF */
#define _GPIO_IF_EXT_SHIFT                                0                           /**< Shift value for GPIO_EXT */
#define _GPIO_IF_EXT_MASK                                 0xFFFFUL                    /**< Bit mask for GPIO_EXT */
#define _GPIO_IF_EXT_DEFAULT                              0x00000000UL                /**< Mode DEFAULT for GPIO_IF */
#define GPIO_IF_EXT_DEFAULT                               (_GPIO_IF_EXT_DEFAULT << 0) /**< Shifted mode DEFAULT for GPIO_IF */

/* Bit fields for GPIO IFS */
#define _GPIO_IFS_RESETVALUE                              0x00000000UL                 /**< Default value for GPIO_IFS */
#define _GPIO_IFS_MASK                                    0x0000FFFFUL                 /**< Mask for GPIO_IFS */
#define _GPIO_IFS_EXT_SHIFT                               0                            /**< Shift value for GPIO_EXT */
#define _GPIO_IFS_EXT_MASK                                0xFFFFUL                     /**< Bit mask for GPIO_EXT */
#define _GPIO_IFS_EXT_DEFAULT                             0x00000000UL                 /**< Mode DEFAULT for GPIO_IFS */
#define GPIO_IFS_EXT_DEFAULT                              (_GPIO_IFS_EXT_DEFAULT << 0) /**< Shifted mode DEFAULT for GPIO_IFS */

/* Bit fields for GPIO IFC */
#define _GPIO_IFC_RESETVALUE                              0x00000000UL                 /**< Default value for GPIO_IFC */
#define _GPIO_IFC_MASK                                    0x0000FFFFUL                 /**< Mask for GPIO_IFC */
#define _GPIO_IFC_EXT_SHIFT                               0                            /**< Shift value for GPIO_EXT */
#define _GPIO_IFC_EXT_MASK                                0xFFFFUL                     /**< Bit mask for GPIO_EXT */
#define _GPIO_IFC_EXT_DEFAULT                             0x00000000UL                 /**< Mode DEFAULT for GPIO_IFC */
#define GPIO_IFC_EXT_DEFAULT                              (_GPIO_IFC_EXT_DEFAULT << 0) /**< Shifted mode DEFAULT for GPIO_IFC */

/* Bit fields for GPIO ROUTE */
#define _GPIO_ROUTE_RESETVALUE                            0x00000003UL                        /**< Default value for GPIO_ROUTE */
#define _GPIO_ROUTE_MASK                                  0x00000003UL                        /**< Mask for GPIO_ROUTE */
#define GPIO_ROUTE_SWCLKPEN                               (0x1UL << 0)                        /**< Serial Wire Clock Pin Enable */
#define _GPIO_ROUTE_SWCLKPEN_SHIFT                        0                                   /**< Shift value for GPIO_SWCLKPEN */
#define _GPIO_ROUTE_SWCLKPEN_MASK                         0x1UL                               /**< Bit mask for GPIO_SWCLKPEN */
#define _GPIO_ROUTE_SWCLKPEN_DEFAULT                      0x00000001UL                        /**< Mode DEFAULT for GPIO_ROUTE */
#define GPIO_ROUTE_SWCLKPEN_DEFAULT                       (_GPIO_ROUTE_SWCLKPEN_DEFAULT << 0) /**< Shifted mode DEFAULT for GPIO_ROUTE */
#define GPIO_ROUTE_SWDIOPEN                               (0x1UL << 1)                        /**< Serial Wire Data Pin Enable */
#define _GPIO_ROUTE_SWDIOPEN_SHIFT                        1                                   /**< Shift value for GPIO_SWDIOPEN */
#define _GPIO_ROUTE_SWDIOPEN_MASK                         0x2UL                               /**< Bit mask for GPIO_SWDIOPEN */
#define _GPIO_ROUTE_SWDIOPEN_DEFAULT                      0x00000001UL                        /**< Mode DEFAULT for GPIO_ROUTE */
#define GPIO_ROUTE_SWDIOPEN_DEFAULT                       (_GPIO_ROUTE_SWDIOPEN_DEFAULT << 1) /**< Shifted mode DEFAULT for GPIO_ROUTE */

/* Bit fields for GPIO INSENSE */
#define _GPIO_INSENSE_RESETVALUE                          0x00000003UL                     /**< Default value for GPIO_INSENSE */
#define _GPIO_INSENSE_MASK                                0x00000003UL                     /**< Mask for GPIO_INSENSE */
#define GPIO_INSENSE_INT                                  (0x1UL << 0)                     /**< Interrupt Sense Enable */
#define _GPIO_INSENSE_INT_SHIFT                           0                                /**< Shift value for GPIO_INT */
#define _GPIO_INSENSE_INT_MASK                            0x1UL                            /**< Bit mask for GPIO_INT */
#define _GPIO_INSENSE_INT_DEFAULT                         0x00000001UL                     /**< Mode DEFAULT for GPIO_INSENSE */
#define GPIO_INSENSE_INT_DEFAULT                          (_GPIO_INSENSE_INT_DEFAULT << 0) /**< Shifted mode DEFAULT for GPIO_INSENSE */
#define GPIO_INSENSE_PRS                                  (0x1UL << 1)                     /**< PRS Sense Enable */
#define _GPIO_INSENSE_PRS_SHIFT                           1                                /**< Shift value for GPIO_PRS */
#define _GPIO_INSENSE_PRS_MASK                            0x2UL                            /**< Bit mask for GPIO_PRS */
#define _GPIO_INSENSE_PRS_DEFAULT                         0x00000001UL                     /**< Mode DEFAULT for GPIO_INSENSE */
#define GPIO_INSENSE_PRS_DEFAULT                          (_GPIO_INSENSE_PRS_DEFAULT << 1) /**< Shifted mode DEFAULT for GPIO_INSENSE */

/* Bit fields for GPIO LOCK */
#define _GPIO_LOCK_RESETVALUE                             0x00000000UL                       /**< Default value for GPIO_LOCK */
#define _GPIO_LOCK_MASK                                   0x0000FFFFUL                       /**< Mask for GPIO_LOCK */
#define _GPIO_LOCK_LOCKKEY_SHIFT                          0                                  /**< Shift value for GPIO_LOCKKEY */
#define _GPIO_LOCK_LOCKKEY_MASK                           0xFFFFUL                           /**< Bit mask for GPIO_LOCKKEY */
#define _GPIO_LOCK_LOCKKEY_DEFAULT                        0x00000000UL                       /**< Mode DEFAULT for GPIO_LOCK */
#define _GPIO_LOCK_LOCKKEY_LOCK                           0x00000000UL                       /**< Mode LOCK for GPIO_LOCK */
#define _GPIO_LOCK_LOCKKEY_UNLOCKED                       0x00000000UL                       /**< Mode UNLOCKED for GPIO_LOCK */
#define _GPIO_LOCK_LOCKKEY_LOCKED                         0x00000001UL                       /**< Mode LOCKED for GPIO_LOCK */
#define _GPIO_LOCK_LOCKKEY_UNLOCK                         0x0000A534UL                       /**< Mode UNLOCK for GPIO_LOCK */
#define GPIO_LOCK_LOCKKEY_DEFAULT                         (_GPIO_LOCK_LOCKKEY_DEFAULT << 0)  /**< Shifted mode DEFAULT for GPIO_LOCK */
#define GPIO_LOCK_LOCKKEY_LOCK                            (_GPIO_LOCK_LOCKKEY_LOCK << 0)     /**< Shifted mode LOCK for GPIO_LOCK */
#define GPIO_LOCK_LOCKKEY_UNLOCKED                        (_GPIO_LOCK_LOCKKEY_UNLOCKED << 0) /**< Shifted mode UNLOCKED for GPIO_LOCK */
#define GPIO_LOCK_LOCKKEY_LOCKED                          (_GPIO_LOCK_LOCKKEY_LOCKED << 0)   /**< Shifted mode LOCKED for GPIO_LOCK */
#define GPIO_LOCK_LOCKKEY_UNLOCK                          (_GPIO_LOCK_LOCKKEY_UNLOCK << 0)   /**< Shifted mode UNLOCK for GPIO_LOCK */

/* Bit fields for GPIO CTRL */
#define _GPIO_CTRL_RESETVALUE                             0x00000000UL                     /**< Default value for GPIO_CTRL */
#define _GPIO_CTRL_MASK                                   0x00000001UL                     /**< Mask for GPIO_CTRL */
#define GPIO_CTRL_EM4RET                                  (0x1UL << 0)                     /**< Enable EM4 retention */
#define _GPIO_CTRL_EM4RET_SHIFT                           0                                /**< Shift value for GPIO_EM4RET */
#define _GPIO_CTRL_EM4RET_MASK                            0x1UL                            /**< Bit mask for GPIO_EM4RET */
#define _GPIO_CTRL_EM4RET_DEFAULT                         0x00000000UL                     /**< Mode DEFAULT for GPIO_CTRL */
#define GPIO_CTRL_EM4RET_DEFAULT                          (_GPIO_CTRL_EM4RET_DEFAULT << 0) /**< Shifted mode DEFAULT for GPIO_CTRL */

/* Bit fields for GPIO CMD */
#define _GPIO_CMD_RESETVALUE                              0x00000000UL                      /**< Default value for GPIO_CMD */
#define _GPIO_CMD_MASK                                    0x00000001UL                      /**< Mask for GPIO_CMD */
#define GPIO_CMD_EM4WUCLR                                 (0x1UL << 0)                      /**< EM4 Wake-up clear */
#define _GPIO_CMD_EM4WUCLR_SHIFT                          0                                 /**< Shift value for GPIO_EM4WUCLR */
#define _GPIO_CMD_EM4WUCLR_MASK                           0x1UL                             /**< Bit mask for GPIO_EM4WUCLR */
#define _GPIO_CMD_EM4WUCLR_DEFAULT                        0x00000000UL                      /**< Mode DEFAULT for GPIO_CMD */
#define GPIO_CMD_EM4WUCLR_DEFAULT                         (_GPIO_CMD_EM4WUCLR_DEFAULT << 0) /**< Shifted mode DEFAULT for GPIO_CMD */

/* Bit fields for GPIO EM4WUEN */
#define _GPIO_EM4WUEN_RESETVALUE                          0x00000000UL                         /**< Default value for GPIO_EM4WUEN */
#define _GPIO_EM4WUEN_MASK                                0x0000007FUL                         /**< Mask for GPIO_EM4WUEN */
#define _GPIO_EM4WUEN_EM4WUEN_SHIFT                       0                                    /**< Shift value for GPIO_EM4WUEN */
#define _GPIO_EM4WUEN_EM4WUEN_MASK                        0x7FUL                               /**< Bit mask for GPIO_EM4WUEN */
#define _GPIO_EM4WUEN_EM4WUEN_DEFAULT                     0x00000000UL                         /**< Mode DEFAULT for GPIO_EM4WUEN */
#define _GPIO_EM4WUEN_EM4WUEN_A0                          0x00000001UL                         /**< Mode A0 for GPIO_EM4WUEN */
#define _GPIO_EM4WUEN_EM4WUEN_C9                          0x00000004UL                         /**< Mode C9 for GPIO_EM4WUEN */
#define _GPIO_EM4WUEN_EM4WUEN_F1                          0x00000008UL                         /**< Mode F1 for GPIO_EM4WUEN */
#define _GPIO_EM4WUEN_EM4WUEN_F2                          0x00000010UL                         /**< Mode F2 for GPIO_EM4WUEN */
#define _GPIO_EM4WUEN_EM4WUEN_E13                         0x00000020UL                         /**< Mode E13 for GPIO_EM4WUEN */
#define _GPIO_EM4WUEN_EM4WUEN_C4                          0x00000040UL                         /**< Mode C4 for GPIO_EM4WUEN */
#define GPIO_EM4WUEN_EM4WUEN_DEFAULT                      (_GPIO_EM4WUEN_EM4WUEN_DEFAULT << 0) /**< Shifted mode DEFAULT for GPIO_EM4WUEN */
#define GPIO_EM4WUEN_EM4WUEN_A0                           (_GPIO_EM4WUEN_EM4WUEN_A0 << 0)      /**< Shifted mode A0 for GPIO_EM4WUEN */
#define GPIO_EM4WUEN_EM4WUEN_C9                           (_GPIO_EM4WUEN_EM4WUEN_C9 << 0)      /**< Shifted mode C9 for GPIO_EM4WUEN */
#define GPIO_EM4WUEN_EM4WUEN_F1                           (_GPIO_EM4WUEN_EM4WUEN_F1 << 0)      /**< Shifted mode F1 for GPIO_EM4WUEN */
#define GPIO_EM4WUEN_EM4WUEN_F2                           (_GPIO_EM4WUEN_EM4WUEN_F2 << 0)      /**< Shifted mode F2 for GPIO_EM4WUEN */
#define GPIO_EM4WUEN_EM4WUEN_E13                          (_GPIO_EM4WUEN_EM4WUEN_E13 << 0)     /**< Shifted mode E13 for GPIO_EM4WUEN */
#define GPIO_EM4WUEN_EM4WUEN_C4                           (_GPIO_EM4WUEN_EM4WUEN_C4 << 0)      /**< Shifted mode C4 for GPIO_EM4WUEN */

/* Bit fields for GPIO EM4WUPOL */
#define _GPIO_EM4WUPOL_RESETVALUE                         0x00000000UL                           /**< Default value for GPIO_EM4WUPOL */
#define _GPIO_EM4WUPOL_MASK                               0x0000007FUL                           /**< Mask for GPIO_EM4WUPOL */
#define _GPIO_EM4WUPOL_EM4WUPOL_SHIFT                     0                                      /**< Shift value for GPIO_EM4WUPOL */
#define _GPIO_EM4WUPOL_EM4WUPOL_MASK                      0x7FUL                                 /**< Bit mask for GPIO_EM4WUPOL */
#define _GPIO_EM4WUPOL_EM4WUPOL_DEFAULT                   0x00000000UL                           /**< Mode DEFAULT for GPIO_EM4WUPOL */
#define _GPIO_EM4WUPOL_EM4WUPOL_A0                        0x00000001UL                           /**< Mode A0 for GPIO_EM4WUPOL */
#define _GPIO_EM4WUPOL_EM4WUPOL_C9                        0x00000004UL                           /**< Mode C9 for GPIO_EM4WUPOL */
#define _GPIO_EM4WUPOL_EM4WUPOL_F1                        0x00000008UL                           /**< Mode F1 for GPIO_EM4WUPOL */
#define _GPIO_EM4WUPOL_EM4WUPOL_F2                        0x00000010UL                           /**< Mode F2 for GPIO_EM4WUPOL */
#define _GPIO_EM4WUPOL_EM4WUPOL_E13                       0x00000020UL                           /**< Mode E13 for GPIO_EM4WUPOL */
#define _GPIO_EM4WUPOL_EM4WUPOL_C4                        0x00000040UL                           /**< Mode C4 for GPIO_EM4WUPOL */
#define GPIO_EM4WUPOL_EM4WUPOL_DEFAULT                    (_GPIO_EM4WUPOL_EM4WUPOL_DEFAULT << 0) /**< Shifted mode DEFAULT for GPIO_EM4WUPOL */
#define GPIO_EM4WUPOL_EM4WUPOL_A0                         (_GPIO_EM4WUPOL_EM4WUPOL_A0 << 0)      /**< Shifted mode A0 for GPIO_EM4WUPOL */
#define GPIO_EM4WUPOL_EM4WUPOL_C9                         (_GPIO_EM4WUPOL_EM4WUPOL_C9 << 0)      /**< Shifted mode C9 for GPIO_EM4WUPOL */
#define GPIO_EM4WUPOL_EM4WUPOL_F1                         (_GPIO_EM4WUPOL_EM4WUPOL_F1 << 0)      /**< Shifted mode F1 for GPIO_EM4WUPOL */
#define GPIO_EM4WUPOL_EM4WUPOL_F2                         (_GPIO_EM4WUPOL_EM4WUPOL_F2 << 0)      /**< Shifted mode F2 for GPIO_EM4WUPOL */
#define GPIO_EM4WUPOL_EM4WUPOL_E13                        (_GPIO_EM4WUPOL_EM4WUPOL_E13 << 0)     /**< Shifted mode E13 for GPIO_EM4WUPOL */
#define GPIO_EM4WUPOL_EM4WUPOL_C4                         (_GPIO_EM4WUPOL_EM4WUPOL_C4 << 0)      /**< Shifted mode C4 for GPIO_EM4WUPOL */

/* Bit fields for GPIO EM4WUCAUSE */
#define _GPIO_EM4WUCAUSE_RESETVALUE                       0x00000000UL                               /**< Default value for GPIO_EM4WUCAUSE */
#define _GPIO_EM4WUCAUSE_MASK                             0x0000007FUL                               /**< Mask for GPIO_EM4WUCAUSE */
#define _GPIO_EM4WUCAUSE_EM4WUCAUSE_SHIFT                 0                                          /**< Shift value for GPIO_EM4WUCAUSE */
#define _GPIO_EM4WUCAUSE_EM4WUCAUSE_MASK                  0x7FUL                                     /**< Bit mask for GPIO_EM4WUCAUSE */
#define _GPIO_EM4WUCAUSE_EM4WUCAUSE_DEFAULT               0x00000000UL                               /**< Mode DEFAULT for GPIO_EM4WUCAUSE */
#define _GPIO_EM4WUCAUSE_EM4WUCAUSE_A0                    0x00000001UL                               /**< Mode A0 for GPIO_EM4WUCAUSE */
#define _GPIO_EM4WUCAUSE_EM4WUCAUSE_C9                    0x00000004UL                               /**< Mode C9 for GPIO_EM4WUCAUSE */
#define _GPIO_EM4WUCAUSE_EM4WUCAUSE_F1                    0x00000008UL                               /**< Mode F1 for GPIO_EM4WUCAUSE */
#define _GPIO_EM4WUCAUSE_EM4WUCAUSE_F2                    0x00000010UL                               /**< Mode F2 for GPIO_EM4WUCAUSE */
#define _GPIO_EM4WUCAUSE_EM4WUCAUSE_E13                   0x00000020UL                               /**< Mode E13 for GPIO_EM4WUCAUSE */
#define _GPIO_EM4WUCAUSE_EM4WUCAUSE_C4                    0x00000040UL                               /**< Mode C4 for GPIO_EM4WUCAUSE */
#define GPIO_EM4WUCAUSE_EM4WUCAUSE_DEFAULT                (_GPIO_EM4WUCAUSE_EM4WUCAUSE_DEFAULT << 0) /**< Shifted mode DEFAULT for GPIO_EM4WUCAUSE */
#define GPIO_EM4WUCAUSE_EM4WUCAUSE_A0                     (_GPIO_EM4WUCAUSE_EM4WUCAUSE_A0 << 0)      /**< Shifted mode A0 for GPIO_EM4WUCAUSE */
#define GPIO_EM4WUCAUSE_EM4WUCAUSE_C9                     (_GPIO_EM4WUCAUSE_EM4WUCAUSE_C9 << 0)      /**< Shifted mode C9 for GPIO_EM4WUCAUSE */
#define GPIO_EM4WUCAUSE_EM4WUCAUSE_F1                     (_GPIO_EM4WUCAUSE_EM4WUCAUSE_F1 << 0)      /**< Shifted mode F1 for GPIO_EM4WUCAUSE */
#define GPIO_EM4WUCAUSE_EM4WUCAUSE_F2                     (_GPIO_EM4WUCAUSE_EM4WUCAUSE_F2 << 0)      /**< Shifted mode F2 for GPIO_EM4WUCAUSE */
#define GPIO_EM4WUCAUSE_EM4WUCAUSE_E13                    (_GPIO_EM4WUCAUSE_EM4WUCAUSE_E13 << 0)     /**< Shifted mode E13 for GPIO_EM4WUCAUSE */
#define GPIO_EM4WUCAUSE_EM4WUCAUSE_C4                     (_GPIO_EM4WUCAUSE_EM4WUCAUSE_C4 << 0)      /**< Shifted mode C4 for GPIO_EM4WUCAUSE */

/**************************************************************************//**
 * @defgroup EFM32HG_DEVINFO_BitFields
 * @{
 *****************************************************************************/
/* Bit fields for EFM32HG_DEVINFO */
#define _DEVINFO_CAL_CRC_MASK                           0x0000FFFFUL /**< Integrity CRC checksum mask */
#define _DEVINFO_CAL_CRC_SHIFT                          0            /**< Integrity CRC checksum shift */
#define _DEVINFO_CAL_TEMP_MASK                          0x00FF0000UL /**< Calibration temperature, DegC, mask */
#define _DEVINFO_CAL_TEMP_SHIFT                         16           /**< Calibration temperature shift */
#define _DEVINFO_ADC0CAL0_1V25_GAIN_MASK                0x00007F00UL /**< Gain for 1V25 reference, mask */
#define _DEVINFO_ADC0CAL0_1V25_GAIN_SHIFT               8            /**< Gain for 1V25 reference, shift */
#define _DEVINFO_ADC0CAL0_1V25_OFFSET_MASK              0x0000007FUL /**< Offset for 1V25 reference, mask */
#define _DEVINFO_ADC0CAL0_1V25_OFFSET_SHIFT             0            /**< Offset for 1V25 reference, shift */
#define _DEVINFO_ADC0CAL0_2V5_GAIN_MASK                 0x7F000000UL /**< Gain for 2V5 reference, mask */
#define _DEVINFO_ADC0CAL0_2V5_GAIN_SHIFT                24           /**< Gain for 2V5 reference, shift */
#define _DEVINFO_ADC0CAL0_2V5_OFFSET_MASK               0x007F0000UL /**< Offset for 2V5 reference, mask */
#define _DEVINFO_ADC0CAL0_2V5_OFFSET_SHIFT              16           /**< Offset for 2V5 reference, shift */
#define _DEVINFO_ADC0CAL1_VDD_GAIN_MASK                 0x00007F00UL /**< Gain for VDD reference, mask */
#define _DEVINFO_ADC0CAL1_VDD_GAIN_SHIFT                8            /**< Gain for VDD reference, shift */
#define _DEVINFO_ADC0CAL1_VDD_OFFSET_MASK               0x0000007FUL /**< Offset for VDD reference, mask */
#define _DEVINFO_ADC0CAL1_VDD_OFFSET_SHIFT              0            /**< Offset for VDD reference, shift */
#define _DEVINFO_ADC0CAL1_5VDIFF_GAIN_MASK              0x7F000000UL /**< Gain 5VDIFF for 5VDIFF reference, mask */
#define _DEVINFO_ADC0CAL1_5VDIFF_GAIN_SHIFT             24           /**< Gain for 5VDIFF reference, mask */
#define _DEVINFO_ADC0CAL1_5VDIFF_OFFSET_MASK            0x007F0000UL /**< Offset for 5VDIFF reference, mask */
#define _DEVINFO_ADC0CAL1_5VDIFF_OFFSET_SHIFT           16           /**< Offset for 5VDIFF reference, shift */
#define _DEVINFO_ADC0CAL2_2XVDDVSS_OFFSET_MASK          0x0000007FUL /**< Offset for 2XVDDVSS reference, mask */
#define _DEVINFO_ADC0CAL2_2XVDDVSS_OFFSET_SHIFT         0            /**< Offset for 2XVDDVSS reference, shift */
#define _DEVINFO_ADC0CAL2_TEMP1V25_MASK                 0xFFF00000UL /**< Temperature reading at 1V25 reference, mask */
#define _DEVINFO_ADC0CAL2_TEMP1V25_SHIFT                20           /**< Temperature reading at 1V25 reference, DegC */
#define _DEVINFO_IDAC0CAL0_RANGE0_MASK                  0x000000FFUL /**< Current range 0 tuning value for IDAC0 mask */
#define _DEVINFO_IDAC0CAL0_RANGE0_SHIFT                 0            /**< Current range 0 tuning value for IDAC0 shift */
#define _DEVINFO_IDAC0CAL0_RANGE1_MASK                  0x0000FF00UL /**< Current range 1 tuning value for IDAC0 mask */
#define _DEVINFO_IDAC0CAL0_RANGE1_SHIFT                 8            /**< Current range 1 tuning value for IDAC0 shift */
#define _DEVINFO_IDAC0CAL0_RANGE2_MASK                  0x00FF0000UL /**< Current range 2 tuning value for IDAC0 mask */
#define _DEVINFO_IDAC0CAL0_RANGE2_SHIFT                 16           /**< Current range 2 tuning value for IDAC0 shift */
#define _DEVINFO_IDAC0CAL0_RANGE3_MASK                  0xFF000000UL /**< Current range 3 tuning value for IDAC0 mask */
#define _DEVINFO_IDAC0CAL0_RANGE3_SHIFT                 24           /**< Current range 3 tuning value for IDAC0 shift */
#define _DEVINFO_USHFRCOCAL0_BAND24_TUNING_MASK         0x0000007FUL /**< 24 MHz TUNING value for USFRCO mask */
#define _DEVINFO_USHFRCOCAL0_BAND24_TUNING_SHIFT        0            /**< 24 MHz TUNING value for USFRCO shift */
#define _DEVINFO_USHFRCOCAL0_BAND24_FINETUNING_MASK     0x00003F00UL /**< 24 MHz FINETUNING value for USFRCO mask */
#define _DEVINFO_USHFRCOCAL0_BAND24_FINETUNING_SHIFT    8            /**< 24 MHz FINETUNING value for USFRCO shift */
#define _DEVINFO_USHFRCOCAL0_BAND48_TUNING_MASK         0x007F0000UL /**< 24 MHz TUNING value for USFRCO mask */
#define _DEVINFO_USHFRCOCAL0_BAND48_TUNING_SHIFT        16           /**< 24 MHz TUNING value for USFRCO shift */
#define _DEVINFO_USHFRCOCAL0_BAND48_FINETUNING_MASK     0x3F000000UL /**< 24 MHz FINETUNING value for USFRCO mask */
#define _DEVINFO_USHFRCOCAL0_BAND48_FINETUNING_SHIFT    24           /**< 24 MHz FINETUNING value for USFRCO shift */
#define _DEVINFO_AUXHFRCOCAL0_BAND1_MASK                0x000000FFUL /**< 1MHz tuning value for AUXHFRCO, mask */
#define _DEVINFO_AUXHFRCOCAL0_BAND1_SHIFT               0            /**< 1MHz tuning value for AUXHFRCO, shift */
#define _DEVINFO_AUXHFRCOCAL0_BAND7_MASK                0x0000FF00UL /**< 7MHz tuning value for AUXHFRCO, mask */
#define _DEVINFO_AUXHFRCOCAL0_BAND7_SHIFT               8            /**< 7MHz tuning value for AUXHFRCO, shift */
#define _DEVINFO_AUXHFRCOCAL0_BAND11_MASK               0x00FF0000UL /**< 11MHz tuning value for AUXHFRCO, mask */
#define _DEVINFO_AUXHFRCOCAL0_BAND11_SHIFT              16           /**< 11MHz tuning value for AUXHFRCO, shift */
#define _DEVINFO_AUXHFRCOCAL0_BAND14_MASK               0xFF000000UL /**< 14MHz tuning value for AUXHFRCO, mask */
#define _DEVINFO_AUXHFRCOCAL0_BAND14_SHIFT              24           /**< 14MHz tuning value for AUXHFRCO, shift */
#define _DEVINFO_AUXHFRCOCAL1_BAND21_MASK               0x000000FFUL /**< 21MHz tuning value for AUXHFRCO, mask */
#define _DEVINFO_AUXHFRCOCAL1_BAND21_SHIFT              0            /**< 21MHz tuning value for AUXHFRCO, shift */
#define _DEVINFO_HFRCOCAL0_BAND1_MASK                   0x000000FFUL /**< 1MHz tuning value for HFRCO, mask */
#define _DEVINFO_HFRCOCAL0_BAND1_SHIFT                  0            /**< 1MHz tuning value for HFRCO, shift */
#define _DEVINFO_HFRCOCAL0_BAND7_MASK                   0x0000FF00UL /**< 7MHz tuning value for HFRCO, mask */
#define _DEVINFO_HFRCOCAL0_BAND7_SHIFT                  8            /**< 7MHz tuning value for HFRCO, shift */
#define _DEVINFO_HFRCOCAL0_BAND11_MASK                  0x00FF0000UL /**< 11MHz tuning value for HFRCO, mask */
#define _DEVINFO_HFRCOCAL0_BAND11_SHIFT                 16           /**< 11MHz tuning value for HFRCO, shift */
#define _DEVINFO_HFRCOCAL0_BAND14_MASK                  0xFF000000UL /**< 14MHz tuning value for HFRCO, mask */
#define _DEVINFO_HFRCOCAL0_BAND14_SHIFT                 24           /**< 14MHz tuning value for HFRCO, shift */
#define _DEVINFO_HFRCOCAL1_BAND21_MASK                  0x000000FFUL /**< 21MHz tuning value for HFRCO, mask */
#define _DEVINFO_HFRCOCAL1_BAND21_SHIFT                 0            /**< 21MHz tuning value for HFRCO, shift */
#define _DEVINFO_MEMINFO_FLASH_PAGE_SIZE_MASK           0xFF000000UL /**< Flash page size (refer to ref.man for encoding) mask */
#define _DEVINFO_MEMINFO_FLASH_PAGE_SIZE_SHIFT          24           /**< Flash page size shift */
#define _DEVINFO_UNIQUEL_MASK                           0xFFFFFFFFUL /**< Lower part of  64-bit device unique number */
#define _DEVINFO_UNIQUEL_SHIFT                          0            /**< Unique Low 32-bit shift */
#define _DEVINFO_UNIQUEH_MASK                           0xFFFFFFFFUL /**< High part of  64-bit device unique number */
#define _DEVINFO_UNIQUEH_SHIFT                          0            /**< Unique High 32-bit shift */
#define _DEVINFO_MSIZE_SRAM_MASK                        0xFFFF0000UL /**< Flash size in kilobytes */
#define _DEVINFO_MSIZE_SRAM_SHIFT                       16           /**< Bit position for flash size */
#define _DEVINFO_MSIZE_FLASH_MASK                       0x0000FFFFUL /**< SRAM size in kilobytes */
#define _DEVINFO_MSIZE_FLASH_SHIFT                      0            /**< Bit position for SRAM size */
#define _DEVINFO_PART_PROD_REV_MASK                     0xFF000000UL /**< Production revision */
#define _DEVINFO_PART_PROD_REV_SHIFT                    24           /**< Bit position for production revision */
#define _DEVINFO_PART_DEVICE_FAMILY_MASK                0x00FF0000UL /**< Device Family, 0x47 for Gecko */
#define _DEVINFO_PART_DEVICE_FAMILY_SHIFT               16           /**< Bit position for device family */
/* Legacy family #defines */
#define _DEVINFO_PART_DEVICE_FAMILY_G                   71           /**< Gecko Device Family */
#define _DEVINFO_PART_DEVICE_FAMILY_GG                  72           /**< Giant Gecko Device Family */
#define _DEVINFO_PART_DEVICE_FAMILY_TG                  73           /**< Tiny Gecko Device Family */
#define _DEVINFO_PART_DEVICE_FAMILY_LG                  74           /**< Leopard Gecko Device Family */
#define _DEVINFO_PART_DEVICE_FAMILY_WG                  75           /**< Wonder Gecko Device Family */
#define _DEVINFO_PART_DEVICE_FAMILY_ZG                  76           /**< Zero Gecko Device Family */
#define _DEVINFO_PART_DEVICE_FAMILY_HG                  77           /**< Happy Gecko Device Family */
/* New style family #defines */
#define _DEVINFO_PART_DEVICE_FAMILY_EFM32G              71           /**< Gecko Device Family */
#define _DEVINFO_PART_DEVICE_FAMILY_EFM32GG             72           /**< Giant Gecko Device Family */
#define _DEVINFO_PART_DEVICE_FAMILY_EFM32TG             73           /**< Tiny Gecko Device Family */
#define _DEVINFO_PART_DEVICE_FAMILY_EFM32LG             74           /**< Leopard Gecko Device Family */
#define _DEVINFO_PART_DEVICE_FAMILY_EFM32WG             75           /**< Wonder Gecko Device Family */
#define _DEVINFO_PART_DEVICE_FAMILY_EFM32ZG             76           /**< Zero Gecko Device Family */
#define _DEVINFO_PART_DEVICE_FAMILY_EFM32HG             77           /**< Happy Gecko Device Family */
#define _DEVINFO_PART_DEVICE_FAMILY_EZR32WG             120          /**< EZR Wonder Gecko Device Family */
#define _DEVINFO_PART_DEVICE_FAMILY_EZR32LG             121          /**< EZR Leopard Gecko Device Family */
#define _DEVINFO_PART_DEVICE_FAMILY_EZR32HG             122          /**< EZR Happy Gecko Device Family */
#define _DEVINFO_PART_DEVICE_NUMBER_MASK                0x0000FFFFUL /**< Device number */
#define _DEVINFO_PART_DEVICE_NUMBER_SHIFT               0            /**< Bit position for device number */

/** LFRCO frequency, tuned to below frequency during manufacturing. */
#define EFM32_LFRCO_FREQ  (32768UL)
#define EFM32_ULFRCO_FREQ (1000UL)
#define DEPCTL_WO_BITMASK 0x3c000000UL
#define DCTL_WO_BITMASK \
          (_USB_DCTL_CGOUTNAK_MASK  | _USB_DCTL_SGOUTNAK_MASK | \
           _USB_DCTL_CGNPINNAK_MASK | _USB_DCTL_SGNPINNAK_MASK)
#define USB_DINEPS   ((USB_DIEP_TypeDef *) &USB->DIEP0CTL)
#define USB_DOUTEPS  ((USB_DOEP_TypeDef *) &USB->DOEP0CTL)
#define USB_FIFOS    ((uint32_t *) &USB->FIFO0D)
#define USB_DIEPTXFS ((uint32_t *) &USB->DIEPTXF1)

/* Limits imposed by the USB peripheral */
#define NP_RX_QUE_DEPTH       8
#define HP_RX_QUE_DEPTH       8
#define MAX_XFER_LEN          524287L         /* 2^19 - 1 bytes */
#define MAX_PACKETS_PR_XFER   1023            /* 2^10 - 1 packets */
#define MAX_NUM_TX_FIFOS      3               /* In addition to EP0 Tx FIFO */
#define MAX_NUM_IN_EPS        3               /* In addition to EP0 */
#define MAX_NUM_OUT_EPS       3               /* In addition to EP0 */
#define MAX_DEVICE_FIFO_SIZE_INWORDS 384U
#define MIN_EP_FIFO_SIZE_INWORDS  16U         /* Unit is words (32bit) */
#define MIN_EP_FIFO_SIZE_INBYTES  64U         /* Unit is bytes (8bit) */

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

static inline void watchdog_refresh(void)
{
    // Disable the watchdog timer prior to resetting it
    WDOG->CTRL &= ~WDOG_CTRL_EN;
    while (WDOG->SYNCBUSY & WDOG_SYNCBUSY_CTRL);

    // Enable the watchdog timer, which will cause it to reset the
    // system if the application doesn't clear it within two seconds.
    WDOG->CTRL = WDOG_CTRL_CLKSEL_ULFRCO
               // Reset after 2k ticks of the 1 kHz clocks.
               | (8 << _WDOG_CTRL_PERSEL_SHIFT)
               | WDOG_CTRL_EN;
}

struct efm32hg_rev
{
    uint8_t family;
    uint8_t minor;
    uint8_t major;
};

static inline void efm32hg_revno(struct efm32hg_rev *rev)
{
    uint8_t tmp;

    /* CHIP FAMILY bit [5:2] */
    tmp = ((ROMTABLE->PID1 & 0xf) << 2);
    /* CHIP FAMILY bit [1:0] */
    tmp |= ((ROMTABLE->PID0 & 0xc0) >> 6);
    rev->family = tmp;

    /* CHIP MAJOR bit [3:0] */
    rev->major = (ROMTABLE->PID0 & 0x3f);

    /* CHIP MINOR bit [7:4] */
    tmp = (((ROMTABLE->PID2 & 0xf0) >> 4) << 4);
    /* CHIP MINOR bit [3:0] */
    tmp |= ((ROMTABLE->PID3 & 0xf0) >> 4);
    rev->minor = tmp;
}


#define CORTEX_NUM_VECTORS 24

#define DMA_Handler Vector40
#define GPIO_EVEN_Handler Vector44
#define TIMER0_Handler Vector48
#define ACMP0_Handler Vector4C
#define ADC0_Handler Vector50
#define I2C0_Handler Vector54
#define GPIO_ODD_Handler Vector58
#define TIMER1_Handler Vector5C
#define USART1_RX_Handler Vector60
#define USART1_TX_Handler Vector64
#define LEUART0_Handler Vector68
#define PCNT0_Handler Vector6C
#define RTC_Handler Vector70
#define CMU_Handler Vector74
#define VCMP_Handler Vector78
#define MSC_Handler Vector7C
#define AES_Handler Vector80
#define USART0_RX_Handler Vector84
#define USART0_TX_Handler Vector88
#define USB_Handler Vector8C
#define TIMER2_Handler Vector90

#endif /* MCU_H */