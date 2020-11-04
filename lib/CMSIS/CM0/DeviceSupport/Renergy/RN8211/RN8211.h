/**************************************************************************//**
 * @file     RN8211.h
 * @brief    CMSIS Cortex-M0 Core Peripheral Access Layer Header File
 *           for RN8211 CM0 Device
 * @version  V1.3
 * @date     2017/3/17
 *
 * @note
 * Copyright (C) REnergy Micro-Electronics Co.,LTD.
 *
 *
 ******************************************************************************/


#ifndef __RN8211_H__
#define __RN8211_H__

#ifdef __cplusplus
 extern "C" {
#endif 
/** @addtogroup RN8211_Definitions RN8211 Definitions
  This file defines all structures and symbols for RN8211:
    - registers and bitfields
    - peripheral base address
    - peripheral ID
    - Peripheral definitions
  @{
*/

/******************************************************************************/
/*                Processor and Core Peripherals                              */
/******************************************************************************/
/** @addtogroup RN8211_CMSIS Device CMSIS Definitions
  Configuration of the Cortex-M0 Processor and Core Peripherals
  @{
*/



/*
 * ==========================================================================
 * ---------- Interrupt Number Definition -----------------------------------
 * ==========================================================================
 */

typedef enum IRQn
{
/******  Cortex-M0 Processor Exceptions Numbers ***************************************************/
  NonMaskableInt_IRQn         = -14,    /*!< 2 Non Maskable Interrupt                             */
  HardFault_IRQn              = -13,    /*!< 3 Cortex-M0 Hard Fault Interrupt                     */
  SVCall_IRQn                 = -5,     /*!< 11 Cortex-M0 SV Call Interrupt                       */
  PendSV_IRQn                 = -2,     /*!< 14 Cortex-M0 Pend SV Interrupt                       */
  SysTick_IRQn                = -1,     /*!< 15 Cortex-M0 System Tick Interrupt                   */

/******  RN8211 Specific Interrupt Numbers ********************************************************/
  SYSCLKCAL_IRQn               = 0,      /*!<  16 system clock calculation interrupt              */
  CMP_IRQn                     = 1,      /*!<  17 Voltage compare Interrupt                       */
  VCH_IRQn                     = 2,      /*!<  18 Power Switch Interrupt                          */
  RTC_IRQn                     = 3,      /*!<  19 RTC Interrupt                                   */
  EMU_IRQn                     = 4,      /*!<  20 EMU Interrupt                                   */
  MADC_IRQn                    = 5,      /*!<  21 Mux. ADC Interrupt                              */
  UART0_IRQn                   = 6,      /*!<  22 UART0 Interrupt                                 */
  UART1_IRQn                   = 7,      /*!<  23 UART1 Interrupt                                 */
  UART2_IRQn                   = 8,      /*!<  24 UART2 Interrupt                                 */
  I2C_IRQn                     = 11,     /*!<  27 IIC Interrupt                                   */
  TC0_IRQn                     = 14,     /*!<  30 Timer/Counter 0 Interrupt                       */
  UART5_IRQn                   = 17,     /*!<  33 UART5 Interrupt                                 */
  WDT_IRQn                     = 18,     /*!<  34 Watch Dog Interrupt                             */
  KBI_IRQn                     = 19,     /*!<  35 Key Interrupt                                   */
  LCD_IRQn                     = 20,     /*!<  36 LCD Interrupt                                   */
  NVM_IRQn                     = 23,     /*!<  39 NVM Interrupt                                   */
  EXT0_IRQn                    = 24,     /*!<  40 Extern0 Interrupt                               */
  EXT2_IRQn                    = 26,     /*!<  42 Extern2 Interrupt                               */
  EXT6_IRQn                    = 30,     /*!<  45 Extern6 Interrupt                               */
  EXT7_IRQn                    = 31,     /*!<  46 Extern7 Interrupt                               */
} IRQn_Type;


/*
 * ==========================================================================
 * ----------- Processor and Core Peripheral Section ------------------------
 * ==========================================================================
 */

/* Configuration of the Cortex-M0 Processor and Core Peripherals */
#define __MPU_PRESENT             0         /*!< MPU present or not                               */
#define __NVIC_PRIO_BITS          2         /*!< Number of Bits used for Priority Levels          */
#define __Vendor_SysTickConfig    0         /*!< Set to 1 if different SysTick Config is used     */

#if 0
#if defined ( __CC_ARM   )
  #if defined (__TARGET_FPU_VFP)
    #define __FPU_PRESENT         1         /*!< FPU present or not                               */
  #else
    #define __FPU_PRESENT         0         /*!< FPU present or not                               */
  #endif
#else
    #define __FPU_PRESENT         0         /*!< FPU present or not                               */
#endif
#endif

/*@}*/ /* end of group RN8211_CMSIS */

#include "core_cm0.h"                       /* Cortex-M0 processor and core peripherals           */
#include "system_RN8211.h"                  /* RN8211 System  include file                      */


/******************************************************************************/
/*                Device Specific Peripheral registers structures             */
/******************************************************************************/
/** @addtogroup RN8211_Peripherals RN8211 Peripherals
  RN8211 Device Specific Peripheral registers structures
  @{
*/

#if defined ( __CC_ARM   )
#pragma anon_unions
#endif

/*--------------------- GPIO --------------------------------------------------*/
typedef struct {
  __IO uint32_t PMA        ;               /* offset 0x0          PortA mode register          */
  __IO uint32_t PA         ;               /* offset 0x4          PortA data register          */
  __IO uint32_t PCA0       ;               /* offset 0x8          PortA control register0      */
  __IO uint32_t PCA1       ;               /* offset 0xc          PortA control register1      */
  __IO uint32_t PUA        ;               /* offset 0x10         PortA pull-up register       */
  __IO uint32_t PIMA       ;               /* offset 0x14         PortA input mode register    */
  __IO uint32_t PIEA       ;               /* offset 0x18         PortA input enable register  */
  __IO uint32_t PMB        ;               /* offset 0x1c         PortB mode register          */
  __IO uint32_t PB         ;               /* offset 0x20         PortB data register          */
  __IO uint32_t PCB        ;               /* offset 0x24         PortB control register       */
  __IO uint32_t PUB        ;               /* offset 0x28         PortB pull-up register       */
  __IO uint32_t PIMB       ;               /* offset 0x2c         PortB input mode register    */
  __IO uint32_t PIEB       ;               /* offset 0x30         PortB input enable register  */
  __IO uint32_t PMC        ;               /* offset 0x34         PortC mode register          */
  __IO uint32_t PC         ;               /* offset 0x38         PortC data register          */
  __IO uint32_t PCC        ;               /* offset 0x3c         PortC control register       */
  __IO uint32_t PUC        ;               /* offset 0x40         PortC pull-up register       */
  __IO uint32_t PIEC       ;               /* offset 0x44         PortC input enable register  */
  __IO uint32_t PIMC       ;              /* offset  0x48         */
  __IO uint32_t PCB2       ;              /* offset  0x4c         */
  __IO uint32_t RESERVED[4];       /* offset  0x50~0x5c    */
  __IO uint32_t PCE        ;               /* offset  0x60         */
  __IO uint32_t PASET      ;               /* offset 0x64         PortA data set register      */
  __IO uint32_t PACLR      ;               /* offset 0x68         PortA data clear register    */
  __IO uint32_t PBSET      ;               /* offset 0x6c         PortB data set register      */
  __IO uint32_t PBCLR      ;               /* offset 0x70         PortB data clear register    */
  __IO uint32_t PCSET      ;               /* offset 0x74         PortC data set register      */
  __IO uint32_t PCCLR      ;               /* offset 0x78         PortC data clear register    */
} GPIO_TypeDef;

/*--------------------- EMU --------------------------------------------------*/
typedef struct {
  __IO uint32_t EMUCON        ;             /* offset 0x0          */
  __IO uint32_t RESERVED1     ;             /* offset 0x4          */
  __IO uint32_t HFConst       ;             /* offset 0x8          */
  __IO uint32_t PStart        ;             /* offset 0xc          */
  __IO uint32_t RESERVED2     ;             /* offset 0x10         */
  __IO uint32_t GPQA          ;             /* offset 0x14         */
  __IO uint32_t RESERVED3     ;             /* offset 0x18         */
  __IO uint32_t PhsA          ;             /* offset 0x1c         */
  __IO uint32_t RESERVED4[2]  ;             /* offset 0x20~0x24    */
  __IO uint32_t APOSA         ;             /* offset 0x28         */
  __IO uint32_t RESERVED5[3]  ;             /* offset 0x2c~0x34    */
  __IO uint32_t IARMSOS       ;             /* offset 0x38         */
  __IO uint32_t RESERVED6     ;             /* offset 0x3c         */
  __IO uint32_t URMSOS        ;             /* offset 0x40         */
  __IO uint32_t RESERVED7[47] ;             /* offset 0x44~0xfc    */
  __IO uint32_t PFCnt         ;             /* offset 0x100        */
  __IO uint32_t RESERVED8[2]  ;             /* offset 0x104~0x108  */
  __IO uint32_t IARMS         ;			        /* offset 0x10c        */
  __IO uint32_t RESERVED9     ;             /* offset 0x110        */
  __IO uint32_t URMS          ;             /* offset 0x114        */
  __IO uint32_t Ufreq         ;             /* offset 0x118        */
  __IO uint32_t PowerPA       ;             /* offset 0x11c        */
  __IO uint32_t RESERVED10[5] ;             /* offset 0x120~0x130  */
  __IO uint32_t EnergyP       ;             /* offset 0x134        */
  __IO uint32_t RESERVED11[8] ;             /* offset 0x138~0x154  */
  __IO uint32_t EMUStatus     ;             /* offset 0x158        */
  __IO uint32_t RESERVED12[12];             /* offset 0x15c~0x188  */
  __IO uint32_t IE            ;             /* offset 0x18c        */
  __IO uint32_t IF            ;             /* offset 0x190        */
  __IO uint32_t RESERVED13    ;             /* offset 0x194        */
  __IO uint32_t Rdata         ;             /* offset 0x198        */
  __IO uint32_t Wdata         ;             /* offset 0x19c        */
  __IO uint32_t RESERVED14[2] ;             /* offset 0x1a0~0x1a4  */
  __IO uint32_t SPCMD         ;             /* offset 0x1a8        */
} EMU_TypeDef;

/*--------------------- UART --------------------------------------------------*/
typedef struct {
  __IO uint32_t CTRL;
  __IO uint32_t BAUD;
  __IO uint32_t STA;
  __IO uint32_t TXD;
  __IO uint32_t RXD;
  __IO uint32_t FDIV;
} UART_TypeDef;

/*--------------------- TC --------------------------------------------------*/
typedef struct {
  __IO uint32_t CNT         ;              /* offset 0x0    */
  __IO uint32_t PS          ;              /* offset 0x4    */
  __IO uint32_t RESERVED1   ;              /* offset 0x8    */
  __IO uint32_t DN          ;              /* offset 0xc    */
  __IO uint32_t RESERVED2   ;              /* offset 0x10   */
  __IO uint32_t CCD0        ;              /* offset 0x14   */
  __IO uint32_t CCD1        ;              /* offset 0x18   */
  __IO uint32_t CCFG        ;              /* offset 0x1c   */
  __IO uint32_t CTRL        ;              /* offset 0x20   */
  __IO uint32_t CM0         ;              /* offset 0x24   */
  __IO uint32_t CM1         ;              /* offset 0x28   */
  __IO uint32_t IE          ;              /* offset 0x2c   */
  __IO uint32_t STA         ;              /* offset 0x30   */
} TC_TypeDef;                                       
   
/*--------------------- I2C --------------------------------------------------*/
typedef struct {
  __IO uint32_t CTRL     ;              /* offset 0x0 */
  __IO uint32_t CLK      ;              /* offset 0x4 */
  __IO uint32_t STA      ;              /* offset 0x8 */
  __IO uint32_t ADDR     ;              /* offset 0xc */
  __IO uint32_t DATA     ;              /* offset 0x10 */
} I2C_TypeDef; 

/*--------------------- KBI --------------------------------------------------*/
typedef struct {
  __IO uint32_t CTRL     ;              /* offset 0x0          KBI control register         */
  __IO uint32_t SEL      ;              /* offset 0x4          KBI select register          */
  __IO uint32_t DATA     ;              /* offset 0x8          KBI data register            */
  __IO uint32_t MASK     ;              /* offset 0xc          KBI mask register            */
} KBI_TypeDef;

/*--------------------- WDT --------------------------------------------------*/
typedef struct {
  __IO uint32_t EN       ;              /* offset 0x0 */
} WDT_TypeDef;


/*--------------------- SYSCTL --------------------------------------------------*/
typedef struct {
  __IO uint32_t OSC_CTRL1;           /* offset = 0x0   */       
  __IO uint32_t SYS_MODE;            /* offset = 0x4   */
  __IO uint32_t SYS_PD;              /* offset = 0x8   */
  __IO uint32_t ADC_CTRL;            /* offset = 0xc   */
  __IO uint32_t OSC_CTRL2;           /* offset = 0x10   */
  __IO uint32_t SYS_RST;             /* offset = 0x14   */
  __IO uint32_t MAP_CTRL;            /* offset = 0x18   */
  __IO uint32_t MOD0_EN;             /* offset = 0x1c   */ 
  __IO uint32_t MOD1_EN;             /* offset = 0x20   */
  __IO uint32_t INTC_EN;             /* offset = 0x24   */
  __IO uint32_t KBI_EN;              /* offset = 0x28   */
  __IO uint32_t CHIP_ID;             /* offset = 0x2c   */
  __IO uint32_t SYS_PS;              /* offset = 0x30   */
  __IO uint32_t IRFR_CTRL;               /* offset = 0x34    */
  __IO uint32_t RESERVED[16];            /* offset = 0x68~0x6c */
  __IO uint32_t TRIM_CFG1;              /* offset = 0x78    */
  __IO uint32_t TRIM_START;               /* offset = 0x7c    */
} SYSCTL_TypeDef;

/*-------------------- RTC ---------------------------------------------------*/
typedef struct
{
  __IO uint32_t CTRL        ;           /* offset = 0x00     */
  __IO uint32_t SC          ;           /* offset = 0x04     */
  __IO uint32_t MN          ;           /* offset = 0x08     */
  __IO uint32_t HR          ;           /* offset = 0x0c     */
  __IO uint32_t DT          ;           /* offset = 0x10     */
  __IO uint32_t MO          ;           /* offset = 0x14     */
  __IO uint32_t YR          ;           /* offset = 0x18     */
  __IO uint32_t DW          ;           /* offset = 0x1c     */
  __IO uint32_t CNT1        ;           /* offset = 0x20     */
  __IO uint32_t CNT2        ;           /* offset = 0x24     */
  __IO uint32_t SCA         ;           /* offset = 0x28     */
  __IO uint32_t MNA         ;           /* offset = 0x2c     */
  __IO uint32_t HRA         ;           /* offset = 0x30     */
  __IO uint32_t IE          ;           /* offset = 0x34     */
  __IO uint32_t IF          ;           /* offset = 0x38     */
  __IO uint32_t TEMP        ;           /* offset = 0x3c     */
  __IO uint32_t RESERVED[32];           /* offset = 0x40~BF  */
  __IO uint32_t DMAEN       ;           /* offset = 0xC0     */
} RTC_TypeDef;


/*-------------------- MADC ---------------------------------------------------*/
typedef struct
{
  __IO uint32_t AD_CTRL    ;            /* offset = 0x      */
  __IO uint32_t AD_START   ;            /* offset = 0x      */
  __IO uint32_t AD_STAT    ;            /* offset = 0x      */
  __IO uint32_t AD_DATA    ;            /* offset = 0x      */
  __IO uint32_t LVD_CTRL   ;            /* offset = 0x      */
  __IO uint32_t LVD_STAT   ;            /* offset = 0x      */
} MADC_TypeDef;

/*--------------------- INTC --------------------------------------------------*/
typedef struct {
  __IO uint32_t CTRL       ;            /* offset = 0x         interrupt control register   */
  __IO uint32_t MODE       ;            /* offset = 0x         interrupt mode register      */
  __IO uint32_t MASK       ;            /* offset = 0x         interrupt mask register      */
  __IO uint32_t STA        ;            /* offset = 0x         interrupt status register    */
} INTC_TypeDef; 

/*--------------------- LCD --------------------------------------------------*/
typedef struct {
  __IO uint32_t CTRL       ;            /* offset = 0x       */
  __IO uint32_t STA        ;            /* offset = 0x       */
  __IO uint32_t CLKDIV     ;            /* offset = 0x       */
  __IO uint32_t BLINK      ;            /* offset = 0x       */
  __IO uint32_t PS         ;            /* offset = 0x       */
  __IO uint32_t RESCTL     ;            /* offset = 0x       */
  __IO uint32_t RESERVED[2];            /* offset = 0x       */
  __IO uint8_t  BUF[32]    ;            /* offset = 0x       */
} LCD_TypeDef;
      
/*--------------------- NVM --------------------------------------------------*/
typedef struct {
  __IO uint32_t IE  ;       
  __IO uint32_t IF  ;      
  __IO uint32_t LSCFG;
  __IO uint32_t LSDCOS;     
  __IO uint32_t LSTHO;      
  __IO uint32_t LSRMS;      
  __IO uint32_t LSRMS1;     
  __IO uint32_t DMAEN;   
  __IO uint32_t HFCONST;      
  __IO uint32_t D2FP0;     
  __IO uint32_t D2FP1;  
} NVM_TypeDef;

/******************************************************************************/
/*                         Peripheral memory map                              */
/******************************************************************************/
/* Peripheral and SRAM base address */
#define LS_PERIPH_BASE                  ((uint32_t )0x40000000             )
#define HS_PERIPH_BASE                  ((uint32_t )0x50000000             )

#define FLASH_MEMORY_BASE               ((uint32_t )0x00000000             )
#define EEPROM_MEMORY_BASE              ((uint32_t )0x08000000             )

//AHB base address
#define BB_OFFSET                       0x02000000
#define HSP_BASE                        HS_PERIPH_BASE
#define HSP_BB_BASE                     (HSP_BASE + BB_OFFSET                 )
#define GPIO_BASE                       HSP_BASE
#define GPIO_BB_BASE                    (HSP_BASE + BB_OFFSET                 )
#define EMU_BASE                        (HSP_BASE + 0x4000                    )
#define ROM_TABLE_BASE                  0xF000_0000

//APB base address
#define LSP_BASE                        LS_PERIPH_BASE
#define LSP_BB_BASE                     (LSP_BASE + BB_OFFSET                  )
#define UART0_BASE                      (LSP_BASE                              )
#define UART0_BB_BASE                   (LSP_BASE + BB_OFFSET                  )
#define UART1_BASE                      (LSP_BASE + 0x04000                    )
#define UART1_BB_BASE                   (LSP_BASE + BB_OFFSET + 0x04000 * 32   )
#define UART2_BASE                      (LSP_BASE + 0x08000                    )
#define UART2_BB_BASE                   (LSP_BASE + BB_OFFSET + 0x08000 * 32   )
#define TC0_BASE                        (LSP_BASE + 0x10000                    )
#define TC0_BB_BASE                     (LSP_BASE + BB_OFFSET + 0x10000 * 32   )
#define UART5_BASE                      (LSP_BASE + 0x1C000                    )
#define UART5_BB_BASE                   (LSP_BASE + BB_OFFSET + 0x1C000 * 32   )
#define I2C_BASE                        (LSP_BASE + 0x24000                    )
#define I2C_BB_BASE                     (LSP_BASE + BB_OFFSET + 0x24000 * 32   )
#define KBI_BASE                        (LSP_BASE + 0x28000                    )
#define MADC_BASE                       (LSP_BASE + 0x2C000                    )
#define WDT_BASE                        (LSP_BASE + 0x30000                    )
#define SYSCTL_BASE                     (LSP_BASE + 0x34000                    )
#define RTC_BASE                        (LSP_BASE + 0x3C000                    )
#define INTC_BASE                       (LSP_BASE + 0x44000                    )
#define LCD_BASE                        (LSP_BASE + 0x48000                    )
#define NVM_BASE                        (LSP_BASE + 0x40000                   )

/******************************************************************************/
/*                         Peripheral declaration                             */
/******************************************************************************/
#define GPIO                            ((GPIO_TypeDef    *) GPIO_BASE          )
#define EMU                             ((EMU_TypeDef     *) EMU_BASE           )

#define UART0                           ((UART_TypeDef    *) UART0_BASE         )
#define UART1                           ((UART_TypeDef    *) UART1_BASE         )
#define UART2                           ((UART_TypeDef    *) UART2_BASE         )
                                        
#define TC0                             ((TC_TypeDef      *) TC0_BASE           )
#define UART5                           ((UART_TypeDef      *) UART5_BASE           )
#define I2C                             ((I2C_TypeDef     *) I2C_BASE           )
#define KBI                             ((KBI_TypeDef     *) KBI_BASE           )
#define MADC                            ((MADC_TypeDef    *) MADC_BASE          )
#define WDT                             ((WDT_TypeDef     *) WDT_BASE           )

#define SYSCTL                          ((SYSCTL_TypeDef  *) SYSCTL_BASE        )
#define RTC                             ((RTC_TypeDef     *) RTC_BASE           )
#define INTC                            ((INTC_TypeDef    *) INTC_BASE          )
#define LCD                             ((LCD_TypeDef     *) LCD_BASE           )
#define NVM                             ((NVM_TypeDef     *) NVM_BASE           )

#if 0
#if defined ( __CC_ARM   )
  #if defined (__TARGET_FPU_VFP)
    #define __FPU_PRESENT         1         /*!< FPU present or not                               */
  #else
    #define __FPU_PRESENT         0         /*!< FPU present or not                               */
  #endif
#else
    #define __FPU_PRESENT         0         /*!< FPU present or not                               */
#endif
#endif

/*@}*/ /* end of group RN8211_CMSIS */

#include "core_cm0.h"                       /* Cortex-M0 processor and core peripherals           */
#include "system_RN8211.h"                  /* RN8211 System  include file                      */


/******************************************************************************/
/*                Device Specific Peripheral registers structures             */
/******************************************************************************/
/** @addtogroup RN8211_Peripherals RN8211 Peripherals
  RN8211 Device Specific Peripheral registers structures
  @{
*/

#if defined ( __CC_ARM   )
#pragma anon_unions
#endif


#ifdef __cplusplus
}
#endif


//#endif

#endif  // __RN8211_H__
