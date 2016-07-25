/**************************************************************************//**
 * @file     NUC123.h
 * @version  V3.0
 * $Revision: 46 $
 * $Date: 8/05/14 1:47p $
 * @brief    NUC123 Series Peripheral Access Layer Header File
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/



/**
  \mainpage Introduction
  *
  *
  * This user manual describes the usage of NUC123 Series MCU device driver
  *
  * <b>Disclaimer</b>
  *
  * The Software is furnished "AS IS", without warranty as to performance or results, and
  * the entire risk as to performance or results is assumed by YOU. Nuvoton disclaims all
  * warranties, express, implied or otherwise, with regard to the Software, its use, or
  * operation, including without limitation any and all warranties of merchantability, fitness
  * for a particular purpose, and non-infringement of intellectual property rights.
  *
  * <b>Copyright Notice</b>
  *
  * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
  */

/**
  * \page PG_REV Revision History
  *
  *
  * <b>Revision 3.00.001</b>
  * \li Updated to support new API
*/

#ifndef __NUC123_H__
#define __NUC123_H__

/*
 * ==========================================================================
 * ---------- Interrupt Number Definition -----------------------------------
 * ==========================================================================
*/

/** @addtogroup MCU_CMSIS Device Definitions for CMSIS
  Interrupt Number Definition and Configurations for CMSIS
  @{
*/

/**
 * @details  Interrupt Number Definition. The maximum of 32 Specific Interrupts are possible.
 */

typedef enum IRQn
{
    /******  Cortex-M0 Processor Exceptions Numbers ***************************************************/
    NonMaskableInt_IRQn         = -14,    /*!< 2 Non Maskable Interrupt                             */
    HardFault_IRQn              = -13,    /*!< 3 Cortex-M0 Hard Fault Interrupt                     */
    SVCall_IRQn                 = -5,     /*!< 11 Cortex-M0 SV Call Interrupt                       */
    PendSV_IRQn                 = -2,     /*!< 14 Cortex-M0 Pend SV Interrupt                       */
    SysTick_IRQn                = -1,     /*!< 15 Cortex-M0 System Tick Interrupt                   */

    /******  ARMIKMCU Swift specific Interrupt Numbers ************************************************/
    BOD_IRQn                  = 0,        /*!< Brown-Out Low Voltage Detected Interrupt             */
    WDT_IRQn                  = 1,        /*!< Watch Dog Timer Interrupt                            */
    EINT0_IRQn                = 2,        /*!< EINT0 Interrupt                                      */
    EINT1_IRQn                = 3,        /*!< EINT1 Interrupt                                      */
    GPAB_IRQn                 = 4,        /*!< GPIO_PA/PB Interrupt                                 */
    GPCDF_IRQn                = 5,        /*!< GPIO_PC/PD/PF Interrupt                              */
    PWMA_IRQn                 = 6,        /*!< PWMA Interrupt                                       */
    TMR0_IRQn                 = 8,        /*!< TIMER0 Interrupt                                     */
    TMR1_IRQn                 = 9,        /*!< TIMER1 Interrupt                                     */
    TMR2_IRQn                 = 10,       /*!< TIMER2 Interrupt                                     */
    TMR3_IRQn                 = 11,       /*!< TIMER3 Interrupt                                     */
    UART0_IRQn                = 12,       /*!< UART0 Interrupt                                      */
    UART1_IRQn                = 13,       /*!< UART1 Interrupt                                      */
    SPI0_IRQn                 = 14,       /*!< SPI0 Interrupt                                       */
    SPI1_IRQn                 = 15,       /*!< SPI1 Interrupt                                       */
    SPI2_IRQn                 = 16,       /*!< SPI2 Interrupt                                       */
    I2C0_IRQn                 = 18,       /*!< I2C0 Interrupt                                       */
    I2C1_IRQn                 = 19,       /*!< I2C1 Interrupt                                       */
    CAN0_IRQn                 = 20,       /*!< CAN0 Interrupt                                       */
    CAN1_IRQn                 = 21,       /*!< CAN1 Interrupt                                       */
    USBD_IRQn                 = 23,       /*!< USB device Interrupt                                 */
    PS2_IRQn                  = 24,       /*!< PS/2 device Interrupt                                */
    PDMA_IRQn                 = 26,       /*!< PDMA Interrupt                                       */
    I2S_IRQn                  = 27,       /*!< I2S Interrupt                                        */
    PWRWU_IRQn                = 28,       /*!< Power Down Wake Up Interrupt                         */
    ADC_IRQn                  = 29,       /*!< ADC Interrupt                                        */
    IRC_IRQn                  = 30,       /*!< IRC TRIM Interrupt                                   */
} IRQn_Type;


/*
 * ==========================================================================
 * ----------- Processor and Core Peripheral Section ------------------------
 * ==========================================================================
 */

/* Configuration of the Cortex-M0 Processor and Core Peripherals */
#define __MPU_PRESENT           0       /*!< armikcmu does not provide a MPU present or not       */
#define __NVIC_PRIO_BITS        2       /*!< armikcmu Supports 2 Bits for the Priority Levels     */
#define __Vendor_SysTickConfig  0       /*!< Set to 1 if different SysTick Config is used         */


/*@}*/ /* end of group MCU_CMSIS */


#include "core_cm0.h"                   /* Cortex-M0 processor and core peripherals               */
#include "system_NUC123.h"              /* NUC123 System                                          */

#if defined ( __CC_ARM   )
#pragma anon_unions
#endif


/*-------------------------------- Device Specific Peripheral registers structures ---------------------*/
/** @addtogroup REGISTER Peripheral Register
  Peripheral Control Registers
  @{
 */

/*----------------------------- ADC Controller -------------------------------*/
/** @addtogroup REG_ADC ADC
  Memory Mapped Structure for ADC Controller
  @{
 */
typedef struct
{

    /**
     * ADDRx
     * ===================================================================================================
     * Offset: 0x00-0x1C  ADC Data Register x
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[9:0]   |RSLT      |A/D Conversion Result
     * |        |          |This field contains conversion result of ADC.
     * |[16]    |OVERRUN   |Overrun Flag
     * |        |          |0 = Data in RSLT (ADDRx[9:0], x=0~7) is recent conversion result.
     * |        |          |1 = Data in RSLT (ADDRx[9:0], x=0~7) is overwritten.
     * |        |          |If converted data in RSLT has not been read before new conversion result is loaded to this register, OVERRUN is set to 1 and previous conversion result is gone.
     * |        |          |It is cleared by hardware after ADDR register is read.
     * |        |          |This is a read only bit.
     * |[17]    |VALID     |Valid Flag
     * |        |          |0 = Data in RSLT bits (ADDRx[9:0], x=0~7) is not valid.
     * |        |          |1 = Data in RSLT bits (ADDRx[9:0], x=0~7) is valid.
     * |        |          |This bit is set to 1 when corresponding channel analog input conversion is completed and cleared by hardware after ADDR register is read.
     * |        |          |This is a read only bit
     */
    __I  uint32_t ADDR[8];

    /**
     * ADCR
     * ===================================================================================================
     * Offset: 0x20  ADC Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ADEN      |A/D Converter Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Before starting A/D conversion function, this bit should be set to 1.
     * |        |          |Clear it to 0 to disable A/D converter analog circuit for saving power consumption.
     * |[1]     |ADIE      |A/D Interrupt Enable
     * |        |          |0 = A/D interrupt function Disabled.
     * |        |          |1 = A/D interrupt function Enabled.
     * |        |          |A/D conversion end interrupt request is generated if ADIE bit (ADCR[1]) is set to 1.
     * |[3:2]   |ADMD      |A/D Converter Operation Mode
     * |        |          |00 = Single conversion.
     * |        |          |01 = Reserved.
     * |        |          |10 = Single-cycle scan.
     * |        |          |11 = Continuous scan.
     * |        |          |When changing the operation mode, software should disable ADST bit (ADCR[11]) firstly.
     * |[5:4]   |TRGS      |Hardware Trigger Source
     * |        |          |00 = A/D conversion is started by external STADC pin.
     * |        |          |11 = A/D conversion is started by PWM Center-aligned trigger.
     * |        |          |Others = Reserved.
     * |        |          |Software should disable TRGEN (ADCR[8]) and ADST (ADCR[11]) before change TRGS.
     * |[7:6]   |TRGCOND   |External Trigger Condition
     * |        |          |These two bits decide external pin STADC trigger event is level or edge.
     * |        |          |The signal must be kept at stable state at least 8 PCLKs for level trigger and 4 PCLKs at high and low state for edge trigger.
     * |        |          |00 = Low level.
     * |        |          |01 = High level.
     * |        |          |10 = Falling edge.
     * |        |          |11 = Rising edge.
     * |[8]     |TRGEN     |Hardware Trigger Enable
     * |        |          |Enable or disable triggering of A/D conversion by hardware (external STADC pin or PWM Center-aligned trigger).
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |ADC hardware trigger function is only supported in single-cycle scan mode.
     * |        |          |If hardware trigger mode, the ADST bit (ADCR[11]) can be set to 1 by the selected hardware trigger source.
     * |[9]     |PTEN      |PDMA Transfer Enable
     * |        |          |0 = PDMA data transfer Disabled.
     * |        |          |1 = PDMA data transfer in ADDR 0~7 Enabled.
     * |        |          |When A/D conversion is completed, the converted data is loaded into ADDR 0~7, software can enable this bit to generate a PDMA data transfer request.
     * |        |          |When PTEN=1, software must set ADIE=0 (ADCR[1]) to disable interrupt.
     * |[11]    |ADST      |A/D Conversion Start
     * |        |          |0 = Conversion stops and A/D converter enter idle state.
     * |        |          |1 = Conversion starts.
     * |        |          |ADST bit can be set to 1 from three sources: software, PWM Center-aligned trigger and external pin STADC.
     * |        |          |ADST will be cleared to 0 by hardware automatically at the ends of single mode and single-cycle scan mode.
     * |        |          |In continuous scan mode, A/D conversion is continuously performed until software writes 0 to this bit or chip reset.
     */
    __IO uint32_t ADCR;

    /**
     * ADCHER
     * ===================================================================================================
     * Offset: 0x24  ADC Channel Enable Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |CHEN      |Analog Input Channel Enable
     * |        |          |Set CHEN[7:0] to enable the corresponding analog input channel 7 ~ 0.
     * |        |          |If DIFFEN bit (ADCR[10]) is set to 1, only the even number channels need to be enabled.
     * |        |          |0 = ADC input channel Disabled.
     * |        |          |1 = ADC input channel Enabled.
     * |[8]     |PRESEL    |Analog Input Channel 7 Select
     * |        |          |0 = External analog input.
     * |        |          |1 = Internal band-gap voltage.
     * |        |          |Note:
     * |        |          |When software select the band-gap voltage as the analog input source of ADC channel 7, ADC clock rate needs to be limited to slower than 300 kHz.
     */
    __IO uint32_t ADCHER;

    /**
     * ADCMPRx
     * ===================================================================================================
     * Offset: 0x28-0x2C  ADC Compare Register x
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CMPEN     |Compare Enable
     * |        |          |0 = Compare function Disabled.
     * |        |          |1 = Compare function Enabled.
     * |        |          |Set this bit to 1 to enable ADC controller to compare CMPD (ADCMPR0/1[27:16]) with specified channel conversion result when converted data is loaded into ADDR register.
     * |[1]     |CMPIE     |Compare Interrupt Enable
     * |        |          |0 = Compare function interrupt Disabled.
     * |        |          |1 = Compare function interrupt Enabled.
     * |        |          |If the compare function is enabled and the compare condition matches the setting of CMPCOND (ADCMPR0/1[2]) and CMPMATCNT (ADCMPR0/1[11:8]), CMPF0/1 bit (ADSR[1]/[2]) will be asserted, in the meanwhile, if CMPIE (ADCMPR0/1[1]) is set to 1, a compare interrupt request is generated.
     * |[2]     |CMPCOND   |Compare Condition
     * |        |          |0 = Set the compare condition as that when a 12-bit A/D conversion result is less than the 12-bit CMPD (ADCMPR0/1[27:16]), the internal match counter will increase one.
     * |        |          |1 = Set the compare condition as that when a 12-bit A/D conversion result is greater or equal to the 12-bit CMPD (ADCMPR0/1[27:16]), the internal match counter will increase one.
     * |        |          |Note: When the internal counter reaches the value to (CMPMATCNT (ADCMPR0/1[11:8])+1), the CMPF0/1 bit (ADSR[1]/[2]) will be set.
     * |[5:3]   |CMPCH     |Compare Channel Selection
     * |        |          |000 = Channel 0 conversion result is selected to be compared.
     * |        |          |001 = Channel 1 conversion result is selected to be compared.
     * |        |          |010 = Channel 2 conversion result is selected to be compared.
     * |        |          |011 = Channel 3 conversion result is selected to be compared.
     * |        |          |100 = Channel 4 conversion result is selected to be compared.
     * |        |          |101 = Channel 5 conversion result is selected to be compared.
     * |        |          |110 = Channel 6 conversion result is selected to be compared.
     * |        |          |111 = Channel 7 conversion result is selected to be compared.
     * |[11:8]  |CMPMATCNT |Compare Match Count
     * |        |          |When the specified A/D channel analog conversion result matches the compare condition defined by CMPCOND (ADCMPR0/1[2]), the internal match counter will increase 1.
     * |        |          |When the internal counter reaches the value to (CMPMATCNT (ADCMPR0/1[11:8]) +1), the CMPF0/1 bit (ADSR[1]/[2]) will be set.
     * |[27:16] |CMPD      |Comparison Data
     * |        |          |The 12-bit data is used to compare with conversion result of specified channel.
     */
    __IO uint32_t ADCMPR[2];

    /**
     * ADSR
     * ===================================================================================================
     * Offset: 0x30  ADC Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ADF       |A/D Conversion End Flag
     * |        |          |A status flag that indicates the end of A/D conversion.
     * |        |          |ADF is set to 1 at these two conditions:
     * |        |          |1. When A/D conversion ends in Single mode.
     * |        |          |2. When A/D conversion ends on all specified channels in Scan mode.
     * |        |          |This flag can be cleared by writing 1 to itself.
     * |[1]     |CMPF0     |Compare Flag
     * |        |          |When the selected channel A/D conversion result meets setting condition in ADCMPR0 then this bit is set to 1.
     * |        |          |And it is cleared by writing 1 to self.
     * |        |          |0 = Conversion result in ADDR does not meet ADCMPR0 setting.
     * |        |          |1 = Conversion result in ADDR meets ADCMPR0 setting.
     * |[2]     |CMPF1     |Compare Flag
     * |        |          |When the selected channel A/D conversion result meets setting condition in ADCMPR1 then this bit is set to 1.
     * |        |          |And it is cleared by writing 1 to self.
     * |        |          |0 = Conversion result in ADDR does not meet ADCMPR1 setting.
     * |        |          |1 = Conversion result in ADDR meets ADCMPR1 setting.
     * |[3]     |BUSY      |BUSY/IDLE
     * |        |          |0 = A/D converter is in idle state.
     * |        |          |1 = A/D converter is busy at conversion.
     * |        |          |This bit is mirror of as ADST bit (ADCR[11]).
     * |        |          |It is read only.
     * |[6:4]   |CHANNEL   |Current Conversion Channel
     * |        |          |This field reflects the current conversion channel when BUSY = 1 (ADSR[3]).
     * |        |          |When BUSY = 0, it shows the number of the next converted channel.
     * |        |          |It is read only.
     * |[15:8]  |VALID     |Data Valid Flag
     * |        |          |It is a mirror of VALID bit (ADDR0~7[17]).
     * |        |          |It is read only.
     * |[23:16] |OVERRUN   |Overrun Flag
     * |        |          |It is a mirror to OVERRUN bit (ADDR0~7[16]).
     * |        |          |It is read only.
     */
    __IO uint32_t ADSR;

    /**
     * @cond HIDDEN_SYMBOLS
     */
    __I  uint32_t RESERVE0[3];
    /**
     * @endcond
     */

    /**
     * ADPDMA
     * ===================================================================================================
     * Offset: 0x40  ADC PDMA Current Transfer Data Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[9:0]   |AD_PDMA   |ADC PDMA Current Transfer Data Register
     * |        |          |When PDMA transferring, read this register can monitor current PDMA transfer data.
     * |        |          |Current PDMA transfer data is the content of ADDR0 ~ ADDR7.
     * |        |          |This is a read only register.
     */
    __I  uint32_t ADPDMA;
} ADC_T;

/** @addtogroup REG_ADC_BITMASK ADC Bit Mask
  @{
 */

/* ADDR Bit Field Definitions */
#define ADC_ADDR_VALID_Pos      17                                /*!< ADC ADDR: VALID Position */
#define ADC_ADDR_VALID_Msk      (1ul << ADC_ADDR_VALID_Pos)       /*!< ADC ADDR: VALID Mask */

#define ADC_ADDR_OVERRUN_Pos    16                                /*!< ADC ADDR: OVERRUN Position */
#define ADC_ADDR_OVERRUN_Msk    (1ul << ADC_ADDR_OVERRUN_Pos)     /*!< ADC ADDR: OVERRUN Mask */

#define ADC_ADDR_RSLT_Pos       0                                 /*!< ADC ADDR: RSLT Position */
#define ADC_ADDR_RSLT_Msk       (0x3FFul << ADC_ADDR_RSLT_Pos)    /*!< ADC ADDR: RSLT Mask */

/* ADCR Bit Field Definitions */
#define ADC_ADCR_ADST_Pos       11                                /*!< ADC ADCR: ADST Position */
#define ADC_ADCR_ADST_Msk       (1ul << ADC_ADCR_ADST_Pos)        /*!< ADC ADCR: ADST Mask */

#define ADC_ADCR_PTEN_Pos       9                                 /*!< ADC ADCR: PTEN Position */
#define ADC_ADCR_PTEN_Msk       (1ul << ADC_ADCR_PTEN_Pos)        /*!< ADC ADCR: PTEN Mask */

#define ADC_ADCR_TRGEN_Pos      8                                 /*!< ADC ADCR: TRGEN Position */
#define ADC_ADCR_TRGEN_Msk      (1ul << ADC_ADCR_TRGEN_Pos)       /*!< ADC ADCR: TRGEN Mask */

#define ADC_ADCR_TRGCOND_Pos    6                                 /*!< ADC ADCR: TRGCOND Position */
#define ADC_ADCR_TRGCOND_Msk    (3ul << ADC_ADCR_TRGCOND_Pos)     /*!< ADC ADCR: TRGCOND Mask */

#define ADC_ADCR_TRGS_Pos       4                                 /*!< ADC ADCR: TRGS Position */
#define ADC_ADCR_TRGS_Msk       (3ul << ADC_ADCR_TRGS_Pos)        /*!< ADC ADCR: TRGS Mask */

#define ADC_ADCR_ADMD_Pos       2                                 /*!< ADC ADCR: ADMD Position */
#define ADC_ADCR_ADMD_Msk       (3ul << ADC_ADCR_ADMD_Pos)        /*!< ADC ADCR: ADMD Mask */

#define ADC_ADCR_ADIE_Pos       1                                 /*!< ADC ADCR: ADIE Position */
#define ADC_ADCR_ADIE_Msk       (1ul << ADC_ADCR_ADIE_Pos)        /*!< ADC ADCR: ADIE Mask */

#define ADC_ADCR_ADEN_Pos       0                                 /*!< ADC ADCR: ADEN Position */
#define ADC_ADCR_ADEN_Msk       (1ul << ADC_ADCR_ADEN_Pos)        /*!< ADC ADCR: ADEN Mask */

/* ADCHER Bit Field Definitions */
#define ADC_ADCHER_PRESEL_Pos   8                                 /*!< ADC ADCHER: PRESEL Position */
#define ADC_ADCHER_PRESEL_Msk   (1ul << ADC_ADCHER_PRESEL_Pos)    /*!< ADC ADCHER: PRESEL Mask */

#define ADC_ADCHER_CHEN_Pos     0                                 /*!< ADC ADCHER: CHEN Position */
#define ADC_ADCHER_CHEN_Msk     (0xFFul << ADC_ADCHER_CHEN_Pos)   /*!< ADC ADCHER: CHEN Mask */

/* ADCMPR Bit Field Definitions */
#define ADC_ADCMPR_CMPD_Pos        16                                    /*!< ADC ADCMPR: CMPD Position */
#define ADC_ADCMPR_CMPD_Msk        (0xFFFul << ADC_ADCMPR_CMPD_Pos)      /*!< ADC ADCMPR: CMPD Mask */

#define ADC_ADCMPR_CMPMATCNT_Pos   8                                     /*!< ADC ADCMPR: CMPMATCNT Position */
#define ADC_ADCMPR_CMPMATCNT_Msk   (0xFul << ADC_ADCMPR_CMPMATCNT_Pos)   /*!< ADC ADCMPR: CMPMATCNT Mask */

#define ADC_ADCMPR_CMPCH_Pos       3                                     /*!< ADC ADCMPR: CMPCH Position */
#define ADC_ADCMPR_CMPCH_Msk       (7ul << ADC_ADCMPR_CMPCH_Pos)         /*!< ADC ADCMPR: CMPCH Mask */

#define ADC_ADCMPR_CMPCOND_Pos     2                                     /*!< ADC ADCMPR: CMPCOND Position */
#define ADC_ADCMPR_CMPCOND_Msk     (1ul << ADC_ADCMPR_CMPCOND_Pos)       /*!< ADC ADCMPR: CMPCOND Mask */

#define ADC_ADCMPR_CMPIE_Pos       1                                     /*!< ADC ADCMPR: CMPIE Position */
#define ADC_ADCMPR_CMPIE_Msk       (1ul << ADC_ADCMPR_CMPIE_Pos)         /*!< ADC ADCMPR: CMPIE Mask */

#define ADC_ADCMPR_CMPEN_Pos       0                                     /*!< ADC ADCMPR: CMPEN Position */
#define ADC_ADCMPR_CMPEN_Msk       (1ul << ADC_ADCMPR_CMPEN_Pos)         /*!< ADC ADCMPR: CMPEN Mask */

/* ADSR Bit Field Definitions */
#define ADC_ADSR_OVERRUN_Pos       16                                    /*!< ADC ADSR: OVERRUN Position */
#define ADC_ADSR_OVERRUN_Msk       (0xFFul << ADC_ADSR_OVERRUN_Pos)      /*!< ADC ADSR: OVERRUN Mask */

#define ADC_ADSR_VALID_Pos         8                                     /*!< ADC ADSR: VALID Position */
#define ADC_ADSR_VALID_Msk         (0xFFul << ADC_ADSR_VALID_Pos)        /*!< ADC ADSR: VALID Mask */

#define ADC_ADSR_CHANNEL_Pos       4                                     /*!< ADC ADSR: CHANNEL Position */
#define ADC_ADSR_CHANNEL_Msk       (7ul << ADC_ADSR_CHANNEL_Pos)         /*!< ADC ADSR: CHANNEL Mask */

#define ADC_ADSR_BUSY_Pos          3                                     /*!< ADC ADSR: BUSY Position */
#define ADC_ADSR_BUSY_Msk          (1ul << ADC_ADSR_BUSY_Pos)            /*!< ADC ADSR: BUSY Mask */

#define ADC_ADSR_CMPF1_Pos         2                                     /*!< ADC ADSR: CMPF1 Position */
#define ADC_ADSR_CMPF1_Msk         (1ul << ADC_ADSR_CMPF1_Pos)           /*!< ADC ADSR: CMPF1 Mask */

#define ADC_ADSR_CMPF0_Pos         1                                     /*!< ADC ADSR: CMPF0 Position */
#define ADC_ADSR_CMPF0_Msk         (1ul << ADC_ADSR_CMPF0_Pos)           /*!< ADC ADSR: CMPF0 Mask */

#define ADC_ADSR_ADF_Pos           0                                     /*!< ADC ADSR: ADF Position */
#define ADC_ADSR_ADF_Msk           (1ul << ADC_ADSR_ADF_Pos)             /*!< ADC ADSR: ADF Mask */

/* ADPDMA Bit Field Definitions */
#define ADC_ADPDMA_AD_PDMA_Pos     0                                     /*!< ADC ADPDMA: AD_PDMA Position */
#define ADC_ADPDMA_AD_PDMA_Msk     (0x3FFul << ADC_ADPDMA_AD_PDMA_Pos)   /*!< ADC ADPDMA: AD_PDMA Mask */
/*@}*/ /* end of group REG_ADC_BITMASK */
/*@}*/ /* end of group REG_ADC */


/*---------------------------- Clock Controller ------------------------------*/
/** @addtogroup REG_CLK System Clock Controller
  Memory Mapped Structure for System Clock Controller
  @{
 */
typedef struct
{

    /**
     * PWRCON
     * ===================================================================================================
     * Offset: 0x00  System Power-down Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |XTL12M_EN |External 4~24 MHz High Speed Crystal Enable (HXT) Control (Write Protect)
     * |        |          |The bit default value is set by flash controller user configuration register CONFIG0 [26:24].
     * |        |          |When the default clock source is from external 4~24 MHz high speed crystal, this bit is set to 1 automatically.
     * |        |          |0 = External 4~24 MHz high speed crystal oscillator (HXT) Disabled.
     * |        |          |1 = External 4~24 MHz high speed crystal oscillator (HXT) Enabled.
     * |        |          |Note: This bit is the protected bit, and programming it needs to write "59h", "16h", and "88h" to address 0x5000_0100 to disable register protection.
     * |        |          |Refer to the register REGWRPROT at address GCR_BA+0x100.
     * |[2]     |OSC22M_EN |Internal 22.1184 MHz High Speed Oscillator (HIRC) Enable Control (Write Protect)
     * |        |          |0 = Internal 22.1184 MHz high speed oscillator (HIRC) Disabled.
     * |        |          |1 = Internal 22.1184 MHz high speed oscillator (HIRC) Enabled.
     * |        |          |Note: This bit is the protected bit, and programming it needs to write "59h", "16h", and "88h" to address 0x5000_0100 to disable register protection.
     * |        |          |Refer to the register REGWRPROT at address GCR_BA+0x100.
     * |[3]     |OSC10K_EN |Internal 10 KHz Low Speed Oscillator (LIRC) Enable Control (Write Protect)
     * |        |          |0 = Internal 10 kHz low speed oscillator (LIRC) Disabled.
     * |        |          |1 = Internal 10 kHz low speed oscillator (LIRC) Enabled.
     * |        |          |Note: This bit is the protected bit, and programming it needs to write "59h", "16h", and "88h" to address 0x5000_0100 to disable register protection.
     * |        |          |Refer to the register REGWRPROT at address GCR_BA+0x100.
     * |[4]     |PD_WU_DLY |Wake-up Delay Counter Enable Control (Write Protect)
     * |        |          |When the chip wakes up from Power-down mode, the clock control will delay certain clock cycles to wait system clock stable.
     * |        |          |The delayed clock cycle is 4096 clock cycles when chip work at external 4~24 MHz high speed crystal, and 256 clock cycles when chip work at internal 22.1184 MHz high speed oscillator.
     * |        |          |0 = Clock cycles delay Disabled.
     * |        |          |1 = Clock cycles delay Enabled.
     * |        |          |Note: This bit is the protected bit, and programming it needs to write "59h", "16h", and "88h" to address 0x5000_0100 to disable register protection.
     * |        |          |Refer to the register REGWRPROT at address GCR_BA+0x100.
     * |[5]     |PD_WU_INT_EN|Power-Down Mode Wake-Up Interrupt Enable Control (Write Protect)
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Note1: The interrupt will occur when both PD_WU_STS and PD_WU_INT_EN are high.
     * |        |          |Note2: This bit is the protected bit, and programming it needs to write "59h", "16h", and "88h" to address 0x5000_0100 to disable register protection.
     * |        |          |Refer to the register REGWRPROT at address GCR_BA+0x100.
     * |[6]     |PD_WU_STS |Power-Down Mode Wake-Up Interrupt Status
     * |        |          |Set by "Power-down wake-up event", it indicates that resume from Power-down mode"
     * |        |          |The flag is set if the GPIO, USB, UART, WDT or BOD wakeup occurred.
     * |        |          |Write 1 to clear the bit to 0.
     * |        |          |Note: This bit is working only if PD_WU_INT_EN (PWRCON[5]) set to 1.
     * |[7]     |PWR_DOWN_EN|System Power-Down Enable Bit (Write Protect)
     * |        |          |When this bit is set to 1, Power-down mode is enabled and chip Power-down behavior will depends on the PD_WAIT_CPU bit
     * |        |          |(a) If the PD_WAIT_CPU is 0, then the chip enters Power-down mode immediately after the PWR_DOWN_EN bit set.
     * |        |          |(b) if the PD_WAIT_CPU is 1, then the chip keeps active till the CPU sleep mode is also active and then the chip enters Power-down mode (recommend)
     * |        |          |When chip wakes up from Power-down mode, this bit is cleared by hardware.
     * |        |          |User needs to set this bit again for next Power-down.
     * |        |          |In Power-down mode, external 4~24 MHz high speed crystal oscillator and the internal 22.1184 MHz high speed oscillator will be disabled in this mode, but the external 32.768 kHz low speed crystal and internal 10 kHz low speed oscillator are not controlled by Power-down mode.
     * |        |          |In Power- down mode, the PLL and system clock are disabled, and ignored the clock source selection.
     * |        |          |The clocks of peripheral are not controlled by Power-down mode, if the peripheral clock source is from external 32.768 kHz low speed crystal oscillator or the internal 10 kHz low speed oscillator.
     * |        |          |0 = Chip operating normally or chip in Idle mode because of WFI command.
     * |        |          |1 = Chip enters Power-down mode instantly or waits CPU sleep command WFI.
     * |        |          |Note: This bit is the protected bit, and programming it needs to write "59h", "16h", and "88h" to address 0x5000_0100 to disable register protection.
     * |        |          |Refer to the register REGWRPROT at address GCR_BA+0x100.
     * |[8]     |PD_WAIT_CPU|This Bit Control The Power-Down Entry Condition (Write Protect)
     * |        |          |0 = Chip enters Power-down mode when the PWR_DOWN_EN bit is set to 1.
     * |        |          |1 = Chip enters Power- down mode when the both PD_WAIT_CPU and PWR_DOWN_EN bits are set to 1 and CPU run WFI instruction.
     * |        |          |Note: This bit is the protected bit, and programming it needs to write "59h", "16h", and "88h" to address 0x5000_0100 to disable register protection.
     * |        |          |Refer to the register REGWRPROT at address GCR_BA+0x100.
     */
    __IO uint32_t PWRCON;

    /**
    * AHBCLK
    * ===================================================================================================
    * Offset: 0x04  AHB Devices Clock Enable Control Register
    * ---------------------------------------------------------------------------------------------------
    * |Bits    |Field     |Descriptions
    * | :----: | :----:   | :---- |
    * |[1]     |PDMA_EN   |PDMA Controller Clock Enable Control
    * |        |          |0 = PDMA peripheral clock Disabled.
    * |        |          |1 = PDMA peripheral clock Enabled.
    * |[2]     |ISP_EN    |Flash ISP Controller Clock Enable Control
    * |        |          |0 = Flash ISP peripheral clock Disabled.
    * |        |          |1 = Flash ISP peripheral clock Enabled.
    */
    __IO uint32_t AHBCLK;

    /**
     * APBCLK
     * ===================================================================================================
     * Offset: 0x08  APB Devices Clock Enable Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |WDT_EN    |Watchdog Timer Clock Enable Control (Write Protect)
     * |        |          |0 = Watchdog Timer clock Disabled.
     * |        |          |1 = Watchdog Timer clock Enabled.
     * |        |          |Note: This bit is the protected bit, and programming it needs to write "59h", "16h", and "88h" to address 0x5000_0100 to disable register protection.
     * |        |          |Refer to the register REGWRPROT at address GCR_BA+0x100.
     * |[2]     |TMR0_EN   |Timer0 Clock Enable Control
     * |        |          |0 = Timer0 clock Disabled.
     * |        |          |1 = Timer0 clock Enabled.
     * |[3]     |TMR1_EN   |Timer1 Clock Enable Control
     * |        |          |0 = Timer1 clock Disabled.
     * |        |          |1 = Timer1 clock Enabled.
     * |[4]     |TMR2_EN   |Timer2 Clock Enable Control
     * |        |          |0 = Timer2 clock Disabled.
     * |        |          |1 = Timer2 clock Enabled.
     * |[5]     |TMR3_EN   |Timer3 Clock Enable Control
     * |        |          |0 = Timer3 clock Disabled.
     * |        |          |1 = Timer3 clock Enabled.
     * |[6]     |FDIV_EN   |Frequency Divider Output Clock Enable Control
     * |        |          |0 = FDIV clock Disabled.
     * |        |          |1 = FDIV clock Enabled.
     * |[8]     |I2C0_EN   |I2C0 Clock Enable Control
     * |        |          |0 = I2C0 clock Disabled.
     * |        |          |1 = I2C0 clock Enabled.
     * |[9]     |I2C1_EN   |I2C1 Clock Enable Control
     * |        |          |0 = I2C1 clock Disabled.
     * |        |          |1 = I2C1 clock Enabled.
     * |[12]    |SPI0_EN   |SPI0 Clock Enable Control
     * |        |          |0 = SPI0 clock Disabled.
     * |        |          |1 = SPI0 clock Enabled.
     * |[13]    |SPI1_EN   |SPI1 Clock Enable Control
     * |        |          |0 = SPI1 clock Disabled.
     * |        |          |1 = SPI1 clock Enabled.
     * |[14]    |SPI2_EN   |SPI2 Clock Enable Control
     * |        |          |0 = SPI2 clock Disabled.
     * |        |          |1 = SPI2 clock Enabled.
     * |[16]    |UART0_EN  |UART0 Clock Enable Control
     * |        |          |0 = UART0 clock Disabled.
     * |        |          |1 = UART0 clock Enabled.
     * |[17]    |UART1_EN  |UART1 Clock Enable Control
     * |        |          |0 = UART1 clock Disabled.
     * |        |          |1 = UART1 clock Enabled.
     * |[20]    |PWM01_EN  |PWM_01 Clock Enable Control
     * |        |          |0 = PWM01 clock Disabled.
     * |        |          |1 = PWM01 clock Enabled.
     * |[21]    |PWM23_EN  |PWM_23 Clock Enable Control
     * |        |          |0 = PWM23 clock Disabled.
     * |        |          |1 = PWM23 clock Enabled.
     * |[27]    |USBD_EN   |USB 2.0 FS Device Controller Clock Enable Control
     * |        |          |0 = USB clock Disabled.
     * |        |          |1 = USB clock Enabled.
     * |[28]    |ADC_EN    |Analog-Digital-Converter (ADC) Clock Enable Control
     * |        |          |0 = ADC clock Disabled.
     * |        |          |1 = ADC clock Enabled.
     * |[29]    |I2S_EN    |I2S Clock Enable Control
     * |        |          |0 = I2S clock Disabled.
     * |        |          |1 = I2S clock Enabled.
     * |[31]    |PS2_EN    |PS/2 Clock Enable Control
     * |        |          |0 = PS/2 clock Disabled.
     * |        |          |1 = PS/2 clock Enabled.
     */
    __IO uint32_t APBCLK;

    /**
     * CLKSTATUS
     * ===================================================================================================
     * Offset: 0x0C  Clock status monitor Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |XTL12M_STB|External 4~24 MHz High Speed Crystal (HXT) Clock Source Stable Flag (Read Only)
     * |        |          |0 = External 4~24 MHz high speed crystal clock (HXT) is not stable or disabled.
     * |        |          |1 = External 4~24 MHz high speed crystal clock (HXT) is stable.
     * |[2]     |PLL_STB   |Internal PLL Clock Source Stable Flag (Read Only)
     * |        |          |0 = Internal PLL clock is not stable or disabled.
     * |        |          |1 = Internal PLL clock is stable.
     * |[3]     |OSC10K_STB|Internal 10 KHz Low Speed Oscillator (LIRC) Clock Source Stable Flag (Read Only)
     * |        |          |0 = Internal 10 kHz low speed oscillator clock (LIRC) is not stable or disabled.
     * |        |          |1 = Internal 10 kHz low speed oscillator clock (LIRC) is stable.
     * |[4]     |OSC22M_STB|Internal 22.1184 MHz High Speed Oscillator (HIRC) Clock Source Stable Flag (Read Only)
     * |        |          |0 = Internal 22.1184 MHz high speed oscillator (HIRC) clock is not stable or disabled.
     * |        |          |1 = Internal 22.1184 MHz high speed oscillator (HIRC) clock is stable.
     * |[7]     |CLK_SW_FAIL|Clock Switching Fail Flag (Read Only)
     * |        |          |0 = Clock switching success.
     * |        |          |1 = Clock switching failure.
     * |        |          |This bit is an index that if current system clock source is match as user defined at HCLK_S (CLKSEL[2:0]).
     * |        |          |When user switch system clock, the system clock source will keep old clock until the new clock is stable.
     * |        |          |During the period that waiting new clock stable, this bit will be an index shows system clock source is not match as user wanted.
     */
    __IO uint32_t CLKSTATUS;

    /**
     * CLKSEL0
     * ===================================================================================================
     * Offset: 0x10  Clock Source Select Control Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |HCLK_S    |HCLK Clock Source Select (Write Protect)
     * |        |          |1. Before clock switching, the related clock sources (both pre-select and new-select) must be turn on
     * |        |          |2.
     * |        |          |The 3-bit default value is reloaded from the value of CFOSC (CONFIG0[26:24]) in user configuration register of Flash controller by any reset.
     * |        |          |Therefore the default value is either 000b or 111b.
     * |        |          |3.
     * |        |          |These bits are protected bit, It means programming this bit needs to write "59h", "16h", "88h" to address 0x5000_0100 to disable register protection.
     * |        |          |Refer to the register REGWRPROT at address GCR_BA+0x100.
     * |        |          |000 = Clock source from external 4~24 MHz high speed crystal oscillator clock.
     * |        |          |010 = Clock source from PLL/2 clock.
     * |        |          |010 = Clock source from PLL clock.
     * |        |          |011 = Clock source from internal 10 kHz low speed oscillator clock.
     * |        |          |111 = Clock source from internal 22.1184 MHz high speed oscillator clock.
     * |        |          |Note: This bit is the protected bit, and programming it needs to write "59h", "16h", and "88h" to address 0x5000_0100 to disable register protection.
     * |        |          |Refer to the register REGWRPROT at address GCR_BA+0x100.
     * |[5:3]   |STCLK_S   |Cortex-M0 SysTick Clock Source Select (Write Protect)
     * |        |          |If SYST_CSR[2] = 1, SysTick clock source is from HCLK.
     * |        |          |If SYST_CSR[2] = 0, SysTick clock source is defined by STCLK_S(CLKSEL0[5:3]).
     * |        |          |000 = Clock source from external 4~24 MHz high speed crystal clock.
     * |        |          |010 = Clock source from external 4~24 MHz high speed crystal clock/2.
     * |        |          |011 = Clock source from HCLK/2.
     * |        |          |111 = Clock source from internal 22.1184 MHz high speed oscillator clock/2.
     * |        |          |Note1: These bits are protected bit.
     * |        |          |It means programming this bit needs to write "59h", "16h", "88h" to address 0x5000_0100 to disable register protection.
     * |        |          |Refer to the register REGWRPROT at address GCR_BA+0x100.
     * |        |          |Note2: if SysTick clock source is not from HCLK (i.e.
     * |        |          |SYST_CSR[2] = 0), SysTick clock source must less than or equal to HCLK/2.
     */
    __IO uint32_t CLKSEL0;

    /**
     * CLKSEL1
     * ===================================================================================================
     * Offset: 0x14  Clock Source Select Control Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1:0]   |WDT_S     |Watchdog Timer Clock Source Select (Write Protect)
     * |        |          |10 = Clock source from HCLK/2048 clock.
     * |        |          |11 = Clock source from internal 10 kHz low speed oscillator clock.
     * |        |          |Note: This bit is the protected bit, and programming it needs to write "59h", "16h", and "88h" to address 0x5000_0100 to disable register protection.
     * |        |          |Refer to the register REGWRPROT at address GCR_BA+0x100.
     * |[3:2]   |ADC_S     |ADC Clock Source Select
     * |        |          |00 = Clock source from external 4~24 MHz high speed crystal oscillator clock.
     * |        |          |01 = Clock source from PLL clock.
     * |        |          |10 = Clock source from HCLK.
     * |        |          |11 = Clock source from internal 22.1184 MHz high speed oscillator clock.
     * |[4]     |SPI0_S    |SPI0 Clock Source Selection
     * |        |          |0 = Clock source from PLL clock.
     * |        |          |1 = Clock source from HCLK.
     * |[5]     |SPI1_S    |SPI1 Clock Source Selection
     * |        |          |0 = Clock source from PLL clock.
     * |        |          |1 = Clock source from HCLK.
     * |[6]     |SPI2_S    |SPI2 Clock Source Selection
     * |        |          |0 = Clock source from PLL clock.
     * |        |          |1 = Clock source from HCLK.
     * |[10:8]  |TMR0_S    |TIMER0 Clock Source Selection
     * |        |          |000 = Clock source from external 4~24 MHz high speed crystal clock.
     * |        |          |010 = Clock source from HCLK.
     * |        |          |011 = Clock source from external trigger.
     * |        |          |101 = Clock source from internal 10 kHz low speed oscillator clock.
     * |        |          |111 = Clock source from internal 22.1184 MHz high speed oscillator clock.
     * |        |          |Others = reserved.
     * |[14:12] |TMR1_S    |TIMER1 Clock Source Selection
     * |        |          |000 = Clock source from external 4~24 MHz high speed crystal clock.
     * |        |          |010 = Clock source from HCLK.
     * |        |          |011 = Clock source from external trigger.
     * |        |          |101 = Clock source from internal 10 kHz low speed oscillator clock.
     * |        |          |111 = Clock source from internal 22.1184 MHz high speed oscillator clock.
     * |        |          |Others = reserved.
     * |[18:16] |TMR2_S    |TIMER2 Clock Source Selection
     * |        |          |000 = Clock source from external 4~24 MHz high speed crystal clock.
     * |        |          |010 = Clock source from HCLK.
     * |        |          |011 = Clock source from external trigger.
     * |        |          |101 = Clock source from internal 10 kHz low speed oscillator clock.
     * |        |          |111 = Clock source from internal 22.1184 MHz high speed oscillator clock.
     * |        |          |Others = reserved.
     * |[22:20] |TMR3_S    |TIMER3 Clock Source Selection
     * |        |          |000 = Clock source from external 4~24 MHz high speed crystal clock.
     * |        |          |010 = Clock source from HCLK.
     * |        |          |011 = Clock source from external trigger.
     * |        |          |101 = Clock source from internal 10 kHz low speed oscillator clock.
     * |        |          |111 = Clock source from internal 22.1184 MHz high speed oscillator clock.
     * |        |          |Others = reserved.
     * |[25:24] |UART_S    |UART Clock Source Selection
     * |        |          |00 = Clock source from external 4~24 MHz high speed crystal oscillator clock.
     * |        |          |01 = Clock source from PLL clock.
     * |        |          |11 = Clock source from internal 22.1184 MHz high speed oscillator clock.
     * |[29:28] |PWM01_S   |PWM0 And PWM1 Clock Source Selection
     * |        |          |PWM0 and PWM1 used the same peripheral clock source; both of them used the same prescaler.
     * |        |          |The Engine clock source of PWM0 and PWM1 is defined by PWM01_S (CLKSEL1[29:28]) and PWM01_S_E (CLKSEL2[8]).
     * |        |          |If PWM01_S_E = 0, the peripheral clock source of PWM0 and PWM1 defined by PWM01_S list below:
     * |        |          |00 = Clock source from external 4~24 MHz high speed crystal oscillator clock.
     * |        |          |10 = Clock source from HCLK.
     * |        |          |11 = Clock source from internal 22.1184 MHz high speed oscillator clock.
     * |        |          |If PWM01_S_E = 1, the engine clock source of PWM0 and PWM1 defined by PWM01_S list below:
     * |        |          |00 = Reserved.
     * |        |          |01 = Reserved.
     * |        |          |10 = Reserved.
     * |        |          |11 = Clock source from internal 10 kHz low speed oscillator clock.
     * |[31:30] |PWM23_S   |PWM2 And PWM3 Clock Source Selection
     * |        |          |PWM2 and PWM3 used the same peripheral clock source; both of them used the same prescaler.
     * |        |          |The Engine clock source of PWM2 and PWM3 is defined by PWM23_S (CLKSEL1[31:30]) and PWM23_S_E (CLKSEL2[9]).
     * |        |          |If PWM23_S_E = 0, the peripheral clock source of PWM2 and PWM3 defined by PWM23_S list below:
     * |        |          |00 = Clock source from external 4~24 MHz high speed crystal oscillator clock.
     * |        |          |10 = Clock source from HCLK.
     * |        |          |11 = Clock source from internal 22.1184 MHz high speed oscillator clock.
     * |        |          |If PWM23_S_E = 1, the engine clock source of PWM2 and PWM3 defined by PWM23_S list below:
     * |        |          |00 = Reserved.
     * |        |          |01 = Reserved.
     * |        |          |10 = Reserved.
     * |        |          |11 = Clock source from internal 10 kHz low speed oscillator clock.
     */
    __IO uint32_t CLKSEL1;

    /**
     * CLKDIV
     * ===================================================================================================
     * Offset: 0x18  Clock Divider Number Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |HCLK_N    |HCLK Clock Divide Number From HCLK Clock Source
     * |        |          |HCLK clock frequency = (HCLK clock source frequency) / (HCLK_N + 1).
     * |[7:4]   |USB_N     |USB Clock Divide Number From PLL Clock
     * |        |          |USB clock frequency = (PLL frequency) / (USB_N + 1).
     * |[11:8]  |UART_N    |UART Clock Divide Number From UART Clock Source
     * |        |          |UART clock frequency = (UART clock source frequency) / (UART_N + 1).
     * |[23:16] |ADC_N     |ADC Clock Divide Number From ADC Clock Source
     * |        |          |ADC clock frequency = (ADC clock source frequency) / (ADC_N + 1).
     */
    __IO uint32_t CLKDIV;

    /**
     * CLKSEL2
     * ===================================================================================================
     * Offset: 0x1C  Clock Source Select Control Register 2
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1:0]   |I2S_S     |I2S Clock Source Selection
     * |        |          |00 = Clock source from external 4~24 MHz high speed crystal oscillator clock.
     * |        |          |01 = Clock source from PLL clock.
     * |        |          |10 = Clock source from HCLK.
     * |        |          |11 = Clock source from internal 22.1184 MHz high speed oscillator clock.
     * |[3:2]   |FRQDIV_S  |Clock Divider Clock Source Selection
     * |        |          |00 = Clock source from external 4~24 MHz high speed crystal oscillator clock.
     * |        |          |01 = Clock source from external 32.768 kHz low speed crystal oscillator clock.
     * |        |          |10 = Clock source from HCLK.
     * |        |          |11 = Clock source from internal 22.1184 MHz high speed oscillator clock.
     * |[8]     |PWM01_S_E |PWM0 And PWM1 Clock Source Selection Extend
     * |        |          |PWM0 and PWM1 used the same peripheral clock source; both of them used the same prescaler.
     * |        |          |The peripheral clock source of PWM0 and PWM1 is defined by PWM01_S (CLKSEL1[29:28]) and PWM01_S_E (CLKSEL2[8]).
     * |        |          |If PWM01_S_E = 0, the peripheral clock source of PWM0 and PWM1 defined by PWM01_S list below:
     * |        |          |00 = Clock source from external 4~24 MHz high speed crystal oscillator clock.
     * |        |          |10 = Clock source from HCLK.
     * |        |          |11 = Clock source from internal 22.1184 MHz high speed oscillator clock.
     * |        |          |If PWM01_S_E = 1, the peripheral clock source of PWM0 and PWM1 defined by PWM01_S list below:
     * |        |          |00 = Reserved.
     * |        |          |01 = Reserved.
     * |        |          |10 = Reserved.
     * |        |          |11 = Clock source from internal 10 kHz low speed oscillator clock.
     * |[9]     |PWM23_S_E |PWM2 And PWM3 Clock Source Selection Extend
     * |        |          |PWM2 and PWM3 used the same Engine clock source; both of them used the same prescaler.
     * |        |          |The Engine clock source of PWM2 and PWM3 is defined by PWM23_S (CLKSEL1[31:30]) and PWM23_S_E (CLKSEL2[9]).
     * |        |          |If PWM23_S_E = 0, the peripheral clock source of PWM2 and PWM3 defined by PWM23_S list below:
     * |        |          |00 = Clock source from external 4~24 MHz high speed crystal oscillator clock.
     * |        |          |10 = Clock source from HCLK.
     * |        |          |11 = Clock source from internal 22.1184 MHz high speed oscillator clock.
     * |        |          |If PWM23_S_E = 1, the peripheral clock source of PWM2 and PWM3 defined by PWM23_S list below:
     * |        |          |00 = Reserved.
     * |        |          |01 = Reserved.
     * |        |          |10 = Reserved.
     * |        |          |11 = Clock source from internal 10 kHz low speed oscillator clock.
     * |[17:16] |WWDT_S    |Window Watchdog Timer Clock Source Selection
     * |        |          |10 = Clock source from HCLK/2048 clock.
     * |        |          |11 = Clock source from internal 10 kHz low speed oscillator clock.
     */
    __IO uint32_t CLKSEL2;

    /**
     * PLLCON
     * ===================================================================================================
     * Offset: 0x20  PLL Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[8:0]   |FB_DV     |PLL Feedback Divider Control Bits
     * |        |          |Refer to the formulas below the table.
     * |[13:9]  |IN_DV     |PLL Input Divider Control Bits
     * |        |          |Refer to the formulas below the table.
     * |[15:14] |OUT_DV    |PLL Output Divider Control Bits
     * |        |          |Refer to the formulas below the table.
     * |[16]    |PD        |Power-Down Mode
     * |        |          |If the PWR_DOWN_EN bit is set to 1 in PWRCON register, the PLL will enter Power-down mode too.
     * |        |          |0 = PLL is in Normal mode.
     * |        |          |1 = PLL is in Power-down mode (default).
     * |[17]    |BP        |PLL Bypass Control
     * |        |          |0 = PLL is in Normal mode (default).
     * |        |          |1 = PLL clock output is same as PLL source clock input.
     * |[18]    |OE        |PLL OE (FOUT Enable) Pin Control
     * |        |          |0 = PLL FOUT Enabled.
     * |        |          |1 = PLL FOUT is fixed low.
     * |[19]    |PLL_SRC   |PLL Source Clock Selection
     * |        |          |0 = PLL source clock from external 4~24 MHz high speed crystal.
     * |        |          |1 = PLL source clock from internal 22.1184 MHz high speed oscillator.
     */
    __IO uint32_t PLLCON;

    /**
     * FRQDIV
     * ===================================================================================================
     * Offset: 0x24  Frequency Divider Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |FSEL      |Divider Output Frequency Selection Bits
     * |        |          |The formula of output frequency is
     * |        |          |Fout = Fin/2(N+1).
     * |        |          |Fin is the input clock frequency.
     * |        |          |Fout is the frequency of divider output clock.
     * |        |          |N is the 4-bit value of FSEL[3:0].
     * |[4]     |DIVIDER_EN|Frequency Divider Enable Bit
     * |        |          |0 = Frequency Divider function Disabled.
     * |        |          |1 = Frequency Divider function Enabled.
     */
    __IO uint32_t FRQDIV;

    /**
     * @cond HIDDEN_SYMBOLS
     */
    uint32_t RESERVE0;
    /**
     * @endcond
     */

    /**
     * APBDIV
     * ===================================================================================================
     * Offset: 0x2C  APB Divider Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |APBDIV    |APB Divider Enable Bit
     * |        |          |0 = PCLK is HCLK.
     * |        |          |1 = PCLK is HCLK / 2.
     */
    __IO uint32_t APBDIV;

} CLK_T;

/** @addtogroup REG_CLK_BITMASK CLK Bit Mask
  @{
 */

/* CLK PWRCON Bit Field Definitions */
#define CLK_PWRCON_PD_WAIT_CPU_Pos           8                                    /*!< CLK PWRCON: PD_WAIT_CPU Position */
#define CLK_PWRCON_PD_WAIT_CPU_Msk           (1ul << CLK_PWRCON_PD_WAIT_CPU_Pos)  /*!< CLK PWRCON: PD_WAIT_CPU Mask */

#define CLK_PWRCON_PWR_DOWN_EN_Pos           7                                    /*!< CLK PWRCON: PWR_DOWN_EN Position */
#define CLK_PWRCON_PWR_DOWN_EN_Msk           (1ul << CLK_PWRCON_PWR_DOWN_EN_Pos)  /*!< CLK PWRCON: PWR_DOWN_EN Mask */

#define CLK_PWRCON_PD_WU_STS_Pos             6                                    /*!< CLK PWRCON: PD_WU_STS Position */
#define CLK_PWRCON_PD_WU_STS_Msk             (1ul << CLK_PWRCON_PD_WU_STS_Pos)    /*!< CLK PWRCON: PD_WU_STS Mask */

#define CLK_PWRCON_PD_WU_INT_EN_Pos          5                                    /*!< CLK PWRCON: PD_WU_INT_EN Position */
#define CLK_PWRCON_PD_WU_INT_EN_Msk          (1ul << CLK_PWRCON_PD_WU_INT_EN_Pos) /*!< CLK PWRCON: PD_WU_INT_EN Mask */

#define CLK_PWRCON_PD_WU_DLY_Pos             4                                    /*!< CLK PWRCON: PD_WU_DLY Position */
#define CLK_PWRCON_PD_WU_DLY_Msk             (1ul << CLK_PWRCON_PD_WU_DLY_Pos)    /*!< CLK PWRCON: PD_WU_DLY Mask */

#define CLK_PWRCON_OSC10K_EN_Pos             3                                    /*!< CLK PWRCON: OSC10K_EN Position */
#define CLK_PWRCON_OSC10K_EN_Msk             (1ul << CLK_PWRCON_OSC10K_EN_Pos)    /*!< CLK PWRCON: OSC10K_EN Mask */
#define CLK_PWRCON_IRC10K_EN_Pos             3                                    /*!< CLK PWRCON: IRC10K_EN Position */
#define CLK_PWRCON_IRC10K_EN_Msk             (1ul << CLK_PWRCON_IRC10K_EN_Pos)    /*!< CLK PWRCON: IRC10K_EN Mask */

#define CLK_PWRCON_OSC22M_EN_Pos             2                                    /*!< CLK PWRCON: OSC22M_EN Position */
#define CLK_PWRCON_OSC22M_EN_Msk             (1ul << CLK_PWRCON_OSC22M_EN_Pos)    /*!< CLK PWRCON: OSC22M_EN Mask */
#define CLK_PWRCON_IRC22M_EN_Pos             2                                    /*!< CLK PWRCON: IRC22M_EN Position */
#define CLK_PWRCON_IRC22M_EN_Msk             (1ul << CLK_PWRCON_IRC22M_EN_Pos)    /*!< CLK PWRCON: IRC22M_EN Mask */

#define CLK_PWRCON_XTL12M_EN_Pos             0                                    /*!< CLK PWRCON: XTL12M_EN Position */
#define CLK_PWRCON_XTL12M_EN_Msk             (1ul << CLK_PWRCON_XTL12M_EN_Pos)    /*!< CLK PWRCON: XTL12M_EN Mask */

/* CLK AHBCLK Bit Field Definitions */
#define CLK_AHBCLK_ISP_EN_Pos                2                                    /*!< CLK AHBCLK: ISP_EN Position */
#define CLK_AHBCLK_ISP_EN_Msk                (1ul << CLK_AHBCLK_ISP_EN_Pos)       /*!< CLK AHBCLK: ISP_EN Mask */

#define CLK_AHBCLK_PDMA_EN_Pos               1                                    /*!< CLK AHBCLK: PDMA_EN Position */
#define CLK_AHBCLK_PDMA_EN_Msk               (1ul << CLK_AHBCLK_PDMA_EN_Pos)      /*!< CLK AHBCLK: PDMA_EN Mask */


/* CLK APBCLK Bit Field Definitions */
#define CLK_APBCLK_PS2_EN_Pos                31                                   /*!< CLK APBCLK: PS2_EN Position */
#define CLK_APBCLK_PS2_EN_Msk                (1ul << CLK_APBCLK_PS2_EN_Pos)       /*!< CLK APBCLK: PS2_EN Mask */

#define CLK_APBCLK_I2S_EN_Pos                29                                   /*!< CLK APBCLK: I2S_EN Position */
#define CLK_APBCLK_I2S_EN_Msk                (1ul << CLK_APBCLK_I2S_EN_Pos)       /*!< CLK APBCLK: I2S_EN Mask */

#define CLK_APBCLK_ADC_EN_Pos                28                                   /*!< CLK APBCLK: ADC_EN Position */
#define CLK_APBCLK_ADC_EN_Msk                (1ul << CLK_APBCLK_ADC_EN_Pos)       /*!< CLK APBCLK: ADC_EN Mask */

#define CLK_APBCLK_USBD_EN_Pos               27                                   /*!< CLK APBCLK: USBD_EN Position */
#define CLK_APBCLK_USBD_EN_Msk               (1ul << CLK_APBCLK_USBD_EN_Pos)      /*!< CLK APBCLK: USBD_EN Mask */

#define CLK_APBCLK_PWM23_EN_Pos              21                                   /*!< CLK APBCLK: PWM23_EN Position */
#define CLK_APBCLK_PWM23_EN_Msk              (1ul << CLK_APBCLK_PWM23_EN_Pos)     /*!< CLK APBCLK: PWM23_EN Mask */

#define CLK_APBCLK_PWM01_EN_Pos              20                                   /*!< CLK APBCLK: PWM01_EN Position */
#define CLK_APBCLK_PWM01_EN_Msk              (1ul << CLK_APBCLK_PWM01_EN_Pos)     /*!< CLK APBCLK: PWM01_EN Mask */

#define CLK_APBCLK_UART1_EN_Pos              17                                   /*!< CLK APBCLK: UART1_EN Position */
#define CLK_APBCLK_UART1_EN_Msk              (1ul << CLK_APBCLK_UART1_EN_Pos)     /*!< CLK APBCLK: UART1_EN Mask */

#define CLK_APBCLK_UART0_EN_Pos              16                                   /*!< CLK APBCLK: UART0_EN Position */
#define CLK_APBCLK_UART0_EN_Msk              (1ul << CLK_APBCLK_UART0_EN_Pos)     /*!< CLK APBCLK: UART0_EN Mask */

#define CLK_APBCLK_SPI2_EN_Pos               14                                   /*!< CLK APBCLK: SPI2_EN Position */
#define CLK_APBCLK_SPI2_EN_Msk               (1ul << CLK_APBCLK_SPI2_EN_Pos)      /*!< CLK APBCLK: SPI2_EN Mask */

#define CLK_APBCLK_SPI1_EN_Pos               13                                   /*!< CLK APBCLK: SPI1_EN Position */
#define CLK_APBCLK_SPI1_EN_Msk               (1ul << CLK_APBCLK_SPI1_EN_Pos)      /*!< CLK APBCLK: SPI1_EN Mask */

#define CLK_APBCLK_SPI0_EN_Pos               12                                   /*!< CLK APBCLK: SPI0_EN Position */
#define CLK_APBCLK_SPI0_EN_Msk               (1ul << CLK_APBCLK_SPI0_EN_Pos)      /*!< CLK APBCLK: SPI0_EN Mask */

#define CLK_APBCLK_I2C1_EN_Pos               9                                    /*!< CLK APBCLK: I2C1_EN Position */
#define CLK_APBCLK_I2C1_EN_Msk               (1ul << CLK_APBCLK_I2C1_EN_Pos)      /*!< CLK APBCLK: I2C1_EN Mask */

#define CLK_APBCLK_I2C0_EN_Pos               8                                    /*!< CLK APBCLK: I2C0_EN_ Position */
#define CLK_APBCLK_I2C0_EN_Msk               (1ul << CLK_APBCLK_I2C0_EN_Pos)      /*!< CLK APBCLK: I2C0_EN_ Mask */

#define CLK_APBCLK_FDIV_EN_Pos               6                                    /*!< CLK APBCLK: FDIV_EN Position */
#define CLK_APBCLK_FDIV_EN_Msk               (1ul << CLK_APBCLK_FDIV_EN_Pos)      /*!< CLK APBCLK: FDIV_EN Mask */

#define CLK_APBCLK_TMR3_EN_Pos               5                                    /*!< CLK APBCLK: TMR3_EN Position */
#define CLK_APBCLK_TMR3_EN_Msk               (1ul << CLK_APBCLK_TMR3_EN_Pos)      /*!< CLK APBCLK: TMR3_EN Mask */

#define CLK_APBCLK_TMR2_EN_Pos               4                                    /*!< CLK APBCLK: TMR2_EN Position */
#define CLK_APBCLK_TMR2_EN_Msk               (1ul << CLK_APBCLK_TMR2_EN_Pos)      /*!< CLK APBCLK: TMR2_EN Mask */

#define CLK_APBCLK_TMR1_EN_Pos               3                                    /*!< CLK APBCLK: TMR1_EN Position */
#define CLK_APBCLK_TMR1_EN_Msk               (1ul << CLK_APBCLK_TMR1_EN_Pos)      /*!< CLK APBCLK: TMR1_EN Mask */

#define CLK_APBCLK_TMR0_EN_Pos               2                                    /*!< CLK APBCLK: TMR0_EN Position */
#define CLK_APBCLK_TMR0_EN_Msk               (1ul << CLK_APBCLK_TMR0_EN_Pos)      /*!< CLK APBCLK: TMR0_EN Mask */

#define CLK_APBCLK_WDT_EN_Pos                0                                    /*!< CLK APBCLK: WDT_EN Position */
#define CLK_APBCLK_WDT_EN_Msk                (1ul << CLK_APBCLK_WDT_EN_Pos)       /*!< CLK APBCLK: WDT_EN Mask */


/* CLK CLKSTATUS Bit Field Definitions */
#define CLK_CLKSTATUS_CLK_SW_FAIL_Pos        7                                        /*!< CLK CLKSTATUS: CLK_SW_FAIL Position */
#define CLK_CLKSTATUS_CLK_SW_FAIL_Msk        (1ul << CLK_CLKSTATUS_CLK_SW_FAIL_Pos)   /*!< CLK CLKSTATUS: CLK_SW_FAIL Mask */

#define CLK_CLKSTATUS_OSC22M_STB_Pos         4                                        /*!< CLK CLKSTATUS: OSC22M_STB Position */
#define CLK_CLKSTATUS_OSC22M_STB_Msk         (1ul << CLK_CLKSTATUS_OSC22M_STB_Pos)    /*!< CLK CLKSTATUS: OSC22M_STB Mask */
#define CLK_CLKSTATUS_IRC22M_STB_Pos         4                                        /*!< CLK CLKSTATUS: IRC22M_STB Position */
#define CLK_CLKSTATUS_IRC22M_STB_Msk         (1ul << CLK_CLKSTATUS_IRC22M_STB_Pos)    /*!< CLK CLKSTATUS: IRC22M_STB Mask */

#define CLK_CLKSTATUS_OSC10K_STB_Pos         3                                        /*!< CLK CLKSTATUS: OSC10K_STB Position */
#define CLK_CLKSTATUS_OSC10K_STB_Msk         (1ul << CLK_CLKSTATUS_OSC10K_STB_Pos)    /*!< CLK CLKSTATUS: OSC10K_STB Mask */
#define CLK_CLKSTATUS_IRC10K_STB_Pos         3                                        /*!< CLK CLKSTATUS: IRC10K_STB Position */
#define CLK_CLKSTATUS_IRC10K_STB_Msk         (1ul << CLK_CLKSTATUS_IRC10K_STB_Pos)    /*!< CLK CLKSTATUS: IRC10K_STB Mask */

#define CLK_CLKSTATUS_PLL_STB_Pos            2                                        /*!< CLK CLKSTATUS: PLL_STB Position */
#define CLK_CLKSTATUS_PLL_STB_Msk            (1ul << CLK_CLKSTATUS_PLL_STB_Pos)       /*!< CLK CLKSTATUS: PLL_STB Mask */

#define CLK_CLKSTATUS_XTL12M_STB_Pos         0                                        /*!< CLK CLKSTATUS: XTL12M_STB Position */
#define CLK_CLKSTATUS_XTL12M_STB_Msk         (1ul << CLK_CLKSTATUS_XTL12M_STB_Pos)    /*!< CLK CLKSTATUS: XTL12M_STB Mask */

/* CLK CLKSEL0 Bit Field Definitions */
#define CLK_CLKSEL0_STCLK_S_Pos              3                                        /*!< CLK CLKSEL0: STCLK_S Position */
#define CLK_CLKSEL0_STCLK_S_Msk              (7ul << CLK_CLKSEL0_STCLK_S_Pos)         /*!< CLK CLKSEL0: STCLK_S Mask */

#define CLK_CLKSEL0_HCLK_S_Pos               0                                        /*!< CLK CLKSEL0: HCLK_S Position */
#define CLK_CLKSEL0_HCLK_S_Msk               (7ul << CLK_CLKSEL0_HCLK_S_Pos)          /*!< CLK CLKSEL0: HCLK_S Mask */

/* CLK CLKSEL1 Bit Field Definitions */
#define CLK_CLKSEL1_PWM23_S_Pos              30                                       /*!< CLK CLKSEL1: PWM23_S Position */
#define CLK_CLKSEL1_PWM23_S_Msk              (3ul << CLK_CLKSEL1_PWM23_S_Pos)         /*!< CLK CLKSEL1: PWM23_S Mask */

#define CLK_CLKSEL1_PWM01_S_Pos              28                                       /*!< CLK CLKSEL1: PWM01_S Position */
#define CLK_CLKSEL1_PWM01_S_Msk              (3ul << CLK_CLKSEL1_PWM01_S_Pos)         /*!< CLK CLKSEL1: PWM01_S Mask */

#define CLK_CLKSEL1_UART_S_Pos               24                                       /*!< CLK CLKSEL1: UART_S Position */
#define CLK_CLKSEL1_UART_S_Msk               (3ul << CLK_CLKSEL1_UART_S_Pos)          /*!< CLK CLKSEL1: UART_S Mask */

#define CLK_CLKSEL1_TMR3_S_Pos               20                                       /*!< CLK CLKSEL1: TMR3_S Position */
#define CLK_CLKSEL1_TMR3_S_Msk               (7ul << CLK_CLKSEL1_TMR3_S_Pos)          /*!< CLK CLKSEL1: TMR3_S Mask */

#define CLK_CLKSEL1_TMR2_S_Pos               16                                       /*!< CLK CLKSEL1: TMR2_S Position */
#define CLK_CLKSEL1_TMR2_S_Msk               (7ul << CLK_CLKSEL1_TMR2_S_Pos)          /*!< CLK CLKSEL1: TMR2_S Mask */

#define CLK_CLKSEL1_TMR1_S_Pos               12                                       /*!< CLK CLKSEL1: TMR1_S Position */
#define CLK_CLKSEL1_TMR1_S_Msk               (7ul << CLK_CLKSEL1_TMR1_S_Pos)          /*!< CLK CLKSEL1: TMR1_S Mask */

#define CLK_CLKSEL1_TMR0_S_Pos               8                                        /*!< CLK CLKSEL1: TMR0_S Position */
#define CLK_CLKSEL1_TMR0_S_Msk               (7ul << CLK_CLKSEL1_TMR0_S_Pos)          /*!< CLK CLKSEL1: TMR0_S Mask */

#define CLK_CLKSEL1_SPI2_S_Pos               6                                        /*!< CLK CLKSEL1: SPI2_S Position */
#define CLK_CLKSEL1_SPI2_S_Msk               (1ul << CLK_CLKSEL1_SPI2_S_Pos)          /*!< CLK CLKSEL1: SPI2_S Mask */

#define CLK_CLKSEL1_SPI1_S_Pos               5                                        /*!< CLK CLKSEL1: SPI1_S Position */
#define CLK_CLKSEL1_SPI1_S_Msk               (1ul << CLK_CLKSEL1_SPI1_S_Pos)          /*!< CLK CLKSEL1: SPI1_S Mask */

#define CLK_CLKSEL1_SPI0_S_Pos               4                                        /*!< CLK CLKSEL1: SPI0_S Position */
#define CLK_CLKSEL1_SPI0_S_Msk               (1ul << CLK_CLKSEL1_SPI0_S_Pos)          /*!< CLK CLKSEL1: SPI0_S Mask */

#define CLK_CLKSEL1_ADC_S_Pos                2                                        /*!< CLK CLKSEL1: ADC_S Position */
#define CLK_CLKSEL1_ADC_S_Msk                (3ul << CLK_CLKSEL1_ADC_S_Pos)           /*!< CLK CLKSEL1: ADC_S Mask */

#define CLK_CLKSEL1_WDT_S_Pos                0                                        /*!< CLK CLKSEL1: WDT_S Position */
#define CLK_CLKSEL1_WDT_S_Msk                (3ul << CLK_CLKSEL1_WDT_S_Pos)           /*!< CLK CLKSEL1: WDT_S Mask */

/* CLK CLKSEL2 Bit Field Definitions */
#define CLK_CLKSEL2_WWDT_S_Pos               16                                       /*!< CLK CLKSEL2: WWDT_S Position */
#define CLK_CLKSEL2_WWDT_S_Msk               (3ul << CLK_CLKSEL2_WWDT_S_Pos)          /*!< CLK CLKSEL2: WWDT_S Mask */

#define CLK_CLKSEL2_PWM23_S_E_Pos            9                                        /*!< CLK CLKSEL2: PWM23_S_E Position */
#define CLK_CLKSEL2_PWM23_S_E_Msk            (1ul << CLK_CLKSEL2_PWM23_S_E_Pos)       /*!< CLK CLKSEL2: PWM23_S_E Mask */
#define CLK_CLKSEL2_PWM23_S_EXT_Pos          9                                        /*!< CLK CLKSEL2: PWM23_S_EXT Position */
#define CLK_CLKSEL2_PWM23_S_EXT_Msk          (1ul << CLK_CLKSEL2_PWM23_S_EXT_Pos)     /*!< CLK CLKSEL2: PWM23_S_EXT Mask */

#define CLK_CLKSEL2_PWM01_S_E_Pos            8                                        /*!< CLK CLKSEL2: PWM01_S_E Position */
#define CLK_CLKSEL2_PWM01_S_E_Msk            (1ul << CLK_CLKSEL2_PWM01_S_E_Pos)       /*!< CLK CLKSEL2: PWM01_S_E Mask */
#define CLK_CLKSEL2_PWM01_S_EXT_Pos          8                                        /*!< CLK CLKSEL2: PWM01_S_EXT Position */
#define CLK_CLKSEL2_PWM01_S_EXT_Msk          (1ul << CLK_CLKSEL2_PWM01_S_EXT_Pos)     /*!< CLK CLKSEL2: PWM01_S_EXT Mask */

#define CLK_CLKSEL2_FRQDIV_S_Pos             2                                        /*!< CLK CLKSEL2: FRQDIV_S Position */
#define CLK_CLKSEL2_FRQDIV_S_Msk             (3ul << CLK_CLKSEL2_FRQDIV_S_Pos)        /*!< CLK CLKSEL2: FRQDIV_S Mask */

#define CLK_CLKSEL2_I2S_S_Pos                0                                        /*!< CLK CLKSEL2: I2S_S Position */
#define CLK_CLKSEL2_I2S_S_Msk                (3ul << CLK_CLKSEL2_I2S_S_Pos)           /*!< CLK CLKSEL2: I2S_S Mask */

/* CLK CLKDIV Bit Field Definitions */
#define CLK_CLKDIV_ADC_N_Pos                 16                                       /*!< CLK CLKDIV: ADC_N Position */
#define CLK_CLKDIV_ADC_N_Msk                 (0xFFul << CLK_CLKDIV_ADC_N_Pos)         /*!< CLK CLKDIV: ADC_N Mask */

#define CLK_CLKDIV_UART_N_Pos                8                                        /*!< CLK CLKDIV: UART_N Position */
#define CLK_CLKDIV_UART_N_Msk                (0xFul << CLK_CLKDIV_UART_N_Pos)         /*!< CLK CLKDIV: UART_N Mask */

#define CLK_CLKDIV_USB_N_Pos                 4                                        /*!< CLK CLKDIV: USB_N Position */
#define CLK_CLKDIV_USB_N_Msk                 (0xFul << CLK_CLKDIV_USB_N_Pos)          /*!< CLK CLKDIV: USB_N Mask */

#define CLK_CLKDIV_HCLK_N_Pos                0                                        /*!< CLK CLKDIV: HCLK_N Position */
#define CLK_CLKDIV_HCLK_N_Msk                (0xFul << CLK_CLKDIV_HCLK_N_Pos)         /*!< CLK CLKDIV: HCLK_N Mask */

/* CLK PLLCON Bit Field Definitions */
#define CLK_PLLCON_PLL_SRC_Pos               19                                       /*!< CLK PLLCON: PLL_SRC Position */
#define CLK_PLLCON_PLL_SRC_Msk               (1ul << CLK_PLLCON_PLL_SRC_Pos)          /*!< CLK PLLCON: PLL_SRC Mask */

#define CLK_PLLCON_OE_Pos                    18                                       /*!< CLK PLLCON: PLL_SRC Position */
#define CLK_PLLCON_OE_Msk                    (1ul << CLK_PLLCON_OE_Pos)               /*!< CLK PLLCON: PLL_SRC Mask */

#define CLK_PLLCON_BP_Pos                    17                                       /*!< CLK PLLCON: OE Position */
#define CLK_PLLCON_BP_Msk                    (1ul << CLK_PLLCON_BP_Pos)               /*!< CLK PLLCON: OE Mask */

#define CLK_PLLCON_PD_Pos                    16                                       /*!< CLK PLLCON: PD Position */
#define CLK_PLLCON_PD_Msk                    (1ul << CLK_PLLCON_PD_Pos)               /*!< CLK PLLCON: PD Mask */

#define CLK_PLLCON_OUT_DV_Pos                14                                       /*!< CLK PLLCON: OUT_DV Position */
#define CLK_PLLCON_OUT_DV_Msk                (3ul << CLK_PLLCON_OUT_DV_Pos)           /*!< CLK PLLCON: OUT_DV Mask */

#define CLK_PLLCON_IN_DV_Pos                 9                                        /*!< CLK PLLCON: IN_DV Position */
#define CLK_PLLCON_IN_DV_Msk                 (0x1Ful << CLK_PLLCON_IN_DV_Pos)         /*!< CLK PLLCON: IN_DV Mask */

#define CLK_PLLCON_FB_DV_Pos                 0                                        /*!< CLK PLLCON: FB_DV Position */
#define CLK_PLLCON_FB_DV_Msk                 (0x1FFul << CLK_PLLCON_FB_DV_Pos)        /*!< CLK PLLCON: FB_DV Mask */

/* CLK FRQDIV Bit Field Definitions */
#define CLK_FRQDIV_DIVIDER_EN_Pos            4                                        /*!< CLK FRQDIV: DIVIDER_EN Position */
#define CLK_FRQDIV_DIVIDER_EN_Msk            (1ul << CLK_FRQDIV_DIVIDER_EN_Pos)       /*!< CLK FRQDIV: DIVIDER_EN Mask */

#define CLK_FRQDIV_FSEL_Pos                  0                                        /*!< CLK FRQDIV: FRQDIV_FSEL Position */
#define CLK_FRQDIV_FSEL_Msk                  (0xFul << CLK_FRQDIV_FSEL_Pos)           /*!< CLK FRQDIV: FRQDIV_FSEL Mask */

/* CLK APBDIV Bit Field Definitions */
#define CLK_APBDIV_APBDIV_Pos                0                                        /*!< CLK APBDIV: APBDIV Position */
#define CLK_APBDIV_APBDIV_Msk                (1ul << CLK_APBDIV_APBDIV_Pos)           /*!< CLK APBDIV: APBDIV Mask */
/*@}*/ /* end of group REG_CLK_BITMASK */
/*@}*/ /* end of group REG_CLK */


/*----------------------------- Cyclic Redundancy Check (CRC) Controller -----------------------------*/
/** @addtogroup REG_CRC CRC
  Memory Mapped Structure for Cyclic Redundancy Check
  @{
 */
typedef struct
{
    /**
     * CRC_CTL
     * ===================================================================================================
     * Offset: 0x00  CRC Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CRCCEN    |CRC Channel Enable
     * |        |          |0 = No effect.
     * |        |          |1 = CRC operation Enabled.
     * |        |          |Note1: When operating in CRC DMA mode (TRIG_EN (CRC_CTL[23]) = 1), if user clears this bit, the DMA operation will be continuous until all CRC DMA operation is done, and the TRIG_EN (CRC_CTL[23]) bit will keep 1until all CRC DMA operation done.
     * |        |          |But in this case, the CRC_BLKD_IF (CRC_DMAISR[1])flag will inactive, user can read CRC checksum result only if TRIG_EN (CRC_CTL[23]) clears to 0.
     * |        |          |Note2: When operating in CRC DMA mode (TRIG_EN (CRC_CTL[23]) = 1), if user wants to stop the transfer immediately, user can write 1 to CRC_RST (CRC_CTL [1]) bit to stop the transmission.
     * |[1]     |CRC_RST   |CRC Engine Reset
     * |        |          |0 = No effect.
     * |        |          |1 = Reset the internal CRC state machine and internal buffer.
     * |        |          |The others contents of CRC_CTL register will not be cleared.
     * |        |          |This bit will be cleared automatically.
     * |        |          |Note: When operated in CPU PIO mode, setting this bit will reload the initial seed value (CRC_SEED register).
     * |[23]    |TRIG_EN   |Trigger Enable
     * |        |          |This bit is used to trigger the CRC DMA transfer.
     * |        |          |0 = No effect.
     * |        |          |1 = CRC DMA data read or write transfer Enabled.
     * |        |          |Note1: If this bit asserts which indicates the CRC engine operation in CRC DMA mode, do not fill in any data in CRC_WDATA register.
     * |        |          |Note2: When CRC DMA transfer completed, this bit will be cleared automatically.
     * |        |          |Note3: If the bus error occurs when CRC DMA transfer data, all CRC DMA transfer will be stopped.
     * |        |          |Software must reset all DMA channel before trigger DMA again.
     * |[24]    |WDATA_RVS |Write Data Order Reverse
     * |        |          |This bit is used to enable the bit order reverse function for write data value in CRC_WDATA register.
     * |        |          |0 = Bit order reverse for CRC write data in Disabled.
     * |        |          |1 = Bit order reverse for CRC write data in Enabled (per byre).
     * |        |          |Note: If the write data is 0xAABBCCDD, the bit order reverse for CRC write data in is 0x55DD33BB
     * |[25]    |CHECKSUM_RVS|Checksum Reverse
     * |        |          |This bit is used to enable the bit order reverse function for write data value in CRC_CHECKSUM register.
     * |        |          |0 = Bit order reverse for CRC checksum Disabled.
     * |        |          |1 = Bit order reverse for CRC checksum Enabled.
     * |        |          |Note: If the checksum result is 0XDD7B0F2E, the bit order reverse for CRC checksum is 0x74F0DEBB
     * |[26]    |WDATA_COM |Write Data 1's Complement
     * |        |          |This bit is used to enable the 1's complement function for write data value in CRC_WDATA register.
     * |        |          |0 = 1's complement for CRC write data in Disabled.
     * |        |          |1 = 1's complement for CRC write data in Enabled.
     * |[27]    |CHECKSUM_COM|Checksum 1's Complement
     * |        |          |This bit is used to enable the 1's complement function for checksum result in CRC_CHECKSUM register.
     * |        |          |0 = 1's complement for CRC checksum Disabled.
     * |        |          |1 = 1's complement for CRC checksum Enabled.
     * |[29:28] |CPU_WDLEN |CPU Write Data Length
     * |        |          |This field indicates the CPU write data length only when operating in CPU PIO mode.
     * |        |          |00 = The write data length is 8-bit mode.
     * |        |          |01 = The write data length is 16-bit mode.
     * |        |          |10 = The write data length is 32-bit mode.
     * |        |          |11 = Reserved.
     * |        |          |Note1: This field is only valid when operating in CPU PIO mode.
     * |        |          |Note2: When the write data length is 8-bit mode, the valid data in CRC_WDATA register is only CRC_WDATA [7:0] bits; if the write data length is 16-bit mode, the valid data in CRC_WDATA register is only CRC_WDATA [15:0].
     * |[31:30] |CRC_MODE  |CRC Polynomial Mode
     * |        |          |This field indicates the CRC operation polynomial mode.
     * |        |          |00 = CRC-CCITT Polynomial Mode.
     * |        |          |01 = CRC-8 Polynomial Mode.
     * |        |          |10 = CRC-16 Polynomial Mode.
     * |        |          |11 = CRC-32 Polynomial Mode.
     */
    __IO uint32_t  CTL;

    /**
     * CRC_DMASAR
     * ===================================================================================================
     * Offset: 0x04  CRC DMA Source Address Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |CRC_DMASAR|CRC DMA Transfer Source Address Register
     * |        |          |This field indicates a 32-bit source address of CRC DMA.
     * |        |          |(CRC_DMASAR + CRC_DMABCR) = (CRC_DMACSAR + CRC_DMACBCR).
     * |        |          |Note: The source address must be word alignment
     */
    __IO uint32_t  DMASAR;

    /**
     * @cond HIDDEN_SYMBOLS
     */
    __I  uint32_t  RESERVED0;
    /**
     * @endcond
     */

    /**
     * CRC_DMABCR
     * ===================================================================================================
     * Offset: 0x0C  CRC DMA Transfer Byte Count Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |CRC_DMABCR|CRC DMA Transfer Byte Count Register
     * |        |          |This field indicates a 16-bit total transfer byte count number of CRC DMA
     * |        |          |(CRC_DMASAR + CRC_DMABCR) = (CRC_DMACSAR + CRC_DMACBCR).
     */
    __IO uint32_t  DMABCR ;

    /**
     * @cond HIDDEN_SYMBOLS
     */
    __I  uint32_t  RESERVED1;
    /**
     * @endcond
     */

    /**
     * CRC_DMACSAR
     * ===================================================================================================
     * Offset: 0x14  CRC DMA Current Source Address Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |CRC_DMACSAR|CRC DMA Current Source Address Register (Read Only)
     * |        |          |This field indicates the current source address where the CRC DMA transfer just occurs.
     * |        |          |(CRC_DMASAR + CRC_DMABCR) = (CRC_DMACSAR + CRC_DMACBCR).
     */
    __I  uint32_t  DMACSAR;

    /**
     * @cond HIDDEN_SYMBOLS
     */
    __I  uint32_t  RESERVED2;
    /**
     * @endcond
     */

    /**
     * CRC_DMACBCR
     * ===================================================================================================
     * Offset: 0x1C  CRC DMA Current Transfer Byte Count Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |CRC_DMACBCR|CRC DMA Current Remained Byte Count Register (Read Only)
     * |        |          |This field indicates the current remained byte count of CRC DMA.
     * |        |          |(CRC_DMASAR + CRC_DMABCR) = (CRC_DMACSAR + CRC_DMACBCR).
     * |        |          |Note: Setting CRC_RST (CRC_CTL[1]) bit to 1 will clear this register value.
     */
    __I  uint32_t  DMACBCR;

    /**
     * CRC_DMAIER
     * ===================================================================================================
     * Offset: 0x20  CRC DMA Interrupt Enable Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CRC_TABORT_IE|CRC DMA Read/Write Target Abort Interrupt Enable
     * |        |          |Enable this bit will generate the CRC DMA Target Abort interrupt signal while CRC_TARBOT_IF (CRC_DMAISR[0]) bit is set to 1.
     * |        |          |0 = Target abort interrupt generation Disabled during CRC DMA transfer.
     * |        |          |1 = Target abort interrupt generation Enabled during CRC DMA transfer.
     * |[1]     |CRC_BLKD_IE|CRC DMA Block Transfer Done Interrupt Enable
     * |        |          |Enable this bit will generate the CRC DMA Transfer Done interrupt signal while CRC_BLKD_IF (CRC_DMAISR[1]) bit is set to 1.
     * |        |          |0 = Interrupt generator Disabled when CRC DMA transfer done.
     * |        |          |1 = Interrupt generator Enabled when CRC DMA transfer done.
     */
    __IO uint32_t  DMAIER ;

    /**
     * CRC_DMAISR
     * ===================================================================================================
     * Offset: 0x24  CRC DMA Interrupt Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CRC_TABORT_IF|CRC DMA Read/Write Target Abort Interrupt Flag
     * |        |          |This bit indicates that CRC bus has error or not during CRC DMA transfer.
     * |        |          |0 = No bus error response received during CRC DMA transfer.
     * |        |          |1 = Bus error response received during CRC DMA transfer.
     * |        |          |It is cleared by writing 1 to it through software.
     * |        |          |Note: The bit filed indicate bus master received error response or not.
     * |        |          |If bus master received error response, it means that CRC transfer target abort is happened.
     * |        |          |DMA will stop transfer and respond this event to software then CRC state machine goes to IDLE state.
     * |        |          |When target abort occurred, software must reset DMA before transfer those data again.
     * |[1]     |CRC_BLKD_IF|CRC DMA Block Transfer Done Interrupt Flag
     * |        |          |This bit indicates that CRC DMA transfer has finished or not.
     * |        |          |0 = Not finished if TRIG_EN (CRC_CTL[23]) bit has enabled.
     * |        |          |1 = CRC transfer done if TRIG_EN (CRC_CTL[23]) bit has enabled.
     * |        |          |It is cleared by writing 1 to it through software.
     * |        |          |(When CRC DMA transfer done, TRIG_EN (CRC_CTL[23]) bit will be cleared automatically)
     */
    __IO uint32_t  DMAISR;

    /**
     * @cond HIDDEN_SYMBOLS
     */
    __I  uint32_t  RESERVED3[22];
    /**
     * @endcond
     */

    /**
     * CRC_WDATA
     * ===================================================================================================
     * Offset: 0x80  CRC Write Data Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |CRC_WDATA |CRC Write Data Register
     * |        |          |When operating in CPU PIO mode, software can write data to this field to perform CRC operation.
     * |        |          |When operating in DMA mode, this field indicates the DMA read data from memory and cannot be written.
     * |        |          |Note: When the write data length is 8-bit mode, the valid data in CRC_WDATA register is only CRC_WDATA [7:0] bits; if the write data length is 16-bit mode, the valid data in CRC_WDATA register is only CRC_WDATA [15:0].
     */
    __IO uint32_t  WDATA;

    /**
     * CRC_SEED
     * ===================================================================================================
     * Offset: 0x84  CRC Seed Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |CRC_SEED  |CRC Seed Register
     * |        |          |This field indicates the CRC seed value.
     */
    __IO uint32_t  SEED;

    /**
     * CRC_CHECKSUM
     * ===================================================================================================
     * Offset: 0x88  CRC Checksum Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |CRC_CHECKSUM|CRC Checksum Register
     * |        |          |This fields indicates the CRC checksum result
     */
    __I  uint32_t  CHECKSUM;
} CRC_T;

/** @addtogroup REG_CRC_BITMASK CRC Bit Mask
  @{
 */

/* CRC CTL Bit Field Definitions */
#define CRC_CTL_CRC_MODE_Pos            30                                      /*!< CRC CRC_CTL : CRC_MODE Position */
#define CRC_CTL_CRC_MODE_Msk            (0x3ul << CRC_CTL_CRC_MODE_Pos)         /*!< CRC CRC_CTL : CRC_MODE Mask */

#define CRC_CTL_CPU_WDLEN_Pos           28                                      /*!< CRC CRC_CTL : CPU_WDLEN Position */
#define CRC_CTL_CPU_WDLEN_Msk           (0x3ul << CRC_CTL_CPU_WDLEN_Pos)        /*!< CRC CRC_CTL : CPU_WDLEN Mask */

#define CRC_CTL_CHECKSUM_COM_Pos        27                                      /*!< CRC CRC_CTL : CHECKSUM_COM Position */
#define CRC_CTL_CHECKSUM_COM_Msk        (1ul << CRC_CTL_CHECKSUM_COM_Pos)       /*!< CRC CRC_CTL : CHECKSUM_COM Mask */

#define CRC_CTL_WDATA_COM_Pos           26                                      /*!< CRC CRC_CTL : WDATA_COM Position */
#define CRC_CTL_WDATA_COM_Msk           (1ul << CRC_CTL_WDATA_COM_Pos)          /*!< CRC CRC_CTL : WDATA_COM Mask */

#define CRC_CTL_CHECKSUM_RVS_Pos        25                                      /*!< CRC CRC_CTL : CHECKSUM_RVS Position */
#define CRC_CTL_CHECKSUM_RVS_Msk        (1ul << CRC_CTL_CHECKSUM_RVS_Pos)       /*!< CRC CRC_CTL : CHECKSUM_RVS Mask */

#define CRC_CTL_WDATA_RVS_Pos           24                                      /*!< CRC CRC_CTL : WDATA_RVS Position */
#define CRC_CTL_WDATA_RVS_Msk           (1ul << CRC_CTL_WDATA_RVS_Pos)          /*!< CRC CRC_CTL : WDATA_RVS Mask */

#define CRC_CTL_TRIG_EN_Pos             23                                      /*!< CRC CRC_CTL : TRIG_EN Position */
#define CRC_CTL_TRIG_EN_Msk             (1ul << CRC_CTL_TRIG_EN_Pos)            /*!< CRC CRC_CTL : TRIG_EN Mask */

#define CRC_CTL_CRC_RST_Pos             1                                       /*!< CRC CRC_CTL : CRC_RST Position */
#define CRC_CTL_CRC_RST_Msk             (1ul << CRC_CTL_CRC_RST_Pos)            /*!< CRC CRC_CTL : CRC_RST Mask */

#define CRC_CTL_CRCCEN_Pos              0                                       /*!< CRC CRC_CTL : CRCCEN Position */
#define CRC_CTL_CRCCEN_Msk              (1ul << CRC_CTL_CRCCEN_Pos)             /*!< CRC CRC_CTL : CRCCEN Mask */

/* CRC DMASAR Bit Field Definitions */
#define CRC_DMASAR_CRC_DMASAR_Pos       0                                               /*!< CRC CRC_DMASAR : CRC_DMASAR Position */
#define CRC_DMASAR_CRC_DMASAR_Msk       (0xFFFFFFFFul << CRC_DMASAR_CRC_DMASAR_Pos)     /*!< CRC CRC_DMASAR : CRC_DMASAR Mask */

/* CRC DMABCR Bit Field Definitions */
#define CRC_DMABCR_CRC_DMABCR_Pos       0                                               /*!< CRC CRC_DMABCR : CRC_DMABCR Position */
#define CRC_DMABCR_CRC_DMABCR_Msk       (0xFFFFul << CRC_DMABCR_CRC_DMABCR_Pos)         /*!< CRC CRC_DMABCR : CRC_DMABCR Mask */

/* CRC DMACSAR Bit Field Definitions */
#define CRC_DMACSAR_CRC_DMACSAR_Pos     0                                               /*!< CRC CRC_DMACSAR : CRC_DMACSAR Position */
#define CRC_DMACSAR_CRC_DMACSAR_Msk     (0xFFFFFFFFul << CRC_DMACSAR_CRC_DMACSAR_Pos)   /*!< CRC CRC_DMACSAR : CRC_DMACSAR Mask */

/* CRC DMACBCR Bit Field Definitions */
#define CRC_DMACBCR_CRC_DMACBCR_Pos     0                                               /*!< CRC CRC_DMACBCR : DMACBCR Position */
#define CRC_DMACBCR_CRC_DMACBCR_Msk     (0xFFFFul << CRC_DMACBCR_CRC_DMACBCR_Pos)       /*!< CRC CRC_DMACBCR : DMACBCR Mask */

/* CRC DMAIER Bit Field Definitions */
#define CRC_DMAIER_CRC_BLKD_IE_Pos      1                                               /*!< CRC CRC_DMAIER : CRC_BLKD_IE Position */
#define CRC_DMAIER_CRC_BLKD_IE_Msk      (1ul << CRC_DMAIER_CRC_BLKD_IE_Pos)             /*!< CRC CRC_DMAIER : CRC_BLKD_IE Mask */

#define CRC_DMAIER_CRC_TABORT_IE_Pos    0                                               /*!< CRC CRC_DMAIER : CRC_TABORT_IE Position */
#define CRC_DMAIER_CRC_TABORT_IE_Msk    (1ul << CRC_DMAIER_CRC_TABORT_IE_Pos)           /*!< CRC CRC_DMAIER : CRC_TABORT_IE Mask */

/* CRC DMAISR Bit Field Definitions */
#define CRC_DMAISR_CRC_BLKD_IF_Pos      1                                               /*!< CRC CRC_DMAISR : CRC_BLKD_IF Position */
#define CRC_DMAISR_CRC_BLKD_IF_Msk      (1ul << CRC_DMAISR_CRC_BLKD_IF_Pos)             /*!< CRC CRC_DMAISR : CRC_BLKD_IF Mask */

#define CRC_DMAISR_CRC_TABORT_IF_Pos    0                                               /*!< CRC CRC_DMAISR : CRC_TABORT_IF Position */
#define CRC_DMAISR_CRC_TABORT_IF_Msk    (1ul << CRC_DMAISR_CRC_TABORT_IF_Pos)           /*!< CRC CRC_DMAISR : CRC_TABORT_IF Mask */

/* CRC WDATA Bit Field Definitions */
#define CRC_WDATA_CRC_WDATA_Pos         0                                               /*!< CRC CRC_WDATA : CRC_WDATA Position */
#define CRC_WDATA_CRC_WDATA_Msk         (0xFFFFFFFFul << CRC_WDATA_CRC_WDATA_Pos)       /*!< CRC CRC_WDATA : CRC_WDATA Mask */

/* CRC SEED Bit Field Definitions */
#define CRC_SEED_CRC_SEED_Pos           0                                               /*!< CRC CRC_SEED : CRC_SEED Position */
#define CRC_SEED_CRC_SEED_Msk           (0xFFFFFFFFul << CRC_SEED_CRC_SEED_Pos)         /*!< CRC CRC_SEED : CRC_SEED Mask */

/* CRC CHECKSUM Bit Field Definitions */
#define CRC_CHECKSUM_CRC_CHECKSUM_Pos   0                                               /*!< CRC CRC_CHECKSUM : CRC_CHECKSUM Position */
#define CRC_CHECKSUM_CRC_CHECKSUM_Msk   (0xFFFFFFFFul << CRC_CHECKSUM_CRC_CHECKSUM_Pos) /*!< CRC CRC_CHECKSUM : CRC_CHECKSUM Mask */
/*@}*/ /* end of group REG_CRC_BITMASK */
/*@}*/ /* end of group REG_CRC */

/*-------------------------- FLASH Memory Controller -------------------------*/
/** @addtogroup REG_FMC FMC
  Memory Mapped Structure for Flash Memory Controller
  @{
 */
typedef struct
{
    /**
     * ISPCON
     * ===================================================================================================
     * Offset: 0x00  ISP Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ISPEN     |ISP Enable
     * |        |          |This bit is protected bit. ISP function enable bit. Set this bit to enable ISP function.
     * |        |          |1 = Enable ISP function
     * |        |          |0 = Disable ISP function
     * |[1]     |BS        |Boot Select
     * |        |          |This bit is protected bit. Set/clear this bit to select next booting from LDROM/APROM,
     * |        |          |respectively. This bit also functions as MCU booting status flag, which can be used to check where
     * |        |          |MCU booted from. This bit is initiated with the inverted value of CBS in Config0 after power-
     * |        |          |on reset; It keeps the same value at other reset.
     * |        |          |1 = boot from LDROM
     * |        |          |0 = boot from APROM
     * |[4]     |CFGUEN    |Config Update Enable
     * |        |          |Writing this bit to 1 enables s/w to update Config value by ISP procedure regardless of program
     * |        |          |code is running in APROM or LDROM.
     * |        |          |1 = Config update enable
     * |        |          |0 = Config update disable
     * |[5]     |LDUEN     |LDROM Update Enable
     * |        |          |LDROM update enable bit.
     * |        |          |1 = LDROM can be updated when the MCU runs in APROM.
     * |        |          |0 = LDROM cannot be updated
     * |[6]     |ISPFF     |ISP Fail Flag
     * |        |          |This bit is set by hardware when a triggered ISP meets any of the following conditions:
     * |        |          |(1) APROM writes to itself.
     * |        |          |(2) LDROM writes to itself.
     * |        |          |(3) Destination address is illegal, such as over an available range.
     * |        |          |Write 1 to clear.
     * |[7]     |SWRST     |Software Reset
     * |        |          |Writing 1 to this bit to start software reset.
     * |        |          |It is cleared by hardware after reset is finished.
     */
    __IO uint32_t ISPCON;

    /**
     * ISPADR
     * ===================================================================================================
     * Offset: 0x04  ISP Address Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |ISPADR    |ISP Address
     * |        |          |it supports word program only. ISPARD[1:0] must be kept 2'b00 for ISP operation.
     */
    __IO uint32_t ISPADR;

    /**
     * ISPDAT
     * ===================================================================================================
     * Offset: 0x08  ISP Data Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |ISPDAT    |ISP Data
     * |        |          |Write data to this register before ISP program operation
     * |        |          |Read data from this register after ISP read operation
     */
    __IO uint32_t ISPDAT;

    /**
     * ISPCMD
     * ===================================================================================================
     * Offset: 0x0C  ISP Command Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[5:0]   |ISPCMD    |ISP Command
     * |        |          |ISP command table is shown below:
     * |        |          |0x00 = Read.
     * |        |          |0x21 = Program.
     * |        |          |0x22 = Page Erase.
     */
    __IO uint32_t ISPCMD;

    /**
     * ISPTRG
     * ===================================================================================================
     * Offset: 0x10  IISP Trigger Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ISPGO     |ISP start trigger
     * |        |          |Write 1 to start ISP operation and this bit will be cleared to 0 by hardware automatically when ISP
     * |        |          |operation is finish.
     * |        |          |1 = ISP is on going
     * |        |          |0 = ISP done
     */
    __IO uint32_t ISPTRG;

    /**
     * DFBADR
     * ===================================================================================================
     * Offset: 0x14  Data Flash Base Address Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |DFBA      |Data Flash Base Address
     * |        |          |This register indicates data flash start address.
     * |        |          |It is a read only register.
     * |        |          |For 8/16/32/64kB flash memory device, the data flash size is 4kB and it start address is fixed at
     * |        |          |0x01F000 by hardware internally.
     */
    __I  uint32_t DFBADR;

    /**
     * FATCON
     * ===================================================================================================
     * Offset: 0x18  Flash Access Time Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |FPSEN     |Flash Power Save Enable
     * |        |          |If CPU clock is slower than 24 MHz, then s/w can enable flash power saving function.
     * |        |          |1 = Enable flash power saving
     * |        |          |0 = Disable flash power saving
     * |[4]     |L_SPEED   |Flash Low Speed Mode Enable
     * |        |          |1 = Flash access always no wait state (zero wait state)
     * |        |          |0 = Insert wait state while Flash access discontinued address.
     * |        |          |Note: Set this bit only when HCLK <= 25MHz. If HCLK > 25MHz, CPU will fetch wrong
     * |        |          |code and cause fail result.
     */
    __IO uint32_t FATCON;

    /**
     * @cond HIDDEN_SYMBOLS
     */
    __I  uint32_t  RESERVED[9];
    /**
     * @endcond
     */

    __IO uint32_t ISPSTA;

} FMC_T;

/** @addtogroup REG_FMC_BITMASK FMC Bit Mask
  @{
 */

/* FMC ISPCON Bit Field Definitions */
#define FMC_ISPCON_ET_Pos                       12                                      /*!< FMC ISPCON: ET Position */
#define FMC_ISPCON_ET_Msk                       (7ul << FMC_ISPCON_ET_Pos)              /*!< FMC ISPCON: ET Mask     */

#define FMC_ISPCON_PT_Pos                       8                                       /*!< FMC ISPCON: PT Position */
#define FMC_ISPCON_PT_Msk                       (7ul << FMC_ISPCON_PT_Pos)              /*!< FMC ISPCON: PT Mask     */

#define FMC_ISPCON_ISPFF_Pos                    6                                       /*!< FMC ISPCON: ISPFF Position */
#define FMC_ISPCON_ISPFF_Msk                    (1ul << FMC_ISPCON_ISPFF_Pos)           /*!< FMC ISPCON: ISPFF Mask */

#define FMC_ISPCON_LDUEN_Pos                    5                                       /*!< FMC ISPCON: LDUEN Position */
#define FMC_ISPCON_LDUEN_Msk                    (1ul << FMC_ISPCON_LDUEN_Pos)           /*!< FMC ISPCON: LDUEN Mask */

#define FMC_ISPCON_CFGUEN_Pos                   4                                       /*!< FMC ISPCON: CFGUEN Position */
#define FMC_ISPCON_CFGUEN_Msk                   (1ul << FMC_ISPCON_CFGUEN_Pos)          /*!< FMC ISPCON: CFGUEN Mask */

#define FMC_ISPCON_APUEN_Pos                    3                                       /*!< FMC ISPCON: APUEN Position */
#define FMC_ISPCON_APUEN_Msk                    (1ul << FMC_ISPCON_APUEN_Pos)           /*!< FMC ISPCON: APUEN Mask */

#define FMC_ISPCON_BS_Pos                       1                                       /*!< FMC ISPCON: BS Position */
#define FMC_ISPCON_BS_Msk                       (0x1ul << FMC_ISPCON_BS_Pos)            /*!< FMC ISPCON: BS Mask */

#define FMC_ISPCON_ISPEN_Pos                    0                                       /*!< FMC ISPCON: ISPEN Position */
#define FMC_ISPCON_ISPEN_Msk                    (1ul << FMC_ISPCON_ISPEN_Pos)           /*!< FMC ISPCON: ISPEN Mask */

/* FMC ISPADR Bit Field Definitions */
#define FMC_ISPADR_ISPADR_Pos                   0                                       /*!< FMC ISPADR: ISPADR Position */
#define FMC_ISPADR_ISPADR_Msk                   (0xFFFFFFFFul << FMC_ISPADR_ISPADR_Pos) /*!< FMC ISPADR: ISPADR Mask     */

/* FMC ISPADR Bit Field Definitions */
#define FMC_ISPDAT_ISPDAT_Pos                   0                                       /*!< FMC ISPDAT: ISPDAT Position */
#define FMC_ISPDAT_ISPDAT_Msk                   (0xFFFFFFFFul << FMC_ISPDAT_ISPDAT_Pos) /*!< FMC ISPDAT: ISPDAT Mask     */

/* FMC ISPCMD Bit Field Definitions */
#define FMC_ISPCMD_FOEN_Pos                     5                                       /*!< FMC ISPCMD: FOEN Position */
#define FMC_ISPCMD_FOEN_Msk                     (1ul << FMC_ISPCMD_FOEN_Pos)            /*!< FMC ISPCMD: FOEN Mask */

#define FMC_ISPCMD_FCEN_Pos                     4                                       /*!< FMC ISPCMD: FCEN Position */
#define FMC_ISPCMD_FCEN_Msk                     (1ul << FMC_ISPCMD_FCEN_Pos)            /*!< FMC ISPCMD: FCEN Mask */

#define FMC_ISPCMD_FCTRL_Pos                    0                                       /*!< FMC ISPCMD: FCTRL Position */
#define FMC_ISPCMD_FCTRL_Msk                    (0xFul << FMC_ISPCMD_FCTRL_Pos)         /*!< FMC ISPCMD: FCTRL Mask */

/* FMC ISPTRG Bit Field Definitions */
#define FMC_ISPTRG_ISPGO_Pos                    0                                       /*!< FMC ISPTRG: ISPGO Position */
#define FMC_ISPTRG_ISPGO_Msk                    (1ul << FMC_ISPTRG_ISPGO_Pos)           /*!< FMC ISPTRG: ISPGO Mask */

/* FMC DFBADR Bit Field Definitions */
#define FMC_DFBADR_DFBA_Pos                     0                                       /*!< FMC DFBADR: DFBA Position */
#define FMC_DFBADR_DFBA_Msk                     (0xFFFFFFFFul << FMC_DFBADR_DFBA_Pos)   /*!< FMC DFBADR: DFBA Mask     */

/* FMC FATCON Bit Field Definitions */
#define FMC_FATCON_FOMSEL1_Pos                  6                                       /*!< FMC FATCON: FOMSEL1 Position */
#define FMC_FATCON_FOMSEL1_Msk                  (1ul << FMC_FATCON_FOMSEL1_Pos)         /*!< FMC FATCON: FOMSEL1 Mask */

#define FMC_FATCON_FOMSEL0_Pos                  4                                       /*!< FMC FATCON: FOMSEL0 Position */
#define FMC_FATCON_FOMSEL0_Msk                  (1ul << FMC_FATCON_FOMSEL0_Pos)         /*!< FMC FATCON: FOMSEL0 Mask */

#define FMC_FATCON_FATS_Pos                     1                                       /*!< FMC FATCON: FATS Position */
#define FMC_FATCON_FATS_Msk                     (7ul << FMC_FATCON_FATS_Pos)            /*!< FMC FATCON: FATS Mask */

#define FMC_FATCON_FPSEN_Pos                    0                                       /*!< FMC FATCON: FPSEN Position */
#define FMC_FATCON_FPSEN_Msk                    (1ul << FMC_FATCON_FPSEN_Pos)           /*!< FMC FATCON: FPSEN Mask */


#define FMC_ISPSTA_ISPGO_Pos                    0                                       /*!< FMC ISPSTA: ISPGO Position */
#define FMC_ISPSTA_ISPGO_Msk                    (1ul << FMC_ISPSTA_ISPGO_Pos)           /*!< FMC ISPSTA: ISPGO Mask */

#define FMC_ISPSTA_CBS_Pos                      1                                       /*!< FMC ISPSTA: CBS Position */
#define FMC_ISPSTA_CBS_Msk                      (0x3ul << FMC_ISPSTA_CBS_Pos)           /*!< FMC ISPSTA: CBS Mask */

#define FMC_ISPSTA_ISPFF_Pos                    6                                       /*!< FMC ISPSTA: ISPFF Position */
#define FMC_ISPSTA_ISPFF_Msk                    (0x3ul << FMC_ISPSTA_ISPFF_Pos)         /*!< FMC ISPSTA: ISPFF Mask */

#define FMC_ISPSTA_VECMAP_Pos                   9                                       /*!< FMC ISPSTA: VECMAP Position */
#define FMC_ISPSTA_VECMAP_Msk                   (0xFFFul << FMC_ISPSTA_VECMAP_Pos)      /*!< FMC ISPSTA: VECMAP Mask */

/*@}*/ /* end of group REG_FMC_BITMASK */
/*@}*/ /* end of group REG_FMC */



/*--------------------- General Purpose I/O (GPIO) ---------------------*/
/** @addtogroup REG_GPIO GPIO
  Memory Mapped Structure for General Purpose I/O
  @{
 */
typedef struct
{
    /**
     * GPIOx_PMD
     * ===================================================================================================
     * Offset: 0x00  GPIO Port [A/B/C/D/E/F] Pin I/O Mode Control
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2n+1:2n]|PMDn     |GPIOx I/O Pin[N] Mode Control
     * |        |          |Determine each I/O mode of GPIOx pins.
     * |        |          |00 = GPIO port [n] pin is in Input mode.
     * |        |          |01 = GPIO port [n] pin is in Push-pull Output mode.
     * |        |          |10 = GPIO port [n] pin is in Open-drain Output mode.
     * |        |          |11 = GPIO port [n] pin is in Quasi-bidirectional mode.
     * |        |          |Note:
     * |        |          |GPIOA: valid n are 15~10. Others are reserved.
     * |        |          |GPIOB: valid n are 15~12, 10~0. Others are reserved.
     * |        |          |GPIOC: valid n are 13~8, 5~0. Others are reserved.
     * |        |          |GPIOD: valid n are 11~8, 5~0. Others are reserved.
     * |        |          |GPIOF: valid n are 3~0. Others are reserved.
     */
    __IO uint32_t  PMD;

    /**
     * GPIOx_OFFD
     * ===================================================================================================
     * Offset: 0x04  GPIO Port [A/B/C/D/E/F] Pin Digital Input Path Disable Control
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:16] |OFFD      |GPIOx Pin[N] Digital Input Path Disable Control
     * |        |          |Each of these bits is used to control if the digital input path of corresponding GPIO pin is disabled.
     * |        |          |If input is analog signal, users can disable GPIO digital input path to avoid current leakage.
     * |        |          |0 = I/O digital input path Enabled.
     * |        |          |1 = I/O digital input path Disabled (digital input tied to low).
     * |        |          |Note:
     * |        |          |GPIOA: valid n are 15~10. Others are reserved.
     * |        |          |GPIOB: valid n are 15~12, 10~0. Others are reserved.
     * |        |          |GPIOC: valid n are 13~8, 5~0. Others are reserved.
     * |        |          |GPIOD: valid n are 11~8, 5~0. Others are reserved.
     * |        |          |GPIOF: valid n are 3~0. Others are reserved.
     */
    __IO uint32_t  OFFD;

    /**
     * GPIOx_DOUT
     * ===================================================================================================
     * Offset: 0x08  GPIO Port [A/B/C/D/E/F] Data Output Value
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[n]     |DOUTn     |GPIOx Pin[N] Output Value
     * |        |          |Each of these bits controls the status of a GPIO pin when the GPIO pin is configured as Push-pull output, open-drain output or quasi-bidirectional mode.
     * |        |          |0 = GPIO port [A/B/C/D/E/F] Pin[n] will drive Low if the GPIO pin is configured as Push-pull output, Open-drain output or Quasi-bidirectional mode.
     * |        |          |1 = GPIO port [A/B/C/D/E/F] Pin[n] will drive High if the GPIO pin is configured as Push-pull output or Quasi-bidirectional mode.
     * |        |          |Note:
     * |        |          |GPIOA: valid n are 15~10. Others are reserved.
     * |        |          |GPIOB: valid n are 15~12, 10~0. Others are reserved.
     * |        |          |GPIOC: valid n are 13~8, 5~0. Others are reserved.
     * |        |          |GPIOD: valid n are 11~8, 5~0. Others are reserved.
     * |        |          |GPIOF: valid n are 3~0. Others are reserved.
     */
    __IO uint32_t  DOUT;

    /**
     * GPIOx_DMASK
     * ===================================================================================================
     * Offset: 0x0C  GPIO Port [A/B/C/D/E/F] Data Output Write Mask
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[n]     |DMASKn    |Port [A/B/C/D/E/F] Data Output Write Mask
     * |        |          |These bits are used to protect the corresponding register of GPIOx_DOUT bit[n].
     * |        |          |When the DMASK bit[n] is set to 1, the corresponding GPIOx_DOUT[n] bit is protected.
     * |        |          |If the write signal is masked, write data to the protect bit is ignored.
     * |        |          |0 = Corresponding GPIOx_DOUT[n] bit can be updated.
     * |        |          |1 = Corresponding GPIOx_DOUT[n] bit protected.
     * |        |          |Note: This function only protects the corresponding GPIOx_DOUT[n] bit, and will not protect the corresponding bit control register (GPIOAx_DOUT, GPIOBx_DOUT, GPIOCx_DOUT, GPIODx_DOUT, GPIOEx_DOUT and GPIOFx_DOUT).
     * |        |          |Note:
     * |        |          |GPIOA: valid n are 15~10. Others are reserved.
     * |        |          |GPIOB: valid n are 15~12, 10~0. Others are reserved.
     * |        |          |GPIOC: valid n are 13~8, 5~0. Others are reserved.
     * |        |          |GPIOD: valid n are 11~8, 5~0. Others are reserved.
     * |        |          |GPIOF: valid n are 3~0. Others are reserved.
     */
    __IO uint32_t  DMASK;

    /**
     * GPIOx_PIN
     * ===================================================================================================
     * Offset: 0x10  GPIO Port [A/B/C/D/E/F] Pin Value
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[n]     |PINn      |Port [A/B/C/D/E/F] Pin Values
     * |        |          |Each bit of the register reflects the actual status of the respective GPIO pin.
     * |        |          |If the bit is 1, it indicates the corresponding pin status is high, else the pin status is low.
     * |        |          |Note:
     * |        |          |GPIOA: valid n are 15~10. Others are reserved.
     * |        |          |GPIOB: valid n are 15~12, 10~0. Others are reserved.
     * |        |          |GPIOC: valid n are 13~8, 5~0. Others are reserved.
     * |        |          |GPIOD: valid n are 11~8, 5~0. Others are reserved.
     * |        |          |GPIOF: valid n are 3~0. Others are reserved.
     */
    __I  uint32_t  PIN;

    /**
     * GPIOx_DBEN
     * ===================================================================================================
     * Offset: 0x14  GPIO Port [A/B/C/D/E/F] De-bounce Enable
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[n]     |DBENn     |Port [A/B/C/D/E/F] Input Signal De-Bounce Enable
     * |        |          |DBEN[n] is used to enable the de-bounce function for each corresponding bit.
     * |        |          |If the input signal pulse width cannot be sampled by continuous two de-bounce sample cycle, the input signal transition is seen as the signal bounce and will not trigger the interrupt.
     * |        |          |The de-bounce clock source is controlled by DBNCECON[4], one de-bounce sample cycle period is controlled by DBNCECON[3:0].
     * |        |          |0 = Bit[n] de-bounce function Disabled.
     * |        |          |1 = Bit[n] de-bounce function Enabled.
     * |        |          |The de-bounce function is valid only for edge triggered interrupt.
     * |        |          |If the interrupt mode is level triggered, the de-bounce enable bit is ignored.
     * |        |          |Note:
     * |        |          |GPIOA: valid n are 15~10. Others are reserved.
     * |        |          |GPIOB: valid n are 15~12, 10~0. Others are reserved.
     * |        |          |GPIOC: valid n are 13~8, 5~0. Others are reserved.
     * |        |          |GPIOD: valid n are 11~8, 5~0. Others are reserved.
     * |        |          |GPIOF: valid n are 3~0. Others are reserved.
     */
    __IO uint32_t  DBEN;

    /**
     * GPIOA_IMD
     * ===================================================================================================
     * Offset: 0x18  GPIO Port [A/B/C/D/E/F] Interrupt Mode Control
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[n]     |IMDn      |Port [A/B/C/D/E/F] Edge Or Level Detection Interrupt Control
     * |        |          |IMD[n] is used to control the interrupt is by level trigger or by edge trigger.
     * |        |          |If the interrupt is by edge trigger, the trigger source can be controlled by de-bounce.
     * |        |          |If the interrupt is by level trigger, the input source is sampled by one HCLK.
     * |        |          |clock and generates the interrupt.
     * |        |          |0 = Edge trigger interrupt.
     * |        |          |1 = Level trigger interrupt.
     * |        |          |If the pin is set as the level trigger interrupt, only one level can be set on the registers GPIOx_IEN.
     * |        |          |If both levels to trigger interrupt are set, the setting is ignored and no interrupt will occur.
     * |        |          |The de-bounce function is valid only for edge triggered interrupt.
     * |        |          |If the interrupt mode is level triggered, the de-bounce enable bit is ignored.
     * |        |          |Note:
     * |        |          |GPIOA: valid n are 15~10. Others are reserved.
     * |        |          |GPIOB: valid n are 15~12, 10~0. Others are reserved.
     * |        |          |GPIOC: valid n are 13~8, 5~0. Others are reserved.
     * |        |          |GPIOD: valid n are 11~8, 5~0. Others are reserved.
     * |        |          |GPIOF: valid n are 3~0. Others are reserved.
     */
    __IO uint32_t  IMD;

    /**
     * GPIOx_IEN
     * ===================================================================================================
     * Offset: 0x1C  GPIO Port [A/B/C/D/E/F] Interrupt Enable
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[n]     |IF_ENn    |Port [A/B/C/D/E/F] Interrupt Enable By Input Falling Edge Or Input Level Low
     * |        |          |IF_EN[n] is used to enable the interrupt for each of the corresponding input GPIO_PIN[n].
     * |        |          |Set bit to 1 also enable the pin wake-up function.
     * |        |          |When setting the IF_EN[n] bit to 1:
     * |        |          |If the interrupt is level trigger, the input PIN[n] state at level "low" will generate the interrupt.
     * |        |          |If the interrupt is edge trigger, the input PIN[n] state change from "high-to-low" will generate the interrupt.
     * |        |          |0 = PIN[n] state low-level or high-to-low change interrupt Disabled.
     * |        |          |1 = PIN[n] state low-level or high-to-low change interrupt Enabled.
     * |        |          |Note:
     * |        |          |GPIOA: valid n are 15~10. Others are reserved.
     * |        |          |GPIOB: valid n are 15~12, 10~0. Others are reserved.
     * |        |          |GPIOC: valid n are 13~8, 5~0. Others are reserved.
     * |        |          |GPIOD: valid n are 11~8, 5~0. Others are reserved.
     * |        |          |GPIOF: valid n are 3~0. Others are reserved.
     * |[n+16]  |IR_ENn    |Port [A/B/C/D/E/F] Interrupt Enable By Input Rising Edge Or Input Level High
     * |        |          |IR_EN[n] used to enable the interrupt for each of the corresponding input GPIO_PIN[n].
     * |        |          |Set bit to 1 also enable the pin wake-up function.
     * |        |          |When setting the IR_EN[n] bit to 1:
     * |        |          |If the interrupt is level trigger, the input PIN[n] state at level "high" will generate the interrupt.
     * |        |          |If the interrupt is edge trigger, the input PIN[n] state change from "low-to-high" will generate the interrupt.
     * |        |          |0 = PIN[n] level-high or low-to-high interrupt Disabled.
     * |        |          |1 = PIN[n] level-high or low-to-high interrupt Enabled.
     * |        |          |Note:
     * |        |          |GPIOA: valid n are 15~10. Others are reserved.
     * |        |          |GPIOB: valid n are 15~12, 10~0. Others are reserved.
     * |        |          |GPIOC: valid n are 13~8, 5~0. Others are reserved.
     * |        |          |GPIOD: valid n are 11~8, 5~0. Others are reserved.
     * |        |          |GPIOF: valid n are 3~0. Others are reserved.
     */
    __IO uint32_t  IEN;

    /**
     * GPIOx_ISRC
     * ===================================================================================================
     * Offset: 0x20  GPIO Port [A/B/C/D/E/F] Interrupt Source Flag
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[n]     |ISRCn     |Port [A/B/C/D/E/F] Interrupt Source Flag
     * |        |          |Read :
     * |        |          |0 = No interrupt at GPIOx[n].
     * |        |          |1 = GPIOx[n] generates an interrupt.
     * |        |          |Write :
     * |        |          |0= No action.
     * |        |          |1= Clear the corresponding pending interrupt.
     * |        |          |Note:
     * |        |          |GPIOA: valid n are 15~10. Others are reserved.
     * |        |          |GPIOB: valid n are 15~12, 10~0. Others are reserved.
     * |        |          |GPIOC: valid n are 13~8, 5~0. Others are reserved.
     * |        |          |GPIOD: valid n are 11~8, 5~0. Others are reserved.
     * |        |          |GPIOF: valid n are 3~0. Others are reserved.
     */
    __IO uint32_t  ISRC;

} GPIO_T;

typedef struct
{
    /**
     * DBNCECON
     * ===================================================================================================
     * Offset: 0x180  External Interrupt De-bounce Control
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |DBCLKSEL  |De-Bounce Sampling Cycle Selection
     * |        |          |0000 = Sample interrupt input once per 1 clocks
     * |        |          |0001 = Sample interrupt input once per 2 clocks
     * |        |          |0010 = Sample interrupt input once per 4 clocks
     * |        |          |0011 = Sample interrupt input once per 8 clocks
     * |        |          |0100 = Sample interrupt input once per 16 clocks
     * |        |          |0101 = Sample interrupt input once per 32 clocks
     * |        |          |0110 = Sample interrupt input once per 64 clocks
     * |        |          |0111 = Sample interrupt input once per 128 clocks
     * |        |          |1000 = Sample interrupt input once per 256 clocks
     * |        |          |1001 = Sample interrupt input once per 2*256 clocks
     * |        |          |1010 = Sample interrupt input once per 4*256clocks
     * |        |          |1011 = Sample interrupt input once per 8*256 clocks
     * |        |          |1100 = Sample interrupt input once per 16*256 clocks
     * |        |          |1101 = Sample interrupt input once per 32*256 clocks
     * |        |          |1110 = Sample interrupt input once per 64*256 clocks
     * |        |          |1111 = Sample interrupt input once per 128*256 clocks
     * |        |          |Sample   interrupt input once per 128*256 clocks
     * |[4]     |DBCLKSRC  |De-Bounce Counter Clock Source Selection
     * |        |          |0 = De-bounce counter clock source is the HCLK.
     * |        |          |1 = De-bounce counter clock source is the internal 10 kHz low speed oscillator.
     * |[5]     |ICLK_ON   |Interrupt Clock On Mode
     * |        |          |0 = Edge detection circuit is active only if I/O pin corresponding GPIOx_IEN bit is set to 1.
     * |        |          |1 = All I/O pins edge detection circuit is always active after reset.
     * |        |          |It is recommended to turn off this bit to save system power if no special application concern.
     */
    __IO uint32_t  DBNCECON;
} GPIO_DBNCECON_T;

/** @addtogroup REG_GPIO_BITMASK GPIO Bit Mask
  @{
 */

/* GPIO PMD Bit Field Definitions */
#define GPIO_PMD_PMD15_Pos          30                                          /*!< GPIO PMD: PMD15 Position */
#define GPIO_PMD_PMD15_Msk          (0x3ul << GPIO_PMD_PMD15_Pos)               /*!< GPIO PMD: PMD15 Mask */

#define GPIO_PMD_PMD14_Pos          28                                          /*!< GPIO PMD: PMD14 Position */
#define GPIO_PMD_PMD14_Msk          (0x3ul << GPIO_PMD_PMD14_Pos)               /*!< GPIO PMD: PMD14 Mask */

#define GPIO_PMD_PMD13_Pos          26                                          /*!< GPIO PMD: PMD13 Position */
#define GPIO_PMD_PMD13_Msk          (0x3ul << GPIO_PMD_PMD13_Pos)               /*!< GPIO PMD: PMD13 Mask */

#define GPIO_PMD_PMD12_Pos          24                                          /*!< GPIO PMD: PMD12 Position */
#define GPIO_PMD_PMD12_Msk          (0x3ul << GPIO_PMD_PMD12_Pos)               /*!< GPIO PMD: PMD12 Mask */

#define GPIO_PMD_PMD11_Pos          22                                          /*!< GPIO PMD: PMD11 Position */
#define GPIO_PMD_PMD11_Msk          (0x3ul << GPIO_PMD_PMD11_Pos)               /*!< GPIO PMD: PMD11 Mask */

#define GPIO_PMD_PMD10_Pos          20                                          /*!< GPIO PMD: PMD10 Position */
#define GPIO_PMD_PMD10_Msk          (0x3ul << GPIO_PMD_PMD10_Pos)               /*!< GPIO PMD: PMD10 Mask */

#define GPIO_PMD_PMD9_Pos           18                                          /*!< GPIO PMD: PMD9 Position */
#define GPIO_PMD_PMD9_Msk           (0x3ul << GPIO_PMD_PMD9_Pos)                /*!< GPIO PMD: PMD9 Mask */

#define GPIO_PMD_PMD8_Pos           16                                          /*!< GPIO PMD: PMD8 Position */
#define GPIO_PMD_PMD8_Msk           (0x3ul << GPIO_PMD_PMD8_Pos)                /*!< GPIO PMD: PMD8 Mask */

#define GPIO_PMD_PMD7_Pos           14                                          /*!< GPIO PMD: PMD7 Position */
#define GPIO_PMD_PMD7_Msk           (0x3ul << GPIO_PMD_PMD7_Pos)                /*!< GPIO PMD: PMD7 Mask */

#define GPIO_PMD_PMD6_Pos           12                                          /*!< GPIO PMD: PMD6 Position */
#define GPIO_PMD_PMD6_Msk           (0x3ul << GPIO_PMD_PMD6_Pos)                /*!< GPIO PMD: PMD6 Mask */

#define GPIO_PMD_PMD5_Pos           10                                          /*!< GPIO PMD: PMD5 Position */
#define GPIO_PMD_PMD5_Msk           (0x3ul << GPIO_PMD_PMD5_Pos)                /*!< GPIO PMD: PMD5 Mask */

#define GPIO_PMD_PMD4_Pos           8                                           /*!< GPIO PMD: PMD4 Position */
#define GPIO_PMD_PMD4_Msk           (0x3ul << GPIO_PMD_PMD4_Pos)                /*!< GPIO PMD: PMD4 Mask */

#define GPIO_PMD_PMD3_Pos           6                                           /*!< GPIO PMD: PMD3 Position */
#define GPIO_PMD_PMD3_Msk           (0x3ul << GPIO_PMD_PMD3_Pos)                /*!< GPIO PMD: PMD3 Mask */

#define GPIO_PMD_PMD2_Pos           4                                           /*!< GPIO PMD: PMD2 Position */
#define GPIO_PMD_PMD2_Msk           (0x3ul << GPIO_PMD_PMD2_Pos)                /*!< GPIO PMD: PMD2 Mask */

#define GPIO_PMD_PMD1_Pos           2                                           /*!< GPIO PMD: PMD1 Position */
#define GPIO_PMD_PMD1_Msk           (0x3ul << GPIO_PMD_PMD1_Pos)                /*!< GPIO PMD: PMD1 Mask */

#define GPIO_PMD_PMD0_Pos           0                                           /*!< GPIO PMD: PMD0 Position */
#define GPIO_PMD_PMD0_Msk           (0x3ul << GPIO_PMD_PMD0_Pos)                /*!< GPIO PMD: PMD0 Mask */

/* GPIO OFFD Bit Field Definitions */
#define GPIO_OFFD_OFFD_Pos          16                                          /*!< GPIO OFFD: OFFD Position */
#define GPIO_OFFD_OFFD_Msk          (0xFFFFul << GPIO_OFFD_OFFD_Pos)            /*!< GPIO OFFD: OFFD Mask */

/* GPIO DOUT Bit Field Definitions */
#define GPIO_DOUT_DOUT_Pos          0                                           /*!< GPIO DOUT: DOUT Position */
#define GPIO_DOUT_DOUT_Msk          (0xFFFFul << GPIO_DOUT_DOUT_Pos)            /*!< GPIO DOUT: DOUT Mask */

/* GPIO DMASK Bit Field Definitions */
#define GPIO_DMASK_DMASK_Pos        0                                           /*!< GPIO DMASK: DMASK Position */
#define GPIO_DMASK_DMASK_Msk        (0xFFFFul << GPIO_DMASK_DMASK_Pos)          /*!< GPIO DMASK: DMASK Mask */

/* GPIO PIN Bit Field Definitions */
#define GPIO_PIN_PIN_Pos            0                                           /*!< GPIO PIN: PIN Position */
#define GPIO_PIN_PIN_Msk            (0xFFFFul << GPIO_PIN_PIN_Pos)              /*!< GPIO PIN: PIN Mask */

/* GPIO DBEN Bit Field Definitions */
#define GPIO_DBEN_DBEN_Pos          0                                           /*!< GPIO DBEN: DBEN Position */
#define GPIO_DBEN_DBEN_Msk          (0xFFFFul << GPIO_DBEN_DBEN_Pos)            /*!< GPIO DBEN: DBEN Mask */

/* GPIO IMD Bit Field Definitions */
#define GPIO_IMD_IMD_Pos            0                                           /*!< GPIO IMD: IMD Position */
#define GPIO_IMD_IMD_Msk            (0xFFFFul << GPIO_IMD_IMD_Pos)              /*!< GPIO IMD: IMD Mask */

/* GPIO IEN Bit Field Definitions */
#define GPIO_IEN_IR_EN_Pos          16                                          /*!< GPIO IEN: IR_EN Position */
#define GPIO_IEN_IR_EN_Msk          (0xFFFFul << GPIO_IEN_IR_EN_Pos)            /*!< GPIO IEN: IR_EN Mask */

#define GPIO_IEN_IF_EN_Pos          0                                           /*!< GPIO IEN: IF_EN Position */
#define GPIO_IEN_IF_EN_Msk          (0xFFFFul << GPIO_IEN_IF_EN_Pos)            /*!< GPIO IEN: IF_EN Mask */

/* GPIO ISRC Bit Field Definitions */
#define GPIO_ISRC_ISRC_Pos          0                                           /*!< GPIO ISRC: ISRC Position */
#define GPIO_ISRC_ISRC_Msk          (0xFFFFul << GPIO_ISRC_ISRC_Pos)            /*!< GPIO ISRC: ISRC Mask */

/* GPIO DBNCECON Bit Field Definitions */
#define GPIO_DBNCECON_ICLK_ON_Pos   5                                           /*!< GPIO DBNCECON: ICLK_ON  Position */
#define GPIO_DBNCECON_ICLK_ON_Msk   (1ul << GPIO_DBNCECON_ICLK_ON_Pos)          /*!< GPIO DBNCECON: ICLK_ON  Mask */

#define GPIO_DBNCECON_DBCLKSRC_Pos  4                                           /*!< GPIO DBNCECON: DBCLKSRC Position */
#define GPIO_DBNCECON_DBCLKSRC_Msk  (1ul << GPIO_DBNCECON_DBCLKSRC_Pos)         /*!< GPIO DBNCECON: DBCLKSRC Mask */

#define GPIO_DBNCECON_DBCLKSEL_Pos  0                                           /*!< GPIO DBNCECON: DBCLKSEL Position */
#define GPIO_DBNCECON_DBCLKSEL_Msk  (0xFul << GPIO_DBNCECON_DBCLKSEL_Pos)       /*!< GPIO DBNCECON: DBCLKSEL Mask */
/*@}*/ /* end of group REG_GPIO_BITMASK */
/*@}*/ /* end of group REG_GPIO */

/*------------------------------ I2C Controller ------------------------------*/
/** @addtogroup REG_I2C I2C
  Memory Mapped Structure for I2C Serial Interface Controller
  @{
 */
typedef struct
{
    /**
     * I2CON
     * ===================================================================================================
     * Offset: 0x00  I2C Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2]     |AA        |Assert Acknowledge Control
     * |        |          |When AA =1 prior to address or data received, an acknowledged (low level to I2Cn_SDA) will be returned during the acknowledge clock pulse on the I2Cn_SCL line when 1.) A slave is acknowledging the address sent from master, 2.) The receiver devices are acknowledging the data sent by transmitter.
     * |        |          |When AA=0 prior to address or data received, a Not acknowledged (high level to I2Cn_SDA) will be returned during the acknowledge clock pulse on the I2Cn_SCL line.
     * |[3]     |SI        |I2C Interrupt Flag
     * |        |          |When a new I2C state is present in the I2CSTATUS register, the SI flag is set by hardware, and if bit EI (I2CON [7]) is set, the I2C interrupt is requested.
     * |        |          |SI must be cleared by software.
     * |        |          |Clear SI by writing 1 to this bit.
     * |[4]     |STO       |I2C STOP Control
     * |        |          |In Master mode, setting STO to transmit a STOP condition to bus then I2C hardware will check the bus condition if a STOP condition is detected this bit will be cleared by hardware automatically.
     * |        |          |In a slave mode, setting STO resets I2C hardware to the defined "not addressed" slave mode.
     * |        |          |This means it is NO LONGER in the slave receiver mode to receive data from the master transmit device.
     * |[5]     |STA       |I2C START Control
     * |        |          |Setting STA to logic 1 to enter Master mode, the I2C hardware sends a START or repeat START condition to bus when the bus is free.
     * |[6]     |ENS1      |I2C Controller Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Set to enable I2C serial function controller.
     * |        |          |When ENS1=1 the I2C serial function enables.
     * |        |          |The multi-function pin function of I2Cn_SDA and I2Cn_SCL must set to I2C function first.
     * |[7]     |EI        |Enable Interrupt
     * |        |          |0 = I2C interrupt Disabled.
     * |        |          |1 = I2C interrupt Enabled.
     */
    __IO uint32_t I2CON;

    /**
     * I2CADDR0
     * ===================================================================================================
     * Offset: 0x04  I2C Slave Address Register0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |GC        |General Call Function
     * |        |          |0 = General Call Function Disabled.
     * |        |          |1 = General Call Function Enabled.
     * |[7:1]   |I2CADDR   |I2C Address Register
     * |        |          |The content of this register is irrelevant when I2C is in Master mode.
     * |        |          |In the slave mode, the seven most significant bits must be loaded with the chip's own address.
     * |        |          |The I2C hardware will react if either of the address is matched.
     */
    __IO uint32_t I2CADDR0;

    /**
     * I2CDAT
     * ===================================================================================================
     * Offset: 0x08  I2C Data Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |I2CDAT    |I2C Data Register
     * |        |          |Bit [7:0] is located with the 8-bit transferred data of I2C serial port.
     */
    __IO uint32_t I2CDAT;

    /**
     * I2CSTATUS
     * ===================================================================================================
     * Offset: 0x0C  I2C Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |I2CSTATUS |I2C Status Register
     * |        |          |The status register of I2C:
     * |        |          |The three least significant bits are always 0.
     * |        |          |The five most significant bits contain the status code.
     * |        |          |There are 26 possible status codes.
     * |        |          |When I2CSTATUS contains F8H, no serial interrupt is requested.
     * |        |          |All other I2CSTATUS values correspond to defined I2C states.
     * |        |          |When each of these states is entered, a status interrupt is requested (SI = 1).
     * |        |          |A valid status code is present in I2CSTATUS one cycle after SI is set by hardware and is still present one cycle after SI has been reset by software.
     * |        |          |In addition, states 00H stands for a Bus Error.
     * |        |          |A Bus Error occurs when a START or STOP condition is present at an illegal position in the formation frame.
     * |        |          |Example of illegal position are during the serial transfer of an address byte, a data byte or an acknowledge bit.
     */
    __I  uint32_t I2CSTATUS;

    /**
     * I2CLK
     * ===================================================================================================
     * Offset: 0x10  I2C Clock Divided Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |I2CLK     |I2C clock divided Register
     * |        |          |The I2C clock rate bits: Data Baud Rate of I2C = (system clock) / (4x (I2CLK+1)).
     * |        |          |Note: The minimum value of I2CLK is 4.
     */
    __IO uint32_t I2CLK;

    /**
     * I2CTOC
     * ===================================================================================================
     * Offset: 0x14  I2C Time-out Counter Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |TIF       |Time-out Flag
     * |        |          |This bit is set by H/W when I2C time-out happened and it can interrupt CPU if I2C interrupt enable bit (EI) is set to 1.
     * |        |          |Note: Write 1 to clear this bit.
     * |[1]     |DIV4      |Time-out Counter Input Clock Divided by 4
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |When Enabled, The time-out period is extend 4 times.
     * |[2]     |ENTI      |Time-out Counter Enable/Disable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |When Enabled, the 14-bit time-out counter will start counting when SI is clear.
     * |        |          |Setting flag SI to high will reset counter and re-start up counting after SI is cleared.
     */
    __IO uint32_t I2CTOC;

    /**
     * I2CADDR1
     * ===================================================================================================
     * Offset: 0x18  I2C Slave Address Register1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |GC        |General Call Function
     * |        |          |0 = General Call Function Disabled.
     * |        |          |1 = General Call Function Enabled.
     * |[7:1]   |I2CADDR   |I2C Address Register
     * |        |          |The content of this register is irrelevant when I2C is in Master mode.
     * |        |          |In the slave mode, the seven most significant bits must be loaded with the chip's own address.
     * |        |          |The I2C hardware will react if either of the address is matched.
     */
    __IO uint32_t I2CADDR1;

    /**
     * I2CADDR2
     * ===================================================================================================
     * Offset: 0x1C  I2C Slave Address Register2
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |GC        |General Call Function
     * |        |          |0 = General Call Function Disabled.
     * |        |          |1 = General Call Function Enabled.
     * |[7:1]   |I2CADDR   |I2C Address Register
     * |        |          |The content of this register is irrelevant when I2C is in Master mode.
     * |        |          |In the slave mode, the seven most significant bits must be loaded with the chip's own address.
     * |        |          |The I2C hardware will react if either of the address is matched.
     */
    __IO uint32_t I2CADDR2;

    /**
     * I2CADDR3
     * ===================================================================================================
     * Offset: 0x20  I2C Slave Address Register3
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |GC        |General Call Function
     * |        |          |0 = General Call Function Disabled.
     * |        |          |1 = General Call Function Enabled.
     * |[7:1]   |I2CADDR   |I2C Address Register
     * |        |          |The content of this register is irrelevant when I2C is in Master mode.
     * |        |          |In the slave mode, the seven most significant bits must be loaded with the chip's own address.
     * |        |          |The I2C hardware will react if either of the address is matched.
     */
    __IO uint32_t I2CADDR3;

    /**
     * I2CADM0
     * ===================================================================================================
     * Offset: 0x24  I2C Slave Address Mask Register0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:1]   |I2CADM    |I2C Address Mask Register
     * |        |          |0 = Mask Disabled (the received corresponding register bit should be exact the same as address register.).
     * |        |          |1 = Mask Enabled (the received corresponding address bit is don't care.).
     * |        |          |I2C bus controllers support multiple address recognition with four address mask register.
     * |        |          |When the bit in the address mask register is set to one, it means the received corresponding address bit is don't-care.
     * |        |          |If the bit is set to zero, that means the received corresponding register bit should be exact the same as address register.
     */
    __IO uint32_t I2CADM0;

    /**
     * I2CADM1
     * ===================================================================================================
     * Offset: 0x28  I2C Slave Address Mask Register1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:1]   |I2CADM    |I2C Address Mask Register
     * |        |          |0 = Mask Disabled (the received corresponding register bit should be exact the same as address register.).
     * |        |          |1 = Mask Enabled (the received corresponding address bit is don't care.).
     * |        |          |I2C bus controllers support multiple address recognition with four address mask register.
     * |        |          |When the bit in the address mask register is set to one, it means the received corresponding address bit is don't-care.
     * |        |          |If the bit is set to zero, that means the received corresponding register bit should be exact the same as address register.
     */
    __IO uint32_t I2CADM1;

    /**
     * I2CADM2
     * ===================================================================================================
     * Offset: 0x2C  I2C Slave Address Mask Register2
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:1]   |I2CADM    |I2C Address Mask Register
     * |        |          |0 = Mask Disabled (the received corresponding register bit should be exact the same as address register.).
     * |        |          |1 = Mask Enabled (the received corresponding address bit is don't care.).
     * |        |          |I2C bus controllers support multiple address recognition with four address mask register.
     * |        |          |When the bit in the address mask register is set to one, it means the received corresponding address bit is don't-care.
     * |        |          |If the bit is set to zero, that means the received corresponding register bit should be exact the same as address register.
     */
    __IO uint32_t I2CADM2;

    /**
     * I2CADM3
     * ===================================================================================================
     * Offset: 0x30  I2C Slave Address Mask Register3
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:1]   |I2CADM    |I2C Address Mask Register
     * |        |          |0 = Mask Disabled (the received corresponding register bit should be exact the same as address register.).
     * |        |          |1 = Mask Enabled (the received corresponding address bit is don't care.).
     * |        |          |I2C bus controllers support multiple address recognition with four address mask register.
     * |        |          |When the bit in the address mask register is set to one, it means the received corresponding address bit is don't-care.
     * |        |          |If the bit is set to zero, that means the received corresponding register bit should be exact the same as address register.
     */
    __IO uint32_t I2CADM3;

    /**
     * @cond HIDDEN_SYMBOLS
     * RESERVED0
     * ===================================================================================================
     *
     * ---------------------------------------------------------------------------------------------------
     */
    uint32_t RESERVED0[2];
    /**
     * @endcond
     */

    /**
     * I2CWKUPCON
     * ===================================================================================================
     * Offset: 0x3C  I2C Wake-up Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |WKUPEN    |I2C Wake-up Enable
     * |        |          |0 = I2C wake-up function Disabled.
     * |        |          |1= I2C wake-up function Enabled.
     */
    __IO uint32_t I2CWKUPCON;

    /**
     * I2CWKUPSTS
     * ===================================================================================================
     * Offset: 0x40  I2C Wake-up Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |WKUPIF    |I2C Wake-up Flag
     * |        |          |When chip is woken up from Power-down mode by I2C, this bit is set to 1.
     * |        |          |Software can write 1 to clear this bit.
     */
    __IO uint32_t I2CWKUPSTS;
} I2C_T;

/** @addtogroup REG_I2C_BITMASK I2C Bit Mask
  @{
 */

/* I2C I2CON Bit Field Definitions */
#define I2C_I2CON_EI_Pos                        7                                       /*!< I2C I2CON: EI Position */
#define I2C_I2CON_EI_Msk                        (1ul << I2C_I2CON_EI_Pos)               /*!< I2C I2CON: EI Mask */

#define I2C_I2CON_ENS1_Pos                      6                                       /*!< I2C I2CON: ENS1 Position */
#define I2C_I2CON_ENS1_Msk                      (1ul << I2C_I2CON_ENS1_Pos)             /*!< I2C I2CON: ENS1 Mask */

#define I2C_I2CON_STA_Pos                       5                                       /*!< I2C I2CON: STA Position */
#define I2C_I2CON_STA_Msk                       (1ul << I2C_I2CON_STA_Pos)              /*!< I2C I2CON: STA Mask */

#define I2C_I2CON_STO_Pos                       4                                       /*!< I2C I2CON: STO Position */
#define I2C_I2CON_STO_Msk                       (1ul << I2C_I2CON_STO_Pos)              /*!< I2C I2CON: STO Mask */

#define I2C_I2CON_SI_Pos                        3                                       /*!< I2C I2CON: SI Position */
#define I2C_I2CON_SI_Msk                        (1ul << I2C_I2CON_SI_Pos)               /*!< I2C I2CON: SI Mask */

#define I2C_I2CON_AA_Pos                        2                                       /*!< I2C I2CON: AA Position */
#define I2C_I2CON_AA_Msk                        (1ul << I2C_I2CON_AA_Pos)               /*!< I2C I2CON: AA Mask */

/* I2C I2CADDR Bit Field Definitions */
#define I2C_I2CADDR_I2CADDR_Pos                 1                                       /*!< I2C I2CADDR: I2CADDR Position */
#define I2C_I2CADDR_I2CADDR_Msk                 (0x7Ful << I2C_I2CADDR_I2CADDR_Pos)     /*!< I2C I2CADDR: I2CADDR Mask */

#define I2C_I2CADDR_GC_Pos                      0                                       /*!< I2C I2CADDR: GC Position */
#define I2C_I2CADDR_GC_Msk                      (1ul << I2C_I2CADDR_GC_Pos)             /*!< I2C I2CADDR: GC Mask */

/* I2C I2CDAT Bit Field Definitions */
#define I2C_I2CDAT_I2CDAT_Pos                   0                                       /*!< I2C I2CDAT: I2CDAT Position */
#define I2C_I2CDAT_I2CDAT_Msk                   (0xFFul << I2C_I2CDAT_I2CDAT_Pos)       /*!< I2C I2CDAT: I2CDAT Mask */

/* I2C I2CSTATUS Bit Field Definitions */
#define I2C_I2CSTATUS_I2CSTATUS_Pos             0                                       /*!< I2C I2CSTATUS: I2CSTATUS Position */
#define I2C_I2CSTATUS_I2CSTATUS_Msk             (0xFFul << I2C_I2CSTATUS_I2CSTATUS_Pos) /*!< I2C I2CSTATUS: I2CSTATUS Mask */

/* I2C I2CLK Bit Field Definitions */
#define I2C_I2CLK_I2CLK_Pos                     0                                       /*!< I2C I2CLK: I2CLK Position */
#define I2C_I2CLK_I2CLK_Msk                     (0xFFul << I2C_I2CLK_I2CLK_Pos)         /*!< I2C I2CLK: I2CLK Mask */

/* I2C I2CTOC Bit Field Definitions */
#define I2C_I2CTOC_ENTI_Pos                     2                                       /*!< I2C I2CTOC: ENTI Position */
#define I2C_I2CTOC_ENTI_Msk                     (1ul << I2C_I2CTOC_ENTI_Pos)            /*!< I2C I2CTOC: ENTI Mask */

#define I2C_I2CTOC_DIV4_Pos                     1                                       /*!< I2C I2CTOC: DIV4 Position */
#define I2C_I2CTOC_DIV4_Msk                     (1ul << I2C_I2CTOC_DIV4_Pos)            /*!< I2C I2CTOC: DIV4 Mask */

#define I2C_I2CTOC_TIF_Pos                      0                                       /*!< I2C I2CTOC: TIF Position */
#define I2C_I2CTOC_TIF_Msk                      (1ul << I2C_I2CTOC_TIF_Pos)             /*!< I2C I2CTOC: TIF Mask */

/* I2C I2CADM Bit Field Definitions */
#define I2C_I2CADM_I2CADM_Pos                   1                                       /*!< I2C I2CADM: I2CADM Position */
#define I2C_I2CADM_I2CADM_Msk                   (0x7Ful << I2C_I2CADM_I2CADM_Pos)       /*!< I2C I2CADM: I2CADM Mask */

/* I2C I2CWKUPCON Bit Field Definitions */
#define I2C_I2CWKUPCON_WKUPEN_Pos               0                                       /*!< I2C I2CWKUPCON: WKUPEN Position */
#define I2C_I2CWKUPCON_WKUPEN_Msk               (1ul << I2C_I2CWKUPCON_WKUPEN_Pos)      /*!< I2C I2CWKUPCON: WKUPEN Mask */

/* I2C I2CWKUPSTS Bit Field Definitions */
#define I2C_I2CWKUPSTS_WKUPIF_Pos               0                                       /*!< I2C I2CWKUPSTS: WKUPIF Position */
#define I2C_I2CWKUPSTS_WKUPIF_Msk               (1ul << I2C_I2CWKUPSTS_WKUPIF_Pos)      /*!< I2C I2CWKUPSTS: WKUPIF Mask */
/*@}*/ /* end of group REG_I2C_BITMASK */
/*@}*/ /* end of group REG_I2C */

/*----------------------------- I2S Controller -------------------------------*/
/** @addtogroup REG_I2S I2S
  Memory Mapped Structure for I2S Interface Controller
  @{
 */
typedef struct
{
    /**
     * I2SCON
     * ===================================================================================================
     * Offset: 0x00  I2S Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |I2SEN     |I2S Controller Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[1]     |TXEN      |Transmit Enable
     * |        |          |0 = Data transmit Disabled.
     * |        |          |1 = Data transmit Enabled.
     * |[2]     |RXEN      |Receive Enable
     * |        |          |0 = Data receiving Disabled.
     * |        |          |1 = Data receiving Enabled.
     * |[3]     |MUTE      |Transmit Mute Enable
     * |        |          |0 = Transmit data is shifted from buffer.
     * |        |          |1 = Send zero on transmit channel.
     * |[5:4]   |WORDWIDTH |Word Width
     * |        |          |00 = data is 8-bit word.
     * |        |          |01 = data is 16-bit word.
     * |        |          |10 = data is 24-bit word.
     * |        |          |11 = data is 32-bit word.
     * |[6]     |MONO      |Monaural Data
     * |        |          |0 = Data is stereo format.
     * |        |          |1 = Data is monaural format.
     * |[7]     |FORMAT    |Data Format
     * |        |          |0 = I2S data format.
     * |        |          |1 = MSB justified data format.
     * |[8]     |SLAVE     |Slave Mode
     * |        |          |I2S can operate as master or slave.
     * |        |          |For Master mode, I2SBCLK and I2SLRCLK pins are output mode and send bit clock from NuMicro NUC123 series to Audio CODEC chip.
     * |        |          |In Slave mode, I2SBCLK and I2SLRCLK pins are input mode and I2SBCLK and I2SLRCLK signals are received from outer Audio CODEC chip.
     * |        |          |0 = Master mode.
     * |        |          |1 = Slave mode.
     * |[11:9]  |TXTH      |Transmit FIFO Threshold Level
     * |        |          |If the count of remaining data word (32 bits) in transmit FIFO is equal to or less than threshold level then TXTHF (I2SSTATUS[18]) is set.
     * |        |          |000 = 0 word data in transmit FIFO.
     * |        |          |001 = 1 word data in transmit FIFO.
     * |        |          |010 = 2 words data in transmit FIFO.
     * |        |          |011 = 3 words data in transmit FIFO.
     * |        |          |100 = 4 words data in transmit FIFO.
     * |        |          |101 = 5 words data in transmit FIFO.
     * |        |          |110 = 6 words data in transmit FIFO.
     * |        |          |111 = 7 words data in transmit FIFO.
     * |[14:12] |RXTH      |Receive FIFO Threshold Level
     * |        |          |When the count of received data word(s) in buffer is equal to or higher than threshold level, RXTHF (I2SSTATUS[10]) will be set.
     * |        |          |000 = 1 word data in receive FIFO.
     * |        |          |001 = 2 word data in receive FIFO.
     * |        |          |010 = 3 word data in receive FIFO.
     * |        |          |011 = 4 word data in receive FIFO.
     * |        |          |100 = 5 word data in receive FIFO.
     * |        |          |101 = 6 word data in receive FIFO.
     * |        |          |110 = 7 word data in receive FIFO.
     * |        |          |111 = 8 word data in receive FIFO.
     * |[15]    |MCLKEN    |Master Clock Enable
     * |        |          |If MCLKEN is set to 1, I2S controller will generate master clock on I2S_MCLK pin for external audio devices.
     * |        |          |0 = Master clock Disabled.
     * |        |          |1 = Master clock Enabled.
     * |[16]    |RCHZCEN   |Right Channel Zero Cross Detection Enable
     * |        |          |If this bit is set to 1, when right channel data sign bit change or next shift data bits are all 0 then RZCF flag in I2SSTATUS register is set to 1.
     * |        |          |This function is only available in transmit operation.
     * |        |          |0 = Right channel zero cross detection Disabled.
     * |        |          |1 = Right channel zero cross detection Enabled.
     * |[17]    |LCHZCEN   |Left Channel Zero Cross Detection Enable
     * |        |          |If this bit is set to 1, when left channel data sign bit changes or next shift data bits are all 0 then LZCF flag in I2SSTATUS register is set to 1.
     * |        |          |This function is only available in transmit operation.
     * |        |          |0 = Left channel zero cross detection Disabled.
     * |        |          |1 = Left channel zero cross detection Enabled.
     * |[18]    |CLR_TXFIFO|Clear Transmit FIFO
     * |        |          |Write 1 to clear transmit FIFO, internal pointer is reset to FIFO start point, and TX_LEVEL[3:0] returns to 0 and
     * |        |          |transmit FIFO becomes empty but data in transmit FIFO is not changed.
     * |        |          |This bit is cleared by hardware automatically. Returns 0 on read.
     * |[19]    |CLR_RXFIFO|Clear Receive FIFO
     * |        |          |Write 1 to clear receive FIFO, internal pointer is reset to FIFO start point, and RX_LEVEL[3:0] returns 0 and receive FIFO becomes empty.
     * |        |          |This bit is cleared by hardware automatically. Returns 0 on read.
     * |[20]    |TXDMA     |Enable Transmit DMA
     * |        |          |When TX DMA is enabled, I2S request DMA to transfer data from SRAM to transmit FIFO if FIFO is not full.
     * |        |          |0 = TX DMA Disabled.
     * |        |          |1 = TX DMA Enabled.
     * |[21]    |RXDMA     |Enable Receive DMA
     * |        |          |When RX DMA is enabled, I2S requests DMA to transfer data from receive FIFO to SRAM if FIFO is not empty.
     * |        |          |0 = RX DMA Disabled.
     * |        |          |1 = RX DMA Enabled.
     * |[23]    |RXLCH     |Receive Left Channel Enable
     * |        |          |When monaural format is selected (MONO = 1), I2S controller will receive right channel data if RXLCH is set to 0,
     * |        |          |and receive left channel data if RXLCH is set to 1.
     * |        |          |0 = Receive right channel data in Mono mode.
     * |        |          |1 = Receive left channel data in Mono mode.
     */
    __IO uint32_t CON;

    /**
     * I2SCLKDIV
     * ===================================================================================================
     * Offset: 0x04  I2S Clock Divider Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |MCLK_DIV  |Master Clock Divider
     * |        |          |If MCLKEN is set to 1, I2S controller will generate master clock for external audio devices.
     * |        |          |The master clock rate, F_MCLK, is determined by the following expressions.
     * |        |          |If MCLK_DIV >= 1, F_MCLK = F_I2SCLK/(2x(MCLK_DIV)).
     * |        |          |If MCLK_DIV = 0, F_MCLK = F_I2SCLK.
     * |        |          |F_I2SCLK is the frequency of I2S peripheral clock.
     * |        |          |In general, the master clock rate is 256 times sampling clock rate.
     * |[15:8]  |BCLK_DIV  |Bit Clock Divider
     * |        |          |The I2S controller will generate bit clock in Master mode.
     * |        |          |The bit clock rate, F_BCLK, is determined by the following expression.
     * |        |          |F_BCLK = F_I2SCLK /(2x(BCLK_DIV + 1)) , where F_I2SCLK is the frequency of I2S peripheral clock.
     */
    __IO uint32_t CLKDIV;

    /**
     * I2SIE
     * ===================================================================================================
     * Offset: 0x08  I2S Interrupt Enable Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |RXUDFIE   |Receive FIFO Underflow Interrupt Enable
     * |        |          |0 = Interrupt Disabled.
     * |        |          |1 = Interrupt Enabled.
     * |[1]     |RXOVFIE   |Receive FIFO Overflow Interrupt Enable
     * |        |          |0 = Interrupt Disabled.
     * |        |          |1 = Interrupt Enabled.
     * |[2]     |RXTHIE    |Receive FIFO Threshold Level Interrupt Enable
     * |        |          |When the count of data words in receive FIFO is equal to or higher than RXTH (I2SCON[14:12]) and
     * |        |          |this bit is set to 1, receive FIFO threshold level interrupt will be asserted.
     * |        |          |0 = Interrupt Disabled.
     * |        |          |1 = Interrupt Enabled.
     * |[8]     |TXUDFIE   |Transmit FIFO Underflow Interrupt Enable
     * |        |          |Interrupt occurs if this bit is set to 1 and the transmit FIFO underflow flag is set to 1.
     * |        |          |0 = Interrupt Disabled.
     * |        |          |1 = Interrupt Enabled.
     * |[9]     |TXOVFIE   |Transmit FIFO Overflow Interrupt Enable
     * |        |          |Interrupt occurs if this bit is set to 1 and the transmit FIFO overflow flag is set to 1
     * |        |          |0 = Interrupt Disabled.
     * |        |          |1 = Interrupt Enabled.
     * |[10]    |TXTHIE    |Transmit FIFO Threshold Level Interrupt Enable
     * |        |          |Interrupt occurs if this bit is set to 1 and the count of data words in transmit FIFO is less than TXTH (I2SCON[11:9]).
     * |        |          |0 = Interrupt Disabled.
     * |        |          |1 = Interrupt Enabled.
     * |[11]    |RZCIE     |Right Channel Zero-Cross Interrupt Enable
     * |        |          |Interrupt occurs if this bit is set to 1 and right channel zero-cross event is detected.
     * |        |          |0 = Interrupt Disabled.
     * |        |          |1 = Interrupt Enabled.
     * |[12]    |LZCIE     |Left Channel Zero-Cross Interrupt Enable
     * |        |          |Interrupt occurs if this bit is set to 1 and left channel zero-cross event is detected.
     * |        |          |0 = Interrupt Disabled.
     * |        |          |1 = Interrupt Enabled.
     */
    __IO uint32_t IE;

    /**
     * I2SSTATUS
     * ===================================================================================================
     * Offset: 0x0C  I2S Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |I2SINT    |I2S Interrupt Flag
     * |        |          |This bit is wire-OR of I2STXINT and I2SRXINT bits.
     * |        |          |0 = No I2S interrupt.
     * |        |          |1 = I2S interrupt.
     * |        |          |Note: This bit is read only.
     * |[1]     |I2SRXINT  |I2S Receive Interrupt
     * |        |          |0 = No receive interrupt.
     * |        |          |1 = Receive interrupt.
     * |        |          |Note: This bit is read only.
     * |[2]     |I2STXINT  |I2S Transmit Interrupt
     * |        |          |0 = No transmit interrupt.
     * |        |          |1 = Transmit interrupt.
     * |        |          |Note: This bit is read only.
     * |[3]     |RIGHT     |Right Channel
     * |        |          |This bit indicates current transmit data is belong to which channel
     * |        |          |0 = Left channel.
     * |        |          |1 = Right channel.
     * |        |          |Note: This bit is read only.
     * |[8]     |RXUDF     |Receive FIFO Underflow Flag
     * |        |          |Underflow event will occur if read the empty receive FIFO.
     * |        |          |0 = No underflow event occurred.
     * |        |          |1 = Underflow.
     * |        |          |Note: Write 1 to clear this bit to 0.
     * |[9]     |RXOVF     |Receive FIFO Overflow Flag
     * |        |          |When receive FIFO is full and hardware attempt to write data to receive FIFO, this bit will be set to 1, data in 1st buffer will be overwrote.
     * |        |          |0 = No overflow.
     * |        |          |1 = Overflow.
     * |        |          |Note: Write 1 to clear this bit to 0.
     * |[10]    |RXTHF     |Receive FIFO Threshold Flag
     * |        |          |When data word(s) in receive FIFO is equal to or larger than threshold value set in RXTH (I2SCON[14:12]).
     * |        |          |The RXTHF bit becomes to 1.
     * |        |          |It keeps at 1 till RX_LEVEL (I2SSTATUS[27:24]) is less than RXTH.
     * |        |          |0 = Data word(s) in FIFO is less than threshold level.
     * |        |          |1 = Data word(s) in FIFO is equal to or larger than threshold level.
     * |        |          |Note: This bit is read only.
     * |[11]    |RXFULL    |Receive FIFO Full
     * |        |          |This bit reflects the count of data in receive FIFO is 8
     * |        |          |0 = Not full.
     * |        |          |1 = Full.
     * |        |          |Note: This bit is read only.
     * |[12]    |RXEMPTY   |Receive FIFO Empty
     * |        |          |This bit reflects the count of data in receive FIFO is 0
     * |        |          |0 = Not empty.
     * |        |          |1 = Empty.
     * |        |          |Note: This bit is read only.
     * |[16]    |TXUDF     |Transmit FIFO Underflow Flag
     * |        |          |If transmit FIFO is empty and hardware reads data from transmit FIFO. This bit will be set to 1.
     * |        |          |0 = No underflow.
     * |        |          |1 = Underflow.
     * |        |          |Note: Software can write 1 to clear this bit to 0.
     * |[17]    |TXOVF     |Transmit FIFO Overflow Flag
     * |        |          |This bit will be set to 1 if writes data to transmit FIFO when transmit FIFO is full.
     * |        |          |0 = No overflow.
     * |        |          |1 = Overflow.
     * |        |          |Note: Write 1 to clear this bit to 0.
     * |[18]    |TXTHF     |Transmit FIFO Threshold Flag
     * |        |          |When the count of data stored in transmit-FIFO is equal to or less than threshold value set in TXTH (I2SCON[11:9]).
     * |        |          |The TXTHF bit becomes to 1.
     * |        |          |It keeps at 1 till TX_LEVEL (I2SSTATUS[31:28]) is larger than TXTH.
     * |        |          |0 = Data word(s) in FIFO is larger than threshold level.
     * |        |          |1 = Data word(s) in FIFO is equal to or less than threshold level.
     * |        |          |Note: This bit is read only.
     * |[19]    |TXFULL    |Transmit FIFO Full
     * |        |          |This bit reflects data word number in transmit FIFO is 8
     * |        |          |0 = Not full.
     * |        |          |1 = Full.
     * |        |          |Note: This bit is read only.
     * |[20]    |TXEMPTY   |Transmit FIFO Empty
     * |        |          |This bit reflects data word number in transmit FIFO is 0
     * |        |          |0 = Not empty.
     * |        |          |1 = Empty.
     * |        |          |Note: This bit is read only.
     * |[21]    |TXBUSY    |Transmit Busy
     * |        |          |This bit is cleared to 0 when all data in transmit FIFO and shift buffer is shifted out.
     * |        |          |And set to 1 when 1st data is load to shift buffer.
     * |        |          |0 = Transmit shift buffer is empty.
     * |        |          |1 = Transmit shift buffer is not empty.
     * |        |          |Note: This bit is read only.
     * |[22]    |RZCF      |Right Channel Zero-Cross Flag
     * |        |          |It indicates the sign bit of right channel sample data is changed or all data bits are 0.
     * |        |          |0 = No zero-cross.
     * |        |          |1 = Right channel zero-cross event is detected.
     * |        |          |Note: Write 1 to clear this bit to 0.
     * |[23]    |LZCF      |Left Channel Zero-Cross Flag
     * |        |          |It indicates the sign bit of left channel sample data is changed or all data bits are 0.
     * |        |          |0 = No zero-cross.
     * |        |          |1 = Left channel zero-cross event is detected.
     * |        |          |Note: Write 1 to clear this bit to 0.
     * |[27:24] |RX_LEVEL  |Receive FIFO Level
     * |        |          |These bits indicate word number in receive FIFO
     * |        |          |0000 = No data.
     * |        |          |0001 = 1 word in receive FIFO.
     * |        |          |....
     * |        |          |1000 = 8 words in receive FIFO.
     * |[31:28] |TX_LEVEL  |Transmit FIFO Level
     * |        |          |These bits indicate word number in transmit FIFO
     * |        |          |0000 = No data.
     * |        |          |0001 = 1 word in transmit FIFO.
     * |        |          |....
     * |        |          |1000 = 8 words in transmit FIFO.
     */
    __IO uint32_t STATUS;

    /**
     * I2STXFIFO
     * ===================================================================================================
     * Offset: 0x10  I2S Transmit FIFO Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |TXFIFO    |Transmit FIFO Register
     * |        |          |I2S contains 8 words (8x32 bits) data buffer for data transmit.
     * |        |          |Write data to this register to prepare data for transmission.
     * |        |          |The remaining word number is indicated by TX_LEVEL (I2SSTATUS[31:28]).
     */
    __O  uint32_t TXFIFO;

    /**
     * I2SRXFIFO
     * ===================================================================================================
     * Offset: 0x14  I2S Receive FIFO Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |RXFIFO    |Receive FIFO Register
     * |        |          |I2S contains 8 words (8x32 bits) data buffer for data receive.
     * |        |          |Read this register to get data of receive FIFO.
     * |        |          |The remaining data word number is indicated by RX_LEVEL (I2SSTATUS[27:24]).
     */
    __I  uint32_t RXFIFO;
} I2S_T;

/** @addtogroup REG_I2S_BITMASK I2S Bit Mask
  @{
 */

/* I2S I2SCON Bit Field Definitions */
#define I2S_CON_PCM_Pos                      24                                   /*!< I2S I2SCON: PCM Position           */
#define I2S_CON_PCM_Msk                      (1ul << I2S_CON_PCM_Pos)             /*!< I2S I2SCON: PCM Mask               */

#define I2S_CON_RXLCH_Pos                    23                                   /*!< I2S I2SCON: RXLCH Position         */
#define I2S_CON_RXLCH_Msk                    (1ul << I2S_CON_RXLCH_Pos)           /*!< I2S I2SCON: RXLCH Mask             */

#define I2S_CON_RXDMA_Pos                    21                                   /*!< I2S I2SCON: RXDMA Position         */
#define I2S_CON_RXDMA_Msk                    (1ul << I2S_CON_RXDMA_Pos)           /*!< I2S I2SCON: RXDMA Mask             */

#define I2S_CON_TXDMA_Pos                    20                                   /*!< I2S I2SCON: TXDMA Position         */
#define I2S_CON_TXDMA_Msk                    (1ul << I2S_CON_TXDMA_Pos)           /*!< I2S I2SCON: TXDMA Mask             */

#define I2S_CON_CLR_RXFIFO_Pos               19                                   /*!< I2S I2SCON: CLR_RXFIFO Position    */
#define I2S_CON_CLR_RXFIFO_Msk               (1ul << I2S_CON_CLR_RXFIFO_Pos)      /*!< I2S I2SCON: CLR_RXFIFO Mask        */

#define I2S_CON_CLR_TXFIFO_Pos               18                                   /*!< I2S I2SCON: CLR_TXFIFO Position    */
#define I2S_CON_CLR_TXFIFO_Msk               (1ul << I2S_CON_CLR_TXFIFO_Pos)      /*!< I2S I2SCON: CLR_TXFIFO Mask        */

#define I2S_CON_LCHZCEN_Pos                  17                                   /*!< I2S I2SCON: LCHZCEN Position       */
#define I2S_CON_LCHZCEN_Msk                  (1ul << I2S_CON_LCHZCEN_Pos)         /*!< I2S I2SCON: LCHZCEN Mask           */

#define I2S_CON_RCHZCEN_Pos                  16                                   /*!< I2S I2SCON: RCHZCEN Position       */
#define I2S_CON_RCHZCEN_Msk                  (1ul << I2S_CON_RCHZCEN_Pos)         /*!< I2S I2SCON: RCHZCEN Mask           */

#define I2S_CON_MCLKEN_Pos                   15                                   /*!< I2S I2SCON: MCLKEN Position        */
#define I2S_CON_MCLKEN_Msk                   (1ul << I2S_CON_MCLKEN_Pos)          /*!< I2S I2SCON: MCLKEN Mask            */

#define I2S_CON_RXTH_Pos                     12                                   /*!< I2S I2SCON: RXTH Position          */
#define I2S_CON_RXTH_Msk                     (7ul << I2S_CON_RXTH_Pos)            /*!< I2S I2SCON: RXTH Mask              */

#define I2S_CON_TXTH_Pos                     9                                    /*!< I2S I2SCON: TXTH Position          */
#define I2S_CON_TXTH_Msk                     (7ul << I2S_CON_TXTH_Pos)            /*!< I2S I2SCON: TXTH Mask              */

#define I2S_CON_SLAVE_Pos                    8                                    /*!< I2S I2SCON: SLAVE Position         */
#define I2S_CON_SLAVE_Msk                    (1ul << I2S_CON_SLAVE_Pos)           /*!< I2S I2SCON: SLAVE Mask             */

#define I2S_CON_FORMAT_Pos                   7                                    /*!< I2S I2SCON: FORMAT Position        */
#define I2S_CON_FORMAT_Msk                   (1ul << I2S_CON_FORMAT_Pos)          /*!< I2S I2SCON: FORMAT Mask            */

#define I2S_CON_MONO_Pos                     6                                    /*!< I2S I2SCON: MONO Position          */
#define I2S_CON_MONO_Msk                     (1ul << I2S_CON_MONO_Pos)            /*!< I2S I2SCON: MONO Mask              */

#define I2S_CON_WORDWIDTH_Pos                4                                    /*!< I2S I2SCON: WORDWIDTH Position     */
#define I2S_CON_WORDWIDTH_Msk                (3ul << I2S_CON_WORDWIDTH_Pos)       /*!< I2S I2SCON: WORDWIDTH Mask         */

#define I2S_CON_MUTE_Pos                     3                                    /*!< I2S I2SCON: MUTE Position          */
#define I2S_CON_MUTE_Msk                     (1ul << I2S_CON_MUTE_Pos)            /*!< I2S I2SCON: MUTE Mask              */

#define I2S_CON_RXEN_Pos                     2                                    /*!< I2S I2SCON: RXEN Position          */
#define I2S_CON_RXEN_Msk                     (1ul << I2S_CON_RXEN_Pos)            /*!< I2S I2SCON: RXEN Mask              */

#define I2S_CON_TXEN_Pos                     1                                    /*!< I2S I2SCON: TXEN Position          */
#define I2S_CON_TXEN_Msk                     (1ul << I2S_CON_TXEN_Pos)            /*!< I2S I2SCON: TXEN Mask              */

#define I2S_CON_I2SEN_Pos                    0                                    /*!< I2S I2SCON: I2SEN Position         */
#define I2S_CON_I2SEN_Msk                    (1ul << I2S_CON_I2SEN_Pos)           /*!< I2S I2SCON: I2SEN Mask             */

/* I2S I2SCLKDIV Bit Field Definitions */
#define I2S_CLKDIV_BCLK_DIV_Pos              8                                    /*!< I2S I2SCLKDIV: BCLK_DIV Position   */
#define I2S_CLKDIV_BCLK_DIV_Msk              (0xFFul << I2S_CLKDIV_BCLK_DIV_Pos)  /*!< I2S I2SCLKDIV: BCLK_DIV Mask       */

#define I2S_CLKDIV_MCLK_DIV_Pos              0                                    /*!< I2S I2SCLKDIV: MCLK_DIV Position   */
#define I2S_CLKDIV_MCLK_DIV_Msk              (7ul << I2S_CLKDIV_MCLK_DIV_Pos)     /*!< I2S I2SCLKDIV: MCLK_DIV Mask       */

/* I2S I2SIE Bit Field Definitions */
#define I2S_IE_LZCIE_Pos                     12                                   /*!< I2S I2SIE: LZCIE Position          */
#define I2S_IE_LZCIE_Msk                     (1ul << I2S_IE_LZCIE_Pos)            /*!< I2S I2SIE: LZCIE Mask              */

#define I2S_IE_RZCIE_Pos                     11                                   /*!< I2S I2SIE: RZCIE Position          */
#define I2S_IE_RZCIE_Msk                     (1ul << I2S_IE_RZCIE_Pos)            /*!< I2S I2SIE: RZCIE Mask              */

#define I2S_IE_TXTHIE_Pos                    10                                   /*!< I2S I2SIE: TXTHIE Position         */
#define I2S_IE_TXTHIE_Msk                    (1ul << I2S_IE_TXTHIE_Pos)           /*!< I2S I2SIE: TXTHIE Mask             */

#define I2S_IE_TXOVFIE_Pos                   9                                    /*!< I2S I2SIE: TXOVFIE Position        */
#define I2S_IE_TXOVFIE_Msk                   (1ul << I2S_IE_TXOVFIE_Pos)          /*!< I2S I2SIE: TXOVFIE Mask            */

#define I2S_IE_TXUDFIE_Pos                   8                                    /*!< I2S I2SIE: TXUDFIE Position        */
#define I2S_IE_TXUDFIE_Msk                   (1ul << I2S_IE_TXUDFIE_Pos)          /*!< I2S I2SIE: TXUDFIE Mask            */

#define I2S_IE_RXTHIE_Pos                    2                                    /*!< I2S I2SIE: RXTHIE Position         */
#define I2S_IE_RXTHIE_Msk                    (1ul << I2S_IE_RXTHIE_Pos)           /*!< I2S I2SIE: RXTHIE Mask             */

#define I2S_IE_RXOVFIE_Pos                   1                                    /*!< I2S I2SIE: RXOVFIE Position        */
#define I2S_IE_RXOVFIE_Msk                   (1ul << I2S_IE_RXOVFIE_Pos)          /*!< I2S I2SIE: RXOVFIE Mask            */

#define I2S_IE_RXUDFIE_Pos                   0                                    /*!< I2S I2SIE: RXUDFIE Position        */
#define I2S_IE_RXUDFIE_Msk                   (1ul << I2S_IE_RXUDFIE_Pos)          /*!< I2S I2SIE: RXUDFIE Mask            */


/* I2S I2SSTATUS Bit Field Definitions */
#define I2S_STATUS_TX_LEVEL_Pos              28                                   /*!< I2S I2SSTATUS: TX_LEVEL Position   */
#define I2S_STATUS_TX_LEVEL_Msk              (0xFul << I2S_STATUS_TX_LEVEL_Pos)   /*!< I2S I2SSTATUS: TX_LEVEL Mask       */

#define I2S_STATUS_RX_LEVEL_Pos              24                                   /*!< I2S I2SSTATUS: RX_LEVEL Position   */
#define I2S_STATUS_RX_LEVEL_Msk              (0xFul << I2S_STATUS_RX_LEVEL_Pos)   /*!< I2S I2SSTATUS: RX_LEVEL Mask       */

#define I2S_STATUS_LZCF_Pos                  23                                   /*!< I2S I2SSTATUS: LZCF Position       */
#define I2S_STATUS_LZCF_Msk                  (1ul << I2S_STATUS_LZCF_Pos)         /*!< I2S I2SSTATUS: LZCF Mask           */

#define I2S_STATUS_RZCF_Pos                  22                                   /*!< I2S I2SSTATUS: RZCF Position       */
#define I2S_STATUS_RZCF_Msk                  (1ul << I2S_STATUS_RZCF_Pos)         /*!< I2S I2SSTATUS: RZCF Mask           */

#define I2S_STATUS_TXBUSY_Pos                21                                   /*!< I2S I2SSTATUS: TXBUSY Position     */
#define I2S_STATUS_TXBUSY_Msk                (1ul << I2S_STATUS_TXBUSY_Pos)       /*!< I2S I2SSTATUS: TXBUSY Mask         */

#define I2S_STATUS_TXEMPTY_Pos               20                                   /*!< I2S I2SSTATUS: TXEMPTY Position    */
#define I2S_STATUS_TXEMPTY_Msk               (1ul << I2S_STATUS_TXEMPTY_Pos)      /*!< I2S I2SSTATUS: TXEMPTY Mask        */

#define I2S_STATUS_TXFULL_Pos                19                                   /*!< I2S I2SSTATUS: TXFULL Position     */
#define I2S_STATUS_TXFULL_Msk                (1ul << I2S_STATUS_TXFULL_Pos)       /*!< I2S I2SSTATUS: TXFULL Mask         */

#define I2S_STATUS_TXTHF_Pos                 18                                   /*!< I2S I2SSTATUS: TXTHF Position      */
#define I2S_STATUS_TXTHF_Msk                 (1ul << I2S_STATUS_TXTHF_Pos)        /*!< I2S I2SSTATUS: TXTHF Mask          */

#define I2S_STATUS_TXOVF_Pos                 17                                   /*!< I2S I2SSTATUS: TXOVF Position      */
#define I2S_STATUS_TXOVF_Msk                 (1ul << I2S_STATUS_TXOVF_Pos)        /*!< I2S I2SSTATUS: TXOVF Mask          */

#define I2S_STATUS_TXUDF_Pos                 16                                   /*!< I2S I2SSTATUS: TXUDF Position      */
#define I2S_STATUS_TXUDF_Msk                 (1ul << I2S_STATUS_TXUDF_Pos)        /*!< I2S I2SSTATUS: TXUDF Mask          */

#define I2S_STATUS_RXEMPTY_Pos               12                                   /*!< I2S I2SSTATUS: RXEMPTY Position    */
#define I2S_STATUS_RXEMPTY_Msk               (1ul << I2S_STATUS_RXEMPTY_Pos)      /*!< I2S I2SSTATUS: RXEMPTY Mask        */

#define I2S_STATUS_RXFULL_Pos                11                                   /*!< I2S I2SSTATUS: RXFULL Position     */
#define I2S_STATUS_RXFULL_Msk                (1ul << I2S_STATUS_RXFULL_Pos)       /*!< I2S I2SSTATUS: RXFULL Mask         */

#define I2S_STATUS_RXTHF_Pos                 10                                   /*!< I2S I2SSTATUS: RXTHF Position      */
#define I2S_STATUS_RXTHF_Msk                 (1ul << I2S_STATUS_RXTHF_Pos)        /*!< I2S I2SSTATUS: RXTHF Mask          */

#define I2S_STATUS_RXOVF_Pos                 9                                    /*!< I2S I2SSTATUS: RXOVF Position      */
#define I2S_STATUS_RXOVF_Msk                 (1ul << I2S_STATUS_RXOVF_Pos)        /*!< I2S I2SSTATUS: RXOVF Mask          */

#define I2S_STATUS_RXUDF_Pos                 8                                    /*!< I2S I2SSTATUS: RXUDF Position      */
#define I2S_STATUS_RXUDF_Msk                 (1ul << I2S_STATUS_RXUDF_Pos)        /*!< I2S I2SSTATUS: RXUDF Mask          */

#define I2S_STATUS_RIGHT_Pos                 3                                    /*!< I2S I2SSTATUS: RIGHT Position      */
#define I2S_STATUS_RIGHT_Msk                 (1ul << I2S_STATUS_RIGHT_Pos)        /*!< I2S I2SSTATUS: RIGHT Mask          */

#define I2S_STATUS_I2STXINT_Pos              2                                    /*!< I2S I2SSTATUS: I2STXINT Position   */
#define I2S_STATUS_I2STXINT_Msk              (1ul << I2S_STATUS_I2STXINT_Pos)     /*!< I2S I2SSTATUS: I2STXINT Mask       */

#define I2S_STATUS_I2SRXINT_Pos              1                                    /*!< I2S I2SSTATUS: I2SRXINT Position   */
#define I2S_STATUS_I2SRXINT_Msk              (1ul << I2S_STATUS_I2SRXINT_Pos)     /*!< I2S I2SSTATUS: I2SRXINT Mask       */

#define I2S_STATUS_I2SINT_Pos                0                                    /*!< I2S I2SSTATUS: I2SINT Position     */
#define I2S_STATUS_I2SINT_Msk                (1ul << I2S_STATUS_I2SINT_Pos)       /*!< I2S I2SSTATUS: I2SINT Mask         */
/*@}*/ /* end of group REG_I2S_BITMASK */
/*@}*/ /* end of group REG_I2S */

/*------------------------------ DMA Controller -----------------------------*/
/** @addtogroup REG_PDMA PDMA
  Memory Mapped Structure for PDMA Controller
  @{
 */
typedef struct
{
    /**
     * PDMA_CSRx
     * ===================================================================================================
     * Offset: 0x00  PDMA Channel x Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |PDMACEN   |PDMA Channel Enable
     * |        |          |Setting this bit to 1 enables PDMA operation.
     * |        |          |If this bit is cleared, PDMA will ignore all PDMA request and force Bus Master into IDLE state.
     * |        |          |Note: SW_RST(PDMA_CSRx[1], x= 0~8) will clear this bit.
     * |[1]     |SW_RST    |Software Engine Reset
     * |        |          |0 = No effect.
     * |        |          |1 = Reset the internal state machine, pointers and internal buffer.
     * |        |          |The contents of control register will not be cleared.
     * |        |          |This bit will be automatically cleared after few clock cycles.
     * |[3:2]   |MODE_SEL  |PDMA Mode Selection
     * |        |          |00 = Memory to Memory mode (Memory-to-Memory).
     * |        |          |01 = Peripheral to Memory mode (Peripheral-to-Memory).
     * |        |          |10 = Memory to Peripheral mode (Memory-to-Peripheral).
     * |[5:4]   |SAD_SEL   |Transfer Source Address Direction Selection
     * |        |          |00 = Transfer source address is increasing successively.
     * |        |          |01 = Reserved.
     * |        |          |10 = Transfer source address is fixed (This feature can be used when data where transferred from a single source to multiple destinations).
     * |        |          |11 = Reserved.
     * |[7:6]   |DAD_SEL   |Transfer Destination Address Direction Selection
     * |        |          |00 = Transfer destination address is increasing successively.
     * |        |          |01 = Reserved.
     * |        |          |10 = Transfer destination address is fixed.
     * |        |          |(This feature can be used when data where transferred from multiple sources to a single destination).
     * |        |          |11 = Reserved.
     * |[20:19] |APB_TWS   |Peripheral Transfer Width Selection
     * |        |          |00 = One word (32-bit) is transferred for every PDMA operation.
     * |        |          |01 = One byte (8-bit) is transferred for every PDMA operation.
     * |        |          |10 = One half-word (16-bit) is transferred for every PDMA operation.
     * |        |          |11 = Reserved.
     * |        |          |Note: This field is meaningful only when MODE_SEL (PDMA_CSRx[3:2]) is Peripheral to Memory mode (Peripheral-to-Memory) or Memory to Peripheral mode (Memory-to-Peripheral).
     * |[23]    |TRIG_EN   |Trigger Enable
     * |        |          |0 = No effect.
     * |        |          |1 = PDMA data read or write transfer Enabled.
     * |        |          |Note: When PDMA transfer completed, this bit will be cleared automatically.
     * |        |          |If the bus error occurs, all PDMA transfer will be stopped.
     * |        |          |Software must reset all PDMA channel, and then trigger again.
     */
    __IO uint32_t CSR;
    /**
     * PDMA_SARx
     * ===================================================================================================
     * Offset: 0x04  PDMA Channel x Source Address Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |PDMA_SAR  |PDMA Transfer Source Address Register
     * |        |          |This field indicates a 32-bit source address of PDMA.
     * |        |          |Note: The source address must be word alignment.
     */
    __IO uint32_t SAR;
    /**
     * PDMA_DARx
     * ===================================================================================================
     * Offset: 0x08  PDMA Channel x Destination Address Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |PDMA_DAR  |PDMA Transfer Destination Address Register
     * |        |          |This field indicates a 32-bit destination address of PDMA.
     * |        |          |Note: The destination address must be word alignment
     */
    __IO uint32_t DAR;
    /**
     * PDMA_BCRx
     * ===================================================================================================
     * Offset: 0x0C  PDMA Channel x Transfer Byte Count Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |PDMA_BCR  |PDMA Transfer Byte Count Register
     * |        |          |This field indicates a 16-bit transfer byte count number of PDMA; it must be word alignment.
     */
    __IO uint32_t BCR;

    /**
     * PDMA_POINTx
     * ===================================================================================================
     * Offset: 0x10  PDMA Channel x Internal buffer pointer Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |PDMA_POINT|PDMA Internal Buffer Pointer Register (Read Only)
     * |        |          |This field indicates the internal buffer pointer.
     */
    __I  uint32_t POINT;
    /**
     * PDMA_CSARx
     * ===================================================================================================
     * Offset: 0x14  PDMA Channel x Current Source Address Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |PDMA_CSAR |PDMA Current Source Address Register (Read Only)
     * |        |          |This field indicates the source address where the PDMA transfer just occurred.
     */
    __I  uint32_t CSAR;
    /**
     * PDMA_CDARx
     * ===================================================================================================
     * Offset: 0x18  PDMA Channel x Current Destination Address Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |PDMA_CDAR |PDMA Current Destination Address Register (Read Only)
     * |        |          |This field indicates the destination address where the PDMA transfer just occurred.
     */
    __I  uint32_t CDAR;

    /**
     * PDMA_CBCRx
     * ===================================================================================================
     * Offset: 0x1C  PDMA Channel x Current Transfer Byte Count Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |PDMA_CBCR |PDMA Current Byte Count Register (Read Only)
     * |        |          |This field indicates the current remained byte count of PDMA.
     * |        |          |Note: This field value will be cleared to 0, when software set SW_RST (PDMA_CSRx[1]) to "1".
     */
    __I  uint32_t CBCR;
    /**
     * PDMA_IERx
     * ===================================================================================================
     * Offset: 0x20  PDMA Channel x Interrupt Enable Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |TABORT_IE |PDMA Read/Write Target Abort Interrupt Enable
     * |        |          |0 = Target abort interrupt generation Disabled during PDMA transfer.
     * |        |          |1 = Target abort interrupt generation Enabled during PDMA transfer.
     * |[1]     |BLKD_IE   |PDMA Block Transfer Done Interrupt Enable
     * |        |          |0 = Interrupt generator Disabled when PDMA transfer is done.
     * |        |          |1 = Interrupt generator Enabled when PDMA transfer is done.
     */
    __IO uint32_t IER;
    /**
     * PDMA_ISRx
     * ===================================================================================================
     * Offset: 0x24  PDMA Channel x Interrupt Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |TABORT_IF |PDMA Read/Write Target Abort Interrupt Flag
     * |        |          |Write 1 to clear this bit to 0.
     * |        |          |0 = No bus ERROR response received.
     * |        |          |1 = Bus ERROR response received.
     * |        |          |Note: This bit filed indicates bus master received ERROR response or not.
     * |        |          |If bus master received ERROR response, it means that target abort is happened.
     * |        |          |PDMA controller will stop transfer and respond this event to software then goes to IDLE state.
     * |        |          |When target abort occurred, software must reset PDMA, and then transfer those data again.
     * |[1]     |BLKD_IF   |PDMA Block Transfer Done Interrupt Flag
     * |        |          |This bit indicates that PDMA has finished all transfers.
     * |        |          |0 = Not finished.
     * |        |          |1 = Done.
     * |        |          |Write 1 to clear this bit to 0.
     */
    __IO uint32_t ISR;

    /**
     * @cond HIDDEN_SYMBOLS
     */
    __I  uint32_t RESERVE[22];
    /**
     * @endcond
     */

    /**
     * PDMA_SBUF0_Cx
     * ===================================================================================================
     * Offset: 0x80  PDMA Channel x Shared Buffer FIFO x Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |PDMA_SBUF0|PDMA Shared Buffer FIFO 0 (Read Only)
     * |        |          |Each channel has its own 1 word internal buffer.
     */
    __I  uint32_t SBUF;
} PDMA_T;

typedef struct
{
    /**
     * PDMA_GCRCSR
     * ===================================================================================================
     * Offset: 0x00  PDMA Global Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[8]     |CLK0_EN   |PDMA Controller Channel 0 Clock Enable Control
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[9]     |CLK1_EN   |PDMA Controller Channel 1 Clock Enable Control
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[10]    |CLK2_EN   |PDMA Controller Channel 2 Clock Enable Control
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[11]    |CLK3_EN   |PDMA Controller Channel 3 Clock Enable Control
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[12]    |CLK4_EN   |PDMA Controller Channel 4 Clock Enable Control
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[13]    |CLK5_EN   |PDMA Controller Channel 5 Clock Enable Control
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[24]    |CRC_CLK_EN|CRC Controller Clock Enable Control
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     */
    __IO uint32_t GCRCSR;
    /**
     * PDMA_PDSSR0
     * ===================================================================================================
     * Offset: 0x04  PDMA Service Selection Control Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |SPI0_RXSEL|PDMA SPI0 RX Selection
     * |        |          |This field defines which PDMA channel is connected to the on-chip peripheral SPI0 RX.
     * |        |          |Software can change the channel RX setting by this field.
     * |        |          |For example, SPI0_RXSEL (PDMA_PDSSR0[3:0]) = 0100, that means SPI0_RX is connected to PDMA_CH4.
     * |        |          |0000: CH0
     * |        |          |0001: CH1
     * |        |          |0010: CH2
     * |        |          |0011: CH3
     * |        |          |0100: CH4
     * |        |          |0101: CH5
     * |        |          |Others : Reserved
     * |[7:4]   |SPI0_TXSEL|PDMA SPI0 TX Selection
     * |        |          |This field defines which PDMA channel is connected to the on-chip peripheral SPI0 TX.
     * |        |          |Software can configure the TX channel setting by this field.
     * |        |          |The channel configuration is the same as SPI0_RXSEL (PDMA_PDSSR0[3:0]) field.
     * |        |          |Please refer to the explanation of SPI0_RXSEL (PDMA_PDSSR0[3:0]).
     * |[11:8]  |SPI1_RXSEL|PDMA SPI1 RX Selection
     * |        |          |This field defines which PDMA channel is connected to the on-chip peripheral SPI1 RX.
     * |        |          |Software can configure the RX channel setting by this field.
     * |        |          |The channel configuration is the same as SPI0_RXSEL (PDMA_PDSSR0[3:0]) field.
     * |        |          |Please refer to the explanation of SPI0_RXSEL (PDMA_PDSSR0[3:0]).
     * |[15:12] |SPI1_TXSEL|PDMA SPI1 TX Selection
     * |        |          |This field defines which PDMA channel is connected to the on-chip peripheral SPI1 TX.
     * |        |          |Software can configure the TX channel setting by this field.
     * |        |          |The channel configuration is the same as SPI0_RXSEL (PDMA_PDSSR0[3:0]) field.
     * |        |          |Please refer to the explanation of SPI0_RXSEL (PDMA_PDSSR0[3:0]).
     * |[19:16] |SPI2_RXSEL|PDMA SPI2 RX Selection
     * |        |          |This field defines which PDMA channel is connected to the on-chip peripheral SPI2 RX.
     * |        |          |Software can configure the RX channel setting by this field.
     * |        |          |The channel configuration is the same as SPI0_RXSEL (PDMA_PDSSR0[3:0]) field.
     * |        |          |Please refer to the explanation of SPI0_RXSEL (PDMA_PDSSR0[3:0]).
     * |[23:20] |SPI2_TXSEL|PDMA SPI2 TX Selection
     * |        |          |This field defines which PDMA channel is connected to the on-chip peripheral SPI2 TX.
     * |        |          |Software can configure the TX channel setting by this field.
     * |        |          |The channel configuration is the same as SPI0_RXSEL (PDMA_PDSSR0[3:0]) field.
     * |        |          |Please refer to the explanation of SPI0_RXSEL (PDMA_PDSSR0[3:0]).
     */
    __IO uint32_t PDSSR0;
    /**
     * PDMA_PDSSR1
     * ===================================================================================================
     * Offset: 0x08  PDMA Service Selection Control Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |UART0_RXSEL|PDMA UART0 RX Selection
     * |        |          |This field defines which PDMA channel is connected to the on-chip peripheral UART0 RX.
     * |        |          |Software can change the channel RX setting by this field.
     * |        |          |For example, UART0_RXSEL (PDMA_PDSSR1[3:0]) = 0100, which means UART0_RX is connected to PDMA_CH4.
     * |        |          |0000: CH0
     * |        |          |0001: CH1
     * |        |          |0010: CH2
     * |        |          |0011: CH3
     * |        |          |0100: CH4
     * |        |          |0101: CH5
     * |        |          |Others : Reserved
     * |[7:4]   |UART0_TXSEL|PDMA UART0 TX Selection
     * |        |          |This field defines which PDMA channel is connected to the on-chip peripheral UART0 TX.
     * |        |          |Software can configure the TX channel setting by this field.
     * |        |          |The channel configuration is the same as UART0_RXSEL (PDMA_PDSSR1[3:0]) field.
     * |        |          |Please refer to the explanation of UART0_RXSEL (PDMA_PDSSR1[3:0]).
     * |[11:8]  |UART1_RXSEL|PDMA UART1 RX Selection
     * |        |          |This field defines which PDMA channel is connected to the on-chip peripheral UART1 RX.
     * |        |          |Software can configure the RX channel setting by this field.
     * |        |          |The channel configuration is the same as UART0_RXSEL (PDMA_PDSSR1[3:0]) field.
     * |        |          |Please refer to the explanation of UART0_RXSEL (PDMA_PDSSR1[3:0]).
     * |[15:12] |UART1_TXSEL|PDMA UART1 TX Selection
     * |        |          |This field defines which PDMA channel is connected to the on-chip peripheral UART1 TX.
     * |        |          |Software can configure the TX channel setting by this field.
     * |        |          |The channel configuration is the same as UART0_RXSEL (PDMA_PDSSR1[3:0]) field.
     * |        |          |Please refer to the explanation of UART0_RXSEL (PDMA_PDSSR1[3:0]).
     * |[27:24] |ADC_RXSEL |PDMA ADC RX Selection
     * |        |          |This field defines which PDMA channel is connected to the on-chip peripheral ADC RX.
     * |        |          |Software can configure the RX channel setting by this field.
     * |        |          |The channel configuration is the same as UART0_RXSEL (PDMA_PDSSR1[3:0]) field.
     * |        |          |Please refer to the explanation of UART0_RXSEL (PDMA_PDSSR1[3:0]).
     */
    __IO uint32_t PDSSR1;
    /**
     * PDMA_GCRISR
     * ===================================================================================================
     * Offset: 0x0C  PDMA Global Interrupt Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |INTR0     |Interrupt Status Of Channel 0
     * |        |          |This bit is the interrupt status of PDMA channel0.
     * |        |          |Note: This bit is read only
     * |[1]     |INTR1     |Interrupt Status Of Channel 1
     * |        |          |This bit is the interrupt status of PDMA channel1.
     * |        |          |Note: This bit is read only
     * |[2]     |INTR2     |Interrupt Status Of Channel 2
     * |        |          |This bit is the interrupt status of PDMA channel2.
     * |        |          |Note: This bit is read only
     * |[3]     |INTR3     |Interrupt Status Of Channel 3
     * |        |          |This bit is the interrupt status of PDMA channel3.
     * |        |          |Note: This bit is read only
     * |[4]     |INTR4     |Interrupt Status Of Channel 4
     * |        |          |This bit is the interrupt status of PDMA channel4.
     * |        |          |Note: This bit is read only
     * |[5]     |INTR5     |Interrupt Status Of Channel 5
     * |        |          |This bit is the interrupt status of PDMA channel5.
     * |        |          |Note: This bit is read only
     * |[16]    |INTRCRC   |Interrupt Status Of CRC Controller
     * |        |          |This bit is the interrupt status of CRC controller
     * |        |          |Note: This bit is read only
     * |[31]    |INTR      |Interrupt Status
     * |        |          |This bit is the interrupt status of PDMA controller.
     * |        |          |Note: This bit is read only
     */
    __IO uint32_t GCRISR;
    /**
     * PDMA_PDSSR2
     * ===================================================================================================
     * Offset: 0x10  PDMA Service Selection Control Register 2
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |I2S_RXSEL |PDMA I2S RX Selection
     * |        |          |This field defines which PDMA channel is connected to the on-chip peripheral I2S RX.
     * |        |          |Software can change the channel RX setting by this field.
     * |        |          |For example: I2S_RXSEL (PDMA_PDSSR2[3:0]) = 0100, that means I2S_RX is connected to PDMA_CH4.
     * |        |          |0000: CH0
     * |        |          |0001: CH1
     * |        |          |0010: CH2
     * |        |          |0011: CH3
     * |        |          |0100: CH4
     * |        |          |0101: CH5
     * |        |          |Others : Reserved
     * |[7:4]   |I2S_TXSEL |PDMA I2S TX Selection
     * |        |          |This field defines which PDMA channel is connected to the on-chip peripheral I2S TX.
     * |        |          |Software can configure the TX channel setting by this field.
     * |        |          |The channel configuration is the same as I2S_RXSEL (PDMA_PDSSR2[3:0]) field.
     * |        |          |Please refer to the explanation of I2S_RXSEL (PDMA_PDSSR2[3:0]).
     * |[11:8]  |PWM0_RXSEL|PDMA PWM0 RX Selection
     * |        |          |This filed defines which PDMA channel is connected to the on-chip peripheral PWM0 RX.
     * |        |          |Software can configure the RX channel setting by PWM0_RXSEL.
     * |        |          |The channel configuration is the same as I2S_RXSEL (PDMA_PDSSR0[3:0]) field.
     * |        |          |Please refer to the explanation of I2S_RXSEL (PDMA_PDSSR2[3:0]).
     * |[15:12] |PWM1_RXSEL|PDMA PWM1 RX Selection
     * |        |          |This filed defines which PDMA channel is connected to the on-chip peripheral PWM1 RX.
     * |        |          |Software can configure the RX channel setting by PWM1_RXSEL.
     * |        |          |The channel configuration is the same as I2S_RXSEL (PDMA_PDSSR2[3:0]) field.
     * |        |          |Please refer to the explanation of I2S_RXSEL (PDMA_PDSSR2[3:0]).
     * |[19:16] |PWM2_RXSEL|PDMA PWM2 RX Selection
     * |        |          |This field defines which PDMA channel is connected to the on-chip peripheral PWM2 RX.
     * |        |          |Software can configure the RX channel setting by PWM2_RXSEL.
     * |        |          |The channel configuration is the same as I2S_RXSEL (PDMA_PDSSR2[3:0]) field.
     * |        |          |Please refer to the explanation of I2S_RXSEL (PDMA_PDSSR2[3:0]).
     * |[23:20] |PWM3_RXSEL|PDMA PWM3 RX Selection
     * |        |          |This field defines which PDMA channel is connected to the on-chip peripheral PWM3 RX.
     * |        |          |Software can configure the RX channel setting by PWM3_RXSEL.
     * |        |          |The channel configuration is the same as I2S_RXSEL (PDMA_PDSSR2[3:0]) field.
     * |        |          |Please refer to the explanation of I2S_RXSEL (PDMA_PDSSR2[3:0]).
     */
    __IO uint32_t PDSSR2;
} PDMA_GCR_T;


/** @addtogroup REG_PDMA_BITMASK PDMA Bit Mask
  @{
 */

/* PDMA CSR Bit Field Definitions */
#define PDMA_CSR_TRIG_EN_Pos                        23                              /*!< PDMA CSR: TRIG_EN Position */
#define PDMA_CSR_TRIG_EN_Msk                        (1ul << PDMA_CSR_TRIG_EN_Pos)   /*!< PDMA CSR: TRIG_EN Mask */

#define PDMA_CSR_APB_TWS_Pos                        19                              /*!< PDMA CSR: APB_TWS Position */
#define PDMA_CSR_APB_TWS_Msk                        (3ul << PDMA_CSR_APB_TWS_Pos)   /*!< PDMA CSR: APB_TWS Mask */

#define PDMA_CSR_DAD_SEL_Pos                        6                               /*!< PDMA CSR: DAD_SEL Position */
#define PDMA_CSR_DAD_SEL_Msk                        (3ul << PDMA_CSR_DAD_SEL_Pos)   /*!< PDMA CSR: DAD_SEL Mask */

#define PDMA_CSR_SAD_SEL_Pos                        4                               /*!< PDMA CSR: SAD_SEL Position */
#define PDMA_CSR_SAD_SEL_Msk                        (3ul << PDMA_CSR_SAD_SEL_Pos)   /*!< PDMA CSR: SAD_SEL Mask */

#define PDMA_CSR_MODE_SEL_Pos                       2                               /*!< PDMA CSR: MODE_SEL Position */
#define PDMA_CSR_MODE_SEL_Msk                       (3ul << PDMA_CSR_MODE_SEL_Pos)  /*!< PDMA CSR: MODE_SEL Mask */

#define PDMA_CSR_SW_RST_Pos                         1                               /*!< PDMA CSR: SW_RST Position */
#define PDMA_CSR_SW_RST_Msk                         (1ul << PDMA_CSR_SW_RST_Pos)    /*!< PDMA CSR: SW_RST Mask */

#define PDMA_CSR_PDMACEN_Pos                        0                               /*!< PDMA CSR: PDMACEN Position */
#define PDMA_CSR_PDMACEN_Msk                        (1ul << PDMA_CSR_PDMACEN_Pos)   /*!< PDMA CSR: PDMACEN Mask */

/* PDMA BCR Bit Field Definitions */
#define PDMA_BCR_BCR_Pos                            0                               /*!< PDMA BCR: BCR Position */
#define PDMA_BCR_BCR_Msk                            (0xFFFFul << PDMA_BCR_BCR_Pos)  /*!< PDMA BCR: BCR Mask */

/* PDMA POINT Bit Field Definitions */
#define PDMA_POINT_POINT_Pos                        0                               /*!< PDMA POINT: POINT Position */
#define PDMA_POINT_POINT_Msk                        (0xFul << PDMA_POINT_POINT_Pos) /*!< PDMA POINT: POINT Mask */

/* PDMA CBCR Bit Field Definitions */
#define PDMA_CBCR_CBCR_Pos                          0                                   /*!< PDMA CBCR: CBCR Position */
#define PDMA_CBCR_CBCR_Msk                          (0xFFFFul << PDMA_CBCR_CBCR_Pos)    /*!< PDMA CBCR: CBCR Mask */

/* PDMA IER Bit Field Definitions */
#define PDMA_IER_BLKD_IE_Pos                        1                               /*!< PDMA IER: BLKD_IE Position */
#define PDMA_IER_BLKD_IE_Msk                        (1ul << PDMA_IER_BLKD_IE_Pos)   /*!< PDMA IER: BLKD_IE Mask */

#define PDMA_IER_TABORT_IE_Pos                      0                               /*!< PDMA IER: TABORT_IE Position */
#define PDMA_IER_TABORT_IE_Msk                      (1ul << PDMA_IER_TABORT_IE_Pos) /*!< PDMA IER: TABORT_IE Mask */

/* PDMA ISR Bit Field Definitions */
#define PDMA_ISR_BLKD_IF_Pos                        1                               /*!< PDMA ISR: BLKD_IF Position */
#define PDMA_ISR_BLKD_IF_Msk                        (1ul << PDMA_ISR_BLKD_IF_Pos)   /*!< PDMA ISR: BLKD_IF Mask */

#define PDMA_ISR_TABORT_IF_Pos                      0                               /*!< PDMA ISR: TABORT_IF Position */
#define PDMA_ISR_TABORT_IF_Msk                      (1ul << PDMA_ISR_TABORT_IF_Pos) /*!< PDMA ISR: TABORT_IF Mask */

/* PDMA GCRCSR Bit Field Definitions */
#define PDMA_GCRCSR_CRC_CLK_EN_Pos                  24                                  /*!< PDMA GCRCSR: CRC_CLK_EN Position */
#define PDMA_GCRCSR_CRC_CLK_EN_Msk                  (1ul << PDMA_GCRCSR_CRC_CLK_EN_Pos) /*!< PDMA GCRCSR: CRC_CLK_EN Mask */

#define PDMA_GCRCSR_CLK5_EN_Pos                     13                                  /*!< PDMA GCRCSR: CLK5_EN Position */
#define PDMA_GCRCSR_CLK5_EN_Msk                     (1ul << PDMA_GCRCSR_CLK5_EN_Pos)    /*!< PDMA GCRCSR: CLK5_EN Mask */

#define PDMA_GCRCSR_CLK4_EN_Pos                     12                                  /*!< PDMA GCRCSR: CLK4_EN Position */
#define PDMA_GCRCSR_CLK4_EN_Msk                     (1ul << PDMA_GCRCSR_CLK4_EN_Pos)    /*!< PDMA GCRCSR: CLK4_EN Mask */

#define PDMA_GCRCSR_CLK3_EN_Pos                     11                                  /*!< PDMA GCRCSR: CLK3_EN Position */
#define PDMA_GCRCSR_CLK3_EN_Msk                     (1ul << PDMA_GCRCSR_CLK3_EN_Pos)    /*!< PDMA GCRCSR: CLK3_EN Mask */

#define PDMA_GCRCSR_CLK2_EN_Pos                     10                                  /*!< PDMA GCRCSR: CLK2_EN Position */
#define PDMA_GCRCSR_CLK2_EN_Msk                     (1ul << PDMA_GCRCSR_CLK2_EN_Pos)    /*!< PDMA GCRCSR: CLK2_EN Mask */

#define PDMA_GCRCSR_CLK1_EN_Pos                     9                                   /*!< PDMA GCRCSR: CLK1_EN Position */
#define PDMA_GCRCSR_CLK1_EN_Msk                     (1ul << PDMA_GCRCSR_CLK1_EN_Pos)    /*!< PDMA GCRCSR: CLK1_EN Mask */

#define PDMA_GCRCSR_CLK0_EN_Pos                     8                                   /*!< PDMA GCRCSR: CLK0_EN Position */
#define PDMA_GCRCSR_CLK0_EN_Msk                     (1ul << PDMA_GCRCSR_CLK0_EN_Pos)    /*!< PDMA GCRCSR: CLK0_EN Mask */

/* PDMA PDSSR0 Bit Field Definitions */
#define PDMA_PDSSR0_SPI2_TXSEL_Pos                  20                                      /*!< PDMA PDSSR0: SPI2_TXSEL Position */
#define PDMA_PDSSR0_SPI2_TXSEL_Msk                  (0xFul << PDMA_PDSSR0_SPI2_TXSEL_Pos)   /*!< PDMA PDSSR0: SPI2_TXSEL Mask */

#define PDMA_PDSSR0_SPI2_RXSEL_Pos                  16                                      /*!< PDMA PDSSR0: SPI2_RXSEL Position */
#define PDMA_PDSSR0_SPI2_RXSEL_Msk                  (0xFul << PDMA_PDSSR0_SPI2_RXSEL_Pos)   /*!< PDMA PDSSR0: SPI2_RXSEL Mask */

#define PDMA_PDSSR0_SPI1_TXSEL_Pos                  12                                      /*!< PDMA PDSSR0: SPI1_TXSEL Position */
#define PDMA_PDSSR0_SPI1_TXSEL_Msk                  (0xFul << PDMA_PDSSR0_SPI1_TXSEL_Pos)   /*!< PDMA PDSSR0: SPI1_TXSEL Mask */

#define PDMA_PDSSR0_SPI1_RXSEL_Pos                  8                                       /*!< PDMA PDSSR0: SPI1_RXSEL Position */
#define PDMA_PDSSR0_SPI1_RXSEL_Msk                  (0xFul << PDMA_PDSSR0_SPI1_RXSEL_Pos)   /*!< PDMA PDSSR0: SPI1_RXSEL Mask */

#define PDMA_PDSSR0_SPI0_TXSEL_Pos                  4                                       /*!< PDMA PDSSR0: SPI0_TXSEL Position */
#define PDMA_PDSSR0_SPI0_TXSEL_Msk                  (0xFul << PDMA_PDSSR0_SPI0_TXSEL_Pos)   /*!< PDMA PDSSR0: SPI0_TXSEL Mask */

#define PDMA_PDSSR0_SPI0_RXSEL_Pos                  0                                       /*!< PDMA PDSSR0: SPI0_RXSEL Position */
#define PDMA_PDSSR0_SPI0_RXSEL_Msk                  (0xFul << PDMA_PDSSR0_SPI0_RXSEL_Pos)   /*!< PDMA PDSSR0: SPI0_RXSEL Mask */

/* PDMA PDSSR1 Bit Field Definitions */
#define PDMA_PDSSR1_ADC_RXSEL_Pos                   24                                      /*!< PDMA PDSSR1: ADC_RXSEL Position */
#define PDMA_PDSSR1_ADC_RXSEL_Msk                   (0xFul << PDMA_PDSSR1_ADC_RXSEL_Pos)    /*!< PDMA PDSSR1: ADC_RXSEL Mask */

#define PDMA_PDSSR1_UART1_TXSEL_Pos                 12                                      /*!< PDMA PDSSR1: UART1_TXSEL Position */
#define PDMA_PDSSR1_UART1_TXSEL_Msk                 (0xFul << PDMA_PDSSR1_UART1_TXSEL_Pos)  /*!< PDMA PDSSR1: UART1_TXSEL Mask */

#define PDMA_PDSSR1_UART1_RXSEL_Pos                 8                                       /*!< PDMA PDSSR1: UART1_RXSEL Position */
#define PDMA_PDSSR1_UART1_RXSEL_Msk                 (0xFul << PDMA_PDSSR1_UART1_RXSEL_Pos)  /*!< PDMA PDSSR1: UART1_RXSEL Mask */

#define PDMA_PDSSR1_UART0_TXSEL_Pos                 4                                       /*!< PDMA PDSSR1: UART0_TXSEL Position */
#define PDMA_PDSSR1_UART0_TXSEL_Msk                 (0xFul << PDMA_PDSSR1_UART0_TXSEL_Pos)  /*!< PDMA PDSSR1: UART0_TXSEL Mask */

#define PDMA_PDSSR1_UART0_RXSEL_Pos                 0                                       /*!< PDMA PDSSR1: UART0_RXSEL Position */
#define PDMA_PDSSR1_UART0_RXSEL_Msk                 (0xFul << PDMA_PDSSR1_UART0_RXSEL_Pos)  /*!< PDMA PDSSR1: UART0_RXSEL Mask */

/* PDMA GCRISR Bit Field Definitions */
#define PDMA_GCRISR_INTR_Pos                        31                              /*!< PDMA GCRISR: INTR Position */
#define PDMA_GCRISR_INTR_Msk                        (1ul << PDMA_GCRISR_INTR_Pos)   /*!< PDMA GCRISR: INTR Mask */

#define PDMA_GCRISR_INTRCRC_Pos                     16                               /*!< PDMA GCRISR: INTRCRC Position */
#define PDMA_GCRISR_INTRCRC_Msk                     (1ul << PDMA_GCRISR_INTRCRC_Pos) /*!< PDMA GCRISR: INTRCRC Mask */

#define PDMA_GCRISR_INTR5_Pos                       5                               /*!< PDMA GCRISR: INTR5 Position */
#define PDMA_GCRISR_INTR5_Msk                       (1ul << PDMA_GCRISR_INTR5_Pos)  /*!< PDMA GCRISR: INTR5 Mask */

#define PDMA_GCRISR_INTR4_Pos                       4                               /*!< PDMA GCRISR: INTR4 Position */
#define PDMA_GCRISR_INTR4_Msk                       (1ul << PDMA_GCRISR_INTR4_Pos)  /*!< PDMA GCRISR: INTR4 Mask */

#define PDMA_GCRISR_INTR3_Pos                       3                               /*!< PDMA GCRISR: INTR3 Position */
#define PDMA_GCRISR_INTR3_Msk                       (1ul << PDMA_GCRISR_INTR3_Pos)  /*!< PDMA GCRISR: INTR3 Mask */

#define PDMA_GCRISR_INTR2_Pos                       2                               /*!< PDMA GCRISR: INTR2 Position */
#define PDMA_GCRISR_INTR2_Msk                       (1ul << PDMA_GCRISR_INTR2_Pos)  /*!< PDMA GCRISR: INTR2 Mask */

#define PDMA_GCRISR_INTR1_Pos                       1                               /*!< PDMA GCRISR: INTR1 Position */
#define PDMA_GCRISR_INTR1_Msk                       (1ul << PDMA_GCRISR_INTR1_Pos)  /*!< PDMA GCRISR: INTR1 Mask */

#define PDMA_GCRISR_INTR0_Pos                       0                               /*!< PDMA GCRISR: INTR0 Position */
#define PDMA_GCRISR_INTR0_Msk                       (1ul << PDMA_GCRISR_INTR0_Pos)  /*!< PDMA GCRISR: INTR0 Mask */

/* PDMA PDSSR2 Bit Field Definitions */
#define PDMA_PDSSR2_PWM3_RXSEL_Pos                  20                                      /*!< PDMA PDSSR2: PWM3_RXSEL Position */
#define PDMA_PDSSR2_PWM3_RXSEL_Msk                  (0xFul << PDMA_PDSSR2_PWM3_RXSEL_Pos)   /*!< PDMA PDSSR2: PWM3_RXSEL Mask */

#define PDMA_PDSSR2_PWM2_RXSEL_Pos                  16                                      /*!< PDMA PDSSR2: PWM2_RXSEL Position */
#define PDMA_PDSSR2_PWM2_RXSEL_Msk                  (0xFul << PDMA_PDSSR2_PWM2_RXSEL_Pos)   /*!< PDMA PDSSR2: PWM2_RXSEL Mask */

#define PDMA_PDSSR2_PWM1_RXSEL_Pos                  12                                      /*!< PDMA PDSSR2: PWM1_RXSEL Position */
#define PDMA_PDSSR2_PWM1_RXSEL_Msk                  (0xFul << PDMA_PDSSR2_PWM1_RXSEL_Pos)   /*!< PDMA PDSSR2: PWM1_RXSEL Mask */

#define PDMA_PDSSR2_PWM0_RXSEL_Pos                  8                                       /*!< PDMA PDSSR2: PWM0_RXSEL Position */
#define PDMA_PDSSR2_PWM0_RXSEL_Msk                  (0xFul << PDMA_PDSSR2_PWM0_RXSEL_Pos)   /*!< PDMA PDSSR2: PWM0_RXSEL Mask */

#define PDMA_PDSSR2_I2S_TXSEL_Pos                   4                                       /*!< PDMA PDSSR2: I2S_TXSEL Position */
#define PDMA_PDSSR2_I2S_TXSEL_Msk                   (0xFul << PDMA_PDSSR2_I2S_TXSEL_Pos)    /*!< PDMA PDSSR2: I2S_TXSEL Mask */

#define PDMA_PDSSR2_I2S_RXSEL_Pos                   0                                       /*!< PDMA PDSSR2: I2S_RXSEL Position */
#define PDMA_PDSSR2_I2S_RXSEL_Msk                   (0xFul << PDMA_PDSSR2_I2S_RXSEL_Pos)    /*!< PDMA PDSSR2: I2S_RXSEL Mask */
/*@}*/ /* end of group REG_PDMA_BITMASK */
/*@}*/ /* end of group REG_PDMA */


/*------------------------------ PS2 Controller ------------------------------*/
/** @addtogroup REG_PS2 PS2
  Memory Mapped Structure for PS2 Serial Interface Controller
  @{
 */
typedef struct
{
    /**
     * PS2CON
     * ===================================================================================================
     * Offset: 0x00  PS/2 Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |PS2EN     |Enable PS/2 Device
     * |        |          |Enable PS/2 device controller
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[1]     |TXINTEN   |Enable Transmit Interrupt
     * |        |          |0 = Data transmit complete interrupt Disabled.
     * |        |          |1 = Data transmit complete interrupt Enabled.
     * |[2]     |RXINTEN   |Enable Receive Interrupt
     * |        |          |0 = Data receive complete interrupt Disabled.
     * |        |          |1 = Data receive complete interrupt Enabled.
     * |[6:3]   |TXFIFO_DEPTH|Transmit Data FIFO Depth
     * |        |          |There are 16 bytes buffer for data transmit.
     * |        |          |Software can define the FIFO depth from 1 to 16 bytes depends on application needs.
     * |        |          |0 = 1 byte.
     * |        |          |1 = 2 bytes.
     * |        |          |...
     * |        |          |14 = 15 bytes.
     * |        |          |15 = 16 bytes.
     * |[7]     |ACK       |Acknowledge Enable
     * |        |          |0 = Always send acknowledge to host at 12th clock for host to device communication.
     * |        |          |1 = If parity bit error or stop bit is not received correctly, acknowledge bit will not be sent to host at 12th clock.
     * |[8]     |CLRFIFO   |Clear TX FIFO
     * |        |          |Write 1 to this bit to terminate device to host transmission.
     * |        |          |The TXEMPTY(PS2STATUS[7]) bit will be set to 1 and pointer BYTEIDEX(PS2STATUS[11:8]) is reset to 0 regardless there is residue data in buffer or not.
     * |        |          |The buffer content is not been cleared.
     * |        |          |0 = Not active.
     * |        |          |1 = Clear FIFO.
     * |[9]     |OVERRIDE  |Software Override PS/2 CLK/DATA Pin State
     * |        |          |0 = PS2_CLK and PS2_DATA pins are controlled by internal state machine.
     * |        |          |1 = PS2_CLK and PS2_DATA pins are controlled by software.
     * |[10]    |FPS2CLK   |Force PS2CLK Line
     * |        |          |It forces PS2_CLK line high or low regardless of the internal state of the device controller if OVERRIDE(PS2CON[9]) is set to 1.
     * |        |          |0 = Force PS2_CLK line low.
     * |        |          |1 = Force PS2_CLK line high.
     * |[11]    |FPS2DAT   |Force PS2DATA Line
     * |        |          |It forces PS2_DATA high or low regardless of the internal state of the device controller if OVERRIDE (PS2CON[9]) is set to 1.
     * |        |          |0 = Force PS2_DATA low.
     * |        |          |1 = Force PS2_DATA high.
     */
    __IO uint32_t PS2CON;

    /**
     * PS2TXDATA0
     * ===================================================================================================
     * Offset: 0x04  PS/2 Transmit Data Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |PS2TXDATAx|Transmit Data
     * |        |          |Writing data to this register starts in device to host communication if bus is in IDLE state.
     * |        |          |Software must enable PS2EN(PS2CON[0]) before writing data to TX buffer.
     */
    __IO uint32_t PS2TXDATA0;

    /**
     * PS2TXDATA1
     * ===================================================================================================
     * Offset: 0x08  PS/2 Transmit Data Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |PS2TXDATAx|Transmit Data
     * |        |          |Writing data to this register starts in device to host communication if bus is in IDLE state.
     * |        |          |Software must enable PS2EN(PS2CON[0]) before writing data to TX buffer.
     */
    __IO uint32_t PS2TXDATA1;

    /**
     * PS2TXDATA2
     * ===================================================================================================
     * Offset: 0x0C  PS/2 Transmit Data Register 2
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |PS2TXDATAx|Transmit Data
     * |        |          |Writing data to this register starts in device to host communication if bus is in IDLE state.
     * |        |          |Software must enable PS2EN(PS2CON[0]) before writing data to TX buffer.
     */
    __IO uint32_t PS2TXDATA2;

    /**
     * PS2TXDATA3
     * ===================================================================================================
     * Offset: 0x10  PS/2 Transmit Data Register 3
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |PS2TXDATAx|Transmit Data
     * |        |          |Writing data to this register starts in device to host communication if bus is in IDLE state.
     * |        |          |Software must enable PS2EN(PS2CON[0]) before writing data to TX buffer.
     */
    __IO uint32_t PS2TXDATA3;

    /**
     * PS2RXDATA
     * ===================================================================================================
     * Offset: 0x14  PS/2 Receive Data Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |RXDATA    |Received Data
     * |        |          |For host to device communication, after acknowledge bit is sent, the received data is copied from receive shift register to PS2RXDATA register.
     * |        |          |CPU must read this register before next byte reception complete, otherwise the data will be overwritten and RXOVF(PS2STATUS[6]) bit will be set to 1.
     */
    __IO uint32_t PS2RXDATA;

    /**
     * PS2STATUS
     * ===================================================================================================
     * Offset: 0x18  PS/2 Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |PS2CLK    |CLK Pin State
     * |        |          |This bit reflects the status of the PS2_CLK line after synchronizing.
     * |[1]     |PS2DATA   |DATA Pin State
     * |        |          |This bit reflects the status of the PS2_DATA line after synchronizing and sampling.
     * |[2]     |FRAMERR   |Frame Error
     * |        |          |For host to device communication, this bit sets to 1 if STOP bit (logic 1) is not received.
     * |        |          |If frame error occurs, the PS/2_DATA line may keep at low state after 12th clock.
     * |        |          |At this moment, software overrides PS2_CLK to send clock till PS2_DATA release to high state.
     * |        |          |After that, device sends a "Resend" command to host.
     * |        |          |0 = No frame error.
     * |        |          |1 = Frame error occur.
     * |        |          |Write 1 to clear this bit.
     * |[3]     |RXPARITY  |Received Parity
     * |        |          |This bit reflects the parity bit for the last received data byte (odd parity).
     * |        |          |This bit is read only.
     * |[4]     |RXBUSY    |Receive Busy
     * |        |          |This bit indicates that the PS/2 device is currently receiving data.
     * |        |          |0 = Idle.
     * |        |          |1 = Currently receiving data.
     * |        |          |This bit is read only.
     * |[5]     |TXBUSY    |Transmit Busy
     * |        |          |This bit indicates that the PS/2 device is currently sending data.
     * |        |          |0 = Idle.
     * |        |          |1 = Currently sending data.
     * |        |          |This bit is read only.
     * |[6]     |RXOVF     |RX Buffer Overwrite
     * |        |          |0 = No overwrite.
     * |        |          |1 = Data in PS2RXDATA register is overwritten by new received data.
     * |        |          |Write 1 to clear this bit.
     * |[7]     |TXEMPTY   |TX FIFO Empty
     * |        |          |When software writes data to PS2TXDATA0-3, the TXEMPTY bit is cleared to 0 immediately if PS2EN(PS2CON[0]) is enabled.
     * |        |          |When transmitted data byte number is equal to FIFODEPTH(PS2CON[6:3]) then TXEMPTY bit is set to 1.
     * |        |          |0 = There is data to be transmitted.
     * |        |          |1 = FIFO is empty.
     * |        |          |This bit is read only.
     * |[11:8]  |BYTEIDX   |Byte Index
     * |        |          |It indicates which data byte in transmit data shift register.
     * |        |          |When all data in FIFO is transmitted and it will be cleared to 0.
     * |        |          |This bit is read only.
     * |        |          |BYTEIDX,    DATA Transmit , BYTEIDX,    DATA Transmit
     * |        |          |0000   , PS2TXDATA0[ 7: 0], 1000   , PS2TXDATA2[ 7: 0],
     * |        |          |0001   , PS2TXDATA0[15: 8], 1001   , PS2TXDATA2[15: 8],
     * |        |          |0010   , PS2TXDATA0[23:16], 1010   , PS2TXDATA2[23:16],
     * |        |          |0011   , PS2TXDATA0[31:24], 1011   , PS2TXDATA2[31:24],
     * |        |          |0100   , PS2TXDATA1[ 7: 0], 1100   , PS2TXDATA3[ 7: 0],
     * |        |          |0101   , PS2TXDATA1[15: 8], 1101   , PS2TXDATA3[15: 8],
     * |        |          |0110   , PS2TXDATA1[23:16], 1110   , PS2TXDATA3[23:16],
     * |        |          |0111   , PS2TXDATA1[31:24], 1111   , PS2TXDATA3[31:24],
     */
    __IO uint32_t PS2STATUS;

    /**
     * PS2INTID
     * ===================================================================================================
     * Offset: 0x1C  PS/2 Interrupt Identification Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |RXINT     |Receive Interrupt
     * |        |          |This bit is set to 1 when acknowledge bit is sent for Host to device communication.
     * |        |          |Interrupt occurs if RXINTEN(PS2CON[2]) bit is set to 1.
     * |        |          |0 = No interrupt.
     * |        |          |1 = Receive interrupt occurs.
     * |        |          |Write 1 to clear this bit to 0.
     * |[1]     |TXINT     |Transmit Interrupt
     * |        |          |This bit is set to 1 after STOP bit is transmitted.
     * |        |          |Interrupt occur if TXINTEN(PS2CON[1]) bit is set to 1.
     * |        |          |0 = No interrupt.
     * |        |          |1 = Transmit interrupt occurs.
     * |        |          |Write 1 to clear this bit to 0.
     */
    __IO uint32_t PS2INTID;
} PS2_T;

/** @addtogroup REG_PS2_BITMASK PS2 Bit Mask
  @{
 */

/* PS2 PS2CON Bit Field Definitions */
#define PS2_PS2CON_PS2EN_Pos                       0                                        /*!< PS2 PS2CON: PS2EN Position */
#define PS2_PS2CON_PS2EN_Msk                       (1ul << PS2_PS2CON_PS2EN_Pos)            /*!< PS2 PS2CON: PS2EN Mask */

#define PS2_PS2CON_TXINTEN_Pos                     1                                        /*!< PS2 PS2CON: TXINTEN Position */
#define PS2_PS2CON_TXINTEN_Msk                     (1ul << PS2_PS2CON_TXINTEN_Pos)          /*!< PS2 PS2CON: TXINTEN Mask */

#define PS2_PS2CON_RXINTEN_Pos                     2                                        /*!< PS2 PS2CON: RXINTEN Position */
#define PS2_PS2CON_RXINTEN_Msk                     (1ul << PS2_PS2CON_RXINTEN_Pos)          /*!< PS2 PS2CON: RXINTEN Mask */

#define PS2_PS2CON_TXFIFO_DEPTH_Pos                3                                        /*!< PS2 PS2CON: TXFIFO_DEPTH Position */
#define PS2_PS2CON_TXFIFO_DEPTH_Msk                (0xFul << PS2_PS2CON_TXFIFO_DEPTH_Pos)   /*!< PS2 PS2CON: TXFIFO_DEPTH Mask */

#define PS2_PS2CON_ACK_Pos                         7                                        /*!< PS2 PS2CON: ACK Position */
#define PS2_PS2CON_ACK_Msk                         (1ul << PS2_PS2CON_ACK_Pos)              /*!< PS2 PS2CON: ACK Mask */

#define PS2_PS2CON_CLRFIFO_Pos                     8                                        /*!< PS2 PS2CON: CLRFIFO Position */
#define PS2_PS2CON_CLRFIFO_Msk                     (1ul << PS2_PS2CON_CLRFIFO_Pos)          /*!< PS2 PS2CON: CLRFIFO Mask */

#define PS2_PS2CON_OVERRIDE_Pos                    9                                        /*!< PS2 PS2CON: OVERRIDE Position */
#define PS2_PS2CON_OVERRIDE_Msk                    (1ul << PS2_PS2CON_OVERRIDE_Pos)         /*!< PS2 PS2CON: OVERRIDE Mask */

#define PS2_PS2CON_FPS2CLK_Pos                     10                                       /*!< PS2 PS2CON: FPS2CLK Position */
#define PS2_PS2CON_FPS2CLK_Msk                     (1ul << PS2_PS2CON_FPS2CLK_Pos)          /*!< PS2 PS2CON: FPS2CLK Mask */

#define PS2_PS2CON_FPS2DAT_Pos                     11                                       /*!< PS2 PS2CON: FPS2DAT Position */
#define PS2_PS2CON_FPS2DAT_Msk                     (1ul << PS2_PS2CON_FPS2DAT_Pos)          /*!< PS2 PS2CON: FPS2DAT Mask */

/* PS/2 PS2RXDATA Bit Field Definitions */
#define PS2_PS2RXDATA_RXDATA_Pos                   0                                        /*!< PS2 PS2RXDATA: RXDATA Position */
#define PS2_PS2RXDATA_RXDATA_Msk                   (0xFFul << PS2_PS2RXDATA_RXDATA_Pos)     /*!< PS2 PS2RXDATA: RXDATA Mask */

/* PS/2 PS2STATUS Bit Field Definitions */
#define PS2_PS2STATUS_PS2CLK_Pos                   0                                        /*!< PS2 PS2STATUS: PS2CLK Position */
#define PS2_PS2STATUS_PS2CLK_Msk                   (1ul << PS2_PS2STATUS_PS2CLK_Pos)        /*!< PS2 PS2STATUS: PS2CLK Mask */

#define PS2_PS2STATUS_PS2DATA_Pos                  1                                        /*!< PS2 PS2STATUS: PS2DATA Position */
#define PS2_PS2STATUS_PS2DATA_Msk                  (1ul << PS2_PS2STATUS_PS2DATA_Pos)       /*!< PS2 PS2STATUS: PS2DATA Mask */

#define PS2_PS2STATUS_FRAMERR_Pos                  2                                        /*!< PS2 PS2STATUS: FRAMERR Position */
#define PS2_PS2STATUS_FRAMERR_Msk                  (1ul << PS2_PS2STATUS_FRAMERR_Pos)       /*!< PS2 PS2STATUS: FRAMERR Mask */

#define PS2_PS2STATUS_RXPARITY_Pos                 3                                        /*!< PS2 PS2STATUS: RXPARITY Position */
#define PS2_PS2STATUS_RXPARITY_Msk                 (1ul << PS2_PS2STATUS_RXPARITY_Pos)      /*!< PS2 PS2STATUS: RXPARITY Mask */

#define PS2_PS2STATUS_RXBUSY_Pos                   4                                        /*!< PS2 PS2STATUS: RXBUSY Position */
#define PS2_PS2STATUS_RXBUSY_Msk                   (1ul << PS2_PS2STATUS_RXBUSY_Pos)        /*!< PS2 PS2STATUS: RXBUSY Mask */

#define PS2_PS2STATUS_TXBUSY_Pos                   5                                        /*!< PS2 PS2STATUS: TXBUSY Position */
#define PS2_PS2STATUS_TXBUSY_Msk                   (1ul << PS2_PS2STATUS_TXBUSY_Pos)        /*!< PS2 PS2STATUS: TXBUSY Mask */

#define PS2_PS2STATUS_RXOVF_Pos                    6                                        /*!< PS2 PS2STATUS: RXOVF Position */
#define PS2_PS2STATUS_RXOVF_Msk                    (1ul << PS2_PS2STATUS_RXOVF_Pos)         /*!< PS2 PS2STATUS: RXOVF Mask */

#define PS2_PS2STATUS_TXEMPTY_Pos                  7                                        /*!< PS2 PS2STATUS: TXEMPTY Position */
#define PS2_PS2STATUS_TXEMPTY_Msk                  (1ul << PS2_PS2STATUS_TXEMPTY_Pos)       /*!< PS2 PS2STATUS: TXEMPTY Mask */

#define PS2_PS2STATUS_BYTEIDX_Pos                  8                                        /*!< PS2 PS2STATUS: BYTEIDX Position */
#define PS2_PS2STATUS_BYTEIDX_Msk                  (0xFul << PS2_PS2STATUS_BYTEIDX_Pos)     /*!< PS2 PS2STATUS: BYTEIDX Mask */

/* PS/2 PS2INTID Bit Field Definitions */
#define PS2_PS2INTID_RXINT_Pos                     0                                        /*!< PS2 PS2INTID : RXINT Position */
#define PS2_PS2INTID_RXINT_Msk                     (1ul << PS2_PS2INTID_RXINT_Pos)          /*!< PS2 PS2INTID : RXINT Mask */

#define PS2_PS2INTID_TXINT_Pos                     1                                        /*!< PS2 PS2INTID : TXINT Position */
#define PS2_PS2INTID_TXINT_Msk                     (1ul << PS2_PS2INTID_TXINT_Pos)          /*!< PS2 PS2INTID : TXINT Mask */
/*@}*/ /* end of group REG_PS2_BITMASK */
/*@}*/ /* end of group REG_PS2 */

/*----------------------------- PWM Controller -------------------------------*/
/** @addtogroup REG_PWM PWM
  Memory Mapped Structure for PWM Generator and Capture Timer
  @{
 */
typedef struct
{
    /**
     * PPR
     * ===================================================================================================
     * Offset: 0x00  PWM Prescaler Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits     |Field     |Descriptions
     * | :----:  | :----:   | :---- |
     * |[7:0]    |CP01      |Clock Prescaler 0 (PWM-Timer 0 / 1 For Group A)
     * |         |          |Clock input is divided by (CP01 + 1) before it is fed to the corresponding PWM-timer
     * |         |          |If CP01=0, then the clock prescaler 0 output clock will be stopped.
     * |         |          |So corresponding PWM-timer will also be stopped.
     * |[15:8]   |CP23      |Clock Prescaler 2 (PWM-Timer2 / 3 For Group A)
     * |         |          |Clock input is divided by (CP23 + 1) before it is fed to the corresponding PWM-timer
     * |         |          |If CP23=0, then the clock prescaler 2 output clock will be stopped.
     * |         |          |So corresponding PWM-timer will also be stopped.
     * |[23:16]  |DZI01     |Dead-Zone Interval For Pair Of Channel 0 And Channel 1 (PWM0 And PWM1 Pair For PWM Group A)
     * |         |          |These 8-bit determine the Dead-zone length.
     * |         |          |The unit time of Dead-zone length = [(prescale+1)*(clock source divider)]/ PWM01_CLK.
     * |[31:24]  |DZI23     |Dead-Zone Interval For Pair Of Channel2 And Channel3 (PWM2 And PWM3 Pair For PWM Group A)
     * |         |          |These 8-bit determine the Dead-zone length.
     * |         |          |The unit time of Dead-zone length = [(prescale+1)*(clock source divider)]/ PWM23_CLK.
     */
    __IO uint32_t PPR;

    /**
     * CSR
     * ===================================================================================================
     * Offset: 0x04  PWM Clock Source Divider Select Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits      |Field     |Descriptions
     * | :----:   | :----:   | :---- |
     * |[2:0]     |CSR0      |Timer 0 Clock Source Selection(PWM timer 0 for group A)
     * |          |          |Select clock input for timer.
     * |          |          |(Table is the same as CSR3)
     * |[6:4]     |CSR1      |Timer 1 Clock Source Selection(PWM timer 1 for group A)
     * |          |          |Select clock input for timer.
     * |          |          |(Table is the same as CSR3)
     * |[10:8]    |CSR2      |Timer 2 Clock Source Selection(PWM timer 2 for group A)
     * |          |          |Select clock input for timer.
     * |          |          |(Table is the same as CSR3)
     * |[14:12]   |CSR3      |Timer 3 Clock Source Selection (PWM timer 3 for group A)
     * |          |          |Select clock input for timer.
     * |          |          |CSRx[2:0] = Input clock divider
     * |          |          |100 = 1
     * |          |          |011 = 16
     * |          |          |010 = 8
     * |          |          |001 = 4
     * |          |          |000 = 2
     */
    __IO uint32_t CSR;

    /**
     * PCR
     * ===================================================================================================
     * Offset: 0x08  PWM Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits     |Field         |Descriptions
     * | :----:  | :----:       | :---- |
     * |[0]      |CH0EN         |PWM-Timer 0 Enable (PWM Timer 0 For Group A)
     * |         |              |0 = The corresponding PWM-Timer stops running.
     * |         |              |1 = The corresponding PWM-Timer starts running.
     * |[1]      |CH0PINV       |PWM-Timer 0 Output Polar Inverse Enable (PWM Timer 0 For Group A)
     * |         |              |0 = PWM0 output polar inverse Disabled.
     * |         |              |1 = PWM0 output polar inverse Enabled.
     * |[2]      |CH0INV        |PWM-Timer 0 Output Inverter Enable (PWM Timer 0 For Group A)
     * |         |              |0 = Inverter Disabled.
     * |         |              |1 = Inverter Enabled.
     * |[3]      |CH0MOD        |PWM-Timer 0 Auto-Reload/One-Shot Mode (PWM Timer 0 For Group A)
     * |         |              |0 = One-shot mode.
     * |         |              |1 = Auto-reload mode.
     * |         |              |Note: If there is a transition at this bit, it will cause CNR0 and CMR0 be cleared.
     * |[4]      |DZEN01        |Dead-Zone 0 Generator Enable (PWM0 And PWM1 Pair For PWM Group A)
     * |         |              |0 = Disabled.
     * |         |              |1 = Enabled.
     * |         |              |Note: When Dead-zone generator is enabled, the pair of PWM0 and PWM1 becomes a complementary pair for PWM group A.
     * |[5]      |DZEN23        |Dead-Zone 2 Generator Enable (PWM2 And PWM3 Pair For PWM Group A)
     * |         |              |0 = Disabled.
     * |         |              |1 = Enabled.
     * |         |              |Note: When Dead-zone generator is enabled, the pair of PWM2 and PWM3 becomes a complementary pair for PWM group A.
     * |[8]      |CH1EN         |PWM-Timer 1 Enable (PWM Timer 1 For Group A)
     * |         |              |0 = Corresponding PWM-Timer Stopped.
     * |         |              |1 = Corresponding PWM-Timer Start Running.
     * |[9]      |CH1PINV       |PWM-Timer 1 Output Polar Inverse Enable (PWM Timer 1 For Group A)
     * |         |              |0 = PWM1 output polar inverse Disabled.
     * |         |              |1 = PWM1 output polar inverse Enabled.
     * |[10]     |CH1INV        |PWM-Timer 1 Output Inverter Enable (PWM Timer 1 For Group A)
     * |         |              |0 = Inverter Disable.
     * |         |              |1 = Inverter Enable.
     * |[11]     |CH1MOD        |PWM-Timer 1 Auto-Reload/One-Shot Mode (PWM Timer 1 For Group A)
     * |         |              |0 = One-shot mode.
     * |         |              |1 = Auto-reload mode.
     * |         |              |Note: If there is a transition at this bit, it will cause CNR1 and CMR1 be cleared.
     * |[16]     |CH2EN         |PWM-Timer 2 Enable (PWM Timer 2 For Group A)
     * |         |              |0 = Corresponding PWM-Timer Stopped.
     * |         |              |1 = Corresponding PWM-Timer Start Running.
     * |[17]     |CH2PINV       |PWM-Timer 2 Output Polar Inverse Enable (PWM Timer 2 For Group A)
     * |         |              |0 = PWM2 output polar inverse Disabled.
     * |         |              |1 = PWM2 output polar inverse Enabled.
     * |[18]     |CH2INV        |PWM-Timer 2 Output Inverter Enable (PWM Timer 2 For Group A)
     * |         |              |0 = Inverter Disabled.
     * |         |              |1 = Inverter Enabled.
     * |[19]     |CH2MOD        |PWM-Timer 2 Auto-Reload/One-Shot Mode (PWM Timer 2 For Group A)
     * |         |              |0 = One-shot mode.
     * |         |              |1 = Auto-reload mode.
     * |         |              |Note: If there is a transition at this bit, it will cause CNR2 and CMR2 be cleared.
     * |[24]     |CH3EN         |PWM-Timer 3 Enable (PWM Timer 3 For Group A)
     * |         |              |0 = Corresponding PWM-Timer Stopped.
     * |         |              |1 = Corresponding PWM-Timer Start Running.
     * |[25]     |CH3PINV       |PWM-Timer 3 Output Polar Inverse Enable (PWM Timer 3 For Group A)
     * |         |              |0 = PWM3 output polar inverse Disable.
     * |         |              |1 = PWM3 output polar inverse Enable.
     * |[26]     |CH3INV        |PWM-Timer 3 Output Inverter Enable (PWM Timer 3 For Group A)
     * |         |              |0 = Inverter Disabled.
     * |         |              |1 = Inverter Enabled.
     * |[27]     |CH3MOD        |PWM-Timer 3 Auto-Reload/One-Shot Mode (PWM Timer 3 For Group A)
     * |         |              |0 = One-shot mode.
     * |         |              |1 = Auto-reload mode.
     * |         |              |Note: If there is a transition at this bit, it will cause CNR3 and CMR3 be cleared.
     * |[30]     |PWM01TYPE     |PWM01 Aligned Type Selection Bit (PWM0 And PWM1 Pair For PWM Group A)
     * |         |              |0 = Edge-aligned type.
     * |         |              |1 = Center-aligned type.
     * |[31]     |PWM23TYPE     |PWM23 Aligned Type Selection Bit (PWM2 And PWM3 Pair For PWM Group A)
     * |         |              |0 = Edge-aligned type.
     * |         |              |1 = Center-aligned type.
     */
    __IO uint32_t PCR;

    /**
     * CNR0
     * ===================================================================================================
     * Offset: 0x0C  PWM Counter Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |CNRx      |PWM Timer Loaded Value
     * |        |          |CNR determines the PWM period.
     * |        |          |PWM frequency = PWMxy_CLK/[(prescale+1)*(clock divider)*(CNR+1)]; where xy, could be 01 or 23, depends on selected PWM channel.
     * |        |          |For Edge-aligned type:
     * |        |          | Duty ratio = (CMR+1)/(CNR+1).
     * |        |          | CMR >= CNR: PWM output is always high.
     * |        |          | CMR < CNR: PWM low width = (CNR-CMR) unit; PWM high width = (CMR+1) unit.
     * |        |          | CMR = 0: PWM low width = (CNR) unit; PWM high width = 1 unit.
     * |        |          |For Center-aligned type:
     * |        |          | Duty ratio = [(2 x CMR) + 1]/[2 x (CNR+1)].
     * |        |          | CMR > CNR: PWM output is always high.
     * |        |          | CMR <= CNR: PWM low width = 2 x (CNR-CMR) + 1 unit; PWM high width = (2 x CMR) + 1 unit.
     * |        |          | CMR = 0: PWM low width = 2 x CNR + 1 unit; PWM high width = 1 unit.
     * |        |          |(Unit = one PWM clock cycle).
     * |        |          |Note: Any write to CNR will take effect in next PWM cycle.
     * |        |          |Note: When PWM operating at Center-aligned type, CNR value should be set between 0x0000 to 0xFFFE.
     * |        |          |If CNR equal to 0xFFFF, the PWM will work unpredictable.
     * |        |          |Note: When CNR value is set to 0, PWM output is always high.
     */
    __IO uint32_t CNR0;

    /**
     * CMR0
     * ===================================================================================================
     * Offset: 0x10  PWM Counter Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |CNRx      |PWM Timer Loaded Value
     * |        |          |CNR determines the PWM period.
     * |        |          |PWM frequency = PWMxy_CLK/[(prescale+1)*(clock divider)*(CNR+1)]; where xy, could be 01 or 23, depends on selected PWM channel.
     * |        |          |For Edge-aligned type:
     * |        |          | Duty ratio = (CMR+1)/(CNR+1).
     * |        |          | CMR >= CNR: PWM output is always high.
     * |        |          | CMR < CNR: PWM low width = (CNR-CMR) unit; PWM high width = (CMR+1) unit.
     * |        |          | CMR = 0: PWM low width = (CNR) unit; PWM high width = 1 unit.
     * |        |          |For Center-aligned type:
     * |        |          | Duty ratio = [(2 x CMR) + 1]/[2 x (CNR+1)].
     * |        |          | CMR > CNR: PWM output is always high.
     * |        |          | CMR <= CNR: PWM low width = 2 x (CNR-CMR) + 1 unit; PWM high width = (2 x CMR) + 1 unit.
     * |        |          | CMR = 0: PWM low width = 2 x CNR + 1 unit; PWM high width = 1 unit.
     * |        |          |(Unit = one PWM clock cycle).
     * |        |          |Note: Any write to CNR will take effect in next PWM cycle.
     * |        |          |Note: When PWM operating at Center-aligned type, CNR value should be set between 0x0000 to 0xFFFE.
     * |        |          |If CNR equal to 0xFFFF, the PWM will work unpredictable.
     * |        |          |Note: When CNR value is set to 0, PWM output is always high.
     */
    __IO uint32_t CMR0;

    /**
     * PDR0
     * ===================================================================================================
     * Offset: 0x14  PWM Data Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |PDRx      |PWM Data Register
     * |        |          |User can monitor PDR to know the current value in 16-bit counter.
     */
    __I  uint32_t PDR0;

    /**
     * CNR1
     * ===================================================================================================
     * Offset: 0x18  PWM Counter Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |CNRx      |PWM Timer Loaded Value
     * |        |          |CNR determines the PWM period.
     * |        |          |PWM frequency = PWMxy_CLK/[(prescale+1)*(clock divider)*(CNR+1)]; where xy, could be 01 or 23, depends on selected PWM channel.
     * |        |          |For Edge-aligned type:
     * |        |          | Duty ratio = (CMR+1)/(CNR+1).
     * |        |          | CMR >= CNR: PWM output is always high.
     * |        |          | CMR < CNR: PWM low width = (CNR-CMR) unit; PWM high width = (CMR+1) unit.
     * |        |          | CMR = 0: PWM low width = (CNR) unit; PWM high width = 1 unit.
     * |        |          |For Center-aligned type:
     * |        |          | Duty ratio = [(2 x CMR) + 1]/[2 x (CNR+1)].
     * |        |          | CMR > CNR: PWM output is always high.
     * |        |          | CMR <= CNR: PWM low width = 2 x (CNR-CMR) + 1 unit; PWM high width = (2 x CMR) + 1 unit.
     * |        |          | CMR = 0: PWM low width = 2 x CNR + 1 unit; PWM high width = 1 unit.
     * |        |          |(Unit = one PWM clock cycle).
     * |        |          |Note: Any write to CNR will take effect in next PWM cycle.
     * |        |          |Note: When PWM operating at Center-aligned type, CNR value should be set between 0x0000 to 0xFFFE.
     * |        |          |If CNR equal to 0xFFFF, the PWM will work unpredictable.
     * |        |          |Note: When CNR value is set to 0, PWM output is always high.
     */
    __IO uint32_t CNR1;

    /**
     * CMR1
     * ===================================================================================================
     * Offset: 0x1C  PWM Comparator Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |CMRx      |PWM Comparator Register
     * |        |          |CMR determines the PWM duty.
     * |        |          |PWM frequency = PWMxy_CLK/[(prescale+1)*(clock divider)*(CNR+1)]; where xy, could be 01 or 23, depends on selected PWM channel.
     * |        |          |For Edge-aligned type:
     * |        |          | Duty ratio = (CMR+1)/(CNR+1).
     * |        |          | CMR >= CNR: PWM output is always high.
     * |        |          | CMR < CNR: PWM low width = (CNR-CMR) unit; PWM high width = (CMR+1) unit.
     * |        |          | CMR = 0: PWM low width = (CNR) unit; PWM high width = 1 unit.
     * |        |          |For Center-aligned type:
     * |        |          | Duty ratio = [(2 x CMR) + 1]/[2 x (CNR+1)].
     * |        |          | CMR > CNR: PWM output is always high.
     * |        |          | CMR <= CNR: PWM low width = 2 x (CNR-CMR) + 1 unit; PWM high width = (2 x CMR) + 1 unit.
     * |        |          | CMR = 0: PWM low width = 2 x CNR + 1 unit; PWM high width = 1 unit.
     * |        |          |(Unit = one PWM clock cycle).
     * |        |          |Note: Any write to CNR will take effect in next PWM cycle.
     */
    __IO uint32_t CMR1;

    /**
     * PDR1
     * ===================================================================================================
     * Offset: 0x20  PWM Data Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |PDRx      |PWM Data Register
     * |        |          |User can monitor PDR to know the current value in 16-bit counter.
     */
    __I  uint32_t PDR1;

    /**
     * CNR2
     * ===================================================================================================
     * Offset: 0x24  PWM Counter Register 2
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |CNRx      |PWM Timer Loaded Value
     * |        |          |CNR determines the PWM period.
     * |        |          |PWM frequency = PWMxy_CLK/[(prescale+1)*(clock divider)*(CNR+1)]; where xy, could be 01 or 23, depends on selected PWM channel.
     * |        |          |For Edge-aligned type:
     * |        |          | Duty ratio = (CMR+1)/(CNR+1).
     * |        |          | CMR >= CNR: PWM output is always high.
     * |        |          | CMR < CNR: PWM low width = (CNR-CMR) unit; PWM high width = (CMR+1) unit.
     * |        |          | CMR = 0: PWM low width = (CNR) unit; PWM high width = 1 unit.
     * |        |          |For Center-aligned type:
     * |        |          | Duty ratio = [(2 x CMR) + 1]/[2 x (CNR+1)].
     * |        |          | CMR > CNR: PWM output is always high.
     * |        |          | CMR <= CNR: PWM low width = 2 x (CNR-CMR) + 1 unit; PWM high width = (2 x CMR) + 1 unit.
     * |        |          | CMR = 0: PWM low width = 2 x CNR + 1 unit; PWM high width = 1 unit.
     * |        |          |(Unit = one PWM clock cycle).
     * |        |          |Note: Any write to CNR will take effect in next PWM cycle.
     * |        |          |Note: When PWM operating at Center-aligned type, CNR value should be set between 0x0000 to 0xFFFE.
     * |        |          |If CNR equal to 0xFFFF, the PWM will work unpredictable.
     * |        |          |Note: When CNR value is set to 0, PWM output is always high.
     */
    __IO uint32_t CNR2;

    /**
     * CMR2
     * ===================================================================================================
     * Offset: 0x28  PWM Comparator Register 2
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |CMRx      |PWM Comparator Register
     * |        |          |CMR determines the PWM duty.
     * |        |          |PWM frequency = PWMxy_CLK/[(prescale+1)*(clock divider)*(CNR+1)]; where xy, could be 01 or 23, depends on selected PWM channel.
     * |        |          |For Edge-aligned type:
     * |        |          | Duty ratio = (CMR+1)/(CNR+1).
     * |        |          | CMR >= CNR: PWM output is always high.
     * |        |          | CMR < CNR: PWM low width = (CNR-CMR) unit; PWM high width = (CMR+1) unit.
     * |        |          | CMR = 0: PWM low width = (CNR) unit; PWM high width = 1 unit.
     * |        |          |For Center-aligned type:
     * |        |          | Duty ratio = [(2 x CMR) + 1]/[2 x (CNR+1)].
     * |        |          | CMR > CNR: PWM output is always high.
     * |        |          | CMR <= CNR: PWM low width = 2 x (CNR-CMR) + 1 unit; PWM high width = (2 x CMR) + 1 unit.
     * |        |          | CMR = 0: PWM low width = 2 x CNR + 1 unit; PWM high width = 1 unit.
     * |        |          |(Unit = one PWM clock cycle).
     * |        |          |Note: Any write to CNR will take effect in next PWM cycle.
     */
    __IO uint32_t CMR2;

    /**
     * PDR2
     * ===================================================================================================
     * Offset: 0x2C  PWM Data Register 2
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |PDRx      |PWM Data Register
     * |        |          |User can monitor PDR to know the current value in 16-bit counter.
     */
    __I  uint32_t PDR2;

    /**
     * CNR3
     * ===================================================================================================
     * Offset: 0x30  PWM Counter Register 3
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |CNRx      |PWM Timer Loaded Value
     * |        |          |CNR determines the PWM period.
     * |        |          |PWM frequency = PWMxy_CLK/[(prescale+1)*(clock divider)*(CNR+1)]; where xy, could be 01 or 23, depends on selected PWM channel.
     * |        |          |For Edge-aligned type:
     * |        |          | Duty ratio = (CMR+1)/(CNR+1).
     * |        |          | CMR >= CNR: PWM output is always high.
     * |        |          | CMR < CNR: PWM low width = (CNR-CMR) unit; PWM high width = (CMR+1) unit.
     * |        |          | CMR = 0: PWM low width = (CNR) unit; PWM high width = 1 unit.
     * |        |          |For Center-aligned type:
     * |        |          | Duty ratio = [(2 x CMR) + 1]/[2 x (CNR+1)].
     * |        |          | CMR > CNR: PWM output is always high.
     * |        |          | CMR <= CNR: PWM low width = 2 x (CNR-CMR) + 1 unit; PWM high width = (2 x CMR) + 1 unit.
     * |        |          | CMR = 0: PWM low width = 2 x CNR + 1 unit; PWM high width = 1 unit.
     * |        |          |(Unit = one PWM clock cycle).
     * |        |          |Note: Any write to CNR will take effect in next PWM cycle.
     * |        |          |Note: When PWM operating at Center-aligned type, CNR value should be set between 0x0000 to 0xFFFE.
     * |        |          |If CNR equal to 0xFFFF, the PWM will work unpredictable.
     * |        |          |Note: When CNR value is set to 0, PWM output is always high.
     */
    __IO uint32_t CNR3;

    /**
     * CMR3
     * ===================================================================================================
     * Offset: 0x34  PWM Comparator Register 3
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |CMRx      |PWM Comparator Register
     * |        |          |CMR determines the PWM duty.
     * |        |          |PWM frequency = PWMxy_CLK/[(prescale+1)*(clock divider)*(CNR+1)]; where xy, could be 01 or 23, depends on selected PWM channel.
     * |        |          |For Edge-aligned type:
     * |        |          | Duty ratio = (CMR+1)/(CNR+1).
     * |        |          | CMR >= CNR: PWM output is always high.
     * |        |          | CMR < CNR: PWM low width = (CNR-CMR) unit; PWM high width = (CMR+1) unit.
     * |        |          | CMR = 0: PWM low width = (CNR) unit; PWM high width = 1 unit.
     * |        |          |For Center-aligned type:
     * |        |          | Duty ratio = [(2 x CMR) + 1]/[2 x (CNR+1)].
     * |        |          | CMR > CNR: PWM output is always high.
     * |        |          | CMR <= CNR: PWM low width = 2 x (CNR-CMR) + 1 unit; PWM high width = (2 x CMR) + 1 unit.
     * |        |          | CMR = 0: PWM low width = 2 x CNR + 1 unit; PWM high width = 1 unit.
     * |        |          |(Unit = one PWM clock cycle).
     * |        |          |Note: Any write to CNR will take effect in next PWM cycle.
     */
    __IO uint32_t CMR3;

    /**
     * PDR3
     * ===================================================================================================
     * Offset: 0x38  PWM Data Register 3
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |PDRx      |PWM Data Register
     * |        |          |User can monitor PDR to know the current value in 16-bit counter.
     */
    __I  uint32_t PDR3;

    /**
     * @cond HIDDEN_SYMBOLS
     * RESERVED0
     * ===================================================================================================
     *
     * ---------------------------------------------------------------------------------------------------
     */
    __I  uint32_t RESERVED0[1];
    /**
     * @endcond
     */

    /**
     * PIER
     * ===================================================================================================
     * Offset: 0x40  PWM Interrupt Enable Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |PWMIE0    |PWM Channel 0 Period Interrupt Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[1]     |PWMIE1    |PWM Channel 1 Period Interrupt Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[2]     |PWMIE2    |PWM Channel 2 Period Interrupt Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[3]     |PWMIE3    |PWM Channel 3 Period Interrupt Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[8]     |PWMDIE0   |PWM Channel 0 Duty Interrupt Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[9]     |PWMDIE1   |PWM Channel 1 Duty Interrupt Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[10]    |PWMDIE2   |PWM Channel 2 Duty Interrupt Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[11]    |PWMDIE3   |PWM Channel 3 Duty Interrupt Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[16]    |INT01TYPE |PWM01 Interrupt Period Type Selection Bit (PWM0 And PWM1 Pair For PWM Group A)
     * |        |          |0 = PWMIFn will be set if PWM counter underflow.
     * |        |          |1 = PWMIFn will be set if PWM counter matches CNRn register.
     * |        |          |Note: This bit is effective when PWM in Center-aligned type only.
     * |[17]    |INT23TYPE |PWM23 Interrupt Period Type Selection Bit (PWM2 And PWM3 Pair For PWM Group A)
     * |        |          |0 = PWMIFn will be set if PWM counter underflow.
     * |        |          |1 = PWMIFn will be set if PWM counter matches CNRn register.
     * |        |          |Note: This bit is effective when PWM in Center-aligned type only.
     */
    __IO uint32_t PIER;

    /**
     * PIIR
     * ===================================================================================================
     * Offset: 0x44  PWM Interrupt Indication Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |PWMIF0    |PWM Channel 0 Period Interrupt Status
     * |        |          |This bit is set by hardware when PWM0 counter reaches the requirement of interrupt (depend on INT01TYPE bit of PIER register), software can write 1 to clear this bit to 0.
     * |[1]     |PWMIF1    |PWM Channel 1 Period Interrupt Status
     * |        |          |This bit is set by hardware when PWM1 counter reaches the requirement of interrupt (depend on INT01TYPE bit of PIER register), software can write 1 to clear this bit to 0.
     * |[2]     |PWMIF2    |PWM Channel 2 Period Interrupt Status
     * |        |          |This bit is set by hardware when PWM2 counter reaches the requirement of interrupt (depend on INT23TYPE bit of PIER register), software can write 1 to clear this bit to 0.
     * |[3]     |PWMIF3    |PWM Channel 3 Period Interrupt Status
     * |        |          |This bit is set by hardware when PWM3 counter reaches the requirement of interrupt (depend on INT23TYPE bit of PIER register), software can write 1 to clear this bit to 0.
     * |[8]     |PWMDIF0   |PWM Channel 0 Duty Interrupt Flag
     * |        |          |Flag is set by hardware when channel 0 PWM counter down count and reaches CMR0, software can clear this bit by writing a one to it.
     * |        |          |Note: If CMR equal to CNR, this flag is not working in Edge-aligned type selection
     * |[9]     |PWMDIF1   |PWM Channel 1 Duty Interrupt Flag
     * |        |          |Flag is set by hardware when channel 1 PWM counter down count and reaches CMR1, software can clear this bit by writing a one to it.
     * |        |          |Note: If CMR equal to CNR, this flag is not working in Edge-aligned type selection
     * |[10]    |PWMDIF2   |PWM Channel 2 Duty Interrupt Flag
     * |        |          |Flag is set by hardware when channel 2 PWM counter down count and reaches CMR2, software can clear this bit by writing a one to it.
     * |        |          |Note: If CMR equal to CNR, this flag is not working in Edge-aligned type selection
     * |[11]    |PWMDIF3   |PWM Channel 3 Duty Interrupt Flag
     * |        |          |Flag is set by hardware when channel 3 PWM counter down count and reaches CMR3, software can clear this bit by writing a one to it.
     * |        |          |Note: If CMR equal to CNR, this flag is not working in Edge-aligned type selection
     */
    __IO uint32_t PIIR;

    /**
     * @cond HIDDEN_SYMBOLS
     * RESERVE1
     * ===================================================================================================
     *
     * ---------------------------------------------------------------------------------------------------
     */
    __I uint32_t RESERVE1[2];
    /**
     * @endcond
     */

    /**
     * CCR0
     * ===================================================================================================
     * Offset: 0x50  PWM Capture Control Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |INV0      |Channel 0 Inverter Enable
     * |        |          |0 = Inverter Disabled.
     * |        |          |1 = Inverter Enabled. Reverse the input signal from GPIO before fed to Capture timer
     * |[1]     |CRL_IE0   |Channel 0 Rising Latch Interrupt Enable
     * |        |          |0 = Rising latch interrupt Disabled.
     * |        |          |1 = Rising latch interrupt Enabled.
     * |        |          |When Enabled, if Capture detects PWM group channel 0 has rising transition, Capture will issue an Interrupt.
     * |[2]     |CFL_IE0   |Channel 0 Falling Latch Interrupt Enable
     * |        |          |0 = Falling latch interrupt Disabled.
     * |        |          |1 = Falling latch interrupt Enabled.
     * |        |          |When Enabled, if Capture detects PWM group channel 0 has falling transition, Capture will issue an Interrupt.
     * |[3]     |CAPCH0EN  |Channel 0 Capture Function Enable
     * |        |          |0 = Capture function on PWM group channel 0 Disabled.
     * |        |          |1 = Capture function on PWM group channel 0 Enabled.
     * |        |          |When Enabled, Capture latched the PWM-counter value and saved to CRLR (Rising latch) and CFLR (Falling latch).
     * |        |          |When Disabled, Capture does not update CRLR and CFLR, and disable PWM group channel 0 Interrupt.
     * |[4]     |CAPIF0    |Channel 0 Capture Interrupt Indication Flag
     * |        |          |If PWM group channel 0 rising latch interrupt is enabled (CRL_IE0 = 1), a rising transition occurs at PWM group channel 0 will result in CAPIF0 to high; Similarly, a falling transition will cause CAPIF0 to be set high if PWM group channel 0 falling latch interrupt is enabled (CFL_IE0 = 1).
     * |        |          |Write 1 to clear this bit to 0.
     * |[6]     |CRLRI0    |CRLR0 Latched Indicator Bit
     * |        |          |When PWM group input channel 0 has a rising transition, CRLR0 was latched with the value of PWM down-counter and this bit is set by hardware.
     * |        |          |Software can write 1 to clear this bit to 0.
     * |[7]     |CFLRI0    |CFLR0 Latched Indicator Bit
     * |        |          |When PWM group input channel 0 has a falling transition, CFLR0 was latched with the value of PWM down-counter and this bit is set by hardware.
     * |        |          |Software can write 1 to clear this bit to 0.
     * |[16]    |INV1      |Channel 1 Inverter Enable
     * |        |          |0 = Inverter Disabled.
     * |        |          |1 = Inverter Enabled. Reverse the input signal from GPIO before fed to Capture timer
     * |[17]    |CRL_IE1   |Channel 1 Rising Latch Interrupt Enable
     * |        |          |0 = Rising latch interrupt Disabled.
     * |        |          |1 = Rising latch interrupt Enabled.
     * |        |          |When Enabled, if Capture detects PWM group channel 1 has rising transition, Capture will issue an Interrupt.
     * |[18]    |CFL_IE1   |Channel 1 Falling Latch Interrupt Enable
     * |        |          |0 = Falling latch interrupt Disabled.
     * |        |          |1 = Falling latch interrupt Enabled.
     * |        |          |When Enabled, if Capture detects PWM group channel 1 has falling transition, Capture will issue an Interrupt.
     * |[19]    |CAPCH1EN  |Channel 1 Capture Function Enable
     * |        |          |0 = Capture function on PWM group channel 1 Disabled.
     * |        |          |1 = Capture function on PWM group channel 1 Enabled.
     * |        |          |When Enabled, Capture latched the PWM-counter and saved to CRLR (Rising latch) and CFLR (Falling latch).
     * |        |          |When Disabled, Capture does not update CRLR and CFLR, and disable PWM group channel 1 Interrupt.
     * |[20]    |CAPIF1    |Channel 1 Capture Interrupt Indication Flag
     * |        |          |If PWM group channel 1 rising latch interrupt is enabled (CRL_IE1 = 1), a rising transition occurs at PWM group channel 1 will result in CAPIF1 to high; Similarly, a falling transition will cause CAPIF1 to be set high if PWM group channel 1 falling latch interrupt is enabled (CFL_IE1 = 1).
     * |        |          |Write 1 to clear this bit to 0.
     * |[22]    |CRLRI1    |CRLR1 Latched Indicator Bit
     * |        |          |When PWM group input channel 1 has a rising transition, CRLR1 was latched with the value of PWM down-counter and this bit is set by hardware.
     * |        |          |Software can write 1 to clear this bit to 0.
     * |[23]    |CFLRI1    |CFLR1 Latched Indicator Bit
     * |        |          |When PWM group input channel 1 has a falling transition, CFLR1 was latched with the value of PWM down-counter and this bit is set by hardware.
     * |        |          |Software can write 1 to clear this bit to 0.
     */
    __IO uint32_t CCR0;

    /**
     * CCR2
     * ===================================================================================================
     * Offset: 0x54  PWM Capture Control Register 2
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |INV2      |Channel 2 Inverter Enable
     * |        |          |0 = Inverter Disabled.
     * |        |          |1 = Inverter Enabled. Reverse the input signal from GPIO before fed to Capture timer
     * |[1]     |CRL_IE2   |Channel 2 Rising Latch Interrupt Enable
     * |        |          |0 = Rising latch interrupt Disabled.
     * |        |          |1 = Rising latch interrupt Enabled.
     * |        |          |When Enabled, if Capture detects PWM group channel 2 has rising transition, Capture will issue an Interrupt.
     * |[2]     |CFL_IE2   |Channel 2 Falling Latch Interrupt Enable
     * |        |          |0 = Falling latch interrupt Disabled.
     * |        |          |1 = Falling latch interrupt Enabled.
     * |        |          |When Enabled, if Capture detects PWM group channel 2 has falling transition, Capture will issue an Interrupt.
     * |[3]     |CAPCH2EN  |Channel 2 Capture Function Enable
     * |        |          |0 = Capture function on PWM group channel 2 Disabled.
     * |        |          |1 = Capture function on PWM group channel 2 Enabled.
     * |        |          |When Enabled, Capture latched the PWM-counter value and saved to CRLR (Rising latch) and CFLR (Falling latch).
     * |        |          |When Disabled, Capture does not update CRLR and CFLR, and disable PWM group channel 2 Interrupt.
     * |[4]     |CAPIF2    |Channel 2 Capture Interrupt Indication Flag
     * |        |          |If PWM group channel 2 rising latch interrupt is enabled (CRL_IE2=1), a rising transition occurs at PWM group channel 2 will result in CAPIF2 to high; Similarly, a falling transition will cause CAPIF2 to be set high if PWM group channel 2 falling latch interrupt is enabled (CFL_IE2=1).
     * |        |          |Write 1 to clear this bit to 0
     * |[6]     |CRLRI2    |CRLR2 Latched Indicator Bit
     * |        |          |When PWM group input channel 2 has a rising transition, CRLR2 was latched with the value of PWM down-counter and this bit is set by hardware.
     * |        |          |Software can write 1 to clear this bit to 0.
     * |[7]     |CFLRI2    |CFLR2 Latched Indicator Bit
     * |        |          |When PWM group input channel 2 has a falling transition, CFLR2 was latched with the value of PWM down-counter and this bit is set by hardware.
     * |        |          |Software can write 1 to clear this bit to 0.
     * |[16]    |INV3      |Channel 3 Inverter Enable
     * |        |          |0 = Inverter Disabled.
     * |        |          |1 = Inverter Enabled. Reverse the input signal from GPIO before fed to Capture timer
     * |[17]    |CRL_IE3   |Channel 3 Rising Latch Interrupt Enable
     * |        |          |0 = Rising latch interrupt Disabled.
     * |        |          |1 = Rising latch interrupt Enabled.
     * |        |          |When Enabled, if Capture detects PWM group channel 3 has rising transition, Capture will issue an Interrupt.
     * |[18]    |CFL_IE3   |Channel 3 Falling Latch Interrupt Enable
     * |        |          |0 = Falling latch interrupt Disabled.
     * |        |          |1 = Falling latch interrupt Enabled.
     * |        |          |When Enabled, if Capture detects PWM group channel 3 has falling transition, Capture will issue an Interrupt.
     * |[19]    |CAPCH3EN  |Channel 3 Capture Function Enable
     * |        |          |0 = Capture function on PWM group channel 3 Disabled.
     * |        |          |1 = Capture function on PWM group channel 3 Enabled.
     * |        |          |When Enabled, Capture latched the PWM-counter and saved to CRLR (Rising latch) and CFLR (Falling latch).
     * |        |          |When Disabled, Capture does not update CRLR and CFLR, and disable PWM group channel 3 Interrupt.
     * |[20]    |CAPIF3    |Channel 3 Capture Interrupt Indication Flag
     * |        |          |If PWM group channel 3 rising latch interrupt is enabled (CRL_IE3=1), a rising transition occurs at PWM group channel 3 will result in CAPIF3 to high; Similarly, a falling transition will cause CAPIF3 to be set high if PWM group channel 3 falling latch interrupt is enabled (CFL_IE3=1).
     * |        |          |Write 1 to clear this bit to 0
     * |[22]    |CRLRI3    |CRLR3 Latched Indicator Bit
     * |        |          |When PWM group input channel 3 has a rising transition, CRLR3 was latched with the value of PWM down-counter and this bit is set by hardware.
     * |        |          |Software can write 1 to clear this bit to 0.
     * |[23]    |CFLRI3    |CFLR3 Latched Indicator Bit
     * |        |          |When PWM group input channel 3 has a falling transition, CFLR3 was latched with the value of PWM down-counter and this bit is set by hardware.
     * |        |          |Software can write 1 to clear this bit to 0.
      */
    __IO uint32_t CCR2;

    /**
     * CRLR0
     * ===================================================================================================
     * Offset: 0x58  PWM Capture Rising Latch Register (Channel 0)
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |CRLRx     |Capture Rising Latch Register
     * |        |          |Latch the PWM counter when Channel 0/1/2/3 has rising transition.
     */
    __IO uint32_t CRLR0;

    /**
     * CFLR0
     * ===================================================================================================
     * Offset: 0x5C  PWM Capture Falling Latch Register (Channel 0)
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |CFLRx     |Capture Falling Latch Register
     * |        |          |Latch the PWM counter when Channel 0/1/2/3 has Falling transition.
     */
    __IO uint32_t CFLR0;

    /**
     * CRLR1
     * ===================================================================================================
     * Offset: 0x60  PWM Capture Rising Latch Register (Channel 1)
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |CRLRx     |Capture Rising Latch Register
     * |        |          |Latch the PWM counter when Channel 0/1/2/3 has rising transition.
     */
    __IO uint32_t CRLR1;

    /**
     * CFLR1
     * ===================================================================================================
     * Offset: 0x64  PWM Capture Falling Latch Register (Channel 1)
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |CFLRx     |Capture Falling Latch Register
     * |        |          |Latch the PWM counter when Channel 0/1/2/3 has Falling transition.
     */
    __IO uint32_t CFLR1;

    /**
     * CRLR2
     * ===================================================================================================
     * Offset: 0x68  PWM Capture Rising Latch Register (Channel 2)
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |CRLRx     |Capture Rising Latch Register
     * |        |          |Latch the PWM counter when Channel 0/1/2/3 has rising transition.
     */
    __IO uint32_t CRLR2;

    /**
     * CFLR2
     * ===================================================================================================
     * Offset: 0x6C  PWM Capture Falling Latch Register (Channel 2)
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |CFLRx     |Capture Falling Latch Register
     * |        |          |Latch the PWM counter when Channel 0/1/2/3 has Falling transition.
     */
    __IO uint32_t CFLR2;

    /**
     * CRLR3
     * ===================================================================================================
     * Offset: 0x70  PWM Capture Rising Latch Register (Channel 3)
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |CRLRx     |Capture Rising Latch Register
     * |        |          |Latch the PWM counter when Channel 0/1/2/3 has rising transition.
     */
    __IO uint32_t CRLR3;

    /**
     * CFLR3
     * ===================================================================================================
     * Offset: 0x74  PWM Capture Falling Latch Register (Channel 3)
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |CFLRx     |Capture Falling Latch Register
     * |        |          |Latch the PWM counter when Channel 0/1/2/3 has Falling transition.
     */
    __IO uint32_t CFLR3;

    /**
     * CAPENR
     * ===================================================================================================
     * Offset: 0x78  PWM Capture Input 0~3 Enable Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CINEN0    |Channel 0 Capture Input Enable
     * |        |          |0 = PWM Channel 0 capture input path Disabled.
     * |        |          |The input of PWM channel 0 capture function is always regarded as 0.
     * |        |          |1 = PWM Channel 0 capture input path Enabled.
     * |        |          |The input of PWM channel 0 capture function comes from correlative multifunction pin if GPIO multi-function is set as PWM0.
     * |[1]     |CINEN1    |Channel 1 Capture Input Enable
     * |        |          |0 = PWM Channel 1 capture input path Disabled.
     * |        |          |The input of PWM channel 1 capture function is always regarded as 0.
     * |        |          |1 = PWM Channel 1 capture input path Enabled.
     * |        |          |The input of PWM channel 1 capture function comes from correlative multifunction pin if GPIO multi-function is set as PWM1.
     * |[2]     |CINEN2    |Channel 2 Capture Input Enable
     * |        |          |0 = PWM Channel 2 capture input path Disabled.
     * |        |          |The input of PWM channel 2 capture function is always regarded as 0.
     * |        |          |1 = PWM Channel 2 capture input path Enabled.
     * |        |          |The input of PWM channel 2 capture function comes from correlative multifunction pin if GPIO multi-function is set as PWM2.
     * |[3]     |CINEN3    |Channel 3 Capture Input Enable
     * |        |          |0 = PWM Channel 3 capture input path Disabled.
     * |        |          |The input of PWM channel 3 capture function is always regarded as 0.
     * |        |          |1 = PWM Channel 3 capture input path Enabled.
     * |        |          |The input of PWM channel 3 capture function comes from correlative multifunction pin if GPIO multi-function is set as PWM3.
     */
    __IO uint32_t CAPENR;

    /**
     * POE
     * ===================================================================================================
     * Offset: 0x7C  PWM Output Enable for Channel 0~3
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |POE0      |Channel 0 Output Enable Register
     * |        |          |0 = PWM channel 0 output to pin Disabled.
     * |        |          |1 = PWM channel 0 output to pin Enabled.
     * |        |          |Note: The corresponding GPIO pin must also be switched to PWM function
     * |[1]     |POE1      |Channel 1 Output Enable Register
     * |        |          |0 = PWM channel 1 output to pin Disabled.
     * |        |          |1 = PWM channel 1 output to pin Enabled.
     * |        |          |Note: The corresponding GPIO pin must also be switched to PWM function
     * |[2]     |POE2      |Channel 2 Output Enable Register
     * |        |          |0 = PWM channel 2 output to pin Disabled.
     * |        |          |1 = PWM channel 2 output to pin Enabled.
     * |        |          |Note: The corresponding GPIO pin must also be switched to PWM function
     * |[3]     |POE3      |Channel 3 Output Enable Register
     * |        |          |0 = PWM channel 3 output to pin Disabled.
     * |        |          |1 = PWM channel 3 output to pin Enabled.
     * |        |          |Note: The corresponding GPIO pin must also be switched to PWM function
     */
    __IO uint32_t POE;

    /**
     * TCON
     * ===================================================================================================
     * Offset: 0x80  PWM Trigger Control for Channel 0~3
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |PWM0TEN   |Channel 0 Center-Aligned Trigger Enable Register
     * |        |          |0 = PWM channel 0 trigger ADC function Disabled.
     * |        |          |1 = PWM channel 0 trigger ADC function Enabled.
     * |        |          |PWM can trigger ADC to start conversion when PWM counter up count to CNR if this bit is set to 1.
     * |        |          |Note: This function is only supported when PWM operating at Center-aligned type.
     * |[1]     |PWM1TEN   |Channel 1 Center-Aligned Trigger Enable Register
     * |        |          |0 = PWM channel 1 trigger ADC function Disabled.
     * |        |          |1 = PWM channel 1 trigger ADC function Enabled.
     * |        |          |PWM can trigger ADC to start conversion when PWM counter up count to CNR if this bit is set to 1.
     * |        |          |Note: This function is only supported when PWM operating at Center-aligned type.
     * |[2]     |PWM2TEN   |Channel 2 Center-Aligned Trigger Enable Register
     * |        |          |0 = PWM channel 2 trigger ADC function Disabled.
     * |        |          |1 = PWM channel 2 trigger ADC function Enabled.
     * |        |          |PWM can trigger ADC to start conversion when PWM counter up count to CNR if this bit is set to 1.
     * |        |          |Note: This function is only supported when PWM operating at Center-aligned type.
     * |[3]     |PWM3TEN   |Channel 3 Center-Aligned Trigger Enable Register
     * |        |          |0 = PWM channel 3 trigger ADC function Disabled.
     * |        |          |1 = PWM channel 3 trigger ADC function Enabled.
     * |        |          |PWM can trigger ADC to start conversion when PWM counter up count to CNR if this bit is set to 1.
     * |        |          |Note: This function is only supported when PWM operating at Center-aligned type.
     */
    __IO uint32_t TCON;

    /**
     * TSTATUS
     * ===================================================================================================
     * Offset: 0x84  PWM Trigger Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |PWM0TF    |Channel 0 Center-Aligned Trigger Flag
     * |        |          |For Center-aligned Operating mode, this bit is set to 1 by hardware when PWM counter up counts to CNR if PWM0TEN bit is set to 1.
     * |        |          |After this bit is set to 1, ADC will start conversion if ADC triggered source is selected by PWM.
     * |        |          |Software can write 1 to clear this bit.
     * |[1]     |PWM1TF    |Channel 1 Center-Aligned Trigger Flag
     * |        |          |For Center-aligned Operating mode, this bit is set to 1 by hardware when PWM counter up count to CNR if PWM1TEN bit is set to 1.
     * |        |          |After this bit is set to 1, ADC will start conversion if ADC triggered source is selected by PWM.
     * |        |          |Software can write 1 to clear this bit.
     * |[2]     |PWM2TF    |Channel 2 Center-Aligned Trigger Flag
     * |        |          |For Center-aligned Operating mode, this bit is set to 1 by hardware when PWM counter up count to CNR if PWM2TEN bit is set to 1.
     * |        |          |After this bit is set to 1, ADC will start conversion if ADC triggered source is selected by PWM.
     * |        |          |Software can write 1 to clear this bit.
     * |[3]     |PWM3TF    |Channel 3 Center-Aligned Trigger Flag
     * |        |          |For Center-aligned Operating mode, this bit is set to 1 by hardware when PWM counter up count to CNR if PWM3TEN bit is set to 1.
     * |        |          |After this bit is set to 1, ADC will start conversion if ADC triggered source is selected by PWM.
     * |        |          |Software can write 1 to clear this bit.
     */
    __IO uint32_t TSTATUS;

    /**
     * SYNCBUSY0
     * ===================================================================================================
     * Offset: 0x88  PWM0 Synchronous Busy Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |S_BUSY    |PWM Synchronous Busy
     * |        |          |When software writes CNR0/CMR0/PPR or switches PWM0 operation mode (PCR[3]), PWM will have a busy time to update these values completely because PWM clock may be different from system clock domain.
     * |        |          |Software needs to check this busy status before writing CNR0/CMR0/PPR or switching PWM0 operation mode (PCR[3]) to make sure previous setting has been updated completely.
     * |        |          |This bit will be set when software writes CNR0/CMR0/PPR or switches PWM0 operation mode (PCR[3]) and will be cleared by hardware automatically when PWM update these value completely.
     */
    __IO uint32_t  SYNCBUSY0;

    /**
     * SYNCBUSY1
     * ===================================================================================================
     * Offset: 0x8C  PWM1 Synchronous Busy Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |S_BUSY    |PWM Synchronous Busy
     * |        |          |When Software writes CNR1/CMR1/PPR or switches PWM1 operation mode (PCR[11]), PWM will have a busy time to update these values completely because PWM clock may be different from system clock domain.
     * |        |          |Software needs to check this busy status before writing CNR1/CMR1/PPR or switching PWM1 operation mode (PCR[11]) to make sure previous setting has been updated completely.
     * |        |          |This bit will be set when software writes CNR1/CMR1/PPR or switches PWM1 operation mode (PCR[11]) and will be cleared by hardware automatically when PWM update these value completely.
     */
    __IO uint32_t  SYNCBUSY1;

    /**
     * SYNCBUSY2
     * ===================================================================================================
     * Offset: 0x90  PWM2 Synchronous Busy Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |S_BUSY    |PWM Synchronous Busy
     * |        |          |When Software writes CNR2/CMR2/PPR or switch PWM2 operation mode (PCR[19]), PWM will have a busy time to update these values completely because PWM clock may be different from system clock domain.
     * |        |          |Software needs to check this busy status before writing CNR2/CMR2/PPR or switching PWM2 operation mode (PCR[19]) to make sure previous setting has been updated completely.
     * |        |          |This bit will be set when software writes CNR2/CMR2/PPR or switch PWM2 operation mode (PCR[19]) and will be cleared by hardware automatically when PWM update these value completely.
     */
    __IO uint32_t  SYNCBUSY2;

    /**
     * SYNCBUSY3
     * ===================================================================================================
     * Offset: 0x94  PWM3 Synchronous Busy Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |S_BUSY    |PWM Synchronous Busy
     * |        |          |When Software writes CNR3/CMR3/PPR or switch PWM3 operation mode (PCR[27]), PWM will have a busy time to update these values completely because PWM clock may be different from system clock domain.
     * |        |          |Software need to check this busy status before writing CNR3/CMR3/PPR or switching PWM3 operation mode (PCR[27]) to make sure previous setting has been updated completely.
     * |        |          |This bit will be set when Software writes CNR3/CMR3/PPR or switch PWM3 operation mode (PCR[27]) and will be cleared by hardware automatically when PWM update these value completely.
     */
    __IO uint32_t  SYNCBUSY3;

    /**
     * @cond HIDDEN_SYMBOLS
     * RESERVE2
     * ===================================================================================================
     *
     * ---------------------------------------------------------------------------------------------------
     */
    __I uint32_t RESERVE2[10];
    /**
     * @endcond
     */

    /**
     * CAPPDMACTL
     * ===================================================================================================
     * Offset: 0xC0  PWM Group A Trigger Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field       |Descriptions
     * | :----: | :----:     | :---- |
     * |[0]     |CAP0PDMAEN  |Channel 0 PDMA Enable
     * |        |            |0 = Channel 0 PDMA function Disabled.
     * |        |            |1 = Channel 0 PDMA function Enabled for the channel 0 captured data and transfer to memory.
     * |[2:1]   |CAP0PDMAMOD |Select CRLR0 or CFLR0 to Transfer PDMA
     * |        |            |00 = Reserved
     * |        |            |01 = CRLR0
     * |        |            |10 = CFLR0
     * |        |            |11 = Both CRLR0 and CFLR0
     * |[3]     |CAP0RFORDER |Capture channel 0 Rising/Falling Order
     * |        |            |Set this bit to determine whether the CRLR0 or CFLR0 is the first captured data transferred to memory through PDMA when CAP0PDMAMOD =11
     * |        |            |0 = CFLR0 is the first captured data to memory.
     * |        |            |1 = CRLR0 is the first captured data to memory.
     * |[8]     |CAP1PDMAEN  |Channel 1 PDMA Enable
     * |        |            |0 = Channel 1 PDMA function Disabled.
     * |        |            |1 = Channel 1 PDMA function Enabled for the channel 1 captured data and transfer to memory.
     * |[10:9]  |CAP1PDMAMOD |Select CRLR1 or CFLR1 to Transfer PDMA
     * |        |            |00 = Reserved
     * |        |            |01 = CRLR1
     * |        |            |10 = CFLR1
     * |        |            |11 = Both CRLR1 and CFLR1
     * |[11]    |CAP1RFORDER |Capture channel 1 Rising/Falling Order
     * |        |            |Set this bit to determine whether the CRLR1 or CFLR1 is the first captured data transferred to memory through PDMA when CAP1PDMAMOD =11
     * |        |            |0 = CFLR1 is the first captured data to memory.
     * |        |            |1 = CRLR1 is the first captured data to memory.
     * |[16]    |CAP2PDMAEN  |Channel 2 PDMA Enable
     * |        |            |0 = Channel 2 PDMA function Disabled.
     * |        |            |1 = Channel 2 PDMA function Enabled for the channel 2 captured data and transfer to memory.
     * |[18:17] |CAP2PDMAMOD |Select CRLR2 or CFLR2 to Transfer PDMA
     * |        |            |00 = Reserved
     * |        |            |01 = CRLR2
     * |        |            |10 = CFLR2
     * |        |            |11 = Both CRLR2 and CFLR2
     * |[19]    |CAP2RFORDER |Capture channel 2 Rising/Falling Order
     * |        |            |Set this bit to determine whether the CRLR2 or CFLR2 is the first captured data transferred to memory through PDMA when CAP2PDMAMOD =11
     * |        |            |0 = CFLR2 is the first captured data to memory.
     * |        |            |1 = CRLR2 is the first captured data to memory.
     * |[24]    |CAP3PDMAEN  |Channel 3 PDMA Enable
     * |        |            |0 = Channel 3 PDMA function Disabled.
     * |        |            |1 = Channel 3 PDMA function Enabled for the channel 3 captured data and transfer to memory.
     * |[26:25] |CAP3PDMAMOD |Select CRLR3 or CFLR3 to Transfer PDMA
     * |        |            |00 = Reserved
     * |        |            |01 = CRLR3
     * |        |            |10 = CFLR3
     * |        |            |11 = Both CRLR3 and CFLR3
     * |[27]    |CAP3RFORDER |Capture channel 3 Rising/Falling Order
     * |        |            |Set this bit to determine whether the CRLR1 or CFLR3 is the first captured data transferred to memory through PDMA when CAP3PDMAMOD =11
     * |        |            |0 = CFLR3 is the first captured data to memory.
     * |        |            |1 = CRLR3 is the first captured data to memory.
     */
    __IO uint32_t CAPPDMACTL;

    /**
     * CAP0PDMA
     * ===================================================================================================
     * Offset: 0xC4  PWM Group A PDMA channel 0 DATA Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |CAP0RFPDMA|PDMA data register for channel 0
     * |        |          |it is the capturing value(CFLR0/CRLR0) for channel 0.
     */
    __IO uint32_t CAP0PDMA;

    /**
     * CAP1PDMA
     * ===================================================================================================
     * Offset: 0xC8  PWM Group A PDMA channel 1 DATA Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |CAP1RFPDMA|PDMA data register for channel 1
     * |        |          |it is the capturing value(CFLR1/CRLR1) for channel 1.
     */
    __IO uint32_t CAP1PDMA;

    /**
     * CAP2PDMA
     * ===================================================================================================
     * Offset: 0xCC  PWM Group A PDMA channel 2 DATA Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |CAP2RFPDMA|PDMA data register for channel 2
     * |        |          |it is the capturing value(CFLR2/CRLR2) for channel 2.
     */
    __IO uint32_t CAP2PDMA;

    /**
     * CAP3PDMA
     * ===================================================================================================
     * Offset: 0xD0  PWM Group A PDMA channel 3 DATA Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |CAP3RFPDMA|PDMA data register for channel 3
     * |        |          |it is the capturing value(CFLR3/CRLR3) for channel 3.
     */
    __IO uint32_t CAP3PDMA;

} PWM_T;


/** @addtogroup REG_PWM_BITMASK PWM Bit Mask
  @{
 */

/* PWM PPR Bit Field Definitions */
#define PWM_PPR_DZI23_Pos                       24                                  /*!< PWM PPR: DZI23 Position */
#define PWM_PPR_DZI23_Msk                       (0xFFul << PWM_PPR_DZI23_Pos)       /*!< PWM PPR: DZI23 Mask */

#define PWM_PPR_DZI01_Pos                       16                                  /*!< PWM PPR: DZI01 Position */
#define PWM_PPR_DZI01_Msk                       (0xFFul << PWM_PPR_DZI01_Pos)       /*!< PWM PPR: DZI01 Mask */

#define PWM_PPR_CP23_Pos                        8                                   /*!< PWM PPR: CP23 Position */
#define PWM_PPR_CP23_Msk                        (0xFFul << PWM_PPR_CP23_Pos)        /*!< PWM PPR: CP23 Mask */

#define PWM_PPR_CP01_Pos                        0                                   /*!< PWM PPR: CP01 Position */
#define PWM_PPR_CP01_Msk                        (0xFFul << PWM_PPR_CP01_Pos)        /*!< PWM PPR: CP01 Mask */

/* PWM CSR Bit Field Definitions */
#define PWM_CSR_CSR3_Pos                        12                                  /*!< PWM CSR: CSR3 Position */
#define PWM_CSR_CSR3_Msk                        (7ul << PWM_CSR_CSR3_Pos)           /*!< PWM CSR: CSR3 Mask */

#define PWM_CSR_CSR2_Pos                        8                                   /*!< PWM CSR: CSR2 Position */
#define PWM_CSR_CSR2_Msk                        (7ul << PWM_CSR_CSR2_Pos)           /*!< PWM CSR: CSR2 Mask */

#define PWM_CSR_CSR1_Pos                        4                                   /*!< PWM CSR: CSR1 Position */
#define PWM_CSR_CSR1_Msk                        (7ul << PWM_CSR_CSR1_Pos)           /*!< PWM CSR: CSR1 Mask */

#define PWM_CSR_CSR0_Pos                        0                                   /*!< PWM CSR: CSR0 Position */
#define PWM_CSR_CSR0_Msk                        (7ul << PWM_CSR_CSR0_Pos)           /*!< PWM CSR: CSR0 Mask */

/* PWM PCR Bit Field Definitions */
#define PWM_PCR_PWM23TYPE_Pos                   31                                  /*!< PWM PCR: PWM23TYPE Position */
#define PWM_PCR_PWM23TYPE_Msk                   (1ul << PWM_PCR_PWM23TYPE_Pos)      /*!< PWM PCR: PWM23TYPE Mask */

#define PWM_PCR_PWM01TYPE_Pos                   30                                  /*!< PWM PCR: PWM01TYPE Position */
#define PWM_PCR_PWM01TYPE_Msk                   (1ul << PWM_PCR_PWM01TYPE_Pos)      /*!< PWM PCR: PWM01TYPE Mask */

#define PWM_PCR_CH3MOD_Pos                      27                                  /*!< PWM PCR: CH3MOD Position */
#define PWM_PCR_CH3MOD_Msk                      (1ul << PWM_PCR_CH3MOD_Pos)         /*!< PWM PCR: CH3MOD Mask */

#define PWM_PCR_CH3INV_Pos                      26                                  /*!< PWM PCR: CH3INV Position */
#define PWM_PCR_CH3INV_Msk                      (1ul << PWM_PCR_CH3INV_Pos)         /*!< PWM PCR: CH3INV Mask */

#define PWM_PCR_CH3PINV_Pos                     25                                  /*!< PWM PCR: CH3PINV Position */
#define PWM_PCR_CH3PINV_Msk                     (1ul << PWM_PCR_CH3PINV_Pos)        /*!< PWM PCR: CH3PINV Mask */

#define PWM_PCR_CH3EN_Pos                       24                                  /*!< PWM PCR: CH3EN Position */
#define PWM_PCR_CH3EN_Msk                       (1ul << PWM_PCR_CH3EN_Pos)          /*!< PWM PCR: CH3EN Mask */

#define PWM_PCR_CH2MOD_Pos                      19                                  /*!< PWM PCR: CH2MOD Position */
#define PWM_PCR_CH2MOD_Msk                      (1ul << PWM_PCR_CH2MOD_Pos)         /*!< PWM PCR: CH2MOD Mask */

#define PWM_PCR_CH2INV_Pos                      18                                  /*!< PWM PCR: CH2INV Position */
#define PWM_PCR_CH2INV_Msk                      (1ul << PWM_PCR_CH2INV_Pos)         /*!< PWM PCR: CH2INV Mask */

#define PWM_PCR_CH2PINV_Pos                     17                                  /*!< PWM PCR: CH2PINV Position */
#define PWM_PCR_CH2PINV_Msk                     (1ul << PWM_PCR_CH2PINV_Pos)        /*!< PWM PCR: CH2PINV Mask */

#define PWM_PCR_CH2EN_Pos                       16                                  /*!< PWM PCR: CH2EN Position */
#define PWM_PCR_CH2EN_Msk                       (1ul << PWM_PCR_CH2EN_Pos)          /*!< PWM PCR: CH2EN Mask */

#define PWM_PCR_CH1MOD_Pos                      11                                  /*!< PWM PCR: CH1MOD Position */
#define PWM_PCR_CH1MOD_Msk                      (1ul << PWM_PCR_CH1MOD_Pos)         /*!< PWM PCR: CH1MOD Mask */

#define PWM_PCR_CH1INV_Pos                      10                                  /*!< PWM PCR: CH1INV Position */
#define PWM_PCR_CH1INV_Msk                      (1ul << PWM_PCR_CH1INV_Pos)         /*!< PWM PCR: CH1INV Mask */

#define PWM_PCR_CH1PINV_Pos                     9                                   /*!< PWM PCR: CH1PINV Position */
#define PWM_PCR_CH1PINV_Msk                     (1ul << PWM_PCR_CH1PINV_Pos)        /*!< PWM PCR: CH1PINV Mask */

#define PWM_PCR_CH1EN_Pos                       8                                   /*!< PWM PCR: CH1EN Position */
#define PWM_PCR_CH1EN_Msk                       (1ul << PWM_PCR_CH1EN_Pos)          /*!< PWM PCR: CH1EN Mask */

#define PWM_PCR_DZEN23_Pos                      5                                   /*!< PWM PCR: DZEN23 Position */
#define PWM_PCR_DZEN23_Msk                      (1ul << PWM_PCR_DZEN23_Pos)         /*!< PWM PCR: DZEN23 Mask */

#define PWM_PCR_DZEN01_Pos                      4                                   /*!< PWM PCR: DZEN01 Position */
#define PWM_PCR_DZEN01_Msk                      (1ul << PWM_PCR_DZEN01_Pos)         /*!< PWM PCR: DZEN01 Mask */

#define PWM_PCR_CH0MOD_Pos                      3                                   /*!< PWM PCR: CH0MOD Position */
#define PWM_PCR_CH0MOD_Msk                      (1ul << PWM_PCR_CH0MOD_Pos)         /*!< PWM PCR: CH0MOD Mask */

#define PWM_PCR_CH0INV_Pos                      2                                   /*!< PWM PCR: CH0INV Position */
#define PWM_PCR_CH0INV_Msk                      (1ul << PWM_PCR_CH0INV_Pos)         /*!< PWM PCR: CH0INV Mask */

#define PWM_PCR_CH0PINV_Pos                     1                                   /*!< PWM PCR: CH0PINV Position */
#define PWM_PCR_CH0PINV_Msk                     (1ul << PWM_PCR_CH0PINV_Pos)        /*!< PWM PCR: CH0PINV Mask */

#define PWM_PCR_CH0EN_Pos                       0                                   /*!< PWM PCR: CH0EN Position */
#define PWM_PCR_CH0EN_Msk                       (1ul << PWM_PCR_CH0EN_Pos)          /*!< PWM PCR: CH0EN Mask */

/* PWM CNR Bit Field Definitions */
#define PWM_CNR_CNR_Pos                         0                                   /*!< PWM CNR: CNR Position */
#define PWM_CNR_CNR_Msk                         (0xFFFFul << PWM_CNR_CNR_Pos)       /*!< PWM CNR: CNR Mask */

/* PWM CMR Bit Field Definitions */
#define PWM_CMR_CMR_Pos                         0                                   /*!< PWM CMR: CMR Position */
#define PWM_CMR_CMR_Msk                         (0xFFFFul << PWM_CMR_CMR_Pos)       /*!< PWM CMR: CMR Mask */

/* PWM PDR Bit Field Definitions */
#define PWM_PDR_PDR_Pos                         0                                   /*!< PWM PDR: PDR Position */
#define PWM_PDR_PDR_Msk                         (0xFFFFul << PWM_PDR_PDR_Pos)       /*!< PWM PDR: PDR Mask */

/* PWM PIER Bit Field Definitions */

#define PWM_PIER_INT23TYPE_Pos                  17                                  /*!< PWM PIER: INT23TYPE Position */
#define PWM_PIER_INT23TYPE_Msk                  (1ul << PWM_PIER_INT23TYPE_Pos)     /*!< PWM PIER: INT23TYPE Mask */

#define PWM_PIER_INT01TYPE_Pos                  16                                  /*!< PWM PIER: INT01TYPE Position */
#define PWM_PIER_INT01TYPE_Msk                  (1ul << PWM_PIER_INT01TYPE_Pos)     /*!< PWM PIER: INT01TYPE Mask */

#define PWM_PIER_PWMDIE3_Pos                    11                                  /*!< PWM PIER: PWMDIE3 Position */
#define PWM_PIER_PWMDIE3_Msk                    (1ul << PWM_PIER_PWMDIE3_Pos)       /*!< PWM PIER: PWMDIE3 Mask */

#define PWM_PIER_PWMDIE2_Pos                    10                                  /*!< PWM PIER: PWMDIE2 Position */
#define PWM_PIER_PWMDIE2_Msk                    (1ul << PWM_PIER_PWMDIE2_Pos)       /*!< PWM PIER: PWMDIE2 Mask */

#define PWM_PIER_PWMDIE1_Pos                    9                                   /*!< PWM PIER: PWMDIE1 Position */
#define PWM_PIER_PWMDIE1_Msk                    (1ul << PWM_PIER_PWMDIE1_Pos)       /*!< PWM PIER: PWMDIE1 Mask */

#define PWM_PIER_PWMDIE0_Pos                    8                                   /*!< PWM PIER: PWMDIE0 Position */
#define PWM_PIER_PWMDIE0_Msk                    (1ul << PWM_PIER_PWMDIE0_Pos)       /*!< PWM PIER: PWMDIE0 Mask */

#define PWM_PIER_PWMIE3_Pos                     3                                   /*!< PWM PIER: PWMIE3 Position */
#define PWM_PIER_PWMIE3_Msk                     (1ul << PWM_PIER_PWMIE3_Pos)        /*!< PWM PIER: PWMIE3 Mask */

#define PWM_PIER_PWMIE2_Pos                     2                                   /*!< PWM PIER: PWMIE2 Position */
#define PWM_PIER_PWMIE2_Msk                     (1ul << PWM_PIER_PWMIE2_Pos)        /*!< PWM PIER: PWMIE2 Mask */

#define PWM_PIER_PWMIE1_Pos                     1                                   /*!< PWM PIER: PWMIE1 Position */
#define PWM_PIER_PWMIE1_Msk                     (1ul << PWM_PIER_PWMIE1_Pos)        /*!< PWM PIER: PWMIE1 Mask */

#define PWM_PIER_PWMIE0_Pos                     0                                   /*!< PWM PIER: PWMIE0 Position */
#define PWM_PIER_PWMIE0_Msk                     (1ul << PWM_PIER_PWMIE0_Pos)        /*!< PWM PIER: PWMIE0 Mask */

/* PWM PIIR Bit Field Definitions */
#define PWM_PIIR_PWMDIF3_Pos                    11                                  /*!< PWM PIIR: PWMDIF3 Position */
#define PWM_PIIR_PWMDIF3_Msk                    (1ul << PWM_PIIR_PWMDIF3_Pos)       /*!< PWM PIIR: PWMDIF3 Mask */

#define PWM_PIIR_PWMDIF2_Pos                    10                                  /*!< PWM PIIR: PWMDIF2 Position */
#define PWM_PIIR_PWMDIF2_Msk                    (1ul << PWM_PIIR_PWMDIF2_Pos)       /*!< PWM PIIR: PWMDIF2 Mask */

#define PWM_PIIR_PWMDIF1_Pos                    9                                   /*!< PWM PIIR: PWMDIF1 Position */
#define PWM_PIIR_PWMDIF1_Msk                    (1ul << PWM_PIIR_PWMDIF1_Pos)       /*!< PWM PIIR: PWMDIF1 Mask */

#define PWM_PIIR_PWMDIF0_Pos                    8                                   /*!< PWM PIIR: PWMDIF0 Position */
#define PWM_PIIR_PWMDIF0_Msk                    (1ul << PWM_PIIR_PWMDIF0_Pos)       /*!< PWM PIIR: PWMDIF0 Mask */

#define PWM_PIIR_PWMIF3_Pos                     3                                   /*!< PWM PIIR: PWMIF3 Position */
#define PWM_PIIR_PWMIF3_Msk                     (1ul << PWM_PIIR_PWMIF3_Pos)        /*!< PWM PIIR: PWMIF3 Mask */

#define PWM_PIIR_PWMIF2_Pos                     2                                   /*!< PWM PIIR: PWMIF2 Position */
#define PWM_PIIR_PWMIF2_Msk                     (1ul << PWM_PIIR_PWMIF2_Pos)        /*!< PWM PIIR: PWMIF2 Mask */

#define PWM_PIIR_PWMIF1_Pos                     1                                   /*!< PWM PIIR: PWMIF1 Position */
#define PWM_PIIR_PWMIF1_Msk                     (1ul << PWM_PIIR_PWMIF1_Pos)        /*!< PWM PIIR: PWMIF1 Mask */

#define PWM_PIIR_PWMIF0_Pos                     0                                   /*!< PWM PIIR: PWMIF0 Position */
#define PWM_PIIR_PWMIF0_Msk                     (1ul << PWM_PIIR_PWMIF0_Pos)        /*!< PWM PIIR: PWMIF0 Mask */

/* PWM CCR0 Bit Field Definitions */
#define PWM_CCR0_CFLRI1_Pos                     23                                  /*!< PWM CCR0: CFLRI1 Position */
#define PWM_CCR0_CFLRI1_Msk                     (1ul << PWM_CCR0_CFLRI1_Pos)        /*!< PWM CCR0: CFLRI1 Mask */

#define PWM_CCR0_CRLRI1_Pos                     22                                  /*!< PWM CCR0: CRLRI1 Position */
#define PWM_CCR0_CRLRI1_Msk                     (1ul << PWM_CCR0_CRLRI1_Pos)        /*!< PWM CCR0: CRLRI1 Mask */

#define PWM_CCR0_CAPIF1_Pos                     20                                  /*!< PWM CCR0: CAPIF1 Position */
#define PWM_CCR0_CAPIF1_Msk                     (1ul << PWM_CCR0_CAPIF1_Pos)        /*!< PWM CCR0: CAPIF1 Mask */

#define PWM_CCR0_CAPCH1EN_Pos                   19                                  /*!< PWM CCR0: CAPCH1EN Position */
#define PWM_CCR0_CAPCH1EN_Msk                   (1ul << PWM_CCR0_CAPCH1EN_Pos)      /*!< PWM CCR0: CAPCH1EN Mask */

#define PWM_CCR0_CFL_IE1_Pos                    18                                  /*!< PWM CCR0: CFL_IE1 Position */
#define PWM_CCR0_CFL_IE1_Msk                    (1ul << PWM_CCR0_CFL_IE1_Pos)       /*!< PWM CCR0: CFL_IE1 Mask */

#define PWM_CCR0_CRL_IE1_Pos                    17                                  /*!< PWM CCR0: CRL_IE1 Position */
#define PWM_CCR0_CRL_IE1_Msk                    (1ul << PWM_CCR0_CRL_IE1_Pos)       /*!< PWM CCR0: CRL_IE1 Mask */

#define PWM_CCR0_INV1_Pos                       16                                  /*!< PWM CCR0: INV1 Position */
#define PWM_CCR0_INV1_Msk                       (1ul << PWM_CCR0_INV1_Pos)          /*!< PWM CCR0: INV1 Mask */

#define PWM_CCR0_CFLRI0_Pos                     7                                   /*!< PWM CCR0: CFLRI0 Position */
#define PWM_CCR0_CFLRI0_Msk                     (1ul << PWM_CCR0_CFLRI0_Pos)        /*!< PWM CCR0: CFLRI0 Mask */

#define PWM_CCR0_CRLRI0_Pos                     6                                   /*!< PWM CCR0: CRLRI0 Position */
#define PWM_CCR0_CRLRI0_Msk                     (1ul << PWM_CCR0_CRLRI0_Pos)        /*!< PWM CCR0: CRLRI0 Mask */

#define PWM_CCR0_CAPIF0_Pos                     4                                   /*!< PWM CCR0: CAPIF0 Position */
#define PWM_CCR0_CAPIF0_Msk                     (1ul << PWM_CCR0_CAPIF0_Pos)        /*!< PWM CCR0: CAPIF0 Mask */

#define PWM_CCR0_CAPCH0EN_Pos                   3                                   /*!< PWM CCR0: CAPCH0EN Position */
#define PWM_CCR0_CAPCH0EN_Msk                   (1ul << PWM_CCR0_CAPCH0EN_Pos)      /*!< PWM CCR0: CAPCH0EN Mask */

#define PWM_CCR0_CFL_IE0_Pos                    2                                   /*!< PWM CCR0: CFL_IE0 Position */
#define PWM_CCR0_CFL_IE0_Msk                    (1ul << PWM_CCR0_CFL_IE0_Pos)       /*!< PWM CCR0: CFL_IE0 Mask */

#define PWM_CCR0_CRL_IE0_Pos                    1                                   /*!< PWM CCR0: CRL_IE0 Position */
#define PWM_CCR0_CRL_IE0_Msk                    (1ul << PWM_CCR0_CRL_IE0_Pos)       /*!< PWM CCR0: CRL_IE0 Mask */

#define PWM_CCR0_INV0_Pos                       0                                   /*!< PWM CCR0: INV0 Position */
#define PWM_CCR0_INV0_Msk                       (1ul << PWM_CCR0_INV0_Pos)          /*!< PWM CCR0: INV0 Mask */

/* PWM CCR2 Bit Field Definitions */
#define PWM_CCR2_CFLRI3_Pos                     23                                  /*!< PWM CCR2: CFLRI3 Position */
#define PWM_CCR2_CFLRI3_Msk                     (1ul << PWM_CCR2_CFLRI3_Pos)        /*!< PWM CCR2: CFLRI3 Mask */

#define PWM_CCR2_CRLRI3_Pos                     22                                  /*!< PWM CCR2: CRLRI3 Position */
#define PWM_CCR2_CRLRI3_Msk                     (1ul << PWM_CCR2_CRLRI3_Pos)        /*!< PWM CCR2: CRLRI3 Mask */

#define PWM_CCR2_CAPIF3_Pos                     20                                  /*!< PWM CCR2: CAPIF3 Position */
#define PWM_CCR2_CAPIF3_Msk                     (1ul << PWM_CCR2_CAPIF3_Pos)        /*!< PWM CCR2: CAPIF3 Mask */

#define PWM_CCR2_CAPCH3EN_Pos                   19                                  /*!< PWM CCR2: CAPCH3EN Position */
#define PWM_CCR2_CAPCH3EN_Msk                   (1ul << PWM_CCR2_CAPCH3EN_Pos)      /*!< PWM CCR2: CAPCH3EN Mask */

#define PWM_CCR2_CFL_IE3_Pos                    18                                  /*!< PWM CCR2: CFL_IE3 Position */
#define PWM_CCR2_CFL_IE3_Msk                    (1ul << PWM_CCR2_CFL_IE3_Pos)       /*!< PWM CCR2: CFL_IE3 Mask */

#define PWM_CCR2_CRL_IE3_Pos                    17                                  /*!< PWM CCR2: CRL_IE3 Position */
#define PWM_CCR2_CRL_IE3_Msk                    (1ul << PWM_CCR2_CRL_IE3_Pos)       /*!< PWM CCR2: CRL_IE3 Mask */

#define PWM_CCR2_INV3_Pos                       16                                  /*!< PWM CCR2: INV3 Position */
#define PWM_CCR2_INV3_Msk                       (1ul << PWM_CCR2_INV3_Pos)          /*!< PWM CCR2: INV3 Mask */

#define PWM_CCR2_CFLRI2_Pos                     7                                   /*!< PWM CCR2: CFLRI2 Position */
#define PWM_CCR2_CFLRI2_Msk                     (1ul << PWM_CCR2_CFLRI2_Pos)        /*!< PWM CCR2: CFLRI2 Mask */

#define PWM_CCR2_CRLRI2_Pos                     6                                   /*!< PWM CCR2: CRLRI2 Position */
#define PWM_CCR2_CRLRI2_Msk                     (1ul << PWM_CCR2_CRLRI2_Pos)        /*!< PWM CCR2: CRLRI2 Mask */

#define PWM_CCR2_CAPIF2_Pos                     4                                   /*!< PWM CCR2: CAPIF2 Position */
#define PWM_CCR2_CAPIF2_Msk                     (1ul << PWM_CCR2_CAPIF2_Pos)        /*!< PWM CCR2: CAPIF2 Mask */

#define PWM_CCR2_CAPCH2EN_Pos                   3                                   /*!< PWM CCR2: CAPCH2EN Position */
#define PWM_CCR2_CAPCH2EN_Msk                   (1ul << PWM_CCR2_CAPCH2EN_Pos)      /*!< PWM CCR2: CAPCH2EN Mask */

#define PWM_CCR2_CFL_IE2_Pos                    2                                   /*!< PWM CCR2: CFL_IE2 Position */
#define PWM_CCR2_CFL_IE2_Msk                    (1ul << PWM_CCR2_CFL_IE2_Pos)       /*!< PWM CCR2: CFL_IE2 Mask */

#define PWM_CCR2_CRL_IE2_Pos                    1                                   /*!< PWM CCR2: CRL_IE2 Position */
#define PWM_CCR2_CRL_IE2_Msk                    (1ul << PWM_CCR2_CRL_IE2_Pos)       /*!< PWM CCR2: CRL_IE2 Mask */

#define PWM_CCR2_INV2_Pos                       0                                   /*!< PWM CCR2: INV2 Position */
#define PWM_CCR2_INV2_Msk                       (1ul << PWM_CCR2_INV2_Pos)          /*!< PWM CCR2: INV2 Mask */

/* PWM CRLR Bit Field Definitions */
#define PWM_CRLR_CRLR_Pos                       0                                   /*!< PWM CRLR: CRLR Position */
#define PWM_CRLR_CRLR_Msk                       (0xFFFFul << PWM_CRLR_CRLR_Pos)     /*!< PWM CRLR: CRLR Mask */

/* PWM CFLR Bit Field Definitions */
#define PWM_CFLR_CFLR_Pos                       0                                   /*!< PWM CFLR: CFLR Position */
#define PWM_CFLR_CFLR_Msk                       (0xFFFFul << PWM_CFLR_CFLR_Pos)     /*!< PWM CFLR: CFLR Mask */

/* PWM CAPENR Bit Field Definitions */
#define PWM_CAPENR_CINEN3_Pos                   3                                   /*!< PWM CAPENR: CINEN3 Position */
#define PWM_CAPENR_CINEN3_Msk                   (1ul << PWM_CAPENR_CINEN3_Pos)      /*!< PWM CAPENR: CINEN3 Mask */

#define PWM_CAPENR_CINEN2_Pos                   2                                   /*!< PWM CAPENR: CINEN2 Position */
#define PWM_CAPENR_CINEN2_Msk                   (1ul << PWM_CAPENR_CINEN2_Pos)      /*!< PWM CAPENR: CINEN2 Mask */

#define PWM_CAPENR_CINEN1_Pos                   1                                   /*!< PWM CAPENR: CINEN1 Position */
#define PWM_CAPENR_CINEN1_Msk                   (1ul << PWM_CAPENR_CINEN1_Pos)      /*!< PWM CAPENR: CINEN1 Mask */

#define PWM_CAPENR_CINEN0_Pos                   0                                   /*!< PWM CAPENR: CINEN0 Position */
#define PWM_CAPENR_CINEN0_Msk                   (1ul << PWM_CAPENR_CINEN0_Pos)      /*!< PWM CAPENR: CINEN0 Mask */

/* PWM POE Bit Field Definitions */
#define PWM_POE_POE3_Pos                        3                                   /*!< PWM POE: POE3 Position */
#define PWM_POE_POE3_Msk                        (1ul << PWM_POE_POE3_Pos)           /*!< PWM POE: POE3 Mask */

#define PWM_POE_POE2_Pos                        2                                   /*!< PWM POE: POE2 Position */
#define PWM_POE_POE2_Msk                        (1ul << PWM_POE_POE2_Pos)           /*!< PWM POE: POE2 Mask */

#define PWM_POE_POE1_Pos                        1                                   /*!< PWM POE: POE1 Position */
#define PWM_POE_POE1_Msk                        (1ul << PWM_POE_POE1_Pos)           /*!< PWM POE: POE1 Mask */

#define PWM_POE_POE0_Pos                        0                                   /*!< PWM POE: POE0 Position */
#define PWM_POE_POE0_Msk                        (1ul << PWM_POE_POE0_Pos)           /*!< PWM POE: POE0 Mask */

/* PWM TCON Bit Field Definitions */

#define PWM_TCON_PWM3TEN_Pos                    3                                   /*!< PWM TCON: PWM3TEN Position */
#define PWM_TCON_PWM3TEN_Msk                    (1ul << PWM_TCON_PWM3TEN_Pos)       /*!< PWM TCON: PWM3TEN Mask */

#define PWM_TCON_PWM2TEN_Pos                    2                                   /*!< PWM TCON: PWM2TEN Position */
#define PWM_TCON_PWM2TEN_Msk                    (1ul << PWM_TCON_PWM2TEN_Pos)       /*!< PWM TCON: PWM2TEN Mask */

#define PWM_TCON_PWM1TEN_Pos                    1                                   /*!< PWM TCON: PWM1TEN Position */
#define PWM_TCON_PWM1TEN_Msk                    (1ul << PWM_TCON_PWM1TEN_Pos)       /*!< PWM TCON: PWM1TEN Mask */

#define PWM_TCON_PWM0TEN_Pos                    0                                   /*!< PWM TCON: PWM0TEN Position */
#define PWM_TCON_PWM0TEN_Msk                    (1ul << PWM_TCON_PWM0TEN_Pos)       /*!< PWM TCON: PWM0TEN Mask */

/* PWM TSTATUS Bit Field Definitions */

#define PWM_TSTATUS_PWM3TF_Pos                  3                                   /*!< PWM TSTATUS: PWM3TF Position */
#define PWM_TSTATUS_PWM3TF_Msk                  (1ul << PWM_TSTATUS_PWM3TF_Pos)     /*!< PWM TSTATUS: PWM3TF Mask */

#define PWM_TSTATUS_PWM2TF_Pos                  2                                   /*!< PWM TSTATUS: PWM2TF Position */
#define PWM_TSTATUS_PWM2TF_Msk                  (1ul << PWM_TSTATUS_PWM2TF_Pos)     /*!< PWM TSTATUS: PWM2TF Mask */

#define PWM_TSTATUS_PWM1TF_Pos                  1                                   /*!< PWM TSTATUS: PWM1TF Position */
#define PWM_TSTATUS_PWM1TF_Msk                  (1ul << PWM_TSTATUS_PWM1TF_Pos)     /*!< PWM TSTATUS: PWM1TF Mask */

#define PWM_TSTATUS_PWM0TF_Pos                  0                                   /*!< PWM TSTATUS: PWM0TF Position */
#define PWM_TSTATUS_PWM0TF_Msk                  (1ul << PWM_TSTATUS_PWM0TF_Pos)     /*!< PWM TSTATUS: PWM0TF Mask */

/* PWM SYNCBUSY0 Bit Field Definitions */
#define PWM_SYNCBUSY0_S_BUSY_Pos                0                                   /*!< PWM SYNCBUSY0: S_BUSY Position */
#define PWM_SYNCBUSY0_S_BUSY_Msk                (1ul << PWM_SYNCBUSY0_S_BUSY_Pos)   /*!< PWM SYNCBUSY0: S_BUSY Mask */

/* PWM SYNCBUSY1 Bit Field Definitions */
#define PWM_SYNCBUSY1_S_BUSY_Pos                0                                   /*!< PWM SYNCBUSY1: S_BUSY Position */
#define PWM_SYNCBUSY1_S_BUSY_Msk                (1ul << PWM_SYNCBUSY1_S_BUSY_Pos)   /*!< PWM SYNCBUSY1: S_BUSY Mask */

/* PWM SYNCBUSY2 Bit Field Definitions */
#define PWM_SYNCBUSY2_S_BUSY_Pos                0                                   /*!< PWM SYNCBUSY2: S_BUSY Position */
#define PWM_SYNCBUSY2_S_BUSY_Msk                (1ul << PWM_SYNCBUSY2_S_BUSY_Pos)   /*!< PWM SYNCBUSY2: S_BUSY Mask */

/* PWM SYNCBUSY3 Bit Field Definitions */
#define PWM_SYNCBUSY3_S_BUSY_Pos                0                                   /*!< PWM SYNCBUSY3: S_BUSY Position */
#define PWM_SYNCBUSY3_S_BUSY_Msk                (1ul << PWM_SYNCBUSY3_S_BUSY_Pos)   /*!< PWM SYNCBUSY3: S_BUSY Mask */

/* PWM CAPPDMACTL Bit Field Definitions */
#define PWM_CAPPDMACTL_CAP3RFORDER_Pos          27                                        /*!< PWM CAPPDMACTL: CAP3RFORDER Position */
#define PWM_CAPPDMACTL_CAP3RFORDER_Msk          (1ul << PWM_CAPPDMACTL_CAP3RFORDER_Pos)   /*!< PWM CAPPDMACTL: CAP3RFORDER Mask */

#define PWM_CAPPDMACTL_CAP3PDMAMOD_Pos          25                                        /*!< PWM CAPPDMACTL: CAP3PDMAMOD Position */
#define PWM_CAPPDMACTL_CAP3PDMAMOD_Msk          (3ul << PWM_CAPPDMACTL_CAP3PDMAMOD_Pos)   /*!< PWM CAPPDMACTL: CAP3PDMAMOD Mask */

#define PWM_CAPPDMACTL_CAP3PDMAEN_Pos           24                                        /*!< PWM CAPPDMACTL: CAP3PDMAEN Position */
#define PWM_CAPPDMACTL_CAP3PDMAEN_Msk           (1ul << PWM_CAPPDMACTL_CAP3PDMAEN_Pos)    /*!< PWM CAPPDMACTL: CAP3PDMAEN Mask */

#define PWM_CAPPDMACTL_CAP2RFORDER_Pos          19                                        /*!< PWM CAPPDMACTL: CAP2RFORDER Position */
#define PWM_CAPPDMACTL_CAP2RFORDER_Msk          (1ul << PWM_CAPPDMACTL_CAP2RFORDER_Pos)   /*!< PWM CAPPDMACTL: CAP2RFORDER Mask */

#define PWM_CAPPDMACTL_CAP2PDMAMOD_Pos          17                                        /*!< PWM CAPPDMACTL: CAP2PDMAMOD Position */
#define PWM_CAPPDMACTL_CAP2PDMAMOD_Msk          (3ul << PWM_CAPPDMACTL_CAP2PDMAMOD_Pos)   /*!< PWM CAPPDMACTL: CAP2PDMAMOD Mask */

#define PWM_CAPPDMACTL_CAP2PDMAEN_Pos           16                                        /*!< PWM CAPPDMACTL: CAP2PDMAEN Position */
#define PWM_CAPPDMACTL_CAP2PDMAEN_Msk           (1ul << PWM_CAPPDMACTL_CAP2PDMAEN_Pos)    /*!< PWM CAPPDMACTL: CAP2PDMAEN Mask */

#define PWM_CAPPDMACTL_CAP1RFORDER_Pos          11                                        /*!< PWM CAPPDMACTL: CAP1RFORDER Position */
#define PWM_CAPPDMACTL_CAP1RFORDER_Msk          (1ul << PWM_CAPPDMACTL_CAP1RFORDER_Pos)   /*!< PWM CAPPDMACTL: CAP1RFORDER Mask */

#define PWM_CAPPDMACTL_CAP1PDMAMOD_Pos          9                                         /*!< PWM CAPPDMACTL: CAP1PDMAMOD Position */
#define PWM_CAPPDMACTL_CAP1PDMAMOD_Msk          (3ul << PWM_CAPPDMACTL_CAP1PDMAMOD_Pos)   /*!< PWM CAPPDMACTL: CAP1PDMAMOD Mask */

#define PWM_CAPPDMACTL_CAP1PDMAEN_Pos           8                                         /*!< PWM CAPPDMACTL: CAP1PDMAEN Position */
#define PWM_CAPPDMACTL_CAP1PDMAEN_Msk           (1ul << PWM_CAPPDMACTL_CAP1PDMAEN_Pos)    /*!< PWM CAPPDMACTL: CAP1PDMAEN Mask */

#define PWM_CAPPDMACTL_CAP0RFORDER_Pos          3                                         /*!< PWM CAPPDMACTL: CAP0RFORDER Position */
#define PWM_CAPPDMACTL_CAP0RFORDER_Msk          (1ul << PWM_CAPPDMACTL_CAP0RFORDER_Pos)   /*!< PWM CAPPDMACTL: CAP0RFORDER Mask */

#define PWM_CAPPDMACTL_CAP0PDMAMOD_Pos          1                                         /*!< PWM CAPPDMACTL: CAP0PDMAMOD Position */
#define PWM_CAPPDMACTL_CAP0PDMAMOD_Msk          (3ul << PWM_CAPPDMACTL_CAP0PDMAMOD_Pos)   /*!< PWM CAPPDMACTL: CAP0PDMAMOD Mask */

#define PWM_CAPPDMACTL_CAP0PDMAEN_Pos           0                                         /*!< PWM CAPPDMACTL: CAP0PDMAEN Position */
#define PWM_CAPPDMACTL_CAP0PDMAEN_Msk           (1ul << PWM_CAPPDMACTL_CAP0PDMAEN_Pos)    /*!< PWM CAPPDMACTL: CAP0PDMAEN Mask */

/* PWM CAP0PDMA Bit Field Definitions */
#define PWM_CAP0PDMA_CAP0RFPDMA_Pos             0                                         /*!< PWM CAP0PDMA: CAP0RFPDMA Position */
#define PWM_CAP0PDMA_CAP0RFPDMA_Msk             (0xFFFFul << PWM_CAP0PDMA_CAP0RFPDMA_Pos) /*!< PWM CAP0PDMA: CAP0RFPDMA Mask */

/* PWM CAP1PDMA Bit Field Definitions */
#define PWM_CAP1PDMA_CAP1RFPDMA_Pos             0                                         /*!< PWM CAP1PDMA: CAP1RFPDMA Position */
#define PWM_CAP1PDMA_CAP1RFPDMA_Msk             (0xFFFFul << PWM_CAP1PDMA_CAP1RFPDMA_Pos) /*!< PWM CAP1PDMA: CAP1RFPDMA Mask */

/* PWM CAP2PDMA Bit Field Definitions */
#define PWM_CAP2PDMA_CAP2RFPDMA_Pos             0                                         /*!< PWM CAP2PDMA: CAP2RFPDMA Position */
#define PWM_CAP2PDMA_CAP2RFPDMA_Msk             (0xFFFFul << PWM_CAP2PDMA_CAP2RFPDMA_Pos) /*!< PWM CAP2PDMA: CAP2RFPDMA Mask */

/* PWM CAP3PDMA Bit Field Definitions */
#define PWM_CAP3PDMA_CAP3RFPDMA_Pos             0                                         /*!< PWM CAP3PDMA: CAP3RFPDMA Position */
#define PWM_CAP3PDMA_CAP3RFPDMA_Msk             (0xFFFFul << PWM_CAP3PDMA_CAP3RFPDMA_Pos) /*!< PWM CAP3PDMA: CAP3RFPDMA Mask */
/*@}*/ /* end of group REG_PWM_BITMASK */
/*@}*/ /* end of group REG_PWM */

/*------------------------- SPI Interface Controller -------------------------*/
/** @addtogroup REG_SPI SPI
  Memory Mapped Structure for SPI Controller
  @{
 */
typedef struct
{
    /**
     * SPI_CNTRL
     * ===================================================================================================
     * Offset: 0x00  Control and Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |GO_BUSY   |SPI Transfer Control Bit And Busy Status
     * |        |          |0 = Data transfer stopped.
     * |        |          |1 = In Master mode, writing 1 to this bit to start the SPI data transfer; in Slave mode,
     * |        |          |    writing 1 to this bit indicates that the slave is ready to communicate with a master.
     * |        |          |If FIFO mode is disabled, during the data transfer, this bit keeps the value of 1.
     * |        |          |As the transfer is finished, this bit will be cleared automatically.
     * |        |          |Software can read this bit to check if the SPI is in busy status.
     * |        |          |In FIFO mode, this bit will be controlled by hardware.
     * |        |          |Software should not modify this bit.
     * |        |          |In Slave mode, this bit always returns 1 when this register is read by software.
     * |        |          |In Master mode, this bit reflects the busy or idle status of SPI.
     * |        |          |Note:
     * |        |          |1. When FIFO mode is disabled, all configurations should be set before writing 1 to this GO_BUSY bit.
     * |        |          |2. When FIFO mode is disabled and the software uses TX or RX PDMA function to transfer data, this bit
     * |        |          |   will be cleared after the PDMA finishes the data transfer.
     * |[1]     |RX_NEG    |Receive On Negative Edge
     * |        |          |0 = Received data input signal is latched on the rising edge of SPI bus clock.
     * |        |          |1 = Received data input signal is latched on the falling edge of SPI bus clock.
     * |[2]     |TX_NEG    |Transmit On Negative Edge
     * |        |          |0 = Transmitted data output signal is changed on the rising edge of SPI bus clock.
     * |        |          |1 = Transmitted data output signal is changed on the falling edge of SPI bus clock.
     * |[7:3]   |TX_BIT_LEN|Transmit Bit Length
     * |        |          |This field specifies how many bits can be transmitted / received in one transaction.
     * |        |          |The minimum bit length is 8 bits and can up to 32 bits.
     * |        |          |TX_BIT_LEN = 0x08 ... 8 bits.
     * |        |          |TX_BIT_LEN = 0x09 ... 9 bits.
     * |        |          |......
     * |        |          |TX_BIT_LEN = 0x1F ... 31 bits.
     * |        |          |TX_BIT_LEN = 0x00 ... 32 bits.
     * |[10]    |LSB       |Send LSB First
     * |        |          |0 = The MSB, which bit of transmit/receive register depends on the setting of TX_BIT_LEN, is transmitted/received first.
     * |        |          |1 = The LSB, bit 0 of the SPI TX0/1 register, is sent first to the SPI data output pin, and the first bit received from
     * |        |          |    the SPI data input pin will be put in the LSB position of the RX register (bit 0 of SPI_RX0/1).
     * |[11]    |CLKP      |Clock Polarity
     * |        |          |0 = SPI bus clock is idle low.
     * |        |          |1 = SPI bus clock is idle high.
     * |[15:12] |SP_CYCLE  |Suspend Interval (Master Only)
     * |        |          |The four bits provide configurable suspend interval between two successive transmit/receive transaction in a transfer.
     * |        |          |The definition of the suspend interval is the interval between the last clock edge of the preceding transaction word
     * |        |          |and the first clock edge of the following transaction word.
     * |        |          |The default value is 0x3.
     * |        |          |The period of the suspend interval is obtained according to the following equation.
     * |        |          |(SP_CYCLE[3:0] + 0.5) * period of SPI bus clock cycle
     * |        |          |Example:
     * |        |          |SP_CYCLE = 0x0 ... 0.5 SPI bus clock cycle.
     * |        |          |SP_CYCLE = 0x1 ... 1.5 SPI bus clock cycle.
     * |        |          |......
     * |        |          |SP_CYCLE = 0xE ... 14.5 SPI bus clock cycle.
     * |        |          |SP_CYCLE = 0xF ... 15.5 SPI bus clock cycle.
     * |        |          |If the variable clock function is enabled and the transmit FIFO buffer is not empty, the minimum period of suspend
     * |        |          |interval between the successive transactions is (6.5 + SP_CYCLE) * SPI bus clock cycle.
     * |[16]    |IF        |Unit Transfer Interrupt Flag
     * |        |          |0 = No transaction has been finished since this bit was cleared to 0.
     * |        |          |1 = SPI controller has finished one unit transfer.
     * |        |          |Note: This bit will be cleared by writing 1 to itself.
     * |[17]    |IE        |Unit Transfer Interrupt Enable
     * |        |          |0 = SPI unit transfer interrupt Disabled.
     * |        |          |1 = SPI unit transfer interrupt Enabled.
     * |[18]    |SLAVE     |Slave Mode Enable
     * |        |          |0 = Master mode.
     * |        |          |1 = Slave mode.
     * |[19]    |REORDER   |Byte Reorder Function Enable
     * |        |          |0 = Byte Reorder function Disabled.
     * |        |          |1 = Byte Reorder function Enabled.
     * |        |          |A byte suspend interval will be inserted among each byte.
     * |        |          |The period of the byte suspend interval depends on the setting of SP_CYCLE.
     * |        |          |Note:
     * |        |          |1. Byte Reorder function is only available if TX_BIT_LEN is defined as 16, 24, and 32 bits.
     * |        |          |2. In Slave mode with level-trigger configuration, the slave select pin must be kept at active state during the
     * |        |          |   byte suspend interval.
     * |        |          |3. The Byte Reorder function is not supported when the variable bus clock function or Dual I/O mode is enabled.
     * |[21]    |FIFO      |FIFO Mode Enable
     * |        |          |0 = FIFO mode Disabled.
     * |        |          |1 = FIFO mode Enabled.
     * |        |          |Note:
     * |        |          |1. Before enabling FIFO mode, the other related settings should be set in advance.
     * |        |          |2. In Master mode, if the FIFO mode is enabled, the GO_BUSY bit will be set to 1 automatically after writing data
     * |        |          |   to the transmit FIFO buffer; the GO_BUSY bit will be cleared to 0 automatically when the SPI controller is in idle.
     * |        |          |   If all data stored at transmit FIFO buffer are sent out, the TX_EMPTY bit will be set to 1 and the GO_BUSY bit will be cleared to 0.
     * |        |          |3. After clearing this bit to 0, user must wait for at least 2 peripheral clock periods before setting this bit to 1 again.
     * |[22]    |TWOB      |2-Bit Transfer Mode Enable
     * |        |          |0 = 2-bit Transfer mode Disabled.
     * |        |          |1 = 2-bit Transfer mode Enabled.
     * |        |          |Note: When 2-bit Transfer mode is enabled, the serial transmitted 2-bit data are from SPI_TX1/0, and the received 2-bit data input are put in SPI_RX1/0.
     * |[23]    |VARCLK_EN |Variable Clock Enable (Master Only)
     * |        |          |0 = SPI clock output frequency is fixed and decided only by the value of DIVIDER.
     * |        |          |1 = SPI clock output frequency is variable.
     * |        |          |The output frequency is decided by the value of VARCLK, DIVIDER, and DIVIDER2.
     * |        |          |Note: When this VARCLK_EN bit is set to 1, the setting of TX_BIT_LEN must be programmed as 0x10 (16-bit mode).
     * |[24]    |RX_EMPTY  |Receive FIFO Buffer Empty Indicator (Read Only)
     * |        |          |It is a mutual mirror bit of SPI_CNTRL[24].
     * |        |          |0 = Receive FIFO buffer is not empty.
     * |        |          |1 = Receive FIFO buffer is empty.
     * |[25]    |RX_FULL   |Receive FIFO Buffer Full Indicator (Read Only)
     * |        |          |It is a mutual mirror bit of SPI_STATUS[25].
     * |        |          |0 = Receive FIFO buffer is not full.
     * |        |          |1 = Receive FIFO buffer is full.
     * |[26]    |TX_EMPTY  |Transmit FIFO Buffer Empty Indicator (Read Only)
     * |        |          |It is a mutual mirror bit of SPI_STATUS[26].
     * |        |          |0 = Transmit FIFO buffer is not empty.
     * |        |          |1 = Transmit FIFO buffer is empty.
     * |[27]    |TX_FULL   |Transmit FIFO Buffer Full Indicator (Read Only)
     * |        |          |It is a mutual mirror bit of SPI_STATUS[27].
     * |        |          |0 = Transmit FIFO buffer is not full.
     * |        |          |1 = Transmit FIFO buffer is full.
     */
    __IO uint32_t CNTRL;

    /**
     * SPI_DIVIDER
     * ===================================================================================================
     * Offset: 0x04  Clock Divider Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |DIVIDER   |Clock Divider 1 Register
     * |        |          |The value in this field is the frequency divider for generating the SPI peripheral clock and the SPI bus clock of SPI master.
     * |        |          |The frequency is obtained according to the following equation.
     * |        |          |If the bit of BCn, SPI_CNTRL2[31], is set to 0,
     * |        |          |   SPI peripheral clock frequency = system clock frequency / (DIVIDER + 1) / 2
     * |        |          |else if BCn is set to 1,
     * |        |          |   SPI peripheral clock frequency = SPI peripheral clock source frequency / (DIVIDER + 1)
     * |        |          |The SPI peripheral clock source is defined in the CLKSEL1 register.
     * |[23:16] |DIVIDER2  |Clock Divider 2 Register (Master Only)
     * |        |          |The value in this field is the 2nd frequency divider for generating the second clock of the variable clock function.
     * |        |          |The frequency is obtained according to the following equation:
     * |        |          |   f_clk2 = SPI peripheral clock frequency / (DIVIDER2 + 1) / 2
     * |        |          |If the VARCLK_EN bit is cleared to 0, this setting is unmeaning.
     */
    __IO uint32_t DIVIDER;

    /**
     * SPI_SSR
     * ===================================================================================================
     * Offset: 0x08  Slave Select Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1:0]   |SSR       |Slave Select Control Bits (Master Only)
     * |        |          |If AUTOSS bit is cleared, writing 1 to any bit of this field sets the proper SPIn_SPISS0/1
     * |        |          |line to an active state and writing 0 sets the line back to inactive state.
     * |        |          |If the AUTOSS bit is set, writing 0 to any bit location of this field will keep the corresponding
     * |        |          |SPIn_SPISS0/1 line at inactive state; writing 1 to any bit location of this field will select
     * |        |          |appropriate SPIn_SPISS0/1 line to be automatically driven to active state for the duration of the
     * |        |          |transmit/receive, and will be driven to inactive state for the rest of the time.
     * |        |          |The active state of SPIn_SPISS0/1 is specified in SS_LVL.
     * |        |          |Note: SPIn_SPISS0 is defined as the slave select input in Slave mode.
     * |[2]     |SS_LVL    |Slave Select Active Level
     * |        |          |This bit defines the active status of slave select signal (SPIn_SPISS0/1).
     * |        |          |0 = The slave select signal SPIn_SPISS0/1 is active on low-level/falling-edge.
     * |        |          |1 = The slave select signal SPIn_SPISS0/1 is active on high-level/rising-edge.
     * |[3]     |AUTOSS    |Automatic Slave Select Function Enable (Master Only)
     * |        |          |0 = If this bit is cleared, slave select signals will be asserted/de-asserted by setting /clearing
     * |        |          |    the corresponding bits of SPI_SSR[1:0].
     * |        |          |1 = If this bit is set, SPIn_SPISS0/1 signals will be generated automatically.
     * |        |          |    It means that device/slave select signal, which is set in SPI_SSR[1:0], will be asserted by the
     * |        |          |    SPI controller when transmit/receive is started, and will be de-asserted after each transmit/receive is finished.
     * |[4]     |SS_LTRIG  |Slave Select Level Trigger Enable (Slave Only)
     * |        |          |0 = Slave select signal is edge-trigger.
     * |        |          |    This is the default value.
     * |        |          |    The SS_LVL bit decides the signal is active after a falling-edge or rising-edge.
     * |        |          |1 = Slave select signal is level-trigger.
     * |        |          |    The SS_LVL bit decides the signal is active low or active high.
     * |[5]     |LTRIG_FLAG|Level Trigger Accomplish Flag
     * |        |          |In Slave mode, this bit indicates whether the received bit number meets the requirement or not after the current transaction done.
     * |        |          |0 = Transferred bit length of one transaction does not meet the specified requirement.
     * |        |          |1 = Transferred bit length meets the specified requirement which defined in TX_BIT_LEN.
     * |        |          |Note: This bit is READ only.
     * |        |          |As the GO_BUSY bit is set to 1 by software, the LTRIG_FLAG will be cleared to 0 after 4 SPI peripheral clock periods plus 1 system clock period.
     * |        |          |In FIFO mode, this bit has no meaning.
     */
    __IO uint32_t SSR;

    /**
     * @cond HIDDEN_SYMBOLS
     */
    __I  uint32_t RESERVE0;
    /**
     * @endcond
     */

    /**
     * SPI_RX0
     * ===================================================================================================
     * Offset: 0x10  Data Receive Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |RX        |Data Receive Register
     * |        |          |The data receive register holds the datum received from SPI data input pin.
     * |        |          |If FIFO mode is disabled, the last received data can be accessed through software by reading this register.
     * |        |          |If the FIFO bit is set as 1 and the RX_EMPTY bit, SPI_CNTRL[24] or SPI_STATUS[24], is not set to 1, the receive
     * |        |          |FIFO buffer can be accessed through software by reading this register. This is a read-only register.
     */
    /**
     * SPI_RX1
     * ===================================================================================================
     * Offset: 0x14  Data Receive Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |RX        |Data Receive Register
     * |        |          |The data receive register holds the datum received from SPI data input pin.
     * |        |          |If FIFO mode is disabled, the last received data can be accessed through software by reading this register.
     * |        |          |If the FIFO bit is set as 1 and the RX_EMPTY bit, SPI_CNTRL[24] or SPI_STATUS[24], is not set to 1, the receive
     * |        |          |FIFO buffer can be accessed through software by reading this register. This is a read-only register.
     */
    __I  uint32_t RX[2];

    /**
     * @cond HIDDEN_SYMBOLS
     */
    __I  uint32_t RESERVE1;
    __I  uint32_t RESERVE2;
    /**
     * @endcond
     */

    /**
     * SPI_TX0
     * ===================================================================================================
     * Offset: 0x20  Data Transmit Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |TX        |Data Transmit Register
     * |        |          |The data transmit registers hold the data to be transmitted in the next transfer.
     * |        |          |The number of valid bits depends on the setting of transmit bit length field of the SPI_CNTRL register.
     * |        |          |For example, if TX_BIT_LEN is set to 0x08, the bits TX[7:0] will be transmitted in next transfer.
     * |        |          |If TX_BIT_LEN is set to 0x00, the SPI controller will perform a 32-bit transfer.
     * |        |          |Note 1: When the SPI controller is configured as a slave device and FIFO mode is disabled, if the SPI
     * |        |          |        controller attempts to transmit data to a master, the transmit data register should be updated
     * |        |          |        by software before setting the GO_BUSY bit to 1.
     * |        |          |Note 2: In Master mode, SPI controller will start to transfer after 5 peripheral clock cycles after user writes to this register.
     */
    /**
     * SPI_TX1
     * ===================================================================================================
     * Offset: 0x24  Data Transmit Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |TX        |Data Transmit Register
     * |        |          |The data transmit registers hold the data to be transmitted in the next transfer.
     * |        |          |The number of valid bits depends on the setting of transmit bit length field of the SPI_CNTRL register.
     * |        |          |For example, if TX_BIT_LEN is set to 0x08, the bits TX[7:0] will be transmitted in next transfer.
     * |        |          |If TX_BIT_LEN is set to 0x00, the SPI controller will perform a 32-bit transfer.
     * |        |          |Note 1: When the SPI controller is configured as a slave device and FIFO mode is disabled, if the SPI
     * |        |          |        controller attempts to transmit data to a master, the transmit data register should be updated
     * |        |          |        by software before setting the GO_BUSY bit to 1.
     * |        |          |Note 2: In Master mode, SPI controller will start to transfer after 5 peripheral clock cycles after user writes to this register.
     */
    __O  uint32_t TX[2];

    /**
     * @cond HIDDEN_SYMBOLS
     */
    __I  uint32_t RESERVE3;
    __I  uint32_t RESERVE4;
    __I  uint32_t RESERVE5;
    /**
     * @endcond
     */

    /**
     * SPI_VARCLK
     * ===================================================================================================
     * Offset: 0x34  Variable Clock Pattern Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |VARCLK    |Variable Clock Pattern
     * |        |          |This register defines the clock pattern of the SPI transfer.
     * |        |          |If the variable clock function is disabled, this setting is unmeaning.
     */
    __IO uint32_t VARCLK;

    /**
     * SPI_DMA
     * ===================================================================================================
     * Offset: 0x38  SPI DMA Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |TX_DMA_GO |Transmit DMA Start
     * |        |          |Setting this bit to 1 will start the transmit PDMA process.
     * |        |          |SPI controller will issue request to PDMA controller automatically.
     * |        |          |Hardware will clear this bit to 0 automatically after PDMA transfer done.
     * |        |          |If the SPI transmit PDMA function is used to transfer data, the GO_BUSY bit should not be set to 1 by software.
     * |        |          |The PDMA control logic of SPI controller will set it automatically whenever necessary.
     * |        |          |In Slave mode and when FIFO mode is disabled, the minimal suspend interval between two successive transactions
     * |        |          |must be larger than (8 SPI clock periods + 14 APB clock periods) for edge-trigger mode or
     * |        |          |(9.5 SPI clock periods + 14 APB clock periods) for level-trigger mode.
     * |        |          |If the 2-bit Transfer mode is enabled, additional 18 APB clock periods for the above conditions is required.
     * |[1]     |RX_DMA_GO |Receive DMA Start
     * |        |          |Setting this bit to 1 will start the receive PDMA process.
     * |        |          |The SPI controller will issue request to PDMA controller automatically when the SPI receive buffer is not empty.
     * |        |          |This bit will be cleared to 0 by hardware automatically after PDMA transfer is done.
     * |        |          |If the software uses the receive PDMA function to access the received data of SPI and does not use the transmit
     * |        |          |PDMA function, the GO_BUSY bit should be set by software.
     * |        |          |Enabling FIFO mode is recommended if the software uses more than one PDMA channel to transfer data.
     * |        |          |In Slave mode and when FIFO mode is disabled, if the software only uses one PDMA channel for SPI receive PDMA
     * |        |          |function and the other PDMA channels are not in use, the minimal suspend interval between two successive
     * |        |          |transactions must be larger than (9 SPI slave peripheral clock periods + 4 APB clock periods) for Edge-trigger
     * |        |          |mode or (9.5 SPI slave peripheral clock periods + 4 APB clock periods) for Level-trigger mode.
     * |[2]     |PDMA_RST  |PDMA Reset
     * |        |          |0 = No effect.
     * |        |          |1 = Reset the PDMA control logic of the SPI controller. This bit will be cleared to 0 automatically.
     */
    __IO uint32_t DMA;

    /**
     * SPI_CNTRL2
     * ===================================================================================================
     * Offset: 0x3C  Control and Status Register 2
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[8]     |NOSLVSEL  |Slave 3-Wire Mode Enable
     * |        |          |This is used to ignore the slave select signal in Slave mode.
     * |        |          |The SPI controller can work with 3-wire interface including SPIn_CLK, SPIn_MISO, and SPIn_MOSI.
     * |        |          |0 = 4-wire bi-direction interface.
     * |        |          |1 = 3-wire bi-direction interface.
     * |        |          |Note: In Slave 3-wire mode, the SS_LTRIG, SPI_SSR[4] will be set as 1 automatically.
     * |[9]     |SLV_ABORT |Slave 3-Wire Mode Abort Control
     * |        |          |In normal operation, there is an interrupt event when the received data meet the required bits which defined in TX_BIT_LEN.
     * |        |          |If the received bits are less than the requirement and there is no more SPI clock input over the one transfer time in
     * |        |          |Slave 3-wire mode, the user can set this bit to force the current transfer done and then the user can get a transfer done interrupt event.
     * |        |          |Note: This bit will be cleared to 0 automatically by hardware after it is set to 1 by software.
     * |[10]    |SSTA_INTEN|Slave 3-Wire Mode Start Interrupt Enable
     * |        |          |Used to enable interrupt when the transfer has started in Slave 3-wire mode.
     * |        |          |If there is no transfer done interrupt over the time period which is defined by user after the transfer start,
     * |        |          |the user can set the SLV_ABORT bit to force the transfer done.
     * |        |          |0 = Transaction start interrupt Disabled.
     * |        |          |1 = Transaction start interrupt Enabled.
     * |        |          |It will be cleared to 0 as the current transfer is done or the SLV_START_INTSTS bit is cleared.
     * |[11]    |SLV_START_INTSTS|Slave 3-Wire Mode Start Interrupt Status
     * |        |          |This bit indicates if a transaction has started in Slave 3-wire mode.
     * |        |          |It is a mutual mirror bit of SPI_STATUS[11].
     * |        |          |0 = Slave has not detected any SPI clock transition since the SSTA_INTEN bit was set to 1.
     * |        |          |1 = A transaction has started in Slave 3-wire mode.
     * |        |          |It will be cleared automatically when a transaction is done or by writing 1 to this bit.
     * |[12]    |DUAL_IO_DIR|Dual I/O Mode Direction Control
     * |        |          |0 = Dual Input mode.
     * |        |          |1 = Dual Output mode.
     * |[13]    |DUAL_IO_EN|Dual I/O Mode Enable
     * |        |          |0 = Dual I/O mode Disabled.
     * |        |          |1 = Dual I/O mode Enabled.
     * |[16]    |SS_INT_OPT|Slave Select Inactive Interrupt Option
     * |        |          |This setting is only available if the SPI controller is configured as level trigger slave device.
     * |        |          |0 = As the slave select signal goes to inactive level, the IF bit will NOT be set to 1.
     * |        |          |1 = As the slave select signal goes to inactive level, the IF bit will be set to 1.
     * |[31]    |BCn       |SPI Peripheral Clock Backward Compatible Option
     * |        |          |0 = Backward compatible clock configuration.
     * |        |          |1 = Clock configuration is not backward compatible.
     * |        |          |Refer to the description of SPI_DIVIDER register for details.
     */
    __IO uint32_t CNTRL2;

    /**
     * SPI_FIFO_CTL
     * ===================================================================================================
     * Offset: 0x40  SPI FIFO Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |RX_CLR    |Clear Receive FIFO Buffer
     * |        |          |0 = No effect.
     * |        |          |1 = Clear receive FIFO buffer.
     * |        |          |The RX_FULL flag will be cleared to 0 and the RX_EMPTY flag will be set to 1.
     * |        |          |This bit will be cleared to 0 by hardware after it is set to 1 by software.
     * |[1]     |TX_CLR    |Clear Transmit FIFO Buffer
     * |        |          |0 = No effect.
     * |        |          |1 = Clear transmit FIFO buffer.
     * |        |          |The TX_FULL flag will be cleared to 0 and the TX_EMPTY flag will be set to 1.
     * |        |          |This bit will be cleared to 0 by hardware after it is set to 1 by software.
     * |[2]     |RX_INTEN  |Receive Threshold Interrupt Enable
     * |        |          |0 = RX threshold interrupt Disabled.
     * |        |          |1 = RX threshold interrupt Enabled.
     * |[3]     |TX_INTEN  |Transmit Threshold Interrupt Enable
     * |        |          |0 = TX threshold interrupt Disabled.
     * |        |          |1 = TX threshold interrupt Enabled.
     * |[6]     |RXOV_INTEN|Receive FIFO Overrun Interrupt Enable
     * |        |          |0 = Receive FIFO overrun interrupt Disabled.
     * |        |          |1 = Receive FIFO overrun interrupt Enabled.
     * |[21]    |TIMEOUT_INTEN|Receive FIFO Time-Out Interrupt Enable
     * |        |          |0 = Time-out interrupt Disabled.
     * |        |          |1 = Time-out interrupt Enabled.
     * |[26:24] |RX_THRESHOLD|Receive FIFO Threshold
     * |        |          |If the valid data count of the receive FIFO buffer is larger than the RX_THRESHOLD setting,
     * |        |          |the RX_INTSTS bit will be set to 1, else the RX_INTSTS bit will be cleared to 0.
     * |[30:28] |TX_THRESHOLD|Transmit FIFO Threshold
     * |        |          |If the valid data count of the transmit FIFO buffer is less than or equal to the TX_THRESHOLD
     * |        |          |setting, the TX_INTSTS bit will be set to 1, else the TX_INTSTS bit will be cleared to 0.
     */
    __IO uint32_t FIFO_CTL;

    /**
     * SPI_STATUS
     * ===================================================================================================
     * Offset: 0x44  SPI Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |RX_INTSTS |Receive FIFO Threshold Interrupt Status (Read Only)
     * |        |          |0 = The valid data count within the Rx FIFO buffer is smaller than or equal to the setting value of RX_THRESHOLD.
     * |        |          |1 = The valid data count within the receive FIFO buffer is larger than the setting value of RX_THRESHOLD.
     * |        |          |Note: If RX_INTEN = 1 and RX_INTSTS = 1, the SPI controller will generate a SPI interrupt request.
     * |[2]     |RX_OVERRUN|Receive FIFO Overrun Status
     * |        |          |When the receive FIFO buffer is full, the follow-up data will be dropped and this bit will be set to 1.
     * |        |          |Note: This bit will be cleared by writing 1 to itself.
     * |[4]     |TX_INTSTS |Transmit FIFO Threshold Interrupt Status (Read Only)
     * |        |          |0 = The valid data count within the transmit FIFO buffer is larger than the setting value of TX_THRESHOLD.
     * |        |          |1 = The valid data count within the transmit FIFO buffer is less than or equal to the setting value of TX_THRESHOLD.
     * |        |          |Note: If TX_INTEN = 1 and TX_INTSTS = 1, the SPI controller will generate a SPI interrupt request.
     * |[11]    |SLV_START_INTSTS|Slave Start Interrupt Status
     * |        |          |It is used to dedicate if a transaction has started in Slave 3-wire mode.
     * |        |          |It is a mutual mirror bit of SPI_CNTRL2[11].
     * |        |          |0 = Slave has not detected any SPI clock transition since the SSTA_INTEN bit was set to 1.
     * |        |          |1 = A transaction has started in Slave 3-wire mode.
     * |        |          |It will be cleared as a transaction is done or by writing 1 to this bit.
     * |[15:12] |RX_FIFO_COUNT|Receive FIFO Data Count (Read Only)
     * |        |          |This bit field indicates the valid data count of receive FIFO buffer.
     * |[16]    |IF        |SPI Unit Transfer Interrupt Flag
     * |        |          |It is a mutual mirror bit of SPI_CNTRL[16].
     * |        |          |0 = No transaction has been finished since this bit was cleared to 0.
     * |        |          |1 = SPI controller has finished one unit transfer.
     * |        |          |Note: This bit will be cleared by writing 1 to itself.
     * |[20]    |TIMEOUT   |Time-Out Interrupt Flag
     * |        |          |0 = No receive FIFO time-out event.
     * |        |          |1 = Receive FIFO buffer is not empty and no read operation on receive FIFO buffer over 64 SPI clock
     * |        |          |period in Master mode or over 576 SPI peripheral clock period in Slave mode.
     * |        |          |When the received FIFO buffer is read by software, the time-out status will be cleared automatically.
     * |        |          |Note: This bit will be cleared by writing 1 to itself.
     * |[24]    |RX_EMPTY  |Receive FIFO Buffer Empty Indicator (Read Only)
     * |        |          |It is a mutual mirror bit of SPI_CNTRL[24].
     * |        |          |0 = Receive FIFO buffer is not empty.
     * |        |          |1 = Receive FIFO buffer is empty.
     * |[25]    |RX_FULL   |Receive FIFO Buffer Empty Indicator (Read Only)
     * |        |          |It is a mutual mirror bit of SPI_CNTRL[24].
     * |        |          |0 = Receive FIFO buffer is not empty.
     * |        |          |1 = Receive FIFO buffer is empty.
     * |[26]    |TX_EMPTY  |Transmit FIFO Buffer Empty Indicator (Read Only)
     * |        |          |It is a mutual mirror bit of SPI_CNTRL[26].
     * |        |          |0 = Transmit FIFO buffer is not empty.
     * |        |          |1 = Transmit FIFO buffer is empty.
     * |[27]    |TX_FULL   |Transmit FIFO Buffer Full Indicator (Read Only)
     * |        |          |It is a mutual mirror bit of SPI_CNTRL[27].
     * |        |          |0 = Transmit FIFO buffer is not full.
     * |        |          |1 = Transmit FIFO buffer is full.
     * |[31:28] |TX_FIFO_COUNT|Transmit FIFO Data Count (Read Only)
     * |        |          |This bit field indicates the valid data count of transmit FIFO buffer.
     */
    __IO uint32_t STATUS;
} SPI_T;

/** @addtogroup REG_SPI_BITMASK SPI Bit Mask
  @{
 */

/* SPI_CNTRL Bit Field Definitions */
#define SPI_CNTRL_TX_FULL_Pos      27                                     /*!< SPI CNTRL: TX_FULL Position */
#define SPI_CNTRL_TX_FULL_Msk      (1ul << SPI_CNTRL_TX_FULL_Pos)         /*!< SPI CNTRL: TX_FULL Mask     */

#define SPI_CNTRL_TX_EMPTY_Pos     26                                     /*!< SPI CNTRL: TX_EMPTY Position */
#define SPI_CNTRL_TX_EMPTY_Msk     (1ul << SPI_CNTRL_TX_EMPTY_Pos)        /*!< SPI CNTRL: TX_EMPTY Mask     */

#define SPI_CNTRL_RX_FULL_Pos      25                                     /*!< SPI CNTRL: RX_FULL Position */
#define SPI_CNTRL_RX_FULL_Msk      (1ul << SPI_CNTRL_RX_FULL_Pos)         /*!< SPI CNTRL: RX_FULL Mask     */

#define SPI_CNTRL_RX_EMPTY_Pos     24                                     /*!< SPI CNTRL: RX_EMPTY Position */
#define SPI_CNTRL_RX_EMPTY_Msk     (1ul << SPI_CNTRL_RX_EMPTY_Pos)        /*!< SPI CNTRL: RX_EMPTY Mask     */

#define SPI_CNTRL_VARCLK_EN_Pos    23                                     /*!< SPI CNTRL: VARCLK_EN Position */
#define SPI_CNTRL_VARCLK_EN_Msk    (1ul << SPI_CNTRL_VARCLK_EN_Pos)       /*!< SPI CNTRL: VARCLK_EN Mask     */

#define SPI_CNTRL_TWOB_Pos         22                                     /*!< SPI CNTRL: TWOB Position */
#define SPI_CNTRL_TWOB_Msk         (1ul << SPI_CNTRL_TWOB_Pos)            /*!< SPI CNTRL: TWOB Mask     */

#define SPI_CNTRL_FIFO_Pos         21                                     /*!< SPI CNTRL: FIFO Position */
#define SPI_CNTRL_FIFO_Msk         (1ul << SPI_CNTRL_FIFO_Pos)            /*!< SPI CNTRL: FIFO Mask     */

#define SPI_CNTRL_REORDER_Pos      19                                     /*!< SPI CNTRL: REORDER Position */
#define SPI_CNTRL_REORDER_Msk      (1ul << SPI_CNTRL_REORDER_Pos)         /*!< SPI CNTRL: REORDER Mask     */

#define SPI_CNTRL_SLAVE_Pos        18                                     /*!< SPI CNTRL: SLAVE Position */
#define SPI_CNTRL_SLAVE_Msk        (1ul << SPI_CNTRL_SLAVE_Pos)           /*!< SPI CNTRL: SLAVE Mask     */

#define SPI_CNTRL_IE_Pos           17                                     /*!< SPI CNTRL: IE Position */
#define SPI_CNTRL_IE_Msk           (1ul << SPI_CNTRL_IE_Pos)              /*!< SPI CNTRL: IE Mask     */

#define SPI_CNTRL_IF_Pos           16                                     /*!< SPI CNTRL: IF Position */
#define SPI_CNTRL_IF_Msk           (1ul << SPI_CNTRL_IF_Pos)              /*!< SPI CNTRL: IF Mask     */

#define SPI_CNTRL_SP_CYCLE_Pos     12                                     /*!< SPI CNTRL: SP_CYCLE Position */
#define SPI_CNTRL_SP_CYCLE_Msk     (0xFul << SPI_CNTRL_SP_CYCLE_Pos)      /*!< SPI CNTRL: SP_CYCLE Mask     */

#define SPI_CNTRL_CLKP_Pos         11                                     /*!< SPI CNTRL: CLKP Position */
#define SPI_CNTRL_CLKP_Msk         (1ul << SPI_CNTRL_CLKP_Pos)            /*!< SPI CNTRL: CLKP Mask     */

#define SPI_CNTRL_LSB_Pos          10                                     /*!< SPI CNTRL: LSB Position */
#define SPI_CNTRL_LSB_Msk          (1ul << SPI_CNTRL_LSB_Pos)             /*!< SPI CNTRL: LSB Mask     */

#define SPI_CNTRL_TX_BIT_LEN_Pos   3                                      /*!< SPI CNTRL: TX_BIT_LEN Position */
#define SPI_CNTRL_TX_BIT_LEN_Msk   (0x1Ful << SPI_CNTRL_TX_BIT_LEN_Pos)   /*!< SPI CNTRL: TX_BIT_LEN Mask     */

#define SPI_CNTRL_TX_NEG_Pos       2                                      /*!< SPI CNTRL: TX_NEG Position */
#define SPI_CNTRL_TX_NEG_Msk       (1ul << SPI_CNTRL_TX_NEG_Pos)          /*!< SPI CNTRL: TX_NEG Mask     */

#define SPI_CNTRL_RX_NEG_Pos       1                                      /*!< SPI CNTRL: RX_NEG Position */
#define SPI_CNTRL_RX_NEG_Msk       (1ul << SPI_CNTRL_RX_NEG_Pos)          /*!< SPI CNTRL: RX_NEG Mask     */

#define SPI_CNTRL_GO_BUSY_Pos      0                                      /*!< SPI CNTRL: GO_BUSY Position */
#define SPI_CNTRL_GO_BUSY_Msk      (1ul << SPI_CNTRL_GO_BUSY_Pos)         /*!< SPI CNTRL: GO_BUSY Mask     */

/* SPI_DIVIDER Bit Field Definitions */
#define SPI_DIVIDER_DIVIDER2_Pos   16                                     /*!< SPI DIVIDER: DIVIDER2 Position */
#define SPI_DIVIDER_DIVIDER2_Msk   (0xFFul << SPI_DIVIDER_DIVIDER2_Pos)   /*!< SPI DIVIDER: DIVIDER2 Mask */

#define SPI_DIVIDER_DIVIDER_Pos    0                                      /*!< SPI DIVIDER: DIVIDER Position */
#define SPI_DIVIDER_DIVIDER_Msk    (0xFFul << SPI_DIVIDER_DIVIDER_Pos)    /*!< SPI DIVIDER: DIVIDER Mask */

/* SPI_SSR Bit Field Definitions */
#define SPI_SSR_LTRIG_FLAG_Pos     5                                 /*!< SPI SSR: LTRIG_FLAG Position */
#define SPI_SSR_LTRIG_FLAG_Msk     (1ul << SPI_SSR_LTRIG_FLAG_Pos)   /*!< SPI SSR: LTRIG_FLAG Mask */

#define SPI_SSR_SS_LTRIG_Pos       4                                 /*!< SPI SSR: SS_LTRIG Position */
#define SPI_SSR_SS_LTRIG_Msk       (1ul << SPI_SSR_SS_LTRIG_Pos)     /*!< SPI SSR: SS_LTRIG Mask */

#define SPI_SSR_AUTOSS_Pos         3                                 /*!< SPI SSR: AUTOSS Position */
#define SPI_SSR_AUTOSS_Msk         (1ul << SPI_SSR_AUTOSS_Pos)       /*!< SPI SSR: AUTOSS Mask */

#define SPI_SSR_SS_LVL_Pos         2                                 /*!< SPI SSR: SS_LVL Position */
#define SPI_SSR_SS_LVL_Msk         (1ul << SPI_SSR_SS_LVL_Pos)       /*!< SPI SSR: SS_LVL Mask */

#define SPI_SSR_SSR_Pos            0                                 /*!< SPI SSR: SSR Position */
#define SPI_SSR_SSR_Msk            (3ul << SPI_SSR_SSR_Pos)          /*!< SPI SSR: SSR Mask */

/* SPI_DMA Bit Field Definitions */
#define SPI_DMA_PDMA_RST_Pos   2                                     /*!< SPI DMA: PDMA_RST Position */
#define SPI_DMA_PDMA_RST_Msk   (1ul << SPI_DMA_PDMA_RST_Pos)         /*!< SPI DMA: PDMA_RST Mask */

#define SPI_DMA_RX_DMA_GO_Pos   1                                    /*!< SPI DMA: RX_DMA_GO Position */
#define SPI_DMA_RX_DMA_GO_Msk   (1ul << SPI_DMA_RX_DMA_GO_Pos)       /*!< SPI DMA: RX_DMA_GO Mask */

#define SPI_DMA_TX_DMA_GO_Pos   0                                    /*!< SPI DMA: TX_DMA_GO Position */
#define SPI_DMA_TX_DMA_GO_Msk   (1ul << SPI_DMA_TX_DMA_GO_Pos)       /*!< SPI DMA: TX_DMA_GO Mask */

/* SPI_CNTRL2 Bit Field Definitions */
#define SPI_CNTRL2_BCn_Pos   31                                                      /*!< SPI CNTRL2: BCn Position */
#define SPI_CNTRL2_BCn_Msk   (1ul << SPI_CNTRL2_BCn_Pos)                             /*!< SPI CNTRL2: BCn Mask */

#define SPI_CNTRL2_SS_INT_OPT_Pos   16                                               /*!< SPI CNTRL2: SS_INT_OPT Position */
#define SPI_CNTRL2_SS_INT_OPT_Msk   (1ul << SPI_CNTRL2_SS_INT_OPT_Pos)               /*!< SPI CNTRL2: SS_INT_OPT Mask */

#define SPI_CNTRL2_DUAL_IO_EN_Pos   13                                               /*!< SPI CNTRL2: DUAL_IO_EN Position */
#define SPI_CNTRL2_DUAL_IO_EN_Msk   (1ul << SPI_CNTRL2_DUAL_IO_EN_Pos)               /*!< SPI CNTRL2: DUAL_IO_EN Mask */

#define SPI_CNTRL2_DUAL_IO_DIR_Pos   12                                              /*!< SPI CNTRL2: DUAL_IO_DIR Position */
#define SPI_CNTRL2_DUAL_IO_DIR_Msk   (1ul << SPI_CNTRL2_DUAL_IO_DIR_Pos)             /*!< SPI CNTRL2: DUAL_IO_DIR Mask */

#define SPI_CNTRL2_SLV_START_INTSTS_Pos   11                                         /*!< SPI CNTRL2: SLV_START_INTSTS Position */
#define SPI_CNTRL2_SLV_START_INTSTS_Msk   (1ul << SPI_CNTRL2_SLV_START_INTSTS_Pos)   /*!< SPI CNTRL2: SLV_START_INTSTS Mask */

#define SPI_CNTRL2_SSTA_INTEN_Pos   10                                               /*!< SPI CNTRL2: SSTA_INTEN Position */
#define SPI_CNTRL2_SSTA_INTEN_Msk   (1ul << SPI_CNTRL2_SSTA_INTEN_Pos)               /*!< SPI CNTRL2: SSTA_INTEN Mask */

#define SPI_CNTRL2_SLV_ABORT_Pos    9                                                /*!< SPI CNTRL2: SLV_ABORT Position */
#define SPI_CNTRL2_SLV_ABORT_Msk    (1ul << SPI_CNTRL2_SLV_ABORT_Pos)                /*!< SPI CNTRL2: SLV_ABORT Mask */

#define SPI_CNTRL2_NOSLVSEL_Pos     8                                                /*!< SPI CNTRL2: NOSLVSEL Position */
#define SPI_CNTRL2_NOSLVSEL_Msk     (1ul << SPI_CNTRL2_NOSLVSEL_Pos)                 /*!< SPI CNTRL2: NOSLVSEL Mask */

/* SPI_FIFO_CTL Bit Field Definitions */
#define SPI_FIFO_CTL_TX_THRESHOLD_Pos   28                                         /*!< SPI FIFO_CTL: TX_THRESHOLD Position */
#define SPI_FIFO_CTL_TX_THRESHOLD_Msk   (7ul << SPI_FIFO_CTL_TX_THRESHOLD_Pos)     /*!< SPI FIFO_CTL: TX_THRESHOLD Mask */

#define SPI_FIFO_CTL_RX_THRESHOLD_Pos   24                                         /*!< SPI FIFO_CTL: RX_THRESHOLD Position */
#define SPI_FIFO_CTL_RX_THRESHOLD_Msk   (7ul << SPI_FIFO_CTL_RX_THRESHOLD_Pos)     /*!< SPI FIFO_CTL: RX_THRESHOLD Mask */

#define SPI_FIFO_CTL_TIMEOUT_INTEN_Pos   21                                        /*!< SPI FIFO_CTL: TIMEOUT_INTEN Position */
#define SPI_FIFO_CTL_TIMEOUT_INTEN_Msk   (1ul << SPI_FIFO_CTL_TIMEOUT_INTEN_Pos)   /*!< SPI FIFO_CTL: TIMEOUT_INTEN Mask */

#define SPI_FIFO_CTL_RXOV_INTEN_Pos    6                                           /*!< SPI FIFO_CTL: RXOV_INTEN Position */
#define SPI_FIFO_CTL_RXOV_INTEN_Msk    (1ul << SPI_FIFO_CTL_RXOV_INTEN_Pos)        /*!< SPI FIFO_CTL: RXOV_INTEN Mask */

#define SPI_FIFO_CTL_TX_INTEN_Pos    3                                             /*!< SPI FIFO_CTL: TX_INTEN Position */
#define SPI_FIFO_CTL_TX_INTEN_Msk    (1ul << SPI_FIFO_CTL_TX_INTEN_Pos)            /*!< SPI FIFO_CTL: TX_INTEN Mask */

#define SPI_FIFO_CTL_RX_INTEN_Pos    2                                             /*!< SPI FIFO_CTL: RX_INTEN Position */
#define SPI_FIFO_CTL_RX_INTEN_Msk    (1ul << SPI_FIFO_CTL_RX_INTEN_Pos)            /*!< SPI FIFO_CTL: RX_INTEN Mask */

#define SPI_FIFO_CTL_TX_CLR_Pos     1                                              /*!< SPI FIFO_CTL: TX_CLR Position */
#define SPI_FIFO_CTL_TX_CLR_Msk     (1ul << SPI_FIFO_CTL_TX_CLR_Pos)               /*!< SPI FIFO_CTL: TX_CLR Mask */

#define SPI_FIFO_CTL_RX_CLR_Pos      0                                             /*!< SPI FIFO_CTL: RX_CLR Position */
#define SPI_FIFO_CTL_RX_CLR_Msk      (1ul << SPI_FIFO_CTL_RX_CLR_Pos)              /*!< SPI FIFO_CTL: RX_CLR Mask */

/* SPI_STATUS Bit Field Definitions */
#define SPI_STATUS_TX_FIFO_COUNT_Pos   28                                            /*!< SPI STATUS: TX_FIFO_COUNT Position */
#define SPI_STATUS_TX_FIFO_COUNT_Msk   (0xFul << SPI_STATUS_TX_FIFO_COUNT_Pos)       /*!< SPI STATUS: TX_FIFO_COUNT Mask */

#define SPI_STATUS_TX_FULL_Pos   27                                                  /*!< SPI STATUS: TX_FULL Position */
#define SPI_STATUS_TX_FULL_Msk   (1ul << SPI_STATUS_TX_FULL_Pos)                     /*!< SPI STATUS: TX_FULL Mask */

#define SPI_STATUS_TX_EMPTY_Pos   26                                                 /*!< SPI STATUS: TX_EMPTY Position */
#define SPI_STATUS_TX_EMPTY_Msk   (1ul << SPI_STATUS_TX_EMPTY_Pos)                   /*!< SPI STATUS: TX_EMPTY Mask */

#define SPI_STATUS_RX_FULL_Pos   25                                                  /*!< SPI STATUS: RX_FULL Position */
#define SPI_STATUS_RX_FULL_Msk   (1ul << SPI_STATUS_RX_FULL_Pos)                     /*!< SPI STATUS: RX_FULL Mask */

#define SPI_STATUS_RX_EMPTY_Pos   24                                                 /*!< SPI STATUS: RX_EMPTY Position */
#define SPI_STATUS_RX_EMPTY_Msk   (1ul << SPI_STATUS_RX_EMPTY_Pos)                   /*!< SPI STATUS: RX_EMPTY Mask */

#define SPI_STATUS_TIMEOUT_Pos   20                                                  /*!< SPI STATUS: TIMEOUT Position */
#define SPI_STATUS_TIMEOUT_Msk   (1ul << SPI_STATUS_TIMEOUT_Pos)                     /*!< SPI STATUS: TIMEOUT Mask */

#define SPI_STATUS_IF_Pos   16                                                       /*!< SPI STATUS: IF Position */
#define SPI_STATUS_IF_Msk   (1ul << SPI_STATUS_IF_Pos)                               /*!< SPI STATUS: IF Mask     */

#define SPI_STATUS_RX_FIFO_COUNT_Pos   12                                            /*!< SPI STATUS: RX_FIFO_COUNT Position */
#define SPI_STATUS_RX_FIFO_COUNT_Msk   (0xFul << SPI_STATUS_RX_FIFO_COUNT_Pos)       /*!< SPI STATUS: RX_FIFO_COUNT Mask */

#define SPI_STATUS_SLV_START_INTSTS_Pos   11                                         /*!< SPI STATUS: SLV_START_INTSTS Position */
#define SPI_STATUS_SLV_START_INTSTS_Msk   (1ul << SPI_STATUS_SLV_START_INTSTS_Pos)   /*!< SPI STATUS: SLV_START_INTSTS Mask */

#define SPI_STATUS_TX_INTSTS_Pos   4                                                 /*!< SPI STATUS: TX_INTSTS Position */
#define SPI_STATUS_TX_INTSTS_Msk   (1ul << SPI_STATUS_TX_INTSTS_Pos)                 /*!< SPI STATUS: TX_INTSTS Mask */

#define SPI_STATUS_RX_OVERRUN_Pos   2                                                /*!< SPI STATUS: RX_OVERRUN Position */
#define SPI_STATUS_RX_OVERRUN_Msk   (1ul << SPI_STATUS_RX_OVERRUN_Pos)               /*!< SPI STATUS: RX_OVERRUN Mask */

#define SPI_STATUS_RX_INTSTS_Pos   0                                                 /*!< SPI STATUS: RX_INTSTS Position */
#define SPI_STATUS_RX_INTSTS_Msk   (1ul << SPI_STATUS_RX_INTSTS_Pos)                 /*!< SPI STATUS: RX_INTSTS Mask */
/*@}*/ /* end of group REG_SPI_BITMASK */
/*@}*/ /* end of group REG_SPI */


/*---------------------------- Global Controller -----------------------------*/
/** @addtogroup REG_SYS System Controller
  Memory Mapped Structure for System Controller
  @{
 */

typedef struct
{

    /**
     * PDID
     * ===================================================================================================
     * Offset: 0x00  Part Device Identification Number Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |PDID      |Part Device Identification Number
     * |        |          |This register reflects device part number code.
     * |        |          |Software can read this register to identify which device is used.
     */
    __I uint32_t PDID;

    /**
     * RSTSRC
     * ===================================================================================================
     * Offset: 0x04  System Reset Source Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |RSTS_POR  |Power-on Reset Flag
     * |        |          |The RSTS_POR Flag Is Set By The "Reset Signal" From The Power-On Reset (POR) Controller Or Bit CHIP_RST (IPRSTC1[0]) To Indicate The Previous Reset Source
     * |        |          |0 = No reset from POR or CHIP_RST (IPRSTC1[0]).
     * |        |          |1 = Power-on Reset (POR) or CHIP_RST (IPRSTC1[0]) had issued the reset signal to reset the system.
     * |        |          |Note: Write 1 to clear this bit to 0.
     * |[1]     |RSTS_RESET|Reset Pin Reset Flag
     * |        |          |The RSTS_RESET Flag Is Set By The "Reset Signal" From The /RESET Pin To Indicate The Previous Reset Source
     * |        |          |0 = No reset from /RESET pin.
     * |        |          |1 = The Pin /RESET had issued the reset signal to reset the system.
     * |        |          |Note: Write 1 to clear this bit to 0.
     * |[2]     |RSTS_WDT  |Watchdog Reset Flag
     * |        |          |The RSTS_WDT Flag Is Set By The "Reset Signal" From The Watchdog Timer To Indicate The Previous Reset Source
     * |        |          |0 = No reset from watchdog timer.
     * |        |          |1 = The watchdog timer had issued the reset signal to reset the system.
     * |        |          |Note: Write 1 to clear this bit to 0.
     * |[3]     |RSTS_LVR  |Low Voltage Reset Flag
     * |        |          |The RSTS_LVR Flag Is Set By The "Reset Signal" From The Low-Voltage-Reset Controller To Indicate The Previous Reset Source
     * |        |          |0 = No reset from LVR.
     * |        |          |1 = The LVR controller had issued the reset signal to reset the system.
     * |        |          |Note: Write 1 to clear this bit to 0.
     * |[4]     |RSTS_BOD  |Brown-out Detector Reset Flag
     * |        |          |The RSTS_BOD Flag Is Set By The "Reset Signal" From The Brown-Out Detector To Indicate The Previous Reset Source
     * |        |          |0 = No reset from BOD.
     * |        |          |1 = The BOD had issued the reset signal to reset the system.
     * |        |          |Note: Write 1 to clear this bit to 0.
     * |[5]     |RSTS_SYS  |MCU Reset Flag
     * |        |          |The RSTS_SYS Flag Is Set By The "Reset Signal" From The Cortex-M0 Kernel To Indicate The Previous Reset Source
     * |        |          |0 = No reset from Cortex-M0.
     * |        |          |1 = The Cortex-M0 had issued the reset signal to reset the system by writing 1 to bit SYSRESETREQ (AIRCR[2], Application Interrupt and Reset Control Register, address = 0xE000ED0C) in system control registers of Cortex-M0 kernel.
     * |        |          |Note: Write 1 to clear this bit to 0.
     * |[7]     |RSTS_CPU  |CPU Reset Flag
     * |        |          |The RSTS_CPU Flag Is Set By Hardware If Software Writes CPU_RST (IPRSTC1[1]) 1 To Reset Cortex-M0 CPU Kernel And Flash Memory Controller (FMC)
     * |        |          |0 = No reset from CPU.
     * |        |          |1 = Cortex-M0 CPU kernel and FMC are reset by software setting CPU_RST(IPRSTC1[1]) to 1.
     * |        |          |Note: Write 1 to clear this bit to 0.
     */
    __IO uint32_t RSTSRC;

    /**
     * IPRSTC1
     * ===================================================================================================
     * Offset: 0x08  IP Reset Control Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CHIP_RST  |CHIP One-Shot Reset (Write Protect)
     * |        |          |Setting this bit will reset the whole chip, including CPU kernel and all peripherals, and this bit will automatically return to 0 after the 2 clock cycles.
     * |        |          |The CHIP_RST is the same as the POR reset, all the chip controllers are reset and the chip setting from flash are also reload.
     * |        |          |For the difference between CHIP_RST and SYSRESETREQ, please refer to section 5.2.2
     * |        |          |0 = CHIP normal operation.
     * |        |          |1 = CHIP one-shot reset.
     * |        |          |Note: This bit is the protected bit, and programming it needs to write "59h", "16h", and "88h" to address 0x5000_0100 to disable register protection.
     * |        |          |Refer to the register REGWRPROT at address GCR_BA+0x100.
     * |[1]     |CPU_RST   |CPU Kernel One-Shot Reset (Write Protect)
     * |        |          |Setting this bit will only reset the CPU kernel and Flash Memory Controller(FMC), and this bit will automatically return 0 after the two clock cycles
     * |        |          |0 = CPU normal operation.
     * |        |          |1 = CPU one-shot reset.
     * |        |          |Note: This bit is the protected bit, and programming it needs to write "59h", "16h", and "88h" to address 0x5000_0100 to disable register protection.
     * |        |          |Refer to the register REGWRPROT at address GCR_BA+0x100.
     * |[2]     |PDMA_RST  |PDMA Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the PDMA.
     * |        |          |User need to set this bit to 0 to release from reset state.
     * |        |          |0 = PDMA controller normal operation.
     * |        |          |1 = PDMA controller reset.
     * |        |          |Note: This bit is the protected bit, and programming it needs to write "59h", "16h", and "88h" to address 0x5000_0100 to disable register protection.
     * |        |          |Refer to the register REGWRPROT at address GCR_BA+0x100.
     */
    __IO uint32_t IPRSTC1;

    /**
     * IPRSTC2
     * ===================================================================================================
     * Offset: 0x0C  IP Reset Control Register 2
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1]     |GPIO_RST  |GPIO Controller Reset
     * |        |          |0 = GPIO controller normal operation.
     * |        |          |1 = GPIO controller reset.
     * |[2]     |TMR0_RST  |Timer0 Controller Reset
     * |        |          |0 = Timer0 controller normal operation.
     * |        |          |1 = Timer0 controller reset.
     * |[3]     |TMR1_RST  |Timer1 Controller Reset
     * |        |          |0 = Timer1 controller normal operation.
     * |        |          |1 = Timer1 controller reset.
     * |[4]     |TMR2_RST  |Timer2 Controller Reset
     * |        |          |0 = Timer2 controller normal operation.
     * |        |          |1 = Timer2 controller reset.
     * |[5]     |TMR3_RST  |Timer3 Controller Reset
     * |        |          |0 = Timer3 controller normal operation.
     * |        |          |1 = Timer3 controller reset.
     * |[8]     |I2C0_RST  |I2C0 Controller Reset
     * |        |          |0 = I2C0 controller normal operation.
     * |        |          |1 = I2C0 controller reset.
     * |[9]     |I2C1_RST  |I2C1 Controller Reset
     * |        |          |0 = I2C1 controller normal operation.
     * |        |          |1 = I2C1 controller reset.
     * |[12]    |SPI0_RST  |SPI0 Controller Reset
     * |        |          |0 = SPI0 controller normal operation.
     * |        |          |1 = SPI0 controller reset.
     * |[13]    |SPI1_RST  |SPI1 Controller Reset
     * |        |          |0 = SPI1 controller normal operation.
     * |        |          |1 = SPI1 controller reset.
     * |[14]    |SPI2_RST  |SPI2 Controller Reset
     * |        |          |0 = SPI2 controller normal operation.
     * |        |          |1 = SPI2 controller reset.
     * |[16]    |UART0_RST |UART0 Controller Reset
     * |        |          |0 = UART0 controller normal operation.
     * |        |          |1 = UART0 controller reset.
     * |[17]    |UART1_RST |UART1 Controller Reset
     * |        |          |0 = UART1 controller normal operation.
     * |        |          |1 = UART1 controller reset.
     * |[20]    |PWM03_RST |PWM03 Controller Reset
     * |        |          |0 = PWM03 controller normal operation.
     * |        |          |1 = PWM03 controller reset.
     * |[23]    |PS2_RST   |PS/2 Controller Reset
     * |        |          |0 = PS/2 controller normal operation.
     * |        |          |1 = PS/2 controller reset.
     * |[27]    |USBD_RST  |USB Device Controller Reset
     * |        |          |0 = USB device controller normal operation.
     * |        |          |1 = USB device controller reset.
     * |[28]    |ADC_RST   |ADC Controller Reset
     * |        |          |0 = ADC controller normal operation.
     * |        |          |1 = ADC controller reset.
     * |[29]    |I2S_RST   |I2S Controller Reset
     * |        |          |0 = I2S controller normal operation.
     * |        |          |1 = I2S controller reset.
     */
    __IO uint32_t IPRSTC2;

    /**
     * @cond HIDDEN_SYMBOLS
     */
    uint32_t RESERVE0[2];
    /**
     * @endcond
     */

    /**
     * BODCR
     * ===================================================================================================
     * Offset: 0x18  Brown-out Detector Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |BOD_EN    |Brown-Out Detector Enable (Write Protect)
     * |        |          |The default value is set by flash controller user configuration register CONFIG0 bit[23]
     * |        |          |0 = Brown-out Detector function Disabled.
     * |        |          |1 = Brown-out Detector function Enabled.
     * |        |          |Note: This bit is the protected bit.
     * |        |          |It means programming this needs to write "59h", "16h", "88h" to address 0x5000_0100 to disable register protection.
     * |        |          |Refer to the register REGWRPROT at address GCR_BA+0x100.
     * |[2:1]   |BOD_VL    |Brown-Out Detector Threshold Voltage Selection (Write Protect)
     * |        |          |The default value is set by flash controller user configuration register config0 bit[22:21].
     * |        |          |00 = Brown-out voltage is 2.2V.
     * |        |          |01 = Brown-out voltage is 2.7V.
     * |        |          |10 = Brown-out voltage is 3.7V.
     * |        |          |11 = Brown-out voltage is 4.4V.
     * |        |          |Note: This bit is the protected bit.
     * |        |          |It means programming this needs to write "59h", "16h", "88h" to address 0x5000_0100 to disable register protection.
     * |        |          |Refer to the register REGWRPROT at address GCR_BA+0x100.
     * |[3]     |BOD_RSTEN |Brown-Out Reset Enable (Write Protect)
     * |        |          |0 = Brown-out "INTERRUPT" function Enabled.
     * |        |          |1 = Brown-out "RESET" function Enabled.
     * |        |          |While the Brown-out Detector function is enabled (BOD_EN high) and BOD reset function is enabled (BOD_RSTEN high), BOD will assert a signal to reset chip when the detected voltage is lower than the threshold (BOD_OUT high).
     * |        |          |Note1: While the BOD function is enabled (BOD_EN high) and BOD interrupt function is enabled (BOD_RSTEN low), BOD will assert an interrupt if BOD_OUT is high.
     * |        |          |BOD interrupt will keep till to the BOD_EN set to 0.
     * |        |          |BOD interrupt can be blocked by disabling the NVIC BOD interrupt or disabling BOD function (set BOD_EN low).
     * |        |          |Note2: The default value is set by flash controller user configuration register config0 bit[20].
     * |        |          |Note3: This bit is the protected bit.
     * |        |          |It means programming this needs to write "59h", "16h", "88h" to address 0x5000_0100 to disable register protection.
     * |        |          |Refer to the register REGWRPROT at address GCR_BA+0x100.
     * |[4]     |BOD_INTF  |Brown-Out Detector Interrupt Flag
     * |        |          |0 = Brown-out Detector does not detect any voltage draft at VDD down through or up through the voltage of BOD_VL setting.
     * |        |          |1 = When Brown-out Detector detects the VDD is dropped down through the voltage of BOD_VL setting or the VDD is raised up through the voltage of BOD_VL setting, this bit is set to 1 and the Brown-out interrupt is requested if Brown-out interrupt is enabled.
     * |        |          |Note: Write 1 to clear this bit to 0.
     * |[5]     |BOD_LPM   |Brown-Out Detector Low Power Mode (Write Protection)
     * |        |          |0 = BOD operated in Normal mode (default).
     * |        |          |1 = BOD Low Power mode Enabled.
     * |        |          |Note1: The BOD consumes about 100 uA in Normal mode, and the low power mode can reduce the current to about 1/10 but slow the BOD response.
     * |        |          |Note2: This bit is the protected bit, and programming it needs to write "59h", "16h", and "88h" to address 0x5000_0100 to disable register protection.
     * |        |          |Refer to the register REGWRPROT at address GCR_BA+0x100.
     * |[6]     |BOD_OUT   |Brown-Out Detector Output Status
     * |        |          |0 = Brown-out Detector output status is 0.
     * |        |          |It means the detected voltage is higher than BOD_VL setting or BOD_EN is 0.
     * |        |          |1 = Brown-out Detector output status is 1.
     * |        |          |It means the detected voltage is lower than BOD_VL setting.
     * |        |          |If the BOD_EN is 0, BOD function disabled , this bit always responds to 0.
     * |[7]     |LVR_EN    |Low Voltage Reset Enable (Write Protection)
     * |        |          |The LVR function reset the chip when the input power voltage is lower than LVR circuit setting.
     * |        |          |LVR function is enabled by default.
     * |        |          |0 = Low Voltage Reset function Disabled.
     * |        |          |1 = Low Voltage Reset function Enabled - After enabling the bit, the LVR function will be active with 100us delay for LVR output stable (default).
     * |        |          |Note: This bit is the protected bit, and programming it needs to write "59h", "16h", and "88h" to address 0x5000_0100 to disable register protection.
     * |        |          |Refer to the register REGWRPROT at address GCR_BA+0x100.
     */
    __IO uint32_t BODCR;

    /**
     * @cond HIDDEN_SYMBOLS
     */
    uint32_t RESERVE1[2];
    /**
     * @endcond
     */

    /**
     * PORCR
     * ===================================================================================================
     * Offset: 0x24  Power-on-Reset Controller Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |POR_DIS_CODE|Power-On-Reset Enable Control (Write Protect)
     * |        |          |When powered on, the POR circuit generates a reset signal to reset the whole chip function, but noise on the power may cause the POR active again.
     * |        |          |User can disable internal POR circuit to avoid unpredictable noise to cause chip reset by writing 0x5AA5 to this field.
     * |        |          |The POR function will be active again when this field is set to another value or chip is reset by other reset source, including:
     * |        |          |/RESET, Watchdog, LVR reset, BOD reset, ICE reset command and the software-chip reset function
     * |        |          |Note: This bit is the protected bit.
     * |        |          |It means programming this needs to write "59h", "16h", "88h" to address 0x5000_0100 to disable register protection.
     * |        |          |Refer to the register REGWRPROT at address GCR_BA+0x100.
     */
    __IO uint32_t PORCR;

    /**
     * @cond HIDDEN_SYMBOLS
     */
    uint32_t RESERVE2[2];
    /**
     * @endcond
     */

    /**
     * GPA_MFP
     * ===================================================================================================
     * Offset: 0x30  GPIOA Multiple Function and Input Type Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[10]    |GPA_MFP10 |PA.10 Pin Function Selection
     * |        |          |The pin function depends on GPA_MFP[10] and PA10_MFP1(ALT_MFP[12]).
     * |        |          |(PA10_MFP1(ALT_MFP[12]), GPA_MFP[10]) value and function mapping is as following list.
     * |        |          |(0, 0) = GPIO function is selected.
     * |        |          |(0, 1) = I2C1_SDA function is selected.
     * |        |          |(1, 0) = SPI1_MISO0 function is selected.
     * |        |          |(1, 1) = SPI2_MISO0 function is selected.
     * |[11]    |GPA_MFP11 |PA.11 Pin Function Selection
     * |        |          |The pin function depends on GPA_MFP[11] and PA11_MFP1 (ALT_MFP[11]).
     * |        |          |(PA11_MFP1(ALT_MFP[11]), GPA_MFP[11]) value and function mapping is as following list.
     * |        |          |(0, 0) = GPIO function is selected.
     * |        |          |(0, 1) = I2C1_SCL function is selected.
     * |        |          |(1, 0) = SPI1_CLK function is selected.
     * |        |          |(1, 1) = SPI2_MOSI0 function is selected.
     * |[12]    |GPA_MFP12 |PA.12 Pin Function Selection
     * |        |          |The pin function depends on GPA_MFP[12].
     * |        |          |0 = GPIO function is selected.
     * |        |          |1 = PWM0 function is selected.
     * |[13]    |GPA_MFP13 |PA.13 Pin Function Selection
     * |        |          |The pin function depends on GPA_MFP[13].
     * |        |          |0 = GPIO function is selected.
     * |        |          |1 = PWM1 function is selected.
     * |[14]    |GPA_MFP14 |PA.14 Pin Function Selection
     * |        |          |The pin function depends on GPA_MFP[14].
     * |        |          |0 = GPIO function is selected.
     * |        |          |1 = = PWM2 function is selected.
     * |[15]    |GPA_MFP15 |PA.15 Pin Function Selection
     * |        |          |The pin function depends on GPA_MFP[15] and PA15_MFP1 (ALT_MFP[9]).
     * |        |          |(PA15_MFP1(ALT_MFP[9]), GPA_MFP[15]) value and function mapping is as following list.
     * |        |          |(0, 0) = GPIO function is selected.
     * |        |          |(0, 1) = PWM3 function is selected.
     * |        |          |(1, 0) = CLKO function is selected.
     * |        |          |(1, 1) = I2S_MCLK function is selected.
     * |[31:16] |GPA_TYPEn |0 = GPIOA[15:0] I/O input Schmitt Trigger function Disabled.
     * |        |          |1 = GPIOA[15:0] I/O input Schmitt Trigger function Enabled.
     * |        |          |GPA[9:0] are reserved in this chip.
     */
    __IO uint32_t GPA_MFP;

    /**
     * GPB_MFP
     * ===================================================================================================
     * Offset: 0x34  GPIOB Multiple Function and Input Type Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |GPB_MFP0  |PB.0 Pin Function Selection
     * |        |          |0 = GPIO function is selected.
     * |        |          |1 = UART0_RXD function is selected.
     * |[1]     |GPB_MFP1  |PB.1 Pin Function Selection
     * |        |          |0 = GPIO function is selected.
     * |        |          |1 = UART0_TXD0 function is selected.
     * |[2]     |GPB_MFP2  |PB.2 Pin Function Selection
     * |        |          |The pin function depends on GPB_MFP[2] and PB2_MFP1 (ALT_MFP[26]).
     * |        |          |(PB2_MFP1(ALT_MFP[26]), GPB_MFP[2]) value and function mapping is as following list.
     * |        |          |(0, 0) = GPIO function is selected.
     * |        |          |(0, 1) = UART0_nRTS function is selected.
     * |        |          |(1, 1) = TM2_EXT function is selected.
     * |[3]     |GPB_MFP3  |PB.3 Pin Function Selection
     * |        |          |The pin function depends on GPB_MFP[3] and PB3_MFP1 (ALT_MFP[27]).
     * |        |          |(PB3_MFP1(ALT_MFP[27]), GPB_MFP[3]) value and function mapping is as following list.
     * |        |          |(0, 0) = GPIO function is selected.
     * |        |          |(0, 1) = UART0_nCTS function is selected.
     * |        |          |(1, 1) = TM3_EXT function is selected.
     * |[4]     |GPB_MFP4  |PB.4 Pin Function Selection
     * |        |          |The pin function depends on GPB_MFP[4] and PB4_MFP1 (ALT_MFP[15]).
     * |        |          |(PB4_MFP1(ALT_MFP[15]), GPB_MFP[4]) value and function mapping is as following list.
     * |        |          |(0, 0) = GPIO function is selected.
     * |        |          |(0, 1) = UART1_RXD function is selected.
     * |        |          |(1, 0) = SPI2_SS0 function is selected.
     * |        |          |(1, 1) = SPI1_SS1 function is selected.
     * |[5]     |GPB_MFP5  |PB. 5 Pin Function Selection
     * |        |          |The pin function depends on GPB_MFP[5] and PB5_MFP1 (ALT_MFP[18]).
     * |        |          |(PB5_MFP1(ALT_MFP[18]), GPB_MFP[5]) value and function mapping is as following list.
     * |        |          |(0, 0) = GPIO function is selected.
     * |        |          |(0, 1) = UART1_TXD function is selected.
     * |        |          |(1, 1) = SPI2_CLK2 function is selected.
     * |[6]     |GPB_MFP6  |PB.6 Pin Function Selection
     * |        |          |The pin function depends on GPB_MFP[6] and PB6_MFP1 (ALT_MFP[17]).
     * |        |          |(PB6_MFP1(ALT_MFP[17]), GPB_MFP[6]) value and function mapping is as following list.
     * |        |          |(0, 0) = GPIO function is selected.
     * |        |          |(0, 1) = UART1_nRTS function is selected.
     * |        |          |(1, 1) = SPI2_MOSI0 function is selected.
     * |[7]     |GPB_MFP7  |PB.7 Pin Function Selection
     * |        |          |The pin function depends on GPB_MFP[7] and PB7_MFP1 (ALT_MFP[16]).
     * |        |          |(PB7_MFP1(ALT_MFP[16]), GPB_MFP[7]) value and function mapping is as following list.
     * |        |          |(0, 0) = GPIO function is selected.
     * |        |          |(0, 1) = UART1_nCTS function is selected.
     * |        |          |(1, 1) = SPI2_MISO0 function is selected.
     * |[8]     |GPB_MFP8  |PB.8 Pin Function Selection
     * |        |          |0 = GPIO function is selected.
     * |        |          |1 = TM0 function is selected.
     * |[9]     |GPB_MFP9  |PB.9 Pin Function Selection
     * |        |          |The pin function depends on GPB_MFP[9] and PB9_MFP1 (ALT_MFP[1]).
     * |        |          |(PB9_MFP1(ALT_MFP[1]), GPB_MFP[9]) value and function mapping is as following list.
     * |        |          |(0, 0) = GPIO function is selected.
     * |        |          |(0, 1) = TM1 function is selected.
     * |        |          |(1, 1) = SPI1_SS1 function is selected.
     * |[10]    |GPB_MFP10 |PB.10 Pin Function Selection
     * |        |          |The pin function depends on GPB_MFP[10] and PB10_MFP1 (ALT_MFP[0]).
     * |        |          |(PB10_MFP1(ALT_MFP[0]), GPB_MFP[10]) value and function mapping is as following list.
     * |        |          |(0, 0) = GPIO function is selected.
     * |        |          |(0, 1) = TM2 function is selected.
     * |        |          |(1, 1) = SPI0_SS1 function is selected.
     * |[12]    |GPB_MFP12 |PB.12 Pin Function Selection
     * |        |          |The pin function depends on GPB_MFP[12] and PB12_MFP1 (ALT_MFP[10]).
     * |        |          |(PB12_MFP1(ALT_MFP[10]), GPB_MFP[12]) value and function mapping is as following list.
     * |        |          |(0, 0) = GPIO function is selected.
     * |        |          |(0, 1) = SPI1_SS0 function is selected.
     * |        |          |(1, 1) = CLKO function is selected.
     * |[13]    |GPB_MFP13 |PB.13 Pin Function Selection
     * |        |          |The pin is a dedicated GPIO pin.
     * |        |          |0 = GPIO function is selected.
     * |[14]    |GPB_MFP14 |PB.14 Pin Function Selection
     * |        |          |The pin function depends on GPB_MFP[14].
     * |        |          |0 = GPIO function is selected.
     * |        |          |1 = /INT0 function is selected.
     * |[15]    |GPB_MFP15 |PB.15 Pin Function Selection
     * |        |          |Bits PB15_MFP1 (ALT_MFP[24]) and GPB_MFP[15] determine the PB.15 function.
     * |        |          |(PB15_MFP1(ALT_MFP[24]), GPB_MFP[15])  value and function mapping is as following list.
     * |        |          |(0, 0) = GPIO function is selected.
     * |        |          |(0, 1) = /INT1 function is selected.
     * |        |          |(1, 1) = TM0_EXT function is selected.
     * |[31:16] |GPB_TYPEn |0 = GPIOB[15:0] I/O input Schmitt Trigger function Disabled.
     * |        |          |1 = GPIOB[15:0] I/O input Schmitt Trigger function Enabled.
     */
    __IO uint32_t GPB_MFP;

    /**
     * GPC_MFP
     * ===================================================================================================
     * Offset: 0x38  GPIOC Multiple Function and Input Type Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |GPC_MFP0  |PC.0 Pin Function Selection
     * |        |          |Bits PC0_MFP1 (ALT_MFP[5]) and GPC_MFP[0] determine the PC.0 function.
     * |        |          |(PC0_MFP1(ALT_MFP[5]), GPC_MFP[0]) value and function mapping is as following list.
     * |        |          |(0, 0) = GPIO function is selected.
     * |        |          |(0, 1) = SPI0_SS0 function is selected.
     * |        |          |(1, 1) = I2S_LRCLK function is selected.
     * |[1]     |GPC_MFP1  |PC.1 Pin Function Selection
     * |        |          |Bits PC1_MFP1 (ALT_MFP[6]) and GPC_MFP[1] determine the PC.1 function.
     * |        |          |(PC1_MFP1(ALT_MFP[6]), GPC_MFP[1]) value and function mapping is as following list.
     * |        |          |(0, 0) = GPIO function is selected.
     * |        |          |(0, 1) = SPI0_CLK function is selected.
     * |        |          |(1, 1) = I2S_BCLK function is selected.
     * |[2]     |GPC_MFP2  |PC.2 Pin Function Selection
     * |        |          |Bits PC2_MFP1 (ALT_MFP[7]) and GPC_MFP[2] determine the PC.2 function.
     * |        |          |(PC2_MFP1(ALT_MFP[7]), GPC_MFP[2]) value and function mapping is as following list.
     * |        |          |(0, 0) = GPIO function is selected.
     * |        |          |(0, 1) = SPI0_MISO0 function is selected.
     * |        |          |(1, 1) = I2S_DI function is selected.
     * |[3]     |GPC_MFP3  |PC.3 Pin Function Selection
     * |        |          |Bits PC3_MFP1 (ALT_MFP[8]) and GPC_MFP[3] determine the PC.3 function.
     * |        |          |(PC3_MFP1(ALT_MFP[8]), GPC_MFP[3]) value and function mapping is as following list.
     * |        |          |(0, 0) = GPIO function is selected.
     * |        |          |(0, 1) = SPI0_MOSI0 function is selected.
     * |        |          |(1, 1) = I2S_DO function is selected.
     * |[4]     |GPC_MFP4  |PC.4 Pin Function Selection
     * |        |          |Bits PC4_MFP1 (ALT_MFP[29]) and GPC_MFP[4] determine the PC.4 function.
     * |        |          |(PC4_MFP1(ALT_MFP[29]), GPC_MFP[4]) value and function mapping is as following list.
     * |        |          |(0, 0) = GPIO function is selected.
     * |        |          |(0, 1) = SPI0_MISO1 function is selected.
     * |        |          |(1, 1) = UART0_RXD function is selected.
     * |[5]     |GPC_MFP5  |PC.5 Pin Function Selection
     * |        |          |Bits PC5_MFP1 (ALT_MFP[30]) and GPC_MFP[5] determine the PC.5 function.
     * |        |          |(PC5_MFP1(ALT_MFP[30]), GPC_MFP[5]) value and function mapping is as following list.
     * |        |          |(0, 0) = GPIO function is selected.
     * |        |          |(0, 1) = SPI0_MOSI1 function is selected.
     * |        |          |(1, 1) = UART0_TXD function is selected.
     * |[8]     |GPC_MFP8  |PC.8 Pin Function Selection
     * |        |          |The pin function depends on GPC_MFP[8].
     * |        |          |0 = GPIO function is selected.
     * |        |          |1 = SPI1_SS0 function is selected.
     * |[9]     |GPC_MFP9  |PC.9 Pin Function Selection
     * |        |          |0 = GPIO function is selected.
     * |        |          |1 = SPI1_CLK function is selected.
     * |[10]    |GPC_MFP10 |PC.10 Pin Function Selection
     * |        |          |1 = SPI1_MISO0 (SPI1 master input, slave output pin-0) function is selected.
     * |        |          |0 = GPIO function is selected.
     * |[11]    |GPC_MFP11 |PC.11 Pin Function Selection
     * |        |          |0 = GPIO function is selected.
     * |        |          |1 = SPI1_MOSI0 (SPI1 master output, slave input pin-0) function is selected.
     * |[12]    |GPC_MFP12 |PC.12 Pin Function Selection
     * |        |          |Bits PC12_MFP1 (ALT_MFP[20]) and GPC_MFP[12] determine the PC.12 function.
     * |        |          |(PC12_MFP1(ALT_MFP[20]), GPC_MFP[12]) value and function mapping is as following list.
     * |        |          |(0, 0) = GPIO function is selected.
     * |        |          |(0, 1) = SPI1_MISO1 function is selected.
     * |        |          |(1, 0) = I2S_MCLK function is selected.
     * |        |          |(1, 1) = PWM2 function is selected.
     * |[13]    |GPC_MFP13 |PC.13 Pin Function Selection
     * |        |          |Bits PC13_MFP1 (ALT_MFP[21]) and GPC_MFP[13] determine the PC.13 function.
     * |        |          |(PC13_MFP1(ALT_MFP[21]), GPC_MFP[13]) value and function mapping is as following list.
     * |        |          |(0, 0) = GPIO function is selected.
     * |        |          |(0, 1) = SPI1_MOSI1 function is selected.
     * |        |          |(1, 0) = CLKO function is selected.
     * |        |          |(1, 1) = PWM3 function is selected.
     * |[31:16] |GPC_TYPEn |0 = GPIOC[15:0] I/O input Schmitt Trigger function Disabled.
     * |        |          |1 = GPIOC[15:0] I/O input Schmitt Trigger function Enabled.
     */
    __IO uint32_t GPC_MFP;

    /**
     * GPD_MFP
     * ===================================================================================================
     * Offset: 0x3C  GPIOD Multiple Function and Input Type Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |GPD_MFP0  |PD.0 Pin Function Selection
     * |        |          |Bits PD0_MFP1 (ALT_MFP1[16]) and GPD_MFP[0] determine the PD.0 function.
     * |        |          |(PD0_MFP1(ALT_MFP1[16]), GPD_MFP[0]) value and function mapping is as following list.
     * |        |          |(0, 0) = GPIO function is selected.
     * |        |          |(1, 0) = SPI2_SS0 function is selected.
     * |        |          |(1, 1) = ADC0 function is selected.
     * |[1]     |GPD_MFP1  |PD.1 Pin Function Selection
     * |        |          |Bits PD1_MFP1 (ALT_MFP1[17]) and GPD_MFP[1] determine the PD.1 function.
     * |        |          |(PD1_MFP1(ALT_MFP1[17]), GPD_MFP[1]) value and function mapping is as following list.
     * |        |          |(0, 0) = GPIO function is selected.
     * |        |          |(0, 1) = SPI0_SS1 function is selected.
     * |        |          |(1, 0) = SPI2_CLK function is selected.
     * |        |          |(1, 1) = ADC1 function is selected.
     * |[2]     |GPD_MFP2  |PD.2 Pin Function Selection
     * |        |          |Bits PD2_MFP1 (ALT_MFP1[18]) and GPD_MFP[2] determine the PD.2 function.
     * |        |          |(PD2_MFP1(ALT_MFP1[18]), GPD_MFP[2]) value and function mapping is as following list.
     * |        |          |(0, 0) = GPIO function is selected.
     * |        |          |(0, 1) = SPI0_MISO1 function is selected.
     * |        |          |(1, 0) = SPI2_MISO0 function is selected.
     * |        |          |(1, 1) = ADC2 function is selected.
     * |[3]     |GPD_MFP3  |PD.3 Pin Function Selection
     * |        |          |Bits PD3_MFP1 (ALT_MFP1[19]) and GPD_MFP[3] determine the PD.3 function.
     * |        |          |(PD3_MPF1(ALT_MFP1[19]), GPD_MFP[3]) value and function mapping is as following list.
     * |        |          |(0, 0) = GPIO function is selected.
     * |        |          |(0, 1) = SPI0_MOSI1 function is selected.
     * |        |          |(1, 0) = SPI2_MOSI0 function is selected.
     * |        |          |(1, 1) = ADC3 function is selected.
     * |[4]     |GPD_MFP4  |PD.4 Pin Function Selection
     * |        |          |Bits PD4_MFP1 (ALT_MFP1[20]) and GPD_MFP[4] determine the PD.4 function.
     * |        |          |(PD4_MFP1(ALT_MFP1[20]), GPD_MFP[4]) value and function mapping is as following list.
     * |        |          |(0, 0) = GPIO function is selected.
     * |        |          |(1, 0) = SPI2_MISO1 function is selected.
     * |        |          |(1, 1) = ADC4 function is selected.
     * |[5]     |GPD_MFP5  |PD.5 Pin Function Selection
     * |        |          |Bits PD5_MFP1 (ALT_MFP1[21]) and GPD_MFP[5] determine the PD.5 function.
     * |        |          |(PD5_MFP1(ALT_MFP1[21]), GPD_MFP[5]) value and function mapping is as following list.
     * |        |          |(0, 0) = GPIO function is selected.
     * |        |          |(1, 0) = SPI2_MOSI1 function is selected.
     * |        |          |(1, 1) = ADC5 function is selected.
     * |[8]     |GPD_MFP8  |PD.8 Pin Function Selection
     * |        |          |0 = GPIO function is selected.
     * |        |          |1 = SPI1_MOSI0 (SPI1 master output, slave input pin-0) function is selected.
     * |[9]     |GPD_MFP9  |PD.9 Pin Function Selection
     * |        |          |The pin is a dedicated GPIO pin.
     * |        |          |0 = GPIO function is selected.
     * |[10]    |GPD_MFP10 |PD.10 Pin Function Selection
     * |        |          |0 = GPIO function is selected.
     * |        |          |1 = CLKO function is selected.
     * |[11]    |GPD_MFP11 |PD.11 Pin Function Selection
     * |        |          |0 = GPIO function is selected.
     * |        |          |1 = /INT1 function is selected.
     * |[31:16] |GPD_TYPEn |0 = GPIOD[15:0] I/O input Schmitt Trigger function Disabled.
     * |        |          |1 = GPIOD[15:0] I/O input Schmitt Trigger function Enabled.
     */
    __IO uint32_t GPD_MFP;

    /**
     * @cond HIDDEN_SYMBOLS
     */
    uint32_t RESERVE3;
    /**
     * @endcond
     */

    /**
     * GPF_MFP
     * ===================================================================================================
     * Offset: 0x44  GPIOF Multiple Function and Input Type Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |GPF_MFP0  |PF.0 Pin Function Selection
     * |        |          |The reset value of this bit controlled by CFOSC of User Configuration Config0[26:24].
     * |        |          |If CFOSC = 000, the reset value of this bit is 1.
     * |        |          |If CFOSC = 111, the reset value of this bit is 0.
     * |        |          |0 = GPIO function is selected to the pin PF.0.
     * |        |          |1 = XT1_OUT function is selected to the pin PF.0.
     * |[1]     |GPF_MFP1  |PF.1 Pin Function Selection
     * |        |          |The reset value of this bit controlled by CFOSC of User Configuration Config0[26:24].
     * |        |          |If CFOSC = 000, the reset value of this bit is 1.
     * |        |          |If CFOSC = 111, the reset value of this bit is 0.
     * |        |          |0 = GPIO function is selected to the pin PF.1.
     * |        |          |1 = XT1_IN function is selected to the pin PF.1.
     * |[2]     |GPF_MFP2  |PF.2 Pin Function Selection
     * |        |          |PF2_MFP1 (ALT_MFP1[25:24]) and GPF_MFP[2] determine the PF.2 function.
     * |        |          |The reset value of this bit is 1.
     * |        |          |(PF2_MFP1(ALT_MFP1[25:24]), GPF_MFP[2]) value and function mapping is as following list.
     * |        |          |(00, 0) = GPIO function is selected.
     * |        |          |(00, 1) = PS2_DAT function is selected.
     * |        |          |(10, 1) = I2C0_SDA function is selected.
     * |        |          |(11, 1) = ADC6 function is selected.
     * |[3]     |GPF_MFP3  |PF.3 Pin Function Selection
     * |        |          |PF3_MFP1 (ALT_MFP1[27:26]) and GPF_MFP[3] determine the PF.3 function.
     * |        |          |The reset value of this bit is 1.
     * |        |          |(PF3_MFP1(ALT_MFP1[27:26]), GPF_MFP[3]) value and function mapping is as following list.
     * |        |          |(00, 0) = GPIO function is selected.
     * |        |          |(00, 1) = PS2_CLK function is selected.
     * |        |          |(10, 1) = I2C0_SCL function is selected.
     * |        |          |(11, 1) = ADC7 function is selected.
     * |[19:16] |GPF_TYPEn |0 = GPIOF[3:0] I/O input Schmitt Trigger function Disabled.
     * |        |          |1 = GPIOF[3:0] I/O input Schmitt Trigger function Enabled.
     */
    __IO uint32_t GPF_MFP;

    /**
     * @cond HIDDEN_SYMBOLS
     */
    uint32_t RESERVE4[2];
    /**
     * @endcond
     */

    /**
     * ALT_MFP
     * ===================================================================================================
     * Offset: 0x50  Alternative Multiple Function Pin Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |PB10_MFP1 |PB.10 Pin Alternate Function Selection
     * |        |          |The pin function depends on GPB_MFP[10] and PB10_MFP1 (ALT_MFP[0]).
     * |        |          |(PB10_MFP1(ALT_MFP[0]), GPB_MFP[10]) value and function mapping is as following list.
     * |        |          |(0, 0) = GPIO function is selected.
     * |        |          |(0, 1) = TM2 function is selected.
     * |        |          |(1, 1) = SPI0_SS1 function is selected.
     * |[1]     |PB9_MFP1  |PB.9 Pin Alternate Function Selection
     * |        |          |The pin function depends on GPB_MFP[9] and PB9_MFP1 (ALT_MFP[1]).
     * |        |          |(PB9_MFP1(ALT_MFP[1]), GPB_MFP[9]) value and function mapping is as following list.
     * |        |          |(0, 0) = GPIO function is selected.
     * |        |          |(0, 1) = TM1 function is selected.
     * |        |          |(1, 1) = SPI1_SS1 function is selected.
     * |[5]     |PC0_MFP1  |PC.0 Pin Alternate Function Selection
     * |        |          |Bits PC0_MFP1 (ALT_MFP[5]) and GPC_MFP[0] determine the PC.0 function.
     * |        |          |(PC0_MFP1(ALT_MFP[5]), GPC_MFP[0]) value and function mapping is as following list.
     * |        |          |(0, 0) = GPIO function is selected.
     * |        |          |(0, 1) = SPI0_SS0 function is selected.
     * |        |          |(1, 1) = I2S_LRCLK function is selected.
     * |[6]     |PC1_MFP1  |PC.1 Pin Alternate Function Selection
     * |        |          |Bits PC1_MFP1 (ALT_MFP[6]) and GPC_MFP[1] determine the PC.1 function.
     * |        |          |(PC1_MFP1(ALT_MFP[6]), GPC_MFP[1]) value and function mapping is as following list.
     * |        |          |(0, 0) = GPIO function is selected.
     * |        |          |(0, 1) = SPI0_CLK function is selected.
     * |        |          |(1, 1) = I2S_BCLK function is selected.
     * |[7]     |PC2_MFP1  |PC.2 Pin Alternate Function Selection
     * |        |          |Bits PC2_MFP1 (ALT_MFP[7]) and GPC_MFP[2] determine the PC.2 function.
     * |        |          |(PC2_MFP1(ALT_MFP[7]), GPC_MFP[2]) value and function mapping is as following list.
     * |        |          |(0, 0) = GPIO function is selected.
     * |        |          |(0, 1) = SPI0_MISO0 function is selected.
     * |        |          |(1, 1) = I2S_DI function is selected.
     * |[8]     |PC3_MFP1  |PC.3 Pin Alternate Function Selection
     * |        |          |Bits PC3_MFP1 (ALT_MFP[8]) and GPC_MFP[3] determine the PC.3 function.
     * |        |          |(PC3_MFP1(ALT_MFP[8]), GPC_MFP[3]) value and function mapping is as following list.
     * |        |          |(0, 0) = GPIO function is selected.
     * |        |          |(0, 1) = SPI0_MOSI0 function is selected.
     * |        |          |(1, 1) = I2S_DO function is selected.
     * |[9]     |PA15_MFP1 |PA.15 Pin Alternate Function Selection
     * |        |          |The pin function depends on GPA_MFP[15] and PA15_MFP1 (ALT_MFP[9]).
     * |        |          |(PA15_MFP1(ALT_MFP[9]), GPA_MFP[15]) value and function mapping is as following list.
     * |        |          |(0, 0) = GPIO function is selected.
     * |        |          |(0, 1) = PWM3 function is selected.
     * |        |          |(1, 0) = CLKO function is selected.
     * |        |          |(1, 1) = I2S_MCLK function is selected.
     * |[10]    |PB12_MFP1 |PB.12 Pin Alternate Function Selection
     * |        |          |The pin function depends on GPB_MFP[12] and PB12_MFP1 (ALT_MFP[10]).
     * |        |          |(PB12_MFP1(ALT_MFP[10]), GPB_MFP[12]) value and function mapping is as following list.
     * |        |          |(0, 0) = GPIO function is selected.
     * |        |          |(0, 1) = SPI1_SS0 function is selected.
     * |        |          |(1, 1) = CLKO function is selected.
     * |[11]    |PA11_MFP1 |PA.11 Pin Alternate Function Selection
     * |        |          |The pin function depends on GPA_MFP[11] and PA11_MFP1 (ALT_MFP[11]).
     * |        |          |(PA11_MFP1(ALT_MFP[11]), GPA_MFP[11]) value and function mapping is as following list.
     * |        |          |(0, 0) = GPIO function is selected.
     * |        |          |(0, 1) = I2C1_SCL function is selected.
     * |        |          |(1, 0) = SPI1_CLK function is selected.
     * |        |          |(1, 1) = SPI2_MOSI0 function is selected.
     * |[12]    |PA10_MFP1 |PA.10 Pin Alternate Function Selection
     * |        |          |The pin function depends on GPA_MFP[10] and PA10_MFP1(ALT_MFP[12]).
     * |        |          |(PA10_MFP1(ALT_MFP[12]), GPA_MFP[10]) value and function mapping is as following list.
     * |        |          |(0, 0) = GPIO function is selected.
     * |        |          |(0, 1) = I2C1_SDA function is selected.
     * |        |          |(1, 0) = SPI1_MISO0 function is selected.
     * |        |          |(1, 1) = SPI2_MISO0 function is selected.
     * |[15]    |PB4_MFP1  |PB.4 Pin Alternate Function Selection
     * |        |          |The pin function depends on GPB_MFP[4] and PB4_MFP1 (ALT_MFP[15]).
     * |        |          |(PB4_MFP1(ALT_MFP[15]), GPB_MFP[4]) value and function mapping is as following list.
     * |        |          |(0, 0) = GPIO function is selected.
     * |        |          |(0, 1) = UART1_RXD function is selected.
     * |        |          |(1, 0) = SPI2_SS0 function is selected.
     * |        |          |(1, 1) = SPI1_SS1 function is selected.
     * |[16]    |PB7_MFP1  |PB.7 Pin Alternate Alternate Function Selection
     * |        |          |The pin function depends on GPB_MFP[7] and PB7_MFP1 (ALT_MFP[16]).
     * |        |          |(PB7_MFP1(ALT_MFP[16]), GPB_MFP[7]) value and function mapping is as following list.
     * |        |          |(0, 0) = GPIO function is selected.
     * |        |          |(0, 1) = UART1_nCTS function is selected.
     * |        |          |(1, 1) = SPI2_MISO0 function is selected.
     * |[17]    |PB6_MFP1  |PB.6 Pin Alternate Alternate Function Selection
     * |        |          |The pin function depends on GPB_MFP[6] and PB6_MFP1 (ALT_MFP[17]).
     * |        |          |(PB6_MFP1(ALT_MFP[17]), GPB_MFP[6]) value and function mapping is as following list.
     * |        |          |(0, 0) = GPIO function is selected.
     * |        |          |(0, 1) = UART1_nRTS function is selected.
     * |        |          |(1, 1) = SPI2_MOSI0 function is selected.
     * |[18]    |PB5_MFP1  |PB. 5 Pin Alternate Function Selection
     * |        |          |The pin function depends on GPB_MFP[5] and PB5_MFP1 (ALT_MFP[18]).
     * |        |          |(PB5_MFP1(ALT_MFP[18]), GPB_MFP[5]) value and function mapping is as following list.
     * |        |          |(0, 0) = GPIO function is selected.
     * |        |          |(0, 1) = UART1_TXD function is selected.
     * |        |          |(1, 1) = SPI2_CLK2 function is selected.
     * |[20]    |PC12_MFP1 |PC.12 Pin Alternate Function Selection
     * |        |          |Bits PC12_MFP1 (ALT_MFP[20]) and GPC_MFP[12] determine the PC.12 function.
     * |        |          |(PC12_MFP1(ALT_MFP[20]), GPC_MFP[12]) value and function mapping is as following list.
     * |        |          |(0, 0) = GPIO function is selected.
     * |        |          |(0, 1) = SPI1_MISO1 function is selected.
     * |        |          |(1, 0) = I2S_MCLK function is selected.
     * |        |          |(1, 1) = PWM2 function is selected.
     * |[21]    |PC13_MFP1 |PC.13 Pin Alternate Function Selection
     * |        |          |Bits PC13_MFP1 (ALT_MFP[21]) and GPC_MFP[13] determine the PC.13 function.
     * |        |          |(PC13_MFP1(ALT_MFP[21]), GPC_MFP[13]) value and function mapping is as following list.
     * |        |          |(0, 0) = GPIO function is selected.
     * |        |          |(0, 1) = SPI1_MOSI1 function is selected.
     * |        |          |(1, 0) = CLKO function is selected.
     * |        |          |(1, 1) = PWM3 function is selected.
     * |[24]    |PB15_MFP1 |PB.15 Pin Alternate Function Selection
     * |        |          |Bits PB15_MFP1 (ALT_MFP[24]) and GPB_MFP[15] determine the PB.15 function.
     * |        |          |(PB15_MFP1(ALT_MFP[24]), GPB_MFP[15])  value and function mapping is as following list.
     * |        |          |(0, 0) = GPIO function is selected.
     * |        |          |(0, 1) = /INT1 function is selected.
     * |        |          |(1, 1) = TM0_EXT function is selected.
     * |[26]    |PB2_MFP1  |PB.2 Pin Alternate Function Selection
     * |        |          |The pin function depends on GPB_MFP[2] and PB2_MFP1 (ALT_MFP[26]).
     * |        |          |(PB2_MFP1(ALT_MFP[26]), GPB_MFP[2]) value and function mapping is as following list.
     * |        |          |(0, 0) = GPIO function is selected.
     * |        |          |(0, 1) = UART0_nRTS function is selected.
     * |        |          |(1, 1) = TM2_EXT function is selected.
     * |[27]    |PB3_MFP1  |PB.3 Pin Alternate Function Selection
     * |        |          |The pin function depends on GPB_MFP[3] and PB3_MFP1 (ALT_MFP[27]).
     * |        |          |(PB3_MFP1(ALT_MFP[27]), GPB_MFP[3]) value and function mapping is as following list.
     * |        |          |(0, 0) = GPIO function is selected.
     * |        |          |(0, 1) = UART0_nCTS function is selected.
     * |        |          |(1, 1) = TM3_EXT function is selected.
     * |[29]    |PC4_MFP1  |PC.4 Pin Alternate Function Selection
     * |        |          |Bits PC4_MFP1 (ALT_MFP[29]) and GPC_MFP[4] determine the PC.4 function.
     * |        |          |(PC4_MFP1(ALT_MFP[29]), GPC_MFP[4]) value and function mapping is as following list.
     * |        |          |(0, 0) = GPIO function is selected.
     * |        |          |(0, 1) = SPI0_MISO1 function is selected.
     * |        |          |(1, 1) = UART0_RXD function is selected.
     * |[30]    |PC5_MFP1  |PC.5 Pin Alternate Function Selection
     * |        |          |Bits PC5_MFP1 (ALT_MFP[30]) and GPC_MFP[5] determine the PC.5 function.
     * |        |          |(PC5_MFP1(ALT_MFP[30]), GPC_MFP[5]) value and function mapping is as following list.
     * |        |          |(0, 0) = GPIO function is selected.
     * |        |          |(0, 1) = SPI0_MOSI1 function is selected.
     * |        |          |(1, 1) = UART0_TXD function is selected.
     */
    __IO uint32_t ALT_MFP;

    /**
     * ALT_MFP1
     * ===================================================================================================
     * Offset: 0x54  Alternative Multiple Function Pin Control Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[16]    |PD0_MFP1  |PD.0 Pin Function Selection
     * |        |          |Bits PD0_MFP1 (ALT_MFP1[16]) and GPD_MFP[0] determine the PD.0 function.
     * |        |          |(PD0_MFP1(ALT_MFP1[16]), GPD_MFP[0]) value and function mapping is as following list.
     * |        |          |(0, 0) = GPIO function is selected.
     * |        |          |(1, 0) = SPI2_SS0 function is selected.
     * |        |          |(1, 1) = ADC0 function is selected.
     * |[17]    |PD1_MFP1  |PD.1 Pin Function Selection
     * |        |          |Bits PD1_MFP1 (ALT_MFP1[17]) and GPD_MFP[1] determine the PD.1 function.
     * |        |          |(PD1_MFP1(ALT_MFP1[17]), GPD_MFP[1]) value and function mapping is as following list.
     * |        |          |(0, 0) = GPIO function is selected.
     * |        |          |(0, 1) = SPI0_SS1 function is selected.
     * |        |          |(1, 0) = SPI2_CLK function is selected.
     * |        |          |(1, 1) = ADC1 function is selected.
     * |[18]    |PD2_MFP1  |PD.2 Pin Function Selection
     * |        |          |Bits PD2_MFP1 (ALT_MFP1[18]) and GPD_MFP[2] determine the PD.2 function.
     * |        |          |(PD2_MFP1(ALT_MFP1[18]), GPD_MFP[2]) value and function mapping is as following list.
     * |        |          |(0, 0) = GPIO function is selected.
     * |        |          |(0, 1) = SPI0_MISO1 function is selected.
     * |        |          |(1, 0) = SPI2_MISO0 function is selected.
     * |        |          |(1, 1) = ADC2 function is selected.
     * |[19]    |PD3_MFP1  |PD.3 Pin Function Selection
     * |        |          |Bits PD3_MFP1 (ALT_MFP1[19]) and GPD_MFP[3] determine the PD.3 function.
     * |        |          |(PD3_MPF1(ALT_MFP1[19]), GPD_MFP[3]) value and function mapping is as following list.
     * |        |          |(0, 0) = GPIO function is selected.
     * |        |          |(0, 1) = SPI0_MOSI1 function is selected.
     * |        |          |(1, 0) = SPI2_MOSI0 function is selected.
     * |        |          |(1, 1) = ADC3 function is selected.
     * |[20]    |PD4_MFP1  |PD.4 Pin Function Selection
     * |        |          |Bits PD4_MFP1 (ALT_MFP1[20]) and GPD_MFP[4] determine the PD.4 function.
     * |        |          |(PD4_MFP1(ALT_MFP1[20]), GPD_MFP[4]) value and function mapping is as following list.
     * |        |          |(0, 0) = GPIO function is selected.
     * |        |          |(1, 0) = SPI2_MISO1 function is selected.
     * |        |          |(1, 1) = ADC4 function is selected.
     * |[21]    |PD5_MFP1  |PD.5 Pin Function Selection
     * |        |          |Bits PD5_MFP1 (ALT_MFP1[21]) and GPD_MFP[5] determine the PD.5 function.
     * |        |          |(PD5_MFP1(ALT_MFP1[21]), GPD_MFP[5]) value and function mapping is as following list.
     * |        |          |(0, 0) = GPIO function is selected.
     * |        |          |(1, 0) = SPI2_MOSI1 function is selected.
     * |        |          |(1, 1) = ADC5 function is selected.
     * |[25:24] |PF2_MFP1  |PF.2 Pin Function Selection
     * |        |          |PF2_MFP1 (ALT_MFP1[25:24]) and GPF_MFP[2] determine the PF.2 function.
     * |        |          |The reset value of this bit is 1.
     * |        |          |(PF2_MFP1(ALT_MFP1[25:24]), GPF_MFP[2]) value and function mapping is as following list.
     * |        |          |(00, 0) = GPIO function is selected.
     * |        |          |(00, 1) = PS2_DAT function is selected.
     * |        |          |(10, 1) = I2C0_SDA function is selected.
     * |        |          |(11, 1) = ADC6 function is selected.
     * |[27:26] |PF3_MFP1  |PF.3 Pin Function Selection
     * |        |          |PF3_MFP1 (ALT_MFP1[27:26]) and GPF_MFP[3] determine the PF.3 function.
     * |        |          |The reset value of this bit is 1.
     * |        |          |(PF3_MFP1(ALT_MFP1[27:26]), GPF_MFP[3]) value and function mapping is as following list.
     * |        |          |(00, 0) = GPIO function is selected.
     * |        |          |(00, 1) = PS2_CLK function is selected.
     * |        |          |(10, 1) = I2C0_SCL function is selected.
     * |        |          |(11, 1) = ADC7 function is selected.
     */
    __IO uint32_t ALT_MFP1;

    /**
     * @cond HIDDEN_SYMBOLS
     */
    uint32_t RESERVE5[26];
    /**
     * @endcond
     */

    /**
     * GPA_IOCR
     * ===================================================================================================
     * Offset: 0xC0  GPIOA IO Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[10]    |GPA10_DS  |PA.10 Pin Driving Strength Selection
     * |        |          |0 = PA.10 strong driving strength mode Disabled.
     * |        |          |1 = PA.10 strong driving strength mode Enabled.
     * |[11]    |GPA11_DS  |PA.11 Pin Driving Strength Selection
     * |        |          |0 = PA.11 strong driving strength mode Disabled.
     * |        |          |1 = PA.11 strong driving strength mode Enabled.
     */
    __IO uint32_t GPA_IOCR;

    /**
     * GPB_IOCR
     * ===================================================================================================
     * Offset: 0xC4  GPIOB IO Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[4]     |GPB4_DS   |PB.4 Pin Driving Strength Selection
     * |        |          |0 = PB.4 strong driving strength mode Disabled.
     * |        |          |1 = PB.4 strong driving strength mode Enabled.
     * |[5]     |GPB5_DS   |PB.5 Pin Driving Strength Selection
     * |        |          |0 = PB.5 strong driving strength mode Disabled.
     * |        |          |1 = PB.5 strong driving strength mode Enabled.
     * |[6]     |GPB6_DS   |PB.6 Pin Driving Strength Selection
     * |        |          |0 = PB.6 strong driving strength mode Disabled.
     * |        |          |1 = PB.6 strong driving strength mode Enabled.
     * |[7]     |GPB7_DS   |PB.7 Pin Driving Strength Selection
     * |        |          |0 = PB.7 strong driving strength mode Disabled.
     * |        |          |1 = PB.7 strong driving strength mode Enabled.
     * |[8]     |GPB8_DS   |PB.8 Pin Driving Strength Selection
     * |        |          |0 = PB.8 strong driving strength mode Disabled.
     * |        |          |1 = PB.8 strong driving strength mode Enabled.
     * |[12]    |GPB12_DS  |PB.12 Pin Driving Strength Selection
     * |        |          |0 = PB.12 strong driving strength mode Disabled.
     * |        |          |1 = PB.12 strong driving strength mode Enabled.
     * |[13]    |GPB13_DS  |PB.13 Pin Driving Strength Selection
     * |        |          |0 = PB.13 strong driving strength mode Disabled.
     * |        |          |1 = PB.13 strong driving strength mode Enabled.
     * |[14]    |GPB14_DS  |PB.14 Pin Driving Strength Selection
     * |        |          |0 = PB.14 strong driving strength mode Disabled.
     * |        |          |1 = PB.14 strong driving strength mode Enabled.
     */
    __IO uint32_t GPB_IOCR;

    /**
     * @cond HIDDEN_SYMBOLS
     */
    uint32_t RESERVE6[1];
    /**
     * @endcond
     */

    /**
     * GPD_IOCR
     * ===================================================================================================
     * Offset: 0xCC  GPIOD IO Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[8]     |GPD8_DS   |PD.8 Pin Driving Strength Selection
     * |        |          |0 = PD.8 strong driving strength mode Disabled.
     * |        |          |1 = PD.8 strong driving strength mode Enabled.
     * |[9]     |GPD9_DS   |PD.9 Pin Driving Strength Selection
     * |        |          |0 = PD.9 strong driving strength mode Disabled.
     * |        |          |1 = PD.9 strong driving strength mode Enabled.
     * |[10]    |GPD10_DS  |PD.10 Pin Driving Strength Selection
     * |        |          |0 = PD.10 strong driving strength mode Disabled.
     * |        |          |1 = PD.10 strong driving strength mode Enabled.
     * |[11]    |GPD11_DS  |PD.11 Pin Driving Strength Selection
     * |        |          |0 = PD.11 strong driving strength mode Disabled.
     * |        |          |1 = PD.11 strong driving strength mode Enabled.
     */
    __IO uint32_t GPD_IOCR;

    /**
     * @cond HIDDEN_SYMBOLS
     */
    uint32_t RESERVE7[12];
    /**
     * @endcond
     */

    /**
     * REGWRPROT
     * ===================================================================================================
     * Offset: 0x100  Register Write Protect register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |REGPROTDIS|Register Write-Protection Disable index (Read Only)
     * |        |          |1 = Write-protection Disabled for writing protected registers.
     * |        |          |0 = Write-protection Enabled for writing protected registers.
     * |        |          |Any write to the protected register is ignored.
     * |        |          |The Protected registers are:
     * |        |          |IPRSTC1: address 0x5000_0008
     * |        |          |BODCR: address 0x5000_0018
     * |        |          |PORCR: address 0x5000_0024
     * |        |          |PWRCON: address 0x5000_0200 (bit[6] is not protected for power wake-up interrupt clear)
     * |        |          |APBCLK bit[0]: address 0x5000_0208 (bit[0] is watchdog clock enabled)
     * |        |          |CLKSEL0: address 0x5000_0210 (for HCLK and CPU STCLK clock source select)
     * |        |          |CLKSEL1 bit[1:0]: address 0x5000_0214 (for watchdog clock source select)
     * |        |          |ISPCON: address 0x5000_C000 (Flash ISP Control register)
     * |        |          |WTCR: address 0x4000_4000
     * |        |          |FATCON: address 0x5000_C018
     * |[7:0]   |REGWRPROT |Register Write-Protection Code (Write Only)
     * |        |          |Some registers have write-protection function.
     * |        |          |Writing these registers has to disable the protected function by writing the sequence value "59h", "16h", "88h" to this field.
     * |        |          |After this sequence is completed, the REGPROTDIS bit will be set to 1 and write-protection registers can be normal write.
     */
    __IO uint32_t REGWRPROT;

} GCR_T;


/** @addtogroup REG_SYS_BITMASK SYS Bit Mask
  @{
 */

/* GCR RSTSRC Bit Field Definitions */
#define SYS_RSTSRC_RSTS_CPU_Pos                 7                                   /*!< GCR RSTSRC: RSTS_CPU Position */
#define SYS_RSTSRC_RSTS_CPU_Msk                 (1ul << SYS_RSTSRC_RSTS_CPU_Pos)    /*!< GCR RSTSRC: RSTS_CPU Mask */

#define SYS_RSTSRC_RSTS_SYS_Pos                 5                                   /*!< GCR RSTSRC: RSTS_SYS Position */
#define SYS_RSTSRC_RSTS_SYS_Msk                 (1ul << SYS_RSTSRC_RSTS_SYS_Pos)    /*!< GCR RSTSRC: RSTS_SYS Mask */

#define SYS_RSTSRC_RSTS_BOD_Pos                 4                                   /*!< GCR RSTSRC: RSTS_BOD Position */
#define SYS_RSTSRC_RSTS_BOD_Msk                 (1ul << SYS_RSTSRC_RSTS_BOD_Pos)    /*!< GCR RSTSRC: RSTS_BOD Mask */

#define SYS_RSTSRC_RSTS_LVR_Pos                 3                                   /*!< GCR RSTSRC: RSTS_LVR Position */
#define SYS_RSTSRC_RSTS_LVR_Msk                 (1ul << SYS_RSTSRC_RSTS_LVR_Pos)    /*!< GCR RSTSRC: RSTS_LVR Mask */

#define SYS_RSTSRC_RSTS_WDT_Pos                 2                                   /*!< GCR RSTSRC: RSTS_WDT Position */
#define SYS_RSTSRC_RSTS_WDT_Msk                 (1ul << SYS_RSTSRC_RSTS_WDT_Pos)    /*!< GCR RSTSRC: RSTS_WDT Mask */

#define SYS_RSTSRC_RSTS_RESET_Pos               1                                   /*!< GCR RSTSRC: RSTS_RESET Position */
#define SYS_RSTSRC_RSTS_RESET_Msk               (1ul << SYS_RSTSRC_RSTS_RESET_Pos)  /*!< GCR RSTSRC: RSTS_RESET Mask */

#define SYS_RSTSRC_RSTS_POR_Pos                 0                                   /*!< GCR RSTSRC: RSTS_POR Position */
#define SYS_RSTSRC_RSTS_POR_Msk                 (1ul << SYS_RSTSRC_RSTS_POR_Pos)    /*!< GCR RSTSRC: RSTS_POR Mask */

/* GCR IPRSTC1 Bit Field Definitions */
#define SYS_IPRSTC1_PDMA_RST_Pos                2                                   /*!< GCR IPRSTC1: PDMA_RST Position */
#define SYS_IPRSTC1_PDMA_RST_Msk                (1ul << SYS_IPRSTC1_PDMA_RST_Pos)   /*!< GCR IPRSTC1: PDMA_RST Mask */

#define SYS_IPRSTC1_CPU_RST_Pos                 1                                   /*!< GCR IPRSTC1: CPU_RST Position */
#define SYS_IPRSTC1_CPU_RST_Msk                 (1ul << SYS_IPRSTC1_CPU_RST_Pos)    /*!< GCR IPRSTC1: CPU_RST Mask */

#define SYS_IPRSTC1_CHIP_RST_Pos                0                                   /*!< GCR IPRSTC1: CHIP_RST Position */
#define SYS_IPRSTC1_CHIP_RST_Msk                (1ul << SYS_IPRSTC1_CHIP_RST_Pos)   /*!< GCR IPRSTC1: CHIP_RST Mask */

/* GCR IPRSTC2 Bit Field Definitions */
#define SYS_IPRSTC2_I2S_RST_Pos                 29                                  /*!< GCR IPRSTC2: I2S_RST Position */
#define SYS_IPRSTC2_I2S_RST_Msk                 (1ul << SYS_IPRSTC2_I2S_RST_Pos)    /*!< GCR IPRSTC2: I2S_RST Mask */

#define SYS_IPRSTC2_ADC_RST_Pos                 28                                  /*!< GCR IPRSTC2: ADC_RST Position */
#define SYS_IPRSTC2_ADC_RST_Msk                 (1ul << SYS_IPRSTC2_ADC_RST_Pos)    /*!< GCR IPRSTC2: ADC_RST Mask */

#define SYS_IPRSTC2_USBD_RST_Pos                27                                  /*!< GCR IPRSTC2: USBD_RST Position */
#define SYS_IPRSTC2_USBD_RST_Msk                (1ul << SYS_IPRSTC2_USBD_RST_Pos)   /*!< GCR IPRSTC2: USBD_RST Mask */

#define SYS_IPRSTC2_PS2_RST_Pos                 23                                  /*!< GCR IPRSTC2: PS2_RST Position */
#define SYS_IPRSTC2_PS2_RST_Msk                 (1ul << SYS_IPRSTC2_PS2_RST_Pos)    /*!< GCR IPRSTC2: PS2_RST Mask */

#define SYS_IPRSTC2_PWM03_RST_Pos               20                                  /*!< GCR IPRSTC2: PWM03_RST Position */
#define SYS_IPRSTC2_PWM03_RST_Msk               (1ul << SYS_IPRSTC2_PWM03_RST_Pos)  /*!< GCR IPRSTC2: PWM03_RST Mask */

#define SYS_IPRSTC2_UART1_RST_Pos               17                                  /*!< GCR IPRSTC2: UART1_RST Position */
#define SYS_IPRSTC2_UART1_RST_Msk               (1ul << SYS_IPRSTC2_UART1_RST_Pos)  /*!< GCR IPRSTC2: UART1_RST Mask */

#define SYS_IPRSTC2_UART0_RST_Pos               16                                  /*!< GCR IPRSTC2: UART0_RST Position */
#define SYS_IPRSTC2_UART0_RST_Msk               (1ul << SYS_IPRSTC2_UART0_RST_Pos)  /*!< GCR IPRSTC2: UART0_RST Mask */

#define SYS_IPRSTC2_SPI2_RST_Pos                14                                  /*!< GCR IPRSTC2: SPI2_RST Position */
#define SYS_IPRSTC2_SPI2_RST_Msk                (1ul << SYS_IPRSTC2_SPI2_RST_Pos)   /*!< GCR IPRSTC2: SPI2_RST Mask */

#define SYS_IPRSTC2_SPI1_RST_Pos                13                                  /*!< GCR IPRSTC2: SPI1_RST Position */
#define SYS_IPRSTC2_SPI1_RST_Msk                (1ul << SYS_IPRSTC2_SPI1_RST_Pos)   /*!< GCR IPRSTC2: SPI1_RST Mask */

#define SYS_IPRSTC2_SPI0_RST_Pos                12                                  /*!< GCR IPRSTC2: SPI0_RST Position */
#define SYS_IPRSTC2_SPI0_RST_Msk                (1ul << SYS_IPRSTC2_SPI0_RST_Pos)   /*!< GCR IPRSTC2: SPI0_RST Mask */

#define SYS_IPRSTC2_I2C1_RST_Pos                9                                   /*!< GCR IPRSTC2: I2C1_RST Position */
#define SYS_IPRSTC2_I2C1_RST_Msk                (1ul << SYS_IPRSTC2_I2C1_RST_Pos)   /*!< GCR IPRSTC2: I2C1_RST Mask */

#define SYS_IPRSTC2_I2C0_RST_Pos                8                                   /*!< GCR IPRSTC2: I2C0_RST Position */
#define SYS_IPRSTC2_I2C0_RST_Msk                (1ul << SYS_IPRSTC2_I2C0_RST_Pos)   /*!< GCR IPRSTC2: I2C0_RST Mask */

#define SYS_IPRSTC2_TMR3_RST_Pos                5                                   /*!< GCR IPRSTC2: TMR3_RST Position */
#define SYS_IPRSTC2_TMR3_RST_Msk                (1ul << SYS_IPRSTC2_TMR3_RST_Pos)   /*!< GCR IPRSTC2: TMR3_RST Mask */

#define SYS_IPRSTC2_TMR2_RST_Pos                4                                   /*!< GCR IPRSTC2: TMR2_RST Position */
#define SYS_IPRSTC2_TMR2_RST_Msk                (1ul << SYS_IPRSTC2_TMR2_RST_Pos)   /*!< GCR IPRSTC2: TMR2_RST Mask */

#define SYS_IPRSTC2_TMR1_RST_Pos                3                                   /*!< GCR IPRSTC2: TMR1_RST Position */
#define SYS_IPRSTC2_TMR1_RST_Msk                (1ul << SYS_IPRSTC2_TMR1_RST_Pos)   /*!< GCR IPRSTC2: TMR1_RST Mask */

#define SYS_IPRSTC2_TMR0_RST_Pos                2                                   /*!< GCR IPRSTC2: TMR0_RST Position */
#define SYS_IPRSTC2_TMR0_RST_Msk                (1ul << SYS_IPRSTC2_TMR0_RST_Pos)   /*!< GCR IPRSTC2: TMR0_RST Mask */

#define SYS_IPRSTC2_GPIO_RST_Pos                1                                   /*!< GCR IPRSTC2: GPIO_RST Position */
#define SYS_IPRSTC2_GPIO_RST_Msk                (1ul << SYS_IPRSTC2_GPIO_RST_Pos)   /*!< GCR IPRSTC2: GPIO_RST Mask */

/* GCR BODCR Bit Field Definitions */
#define SYS_BODCR_LVR_EN_Pos                    7                                   /*!< GCR BODCR: LVR_EN Position */
#define SYS_BODCR_LVR_EN_Msk                    (1ul << SYS_BODCR_LVR_EN_Pos)       /*!< GCR BODCR: LVR_EN Mask */

#define SYS_BODCR_BOD_OUT_Pos                   6                                   /*!< GCR BODCR: BOD_OUT Position */
#define SYS_BODCR_BOD_OUT_Msk                   (1ul << SYS_BODCR_BOD_OUT_Pos)      /*!< GCR BODCR: BOD_OUT Mask */

#define SYS_BODCR_BOD_LPM_Pos                   5                                   /*!< GCR BODCR: BOD_LPM Position */
#define SYS_BODCR_BOD_LPM_Msk                   (1ul << SYS_BODCR_BOD_LPM_Pos)      /*!< GCR BODCR: BOD_LPM Mask */

#define SYS_BODCR_BOD_INTF_Pos                  4                                   /*!< GCR BODCR: BOD_INTF Position */
#define SYS_BODCR_BOD_INTF_Msk                  (1ul << SYS_BODCR_BOD_INTF_Pos)     /*!< GCR BODCR: BOD_INTF Mask */

#define SYS_BODCR_BOD_RSTEN_Pos                 3                                   /*!< GCR BODCR: BOD_RSTEN Position */
#define SYS_BODCR_BOD_RSTEN_Msk                 (1ul << SYS_BODCR_BOD_RSTEN_Pos)    /*!< GCR BODCR: BOD_RSTEN Mask */

#define SYS_BODCR_BOD_VL_Pos                    1                                   /*!< GCR BODCR: BOD_VL Position */
#define SYS_BODCR_BOD_VL_Msk                    (3ul << SYS_BODCR_BOD_VL_Pos)       /*!< GCR BODCR: BOD_VL Mask */

#define SYS_BODCR_BOD_EN_Pos                    0                                   /*!< GCR BODCR: BOD_EN Position */
#define SYS_BODCR_BOD_EN_Msk                    (1ul << SYS_BODCR_BOD_EN_Pos)       /*!< GCR BODCR: BOD_EN Mask */

/* GCR PORCR Bit Field Definitions */
#define SYS_PORCR_POR_DIS_CODE_Pos              0                                           /*!< GCR PORCR: POR_DIS_CODE Position */
#define SYS_PORCR_POR_DIS_CODE_Msk              (0xFFFFul << SYS_PORCR_POR_DIS_CODE_Pos)    /*!< GCR PORCR: POR_DIS_CODE Mask */

/* GCR GPAMFP Bit Field Definitions */
#define SYS_GPA_MFP_GPA_TYPE_Pos                 16                                         /*!< GCR GPA_MFP: GPA_TYPE Position */
#define SYS_GPA_MFP_GPA_TYPE_Msk                 (0xFFFFul << SYS_GPA_MFP_GPA_TYPE_Pos)     /*!< GCR GPA_MFP: GPA_TYPE Mask */

#define SYS_GPA_MFP_GPA_MFP_Pos                  0                                          /*!< GCR GPA_MFP: GPA_MFP Position */
#define SYS_GPA_MFP_GPA_MFP_Msk                  (0xFFFFul << SYS_GPA_MFP_GPA_MFP_Pos)      /*!< GCR GPA_MFP: GPA_MFP Mask */


/* GCR GPBMFP Bit Field Definitions */
#define SYS_GPB_MFP_GPB_TYPE_Pos                 16                                         /*!< GCR GPB_MFP: GPB_TYPE Position */
#define SYS_GPB_MFP_GPB_TYPE_Msk                 (0xFFFFul << SYS_GPB_MFP_GPB_TYPE_Pos)     /*!< GCR GPB_MFP: GPB_TYPE Mask */

#define SYS_GPB_MFP_GPB_MFP_Pos                  0                                          /*!< GCR GPB_MFP: GPB_MFP Position */
#define SYS_GPB_MFP_GPB_MFP_Msk                  (0xFFFFul << SYS_GPB_MFP_GPB_MFP_Pos)      /*!< GCR GPB_MFP: GPB_MFP Mask */

/* GCR GPCMFP Bit Field Definitions */
#define SYS_GPC_MFP_GPC_TYPE_Pos                 16                                         /*!< GCR GPC_MFP: GPC_TYPE Position */
#define SYS_GPC_MFP_GPC_TYPE_Msk                 (0xFFFFul << SYS_GPC_MFP_GPC_TYPE_Pos)     /*!< GCR GPC_MFP: GPC_TYPE Mask */

#define SYS_GPC_MFP_GPC_MFP_Pos                  0                                          /*!< GCR GPC_MFP: GPC_MFP Position */
#define SYS_GPC_MFP_GPC_MFP_Msk                  (0xFFFFul << SYS_GPC_MFP_GPC_MFP_Pos)      /*!< GCR GPC_MFP: GPC_MFP Mask */

/* GCR GPDMFP Bit Field Definitions */
#define SYS_GPD_MFP_GPD_TYPE_Pos                 16                                         /*!< GCR GPD_MFP: GPD_TYPE Position */
#define SYS_GPD_MFP_GPD_TYPE_Msk                 (0xFFFFul << SYS_GPD_MFP_GPD_TYPE_Pos)     /*!< GCR GPD_MFP: GPD_TYPE Mask */

#define SYS_GPD_MFP_GPD_MFP_Pos                  0                                          /*!< GCR GPD_MFP: GPD_MFP Position */
#define SYS_GPD_MFP_GPD_MFP_Msk                  (0xFFFFul << SYS_GPD_MFP_GPD_MFP_Pos)      /*!< GCR GPD_MFP: GPD_MFP Mask */

/* GCR GPFMFP Bit Field Definitions */
#define SYS_GPF_MFP_GPF_TYPE_Pos                 16                                         /*!< GCR GPF_MFP: GPF_TYPE Position */
#define SYS_GPF_MFP_GPF_TYPE_Msk                 (0xFul << SYS_GPF_MFP_GPF_TYPE_Pos)        /*!< GCR GPF_MFP: GPF_TYPE Mask */

#define SYS_GPF_MFP_GPF_MFP3_Pos                 3                                          /*!< GCR GPF_MFP: GPF_MFP3 Position */
#define SYS_GPF_MFP_GPF_MFP3_Msk                 (1ul << SYS_GPF_MFP_GPF_MFP3_Pos)          /*!< GCR GPF_MFP: GPF_MFP3 Mask */

#define SYS_GPF_MFP_GPF_MFP2_Pos                 2                                          /*!< GCR GPF_MFP: GPF_MFP2 Position */
#define SYS_GPF_MFP_GPF_MFP2_Msk                 (1ul << SYS_GPF_MFP_GPF_MFP2_Pos)          /*!< GCR GPF_MFP: GPF_MFP2 Mask */

#define SYS_GPF_MFP_GPF_MFP1_Pos                 1                                          /*!< GCR GPF_MFP: GPF_MFP1 Position */
#define SYS_GPF_MFP_GPF_MFP1_Msk                 (1ul << SYS_GPF_MFP_GPF_MFP1_Pos)          /*!< GCR GPF_MFP: GPF_MFP1 Mask */

#define SYS_GPF_MFP_GPF_MFP0_Pos                 0                                          /*!< GCR GPF_MFP: GPF_MFP0 Position */
#define SYS_GPF_MFP_GPF_MFP0_Msk                 (1ul << SYS_GPF_MFP_GPF_MFP0_Pos)          /*!< GCR GPF_MFP: GPF_MFP0 Mask */

/* GCR ALTMFP Bit Field Definitions */
#define SYS_ALT_MFP_PC5_MFP1_Pos                 30                                         /*!< GCR ALT_MFP: PC5_MFP1 Position */
#define SYS_ALT_MFP_PC5_MFP1_Msk                 (1ul << SYS_ALT_MFP_PC5_MFP1_Pos)          /*!< GCR ALT_MFP: PC5_MFP1 Mask */

#define SYS_ALT_MFP_PC4_MFP1_Pos                 29                                         /*!< GCR ALT_MFP: PC4_MFP1 Position */
#define SYS_ALT_MFP_PC4_MFP1_Msk                 (1ul << SYS_ALT_MFP_PC4_MFP1_Pos)          /*!< GCR ALT_MFP: PC4_MFP1 Mask */

#define SYS_ALT_MFP_PB3_MFP1_Pos                 27                                         /*!< GCR ALT_MFP: PB3_MFP1 Position */
#define SYS_ALT_MFP_PB3_MFP1_Msk                 (1ul << SYS_ALT_MFP_PB3_MFP1_Pos)          /*!< GCR ALT_MFP: PB3_MFP1 Mask */

#define SYS_ALT_MFP_PB2_MFP1_Pos                 26                                         /*!< GCR ALT_MFP: PB2_MFP1 Position */
#define SYS_ALT_MFP_PB2_MFP1_Msk                 (1ul << SYS_ALT_MFP_PB2_MFP1_Pos)          /*!< GCR ALT_MFP: PB2_MFP1 Mask */

#define SYS_ALT_MFP_PB15_MFP1_Pos                24                                         /*!< GCR ALT_MFP: PB15_MFP1 Position */
#define SYS_ALT_MFP_PB15_MFP1_Msk                (1ul << SYS_ALT_MFP_PB15_MFP1_Pos)         /*!< GCR ALT_MFP: PB15_MFP1 Mask */

#define SYS_ALT_MFP_PC13_MFP1_Pos                21                                         /*!< GCR ALT_MFP: PC13_MFP1 Position */
#define SYS_ALT_MFP_PC13_MFP1_Msk                (1ul << SYS_ALT_MFP_PC13_MFP1_Pos)         /*!< GCR ALT_MFP: PC13_MFP1 Mask */

#define SYS_ALT_MFP_PC12_MFP1_Pos                20                                         /*!< GCR ALT_MFP: PC12_MFP1 Position */
#define SYS_ALT_MFP_PC12_MFP1_Msk                (1ul << SYS_ALT_MFP_PC12_MFP1_Pos)         /*!< GCR ALT_MFP: PC12_MFP1 Mask */

#define SYS_ALT_MFP_PB5_MFP1_Pos                 18                                         /*!< GCR ALT_MFP: PB5_MFP1 Position */
#define SYS_ALT_MFP_PB5_MFP1_Msk                 (1ul << SYS_ALT_MFP_PB5_MFP1_Pos)          /*!< GCR ALT_MFP: PB5_MFP1 Mask */

#define SYS_ALT_MFP_PB6_MFP1_Pos                 17                                         /*!< GCR ALT_MFP: PB6_MFP1 Position */
#define SYS_ALT_MFP_PB6_MFP1_Msk                 (1ul << SYS_ALT_MFP_PB6_MFP1_Pos)          /*!< GCR ALT_MFP: PB6_MFP1 Mask */

#define SYS_ALT_MFP_PB7_MFP1_Pos                 16                                         /*!< GCR ALT_MFP: PB7_MFP1 Position */
#define SYS_ALT_MFP_PB7_MFP1_Msk                 (1ul << SYS_ALT_MFP_PB7_MFP1_Pos)          /*!< GCR ALT_MFP: PB7_MFP1 Mask */

#define SYS_ALT_MFP_PB4_MFP1_Pos                 15                                         /*!< GCR ALT_MFP: PB4_MFP1 Position */
#define SYS_ALT_MFP_PB4_MFP1_Msk                 (1ul << SYS_ALT_MFP_PB4_MFP1_Pos)          /*!< GCR ALT_MFP: PB4_MFP1 Mask */

#define SYS_ALT_MFP_PA10_MFP1_Pos                12                                         /*!< GCR ALT_MFP: PA10_MFP1 Position */
#define SYS_ALT_MFP_PA10_MFP1_Msk                (1ul << SYS_ALT_MFP_PA10_MFP1_Pos)         /*!< GCR ALT_MFP: PA10_MFP1 Mask */

#define SYS_ALT_MFP_PA11_MFP1_Pos                11                                         /*!< GCR ALT_MFP: PA11_MFP1 Position */
#define SYS_ALT_MFP_PA11_MFP1_Msk                (1ul << SYS_ALT_MFP_PA11_MFP1_Pos)         /*!< GCR ALT_MFP: PA11_MFP1 Mask */

#define SYS_ALT_MFP_PB12_MFP1_Pos                10                                         /*!< GCR ALT_MFP: PB12_MFP1 Position */
#define SYS_ALT_MFP_PB12_MFP1_Msk                (1ul << SYS_ALT_MFP_PB12_MFP1_Pos)         /*!< GCR ALT_MFP: PB12_MFP1 Mask */

#define SYS_ALT_MFP_PA15_MFP1_Pos                9                                          /*!< GCR ALT_MFP: A15_MFP1 Position */
#define SYS_ALT_MFP_PA15_MFP1_Msk                (1ul << SYS_ALT_MFP_PA15_MFP1_Pos)         /*!< GCR ALT_MFP: A15_MFP1 Mask */

#define SYS_ALT_MFP_PC3_MFP1_Pos                 8                                          /*!< GCR ALT_MFP: PC3_MFP1 Position */
#define SYS_ALT_MFP_PC3_MFP1_Msk                 (1ul << SYS_ALT_MFP_PC3_MFP1_Pos)          /*!< GCR ALT_MFP: PC3_MFP1 Mask */

#define SYS_ALT_MFP_PC2_MFP1_Pos                 7                                          /*!< GCR ALT_MFP: PC2_MFP1 Position */
#define SYS_ALT_MFP_PC2_MFP1_Msk                 (1ul << SYS_ALT_MFP_PC2_MFP1_Pos)          /*!< GCR ALT_MFP: PC2_MFP1 Mask */

#define SYS_ALT_MFP_PC1_MFP1_Pos                 6                                          /*!< GCR ALT_MFP: PC1_MFP1 Position */
#define SYS_ALT_MFP_PC1_MFP1_Msk                 (1ul << SYS_ALT_MFP_PC1_MFP1_Pos)          /*!< GCR ALT_MFP: PC1_MFP1 Mask */

#define SYS_ALT_MFP_PC0_MFP1_Pos                 5                                          /*!< GCR ALT_MFP: PC0_MFP1 Position */
#define SYS_ALT_MFP_PC0_MFP1_Msk                 (1ul << SYS_ALT_MFP_PC0_MFP1_Pos)          /*!< GCR ALT_MFP: PB0_MFP1 Mask */

#define SYS_ALT_MFP_PB9_MFP1_Pos                 1                                          /*!< GCR ALT_MFP: PB9_MFP1 Position */
#define SYS_ALT_MFP_PB9_MFP1_Msk                 (1ul << SYS_ALT_MFP_PB9_MFP1_Pos)          /*!< GCR ALT_MFP: PB9_MFP1 Mask */

#define SYS_ALT_MFP_PB10_MFP1_Pos                0                                          /*!< GCR ALT_MFP: PB10_MFP1 Position */
#define SYS_ALT_MFP_PB10_MFP1_Msk                (1ul << SYS_ALT_MFP_PB10_MFP1_Pos)         /*!< GCR ALT_MFP: PB10_MFP1 Mask */

/* GCR ALTMFP1 Bit Field Definitions */
#define SYS_ALT_MFP1_PF3_MFP1_Pos                26                                         /*!< GCR ALT_MFP1: PF3_MFP1 Position */
#define SYS_ALT_MFP1_PF3_MFP1_Msk                (0x3ul << SYS_ALT_MFP1_PF3_MFP1_Pos)       /*!< GCR ALT_MFP1: PF3_MFP1 Mask */

#define SYS_ALT_MFP1_PF2_MFP1_Pos                24                                         /*!< GCR ALT_MFP1: PF2_MFP1 Position */
#define SYS_ALT_MFP1_PF2_MFP1_Msk                (0x3ul << SYS_ALT_MFP1_PF2_MFP1_Pos)       /*!< GCR ALT_MFP1: PF2_MFP1 Mask */

#define SYS_ALT_MFP1_PC8_MFP1_Pos                23                                         /*!< GCR ALT_MFP1: PC8_MFP1 Position */
#define SYS_ALT_MFP1_PC8_MFP1_Msk                (1ul << SYS_ALT_MFP1_PC8_MFP1_Pos)         /*!< GCR ALT_MFP1: PC8_MFP1 Mask */

#define SYS_ALT_MFP1_PD5_MFP1_Pos                21                                         /*!< GCR ALT_MFP1: PD5_MFP1 Position */
#define SYS_ALT_MFP1_PD5_MFP1_Msk                (1ul << SYS_ALT_MFP1_PD5_MFP1_Pos)         /*!< GCR ALT_MFP1: PD5_MFP1 Mask */

#define SYS_ALT_MFP1_PD4_MFP1_Pos                20                                         /*!< GCR ALT_MFP1: PD4_MFP1 Position */
#define SYS_ALT_MFP1_PD4_MFP1_Msk                (1ul << SYS_ALT_MFP1_PD4_MFP1_Pos)         /*!< GCR ALT_MFP1: PD4_MFP1 Mask */

#define SYS_ALT_MFP1_PD3_MFP1_Pos                19                                         /*!< GCR ALT_MFP1: PD3_MFP1 Position */
#define SYS_ALT_MFP1_PD3_MFP1_Msk                (1ul << SYS_ALT_MFP1_PD3_MFP1_Pos)         /*!< GCR ALT_MFP1: PD3_MFP1 Mask */

#define SYS_ALT_MFP1_PD2_MFP1_Pos                18                                         /*!< GCR ALT_MFP1: PD2_MFP1 Position */
#define SYS_ALT_MFP1_PD2_MFP1_Msk                (1ul << SYS_ALT_MFP1_PD2_MFP1_Pos)         /*!< GCR ALT_MFP1: PD2_MFP1 Mask */

#define SYS_ALT_MFP1_PD1_MFP1_Pos                17                                         /*!< GCR ALT_MFP1: PD1_MFP1 Position */
#define SYS_ALT_MFP1_PD1_MFP1_Msk                (1ul << SYS_ALT_MFP1_PD1_MFP1_Pos)         /*!< GCR ALT_MFP1: PD1_MFP1 Mask */

#define SYS_ALT_MFP1_PD0_MFP1_Pos                16                                         /*!< GCR ALT_MFP1: PD0_MFP1 Position */
#define SYS_ALT_MFP1_PD0_MFP1_Msk                (1ul << SYS_ALT_MFP1_PD0_MFP1_Pos)         /*!< GCR ALT_MFP1: PD0_MFP1 Mask */

/* GCR GPA_IOCR Bit Field Definitions */
#define SYS_GPA_IOCR_GPA11_DS_Pos                11                                         /*!< GCR GPA_IOCR: GPA11_DS Position */
#define SYS_GPA_IOCR_GPA11_DS_Msk                (0x3ul << SYS_GPA_IOCR_GPA11_DS_Pos)       /*!< GCR GPA_IOCR: GPA11_DS Mask */

#define SYS_GPA_IOCR_GPA10_DS_Pos                10                                         /*!< GCR GPA_IOCR: GPA10_DS Position */
#define SYS_GPA_IOCR_GPA10_DS_Msk                (0x3ul << SYS_GPA_IOCR_GPA10_DS_Pos)       /*!< GCR GPA_IOCR: GPA10_DS Mask */

/* GCR GPB_IOCR Bit Field Definitions */
#define SYS_GPB_IOCR_GPB14_DS_Pos                14                                         /*!< GCR GPB_IOCR: GPB14_DS Position */
#define SYS_GPB_IOCR_GPB14_DS_Msk                (0x3ul << SYS_GPB_IOCR_GPB14_DS_Pos)       /*!< GCR GPB_IOCR: GPB14_DS Mask */

#define SYS_GPB_IOCR_GPB13_DS_Pos                13                                         /*!< GCR GPB_IOCR: GPB13_DS Position */
#define SYS_GPB_IOCR_GPB13_DS_Msk                (0x3ul << SYS_GPB_IOCR_GPB13_DS_Pos)       /*!< GCR GPB_IOCR: GPB13_DS Mask */

#define SYS_GPB_IOCR_GPB12_DS_Pos                12                                         /*!< GCR GPB_IOCR: GPB12_DS Position */
#define SYS_GPB_IOCR_GPB12_DS_Msk                (0x3ul << SYS_GPB_IOCR_GPB12_DS_Pos)       /*!< GCR GPB_IOCR: GPB12_DS Mask */

#define SYS_GPB_IOCR_GPB8_DS_Pos                 8                                          /*!< GCR GPB_IOCR: GPB8_DS Position */
#define SYS_GPB_IOCR_GPB8_DS_Msk                 (0x3ul << SYS_GPB_IOCR_GPB8_DS_Pos)        /*!< GCR GPB_IOCR: GPB8_DS Mask */

#define SYS_GPB_IOCR_GPB7_DS_Pos                 7                                          /*!< GCR GPB_IOCR: GPB7_DS Position */
#define SYS_GPB_IOCR_GPB7_DS_Msk                 (0x3ul << SYS_GPB_IOCR_GPB7_DS_Pos)        /*!< GCR GPB_IOCR: GPB7_DS Mask */

#define SYS_GPB_IOCR_GPB6_DS_Pos                 6                                          /*!< GCR GPB_IOCR: GPB6_DS Position */
#define SYS_GPB_IOCR_GPB6_DS_Msk                 (0x3ul << SYS_GPB_IOCR_GPB6_DS_Pos)        /*!< GCR GPB_IOCR: GPB6_DS Mask */

#define SYS_GPB_IOCR_GPB5_DS_Pos                 5                                          /*!< GCR GPB_IOCR: GPB5_DS Position */
#define SYS_GPB_IOCR_GPB5_DS_Msk                 (0x3ul << SYS_GPB_IOCR_GPB5_DS_Pos)        /*!< GCR GPB_IOCR: GPB5_DS Mask */

#define SYS_GPB_IOCR_GPB4_DS_Pos                 4                                          /*!< GCR GPB_IOCR: GPB4_DS Position */
#define SYS_GPB_IOCR_GPB4_DS_Msk                 (0x3ul << SYS_GPB_IOCR_GPB4_DS_Pos)        /*!< GCR GPB_IOCR: GPB4_DS Mask */

/* GCR GPD_IOCR Bit Field Definitions */
#define SYS_GPD_IOCR_GPD11_DS_Pos                11                                         /*!< GCR GPD_IOCR: GPD11_DS Position */
#define SYS_GPD_IOCR_GPD11_DS_Msk                (0x3ul << SYS_GPD_IOCR_GPD11_DS_Pos)       /*!< GCR GPD_IOCR: GPD11_DS Mask */

#define SYS_GPD_IOCR_GPD10_DS_Pos                10                                         /*!< GCR GPD_IOCR: GPD10_DS Position */
#define SYS_GPD_IOCR_GPD10_DS_Msk                (0x3ul << SYS_GPD_IOCR_GPD10_DS_Pos)       /*!< GCR GPD_IOCR: GPD10_DS Mask */

#define SYS_GPD_IOCR_GPD9_DS_Pos                 9                                          /*!< GCR GPD_IOCR: GPD9_DS Position */
#define SYS_GPD_IOCR_GPD9_DS_Msk                 (0x3ul << SYS_GPD_IOCR_GPD9_DS_Pos)        /*!< GCR GPD_IOCR: GPD9_DS Mask */

#define SYS_GPD_IOCR_GPD8_DS_Pos                 8                                          /*!< GCR GPD_IOCR: GPD8_DS Position */
#define SYS_GPD_IOCR_GPD8_DS_Msk                 (0x3ul << SYS_GPD_IOCR_GPD8_DS_Pos)        /*!< GCR GPD_IOCR: GPD8_DS Mask */

/* GCR REGWRPROT Bit Field Definitions */
#define SYS_REGWRPROT_REGWRPROT_Pos              0                                          /*!< GCR REGWRPROT: REGWRPROT Position */
#define SYS_REGWRPROT_REGWRPROT_Msk              (0xFFul << SYS_REGWRPROT_REGWRPROT_Pos)    /*!< GCR REGWRPROT: REGWRPROT Mask */

#define SYS_REGWRPROT_REGPROTDIS_Pos             0                                          /*!< GCR REGWRPROT: REGPROTDIS Position */
#define SYS_REGWRPROT_REGPROTDIS_Msk             (1ul << SYS_REGWRPROT_REGPROTDIS_Pos)      /*!< GCR REGWRPROT: REGPROTDIS Mask */


/*@}*/ /* end of group REG_SYS_BITMASK */


typedef struct
{
    __I uint32_t IRQSRC[32];
    __IO uint32_t NMISEL;
    __IO uint32_t MCUIRQ;
} GCR_INT_T;

/** @addtogroup REG_INT_BITMASK INT Bit Mask
  @{
 */

/* INT IRQSRC Bit Field Definitions */
#define INT_IRQSRC_INT_SRC_Pos                  0                                   /*!< INT IRQSRC: INT_SRC Position */
#define INT_IRQSRC_INT_SRC_Msk                  (0xFul << INT_IRQSRC_INT_SRC_Pos)

/* INT NMI_SEL Bit Field Definitions */
#define INT_NMI_SEL_NMI_EN_Pos                  8                                   /*!< INT NMI_SEL: NMI_EN Position */
#define INT_NMI_SEL_NMI_EN_Msk                  (1ul << INT_NMI_SEL_NMI_EN_Pos)     /*!< INT NMI_SEL: NMI_EN Mask */

#define INT_NMI_SEL_NMI_SEL_Pos                 0                                   /*!< INT NMI_SEL: NMI_SEL Position */
#define INT_NMI_SEL_NMI_SEL_Msk                 (0x1Ful << INT_NMI_SEL_NMI_SEL_Pos) /*!< INT NMI_SEL: NMI_SEL Mask */
/*@}*/ /* end of group REG_SYS_BITMASK */
/*@}*/ /* end of group REG_SYS*/

/*----------------------------- Timer Controller (TMR) -----------------------------*/
/** @addtogroup REG_TIMER TIMER
  Memory Mapped Structure for Timer Controller
  @{
 */
typedef struct
{
    /**
     * TCSR
     * ===================================================================================================
     * Offset: 0x00  Timer Control and Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |PRESCALE  |Prescale Counter
     * |        |          |Timer input clock source is divided by (PRESCALE+1) before it is fed to the Timer up counter.
     * |        |          |If this field is 0 (PRESCALE = 0), then there is no scaling.
     * |[16]    |TDR_EN    |Data Load Enable Control
     * |        |          |When TDR_EN is set, TDR (Timer Data Register) will be updated continuously with the 24-bit up-timer value as the timer is counting.
     * |        |          |0 = Timer Data Register update Disabled.
     * |        |          |1 = Timer Data Register update Enabled while Timer counter is active.
     * |[23]    |WAKE_EN   |Wake Up Function Enable Control
     * |        |          |0 = Wake-up trigger event Disabled.
     * |        |          |1 = Wake-up trigger event Enabled.
     * |[24]    |CTB       |Counter Mode Enable Control
     * |        |          |This bit is for external counting pin function enabled.
     * |        |          |When timer is used as an event counter, this bit should be set to 1 and select HCLK as timer clock source.
     * |        |          |0 = External counter mode Disabled.
     * |        |          |1 = External counter mode Enabled.
     * |[25]    |CACT      |Timer Active Status (Read Only)
     * |        |          |This bit indicates the 24-bit up counter status.
     * |        |          |0 = 24-bit up counter is not active.
     * |        |          |1 = 24-bit up counter is active.
     * |[26]    |CRST      |Timer Reset
     * |        |          |0 = No effect.
     * |        |          |1 = Reset 8-bit prescale counter, 24-bit up counter value and CEN bit if CACT is 1.
     * |[28:27] |MODE      |Timer Operating Mode
     * |        |          |00 = The Timer controller is operated in One-shot mode.
     * |        |          |01 = The Timer controller is operated in Periodic mode.
     * |        |          |10 = The Timer controller is operated in Toggle-output mode.
     * |        |          |11 = The Timer controller is operated in Continuous Counting mode.
     * |[29]    |IE        |Interrupt Enable Control
     * |        |          |0 = Timer Interrupt function Disabled.
     * |        |          |1 = Timer Interrupt function Enabled.
     * |        |          |If this bit is enabled, when the timer interrupt flag (TISR[0] TIF) is set to 1, the timer interrupt signal is generated and inform to CPU.
     * |[30]    |CEN       |Timer Enable Control
     * |        |          |0 = Stops/Suspends counting.
     * |        |          |1 = Starts counting.
     * |        |          |Note1: In stop status, and then set CEN to 1 will enable the 24-bit up counter to keep counting from the last stop counting value.
     * |        |          |Note2: This bit is auto-cleared by hardware in one-shot mode (TCSR [28:27] = 00) when the timer interrupt flag (TISR[0] TIF) is generated.
     * |[31]    |DBGACK_TMR|ICE Debug Mode Acknowledge Disable (Write Protect)
     * |        |          |0 = ICE debug mode acknowledgment effects TIMER counting.
     * |        |          |TIMER counter will be held while CPU is held by ICE.
     * |        |          |1 = ICE debug mode acknowledgment Disabled.
     * |        |          |TIMER counter will keep going no matter CPU is held by ICE or not.
     */
    __IO uint32_t  TCSR;

    /**
     * TCMPR
     * ===================================================================================================
     * Offset: 0x04  Timer Compare Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[23:0]  |TCMP      |Timer Compared Value
     * |        |          |TCMP is a 24-bit compared value register.
     * |        |          |When the internal 24-bit up counter value is equal to TCMP value, the TIF flag will set to 1.
     * |        |          |Time-out period = (Period of Timer clock input) * (8-bit PRESCALE + 1) * (24-bit TCMP).
     * |        |          |Note1: Never write 0x0 or 0x1 in TCMP field, or the core will run into unknown state.
     * |        |          |Note2: When timer is operating at continuous counting mode, the 24-bit up counter will keep counting continuously even if user writes a new value into TCMP field.
     * |        |          |But if timer is operating at other modes, the 24-bit up counter will restart counting and using newest TCMP value to be the timer compared value if user writes a new value into TCMP field.
     */
    __IO uint32_t  TCMPR;

    /**
     * TISR
     * ===================================================================================================
     * Offset: 0x08  Timer Interrupt Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |TIF       |Timer Interrupt Flag
     * |        |          |This bit indicates the interrupt flag status of Timer while TDR value reaches to TCMP value.
     * |        |          |0 = No effect.
     * |        |          |1 = TDR value matches the TCMP value.
     * |        |          |Note: This bit is cleared by writing 1 to it.
     * |[1]     |TWF       |Timer Wake-Up Flag
     * |        |          |This bit indicates the interrupt wake-up flag status of Timer.
     * |        |          |0 = Timer does not cause CPU wake-up.
     * |        |          |1 = CPU wake-up from Idle or Power-down mode if Timer time-out interrupt signal generated.
     * |        |          |Note: This bit is cleared by writing 1 to it.
     */
    __IO uint32_t  TISR;

    /**
     * TDR
     * ===================================================================================================
     * Offset: 0x0C  Timer Data Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[23:0]  |TDR       |Timer Data Register
     * |        |          |If TDR_EN (TCSR[16]) is set to 1, TDR register will be updated continuously to monitor 24-bit up counter value.
     */
    __I  uint32_t  TDR;

    /**
     * TCAP
     * ===================================================================================================
     * Offset: 0x10  Timer Capture Data Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[23:0]  |TCAP      |Timer Capture Data Register
     * |        |          |When TEXIF flag is set to 1, the current TDR value will be auto-loaded into this TCAP filed immediately.
     */
    __I  uint32_t  TCAP;

    /**
     * TEXCON
     * ===================================================================================================
     * Offset: 0x14  Timer External Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |TX_PHASE  |Timer External Count Pin Phase Detect Selection
     * |        |          |This bit indicates the detection phase of TMx pin.
     * |        |          |0 = A falling edge of TMx pin will be counted.
     * |        |          |1 = A rising edge of TMx pin will be counted.
     * |[2:1]   |TEX_EDGE  |Timer External Capture Pin Edge Detect Selection
     * |        |          |00 = A 1 to 0 transition on TMx_EXT pin will be detected.
     * |        |          |01 = A 0 to 1 transition on TMx_EXT pin will be detected.
     * |        |          |10 = Either 1 to 0 or 0 to 1 transition on TMx_EXT pin will be detected.
     * |        |          |11 = Reserved.
     * |[3]     |TEXEN     |Timer External Pin Function Enable
     * |        |          |This bit enables the RSTCAPSEL function on the TxEX pin.
     * |        |          |0 = RSTCAPSEL function of TxEX pin will be ignored.
     * |        |          |1 = RSTCAPSEL function of TxEX pin is active.
     * |[4]     |RSTCAPSEL |Timer External Reset Counter / Timer External Capture Mode Selection
     * |        |          |0 = Transition on TMx_EXT
     * |        |          |pin is using to save the TDR value into TCAP value if TEXIF flag is set to 1.
     * |        |          |1 = Transition on TMx_EXT pin is using to reset the 24-bit up counter.
     * |[5]     |TEXIEN    |Timer External Capture Interrupt Enable Control
     * |        |          |0 = TMx_EXT pin detection Interrupt Disabled.
     * |        |          |1 = TMx_EXT pin detection Interrupt Enabled.
     * |        |          |If TEXIEN enabled, Timer will raise an external capture interrupt signal and inform to CPU while TEXIF flag is set to 1.
     * |[6]     |TEXDB     |Timer External Capture Input Pin De-Bounce Enable Control
     * |        |          |0 = TMx_EXT pin de-bounce Disabled.
     * |        |          |1 = TMx_EXT pin de-bounce Enabled.
     * |        |          |If this bit is enabled, the edge detection of TMx_EXT pin is detected with de-bounce circuit.
     * |[7]     |TCDB      |Timer External Counter Input Pin De-Bounce Enable Control
     * |        |          |0 = TMx pin de-bounce Disabled.
     * |        |          |1 = TMx pin de-bounce Enabled.
     * |        |          |If this bit is enabled, the edge detection of TMx pin is detected with de-bounce circuit.
     */
    __IO uint32_t  TEXCON;

    /**
     * TEXISR
     * ===================================================================================================
     * Offset: 0x18  Timer External Interrupt Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |TEXIF     |Timer External Capture Interrupt Flag
     * |        |          |This bit indicates the external capture interrupt flag status.
     * |        |          |When TEXEN enabled, TMx_EXT pin selected as external capture function, and a transition on TMx_EXT pin matched the TEX_EDGE setting, this flag will set to 1 by hardware.
     * |        |          |1 = TMx_EXT
     * |        |          |pin interrupt occurred.
     * |        |          |0 = TMx_EXT
     * |        |          |pin interrupt did not occur.
     * |        |          |Note: This bit is cleared by writing 1 to it.
     */
    __IO uint32_t  TEXISR;
} TIMER_T;


/** @addtogroup REG_TIMER_BITMASK TIMER Bit Mask
  @{
 */


/* TIMER TCSR Bit Field Definitions */
#define TIMER_TCSR_DBGACK_TMR_Pos   31                                          /*!< TIMER TCSR: DBGACK_TMR Position */
#define TIMER_TCSR_DBGACK_TMR_Msk   (1ul << TIMER_TCSR_DBGACK_TMR_Pos)          /*!< TIMER TCSR: DBGACK_TMR Mask */

#define TIMER_TCSR_CEN_Pos          30                                          /*!< TIMER TCSR: CEN Position */
#define TIMER_TCSR_CEN_Msk          (1ul << TIMER_TCSR_CEN_Pos)                 /*!< TIMER TCSR: CEN Mask */

#define TIMER_TCSR_IE_Pos           29                                          /*!< TIMER TCSR: IE Position */
#define TIMER_TCSR_IE_Msk           (1ul << TIMER_TCSR_IE_Pos)                  /*!< TIMER TCSR: IE Mask */

#define TIMER_TCSR_MODE_Pos         27                                          /*!< TIMER TCSR: MODE Position */
#define TIMER_TCSR_MODE_Msk         (0x3ul << TIMER_TCSR_MODE_Pos)              /*!< TIMER TCSR: MODE Mask */

#define TIMER_TCSR_CRST_Pos         26                                          /*!< TIMER TCSR: CRST Position */
#define TIMER_TCSR_CRST_Msk         (1ul << TIMER_TCSR_CRST_Pos)                /*!< TIMER TCSR: CRST Mask */

#define TIMER_TCSR_CACT_Pos         25                                          /*!< TIMER TCSR: CACT Position */
#define TIMER_TCSR_CACT_Msk         (1ul << TIMER_TCSR_CACT_Pos)                /*!< TIMER TCSR: CACT Mask */

#define TIMER_TCSR_CTB_Pos          24                                          /*!< TIMER TCSR: CTB Position */
#define TIMER_TCSR_CTB_Msk          (1ul << TIMER_TCSR_CTB_Pos)                 /*!< TIMER TCSR: CTB Mask */

#define TIMER_TCSR_WAKE_EN_Pos      23                                          /*!< TIMER TCSR: WAKE_EN Position */
#define TIMER_TCSR_WAKE_EN_Msk      (1ul << TIMER_TCSR_WAKE_EN_Pos)             /*!< TIMER TCSR: WAKE_EN Mask */

#define TIMER_TCSR_TDR_EN_Pos       16                                          /*!< TIMER TCSR: TDR_EN Position */
#define TIMER_TCSR_TDR_EN_Msk       (1ul << TIMER_TCSR_TDR_EN_Pos)              /*!< TIMER TCSR: TDR_EN Mask */

#define TIMER_TCSR_PRESCALE_Pos     0                                           /*!< TIMER TCSR: PRESCALE Position */
#define TIMER_TCSR_PRESCALE_Msk     (0xFFul << TIMER_TCSR_PRESCALE_Pos)         /*!< TIMER TCSR: PRESCALE Mask */

/* TIMER TCMPR Bit Field Definitions */
#define TIMER_TCMP_TCMP_Pos         0                                           /*!< TIMER TCMPR: TCMP Position */
#define TIMER_TCMP_TCMP_Msk         (0xFFFFFFul << TIMER_TCMP_TCMP_Pos)         /*!< TIMER TCMPR: TCMP Mask */

/* TIMER TISR Bit Field Definitions */
#define TIMER_TISR_TWF_Pos          1                                           /*!< TIMER TISR: TWF Position */
#define TIMER_TISR_TWF_Msk          (1ul << TIMER_TISR_TWF_Pos)                 /*!< TIMER TISR: TWF Mask */

#define TIMER_TISR_TIF_Pos          0                                           /*!< TIMER TISR: TIF Position */
#define TIMER_TISR_TIF_Msk          (1ul << TIMER_TISR_TIF_Pos)                 /*!< TIMER TISR: TIF Mask */

/* TIMER TDR Bit Field Definitions */
#define TIMER_TDR_TDR_Pos           0                                           /*!< TIMER TDR: TDR Position */
#define TIMER_TDR_TDR_Msk           (0xFFFFFFul << TIMER_TDR_TDR_Pos)           /*!< TIMER TDR: TDR Mask */

/* TIMER TCAP Bit Field Definitions */
#define TIMER_TCAP_TCAP_Pos         0                                           /*!< TIMER TCAP: TCAP Position */
#define TIMER_TCAP_TCAP_Msk         (0xFFFFFFul << TIMER_TCAP_TCAP_Pos)         /*!< TIMER TCAP: TCAP Mask */

/* TIMER TEXCON Bit Field Definitions */
#define TIMER_TEXCON_TCDB_Pos       7                                           /*!< TIMER TEXCON: TCDB Position */
#define TIMER_TEXCON_TCDB_Msk       (1ul << TIMER_TEXCON_TCDB_Pos)              /*!< TIMER TEXCON: TCDB Mask */

#define TIMER_TEXCON_TEXDB_Pos      6                                           /*!< TIMER TEXCON: TEXDB Position */
#define TIMER_TEXCON_TEXDB_Msk      (1ul << TIMER_TEXCON_TEXDB_Pos)             /*!< TIMER TEXCON: TEXDB Mask */

#define TIMER_TEXCON_TEXIEN_Pos     5                                           /*!< TIMER TEXCON: TEXIEN Position */
#define TIMER_TEXCON_TEXIEN_Msk     (1ul << TIMER_TEXCON_TEXIEN_Pos)            /*!< TIMER TEXCON: TEXIEN Mask */

#define TIMER_TEXCON_RSTCAPSEL_Pos  4                                           /*!< TIMER TEXCON: RSTCAPSEL Position */
#define TIMER_TEXCON_RSTCAPSEL_Msk  (1ul << TIMER_TEXCON_RSTCAPSEL_Pos)         /*!< TIMER TEXCON: RSTCAPSEL Mask */

#define TIMER_TEXCON_TEXEN_Pos      3                                           /*!< TIMER TEXCON: TEXEN Position */
#define TIMER_TEXCON_TEXEN_Msk      (1ul << TIMER_TEXCON_TEXEN_Pos)             /*!< TIMER TEXCON: TEXEN Mask */

#define TIMER_TEXCON_TEX_EDGE_Pos   1                                           /*!< TIMER TEXCON: TEX_EDGE Position */
#define TIMER_TEXCON_TEX_EDGE_Msk   (0x3ul << TIMER_TEXCON_TEX_EDGE_Pos)        /*!< TIMER TEXCON: TEX_EDGE Mask */

#define TIMER_TEXCON_TX_PHASE_Pos   0                                           /*!< TIMER TEXCON: TX_PHASE Position */
#define TIMER_TEXCON_TX_PHASE_Msk   (1ul << TIMER_TEXCON_TX_PHASE_Pos)          /*!< TIMER TEXCON: TX_PHASE Mask */

/* TIMER TEXISR Bit Field Definitions */
#define TIMER_TEXISR_TEXIF_Pos      0                                           /*!< TIMER TEXISR: TEXIF Position */
#define TIMER_TEXISR_TEXIF_Msk      (1ul << TIMER_TEXISR_TEXIF_Pos)             /*!< TIMER TEXISR: TEXIF Mask */
/*@}*/ /* end of group REG_TIMER_BITMASK */
/*@}*/ /* end of group REG_TIMER */


/*------------------------- UART Interface Controller ------------------------*/

/** @addtogroup REG_UART UART
  Memory Mapped Structure for UART Serial Interface Controller
  @{
 */
typedef struct
{

    union
    {
        __IO uint32_t DATA;

        /**
         * UA_THR
         * ===================================================================================================
         * Offset: 0x00 Transmit Holding DATA
         * ---------------------------------------------------------------------------------------------------
         * |Bits    |Field     |Descriptions
         * | :----: | :----:   | :---- |
         * |[7:0]   | THR      |Bit Transmitted Data
         * |        |          |Transmit Holding Register
         * |        |          |By writing to this register, the UART will send out an 8-bit data through the Tx pin (LSB
         * |        |          |first).
         */
        __IO uint32_t THR;

        /**
         * UA_RBR
         * ===================================================================================================
         * Offset: 0x00  UART Receive Buffer Register
         * ---------------------------------------------------------------------------------------------------
         * |Bits    |Field     |Descriptions
         * | :----: | :----:   | :---- |
         * |[7:0]   |RBR       |Receive Buffer Register (Read Only)
         * |        |          |By reading this register, the UART will return the 8-bit data received from RX pin
         * |        |          |(LSB first).
         */
        __IO uint32_t RBR;
    };

    /**
     * UA_IER
     * ===================================================================================================
     * Offset: 0x04  UART Interrupt Enable Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |RDA_IEN   |Receive Data Available Interrupt Enable Control
     * |        |          |0 = RDA_INT Masked off.
     * |        |          |1 = RDA_INT Enabled.
     * |[1]     |THRE_IEN  |Transmit Holding Register Empty Interrupt Enable Control
     * |        |          |0 = THRE_INT Masked off.
     * |        |          |1 = THRE_INT Enabled.
     * |[2]     |RLS_IEN   |Receive Line Status Interrupt Enable Control
     * |        |          |0 = RLS_INT Masked off.
     * |        |          |1 = RLS_INT Enabled
     * |[3]     |MODEM_IEN |Modem Status Interrupt Enable Control (Not Available In UART2 Channel)
     * |        |          |0 = MODEM_INT Masked off.
     * |        |          |1 = MODEM_INT Enabled.
     * |[4]     |RTO_IEN   |RX Time-Out Interrupt Enable Control
     * |        |          |0 = TOUT_INT Masked off.
     * |        |          |1 = TOUT_INT Enabled.
     * |[5]     |BUF_ERR_IEN|Buffer Error Interrupt Enable Control
     * |        |          |0 = BUF_ERR_INT Masked off.
     * |        |          |1 = BUF_ERR_INT Enabled.
     * |[6]     |WAKE_EN   |UART Wake-Up Function Enable (Not Available In UART2 Channel)
     * |        |          |0 = UART wake-up function Disabled.
     * |        |          |1 = UART wake-up function Enabled, when the chip is in Power-down mode, an external CTS change will wake-up chip from Power-down mode.
     * |[11]    |TIME_OUT_EN|Time-Out Counter Enable
     * |        |          |0 = Time-out counter Disabled.
     * |        |          |1 = Time-out counter Enabled.
     * |[12]    |AUTO_RTS_EN|RTS Auto Flow Control Enable (Not Available In UART2 Channel)
     * |        |          |0 = RTS auto flow control Disabled.
     * |        |          |1 = RTS auto flow control Enabled.
     * |        |          |When RTS auto-flow is enabled, if the number of bytes in the RX FIFO equals the RTS_TRI_LEV (UA_FCR [19:16]), the UART will de-assert RTS signal.
     * |[13]    |AUTO_CTS_EN|CTS Auto Flow Control Enable (Not Available In UART2 Channel)
     * |        |          |0 = CTS auto flow control Disabled.
     * |        |          |1 = CTS auto flow control Enabled.
     * |        |          |When CTS auto-flow is enabled, the UART will send data to external device when CTS input assert (UART will not send data to device until CTS is asserted).
     * |[14]    |DMA_TX_EN |TX DMA Enable (Not Available In UART2 Channel)
     * |        |          |This bit can enable or disable TX DMA service.
     * |        |          |0 = TX DMA Disabled.
     * |        |          |1 = TX DMA Enabled.
     * |[15]    |DMA_RX_EN |RX DMA Enable (Not Available In UART2 Channel)
     * |        |          |This bit can enable or disable RX DMA service.
     * |        |          |0 = RX DMA Disabled.
     * |        |          |1 = RX DMA Enabled.
     */
    __IO uint32_t IER;

    /**
     * UA_FCR
     * ===================================================================================================
     * Offset: 0x08  UART FIFO Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1]     |RFR       |RX Field Software Reset
     * |        |          |When RFR is set, all the byte in the receiver FIFO and RX internal state machine are cleared.
     * |        |          |0 = No effect.
     * |        |          |1 = Reset the RX internal state machine and pointers.
     * |        |          |Note: This bit will automatically clear at least 3 UART peripheral clock cycles.
     * |[2]     |TFR       |TX Field Software Reset
     * |        |          |When TFR is set, all the byte in the transmit FIFO and TX internal state machine are cleared.
     * |        |          |0 = No effect.
     * |        |          |1 = Reset the TX internal state machine and pointers.
     * |        |          |Note: This bit will automatically clear at least 3 UART peripheral clock cycles.
     * |[7:4]   |RFITL     |RX FIFO Interrupt (INT_RDA) Trigger Level
     * |        |          |When the number of bytes in the receive FIFO equals the RFITL, the RDA_IF will be set (if UA_IER [RDA_IEN] enabled, and an interrupt will be generated).
     * |        |          |0000 = RX FIFO Interrupt Trigger Level is 1 byte.
     * |        |          |0001 = RX FIFO Interrupt Trigger Level is 4 bytes.
     * |        |          |0010 = RX FIFO Interrupt Trigger Level is 8 bytes.
     * |        |          |0011 = RX FIFO Interrupt Trigger Level is 14 bytes.
     * |[8]     |RX_DIS    |Receiver Disable Register
     * |        |          |The receiver is disabled or not (set 1 to disable receiver)
     * |        |          |0 = Receiver Enabled.
     * |        |          |1 = Receiver Disabled.
     * |        |          |Note: This field is used for RS-485 Normal Multi-drop mode.
     * |        |          |It should be programmed before UA_ALT_CSR [RS-485_NMM] is programmed.
     * |[19:16] |RTS_TRI_LEV|RTS Trigger Level For Auto-Flow Control Use (Not Available In UART2 Channel)
     * |        |          |0000 = RTS Trigger Level is 1 byte.
     * |        |          |0001 = RTS Trigger Level is 4 bytes.
     * |        |          |0010 = RTS Trigger Level is 8 bytes.
     * |        |          |0011 = RTS Trigger Level is 14 bytes.
     * |        |          |Note: This field is used for RTS auto-flow control.
     */
    __IO uint32_t FCR;

    /**
     * UA_LCR
     * ===================================================================================================
     * Offset: 0x0C  UART Line Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1:0]   |WLS       |Word Length Selection
     * |        |          |00 = Word length is 5-bit.
     * |        |          |01 = Word length is 6-bit.
     * |        |          |10 = Word length is 7-bit
     * |        |          |11 = Word length is 8-bit
     * |[2]     |NSB       |Number Of "STOP Bit"
     * |        |          |0 = One " STOP bit" is generated in the transmitted data.
     * |        |          |1 = When select 5-bit word length, 1.5 "STOP bit" is generated in the transmitted data.
     * |        |          |When select 6-,7- and 8-bit word length, 2 "STOP bit" is generated in the transmitted data.
     * |[3]     |PBE       |Parity Bit Enable
     * |        |          |0 = No parity bit.
     * |        |          |1 = Parity bit is generated on each outgoing character and is checked on each incoming data.
     * |[4]     |EPE       |Even Parity Enable
     * |        |          |0 = Odd number of logic 1's is transmitted and checked in each word.
     * |        |          |1 = Even number of logic 1's is transmitted and checked in each word.
     * |        |          |This bit has effect only when PBE (UA_LCR[3]) is set.
     * |[5]     |SPE       |Stick Parity Enable
     * |        |          |0 = Stick parity Disabled.
     * |        |          |1 = If PBE (UA_LCR[3]) and EBE (UA_LCR[4]) are logic 1, the parity bit is transmitted and checked as logic 0.
     * |        |          |If PBE (UA_LCR[3]) is 1 and EBE (UA_LCR[4]) is 0 then the parity bit is transmitted and checked as 1.
     * |[6]     |BCB       |Break Control Bit
     * |        |          |When this bit is set to logic 1, the serial data output (TX) is forced to the Spacing State (logic 0).
     * |        |          |This bit acts only on TX and has no effect on the transmitter logic.
     */
    __IO uint32_t LCR;

    /**
     * UA_MCR
     * ===================================================================================================
     * Offset: 0x10  UART Modem Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1]     |RTS       |RTS (Request-To-Send) Signal Control (Not Available In UART2 Channel)
     * |        |          |This bit is direct control internal RTS signal active or not, and then drive the RTS pin output with LEV_RTS bit configuration.
     * |        |          |0 = RTS signal is active.
     * |        |          |1 = RTS signal is inactive.
     * |        |          |Note1: This RTS signal control bit is not effective when RTS auto-flow control is enabled in UART function mode.
     * |        |          |Note2: This RTS signal control bit is not effective when RS-485 auto direction mode (AUD) is enabled in RS-485 function mode.
     * |[9]     |LEV_RTS   |RTS Pin Active Level (Not Available In UART2 Channel)
     * |        |          |This bit defines the active level state of RTS pin output.
     * |        |          |0 = RTS pin output is high level active.
     * |        |          |1 = RTS pin output is low level active.
     * |[13]    |RTS_ST    |RTS Pin State (Read Only) (Not Available In UART2 Channel)
     * |        |          |This bit mirror from RTS pin output of voltage logic status.
     * |        |          |0 = RTS pin output is low level voltage logic state.
     * |        |          |1 = RTS pin output is high level voltage logic state.
     */
    __IO uint32_t MCR;

    /**
     * UA_MSR
     * ===================================================================================================
     * Offset: 0x14  UART Modem Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |DCTSF     |Detect CTS State Change Flag (Read Only) (Not Available In UART2 Channel)
     * |        |          |This bit is set whenever CTS input has change state, and it will generate Modem interrupt to CPU when MODEM_IEN (UA_IER [3]) is set to 1.
     * |        |          |0 = CTS input has not change state.
     * |        |          |1 = CTS input has change state.
     * |        |          |Note: This bit is read only, but can be cleared by writing "1" to it.
     * |[4]     |CTS_ST    |CTS Pin Status (Read Only) (Not Available In UART2 Channel)
     * |        |          |This bit mirror from CTS pin input of voltage logic status.
     * |        |          |0 = CTS pin input is low level voltage logic state.
     * |        |          |1 = CTS pin input is high level voltage logic state.
     * |        |          |Note: This bit echoes when UART Controller peripheral clock is enabled, and CTS multi-function port is selected
     * |[8]     |LEV_CTS   |CTS Pin Active Level
     * |        |          |This bit defines the active level state of CTS pin input.
     * |        |          |0 = CTS pin input is high level active.
     * |        |          |1 = CTS pin input is low level active.
     */
    __IO uint32_t MSR;

    /**
     * UA_FSR
     * ===================================================================================================
     * Offset: 0x18  UART FIFO Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |RX_OVER_IF|RX Overflow Error IF (Read Only)
     * |        |          |This bit is set when RX FIFO overflow.
     * |        |          |If the number of bytes of received data is greater than RX_FIFO (UA_RBR) size, 64/16/16 bytes of UART0/UART1/UART2, this bit will be set.
     * |        |          |0 = RX FIFO is not overflow.
     * |        |          |1 = RX FIFO is overflow.
     * |        |          |Note: This bit is read only, but can be cleared by writing "1" to it.
     * |[3]     |RS485_ADD_DETF|RS-485 Address Byte Detection Flag (Read Only)
     * |        |          |0 = Receiver detects a data that is not an address bit (bit 9 ='1').
     * |        |          |1 = Receiver detects a data that is an address bit (bit 9 ='1').
     * |        |          |Note1: This field is used for RS-485 function mode and RS485_ADD_EN (UA_ALT_CSR[15]) is set to 1 to enable Address detection mode.
     * |        |          |Note2: This bit is read only, but can be cleared by writing '1' to it.
     * |[4]     |PEF       |Parity Error Flag (Read Only)
     * |        |          |This bit is set to logic 1 whenever the received character does not have a valid "parity bit", and is reset whenever the CPU writes 1 to this bit.
     * |        |          |0 = No parity error is generated.
     * |        |          |1 = Parity error is generated.
     * |        |          |Note: This bit is read only, but can be cleared by writing "1" to it.
     * |[5]     |FEF       |Framing Error Flag (Read Only)
     * |        |          |This bit is set to logic 1 whenever the received character does not have a valid "stop bit" (that is, the stop bit following the last data bit or parity bit is detected as logic 0), and is reset whenever the CPU writes 1 to this bit.
     * |        |          |0 = No framing error is generated.
     * |        |          |1 = Framing error is generated.
     * |        |          |Note: This bit is read only, but can be cleared by writing "1" to it.
     * |[6]     |BIF       |Break Interrupt Flag (Read Only)
     * |        |          |This bit is set to logic 1 whenever the received data input(RX) is held in the "spacing state" (logic 0) for longer than a full word transmission time (that is, the total time of "start bit" + data bits + parity + stop bits) and is reset whenever the CPU writes 1 to this bit.
     * |        |          |0 = No Break interrupt is generated.
     * |        |          |1 = Break interrupt is generated.
     * |        |          |Note: This bit is read only, but can be cleared by writing "1" to it.
     * |[13:8]  |RX_POINTER|RX FIFO Pointer (Read Only)
     * |        |          |This field indicates the RX FIFO Buffer Pointer.
     * |        |          |When UART receives one byte from external device, then RX_POINTER increases one.
     * |        |          |When one byte of RX FIFO is read by CPU, then RX_POINTER decreases one.
     * |        |          |The Maximum value shown in RX_POINTER is 63/15/15 (UART0/UART1/UART2).
     * |        |          |When the using level of RX FIFO Buffer equal to 64/16/16, the RX_FULL bit is set to 1 and RX_POINTER will show 0.
     * |        |          |As one byte of RX FIFO is read by CPU, the RX_FULL bit is cleared to 0 and RX_POINTER will show 63/15/15 (UART0/UART1/UART2).
     * |[14]    |RX_EMPTY  |Receiver FIFO Empty (Read Only)
     * |        |          |This bit initiate RX FIFO empty or not.
     * |        |          |0 = RX FIFO is not empty.
     * |        |          |1 = RX FIFO is empty.
     * |        |          |Note: When the last byte of RX FIFO has been read by CPU, hardware sets this bit high.
     * |        |          |It will be cleared when UART receives any new data.
     * |[15]    |RX_FULL   |Receiver FIFO Full (Read Only)
     * |        |          |This bit initiates RX FIFO is full or not.
     * |        |          |0 = RX FIFO is not full.
     * |        |          |1 = RX FIFO is full.
     * |        |          |Note: This bit is set when the number of usage in RX FIFO Buffer is equal to 64/16/16(UART0/UART1/UART2), otherwise is cleared by hardware.
     * |[21:16] |TX_POINTER|TX FIFO Pointer (Read Only)
     * |        |          |This field indicates the TX FIFO Buffer Pointer.
     * |        |          |When CPU writes one byte into UA_THR, then TX_POINTER increases one.
     * |        |          |When one byte of TX FIFO is transferred to Transmitter Shift Register, then TX_POINTER decreases one.
     * |        |          |The Maximum value shown in TX_POINTER is 63/15/15 (UART0/UART1/UART2).
     * |        |          |When the using level of TX FIFO Buffer equal to 64/16/16, the TX_FULL bit is set to 1 and TX_POINTER will show 0.
     * |        |          |As one byte of TX FIFO is transferred to Transmitter Shift Register, the TX_FULL bit is cleared to 0 and TX_POINTER will show 63/15/15 (UART0/UART1/UART2).
     * |[22]    |TX_EMPTY  |Transmitter FIFO Empty (Read Only)
     * |        |          |This bit indicates TX FIFO empty or not.
     * |        |          |0 = TX FIFO is not empty.
     * |        |          |1 = TX FIFO is empty.
     * |        |          |Note: When the last byte of TX FIFO has been transferred to Transmitter Shift Register, hardware sets this bit high.
     * |        |          |It will be cleared when writing data into THR (TX FIFO not empty).
     * |[23]    |TX_FULL   |Transmitter FIFO Full (Read Only)
     * |        |          |This bit indicates TX FIFO full or not.
     * |        |          |0 = TX FIFO is not full.
     * |        |          |1 = TX FIFO is full.
     * |        |          |This bit is set when the number of usage in TX FIFO Buffer is equal to 64/16/16(UART0/UART1/UART2), otherwise is cleared by hardware.
     * |[24]    |TX_OVER_IF|TX Overflow Error Interrupt Flag (Read Only)
     * |        |          |If TX FIFO (UA_THR) is full, an additional write to UA_THR will cause this bit to
     * |        |          |logic 1.
     * |        |          |0 = TX FIFO is not overflow.
     * |        |          |1 = TX FIFO is overflow.
     * |        |          |Note: This bit is read only, but can be cleared by writing "1" to it.
     * |[28]    |TE_FLAG   |Transmitter Empty Flag (Read Only)
     * |        |          |This bit is set by hardware when TX FIFO (UA_THR) is empty and the STOP bit of the last byte has been transmitted.
     * |        |          |0 = TX FIFO is not empty.
     * |        |          |1 = TX FIFO is empty.
     * |        |          |Note: This bit is cleared automatically when TX FIFO is not empty or the last byte transmission has not completed.
     */
    __IO uint32_t FSR;

    /**
     * UA_ISR
     * ===================================================================================================
     * Offset: 0x1C  UART Interrupt Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |RDA_IF    |Receive Data Available Interrupt Flag (Read Only)
     * |        |          |When the number of bytes in the RX FIFO equals the RFITL then the RDA_IF(UA_ISR[0]) will be set.
     * |        |          |If RDA_IEN (UA_IER [0]) is enabled, the RDA interrupt will be generated.
     * |        |          |0 = No RDA interrupt flag is generated.
     * |        |          |1 = RDA interrupt flag is generated.
     * |        |          |Note: This bit is read only and it will be cleared when the number of unread bytes of RX FIFO drops below the threshold level (RFITL(UA_FCR[7:4]).
     * |[1]     |THRE_IF   |Transmit Holding Register Empty Interrupt Flag (Read Only)
     * |        |          |This bit is set when the last data of TX FIFO is transferred to Transmitter Shift Register.
     * |        |          |If THRE_IEN (UA_IER[1]) is enabled, the THRE interrupt will be generated.
     * |        |          |0 = No THRE interrupt flag is generated.
     * |        |          |1 = THRE interrupt flag is generated.
     * |        |          |Note: This bit is read only and it will be cleared when writing data into THR (TX FIFO not empty).
     * |[2]     |RLS_IF    |Receive Line Interrupt Flag (Read Only)
     * |        |          |This bit is set when the RX receive data have parity error, frame error or break error (at least one of 3 bits, BIF(UA_FSR[6]), FEF(UA_FSR[5]) and PEF(UA_FSR[4]), is set).
     * |        |          |If RLS_IEN (UA_IER [2]) is enabled, the RLS interrupt will be generated.
     * |        |          |0 = No RLS interrupt flag is generated.
     * |        |          |1 = RLS interrupt flag is generated.
     * |        |          |Note1: In RS-485 function mode, this field is set include receiver detect and received address byte character (bit9 = '1') bit.
     * |        |          |At the same time, the bit of UA_FSR[RS485_ADD_DETF] is also set.
     * |        |          |Note2: This bit is read only and reset to 0 when all bits of BIF(UA_FSR[6]), FEF(UA_FSR[5]) and PEF(UA_FSR[4]) are cleared.
     * |        |          |Note3: In RS-485 function mode, this bit is read only and reset to 0 when all bits of BIF(UA_FSR[6]) , FEF(UA_FSR[5]) and PEF(UA_FSR[4]) and RS485_ADD_DETF (UA_FSR[3]) are cleared.
     * |[3]     |MODEM_IF  |MODEM Interrupt Flag (Read Only) (Not Available In UART2 Channel)
     * |        |          |This bit is set when the CTS pin has state change (DCTSF (UA_MSR[0]) = 1).
     * |        |          |If MODEM_IEN (UA_IER [3]) is enabled, the Modem interrupt will be generated.
     * |        |          |0 = No Modem interrupt flag is generated.
     * |        |          |1 = Modem interrupt flag is generated.
     * |        |          |Note: This bit is read only and reset to 0 when bit DCTSF is cleared by a write 1 on DCTSF(UA_MSR[0]).
     * |[4]     |TOUT_IF   |Time-Out Interrupt Flag (Read Only)
     * |        |          |This bit is set when the RX FIFO is not empty and no activities occurred in the RX FIFO and the time-out counter equal to TOIC.
     * |        |          |If TOUT_IEN (UA_IER [4]) is enabled, the Tout interrupt will be generated.
     * |        |          |0 = No Time-out interrupt flag is generated.
     * |        |          |1 = Time-out interrupt flag is generated.
     * |        |          |Note: This bit is read only and user can read UA_RBR (RX is in active) to clear it
     * |[5]     |BUF_ERR_IF|Buffer Error Interrupt Flag (Read Only)
     * |        |          |This bit is set when the TX FIFO or RX FIFO overflows (TX_OVER_IF (UA_FSR[24]) or RX_OVER_IF (UA_FSR[0]) is set).
     * |        |          |When BUF_ERR_IF (UA_ISR[5])is set, the transfer is not correct.
     * |        |          |If BUF_ERR_IEN (UA_IER [8]) is enabled, the buffer error interrupt will be generated.
     * |        |          |0 = No buffer error interrupt flag is generated.
     * |        |          |1 = Buffer error interrupt flag is generated.0 = No buffer error interrupt flag is generated.
     * |        |          |1 = Buffer error interrupt flag is generated.
     * |        |          |Note: This bit is read only and reset to 0 when all bits of TX_OVER_IF(UA_FSR[24]) and RX_OVER_IF(UA_FSR[0]) are cleared
     * |[8]     |RDA_INT   |Receive Data Available Interrupt Indicator (Read Only)
     * |        |          |This bit is set if RDA_IEN (UA_IER[0]) and RDA_IF (UA_ISR[0]) are both set to 1.
     * |        |          |0 = No RDA interrupt is generated.
     * |        |          |1 = RDA interrupt is generated.
     * |[9]     |THRE_INT  |Transmit Holding Register Empty Interrupt Indicator (Read Only)
     * |        |          |This bit is set if THRE_IEN (UA_IER[1])and THRE_IF(UA_SR[1]) are both set to 1.
     * |        |          |0 = No THRE interrupt is generated.
     * |        |          |1 = THRE interrupt is generated.
     * |[10]    |RLS_INT   |Receive Line Status Interrupt Indicator (Read Only)
     * |        |          |This bit is set if RLS_IEN (UA_IER[2]) and RLS_IF(UA_ISR[2]) are both set to 1.
     * |        |          |0 = No RLS interrupt is generated.
     * |        |          |1 = RLS interrupt is generated
     * |[11]    |MODEM_INT |MODEM Status Interrupt Indicator (Read Only) (Not Available In UART2 Channel)
     * |        |          |This bit is set if MODEM_IEN(UA_IER[3] and MODEM_IF(UA_ISR[4]) are both set to 1
     * |        |          |0 = No Modem interrupt is generated.
     * |        |          |1 = Modem interrupt is generated.
     * |[12]    |TOUT_INT  |Time-Out Interrupt Indicator (Read Only)
     * |        |          |This bit is set if TOUT_IEN(UA_IER[4]) and TOUT_IF(UA_ISR[4]) are both set to 1.
     * |        |          |0 = No Tout interrupt is generated.
     * |        |          |1 = Tout interrupt is generated.
     * |[13]    |BUF_ERR_INT|Buffer Error Interrupt Indicator (Read Only)
     * |        |          |This bit is set if BUF_ERR_IEN(UA_IER[5] and BUF_ERR_IF(UA_ISR[5]) are both set to 1.
     * |        |          |0 = No buffer error interrupt is generated.
     * |        |          |1 = Buffer error interrupt is generated.
     * |[18]    |HW_RLS_IF |In DMA Mode, Receive Line Status Flag (Read Only)
     * |        |          |This bit is set when the RX receive data have parity error, frame error or break error (at least one of 3 bits, BIF (UA_FSR[6]), FEF (UA_FSR[5]) and PEF (UA_FSR[4]) is set).
     * |        |          |If RLS_IEN (UA_IER [2]) is enabled, the RLS interrupt will be generated.
     * |        |          |0 = No RLS interrupt flag is generated.
     * |        |          |1 = RLS interrupt flag is generated.
     * |        |          |Note1: In RS-485 function mode, this field include receiver detect any address byte received address byte character (bit9 = '1') bit.
     * |        |          |Note2: In UART function mode, this bit is read only and reset to 0 when all bits of BIF(UA_FSR[6]) , FEF(UA_FSR[5]) and PEF(UA_FSR[4]) are cleared.
     * |        |          |Note3: In RS-485 function mode, this bit is read only and reset to 0 when all bits of BIF(UA_FSR[6]) , FEF(UA_FSR[5]) and PEF(UA_FSR[4]) and RS485_ADD_DETF (UA_FSR[3]) are cleared.
     * |[19]    |HW_MODEM_IF|In DMA Mode, MODEM Interrupt Flag (Read Only) (Not Available In UART2 Channel)
     * |        |          |This bit is set when the CTS pin has state change (DCTSF (US_MSR[0] =1)).
     * |        |          |If MODEM_IEN (UA_IER [3]) is enabled, the Modem interrupt will be generated.
     * |        |          |0 = No Modem interrupt flag is generated.
     * |        |          |1 = Modem interrupt flag is generated.
     * |        |          |Note: This bit is read only and reset to 0 when the bit DCTSF(US_MSR[0]) is cleared by writing 1 on DCTSF (US_MSR[0]).
     * |[20]    |HW_TOUT_IF|In DMA Mode, Time-Out Interrupt Flag (Read Only)
     * |        |          |This bit is set when the RX FIFO is not empty and no activities occurred in the RX FIFO and the time-out counter equal to TOIC (UA_TOR[7:0]).
     * |        |          |If TOUT_IEN (UA_IER [4]) is enabled, the Tout interrupt will be generated.
     * |        |          |0 = No Time-out interrupt flag is generated.
     * |        |          |1 = Time-out interrupt flag is generated.
     * |        |          |Note: This bit is read only and user can read UA_RBR (RX is in active) to clear it.
     * |[21]    |HW_BUF_ERR_IF|In DMA Mode, Buffer Error Interrupt Flag (Read Only)
     * |        |          |This bit is set when the TX or RX FIFO overflows (TX_OVER_IF (UA__FSR[24]) or RX_OVER_IF (UA_FSR[0]) is set).
     * |        |          |When BUF_ERR_IF (UA_ISR[5]) is set, the transfer maybe is not correct.
     * |        |          |If BUF_ERR_IEN (UA_IER [5]) is enabled, the buffer error interrupt will be generated.
     * |        |          |0 = No buffer error interrupt flag is generated.
     * |        |          |1 = Buffer error interrupt flag is generated.
     * |        |          |Note: This bit is cleared when both TX_OVER_IF (UA_FSR[24]]) and RX_OVER_IF (UA_FSR[0]) are cleared.
     * |[26]    |HW_RLS_INT|In DMA Mode, Receive Line Status Interrupt Indicator (Read Only)
     * |        |          |This bit is set if RLS_IEN (UA_IER[2])and HW_RLS_IF(UA_ISR[18]) are both set to 1.
     * |        |          |0 = No RLS interrupt is generated in DMA mode.
     * |        |          |1 = RLS interrupt is generated in DMA mode.
     * |[27]    |HW_MODEM_INT|In DMA Mode, MODEM Status Interrupt Indicator (Read Only) (Not Available In UART2 Channel)
     * |        |          |This bit is set if MODEM_IEN(UA_IER[3]) and HW_MODEM_IF(UA_ ISR[3]) are both set to 1.
     * |        |          |0 = No Modem interrupt is generated in DMA mode.
     * |        |          |1 = Modem interrupt is generated in DMA mode.
     * |[28]    |HW_TOUT_INT|In DMA Mode, Time-Out Interrupt Indicator (Read Only)
     * |        |          |This bit is set if TOUT_IEN (UA_IER[4])and HW_TOUT_IF(UA_ISR[20]) are both set to 1.
     * |        |          |0 = No Tout interrupt is generated in DMA mode.
     * |        |          |1 = Tout interrupt is generated in DMA mode.
     * |[29]    |HW_BUF_ERR_INT|In DMA Mode, Buffer Error Interrupt Indicator (Read Only)
     * |        |          |This bit is set if BUF_ERR_IEN (UA_IER[5]) and HW_BUF_ERR_IF (UA_ISR[5])are both set to 1.
     * |        |          |0 = No buffer error interrupt is generated in DMA mode.
     * |        |          |1 = Buffer error interrupt is generated in DMA mode.
     */
    __IO uint32_t ISR;

    /**
     * UA_TOR
     * ===================================================================================================
     * Offset: 0x20  UART Time-out Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |TOIC      |Time-Out Interrupt Comparator
     * |        |          |The time-out counter resets and starts counting (the counting clock = baud rate) whenever the RX FIFO receives a new data word.
     * |        |          |Once the content of time-out counter is equal to that of time-out interrupt comparator (TOIC (UA_TOR[7:0])), a receiver time-out interrupt (INT_TOUT) is generated if TOUT_IEN (UA_IER [4]) enabled.
     * |        |          |A new incoming data word or RX FIFO empty will clear TOUT_INT(UA_IER[9]).
     * |        |          |In order to avoid receiver time-out interrupt generation immediately during one character is being received, TOIC (UA_TOR[7:0]) value should be set between 40 and 255.
     * |        |          |So, for example, if TOIC (UA_TOR[7:0]) is set with 40, the time-out interrupt is generated after four characters are not received when 1 stop bit and no parity check is set for UART transfer.
     * |[15:8]  |DLY       |TX Delay Time Value
     * |        |          |This field is used to programming the transfer delay time between the last stop bit and next start bit.
     */
    __IO uint32_t TOR;

    /**
     * UA_BAUD
     * ===================================================================================================
     * Offset: 0x24  UART Baud Rate Divisor Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |BRD       |Baud Rate Divider
     * |        |          |The field indicates the baud rate divider
     * |[27:24] |DIVIDER_X |Divider X
     * |        |          |The baud rate divider M = X+1.
     * |[28]    |DIV_X_ONE |Divider X Equal To 1
     * |        |          |0 = Divider M = X
     * |        |          |1 = Divider M = 1
     * |[29]    |DIV_X_EN  |Divider X Enable
     * |        |          |The BRD = Baud Rate Divider, and the baud rate equation is
     * |        |          |Baud Rate = Clock / [M * (BRD + 2)]; The default value of M is 16.
     * |        |          |0 = Divider X Disabled (the equation of M = 16).
     * |        |          |1 = Divider X Enabled (the equation of M = X+1, but DIVIDER_X [27:24] must >= 8).
     * |        |          |Refer to Table 5-21 for more information.
     * |        |          |Note: In IrDA mode, this bit must disable.
     */
    __IO uint32_t BAUD;

    /**
     * UA_IRCR
     * ===================================================================================================
     * Offset: 0x28  UART IrDA Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1]     |TX_SELECT |TX_SELECT
     * |        |          |0 = IrDA Transmitter Disabled and Receiver Enabled.
     * |        |          |1 = IrDA Transmitter Enabled and Receiver Disabled.
     * |[5]     |INV_TX    |IrDA inverse Transmitting Output Signal Control
     * |        |          |0 = None inverse transmitting signal.
     * |        |          |1 = Inverse transmitting output signal.
     * |[6]     |INV_RX    |IrDA inverse Receive Input Signal Control
     * |        |          |0 = None inverse receiving input signal.
     * |        |          |1 = Inverse receiving input signal.
     */
    __IO uint32_t IRCR;

    /**
     * UA_ALT_CSR
     * ===================================================================================================
     * Offset: 0x2C  UART Alternate Control/Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[8]     |RS485_NMM |RS-485 Normal Multi-Drop Operation Mode (NMM)
     * |        |          |0 = RS-485 Normal Multi-drop Operation mode (NMM) Disabled.
     * |        |          |1 = RS-485 Normal Multi-drop Operation mode (NMM) Enabled.
     * |        |          |Note: It cannot be active with RS-485_AAD operation mode.
     * |[9]     |RS485_AAD |RS-485 Auto Address Detection Operation Mode (AAD)
     * |        |          |0 = RS-485 Auto Address Detection Operation mode (AAD) Disabled.
     * |        |          |1 = RS-485 Auto Address Detection Operation mode (AAD) Enabled.
     * |        |          |Note: It cannot be active with RS-485_NMM operation mode.
     * |[10]    |RS485_AUD |RS-485 Auto Direction Mode (AUD)
     * |        |          |0 = RS-485 Auto Direction Operation mode (AUO) Disabled.
     * |        |          |1 = RS-485 Auto Direction Operation mode (AUO) Enabled.
     * |        |          |Note: It can be active with RS-485_AAD or RS-485_NMM operation mode.
     * |[15]    |RS485_ADD_EN|RS-485 Address Detection Enable
     * |        |          |This bit is used to enable RS-485 Address Detection mode.
     * |        |          |0 = Address detection mode Disabled.
     * |        |          |1 = Address detection mode Enabled.
     * |        |          |Note: This bit is used for RS-485 any operation mode.
     * |[31:24] |ADDR_MATCH|Address Match Value Register
     * |        |          |This field contains the RS-485 address match values.
     * |        |          |Note: This field is used for RS-485 auto address detection mode.
     */
    __IO uint32_t ALT_CSR;

    /**
     * UA_FUN_SEL
     * ===================================================================================================
     * Offset: 0x30  UART Function Select Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1:0]   |FUN_SEL   |Function Select Enable
     * |        |          |00 = UART function Enabled.
     * |        |          |10 = IrDA function Enabled.
     * |        |          |11 = RS-485 function Enabled.
     */
    __IO uint32_t FUN_SEL;

} UART_T;


/** @addtogroup REG_UART_BITMASK UART Bit Mask
  @{
 */

/* UART THR Bit Field Definitions */
#define UART_THR_THR_Pos         0                                          /*!< UART THR: THR Position  */
#define UART_THR_THR_Msk        (0xFul << UART_THR_THR_Pos)                 /*!< UART THR: THR Mask      */

/* UART RBR Bit Field Definitions */
#define UART_RBR_RBR_Pos         0                                          /*!< UART RBR: RBR Position */
#define UART_RBR_RBR_Msk        (0xFul << UART_RBR_RBR_Pos)                 /*!< UART RBR: RBR Mask      */

/* UART IER Bit Field Definitions */
#define UART_IER_DMA_RX_EN_Pos      15                                      /*!< UART IER: RX DMA Enable Position */
#define UART_IER_DMA_RX_EN_Msk      (1ul << UART_IER_DMA_RX_EN_Pos)         /*!< UART IER: RX DMA Enable Mask      */

#define UART_IER_DMA_TX_EN_Pos      14                                      /*!< UART IER: TX DMA Enable Position */
#define UART_IER_DMA_TX_EN_Msk      (1ul << UART_IER_DMA_TX_EN_Pos)         /*!< UART IER: TX DMA Enable Mask      */

#define UART_IER_AUTO_CTS_EN_Pos    13                                      /*!< UART IER: AUTO_CTS_EN Position      */
#define UART_IER_AUTO_CTS_EN_Msk    (1ul << UART_IER_AUTO_CTS_EN_Pos)       /*!< UART IER: AUTO_CTS_EN Mask           */

#define UART_IER_AUTO_RTS_EN_Pos    12                                      /*!< UART IER: AUTO_RTS_EN Position      */
#define UART_IER_AUTO_RTS_EN_Msk    (1ul << UART_IER_AUTO_RTS_EN_Pos)       /*!< UART IER: AUTO_RTS_EN Mask           */

#define UART_IER_TIME_OUT_EN_Pos    11                                      /*!< UART IER: TIME_OUT_EN Position      */
#define UART_IER_TIME_OUT_EN_Msk    (1ul << UART_IER_TIME_OUT_EN_Pos)       /*!< UART IER: TIME_OUT_EN Mask           */

#define UART_IER_WAKE_EN_Pos        6                                       /*!< UART IER: WAKE_EN Position          */
#define UART_IER_WAKE_EN_Msk        (1ul << UART_IER_WAKE_EN_Pos)           /*!< UART IER: WAKE_EN Mask               */

#define UART_IER_BUF_ERR_IEN_Pos    5                                       /*!< UART IER: BUF_ERR_IEN Position      */
#define UART_IER_BUF_ERR_IEN_Msk    (1ul << UART_IER_BUF_ERR_IEN_Pos)       /*!< UART IER: BUF_ERR_IEN Mask           */

#define UART_IER_RTO_IEN_Pos        4                                       /*!< UART IER: RTO_IEN Position          */
#define UART_IER_RTO_IEN_Msk        (1ul << UART_IER_RTO_IEN_Pos)           /*!< UART IER: RTO_IEN Mask               */

#define UART_IER_MODEM_IEN_Pos      3                                       /*!< UART IER: MODEM_IEN Position        */
#define UART_IER_MODEM_IEN_Msk      (1ul << UART_IER_MODEM_IEN_Pos)         /*!< UART IER: MODEM_IEN Mask             */

#define UART_IER_RLS_IEN_Pos        2                                       /*!< UART IER: RLS_IEN Position          */
#define UART_IER_RLS_IEN_Msk        (1ul << UART_IER_RLS_IEN_Pos)           /*!< UART IER: RLS_IEN Mask               */

#define UART_IER_THRE_IEN_Pos       1                                       /*!< UART IER: THRE_IEN Position         */
#define UART_IER_THRE_IEN_Msk       (1ul << UART_IER_THRE_IEN_Pos)          /*!< UART IER: THRE_IEN Mask              */

#define UART_IER_RDA_IEN_Pos        0                                       /*!< UART IER: RDA_IEN Position           */
#define UART_IER_RDA_IEN_Msk        (1ul << UART_IER_RDA_IEN_Pos)           /*!< UART IER: RDA_IEN Mask               */

/* UART FCR Bit Field Definitions */
#define UART_FCR_RTS_TRI_LEV_Pos    16                                      /*!< UART FCR: RTS_TRI_LEV Position       */
#define UART_FCR_RTS_TRI_LEV_Msk    (0xFul << UART_FCR_RTS_TRI_LEV_Pos)     /*!< UART FCR: RTS_TRI_LEV Mask           */

#define UART_FCR_RX_DIS_Pos         8                                       /*!< UART FCR: RX_DIS Position            */
#define UART_FCR_RX_DIS_Msk         (1ul << UART_FCR_RX_DIS_Pos)            /*!< UART FCR: RX_DIS Mask                */

#define UART_FCR_RFITL_Pos          4                                       /*!< UART FCR: RFITL Position             */
#define UART_FCR_RFITL_Msk          (0xFul << UART_FCR_RFITL_Pos)           /*!< UART FCR: RFITL Mask                 */

#define UART_FCR_TFR_Pos            2                                       /*!< UART FCR: TFR Position               */
#define UART_FCR_TFR_Msk            (1ul << UART_FCR_TFR_Pos)               /*!< UART FCR: TFR Mask                   */

#define UART_FCR_RFR_Pos            1                                       /*!< UART FCR: RFR Position               */
#define UART_FCR_RFR_Msk            (1ul << UART_FCR_RFR_Pos)               /*!< UART FCR: RFR Mask                   */

/* UART LCR Bit Field Definitions */
#define UART_LCR_BCB_Pos            6                                       /*!< UART LCR: BCB Position               */
#define UART_LCR_BCB_Msk            (1ul << UART_LCR_BCB_Pos)               /*!< UART LCR: BCB Mask                   */

#define UART_LCR_SPE_Pos            5                                       /*!< UART LCR: SPE Position               */
#define UART_LCR_SPE_Msk            (1ul << UART_LCR_SPE_Pos)               /*!< UART LCR: SPE Mask                   */

#define UART_LCR_EPE_Pos            4                                       /*!< UART LCR: EPE Position               */
#define UART_LCR_EPE_Msk            (1ul << UART_LCR_EPE_Pos)               /*!< UART LCR: EPE Mask                   */

#define UART_LCR_PBE_Pos            3                                       /*!< UART LCR: PBE Position               */
#define UART_LCR_PBE_Msk            (1ul << UART_LCR_PBE_Pos)               /*!< UART LCR: PBE Mask                   */

#define UART_LCR_NSB_Pos            2                                       /*!< UART LCR: NSB Position               */
#define UART_LCR_NSB_Msk            (1ul << UART_LCR_NSB_Pos)               /*!< UART LCR: NSB Mask                   */

#define UART_LCR_WLS_Pos            0                                       /*!< UART LCR: WLS Position               */
#define UART_LCR_WLS_Msk            (0x3ul << UART_LCR_WLS_Pos)             /*!< UART LCR: WLS Mask                   */

/* UART MCR Bit Field Definitions */
#define UART_MCR_RTS_ST_Pos         13                                      /*!< UART MCR: RTS_ST Position            */
#define UART_MCR_RTS_ST_Msk         (1ul << UART_MCR_RTS_ST_Pos)            /*!< UART MCR: RTS_ST Mask                */

#define UART_MCR_LEV_RTS_Pos        9                                       /*!< UART MCR: LEV_RTS Position           */
#define UART_MCR_LEV_RTS_Msk        (1ul << UART_MCR_LEV_RTS_Pos)           /*!< UART MCR: LEV_RTS Mask               */

#define UART_MCR_RTS_Pos            1                                       /*!< UART MCR: RTS Position               */
#define UART_MCR_RTS_Msk            (1ul << UART_MCR_RTS_Pos)               /*!< UART MCR: RTS Mask                   */

/* UART MSR Bit Field Definitions */
#define UART_MSR_LEV_CTS_Pos        8                                       /*!< UART MSR: LEV_CTS Position           */
#define UART_MSR_LEV_CTS_Msk        (1ul << UART_MSR_LEV_CTS_Pos)           /*!< UART MSR: LEV_CTS Mask               */

#define UART_MSR_CTS_ST_Pos         4                                       /*!< UART MSR: CTS_ST Position            */
#define UART_MSR_CTS_ST_Msk         (1ul << UART_MSR_CTS_ST_Pos)            /*!< UART MSR: CTS_ST Mask                */

#define UART_MSR_DCTSF_Pos          0                                       /*!< UART MSR: DCTST Position             */
#define UART_MSR_DCTSF_Msk          (1ul << UART_MSR_DCTSF_Pos)             /*!< UART MSR: DCTST Mask                 */


/* UART FSR Bit Field Definitions */
#define UART_FSR_TE_FLAG_Pos        28                                      /*!< UART FSR: TE_FLAG Position           */
#define UART_FSR_TE_FLAG_Msk        (1ul << UART_FSR_TE_FLAG_Pos)           /*!< UART FSR: TE_FLAG Mask               */

#define UART_FSR_TX_OVER_IF_Pos     24                                      /*!< UART FSR: TX_OVER_IF Position        */
#define UART_FSR_TX_OVER_IF_Msk     (1ul << UART_FSR_TX_OVER_IF_Pos)        /*!< UART FSR: TX_OVER_IF Mask            */

#define UART_FSR_TX_FULL_Pos        23                                      /*!< UART FSR: TX_FULL Position           */
#define UART_FSR_TX_FULL_Msk        (1ul << UART_FSR_TX_FULL_Pos)           /*!< UART FSR: TX_FULL Mask               */

#define UART_FSR_TX_EMPTY_Pos       22                                      /*!< UART FSR: TX_EMPTY Position          */
#define UART_FSR_TX_EMPTY_Msk       (1ul << UART_FSR_TX_EMPTY_Pos)          /*!< UART FSR: TX_EMPTY Mask              */

#define UART_FSR_TX_POINTER_Pos     16                                      /*!< UART FSR: TX_POINTER Position        */
#define UART_FSR_TX_POINTER_Msk     (0x3Ful << UART_FSR_TX_POINTER_Pos)     /*!< UART FSR: TX_POINTER Mask            */

#define UART_FSR_RX_FULL_Pos        15                                      /*!< UART FSR: RX_FULL Position           */
#define UART_FSR_RX_FULL_Msk        (1ul << UART_FSR_RX_FULL_Pos)           /*!< UART FSR: RX_FULL Mask               */

#define UART_FSR_RX_EMPTY_Pos       14                                      /*!< UART FSR: RX_EMPTY Position          */
#define UART_FSR_RX_EMPTY_Msk       (1ul << UART_FSR_RX_EMPTY_Pos)          /*!< UART FSR: RX_EMPTY Mask              */

#define UART_FSR_RX_POINTER_Pos     8                                       /*!< UART FSR: RX_POINTERS Position       */
#define UART_FSR_RX_POINTER_Msk     (0x3Ful << UART_FSR_RX_POINTER_Pos)     /*!< UART FSR: RX_POINTER Mask            */

#define UART_FSR_BIF_Pos            6                                       /*!< UART FSR: BIF Position               */
#define UART_FSR_BIF_Msk            (1ul << UART_FSR_BIF_Pos)               /*!< UART FSR: BIF Mask                   */

#define UART_FSR_FEF_Pos            5                                       /*!< UART FSR: FEF Position               */
#define UART_FSR_FEF_Msk            (1ul << UART_FSR_FEF_Pos)               /*!< UART FSR: FEF Mask                   */

#define UART_FSR_PEF_Pos            4                                       /*!< UART FSR: PEF Position               */
#define UART_FSR_PEF_Msk            (1ul << UART_FSR_PEF_Pos)               /*!< UART FSR: PEF Mask                   */

#define UART_FSR_RS485_ADD_DETF_Pos 3                                       /*!< UART FSR: RS485_ADD_DETF Position    */
#define UART_FSR_RS485_ADD_DETF_Msk (1ul << UART_FSR_RS485_ADD_DETF_Pos)    /*!< UART FSR: RS485_ADD_DETF Mask        */

#define UART_FSR_RX_OVER_IF_Pos     0                                       /*!< UART FSR: RX_OVER_IF Position        */
#define UART_FSR_RX_OVER_IF_Msk     (1ul << UART_FSR_RX_OVER_IF_Pos)        /*!< UART FSR: RX_OVER_IF Mask            */

/* UART ISR Bit Field Definitions */
#define UART_ISR_HW_BUF_ERR_INT_Pos 29                                      /*!< UART ISR: HW BUF_ERR_INT Position    */
#define UART_ISR_HW_BUF_ERR_INT_Msk (1ul << UART_ISR_HW_BUF_ERR_INT_Pos)    /*!< UART ISR: HW BUF_ERR_INT Mask        */

#define UART_ISR_HW_TOUT_INT_Pos    28                                      /*!< UART ISR: HW TOUT_INT Position       */
#define UART_ISR_HW_TOUT_INT_Msk    (1ul << UART_ISR_HW_TOUT_INT_Pos)       /*!< UART ISR: HW TOUT_INT Mask           */

#define UART_ISR_HW_MODEM_INT_Pos   27                                      /*!< UART ISR: HW MODEM_INT Position      */
#define UART_ISR_HW_MODEM_INT_Msk   (1ul << UART_ISR_HW_MODEM_INT_Pos)      /*!< UART ISR: HW MODEM_INT Mask          */

#define UART_ISR_HW_RLS_INT_Pos     26                                      /*!< UART ISR: HW RLS_INT Position        */
#define UART_ISR_HW_RLS_INT_Msk     (1ul << UART_ISR_HW_RLS_INT_Pos)        /*!< UART ISR: HW RLS_INT Position        */

#define UART_ISR_HW_BUF_ERR_IF_Pos  21                                      /*!< UART ISR: HW BUF_ERR_IF Position     */
#define UART_ISR_HW_BUF_ERR_IF_Msk  (1ul << UART_ISR_HW_BUF_ERR_IF_Pos)     /*!< UART ISR: HW BUF_ERR_IF Mask         */

#define UART_ISR_HW_TOUT_IF_Pos     20                                      /*!< UART ISR: HW TOUT_IF Position        */
#define UART_ISR_HW_TOUT_IF_Msk     (1ul << UART_ISR_HW_TOUT_IFF_Pos)       /*!< UART ISR: HW TOUT_IF Mask            */

#define UART_ISR_HW_MODEM_IF_Pos    19                                      /*!< UART ISR: HW MODEM_IF Position       */
#define UART_ISR_HW_MODEM_IF_Msk    (1ul << UART_ISR_HW_MODEM_IF_Pos)       /*!< UART ISR: HW MODEM_IF Mask           */

#define UART_ISR_HW_RLS_IF_Pos      18                                      /*!< UART ISR: HW RLS_IF Position         */
#define UART_ISR_HW_RLS_IF_Msk      (1ul << UART_ISR_HW_RLS_IF_Pos)         /*!< UART ISR: HW RLS_IF Mark             */

#define UART_ISR_BUF_ERR_INT_Pos    13                                      /*!< UART ISR: BUF_ERR_INT Position       */
#define UART_ISR_BUF_ERR_INT_Msk    (1ul << UART_ISR_BUF_ERR_INT_Pos)       /*!< UART ISR: BUF_ERR_INT Mask           */

#define UART_ISR_TOUT_INT_Pos       12                                      /*!< UART ISR: TOUT_INT Position          */
#define UART_ISR_TOUT_INT_Msk       (1ul << UART_ISR_TOUT_INT_Pos)          /*!< UART ISR: TOUT_INT Mask              */

#define UART_ISR_MODEM_INT_Pos      11                                      /*!< UART ISR: MODEM_INT Position         */
#define UART_ISR_MODEM_INT_Msk      (1ul << UART_ISR_MODEM_INT_Pos)         /*!< UART ISR: MODEM_INT Mask             */

#define UART_ISR_RLS_INT_Pos        10                                      /*!< UART ISR: RLS_INT Position           */
#define UART_ISR_RLS_INT_Msk        (1ul << UART_ISR_RLS_INT_Pos)           /*!< UART ISR: RLS_INT Mask               */

#define UART_ISR_THRE_INT_Pos       9                                       /*!< UART ISR: THRE_INT Position          */
#define UART_ISR_THRE_INT_Msk       (1ul << UART_ISR_THRE_INT_Pos)          /*!< UART ISR: THRE_INT Mask              */

#define UART_ISR_RDA_INT_Pos        8                                       /*!< UART ISR: RDA_INT Position           */
#define UART_ISR_RDA_INT_Msk        (1ul << UART_ISR_RDA_INT_Pos)           /*!< UART ISR: RDA_INT Mask               */

#define UART_ISR_BUF_ERR_IF_Pos     5                                       /*!< UART ISR: BUF_ERR_IF Position        */
#define UART_ISR_BUF_ERR_IF_Msk     (1ul << UART_ISR_BUF_ERR_IF_Pos)        /*!< UART ISR: BUF_ERR_IF Mask            */

#define UART_ISR_TOUT_IF_Pos        4                                       /*!< UART ISR: TOUT_IF Position           */
#define UART_ISR_TOUT_IF_Msk        (1ul << UART_ISR_TOUT_IF_Pos)           /*!< UART ISR: TOUT_IF Mask               */

#define UART_ISR_MODEM_IF_Pos       3                                       /*!< UART ISR: MODEM_IF Position          */
#define UART_ISR_MODEM_IF_Msk       (1ul << UART_ISR_MODEM_IF_Pos)          /*!< UART ISR: MODEM_IF Mask              */

#define UART_ISR_RLS_IF_Pos         2                                       /*!< UART ISR: RLS_IF Position            */
#define UART_ISR_RLS_IF_Msk         (1ul << UART_ISR_RLS_IF_Pos)            /*!< UART ISR: RLS_IF Mask                */

#define UART_ISR_THRE_IF_Pos        1                                       /*!< UART ISR: THRE_IF Position           */
#define UART_ISR_THRE_IF_Msk        (1ul << UART_ISR_THRE_IF_Pos)           /*!< UART ISR: THRE_IF Mask               */

#define UART_ISR_RDA_IF_Pos         0                                       /*!< UART ISR: RDA_IF Position            */
#define UART_ISR_RDA_IF_Msk         (1ul << UART_ISR_RDA_IF_Pos)            /*!< UART ISR: RDA_IF Mask                */


/* UART TOR Bit Field Definitions */
#define UART_TOR_DLY_Pos           8                                        /*!< UART TOR: DLY Position               */
#define UART_TOR_DLY_Msk           (0xFFul << UART_TOR_DLY_Pos)             /*!< UART TOR: DLY Mask                   */

#define UART_TOR_TOIC_Pos          0                                        /*!< UART TOR: TOIC Position              */
#define UART_TOR_TOIC_Msk          (0xFFul << UART_TOR_TOIC_Pos)

/* UART BAUD Bit Field Definitions */
#define UART_BAUD_DIV_X_EN_Pos    29                                        /*!< UART BARD: DIV_X_EN Position         */
#define UART_BAUD_DIV_X_EN_Msk    (1ul << UART_BAUD_DIV_X_EN_Pos)           /*!< UART BARD: DIV_X_EN Mask             */

#define UART_BAUD_DIV_X_ONE_Pos   28                                        /*!< UART BARD: DIV_X_ONE Position        */
#define UART_BAUD_DIV_X_ONE_Msk   (1ul << UART_BAUD_DIV_X_ONE_Pos)          /*!< UART BARD: DIV_X_ONE Mask            */

#define UART_BAUD_DIVIDER_X_Pos   24                                        /*!< UART BARD: DIVIDER_X Position        */
#define UART_BAUD_DIVIDER_X_Msk   (0xFul << UART_BAUD_DIVIDER_X_Pos)        /*!< UART BARD: DIVIDER_X Mask            */

#define UART_BAUD_BRD_Pos         0                                         /*!< UART BARD: BRD Position              */
#define UART_BAUD_BRD_Msk         (0xFFFFul << UART_BAUD_BRD_Pos)           /*!< UART BARD: BRD Mask                  */

/* UART IRCR Bit Field Definitions */
#define UART_IRCR_INV_RX_Pos      6                                         /*!< UART IRCR: INV_RX Position           */
#define UART_IRCR_INV_RX_Msk     (1ul << UART_IRCR_INV_RX_Pos)              /*!< UART IRCR: INV_RX Mask               */

#define UART_IRCR_INV_TX_Pos      5                                         /*!< UART IRCR: INV_TX Position           */
#define UART_IRCR_INV_TX_Msk     (1ul << UART_IRCR_INV_TX_Pos)              /*!< UART IRCR: INV_TX Mask               */

#define UART_IRCR_TX_SELECT_Pos   1                                         /*!< UART IRCR: TX_SELECT Position        */
#define UART_IRCR_TX_SELECT_Msk   (1ul << UART_IRCR_TX_SELECT_Pos)          /*!< UART IRCR: TX_SELECT Mask            */

/* UART ALT_CSR Bit Field Definitions */
#define UART_ALT_CSR_ADDR_MATCH_Pos      24                                      /*!< UART ALT_CSR: ADDR_MATCH Position    */
#define UART_ALT_CSR_ADDR_MATCH_Msk     (0xFFul << UART_ALT_CSR_ADDR_MATCH_Pos)  /*!< UART ALT_CSR: ADDR_MATCH Mask        */

#define UART_ALT_CSR_RS485_ADD_EN_Pos   15                                       /*!< UART ALT_CSR: RS485_ADD_EN Position  */
#define UART_ALT_CSR_RS485_ADD_EN_Msk   (1ul << UART_ALT_CSR_RS485_ADD_EN_Pos)   /*!< UART ALT_CSR: RS485_ADD_EN Mask      */

#define UART_ALT_CSR_RS485_AUD_Pos      10                                       /*!< UART ALT_CSR: RS485_AUD Position     */
#define UART_ALT_CSR_RS485_AUD_Msk      (1ul << UART_ALT_CSR_RS485_AUD_Pos)      /*!< UART ALT_CSR: RS485_AUD Mask         */

#define UART_ALT_CSR_RS485_AAD_Pos      9                                        /*!< UART ALT_CSR: RS485_AAD Position     */
#define UART_ALT_CSR_RS485_AAD_Msk      (1ul << UART_ALT_CSR_RS485_AAD_Pos)      /*!< UART ALT_CSR: RS485_AAD Mask         */

#define UART_ALT_CSR_RS485_NMM_Pos      8                                        /*!< UART ALT_CSR: RS485_NMM Position     */
#define UART_ALT_CSR_RS485_NMM_Msk      (1ul << UART_ALT_CSR_RS485_NMM_Pos)      /*!< UART ALT_CSR: RS485_NMM Mask         */

/* UART FUN_SEL Bit Field Definitions */
#define UART_FUN_SEL_FUN_SEL_Pos        0                                        /*!< UART FUN_SEL: FUN_SEL Position       */
#define UART_FUN_SEL_FUN_SEL_Msk       (0x3ul << UART_FUN_SEL_FUN_SEL_Pos)       /*!< UART FUN_SEL: FUN_SEL Mask           */

/*@}*/ /* end of group REG_UART_BITMASK */
/*@}*/ /* end of group REG_UART */

/*--------------------------- USB Device Controller --------------------------*/
/** @addtogroup REG_USBD USBD
  Memory Mapped Structure for USB Device Controller
  @{
 */

typedef struct
{
    /**
     * USB_BUFSEG0~7
     * ===================================================================================================
     * Offset: 0x500/0x510/0x520/0x530/0x540/0x550/0x560/0x570  Endpoint 0~7 Buffer Segmentation Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[8:3]   |BUFSEG    |Endpoint Buffer Segmentation
     * |        |          |It is used to indicate the offset address for each endpoint with the USB SRAM starting address The effective starting address of the endpoint is
     * |        |          |USB_SRAM address + { BUFSEG[8:3], 3'b000}
     * |        |          |Where the USB_SRAM address = USBD_BA+0x100h.
     */
    __IO uint32_t BUFSEG;

    /**
     * USB_MXPLD0~7
     * ===================================================================================================
     * Offset: 0x504/0x514/0x524/0x534/0x544/0x554/0x564/0x574  Endpoint 0~7 Maximal Payload Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[8:0]   |MXPLD     |Maximal Payload
     * |        |          |Define the data length which is transmitted to host (IN token) or the actual data length which is received from the host (OUT token).
     * |        |          |It also used to indicate that the endpoint is ready to be transmitted in IN token or received in OUT token.
     * |        |          |(1) When the register is written by CPU,
     * |        |          |For IN token, the value of MXPLD is used to define the data length to be transmitted and indicate the data buffer is ready.
     * |        |          |For OUT token, it means that the controller is ready to receive data from the host and the value of MXPLD is the maximal data length comes from host.
     * |        |          |(2) When the register is read by CPU,
     * |        |          |For IN token, the value of MXPLD is indicated by the data length be transmitted to host
     * |        |          |For OUT token, the value of MXPLD is indicated the actual data length receiving from host.
     * |        |          |Note: Once MXPLD is written, the data packets will be transmitted/received immediately after IN/OUT token arrived.
     */
    __IO uint32_t MXPLD;

    /**
     * USB_CFG0~7
     * ===================================================================================================
     * Offset: 0x508/0x518/0x528/0x538/0x548/0x558/0x568/0x578  Endpoint 0~7 Configuration Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |EP_NUM    |Endpoint Number
     * |        |          |These bits are used to define the endpoint number of the current endpoint.
     * |[4]     |ISOCH     |Isochronous Endpoint
     * |        |          |This bit is used to set the endpoint as Isochronous endpoint, no handshake.
     * |        |          |0 = No Isochronous endpoint.
     * |        |          |1 = Isochronous endpoint.
     * |[6:5]   |STATE     |Endpoint STATE
     * |        |          |00 = Endpoint is Disabled.
     * |        |          |01 = Out endpoint.
     * |        |          |10 = IN endpoint.
     * |        |          |11 = Undefined.
     * |[7]     |DSQ_SYNC  |Data Sequence Synchronization
     * |        |          |0 = DATA0 PID.
     * |        |          |1 = DATA1 PID.
     * |        |          |Note: It is used to specify the DATA0 or DATA1 PID in the following IN token transaction.
     * |        |          |Hardware will toggle automatically in IN token base on the bit.
     * |[9]     |CSTALL    |Clear STALL Response
     * |        |          |0 = Disable the device to clear the STALL handshake in setup stage.
     * |        |          |1 = Clear the device to response STALL handshake in setup stage.
     */
    __IO uint32_t CFG;

    /**
    * USB_CFGP0~7
    * ===================================================================================================
    * Offset: 0x50C/0x51C/0x52C/0x53C/0x54C/0x55C/0x56C/0x57C  Endpoint 0~7 Set Stall and Clear In/Out Ready Control Register
    * ---------------------------------------------------------------------------------------------------
    * |Bits    |Field     |Descriptions
    * | :----: | :----:   | :---- |
    * |[0]     |CLRRDY    |Clear Ready
    * |        |          |When the USB_MXPLD register is set by user, it means that the endpoint is ready to transmit or receive data.
    * |        |          |If the user wants to turn off this transaction before the transaction start, users can set this bit to 1 to turn it off and it will be cleared to 0 automatically.
    * |        |          |For IN token, write '1' to clear the IN token had ready to transmit the data to USB.
    * |        |          |For OUT token, write '1' to clear the OUT token had ready to receive the data from USB.
    * |        |          |This bit is write 1 only and is always 0 when it is read back.
    * |[1]     |SSTALL    |Set STALL
    * |        |          |0 = Disable the device to response STALL.
    * |        |          |1 = Set the device to respond STALL automatically.
    */
    __IO uint32_t CFGP;

} USBD_EP_T;


typedef struct
{
    /**
     * USB_INTEN
     * ===================================================================================================
     * Offset: 0x00  USB Interrupt Enable Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |BUS_IE    |Bus Event Interrupt Enable
     * |        |          |0 = BUS event interrupt Disabled.
     * |        |          |1 = BUS event interrupt Enabled.
     * |[1]     |USB_IE    |USB Event Interrupt Enable
     * |        |          |0 = USB event interrupt Disabled.
     * |        |          |1 = USB event interrupt Enabled.
     * |[2]     |FLDET_IE  |Floating Detection Interrupt Enable
     * |        |          |0 = Floating detection Interrupt Disabled.
     * |        |          |1 = Floating detection Interrupt Enabled.
     * |[3]     |WAKEUP_IE |USB Wake-Up Interrupt Enable
     * |        |          |0 = Wake-up Interrupt Disabled.
     * |        |          |1 = Wake-up Interrupt Enabled.
     * |[8]     |WAKEUP_EN |Wake-Up Function Enable
     * |        |          |0 = USB wake-up function Disabled.
     * |        |          |1 = USB wake-up function Enabled.
     * |[15]    |INNAK_EN  |Active NAK Function And Its Status In IN Token
     * |        |          |0 = When device responds NAK after receiving IN token, IN NAK status will not be
     * |        |          |    updated to USBD_EPSTS register, so that the USB interrupt event will not be asserted.
     * |        |          |1 = IN NAK status will be updated to USBD_EPSTS register and the USB interrupt event
     * |        |          |    will be asserted, when the device responds NAK after receiving IN token.
     */
    __IO uint32_t INTEN;

    /**
     * USB_INTSTS
     * ===================================================================================================
     * Offset: 0x04  USB Interrupt Event Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |BUS_STS   |BUS Interrupt Status
     * |        |          |The BUS event means that there is one of the suspense or the resume function in the bus.
     * |        |          |0 = No BUS event occurred.
     * |        |          |1 = Bus event occurred; check USB_ATTR[3:0] to know which kind of bus event was occurred, cleared by write 1 to USB_INTSTS[0].
     * |[1]     |USB_STS   |USB Event Interrupt Status
     * |        |          |The USB event includes the SETUP Token, IN Token, OUT ACK, ISO IN, or ISO OUT events in the bus.
     * |        |          |0 = No USB event occurred.
     * |        |          |1 = USB event occurred, check EPSTS0~7 to know which kind of USB event occurred.
     * |        |          |Cleared by write 1 to USB_INTSTS[1] or EPEVT0~7 and SETUP (USB_INTSTS[31]).
     * |[2]     |FLDET_STS |Floating Detection Interrupt Status
     * |        |          |0 = There is not attached/detached event in the USB.
     * |        |          |1 = There is attached/detached event in the USB bus and it is cleared by write 1 to USB_INTSTS[2].
     * |[3]     |WAKEUP_STS|Wake-Up Interrupt Status
     * |        |          |0 = No Wake-up event occurred.
     * |        |          |1 = Wake-up event occurred, cleared by write 1 to USB_INTSTS[3].
     * |[16]    |EPEVT0    |Endpoint 0's USB Event Status
     * |        |          |0 = No event occurred on endpoint 0.
     * |        |          |1 = USB event occurred on Endpoint 0, check USB_EPSTS[10:8] to know which kind of USB event was occurred, cleared by write 1 to USB_INTSTS[16] or USB_INTSTS[1].
     * |[17]    |EPEVT1    |Endpoint 1's USB Event Status
     * |        |          |0 = No event occurred on endpoint 1.
     * |        |          |1 = USB event occurred on Endpoint 1, check USB_EPSTS[13:11] to know which kind of USB event was occurred, cleared by write 1 to USB_INTSTS[17] or USB_INTSTS[1].
     * |[18]    |EPEVT2    |Endpoint 2's USB Event Status
     * |        |          |0 = No event occurred on endpoint 2.
     * |        |          |1 = USB event occurred on Endpoint 2, check USB_EPSTS[16:14] to know which kind of USB event was occurred, cleared by write 1 to USB_INTSTS[18] or USB_INTSTS[1].
     * |[19]    |EPEVT3    |Endpoint 3's USB Event Status
     * |        |          |0 = No event occurred on endpoint 3.
     * |        |          |1 = USB event occurred on Endpoint 3, check USB_EPSTS[19:17] to know which kind of USB event was occurred, cleared by write 1 to USB_INTSTS[19] or USB_INTSTS[1].
     * |[20]    |EPEVT4    |Endpoint 4's USB Event Status
     * |        |          |0 = No event occurred on endpoint 4.
     * |        |          |1 = USB event occurred on Endpoint 4, check USB_EPSTS[22:20] to know which kind of USB event was occurred, cleared by write 1 to USB_INTSTS[20] or USB_INTSTS[1].
     * |[21]    |EPEVT5    |Endpoint 5's USB Event Status
     * |        |          |0 = No event occurred on endpoint 5.
     * |        |          |1 = USB event occurred on Endpoint 5, check USB_EPSTS[25:23] to know which kind of USB event was occurred, cleared by write 1 to USB_INTSTS[21] or USB_INTSTS[1].
     * |[22]    |EPEVT6    |Endpoint 6's USB Event Status
     * |        |          |0 = No event occurred on endpoint 6.
     * |        |          |1 = USB event occurred on Endpoint 6, check USB_EPSTS[28:26] to know which kind of USB event was occurred, cleared by write 1 to USB_INTSTS[22] or USB_INTSTS[1].
     * |[23]    |EPEVT7    |Endpoint 7's USB Event Status
     * |        |          |0 = No event occurred on endpoint 7.
     * |        |          |1 = USB event occurred on Endpoint 7, check USB_EPSTS[31:29] to know which kind of USB event was occurred, cleared by write 1 to USB_INTSTS[23] or USB_INTSTS[1].
     * |[31]    |SETUP     |Setup Event Status
     * |        |          |0 = No Setup event.
     * |        |          |1 = SETUP event occurred, cleared by write 1 to USB_INTSTS[31].
     */
    __IO uint32_t INTSTS;

    /**
     * USB_FADDR
     * ===================================================================================================
     * Offset: 0x08  USB Device Function Address Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[6:0]   |FADDR     |USB Device Function Address
     */
    __IO uint32_t FADDR;

    /**
     * USB_EPSTS
     * ===================================================================================================
     * Offset: 0x0C  USB Endpoint Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7]     |OVERRUN   |Overrun
     * |        |          |It indicates that the received data is over the maximum payload number or not.
     * |        |          |0 = No overrun.
     * |        |          |1 = Out Data is more than the Max Payload in MXPLD register or the Setup Data is more than 8 Bytes.
     * |[10:8]  |EPSTS0    |Endpoint 0 Bus Status
     * |        |          |These bits are used to indicate the current status of this endpoint
     * |        |          |000 = In ACK.
     * |        |          |001 = In NAK.
     * |        |          |010 = Out Packet Data0 ACK.
     * |        |          |110 = Out Packet Data1 ACK.
     * |        |          |011 = Setup ACK.
     * |        |          |111 = Isochronous transfer end.
     * |[13:11] |EPSTS1    |Endpoint 1 Bus Status
     * |        |          |These bits are used to indicate the current status of this endpoint
     * |        |          |000 = In ACK.
     * |        |          |001 = In NAK.
     * |        |          |010 = Out Packet Data0 ACK.
     * |        |          |110 = Out Packet Data1 ACK.
     * |        |          |011 = Setup ACK.
     * |        |          |111 = Isochronous transfer end.
     * |[16:14] |EPSTS2    |Endpoint 2 Bus Status
     * |        |          |These bits are used to indicate the current status of this endpoint
     * |        |          |000 = In ACK.
     * |        |          |001 = In NAK.
     * |        |          |010 = Out Packet Data0 ACK.
     * |        |          |110 = Out Packet Data1 ACK.
     * |        |          |011 = Setup ACK.
     * |        |          |111 = Isochronous transfer end.
     * |[19:17] |EPSTS3    |Endpoint 3 Bus Status
     * |        |          |These bits are used to indicate the current status of this endpoint
     * |        |          |000 = In ACK.
     * |        |          |001 = In NAK.
     * |        |          |010 = Out Packet Data0 ACK.
     * |        |          |110 = Out Packet Data1 ACK.
     * |        |          |011 = Setup ACK.
     * |        |          |111 = Isochronous transfer end.
     * |[22:20] |EPSTS4    |Endpoint 4 Bus Status
     * |        |          |These bits are used to indicate the current status of this endpoint
     * |        |          |000 = In ACK.
     * |        |          |001 = In NAK.
     * |        |          |010 = Out Packet Data0 ACK.
     * |        |          |110 = Out Packet Data1 ACK.
     * |        |          |011 = Setup ACK.
     * |        |          |111 = Isochronous transfer end.
     * |[25:23] |EPSTS5    |Endpoint 5 Bus Status
     * |        |          |These bits are used to indicate the current status of this endpoint
     * |        |          |000 = In ACK.
     * |        |          |001 = In NAK.
     * |        |          |010 = Out Packet Data0 ACK.
     * |        |          |110 = Out Packet Data1 ACK.
     * |        |          |011 = Setup ACK.
     * |        |          |111 = Isochronous transfer end.
     * |[28:26] |EPSTS6    |Endpoint 6 Bus Status
     * |        |          |These bits are used to indicate the current status of this endpoint
     * |        |          |000 = In ACK.
     * |        |          |001 = In NAK.
     * |        |          |010 = Out Packet Data0 ACK.
     * |        |          |110 = Out Packet Data1 ACK.
     * |        |          |011 = Setup ACK.
     * |        |          |111 = Isochronous transfer end.
     * |[31:29] |EPSTS7    |Endpoint 7 Bus Status
     * |        |          |These bits are used to indicate the current status of this endpoint
     * |        |          |000 = In ACK.
     * |        |          |001 = In NAK.
     * |        |          |010 = Out Packet Data0 ACK.
     * |        |          |110 = Out Packet Data1 ACK.
     * |        |          |011 = Setup ACK.
     * |        |          |111 = Isochronous transfer end.
     */
    __I  uint32_t EPSTS;

    /**
     * USB_ATTR
     * ===================================================================================================
     * Offset: 0x10  USB Bus Status and Attribution Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |USBRST    |USB Reset Status
     * |        |          |0 = Bus no reset.
     * |        |          |1 = Bus reset when SE0 (single-ended 0) is presented more than 2.5us.
     * |        |          |Note: This bit is read only.
     * |[1]     |SUSPEND   |Suspend Status
     * |        |          |0 = Bus no suspend.
     * |        |          |1 = Bus idle more than 3ms, either cable is plugged off or host is sleeping.
     * |        |          |Note: This bit is read only.
     * |[2]     |RESUME    |Resume Status
     * |        |          |0 = No bus resume.
     * |        |          |1 = Resume from suspend.
     * |        |          |Note: This bit is read only.
     * |[3]     |TIMEOUT   |Time-Out Status
     * |        |          |0 = No time-out.
     * |        |          |1 = No Bus response more than 18 bits time.
     * |        |          |Note: This bit is read only.
     * |[4]     |PHY_EN    |PHY Transceiver Function Enable
     * |        |          |0 = PHY transceiver function Disabled.
     * |        |          |1 = PHY transceiver function Enabled.
     * |[5]     |RWAKEUP   |Remote Wake-Up
     * |        |          |0 = Release the USB bus from K state.
     * |        |          |1 = Force USB bus to K (USB_D+ low, USB_D- high) state, used for remote wake-up.
     * |[7]     |USB_EN    |USB Controller Enable
     * |        |          |0 = USB Controller Disabled.
     * |        |          |1 = USB Controller Enabled.
     * |[8]     |DPPU_EN   |Pull-Up Resistor On USB_D+ Enable
     * |        |          |0 = Pull-up resistor in USB_D+ pin Disabled.
     * |        |          |1 = Pull-up resistor in USB_D+ pin Enabled.
     * |[9]     |PWRDN     |Power-Down PHY Transceiver, Low Active
     * |        |          |0 = Power-down related circuit of PHY transceiver.
     * |        |          |1 = Turn-on related circuit of PHY transceiver.
     * |[10]    |BYTEM     |CPU Access USB SRAM Size Mode Selection
     * |        |          |0 = Word mode: The size of the transfer from CPU to USB SRAM can be Word only.
     * |        |          |1 = Byte mode: The size of the transfer from CPU to USB SRAM can be Byte only.
     */
    __IO uint32_t ATTR;

    /**
     * USB_FLDET
     * ===================================================================================================
     * Offset: 0x14  USB Floating Detection Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |FLDET     |Device Floating Detected
     * |        |          |0 = Controller is not attached into the USB host.
     * |        |          |1 =Controller is attached into the BUS.
     */
    __I  uint32_t FLDET;

    /**
     * USB_STBUFSEG
     * ===================================================================================================
     * Offset: 0x18  Setup Token Buffer Segmentation Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[8:3]   |STBUFSEG  |Setup Token Buffer Segmentation
     * |        |          |It is used to indicate the offset address for the SETUP token with the USB Device SRAM starting address The effective starting address is
     * |        |          |USB_SRAM address + {STBUFSEG[8:3], 3'b000}
     * |        |          |Where the USB_SRAM address = USBD_BA+0x100h.
     * |        |          |Note: It is used for SETUP token only.
     */
    __IO uint32_t STBUFSEG;

    /**
     * @cond HIDDEN_SYMBOLS
     */
    __I  uint32_t RESERVE0[29];
    /**
     * @endcond
     */

    /**
     * USB_DRVSE0
     * ===================================================================================================
     * Offset: 0x90  USB Drive SE0 Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |DRVSE0    |Drive Single Ended Zero In USB Bus
     * |        |          |The Single Ended Zero (SE0) is when both lines (USB_D+ and USB_D-) are being pulled low.
     * |        |          |0 = None.
     * |        |          |1 = Force USB PHY transceiver to drive SE0.
     */
    __IO uint32_t DRVSE0;

    /**
     * @cond HIDDEN_SYMBOLS
     */
    __I  uint32_t RESERVE1[283];
    /**
     * @endcond
     */

    USBD_EP_T EP[8];

} USBD_T;


/** @addtogroup REG_USBD_BITMASK USBD Bit Mask
  @{
 */

/* USBD INTEN Bit Field Definitions */
#define USBD_INTEN_INNAK_EN_Pos    15                                    /*!< USB INTEN: INNAK_EN Position */
#define USBD_INTEN_INNAK_EN_Msk    (1ul << USBD_INTEN_INNAK_EN_Pos)      /*!< USB INTEN: INNAK_EN Mask */

#define USBD_INTEN_WAKEUP_EN_Pos   8                                     /*!< USB INTEN: RWAKEUP Position */
#define USBD_INTEN_WAKEUP_EN_Msk   (1ul << USBD_INTEN_WAKEUP_EN_Pos)     /*!< USB INTEN: RWAKEUP Mask */

#define USBD_INTEN_WAKEUP_IE_Pos   3                                     /*!< USB INTEN: WAKEUP_IE Position */
#define USBD_INTEN_WAKEUP_IE_Msk   (1ul << USBD_INTEN_WAKEUP_IE_Pos)     /*!< USB INTEN: WAKEUP_IE Mask */

#define USBD_INTEN_FLDET_IE_Pos    2                                     /*!< USB INTEN: FLDET_IE Position */
#define USBD_INTEN_FLDET_IE_Msk    (1ul << USBD_INTEN_FLDET_IE_Pos)      /*!< USB INTEN: FLDET_IE Mask */

#define USBD_INTEN_USB_IE_Pos      1                                     /*!< USB INTEN: USB_IE Position */
#define USBD_INTEN_USB_IE_Msk      (1ul << USBD_INTEN_USB_IE_Pos)        /*!< USB INTEN: USB_IE Mask */

#define USBD_INTEN_BUS_IE_Pos      0                                     /*!< USB INTEN: BUS_IE Position */
#define USBD_INTEN_BUS_IE_Msk      (1ul << USBD_INTEN_BUS_IE_Pos)        /*!< USB INTEN: BUS_IE Mask */

/* USBD INTSTS Bit Field Definitions */
#define USBD_INTSTS_SETUP_Pos        31                                  /*!< USB INTSTS: SETUP Position */
#define USBD_INTSTS_SETUP_Msk        (1ul << USBD_INTSTS_SETUP_Pos)      /*!< USB INTSTS: SETUP Mask */

#define USBD_INTSTS_EPEVT_Pos        16                                  /*!< USB INTSTS: EPEVT Position */
#define USBD_INTSTS_EPEVT_Msk        (0xFFul << USBD_INTSTS_EPEVT_Pos)   /*!< USB INTSTS: EPEVT Mask */

#define USBD_INTSTS_WAKEUP_STS_Pos   3                                   /*!< USB INTSTS: WAKEUP_STS Position */
#define USBD_INTSTS_WAKEUP_STS_Msk   (1ul << USBD_INTSTS_WAKEUP_STS_Pos) /*!< USB INTSTS: WAKEUP_STS Mask */

#define USBD_INTSTS_FLDET_STS_Pos    2                                   /*!< USB INTSTS: FLDET_STS Position */
#define USBD_INTSTS_FLDET_STS_Msk    (1ul << USBD_INTSTS_FLDET_STS_Pos)  /*!< USB INTSTS: FLDET_STS Mask */

#define USBD_INTSTS_USB_STS_Pos      1                                   /*!< USB INTSTS: USB_STS Position */
#define USBD_INTSTS_USB_STS_Msk      (1ul << USBD_INTSTS_USB_STS_Pos)    /*!< USB INTSTS: USB_STS Mask */

#define USBD_INTSTS_BUS_STS_Pos      0                                   /*!< USB INTSTS: BUS_STS Position */
#define USBD_INTSTS_BUS_STS_Msk      (1ul << USBD_INTSTS_BUS_STS_Pos)    /*!< USB INTSTS: BUS_STS Mask */

/* USBD FADDR Bit Field Definitions */
#define USBD_FADDR_FADDR_Pos     0                                       /*!< USB FADDR: FADDR Position */
#define USBD_FADDR_FADDR_Msk     (0x7Ful << USBD_FADDR_FADDR_Pos)        /*!< USB FADDR: FADDR Mask */

/* USBD EPSTS Bit Field Definitions */
#define USBD_EPSTS_EPSTS7_Pos    29                                      /*!< USB EPSTS: EPSTS7 Position */
#define USBD_EPSTS_EPSTS7_Msk    (7ul << USBD_EPSTS_EPSTS7_Pos)          /*!< USB EPSTS: EPSTS7 Mask */

#define USBD_EPSTS_EPSTS6_Pos    26                                      /*!< USB EPSTS: EPSTS6 Position */
#define USBD_EPSTS_EPSTS6_Msk    (7ul << USBD_EPSTS_EPSTS6_Pos)          /*!< USB EPSTS: EPSTS6 Mask */

#define USBD_EPSTS_EPSTS5_Pos    23                                      /*!< USB EPSTS: EPSTS5 Position */
#define USBD_EPSTS_EPSTS5_Msk    (7ul << USBD_EPSTS_EPSTS5_Pos)          /*!< USB EPSTS: EPSTS5 Mask */

#define USBD_EPSTS_EPSTS4_Pos    20                                      /*!< USB EPSTS: EPSTS4 Position */
#define USBD_EPSTS_EPSTS4_Msk    (7ul << USBD_EPSTS_EPSTS4_Pos)          /*!< USB EPSTS: EPSTS4 Mask */

#define USBD_EPSTS_EPSTS3_Pos    17                                      /*!< USB EPSTS: EPSTS3 Position */
#define USBD_EPSTS_EPSTS3_Msk    (7ul << USBD_EPSTS_EPSTS3_Pos)          /*!< USB EPSTS: EPSTS3 Mask */

#define USBD_EPSTS_EPSTS2_Pos    14                                      /*!< USB EPSTS: EPSTS2 Position */
#define USBD_EPSTS_EPSTS2_Msk    (7ul << USBD_EPSTS_EPSTS2_Pos)          /*!< USB EPSTS: EPSTS2 Mask */

#define USBD_EPSTS_EPSTS1_Pos    11                                      /*!< USB EPSTS: EPSTS1 Position */
#define USBD_EPSTS_EPSTS1_Msk    (7ul << USBD_EPSTS_EPSTS1_Pos)          /*!< USB EPSTS: EPSTS1 Mask */

#define USBD_EPSTS_EPSTS0_Pos    8                                       /*!< USB EPSTS: EPSTS0 Position */
#define USBD_EPSTS_EPSTS0_Msk    (7ul << USBD_EPSTS_EPSTS0_Pos)          /*!< USB EPSTS: EPSTS0 Mask */

#define USBD_EPSTS_OVERRUN_Pos   7                                       /*!< USB EPSTS: OVERRUN Position */
#define USBD_EPSTS_OVERRUN_Msk   (1ul << USBD_EPSTS_OVERRUN_Pos)         /*!< USB EPSTS: OVERRUN Mask */

/* USBD ATTR Bit Field Definitions */
#define USBD_ATTR_BYTEM_Pos      10                                      /*!< USB ATTR: BYTEM Position */
#define USBD_ATTR_BYTEM_Msk      (1ul << USBD_ATTR_BYTEM_Pos)            /*!< USB ATTR: BYTEM Mask */

#define USBD_ATTR_PWRDN_Pos      9                                       /*!< USB ATTR: PWRDN Position */
#define USBD_ATTR_PWRDN_Msk      (1ul << USBD_ATTR_PWRDN_Pos)            /*!< USB ATTR: PWRDN Mask */

#define USBD_ATTR_DPPU_EN_Pos    8                                       /*!< USB ATTR: DPPU_EN Position */
#define USBD_ATTR_DPPU_EN_Msk    (1ul << USBD_ATTR_DPPU_EN_Pos)          /*!< USB ATTR: DPPU_EN Mask */

#define USBD_ATTR_USB_EN_Pos     7                                       /*!< USB ATTR: USB_EN Position */
#define USBD_ATTR_USB_EN_Msk     (1ul << USBD_ATTR_USB_EN_Pos)           /*!< USB ATTR: USB_EN Mask */

#define USBD_ATTR_RWAKEUP_Pos    5                                       /*!< USB ATTR: RWAKEUP Position */
#define USBD_ATTR_RWAKEUP_Msk    (1ul << USBD_ATTR_RWAKEUP_Pos)          /*!< USB ATTR: RWAKEUP Mask */

#define USBD_ATTR_PHY_EN_Pos     4                                       /*!< USB ATTR: PHY_EN Position */
#define USBD_ATTR_PHY_EN_Msk     (1ul << USBD_ATTR_PHY_EN_Pos)           /*!< USB ATTR: PHY_EN Mask */

#define USBD_ATTR_TIMEOUT_Pos    3                                       /*!< USB ATTR: TIMEOUT Position */
#define USBD_ATTR_TIMEOUT_Msk    (1ul << USBD_ATTR_TIMEOUT_Pos)          /*!< USB ATTR: TIMEOUT Mask */

#define USBD_ATTR_RESUME_Pos     2                                       /*!< USB ATTR: RESUME Position */
#define USBD_ATTR_RESUME_Msk     (1ul << USBD_ATTR_RESUME_Pos)           /*!< USB ATTR: RESUME Mask */

#define USBD_ATTR_SUSPEND_Pos    1                                       /*!< USB ATTR: SUSPEND Position */
#define USBD_ATTR_SUSPEND_Msk    (1ul << USBD_ATTR_SUSPEND_Pos)          /*!< USB ATTR: SUSPEND Mask */

#define USBD_ATTR_USBRST_Pos     0                                       /*!< USB ATTR: USBRST Position */
#define USBD_ATTR_USBRST_Msk     (1ul << USBD_ATTR_USBRST_Pos)           /*!< USB ATTR: USBRST Mask */

/* USBD FLDET Bit Field Definitions */
#define USBD_FLDET_FLDET_Pos     0                                       /*!< USB FLDET: FLDET Position */
#define USBD_FLDET_FLDET_Msk     (1ul << USBD_FLDET_FLDET_Pos)           /*!< USB FLDET: FLDET Mask */

/* USBD STBUFSEG Bit Field Definitions */
#define USBD_STBUFSEG_STBUFSEG_Pos   3                                        /*!< USB STBUFSEG: STBUFSEG Position */
#define USBD_STBUFSEG_STBUFSEG_Msk   (0x3Ful << USBD_STBUFSEG_STBUFSEG_Pos)   /*!< USB STBUFSEG: STBUFSEG Mask */

/* USBD BUFSEG Bit Field Definitions */
#define USBD_BUFSEG_BUFSEG_Pos   3                                       /*!< USB BUFSEG: BUFSEG Position */
#define USBD_BUFSEG_BUFSEG_Msk   (0x3Ful << USBD_BUFSEG_BUFSEG_Pos)      /*!< USB BUFSEG: BUFSEG Mask */

/* USBD MXPLD Bit Field Definitions */
#define USBD_MXPLD_MXPLD_Pos    0                                        /*!< USB MXPLD: MXPLD Position */
#define USBD_MXPLD_MXPLD_Msk    (0x1FFul << USBD_MXPLD_MXPLD_Pos)        /*!< USB MXPLD: MXPLD Mask */

/* USBD CFG Bit Field Definitions */
#define USBD_CFG_CSTALL_Pos     9                                        /*!< USB CFG: CSTALL Position */
#define USBD_CFG_CSTALL_Msk     (1ul << USBD_CFG_CSTALL_Pos)             /*!< USB CFG: CSTALL Mask */

#define USBD_CFG_DSQ_SYNC_Pos   7                                        /*!< USB CFG: DSQ_SYNC Position */
#define USBD_CFG_DSQ_SYNC_Msk   (1ul << USBD_CFG_DSQ_SYNC_Pos)           /*!< USB CFG: DSQ_SYNC Mask */

#define USBD_CFG_STATE_Pos      5                                        /*!< USB CFG: STATE Position */
#define USBD_CFG_STATE_Msk      (3ul << USBD_CFG_STATE_Pos)              /*!< USB CFG: STATE Mask */

#define USBD_CFG_ISOCH_Pos      4                                        /*!< USB CFG: ISOCH Position */
#define USBD_CFG_ISOCH_Msk      (1ul << USBD_CFG_ISOCH_Pos)              /*!< USB CFG: ISOCH Mask */

#define USBD_CFG_EP_NUM_Pos     0                                        /*!< USB CFG: EP_NUM Position */
#define USBD_CFG_EP_NUM_Msk     (0xFul << USBD_CFG_EP_NUM_Pos)           /*!< USB CFG: EP_NUM Mask */

/* USBD CFGP Bit Field Definitions */
#define USBD_CFGP_SSTALL_Pos    1                                        /*!< USB CFGP: SSTALL Position */
#define USBD_CFGP_SSTALL_Msk    (1ul << USBD_CFGP_SSTALL_Pos)            /*!< USB CFGP: SSTALL Mask */

#define USBD_CFGP_CLRRDY_Pos    0                                        /*!< USB CFGP: CLRRDY Position */
#define USBD_CFGP_CLRRDY_Msk    (1ul << USBD_CFGP_CLRRDY_Pos)            /*!< USB CFGP: CLRRDY Mask */

/* USBD DRVSE0 Bit Field Definitions */
#define USBD_DRVSE0_DRVSE0_Pos   0                                       /*!< USB DRVSE0: DRVSE0 Position */
#define USBD_DRVSE0_DRVSE0_Msk   (1ul << USBD_DRVSE0_DRVSE0_Pos)         /*!< USB DRVSE0: DRVSE0 Mask */
/*@}*/ /* end of group REG_USBD_BITMASK */
/*@}*/ /* end of group REG_USBD */


/*----------------------------- Watchdog Timer (WDT) -----------------------------*/
/** @addtogroup REG_WDT WDT
  Memory Mapped Structure for Watchdog Timer
  @{
 */
typedef struct
{
    /**
     * WTCR
     * ===================================================================================================
     * Offset: 0x00  Watchdog Timer Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |WTR       |Reset Watchdog Timer Up Counter (Write Protect)
     * |        |          |0 = No effect.
     * |        |          |1 = Reset the internal 18-bit WDT up counter value.
     * |        |          |Note: This bit will be automatically cleared by hardware.
     * |[1]     |WTRE      |Watchdog Timer Reset Enable (Write Protect)
     * |        |          |Setting this bit will enable the WDT time-out reset function if the WDT up counter value has not been cleared after the specific WDT reset delay period expires.
     * |        |          |0 = WDT time-out reset function Disabled.
     * |        |          |1 = WDT time-out reset function Enabled.
     * |[2]     |WTRF      |Watchdog Timer Time-out Reset Flag
     * |        |          |This bit indicates the system has been reset by WDT time-out reset or not.
     * |        |          |0 = WDT time-out reset did not occur.
     * |        |          |1 = WDT time-out reset occurred.
     * |        |          |Note: This bit is cleared by writing 1 to it.
     * |[3]     |WTIF      |Watchdog Timer Time-out Interrupt Flag
     * |        |          |This bit will set to 1 while WDT up counter value reaches the selected WDT time-out interval.
     * |        |          |0 = WDT time-out interrupt did not occur.
     * |        |          |1 = WDT time-out interrupt occurred.
     * |        |          |Note: This bit is cleared by writing 1 to it.
     * |[4]     |WTWKE     |Watchdog Timer Time-out Wake-Up Function Control
     * |        |          |(Write Protect)
     * |        |          |If this bit is set to 1, while WTIF is generated to 1 and WTIE enabled, the WDT time-out interrupt signal will generate a wake-up trigger event to chip.
     * |        |          |0 = Wake-up trigger event Disabled if WDT time-out interrupt signal generated.
     * |        |          |1 = Wake-up trigger event Enabled if WDT time-out interrupt signal generated.
     * |        |          |Note: Chip can be woken-up by WDT time-out interrupt signal generated only if WDT clock source is selected to 10 kHz oscillator.
     * |[5]     |WTWKF     |Watchdog Timer Time-out Wake-Up Flag
     * |        |          |This bit indicates the interrupt wake-up flag status of WDT.
     * |        |          |0 = WDT does not cause chip wake-up.
     * |        |          |1 = Chip wake-up from Idle or Power-down mode if WDT time-out interrupt signal generated.
     * |        |          |Note: This bit is cleared by writing 1 to it.
     * |[6]     |WTIE      |Watchdog Timer Time-out Interrupt Enable Control (Write Protect)
     * |        |          |If this bit is enabled, the WDT time-out interrupt signal is generated and inform to CPU.
     * |        |          |0 = WDT time-out interrupt Disabled.
     * |        |          |1 = WDT time-out interrupt Enabled.
     * |[7]     |WTE       |Watchdog Timer Enable Control (Write Protect)
     * |        |          |0 = WDT Disabled. (This action will reset the internal up counter value.)
     * |        |          |1 = WDT Enabled.
     * |        |          |Note: If CWDTEN (CONFIG0[31] Watchdog Enable) bit is set to 0, this bit is forced as 1 and
     * |        |          | user cannot change this bit to 0.
     * |[10:8]  |WTIS      |Watchdog Timer Time-out Interval Selection (Write Protect)
     * |        |          |These three bits select the time-out interval period for the WDT.
     * |        |          |000 = 24 *TWDT.
     * |        |          |001 = 26 * TWDT.
     * |        |          |010 = 28 * TWDT.
     * |        |          |011 = 210 * TWDT.
     * |        |          |100 = 212 * TWDT.
     * |        |          |101 = 214 * TWDT.
     * |        |          |110 = 216 * TWDT.
     * |        |          |111 = 218 * TWDT.
     * |[31]    |DBGACK_WDT|ICE Debug Mode Acknowledge Disable Control (Write Protect)
     * |        |          |0 = ICE debug mode acknowledgment effects WDT counting.
     * |        |          |WDT up counter will be held while CPU is held by ICE.
     * |        |          |1 = ICE debug mode acknowledgment Disabled.
     * |        |          |WDT up counter will keep going no matter CPU is held by ICE or not.
     */
    __IO uint32_t  WTCR;

    /**
     * WTCRALT
     * ===================================================================================================
     * Offset: 0x04  Watchdog Timer Alternative Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1:0]   |WTRDSEL   |Watchdog Timer Reset Delay Selection (Write Protect)
     * |        |          |When WDT time-out happened, user has a time named WDT Reset Delay Period to clear WDT counter to prevent WDT time-out reset happened.
     * |        |          |User can select a suitable value of WDT Reset Delay Period for different WDT time-out period.
     * |        |          |These bits are protected bit.
     * |        |          |It means programming this bit needs to write "59h", "16h", "88h" to address 0x5000_0100 to disable register protection.
     * |        |          |Reference the register REGWRPROT at address GCR_BA+0x100.
     * |        |          |00 = Watchdog Timer Reset Delay Period is 1026 * WDT_CLK.
     * |        |          |01 = Watchdog Timer Reset Delay Period is 130 * WDT_CLK.
     * |        |          |10 = Watchdog Timer Reset Delay Period is 18 * WDT_CLK.
     * |        |          |11 = Watchdog Timer Reset Delay Period is 3 * WDT_CLK.
     * |        |          |Note: This register will be reset to 0 if WDT time-out reset happened.
     */
    __IO uint32_t  WTCRALT;
} WDT_T;


/** @addtogroup REG_WDT_BITMASK WDT Bit Mask
  @{
 */

/* WDT WTCR Bit Field Definitions */
#define WDT_WTCR_DBGACK_WDT_Pos 31                                              /*!< WDT WTCR : DBGACK_WDT Position */
#define WDT_WTCR_DBGACK_WDT_Msk (1ul << WDT_WTCR_DBGACK_WDT_Pos)                /*!< WDT WTCR : DBGACK_WDT Mask */

#define WDT_WTCR_WTIS_Pos       8                                               /*!< WDT WTCR : WTIS Position */
#define WDT_WTCR_WTIS_Msk       (0x7ul << WDT_WTCR_WTIS_Pos)                    /*!< WDT WTCR : WTIS Mask */

#define WDT_WTCR_WTE_Pos        7                                               /*!< WDT WTCR : WTE Position */
#define WDT_WTCR_WTE_Msk        (1ul << WDT_WTCR_WTE_Pos)                       /*!< WDT WTCR : WTE Mask */

#define WDT_WTCR_WTIE_Pos       6                                               /*!< WDT WTCR : WTIE Position */
#define WDT_WTCR_WTIE_Msk       (1ul << WDT_WTCR_WTIE_Pos)                      /*!< WDT WTCR : WTIE Mask */

#define WDT_WTCR_WTWKF_Pos      5                                               /*!< WDT WTCR : WTWKF Position */
#define WDT_WTCR_WTWKF_Msk      (1ul << WDT_WTCR_WTWKF_Pos)                     /*!< WDT WTCR : WTWKF Mask */

#define WDT_WTCR_WTWKE_Pos      4                                               /*!< WDT WTCR : WTWKE Position */
#define WDT_WTCR_WTWKE_Msk      (1ul << WDT_WTCR_WTWKE_Pos)                     /*!< WDT WTCR : WTWKE Mask */

#define WDT_WTCR_WTIF_Pos       3                                               /*!< WDT WTCR : WTIF Position */
#define WDT_WTCR_WTIF_Msk       (1ul << WDT_WTCR_WTIF_Pos)                      /*!< WDT WTCR : WTIF Mask */

#define WDT_WTCR_WTRF_Pos       2                                               /*!< WDT WTCR : WTRF Position */
#define WDT_WTCR_WTRF_Msk       (1ul << WDT_WTCR_WTRF_Pos)                      /*!< WDT WTCR : WTRF Mask */

#define WDT_WTCR_WTRE_Pos       1                                               /*!< WDT WTCR : WTRE Position */
#define WDT_WTCR_WTRE_Msk       (1ul << WDT_WTCR_WTRE_Pos)                      /*!< WDT WTCR : WTRE Mask */

#define WDT_WTCR_WTR_Pos        0                                               /*!< WDT WTCR : WTR Position */
#define WDT_WTCR_WTR_Msk        (1ul << WDT_WTCR_WTR_Pos)                       /*!< WDT WTCR : WTR Mask */

/* WDT WTCRALT Bit Field Definitions */
#define WDT_WTCRALT_WTRDSEL_Pos 0                                               /*!< WDT WTCRALT : WTRDSEL Position */
#define WDT_WTCRALT_WTRDSEL_Msk (0x3ul << WDT_WTCRALT_WTRDSEL_Pos)              /*!< WDT WTCRALT : WTRDSEL Mask */
/*@}*/ /* end of group REG_WDT_BITMASK */
/*@}*/ /* end of group REG_WDT */


/*----------------------------- Window Watchdog Timer (WWDT) -----------------------------*/
/** @addtogroup REG_WWDT WWDT
  Memory Mapped Structure for Window Watchdog Timer
  @{
 */
typedef struct
{
    /**
     * WWDTRLD
     * ===================================================================================================
     * Offset: 0x00  Window Watchdog Timer Reload Counter Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |WWDTRLD   |WWDT Reload Counter Register
     * |        |          |Writing 0x00005AA5 to this register will reload the WWDT counter value to 0x3F.
     * |        |          |Note: User can only write WWDTRLD to reload WWDT counter value when current WWDT
     * |        |          | counter value between 0 and WINCMP. If user writes WWDTRLD when current WWDT
     * |        |          | counter value is larger than WINCMP, WWDT reset signal will generate immediately.
     */
    __IO uint32_t  WWDTRLD;

    /**
     * WWDTCR
     * ===================================================================================================
     * Offset: 0x04  Window Watchdog Timer Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |WWDTEN    |WWDT Enable Control
     * |        |          |0 = WWDT counter is stopped.
     * |        |          |1 = WWDT counter is starting counting.
     * |[1]     |WWDTIE    |WWDT Interrupt Enable Control
     * |        |          |If this bit is enabled, the WWDT counter compare match interrupt signal is generated and inform to CPU.
     * |        |          |0 = WWDT counter compare match interrupt Disabled.
     * |        |          |1 = WWDT counter compare match interrupt Enabled.
     * |[11:8]  |PERIODSEL |WWDT Counter Prescale Period Selection
     * |        |          |0000 = Pre-scale is 1; Max time-out period is 1 * 64 * TWWDT.
     * |        |          |0001 = Pre-scale is 2; Max time-out period is 2 * 64 * TWWDT.
     * |        |          |0010 = Pre-scale is 4; Max time-out period is 4 * 64 * TWWDT.
     * |        |          |0011 = Pre-scale is 8; Max time-out period is 8 * 64 * TWWDT.
     * |        |          |0100 = Pre-scale is 16; Max time-out period is 16 * 64 * TWWDT.
     * |        |          |0101 = Pre-scale is 32; Max time-out period is 32 * 64 * TWWDT.
     * |        |          |0110 = Pre-scale is 64; Max time-out period is 64 * 64 * TWWDT.
     * |        |          |0111 = Pre-scale is 128; Max time-out period is 128 * 64 * TWWDT.
     * |        |          |1000 = Pre-scale is 192; Max time-out period is 192 * 64 * TWWDT.
     * |        |          |1001 = Pre-scale is 256; Max time-out period is 256 * 64 * TWWDT.
     * |        |          |1010 = Pre-scale is 384; Max time-out period is 384 * 64 * TWWDT.
     * |        |          |1011 = Pre-scale is 512; Max time-out period is 512 * 64 * TWWDT.
     * |        |          |1100 = Pre-scale is 768; Max time-out period is 768 * 64 * TWWDT.
     * |        |          |1101 = Pre-scale is 1024; Max time-out period is 1024 * 64 * TWWDT.
     * |        |          |1110 = Pre-scale is 1536; Max time-out period is 1536 * 64 * TWWDT.
     * |        |          |1111 = Pre-scale is 2048; Max time-out period is 2048 * 64 * TWWDT.
     * |[21:16] |WINCMP    |WWDT Window Compare Register
     * |        |          |Set this register to adjust the valid reload window.
     * |        |          |Note: User can only write WWDTRLD to reload WWDT counter value when current WWDT counter value between 0 and WINCMP.
     * |        |          |If user writes WWDTRLD when current WWDT counter value larger than WINCMP, WWDT reset signal will generate immediately.
     * |[31]    |DBGACK_WWDT|ICE Debug Mode Acknowledge Disable Control
     * |        |          |0 = ICE debug mode acknowledgment effects WWDT counting.
     * |        |          |WWDT down counter will be held while CPU is held by ICE.
     * |        |          |1 = ICE debug mode acknowledgment Disabled.
     * |        |          |WWDT down counter will keep going no matter CPU is held by ICE or not.
     */
    __IO uint32_t  WWDTCR;

    /**
     * WWDTSR
     * ===================================================================================================
     * Offset: 0x08  Window Watchdog Timer Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |WWDTIF    |WWDT Compare Match Interrupt Flag
     * |        |          |This bit indicates the interrupt flag status of WWDT while WWDT counter value matches WINCMP value.
     * |        |          |0 = No effect.
     * |        |          |1 = WWDT counter value matches WINCMP value.
     * |        |          |Note: This bit is cleared by writing 1 to it.
     * |[1]     |WWDTRF    |WWDT Time-out Reset Flag
     * |        |          |This bit indicates the system has been reset by WWDT time-out reset or not.
     * |        |          |0 = WWDT time-out reset did not occur.
     * |        |          |1 = WWDT time-out reset occurred.
     * |        |          |Note: This bit is cleared by writing 1 to it.
     */
    __IO uint32_t  WWDTSR;

    /**
     * WWDTCVR
     * ===================================================================================================
     * Offset: 0x0C  Window Watchdog Timer Counter Value Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[5:0]   |WWDTCVAL  |WWDT Counter Value
     * |        |          |WWDTCVAL will be updated continuously to monitor 6-bit down counter value.
     */
    __I  uint32_t  WWDTCVR;
} WWDT_T;

/** @addtogroup REG_WWDT_BITMASK WWDT Bit Mask
  @{
 */

/* WWDT WWDTRLD Bit Field Definitions */
#define WWDT_WWDTRLD_WWDTRLD_Pos    0                                           /*!< WWDT WWDTRLD : WWDTRLD Position */
#define WWDT_WWDTRLD_WWDTRLD_Msk    (0xFFFFFFFFul << WWDT_WWDTRLD_WWDTRLD_Pos)  /*!< WWDT WWDTRLD : WWDTRLD Mask */

/* WWDT WWDTCR Bit Field Definitions */
#define WWDT_WWDTCR_DBGACK_WWDT_Pos 31                                          /*!< WWDT WWDTCR : DBGACK_WWDT Position */
#define WWDT_WWDTCR_DBGACK_WWDT_Msk (1ul << WWDT_WWDTCR_DBGACK_WWDT_Pos)        /*!< WWDT WWDTCR : DBGACK_WWDT Mask */

#define WWDT_WWDTCR_WINCMP_Pos      16                                          /*!< WWDT WWDTCR : WINCMP Position */
#define WWDT_WWDTCR_WINCMP_Msk      (0x3Ful << WWDT_WWDTCR_WINCMP_Pos)          /*!< WWDT WWDTCR : WINCMP Mask */

#define WWDT_WWDTCR_PERIODSEL_Pos   8                                           /*!< WWDT WWDTCR : PERIODSEL Position */
#define WWDT_WWDTCR_PERIODSEL_Msk   (0xFul << WWDT_WWDTCR_PERIODSEL_Pos)        /*!< WWDT WWDTCR : PERIODSEL Mask */

#define WWDT_WWDTCR_WWDTIE_Pos      1                                           /*!< WWDT WWDTCR : WWDTIE Position */
#define WWDT_WWDTCR_WWDTIE_Msk      (1ul << WWDT_WWDTCR_WWDTIE_Pos)             /*!< WWDT WWDTCR : WWDTIE Mask */

#define WWDT_WWDTCR_WWDTEN_Pos      0                                           /*!< WWDT WWDTCR : WWDTEN Position */
#define WWDT_WWDTCR_WWDTEN_Msk      (1ul << WWDT_WWDTCR_WWDTEN_Pos)             /*!< WWDT WWDTCR : WWDTEN Mask */

/* WWDT WWDTSR Bit Field Definitions */
#define WWDT_WWDTSR_WWDTRF_Pos      1                                           /*!< WWDT WWDTSR : WWDTRF Position */
#define WWDT_WWDTSR_WWDTRF_Msk      (1ul << WWDT_WWDTSR_WWDTRF_Pos)             /*!< WWDT WWDTSR : WWDTRF Mask */

#define WWDT_WWDTSR_WWDTIF_Pos      0                                           /*!< WWDT WWDTSR : WWDTIF Position */
#define WWDT_WWDTSR_WWDTIF_Msk      (1ul << WWDT_WWDTSR_WWDTIF_Pos)             /*!< WWDT WWDTSR : WWDTIF Mask */

/* WWDT WWDTCVR Bit Field Definitions */
#define WWDT_WWDTCVR_WWDTCVAL_Pos   0                                           /*!< WWDT WWDTCVR : WWDTRF Position */
#define WWDT_WWDTCVR_WWDTCVAL_Msk   (0x3Ful << WWDT_WWDTCVR_WWDTCVAL_Pos)       /*!< WWDT WWDTCVR : WWDTRF Mask */
/*@}*/ /* end of group REG_WWDT_BITMASK */
/*@}*/ /* end of group REG_WWDT */
/*@}*/ /* end of group REGISTER */


/******************************************************************************/
/*                         Peripheral memory map                              */
/******************************************************************************/
/** @addtogroup PERIPHERAL_MEM_MAP Peripheral Memory Map
  Memory Mapped Structure for Series Peripheral
  @{
 */
/* Peripheral and SRAM base address */
#define FLASH_BASE          ((     uint32_t)0x00000000)
#define SRAM_BASE           ((     uint32_t)0x20000000)
#define AHB_BASE            ((     uint32_t)0x50000000)
#define APB1_BASE           ((     uint32_t)0x40000000)
#define APB2_BASE           ((     uint32_t)0x40100000)

/* Peripheral memory map */
#define GPIO_BASE           (AHB_BASE        + 0x4000)                   /*!< GPIO Base Address                                   */
#define PA_BASE             (GPIO_BASE               )                   /*!< GPIO PORTA Base Address                             */
#define PB_BASE             (GPIO_BASE       + 0x0040)                   /*!< GPIO PORTB Base Address                             */
#define PC_BASE             (GPIO_BASE       + 0x0080)                   /*!< GPIO PORTC Base Address                             */
#define PD_BASE             (GPIO_BASE       + 0x00C0)                   /*!< GPIO PORTD Base Address                             */
#define PE_BASE             (GPIO_BASE       + 0x0100)                   /*!< GPIO PORTE Base Address                             */
#define PF_BASE             (GPIO_BASE       + 0x0140)                   /*!< GPIO PORTF Base Address                             */
#define GPIO_DBNCECON_BASE  (GPIO_BASE       + 0x0180)                   /*!< GPIO De-bounce Cycle Control Base Address           */
#define GPIO_PIN_DATA_BASE  (GPIO_BASE       + 0x0200)                   /*!< GPIO Pin Data Input/Output Control Base Address     */


#define UART0_BASE           (APB1_BASE      + 0x50000)
#define UART1_BASE           (APB2_BASE      + 0x50000)
#define UART2_BASE           (APB2_BASE      + 0x54000)


#define TIMER0_BASE          (APB1_BASE      + 0x10000)                 /*!< Timer0 Base Address                              */
#define TIMER1_BASE          (APB1_BASE      + 0x10020)                 /*!< Timer1 Base Address                              */
#define TIMER2_BASE          (APB2_BASE      + 0x10000)                 /*!< Timer2 Base Address                              */
#define TIMER3_BASE          (APB2_BASE      + 0x10020)                 /*!< Timer3 Base Address                              */

#define WDT_BASE             (APB1_BASE      + 0x4000)                  /*!< Watchdog Timer Base Address                      */

#define WWDT_BASE            (APB1_BASE      + 0x4100)                  /*!< Window Watchdog Timer Base Address               */

#define SPI0_BASE            (APB1_BASE      + 0x30000)                 /*!< SPI0 Base Address                                */
#define SPI1_BASE            (APB1_BASE      + 0x34000)                 /*!< SPI1 Base Address                                */
#define SPI2_BASE            (APB2_BASE      + 0x30000)                 /*!< SPI2 Base Address                                */

#define I2C0_BASE            (APB1_BASE      + 0x20000)                 /*!< I2C0 Base Address                                */
#define I2C1_BASE            (APB2_BASE      + 0x20000)                 /*!< I2C1 Base Address                                */

#define ADC_BASE             (APB1_BASE      + 0xE0000)                 /*!< ADC Base Address                                 */

#define CLK_BASE             (AHB_BASE       + 0x00200)                 /*!< System Clock Controller Base Address             */

#define GCR_BASE             (AHB_BASE       + 0x00000)                 /*!< System Global Controller Base Address            */

#define INT_BASE             (AHB_BASE       + 0x00300)                 /*!< Interrupt Source Controller Base Address         */

#define FMC_BASE             (AHB_BASE       + 0x0C000)

#define PS2_BASE             (APB2_BASE      + 0x00000)                 /*!< PS/2 Base Address                                */

#define USBD_BASE            (APB1_BASE      + 0x60000)                 /*!< USBD Base Address                                */

#define PDMA0_BASE           (AHB_BASE       + 0x08000)                 /*!< PDMA0 Base Address                               */
#define PDMA1_BASE           (AHB_BASE       + 0x08100)                 /*!< PDMA1 Base Address                               */
#define PDMA2_BASE           (AHB_BASE       + 0x08200)                 /*!< PDMA2 Base Address                               */
#define PDMA3_BASE           (AHB_BASE       + 0x08300)                 /*!< PDMA3 Base Address                               */
#define PDMA4_BASE           (AHB_BASE       + 0x08400)                 /*!< PDMA4 Base Address                               */
#define PDMA5_BASE           (AHB_BASE       + 0x08500)                 /*!< PDMA5 Base Address                               */

#define PDMA_GCR_BASE        (AHB_BASE       + 0x08F00)                 /*!< PDMA Global Base Address                         */

#define CRC_BASE             (AHB_BASE       + 0x08E00)                 /*!< CRC Base Address                                 */

#define PWMA_BASE            (APB1_BASE      + 0x40000)                 /*!< PWMA Base Address                                */

#define I2S_BASE             (APB2_BASE      + 0xA0000)                 /*!< I2S Base Address                                 */

/*@}*/ /* end of group PERIPHERAL_MEM_MAP */

/******************************************************************************/
/*                         Peripheral Definitions                             */
/******************************************************************************/

/** @addtogroup PERIPHERAL Peripheral Definitions
  The Definitions of Peripheral
  @{
 */
#define PA                  ((GPIO_T *) PA_BASE)                        /*!< GPIO PORTA Configuration Struct                        */
#define PB                  ((GPIO_T *) PB_BASE)                        /*!< GPIO PORTB Configuration Struct                        */
#define PC                  ((GPIO_T *) PC_BASE)                        /*!< GPIO PORTC Configuration Struct                        */
#define PD                  ((GPIO_T *) PD_BASE)                        /*!< GPIO PORTD Configuration Struct                        */
#define PF                  ((GPIO_T *) PF_BASE)                        /*!< GPIO PORTF Configuration Struct                        */
#define GPIO                ((GPIO_DBNCECON_T *) GPIO_DBNCECON_BASE)    /*!< Interrupt De-bounce Cycle Control Configuration Struct */

#define UART0               ((UART_T *) UART0_BASE)                     /*!< UART0 Configuration Struct                       */
#define UART1               ((UART_T *) UART1_BASE)                     /*!< UART1 Configuration Struct                       */

#define TIMER0              ((TIMER_T *) TIMER0_BASE)                   /*!< Timer0 Configuration Struct                      */
#define TIMER1              ((TIMER_T *) TIMER1_BASE)                   /*!< Timer1 Configuration Struct                      */
#define TIMER2              ((TIMER_T *) TIMER2_BASE)                   /*!< Timer2 Configuration Struct                      */
#define TIMER3              ((TIMER_T *) TIMER3_BASE)                   /*!< Timer3 Configuration Struct                      */

#define WDT                 ((WDT_T *) WDT_BASE)                        /*!< Watchdog Timer Configuration Struct              */

#define WWDT                ((WWDT_T *) WWDT_BASE)                      /*!< Window Watchdog Timer Configuration Struct       */

#define SPI0                ((SPI_T *) SPI0_BASE)                       /*!< SPI0 Configuration Struct                        */
#define SPI1                ((SPI_T *) SPI1_BASE)                       /*!< SPI1 Configuration Struct                        */
#define SPI2                ((SPI_T *) SPI2_BASE)                       /*!< SPI2 Configuration Struct                        */

#define I2C0                ((I2C_T *) I2C0_BASE)                       /*!< I2C0 Configuration Struct                        */
#define I2C1                ((I2C_T *) I2C1_BASE)                       /*!< I2C1 Configuration Struct                        */

#define I2S                 ((I2S_T *) I2S_BASE)                        /*!< I2S Configuration Struct                         */

#define ADC                 ((ADC_T *) ADC_BASE)                        /*!< ADC Configuration Struct                         */

#define CLK                 ((CLK_T *) CLK_BASE)                        /*!< System Clock Controller Configuration Struct     */

#define SYS                 ((GCR_T *) GCR_BASE)                        /*!< System Global Controller Configuration Struct    */

#define SYSINT              ((GCR_INT_T *) INT_BASE)                    /*!< Interrupt Source Controller Configuration Struct */

#define FMC                 ((FMC_T *) FMC_BASE)

#define PS2                 ((PS2_T *) PS2_BASE)                        /*!< PS/2 Configuration Struct                        */

#define USBD                ((USBD_T *) USBD_BASE)                      /*!< USBD Configuration Struct                        */

#define PDMA0               ((PDMA_T *) PDMA0_BASE)                     /*!< PDMA0 Configuration Struct                       */
#define PDMA1               ((PDMA_T *) PDMA1_BASE)                     /*!< PDMA1 Configuration Struct                       */
#define PDMA2               ((PDMA_T *) PDMA2_BASE)                     /*!< PDMA2 Configuration Struct                       */
#define PDMA3               ((PDMA_T *) PDMA3_BASE)                     /*!< PDMA3 Configuration Struct                       */
#define PDMA4               ((PDMA_T *) PDMA4_BASE)                     /*!< PDMA4 Configuration Struct                       */
#define PDMA5               ((PDMA_T *) PDMA5_BASE)                     /*!< PDMA5 Configuration Struct                       */

#define PDMA_GCR            ((PDMA_GCR_T *) PDMA_GCR_BASE)              /*!< PDMA Global Configuration Struct                 */

#define CRC                 ((CRC_T *) CRC_BASE)                        /*!< CRC Configuration Struct                         */

#define PWMA                ((PWM_T *) PWMA_BASE)                       /*!< PWMA Configuration Struct                        */

/*@}*/ /* end of group PERIPHERAL */

#define UNLOCKREG()        do{*((__IO uint32_t *)(GCR_BASE + 0x100)) = 0x59;*((__IO uint32_t *)(GCR_BASE + 0x100)) = 0x16;*((__IO uint32_t *)(GCR_BASE + 0x100)) = 0x88;}while(*((__IO uint32_t *)(GCR_BASE + 0x100))==0)
#define LOCKREG()          *((__IO uint32_t *)(GCR_BASE + 0x100)) = 0x00

#define REGCOPY(dest, src)  *((uint32_t *)&(dest)) = *((uint32_t *)&(src))
#define CLEAR(dest)         *((uint32_t *)&(dest)) = 0

//=============================================================================
/** @addtogroup IO_ROUTINE I/O routines
  The Declaration of I/O routines
  @{
 */

typedef volatile unsigned char  vu8;        ///< Define 8-bit unsigned volatile data type
typedef volatile unsigned short vu16;       ///< Define 16-bit unsigned volatile data type
typedef volatile unsigned long  vu32;       ///< Define 32-bit unsigned volatile data type

/**
  * @brief Get a 8-bit unsigned value from specified address
  * @param[in] addr Address to get 8-bit data from
  * @return  8-bit unsigned value stored in specified address
  */
#define M8(addr)  (*((vu8  *) (addr)))

/**
  * @brief Get a 16-bit unsigned value from specified address
  * @param[in] addr Address to get 16-bit data from
  * @return  16-bit unsigned value stored in specified address
  * @note The input address must be 16-bit aligned
  */
#define M16(addr) (*((vu16 *) (addr)))

/**
  * @brief Get a 32-bit unsigned value from specified address
  * @param[in] addr Address to get 32-bit data from
  * @return  32-bit unsigned value stored in specified address
  * @note The input address must be 32-bit aligned
  */
#define M32(addr) (*((vu32 *) (addr)))

/**
  * @brief Set a 32-bit unsigned value to specified I/O port
  * @param[in] port Port address to set 32-bit data
  * @param[in] value Value to write to I/O port
  * @return  None
  * @note The output port must be 32-bit aligned
  */
#define outpw(port,value)     *((volatile unsigned int *)(port)) = (value)

/**
  * @brief Get a 32-bit unsigned value from specified I/O port
  * @param[in] port Port address to get 32-bit data from
  * @return  32-bit unsigned value stored in specified I/O port
  * @note The input port must be 32-bit aligned
  */
#define inpw(port)            (*((volatile unsigned int *)(port)))

/**
  * @brief Set a 16-bit unsigned value to specified I/O port
  * @param[in] port Port address to set 16-bit data
  * @param[in] value Value to write to I/O port
  * @return  None
  * @note The output port must be 16-bit aligned
  */
#define outps(port,value)     *((volatile unsigned short *)(port)) = (value)

/**
  * @brief Get a 16-bit unsigned value from specified I/O port
  * @param[in] port Port address to get 16-bit data from
  * @return  16-bit unsigned value stored in specified I/O port
  * @note The input port must be 16-bit aligned
  */
#define inps(port)            (*((volatile unsigned short *)(port)))

/**
  * @brief Set a 8-bit unsigned value to specified I/O port
  * @param[in] port Port address to set 8-bit data
  * @param[in] value Value to write to I/O port
  * @return  None
  */
#define outpb(port,value)     *((volatile unsigned char *)(port)) = (value)

/**
  * @brief Get a 8-bit unsigned value from specified I/O port
  * @param[in] port Port address to get 8-bit data from
  * @return  8-bit unsigned value stored in specified I/O port
  */
#define inpb(port)            (*((volatile unsigned char *)(port)))

/**
  * @brief Set a 32-bit unsigned value to specified I/O port
  * @param[in] port Port address to set 32-bit data
  * @param[in] value Value to write to I/O port
  * @return  None
  * @note The output port must be 32-bit aligned
  */
#define outp32(port,value)    *((volatile unsigned int *)(port)) = (value)

/**
  * @brief Get a 32-bit unsigned value from specified I/O port
  * @param[in] port Port address to get 32-bit data from
  * @return  32-bit unsigned value stored in specified I/O port
  * @note The input port must be 32-bit aligned
  */
#define inp32(port)           (*((volatile unsigned int *)(port)))

/**
  * @brief Set a 16-bit unsigned value to specified I/O port
  * @param[in] port Port address to set 16-bit data
  * @param[in] value Value to write to I/O port
  * @return  None
  * @note The output port must be 16-bit aligned
  */
#define outp16(port,value)    *((volatile unsigned short *)(port)) = (value)

/**
  * @brief Get a 16-bit unsigned value from specified I/O port
  * @param[in] port Port address to get 16-bit data from
  * @return  16-bit unsigned value stored in specified I/O port
  * @note The input port must be 16-bit aligned
  */
#define inp16(port)           (*((volatile unsigned short *)(port)))

/**
  * @brief Set a 8-bit unsigned value to specified I/O port
  * @param[in] port Port address to set 8-bit data
  * @param[in] value Value to write to I/O port
  * @return  None
  */
#define outp8(port,value)     *((volatile unsigned char *)(port)) = (value)

/**
  * @brief Get a 8-bit unsigned value from specified I/O port
  * @param[in] port Port address to get 8-bit data from
  * @return  8-bit unsigned value stored in specified I/O port
  */
#define inp8(port)            (*((volatile unsigned char *)(port)))

/*@}*/ /* end of group IO_ROUTINE */




/** @addtogroup legacy_Constants Legacy Constants
  Legacy Constants
  @{
*/


#define E_SUCCESS   0
#ifndef NULL
#define NULL        0
#endif

#define TRUE        1
#define FALSE       0

#define ENABLE     1
#define DISABLE    0

/* Define one bit mask */
#define BIT0    0x00000001
#define BIT1    0x00000002
#define BIT2    0x00000004
#define BIT3    0x00000008
#define BIT4    0x00000010
#define BIT5    0x00000020
#define BIT6    0x00000040
#define BIT7    0x00000080
#define BIT8    0x00000100
#define BIT9    0x00000200
#define BIT10   0x00000400
#define BIT11   0x00000800
#define BIT12   0x00001000
#define BIT13   0x00002000
#define BIT14   0x00004000
#define BIT15   0x00008000
#define BIT16   0x00010000
#define BIT17   0x00020000
#define BIT18   0x00040000
#define BIT19   0x00080000
#define BIT20   0x00100000
#define BIT21   0x00200000
#define BIT22   0x00400000
#define BIT23   0x00800000
#define BIT24   0x01000000
#define BIT25   0x02000000
#define BIT26   0x04000000
#define BIT27   0x08000000
#define BIT28   0x10000000
#define BIT29   0x20000000
#define BIT30   0x40000000
#define BIT31   0x80000000

/* Byte Mask Definitions */
#define BYTE0_Msk               (0x000000FF)
#define BYTE1_Msk               (0x0000FF00)
#define BYTE2_Msk               (0x00FF0000)
#define BYTE3_Msk               (0xFF000000)

#define _GET_BYTE0(u32Param)    (((u32Param) & BYTE0_Msk)      )  /*!< Extract Byte 0 (Bit  0~ 7) from parameter u32Param */
#define _GET_BYTE1(u32Param)    (((u32Param) & BYTE1_Msk) >>  8)  /*!< Extract Byte 1 (Bit  8~15) from parameter u32Param */
#define _GET_BYTE2(u32Param)    (((u32Param) & BYTE2_Msk) >> 16)  /*!< Extract Byte 2 (Bit 16~23) from parameter u32Param */
#define _GET_BYTE3(u32Param)    (((u32Param) & BYTE3_Msk) >> 24)  /*!< Extract Byte 3 (Bit 24~31) from parameter u32Param */

/*@}*/ /* end of group legacy_Constants */


/******************************************************************************/
/*                         Peripheral header files                            */
/******************************************************************************/
#include "SYS.h"
#include "ADC.h"
#include "FMC.h"
#include "GPIO.h"
#include "I2C.h"
#include "PWM.h"
#include "SPI.h"
#include "TIMER.h"
#include "WDT.h"
#include "WWDT.h"
#include "UART.h"
#include "I2S.h"
#include "USBD.h"
#include "PDMA.h"
#include "PS2.h"
#include "CLK.h"
#include "CRC.h"
#endif

