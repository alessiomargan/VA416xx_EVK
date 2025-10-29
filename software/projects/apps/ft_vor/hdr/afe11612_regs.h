/**
 * @file afe11612_regs.h
 * @brief Auto-generated AFE11612 register definitions
 */

#ifndef __AFE11612_REGS_H__
#define __AFE11612_REGS_H__

#include <stdint.h>

#define AFE11612_REG_LT__TEMP                       0x00  /* R */
#define AFE11612_REG_D1__TEMP                       0x01  /* R */
#define AFE11612_REG_D2__TEMP                       0x02  /* R */
#define AFE11612_REG_TEMP__CONFIG                   0x0A  /* R/W */
#define AFE11612_REG_TEMP__CONV__RATE               0x0B  /* R/W */
#define AFE11612_REG_D1_N__ADJUST                   0x21  /* R/W */
#define AFE11612_REG_D2_N__ADJUST                   0x22  /* R/W */
#define AFE11612_REG_ADC_0                          0x23  /* R */
#define AFE11612_REG_ADC_1                          0x24  /* R */
#define AFE11612_REG_ADC_2                          0x25  /* R */
#define AFE11612_REG_ADC_3                          0x26  /* R */
#define AFE11612_REG_ADC_4                          0x27  /* R */
#define AFE11612_REG_ADC_5                          0x28  /* R */
#define AFE11612_REG_ADC_6                          0x29  /* R */
#define AFE11612_REG_ADC_7                          0x2A  /* R */
#define AFE11612_REG_ADC_8                          0x2B  /* R */
#define AFE11612_REG_ADC_9                          0x2C  /* R */
#define AFE11612_REG_ADC_10                         0x2D  /* R */
#define AFE11612_REG_ADC_11                         0x2E  /* R */
#define AFE11612_REG_ADC_12                         0x2F  /* R */
#define AFE11612_REG_ADC_13                         0x30  /* R */
#define AFE11612_REG_ADC_14                         0x31  /* R */
#define AFE11612_REG_ADC_15                         0x32  /* R */
#define AFE11612_REG_DAC_0                          0x33  /* R/W */
#define AFE11612_REG_DAC_1                          0x34  /* R/W */
#define AFE11612_REG_DAC_2                          0x35  /* R/W */
#define AFE11612_REG_DAC_3                          0x36  /* R/W */
#define AFE11612_REG_DAC_4                          0x37  /* R/W */
#define AFE11612_REG_DAC_5                          0x38  /* R/W */
#define AFE11612_REG_DAC_6                          0x39  /* R/W */
#define AFE11612_REG_DAC_7                          0x3A  /* R/W */
#define AFE11612_REG_DAC_8                          0x3B  /* R/W */
#define AFE11612_REG_DAC_9                          0x3C  /* R/W */
#define AFE11612_REG_DAC_10                         0x3D  /* R/W */
#define AFE11612_REG_DAC_11                         0x3E  /* R/W */
#define AFE11612_REG_DAC_0__CLR                     0x3F  /* R/W */
#define AFE11612_REG_DAC_1__CLR                     0x40  /* R/W */
#define AFE11612_REG_DAC_2__CLR                     0x41  /* R/W */
#define AFE11612_REG_DAC_3__CLR                     0x42  /* R/W */
#define AFE11612_REG_DAC_4__CLR                     0x43  /* R/W */
#define AFE11612_REG_DAC_5__CLR                     0x44  /* R/W */
#define AFE11612_REG_DAC_6__CLR                     0x45  /* R/W */
#define AFE11612_REG_DAC_7__CLR                     0x46  /* R/W */
#define AFE11612_REG_DAC_8__CLR                     0x47  /* R/W */
#define AFE11612_REG_DAC_9__CLR                     0x48  /* R/W */
#define AFE11612_REG_DAC_10__CLR                    0x49  /* R/W */
#define AFE11612_REG_DAC_11__CLR                    0x4A  /* R/W */
#define AFE11612_REG_GPIO                           0x4B  /* R/W */
#define AFE11612_REG_AFE__CONFIG_0                  0x4C  /* R/W */
#define AFE11612_REG_AFE__CONFIG_1                  0x4D  /* R/W */
#define AFE11612_REG_ALR_CTRL                       0x4E  /* R/W */
#define AFE11612_REG_STATUS                         0x4F  /* R */
#define AFE11612_REG_ADC_CH0                        0x50  /* R/W */
#define AFE11612_REG_ADC_CH1                        0x51  /* R/W */
#define AFE11612_REG_ADC_GAIN                       0x52  /* R/W */
#define AFE11612_REG_AUTO_DAC__CLR__SOURCE          0x53  /* R/W */
#define AFE11612_REG_AUTO_DAC__CLR_EN               0x54  /* R/W */
#define AFE11612_REG_SW_DAC__CLR                    0x55  /* R/W */
#define AFE11612_REG_HW_DAC__CLR_EN_0               0x56  /* R/W */
#define AFE11612_REG_HW_DAC__CLR_EN_1               0x57  /* R/W */
#define AFE11612_REG_DAC_CONFIG                     0x58  /* R/W */
#define AFE11612_REG_DAC_GAIN                       0x59  /* R/W */
#define AFE11612_REG_IN_0__HIGH__THRESHOLD          0x5A  /* R/W */
#define AFE11612_REG_IN_0__LOW__THRESHOLD           0x5B  /* R/W */
#define AFE11612_REG_IN_1__HIGH__THRESHOLD          0x5C  /* R/W */
#define AFE11612_REG_IN_1__LOW__THRESHOLD           0x5D  /* R/W */
#define AFE11612_REG_IN_2__HIGH__THRESHOLD          0x5E  /* R/W */
#define AFE11612_REG_IN_2__LOW__THRESHOLD           0x5F  /* R/W */
#define AFE11612_REG_IN_3__HIGH__THRESHOLD          0x60  /* R/W */
#define AFE11612_REG_IN_3__LOW__THRESHOLD           0x61  /* R/W */
#define AFE11612_REG_LT__HIGH__THRESHOLD            0x62  /* R/W */
#define AFE11612_REG_LT__LOW__THRESHOLD             0x63  /* R/W */
#define AFE11612_REG_D1__HIGH__THRESHOLD            0x64  /* R/W */
#define AFE11612_REG_D1__LOW__THRESHOLD             0x65  /* R/W */
#define AFE11612_REG_D2__HIGH__THRESHOLD            0x66  /* R/W */
#define AFE11612_REG_D2__LOW__THRESHOLD             0x67  /* R/W */
#define AFE11612_REG_HYST_0                         0x68  /* R/W */
#define AFE11612_REG_HYST_1                         0x69  /* R/W */
#define AFE11612_REG_HYST_2                         0x6A  /* R/W */
#define AFE11612_REG_PWR_DOWN                       0x6B  /* R/W */
#define AFE11612_REG_DEVICE_ID                      0x6C  /* R */
#define AFE11612_REG_SW_RST                         0x7C  /* R/W */

/* AFE__CONFIG_0 Register Bit Definitions */
#define AFE11612_ILDAC_BIT                          11    /* Internal LDAC bit position */
#define AFE11612_ILDAC_MASK                         (1 << AFE11612_ILDAC_BIT)  /* 0x0800 */

#endif /* __AFE11612_REGS_H__ */
