/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-01-18     孙                 the first version
 */
#ifndef APPLICATIONS_INC_JY901S_DRV_H_
#define APPLICATIONS_INC_JY901S_DRV_H_

unsigned char jy901s_init(unsigned int stack_size,
        unsigned char  priority,
        unsigned int tick);
void calibration_acc (void);
void factory_data_reset(void);
void set_direction(unsigned char dir);
void led_change(unsigned char change);
void jy901s_detim_change(signed   int tim);
#define JY901_ADDR      0x50
/*reg*/
#define SAVE            0x00
#define CALSW           0x01
#define RSW             0x02
#define RRATE           0x03
#define BAUD            0x04
#define AXOFFSET        0x05
#define AYOFFSET        0x06
#define AZOFFSET        0x07
#define GXOFFSET        0x08
#define GYOFFSET        0x09
#define GZOFFSET        0x0a
#define HXOFFSET        0x0b
#define HYOFFSET        0x0c
#define HZOFFSET        0x0d
#define D0MODE          0x0e
#define D1MODE          0x0f
#define D2MODE          0x10
#define D3MODE          0x11
#define D0PWMH          0x12
#define D1PWMH          0x13
#define D2PWMH          0x14
#define D3PWMH          0x15
#define D0PWMT          0x16
#define D1PWMT          0x17
#define D2PWMT          0x18
#define D3PWMT          0x19
#define IICADDR         0x1a
#define LEDOFF          0x1b
#define GPSBAUD         0x1c
#define SLEEP           0x22
#define DIRECTION       0x23
#define GYRO            0x63

#define YYMM            0x30
#define DDHH            0x31
#define MMSS            0x32
#define MS              0x33
#define AX              0x34
#define AY              0x35
#define AZ              0x36
#define GX              0x37
#define GY              0x38
#define GZ              0x39
#define HX              0x3a
#define HY              0x3b
#define HZ              0x3c
#define Roll            0x3d
#define Pitch           0x3e
#define Yaw             0x3f
#define TEMP            0x40
#define D0Status        0x41
#define D1Status        0x42
#define D2Status        0x43
#define D3Status        0x44
#define PressureL       0x45
#define PressureH       0x46
#define HeightL         0x47
#define HeightH         0x48
#define LonL            0x49
#define LonH            0x4a
#define LatL            0x4b
#define LatH            0x4c
#define GPSHeight       0x4d
#define GPSYAW          0x4e
#define GPSVL           0x4f
#define GPSVH           0x50
/*reg end*/

#endif /* APPLICATIONS_INC_JY901S_DRV_H_ */
