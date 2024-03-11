#include "GC2145.h"

#include <Wire.h>

#define DEBUG_CAMERA
//#define DEBUG_CAMERA_VERBOSE
#define DEBUG_FLEXIO

#define FLEXIO_TIMER_TRIGGER_SEL_PININPUT(x) ((uint32_t)(x) << 1U)


/*
 * This file is part of the OpenMV project.
 *
 * Copyright (c) 2013-2021 Ibrahim Abdelkader <iabdalkader@openmv.io>
 * Copyright (c) 2013-2021 Kwabena W. Agyeman <kwagyeman@openmv.io>
 *
 * This work is licensed under the MIT license, see the file LICENSE for details.
 *
 * GC2145 driver.
 */
 
/*
 * This file modified to work with Teensy 4.x with Flexio, GPIO, etc
 * https://github.com/openmv/openmv/blob/47e3a567d560c5340ce582bfe78199102fff5775/src/omv/common/sensor_utils.c#L46
 * some comes from Arduino_dvp
 */

#define	camAddress		        0x3C

#define BLANK_LINES             16
#define DUMMY_LINES             16

#define BLANK_COLUMNS           0
#define DUMMY_COLUMNS           8

#define SENSOR_WIDTH            1616
#define SENSOR_HEIGHT           1248

#define ACTIVE_SENSOR_WIDTH     (SENSOR_WIDTH - BLANK_COLUMNS - (2 * DUMMY_COLUMNS))
#define ACTIVE_SENSOR_HEIGHT    (SENSOR_HEIGHT - BLANK_LINES - (2 * DUMMY_LINES))

#define DUMMY_WIDTH_BUFFER      16
#define DUMMY_HEIGHT_BUFFER     8

static int16_t readout_x = 0;
static int16_t readout_y = 0;

static uint16_t readout_w = ACTIVE_SENSOR_WIDTH;
static uint16_t readout_h = ACTIVE_SENSOR_HEIGHT;

static bool fov_wide = false;

#define REG_AMODE1                      (0x17)
#define REG_AMODE1_DEF                  (0x14)
#define REG_AMODE1_SET_HMIRROR(r, x)    ((r & 0xFE) | ((x & 1) << 0))
#define REG_AMODE1_SET_VMIRROR(r, x)    ((r & 0xFD) | ((x & 1) << 1))

#define REG_OUTPUT_FMT                  (0x84)
#define REG_OUTPUT_FMT_RGB565           (0x06)
#define REG_OUTPUT_FMT_YCBYCR           (0x02)
#define REG_OUTPUT_FMT_BAYER            (0x17)
#define REG_OUTPUT_SET_FMT(r, x)        ((r & 0xE0) | (x))

#define REG_SYNC_MODE                   (0x86)
#define REG_SYNC_MODE_DEF               (0x03)
#define REG_SYNC_MODE_COL_SWITCH        (0x10)
#define REG_SYNC_MODE_ROW_SWITCH        (0x20)

// Sensor frame size/resolution table.
const int resolution[][2] = {
    {640,  480 },    /* VGA       */
    {160,  120 },    /* QQVGA     */
    {320,  240 },    /* QVGA      */
    {320,  320 },    /* 320x320   */
    {320,  240 },    /* QVGA      */
    {176,  144 },    /* QCIF      */
    {352,  288 },    /* CIF       */
    {0,    0   },
};

static const uint8_t default_regs[][2] = {
    {0xfe, 0xf0},
    {0xfe, 0xf0},
    {0xfe, 0xf0},

    {0xfc, 0x06},
    {0xf6, 0x00},

    {0xf7, 0x1d}, //37 //17 //37 //1d//05
    {0xf8, 0x83}, //87 //83 //82
    {0xfa, 0x00},
    {0xf9, 0xfe}, //ff
    {0xfd, 0x00},
    {0xc2, 0x00},
    {0xf2, 0x0f},
//////////////////////////////////////////////////////
////////////////////  Analog & Cisctl ////////////////
//////////////////////////////////////////////////////
    {0xfe, 0x00},

    {0x03, 0x04}, //exp time
    {0x04, 0x62}, //exp time

    {0x05, 0x01}, //00 //hb[11:8]
    {0x06, 0x3b}, //0b //hb

    {0x09, 0x00}, //row start
    {0x0a, 0x00}, //
    {0x0b, 0x00}, //col start
    {0x0c, 0x00},
    {0x0d, 0x04}, //height
    {0x0e, 0xc0},
    {0x0f, 0x06}, //width
    {0x10, 0x52},

    {0x12, 0x2e}, //sh_delay 太短 YUV出图异常
    {0x17, 0x14}, //CISCTL Mode1 [1:0]mirror flip
    {0x18, 0x22}, //sdark mode
    {0x19, 0x0f}, // AD pipe number
    {0x1a, 0x01}, //AD manual switch mode

    {0x1b, 0x4b}, //48 restg Width,SH width
    {0x1c, 0x07}, //06  帧率快后，横条纹 //12 //TX Width,Space Width
    {0x1d, 0x10}, //double reset
    {0x1e, 0x88}, //90//98 //fix  竖线//Analog Mode1,TX high,Coln_r
    {0x1f, 0x78}, //78 //38 //18 //Analog Mode2,txlow
    {0x20, 0x03}, //07 //Analog Mode3,comv,ad_clk mode
    {0x21, 0x40}, //10//20//40 //fix 灯管横条纹
    {0x22, 0xa0}, //d0//f0 //a2 //Vref vpix  FPN严重
    {0x24, 0x1e},
    {0x25, 0x01}, //col sel
    {0x26, 0x10}, //Analog PGA gain1
    {0x2d, 0x60}, //40//40 //txl drv mode
    {0x30, 0x01}, //Analog Mode4
    {0x31, 0x90}, //b0//70 // Analog Mode7 [7:5]rsgh_r灯管横条纹[4:3]isp_g
    {0x33, 0x06}, //03//02//01 //EQ_hstart_width
    {0x34, 0x01},
//
///////////////////////////////////////////////////
////////////////////  ISP reg  //////////////////////
//////////////////////////////////////////////////////
    {0x80, 0xff}, //outdoor gamma_en, GAMMA_en, CC_en, EE_en, INTP_en, DN_en, DD_en,LSC_en
    {0x81, 0x24}, //26//24 //BLK dither mode, ll_y_en ,skin_en, edge SA, new_skin_mode, autogray_en,ll_gamma_en,BFF test image
    {0x82, 0xfa}, //FA //auto_SA, auto_EE, auto_DN, auto_DD, auto_LSC, ABS_en, AWB_en, NA
    {0x83, 0x00}, //special_effect
    {0x84, 0x02}, //output format
    {0x86, 0x03}, //c2 //46 //c2 //sync mode
    {0x88, 0x03}, //[1]ctl_auto_gating [0]out_auto_gating
    {0x89, 0x03}, //bypass disable
    {0x85, 0x30}, //60//frame start cut
    {0x8a, 0x00}, //ISP_quiet_mode,close aaa pclk,BLK gate mode,exception,close first pipe clock,close dndd clock,close intp clock,DIV_gatedclk_en
    {0x8b, 0x00}, //[7:6]BFF_gate_mode,[5]BLK switch gain,[4]protect exp,[3:2]pipe gate mode,[1]not split sram,[0]dark current update

    {0xb0, 0x55}, //60 //global gain
    {0xc3, 0x00}, //[7:4]auto_exp_gamma_th1[11:8],[3:0]auto_exp_gamma_th2[11:8]
    {0xc4, 0x80}, //auto_exp_gamma_th1[7:0] into
    {0xc5, 0x90}, //auto_exp_gamma_th2[7:0] out //outdoor gamma
    {0xc6, 0x38}, //auto_gamma_th1
    {0xc7, 0x40}, //auto_gamma_th2

    {0xec, 0x06}, //measure window
    {0xed, 0x04},
    {0xee, 0x60}, //16  col
    {0xef, 0x90}, //8  row

    {0xb6, 0x01}, //[0]aec en

    {0x90, 0x01}, //crop
    {0x91, 0x00},
    {0x92, 0x00},
    {0x93, 0x00},
    {0x94, 0x00}, //08
    {0x95, 0x04},
    {0x96, 0xb0},
    {0x97, 0x06},
    {0x98, 0x40},

///////////////////////////////////////////////
///////////  BLK ////////////////////////
///////////////////////////////////////////////
    {0x18, 0x02},
    {0x40, 0x42}, //2b //27
    {0x41, 0x00}, //80 //dark row sel
    {0x43, 0x54}, //[7:4]BLK start not smooth  [3:0]output start frame

    {0x5e, 0x00}, //00//10 //18
    {0x5f, 0x00}, //00//10 //18
    {0x60, 0x00}, //00//10 //18
    {0x61, 0x00}, //00///10 //18
    {0x62, 0x00}, //00//10 //18
    {0x63, 0x00}, //00//10 //18
    {0x64, 0x00}, //00/10 //18
    {0x65, 0x00}, //00//10 //18
    {0x66, 0x20}, //1e
    {0x67, 0x20}, //1e
    {0x68, 0x20}, //1e
    {0x69, 0x20}, //1e


    {0x76, 0x00}, //0f

    {0x6a, 0x00}, //06
    {0x6b, 0x00}, //06
    {0x6c, 0x3e}, //06
    {0x6d, 0x3e}, //06
    {0x6e, 0x3f}, //06
    {0x6f, 0x3f}, //06
    {0x70, 0x00}, //06
    {0x71, 0x00}, //06 //manual offset

    {0x76, 0x00}, //1f//add offset
    {0x72, 0xf0}, //[7:4]BLK DD th [3:0]BLK various th
    {0x7e, 0x3c}, //ndark
    {0x7f, 0x00},

    {0xfe, 0x02},
    {0x48, 0x15},
    {0x49, 0x00}, //04//04 //ASDE OFFSET SLOPE
    {0x4b, 0x0b}, //ASDE y OFFSET SLOPE
    {0xfe, 0x00},

///////////////////////////////////////////////
/////////// AEC ////////////////////////
///////////////////////////////////////////////
    {0xfe, 0x01},

    {0x01, 0x04}, //AEC X1
    {0x02, 0xc0}, //AEC X2
    {0x03, 0x04}, //AEC Y1
    {0x04, 0x90}, //AEC Y2
    {0x05, 0x30}, //20 //AEC center X1
    {0x06, 0x90}, //40 //AEC center X2
    {0x07, 0x20}, //30 //AEC center Y1
    {0x08, 0x70}, //60 //AEC center Y2

    {0x09, 0x00}, //AEC show mode
    {0x0a, 0xc2}, //[7]col gain enable
    {0x0b, 0x11}, //AEC every N
    {0x0c, 0x10}, //AEC_mode3 center weight
    {0x13, 0x40}, //2a //AEC Y target
    {0x17, 0x00}, //AEC ignore mode
    {0x1c, 0x11}, //
    {0x1e, 0x61}, //
    {0x1f, 0x30}, //40//50 //max pre gain
    {0x20, 0x40}, //60//40 //max post gain
    {0x22, 0x80}, //AEC outdoor THD
    {0x23, 0x20}, //target_Y_low_limit
    {0xfe, 0x02},
    {0x0f, 0x04}, //05
    {0xfe, 0x01},

    {0x12, 0x35}, //35 //[5:4]group_size [3]slope_disable [2]outdoor_enable [0]histogram_enable
    {0x15, 0x50}, //target_Y_high_limit
    {0x10, 0x31}, //num_thd_high
    {0x3e, 0x28}, //num_thd_low
    {0x3f, 0xe0}, //luma_thd
    {0x40, 0x20}, //luma_slope
    {0x41, 0x0f}, //color_diff

    {0xfe, 0x02},
    {0x0f, 0x05}, //max_col_level
///////////////////////////
////// INTPEE /////////////
///////////////////////////
    {0xfe, 0x02}, //page2
    {0x90, 0x6c}, //ac //eeintp mode1
    {0x91, 0x03}, //02 ////eeintp mode2
    {0x92, 0xc8}, //44 //low criteria for direction
    {0x94, 0x66},
    {0x95, 0xb5},
    {0x97, 0x64}, //78 ////edge effect
    {0xa2, 0x11}, //fix direction
    {0xfe, 0x00},

/////////////////////////////
//////// DNDD///////////////
/////////////////////////////
    {0xfe, 0x02},
    {0x80, 0xc1}, //c1 //[7]share mode [6]skin mode  [5]is 5x5 mode [1:0]noise value select 0:2  1:2.5  2:3  3:4
    {0x81, 0x08}, //
    {0x82, 0x08}, //signal a 0.6
    {0x83, 0x08}, //04 //signal b 2.5

    {0x84, 0x0a}, //10 //05 dark_DD_TH
    {0x86, 0xf0}, //a0 Y_value_dd_th2
    {0x87, 0x50}, //90 Y_value_dd_th3
    {0x88, 0x15}, //60 Y_value_dd_th4

    {0x89, 0x50}, //80  // asde th2
    {0x8a, 0x30}, //60  // asde th3
    {0x8b, 0x10}, //30  // asde th4

/////////////////////////////////////////////////
///////////// ASDE ////////////////////////
/////////////////////////////////////////////////
    {0xfe, 0x01}, //page 1
    {0x21, 0x14}, //luma_value_div_sel(分频，与0xef呈2倍关系，增大1，0xef的值减小1倍)
//ff  ef  luma_value read_only

    {0xfe, 0x02}, //page2
    {0xa3, 0x40}, //ASDE_low_luma_value_LSC_th_H
    {0xa4, 0x20}, //ASDE_low_luma_value_LSC_th_L

    {0xa5, 0x40}, //80 //ASDE_LSC_gain_dec_slope_H
    {0xa6, 0x80}, // 80 //ASDE_LSC_gain_dec_slope_L
//ff  a7  ASDE_LSC_gain_dec  //read only

    {0xab, 0x40}, //50 //ASDE_low_luma_value_OT_th

    {0xae, 0x0c}, //[3]EE1_effect_inc_or_dec_high,[2]EE2_effect_inc_or_dec_high,
    //[1]EE1_effect_inc_or_dec_low,[0]EE2_effect_inc_or_dec_low,  1:inc  0:dec

    {0xb3, 0x34}, //44 //ASDE_EE1_effect_slope_low,ASDE_EE2_effect_slope_low
    {0xb4, 0x44}, //12 //ASDE_EE1_effect_slope_high,ASDE_EE2_effect_slope_high

    {0xb6, 0x38}, //40//40 //ASDE_auto_saturation_dec_slope
    {0xb7, 0x02}, //04 //ASDE_sub_saturation_slope
    {0xb9, 0x30}, //[7:0]ASDE_auto_saturation_low_limit
    {0x3c, 0x08}, //[3:0]auto gray_dec_slope
    {0x3d, 0x30}, //[7:0]auto gray_dec_th


    {0x4b, 0x0d}, //y offset slope
    {0x4c, 0x20}, //y offset limit

    {0xfe, 0x00},
//
///////////////////gamma1////////////////////
////Gamma
    {0xfe, 0x02},
    {0x10, 0x10},
    {0x11, 0x15},
    {0x12, 0x1a},
    {0x13, 0x1f},
    {0x14, 0x2c},
    {0x15, 0x39},
    {0x16, 0x45},
    {0x17, 0x54},
    {0x18, 0x69},
    {0x19, 0x7d},
    {0x1a, 0x8f},
    {0x1b, 0x9d},
    {0x1c, 0xa9},
    {0x1d, 0xbd},
    {0x1e, 0xcd},
    {0x1f, 0xd9},
    {0x20, 0xe3},
    {0x21, 0xea},
    {0x22, 0xef},
    {0x23, 0xf5},
    {0x24, 0xf9},
    {0x25, 0xff},

/////auto gamma/////
    {0xfe, 0x02},
    {0x26, 0x0f},
    {0x27, 0x14},
    {0x28, 0x19},
    {0x29, 0x1e},
    {0x2a, 0x27},
    {0x2b, 0x33},
    {0x2c, 0x3b},
    {0x2d, 0x45},
    {0x2e, 0x59},
    {0x2f, 0x69},
    {0x30, 0x7c},
    {0x31, 0x89},
    {0x32, 0x98},
    {0x33, 0xae},
    {0x34, 0xc0},
    {0x35, 0xcf},
    {0x36, 0xda},
    {0x37, 0xe2},
    {0x38, 0xe9},
    {0x39, 0xf3},
    {0x3a, 0xf9},
    {0x3b, 0xff},

///////////////////////////////////////////////
///////////   YCP       ///////////////////////
///////////////////////////////////////////////
    {0xfe, 0x02},
    {0xd1, 0x30}, //32 //
    {0xd2, 0x30}, //32 //
    {0xd3, 0x45},
    {0xdd, 0x14}, //edge sa
    {0xde, 0x86}, //asde auto gray
    {0xed, 0x01}, //
    {0xee, 0x28},
    {0xef, 0x30},
    {0xd8, 0xd8}, //autogray protecy

////////////////////////////
//////// LSC  0.8///////////////
////////////////////////////
    {0xfe, 0x01},
    {0xa1, 0x80}, // center_row
    {0xa2, 0x80}, // center_col
    {0xa4, 0x00}, // sign of b1
    {0xa5, 0x00}, // sign of b1
    {0xa6, 0x70}, // sign of b4
    {0xa7, 0x00}, // sign of b4
    {0xa8, 0x77}, // sign of b22
    {0xa9, 0x77}, // sign of b22
    {0xaa, 0x1f}, // Q1_b1 of R
    {0xab, 0x0d}, // Q1_b1 of G
    {0xac, 0x19}, // Q1_b1 of B
    {0xad, 0x24}, // Q2_b1 of R
    {0xae, 0x0e}, // Q2_b1 of G
    {0xaf, 0x1d}, // Q2_b1 of B
    {0xb0, 0x12}, // Q3_b1 of R
    {0xb1, 0x0c}, // Q3_b1 of G
    {0xb2, 0x06}, // Q3_b1 of B
    {0xb3, 0x13}, // Q4_b1 of R
    {0xb4, 0x10}, // Q4_b1 of G
    {0xb5, 0x0c}, // Q4_b1 of B
    {0xb6, 0x6a}, // right_b2 of R
    {0xb7, 0x46}, // right_b2 of G
    {0xb8, 0x40}, // right_b2 of B
    {0xb9, 0x0b}, // right_b4 of R
    {0xba, 0x04}, // right_b4 of G
    {0xbb, 0x00}, // right_b4 of B
    {0xbc, 0x53}, // left_b2 of R
    {0xbd, 0x37}, // left_b2 of G
    {0xbe, 0x2d}, // left_b2 of B
    {0xbf, 0x0a}, // left_b4 of R
    {0xc0, 0x0a}, // left_b4 of G
    {0xc1, 0x14}, // left_b4 of B
    {0xc2, 0x34}, // up_b2 of R
    {0xc3, 0x22}, // up_b2 of G
    {0xc4, 0x18}, // up_b2 of B
    {0xc5, 0x23}, // up_b4 of R
    {0xc6, 0x0f}, // up_b4 of G
    {0xc7, 0x3c}, // up_b4 of B
    {0xc8, 0x20}, // down_b2 of R
    {0xc9, 0x1f}, // down_b2 of G
    {0xca, 0x17}, // down_b2 of B
    {0xcb, 0x2d}, // down_b4 of R
    {0xcc, 0x12}, // down_b4 of G
    {0xcd, 0x20}, // down_b4 of B
    {0xd0, 0x61}, // right_up_b22 of R
    {0xd1, 0x2f}, // right_up_b22 of G
    {0xd2, 0x39}, // right_up_b22 of B
    {0xd3, 0x45}, // right_down_b22 of R
    {0xd4, 0x2c}, // right_down_b22 of G
    {0xd5, 0x21}, // right_down_b22 of B
    {0xd6, 0x64}, // left_up_b22 of R
    {0xd7, 0x2d}, // left_up_b22 of G
    {0xd8, 0x30}, // left_up_b22 of B
    {0xd9, 0x42}, // left_down_b22 of R
    {0xda, 0x27}, // left_down_b22 of G
    {0xdb, 0x13}, // left_down_b22 of B
    {0xfe, 0x00},

/////////////////////////////////////////////////
/////////////    AWB     ////////////////////////
/////////////////////////////////////////////////
    {0xfe, 0x01},

    {0x4f, 0x00},
    {0x4f, 0x00},
    {0x4b, 0x01},
    {0x4f, 0x00},


    {0x4c, 0x01},
    {0x4d, 0x6f},
    {0x4e, 0x02},
    {0x4c, 0x01},
    {0x4d, 0x70},

    {0x4e, 0x02},
    {0x4c, 0x01},
    {0x4d, 0x8f},
    {0x4e, 0x02},

    {0x4c, 0x01},
    {0x4d, 0x90},
    {0x4e, 0x02}, //light


    {0x4c, 0x01},
    {0x4d, 0xed},
    {0x4e, 0x33}, //light
    {0x4c, 0x01},
    {0x4d, 0xcd},
    {0x4e, 0x33}, //light
    {0x4c, 0x01},
    {0x4d, 0xec},
    {0x4e, 0x03}, //light

    {0x4c, 0x01},
    {0x4d, 0x6c},
    {0x4e, 0x03},
    {0x4c, 0x01},
    {0x4d, 0x6d},
    {0x4e, 0x03},
    {0x4c, 0x01},
    {0x4d, 0x6e},
    {0x4e, 0x03},
    {0x4c, 0x01},
    {0x4d, 0x8c},
    {0x4e, 0x03},
    {0x4c, 0x01},
    {0x4d, 0x8d},
    {0x4e, 0x03},
    {0x4c, 0x01},
    {0x4d, 0x8e},
    {0x4e, 0x03},
    {0x4c, 0x01},
    {0x4d, 0xab},
    {0x4e, 0x03},
    {0x4c, 0x01},
    {0x4d, 0xac},
    {0x4e, 0x03},
    {0x4c, 0x01},
    {0x4d, 0xad},
    {0x4e, 0x03},
    {0x4c, 0x01},
    {0x4d, 0xae},
    {0x4e, 0x03},
    {0x4c, 0x01},
    {0x4d, 0xcb},
    {0x4e, 0x03},

    {0x4c, 0x01},
    {0x4d, 0xcc},
    {0x4e, 0x03},
    {0x4c, 0x01},
    {0x4d, 0xce},
    {0x4e, 0x03},
    {0x4c, 0x01},
    {0x4d, 0xeb},
    {0x4e, 0x03},
    {0x4c, 0x01},
    {0x4d, 0xec},
    {0x4e, 0x03},
    {0x4c, 0x01},
    {0x4d, 0xee},
    {0x4e, 0x03},
    {0x4c, 0x02},
    {0x4d, 0x0c},
    {0x4e, 0x03},
    {0x4c, 0x02},
    {0x4d, 0x0d},
    {0x4e, 0x03},
    {0x4c, 0x01},
    {0x4d, 0xea},
    {0x4e, 0x03},
    {0x4c, 0x01},
    {0x4d, 0xaf},
    {0x4e, 0x03}, //dark
    {0x4c, 0x01},
    {0x4d, 0xcf},
    {0x4e, 0x03}, //dark

    {0x4c, 0x01},
    {0x4d, 0xca},
    {0x4e, 0x04}, //light
    {0x4c, 0x02},
    {0x4d, 0x0b},
    {0x4e, 0x05}, //light
    {0x4c, 0x02},
    {0x4d, 0xc8},
    {0x4e, 0x06}, //light 100lux
    {0x4c, 0x02},
    {0x4d, 0xa8},

    {0x4e, 0x06}, //light
    {0x4c, 0x02},
    {0x4d, 0xa9},
    {0x4e, 0x06}, //light


    {0x4c, 0x02},
    {0x4d, 0x89},
    {0x4e, 0x06}, //400lux
    {0x4c, 0x02},
    {0x4d, 0x69},
    {0x4e, 0x06}, //f12
    {0x4c, 0x02},
    {0x4d, 0x6a},
    {0x4e, 0x06}, //f12
    {0x4c, 0x02},
    {0x4d, 0xc7},
    {0x4e, 0x07},
    {0x4c, 0x02},
    {0x4d, 0xe7},
    {0x4e, 0x07}, //100lux
    {0x4c, 0x03},
    {0x4d, 0x07},
    {0x4e, 0x07}, //light

    {0x4c, 0x02},
    {0x4d, 0xe8},
    {0x4e, 0x07},
    {0x4c, 0x02},
    {0x4d, 0xe9},
    {0x4e, 0x07},
    {0x4c, 0x03},
    {0x4d, 0x08},
    {0x4e, 0x07},
    {0x4c, 0x03},
    {0x4d, 0x09},
    {0x4e, 0x07},
    {0x4c, 0x03},
    {0x4d, 0x27},
    {0x4e, 0x07},
    {0x4c, 0x03},
    {0x4d, 0x28},
    {0x4e, 0x07},
    {0x4c, 0x03},
    {0x4d, 0x29},
    {0x4e, 0x07},
    {0x4c, 0x03},
    {0x4d, 0x47},
    {0x4e, 0x07},
    {0x4c, 0x03},
    {0x4d, 0x48},
    {0x4e, 0x07},
    {0x4c, 0x03},
    {0x4d, 0x49},
    {0x4e, 0x07},
    {0x4c, 0x03},
    {0x4d, 0x67},
    {0x4e, 0x07},
    {0x4c, 0x03},
    {0x4d, 0x68},
    {0x4e, 0x07},
    {0x4c, 0x03},
    {0x4d, 0x69},
    {0x4e, 0x07},

    {0x4f, 0x01},
    {0xfe, 0x01},
    {0x50, 0x80}, //AWB_PRE_mode
    {0x51, 0xa8}, //AWB_pre_THD_min[7:0]
    {0x52, 0x57}, //AWB_pre_THD_min[15:8] Dominiate luma 0.25=639c 0.22=57a8
    {0x53, 0x38}, //AWB_pre_THD_min_MIX[7:0]
    {0x54, 0xc7}, //AWB_pre_THD_min_MIX[15:8] Mix luma 0.5

    {0x56, 0x0e}, //AWB_tone mode
    {0x58, 0x08}, //AWB_C_num_sel,AWB_D_num_sel
    {0x5b, 0x00}, //AWB_mix_mode

    {0x5c, 0x74}, //green_num0[7:0]
    {0x5d, 0x8b}, //green_num0[15:8] 0.35

    {0x61, 0xd3}, //R2G_stand0
    {0x62, 0xb5}, //B2G_stand0
    {0x63, 0x00}, //88//a4 //AWB gray mode [7]enable
    {0x65, 0x04}, //AWB margin

    {0x67, 0xb2}, //R2G_stand3[7:0]  FF/CWF
    {0x68, 0xac}, //B2G_stand3[7:0]
    {0x69, 0x00}, //R2G_stand4[9:8] B2G_stand4[9:8] R2G_stand3[9:8] B2G_stand3[9:8]
    {0x6a, 0xb2}, //R2G_stand4[7:0]  TL84/TL84&CWF
    {0x6b, 0xac}, //B2G_stand4[7:0]
    {0x6c, 0xb2}, //R2G_stand5[7:0]  A
    {0x6d, 0xac}, //B2G_stand5[7:0]
    {0x6e, 0x40}, //AWB_skin_weight R2G_stand5[9:8] B2G_stand5[9:8]
    {0x6f, 0x18}, //AWB_indoor_THD (0x21=17 caculate)
    {0x73, 0x00}, //AWB_indoor_mode

    {0x70, 0x10}, //AWB low luma TH
    {0x71, 0xe8}, //AWB outdoor TH
    {0x72, 0xc0}, //outdoor mode
    {0x74, 0x01}, //[2:0]AWB skip mode 2x2,4x4,4x8,8x8
    {0x75, 0x01}, //[1:0]AWB_every_N
    {0x7f, 0x08}, //[3]gray world frame start

    {0x76, 0x70}, //R limit
    {0x77, 0x58}, //G limit
    {0x78, 0xa0}, //d8 //B limit

    {0xfe, 0x00},
//
//////////////////////////////////////////
///////////  CC   ////////////////////////
//////////////////////////////////////////
    {0xfe, 0x02},

    {0xc0, 0x01}, //[5:4] CC mode [0]CCT enable

    {0xC1, 0x50}, //D50/D65
    {0xc2, 0xF9},
    {0xc3, 0x00}, //0
    {0xc4, 0xe8}, //e0
    {0xc5, 0x48},
    {0xc6, 0xf0},


    {0xC7, 0x50},
    {0xc8, 0xf2},
    {0xc9, 0x00},
    {0xcA, 0xE0},
    {0xcB, 0x45},
    {0xcC, 0xec},

    {0xCd, 0x45},
    {0xce, 0xf0},
    {0xcf, 0x00},
    {0xe3, 0xf0},
    {0xe4, 0x45},
    {0xe5, 0xe8},


    {0xfe, 0x00},

    {0xf2, 0x0f},


//////////////frame rate   50Hz
    {0xfe, 0x00},

    {0xf7, 0x1d},
    {0xf8, 0x84},
    {0xfa, 0x00},

    {0x05, 0x01}, //hb
    {0x06, 0x3b},
    {0x07, 0x01}, //Vb
    {0x08, 0x0b},

    {0xfe, 0x01},
    {0x25, 0x01},
    {0x26, 0x32}, //step
    {0x27, 0x03}, //8.15fps
    {0x28, 0x96},
    {0x29, 0x03}, //8.15fps
    {0x2a, 0x96},
    {0x2b, 0x03}, //8.15fps
    {0x2c, 0x96},
    {0x2d, 0x04}, //8.15fps
    {0x2e, 0x62},
    {0x3c, 0x00},
    {0xfe, 0x00},

/////////dark  sun//////
    {0xfe, 0x00},
    {0x18, 0x22},
    {0xfe, 0x02},
    {0x40, 0xbf},
    {0x46, 0xcf},
    {0xfe, 0x00},

    {0xfe, 0x00},

    {0xf7, 0x1d},
    {0xf8, 0x84},
    {0xfa, 0x10},

    {0x05, 0x01}, //hb
    {0x06, 0x18},
    {0x07, 0x00}, //Vb
    {0x08, 0x2e},

    {0xfe, 0x01},
    {0x25, 0x00},
    {0x26, 0xa2}, //step
    {0x27, 0x01},
    {0x28, 0xe6},
    {0x29, 0x01},
    {0x2a, 0xe6},
    {0x2b, 0x01},
    {0x2c, 0xe6},
    {0x2d, 0x04}, // AEC_exp_level4[12:8]
    {0x2e, 0x62}, // AEC_exp_level4[7:0]
    {0x3c, 0x00},
    {0xfe, 0x00},

    {0x09, 0x01}, //row start
    {0x0a, 0xd0}, //
    {0x0b, 0x02}, //col start
    {0x0c, 0x70},
    {0x0d, 0x01}, //height
    {0x0e, 0x00},
    {0x0f, 0x01}, //width
    {0x10, 0x50},

    {0x90, 0x01}, //crop
    {0x91, 0x00},
    {0x92, 0x00},
    {0x93, 0x00},
    {0x94, 0x00},
    {0x95, 0x00},
    {0x96, 0xf0},
    {0x97, 0x01},
    {0x98, 0x40},


    {0x00, 0x00},
};
    
// Constructor
const int GC2145_D[8] = {
  GC2145_D0, GC2145_D1, GC2145_D2, GC2145_D3, GC2145_D4, GC2145_D5, GC2145_D6, GC2145_D7
};

GC2145::GC2145() :
  _GC2145(NULL),
  _frame_buffer_pointer(NULL)
{
  //setPins(GC2145_VSYNC, GC2145_HREF, GC2145_PLK, GC2145_XCLK, GC2145_RST, GC2145_D);
  setPins(GC2145_XCLK, GC2145_PLK, GC2145_VSYNC, GC2145_HREF, GC2145_RST,
                     GC2145_D0, GC2145_D1, GC2145_D2, GC2145_D3, GC2145_D4, GC2145_D5, GC2145_D6, GC2145_D7, Wire);



}

void GC2145::beginXClk()
{
  // Generates 8 MHz signal using PWM... Will speed up.
#if defined(__IMXRT1062__)  // Teensy 4.x
  analogWriteFrequency(_xclkPin, _xclk_freq);
  analogWrite(_xclkPin, 127); delay(100); // 9mhz works, but try to reduce to debug timings with logic analyzer

#else
  // Generates 16 MHz signal using I2S peripheral
  NRF_I2S->CONFIG.MCKEN = (I2S_CONFIG_MCKEN_MCKEN_ENABLE << I2S_CONFIG_MCKEN_MCKEN_Pos);
  NRF_I2S->CONFIG.MCKFREQ = I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV2  << I2S_CONFIG_MCKFREQ_MCKFREQ_Pos;
  NRF_I2S->CONFIG.MODE = I2S_CONFIG_MODE_MODE_MASTER << I2S_CONFIG_MODE_MODE_Pos;

  NRF_I2S->PSEL.MCK = (digitalPinToPinName(_xclkPin) << I2S_PSEL_MCK_PIN_Pos);

  NRF_I2S->ENABLE = 1;
  NRF_I2S->TASKS_START = 1;
#endif
}

void GC2145::endXClk()
{
#if defined(__IMXRT1062__)  // Teensy 4.x
  analogWrite((_xclk_freq * 1000000), 0);
#else
  NRF_I2S->TASKS_STOP = 1;
#endif
}

void GC2145::end()
{
  endXClk();

  pinMode(_xclkPin, INPUT);

  _wire->end();
  
}

int16_t GC2145::width()
{
  return _width;
}

int16_t GC2145::height()
{
  return _height;
}

int GC2145::bitsPerPixel() const
{
  if (_grayscale) {
    return 8;
  } else {
    return _bytesPerPixel * 8;
  }
}

int GC2145::bytesPerPixel() const
{
  if (_grayscale) {
    return 1;
  } else {
    return _bytesPerPixel;
  }
}


//void OV767X::setPins(int vsync, int href, int pclk, int xclk, int rst, const int dpins[8])
void GC2145::setPins(uint8_t mclk_pin, uint8_t pclk_pin, uint8_t vsync_pin, uint8_t hsync_pin, uint8_t en_pin,
                     uint8_t g0, uint8_t g1, uint8_t g2, uint8_t g3, uint8_t g4, uint8_t g5, uint8_t g6, uint8_t g7, TwoWire &wire)
{
  _vsyncPin = vsync_pin;
  _hrefPin = hsync_pin;
  _pclkPin = pclk_pin;
  _xclkPin = mclk_pin;
  _rst = en_pin;
  _dPins[0] = g0;
  _dPins[1] = g1;
  _dPins[2] = g2;
  _dPins[3] = g3;
  _dPins[4] = g4;
  _dPins[5] = g5;
  _dPins[6] = g6;
  _dPins[7] = g7;
  
  _wire = &wire;

  //memcpy(_dPins, dpins, sizeof(_dPins));
}

//*****************************************************************************
uint16_t GC2145::getModelid()
{
    uint8_t Data;
    uint16_t MID = 0x0000;
cameraWriteRegister(0xFE, 0x00);
    Data = cameraReadRegister(0XF0);
       MID = (Data << 8);
Serial.printf("F0: 0x%x\n", Data);
    Data = cameraReadRegister(0XF1);
        MID |= Data;
Serial.printf("F1: 0x%x\n", Data);
    return MID;
}

//bool GC2145::begin(framesize_t resolution, int format, bool use_gpio)
bool GC2145::begin_omnivision(framesize_t resolution, pixformat_t format, int fps, bool use_gpio)
{
    //_wire = &Wire;
    _wire->begin();

  _use_gpio = use_gpio;
  // BUGBUG::: see where frame is
  pinMode(49, OUTPUT);
    
  Serial.println("GC2145::begin");
   
  _grayscale = false;
  switch (format) {
  case YUV422:
  case RGB565:
    _bytesPerPixel = 2;
    break;
  case GRAYSCALE:
    format = YUV422;    // We use YUV422 but discard U and V bytes
    _bytesPerPixel = 2; // 2 input bytes per pixel of which 1 is discarded
    _grayscale = true;
    break;
  default:
    return 0;
  }
  
  pinMode(_vsyncPin, INPUT_PULLDOWN);
//  const struct digital_pin_bitband_and_config_table_struct *p;
//  p = digital_pin_to_info_PGM + _vsyncPin;
//  *(p->pad) = IOMUXC_PAD_DSE(7) | IOMUXC_PAD_HYS;  // See if I turn on HYS...
  pinMode(_hrefPin, INPUT);
  pinMode(_pclkPin, INPUT_PULLDOWN);
  pinMode(_xclkPin, OUTPUT);
#ifdef DEBUG_CAMERA
  Serial.printf("  VS=%d, HR=%d, PC=%d XC=%d\n", _vsyncPin, _hrefPin, _pclkPin, _xclkPin);
  Serial.printf("  RST=%d\n", _rst);
#endif

  for (int i = 0; i < 8; i++) {
    pinMode(_dPins[i], INPUT);
    Serial.printf("  _dpins(%d)=%d\n", i, _dPins[i]);
  }

  _vsyncPort = portInputRegister(digitalPinToPort(_vsyncPin));
  _vsyncMask = digitalPinToBitMask(_vsyncPin);
  _hrefPort = portInputRegister(digitalPinToPort(_hrefPin));
  _hrefMask = digitalPinToBitMask(_hrefPin);
  _pclkPort = portInputRegister(digitalPinToPort(_pclkPin));
  _pclkMask = digitalPinToBitMask(_pclkPin);

    beginXClk();
    delay(100);

  if(_rst != 0xFF){
    pinMode(_rst, OUTPUT);
    digitalWriteFast(_rst, LOW);      /* Reset */
    for(volatile uint32_t i=0; i<100000; i++)
    {}
    digitalWriteFast(_rst, HIGH);     /* Normal mode. */
    for(volatile uint32_t i=0; i<100000; i++)
    {}
  }
  
  reset();
  
  Serial.println("\nSetting Format");
  setPixelFormat(format);
  Serial.println("\nSetting FrameSize");
  setFramesize(resolution);
  //printRegisters();

//flexIO/DMA
    if(!_use_gpio) {
        flexio_configure();
        setVSyncISRPriority(102);
        setDMACompleteISRPriority(192);
    } else {
        setVSyncISRPriority(102);
        setDMACompleteISRPriority(192);
    }

  return 1;
}

int GC2145::reset() {
    int ret = 0;

    readout_x = 0;
    readout_y = 0;

    readout_w = ACTIVE_SENSOR_WIDTH;
    readout_h = ACTIVE_SENSOR_HEIGHT;

    fov_wide = false;

    for (int i=0; default_regs[i][0] && ret == 0; i++) {
        ret |=  cameraWriteRegister(default_regs[i][0], default_regs[i][1]);
    }

    // Delay 10 ms
    delay(10);

    return ret;
}

int GC2145::sleep(int enable) {
    int ret = 0;

    if (enable) {
        ret |= cameraWriteRegister(0xF2, 0x0);
        ret |= cameraWriteRegister(0xF7, 0x10);
        ret |= cameraWriteRegister(0xFC, 0x01);
    } else {
        ret |= cameraWriteRegister(0xF2, 0x0F);
        ret |= cameraWriteRegister(0xF7, 0x1d);
        ret |= cameraWriteRegister(0xFC, 0x06);
    }

    return ret;
}

int GC2145::setPixelFormat(pixformat_t pixformat)
{
    int ret = 0;
    uint8_t reg;

    // P0 regs
    ret |= cameraWriteRegister(0xFE, 0x00);

    // Read current output format reg
    reg = cameraReadRegister(REG_OUTPUT_FMT);

    switch (pixformat) {
        case RGB565:
            ret |= cameraWriteRegister(
                    REG_OUTPUT_FMT, REG_OUTPUT_SET_FMT(reg, REG_OUTPUT_FMT_RGB565));
            break;
        case YUV422:
        case GRAYSCALE:
            // TODO: There's no support for extracting GS from YUV so we use Bayer for 1BPP for now.
            //ret |= regWrite(GC2145_I2C_ADDR,
            //        REG_OUTPUT_FMT, REG_OUTPUT_SET_FMT(reg, REG_OUTPUT_FMT_YCBYCR));
            //break;
        case BAYER:
            // There's no BAYER support so it will just look off.
            // Make sure odd/even row are switched to work with our bayer conversion.
            ret |= cameraWriteRegister(
                    REG_SYNC_MODE, REG_SYNC_MODE_DEF | REG_SYNC_MODE_ROW_SWITCH);
            ret |= cameraWriteRegister(
                    REG_OUTPUT_FMT, REG_OUTPUT_SET_FMT(reg, REG_OUTPUT_FMT_BAYER));
            break;
        default:
            return -1;
    }

    return ret;
}

int GC2145::setWindow(uint16_t reg, uint16_t x, uint16_t y, uint16_t w, uint16_t h) {
    int ret = 0;

    // P0 regs
    ret |= cameraWriteRegister(0xFE, 0x00);

    // Y/row offset
    ret |= cameraWriteRegister(reg++, y >> 8);
    Serial.printf("0x%x, %x (%d)\n", reg, y >> 8);
    ret |= cameraWriteRegister(reg++, y & 0xff);
    Serial.printf("0x%x, %x (%d)\n", reg, y & 0xff);

    // X/col offset
    ret |= cameraWriteRegister(reg++, x >> 8);
    Serial.printf("0x%x, %x (%d)\n", reg, x >> 8);
    ret |= cameraWriteRegister(reg++, x & 0xff);
    Serial.printf("0x%x, %x (%d)\n", reg,  x & 0xff);
    // Window height
    ret |= cameraWriteRegister(reg++, h >> 8);
    Serial.printf("0x%x, %x (%d)\n", reg, h >> 8);
    ret |= cameraWriteRegister(reg++, h & 0xff);
    Serial.printf("0x%x, %x (%d)\n", reg, h & 0xff);
    // Window width
    ret |= cameraWriteRegister(reg++, w >> 8);
    Serial.printf("0x%x, %x (%d)\n", reg, w >> 8);
    ret |= cameraWriteRegister(reg++, w & 0xff);
    Serial.printf("0x%x, %x (%d)\n", reg, w & 0xff);
    
    return ret;
}

uint8_t GC2145::setFramesize(framesize_t framesize) {
    int ret = 0;

    uint16_t w = resolution[framesize][0];
    uint16_t h = resolution[framesize][1];
    
    _width = w;
    _height = h;

    // Invalid resolution.
    if ((w > ACTIVE_SENSOR_WIDTH) || (h > ACTIVE_SENSOR_HEIGHT)) {
        return -1;
    }

    // Step 0: Clamp readout settings.

    readout_w = max(readout_w, w);
    readout_h = max(readout_h, h);
    
    int readout_x_max = (ACTIVE_SENSOR_WIDTH - readout_w) / 2;
    int readout_y_max = (ACTIVE_SENSOR_HEIGHT - readout_h) / 2;
    readout_x = max(min(readout_x, readout_x_max), -readout_x_max);
    readout_y = max(min(readout_y, readout_y_max), -readout_y_max);


    // Step 1: Determine sub-readout window.
    uint16_t ratio = fast_floorf(min(readout_w / ((float) w), readout_h / ((float) h)));

    // Limit the maximum amount of scaling allowed to keep the frame rate up.
    ratio = min(ratio, (fov_wide ? 5 : 3));

    if (!(ratio % 2)) {
        // camera outputs messed up bayer images at even ratios for some reason...
        ratio -= 1;
    }
    
    uint16_t sub_readout_w = w * ratio;
    uint16_t sub_readout_h = h * ratio;

    // Step 2: Determine horizontal and vertical start and end points.
    uint16_t sensor_w = sub_readout_w + DUMMY_WIDTH_BUFFER; // camera hardware needs dummy pixels to sync
    uint16_t sensor_h = sub_readout_h + DUMMY_HEIGHT_BUFFER; // camera hardware needs dummy lines to sync

    uint16_t sensor_x = max(min((((ACTIVE_SENSOR_WIDTH - sensor_w) / 4) - (readout_x / 2)) * 2,
                                      ACTIVE_SENSOR_WIDTH - sensor_w), -(DUMMY_WIDTH_BUFFER / 2)) + DUMMY_COLUMNS; // must be multiple of 2

    uint16_t sensor_y = max(min((((ACTIVE_SENSOR_HEIGHT - sensor_h) / 4) - (readout_y / 2)) * 2,
                                      ACTIVE_SENSOR_HEIGHT - sensor_h), -(DUMMY_HEIGHT_BUFFER / 2)) + DUMMY_LINES; // must be multiple of 2

    // Step 3: Write regs.
    // Set Readout window first.

#if defined(DEBUG_CAMERA)
    Serial.println("\nSet Framesize:");
    Serial.println("Step 0: Clamp readout settings");
    Serial.printf("ActSenWidth: %d, ActSenHeight: %d\n", ACTIVE_SENSOR_WIDTH, ACTIVE_SENSOR_HEIGHT);
    Serial.printf("Width: %d, Height: %d\n", w, h);
    Serial.printf("ReadoutW: %d, ReadoutH: %d\n", readout_w, readout_h);
    Serial.printf("ReadoutXmax: %d, ReadoutYmax: %d\n", readout_x_max, readout_y_max);
    Serial.printf("ReadoutX: %d, ReadoutY: %d\n", readout_x, readout_y);
    Serial.println("\nStep 1: Determine sub-readout window.");
    Serial.printf("Ratio: %d\n", ratio);
    Serial.printf("Ratio after test: %d\n", ratio);
    Serial.printf("sub_readout_w: %d, sub_readout_h: %d\n", sub_readout_w, sub_readout_h);
    Serial.println("\nStep 2: Determine horizontal and vertical start and end points");
    Serial.printf("sensor_w: %d, sensor_h: %d, sensor_x: %d, sensor_y: %d\n", sensor_w,sensor_h, sensor_x, sensor_y);
    Serial.println("\nStep 3: Write Regs - call set window");
    Serial.printf("SensorX: %d, SensorY: %d, SensorW: %d, SensorH: %d\n\n", sensor_x, sensor_y, sensor_w, sensor_h);
#endif
    
    ret |= setWindow(0x09, sensor_x, sensor_y, sensor_w, sensor_h);

    // Set cropping window next.
    ret |= setWindow(0x91, 0, 0, w, h);

    // Enable crop
    ret |= cameraWriteRegister(0x90, 0x01);

    // Set Sub-sampling ratio and mode
    ret |= cameraWriteRegister(0x99, ((ratio << 4) | (ratio)));
    ret |= cameraWriteRegister(0x9A, 0x0E);
    


    return ret;
}

int GC2145::setHmirror(int enable) {
    int ret = 0;
    uint8_t reg;

    // P0 regs
    ret |= cameraWriteRegister(0xFE, 0x00);
    reg = cameraReadRegister(REG_AMODE1);
    ret |= reg;
    ret |= cameraWriteRegister( REG_AMODE1, REG_AMODE1_SET_HMIRROR(reg, enable));
    return ret;
}

int GC2145::setVflip(int enable) {
    int ret = 0;
    uint8_t reg;

    // P0 regs
    ret |= cameraWriteRegister( 0xFE, 0x00);
    reg = cameraReadRegister(REG_AMODE1);
    ret |= reg;
    ret |= cameraWriteRegister(REG_AMODE1, REG_AMODE1_SET_VMIRROR(reg, enable));
    return ret;
}

int GC2145::setAutoExposure(int enable, int exposure_us) {
    int ret = 0;
    uint8_t reg;
    ret |= cameraWriteRegister( 0xFE, 0x00);
    reg = cameraReadRegister( 0xb6);
    ret |= reg;
    ret |= cameraWriteRegister(0xb6, (reg & 0xFE) | (enable & 0x01));
    return ret;
}

int GC2145::setAutoWhitebal(int enable, float r_gain_db, float g_gain_db, float b_gain_db) {
    int ret = 0;
    uint8_t reg;
    ret |= cameraWriteRegister( 0xFE, 0x00);
    reg = cameraReadRegister( 0x82);
    ret |= reg;
    ret |= cameraWriteRegister(0x82, (reg & 0xFD) | ((enable & 0x01) << 1));
    return ret;
}




typedef struct {
  uint16_t reg;
  const __FlashStringHelper *reg_name;
} GC2145_TO_NAME_t;

static const GC2145_TO_NAME_t GC2145_reg_name_table[] PROGMEM {
    {0x00f0, F(" chip_ID[15:8]")},
    {0x00f1, F(" chip_ID[7:0]")},
    {0x00f2, F(" pad_vb_hiz_mode data_pad_io sync_pad_io")},
    {0x00f3, F(" I2C_open_en")},
    {0x00f6, F(" Up_dn Pwd_dn")},
    {0x00f7, F(" PLL_mode1")},
    {0x00f8, F(" PLL_mode2")},
    {0x00f9, F(" cm_mode")},
    {0x00fa, F(" clk_div_mode")},
    {0x00fb, F(" I2C_device_ID")},
    {0x00fc, F(" analog_pwc")},
    {0x00fd, F(" Scalar mode")},
    {0x00fe, F(" Reset related")},
    {0x0003, F("Exposure[12:8]")},
    {0x0004, F("Exposure[7:0]")},
    {0x0005, F("capt_hb[11:8]")},
    {0x0006, F("capt_hb[7:0]")},
    {0x0007, F("capt_vb[12:8]")},
    {0x0008, F("capt_vb[7:0]")},
    {0x0009, F("capt_row_start[10:8]")},
    {0x000a, F("capt_row_start[7:0]")},
    {0x000b, F("capt_col_start[10:8 ]")},
    {0x000c, F("capt_col_start[7:1]")},
    {0x000d, F("capt_win_height[10:8]")},
    {0x000e, F("capt_win_height[7:0]")},
    {0x000f, F("capt_win_width[10:8]")},
    {0x0010, F("capt_win_width[7:1]")},
    {0x0011, F("Sh_delay[9:8]")},
    {0x0012, F("Sh_delay[7:0]")},
    {0x0013, F("St -> Start time")},
    {0x0014, F("Et -> End time")},
    {0x0017, F("Analog mode1")},
    {0x0018, F("Analog mode2")},
    {0x0020, F("Analog mode3")},
    {0x0024, F("Driver mode")},
    {0x003f, F("dark_current_st able_th")},
    {0x0040, F("Blk_mode1")},
    {0x0042, F("BLK_limit_value")},
    {0x0043, F("BLK_fame_cnt_TH")},
    {0x005c, F("Exp_rate_darkc")},
    {0x005e, F("current_G1_offset_odd_ratio")},
    {0x005f, F("current_G1_offset_even_ratio")},
    {0x0060, F("current_R1_offset_odd_ratio")},
    {0x0061, F("current_R1_offset_even_ratio")},
    {0x0062, F("current_B1_offset_odd_ratio")},
    {0x0063, F("current_B1_offset_even_ratio")},
    {0x0064, F("current_G2_offset_odd_ratio")},
    {0x0065, F("current_G2_offset_even_ratio")},
    {0x0066, F("Dark_current_G1_ratio")},
    {0x0067, F("Dark_current_R_ratio")},
    {0x0068, F("Dark_current_B_ratio")},
    {0x0069, F("Dark_current_G2_ratio")},
    {0x006a, F("manual_G1_odd_offset")},
    {0x006b, F("manual_G1_even_offset")},
    {0x006c, F("manual_R1_odd_offset")},
    {0x006d, F("manual_R1_even_offset")},
    {0x006e, F("manual_B2_odd_offset")},
    {0x006f, F("manual_B2_even_offset")},
    {0x0070, F("manual_G2_odd_offset")},
    {0x0071, F("manual_G2_even_offset")},
    {0x0072, F("BLK_DD_thBLK_various_th")},
    {0x0080, F("Block_enable1")},
    {0x0081, F("Block_enable2")},
    {0x0082, F("Block enable")},
    {0x0083, F("Special effect")},
    {0x0084, F("Output format")},
    {0x0085, F("Frame start")},
    {0x0086, F("Sync mode")},
    {0x0087, F("block_enable3_buf")},
    {0x0088, F("module_gating")},
    {0x0089, F("bypass_mode")},
    {0x008c, F("debug_mode2")},
    {0x008d, F("Debug_mode3")},
    {0x0090, F("Crop enable")},
    {0x0091, F("out_win_y1[10:8]")},
    {0x0092, F("out_win_y1 [7:0]")},
    {0x0093, F("out_win_x1[10:8]")},
    {0x0094, F("out_win_x1[7:0]")},
    {0x0095, F("out_win_height[10:8]")},
    {0x0096, F("out_win_height[7:0]")},
    {0x0097, F("out_win_width[10:8]")},
    {0x0098, F("out_win_width[7:0]")},
    {0x0099, F("subsample")},
    {0x009a, F("Subsample mode")},
    {0x009b, F("Sub_row_N1")},
    {0x009c, F("Sub_row_N2")},
    {0x009d, F("Sub_row_N3")},
    {0x009e, F("Sub_row_N4")},
    {0x009f, F("Sub_col_N1")},
    {0x00a0, F("Sub_col_N2")},
    {0x00a1, F("Sub_col_N3")},
    {0x00a2, F("Sub_col_N4")},
    {0x00a3, F("channel_gain_G1_odd")},
    {0x00a4, F("channel_gain_G1_even")},
    {0x00a5, F("channel_gain_R1_odd")},
    {0x00a6, F("channel_gain_R1_even")},
    {0x00a7, F("channel_gain_B2_odd")},
    {0x00a8, F("channel_gain_")},
    {0x00a9, F("channel_gain_G2_odd")},
    {0x00aa, F("channel_gain_G2_even")},
    {0x00ad, F("R_ratio")},
    {0x00ae, F("G_ratio")},
    {0x00af, F("B_ratio")},
    {0x00b0, F("Global_gain")},
    {0x00b1, F("Auto_pregain")},
    {0x00b2, F("Auto_postgain")},
    {0x00b3, F("AWB_R_gain")},
    {0x00b4, F("AWB_G_gain")},
    {0x00b5, F("AWB_B_gain")},
    {0x00b6, F("AEC_enable")},
    {0x00c2, F("output_buf_enable_buf")},
    {0x0101, F("AEC_x1")},
    {0x0102, F("AEC_x2")},
    {0x0103, F("AEC_y1")},
    {0x0104, F("AEC_y2")},
    {0x0105, F("AEC_center_x1")},
    {0x0106, F("AEC_center_x2")},
    {0x0107, F("AEC_center_y1")},
    {0x0108, F("AEC_center_y2")},
    {0x010a, F("AEC_mode1")},
    {0x010b, F("AEC_mode2")},
    {0x010c, F("AEC_mode3")},
    {0x010d, F("AEC_mode4")},
    {0x010e, F("AEC_high_range")},
    {0x010f, F("AEC_low_range")},
    {0x0113, F("AEC_target_Y")},
    {0x0114, F("Y_average")},
    {0x0115, F("target_Y_limit_from_histogram")},
    {0x0116, F("AEC_number_limit_high_range")},
    {0x0118, F("AEC mode5")},
    {0x0119, F("AEC mode 6")},
    {0x011a, F("AEC gainmode")},
    {0x011f, F("AEC_max_pre_dg_gain")},
    {0x0120, F("AEC_max_post_dg_gain")},
    {0x0125, F("AEC_anti_flicker_step[12:8]")},
    {0x0126, F("AEC_anti_flicker_step[7:0]")},
    {0x0127, F("AEC_exp_level_1[12:8]")},
    {0x0128, F("AEC_exp_level_1[7:0]")},
    {0x0129, F("AEC_exp_level_2[12:8]")},
    {0x012a, F("AEC_exp_level_2[7:0]")},
    {0x012b, F("AEC_exp_level_3[12:8]")},
    {0x012c, F("AEC_exp_level_3[7:0]")},
    {0x012d, F("AEC_exp_level_4[12:8]")},
    {0x012e, F("AEC_exp_level_4[7:0]")},
    {0x012f, F("AEC_exp_level_5[12:8]")},
    {0x0130, F("AEC_exp_level_5[7:0]")},
    {0x0131, F("AEC_exp_level_6[12:8]")},
    {0x0132, F("AEC_exp_level_6[7:0]")},
    {0x0133, F("AEC_exp_level_7[12:8]")},
    {0x0134, F("AEC_exp_level_7[7:0]")},
    {0x0135, F("AEC_max_dg_gain1")},
    {0x0136, F("AEC_max_dg_gain2")},
    {0x0137, F("AEC_max_dg_gain3")},
    {0x0138, F("AEC_max_dg_gain4")},
    {0x0139, F("AEC_max_dg_gain5")},
    {0x013a, F("AEC_max_dg_gain6")},
    {0x013b, F("AEC_max_dg_gain7")},
    {0x013c, F("AEC_max_exp_level")},
    {0x013d, F("AEC_exp_min_l[7:0]")},
    {0x0150, F("AWB mode 1")},
    {0x0151, F("AWBparameter")},
    {0x0152, F("AWBparameter")},
    {0x0153, F("AWBparameter")},
    {0x0154, F("AWBparameter")},
    {0x0155, F("AWBparameter")},
    {0x0156, F("AWBparameter")},
    {0x0157, F("AWBparameter")},
    {0x0158, F("AWBparameter")},
    {0x0159, F("AWB_PRE_RGB_low")},
    {0x015a, F("AWB_PRE_RGB_high")},
    {0x015b, F("AWBparameter")},
    {0x0175, F("AWB_every_N")},
    {0x0176, F("AWB_R_gain_limit")},
    {0x0177, F("AWB_G_gain_limit")},
    {0x0178, F("AWB_B_gain_limit")},
    {0x0179, F("AWB_R_gain_out_h_limit")},
    {0x017a, F("AWB_G_gain_out_h_limit")},
    {0x017b, F("AWB_B_gain_out_h_limit")},
    {0x017c, F("AWB_R_gain_out_l_limit")},
    {0x017d, F("AWB_G_gain_out_l_limit")},
    {0x017e, F("AWB_B_gain_out_l_limit")},
    {0x019a, F("ABS_range_compesateABS_skip_frame")},
    {0x019b, F("ABS_stop_margin")},
    {0x019c, F("Y_S_compesateABS_manual_K")},
    {0x019d, F("Y_stretch_limit")},
    {0x01a0, F("LSC_row_x2LSC_col_x2LSC_pixel_array_select")},
    {0x01a1, F("LSC_row_center")},
    {0x01a2, F("LSC_col_cente")},
    {0x01a4, F("LSC_Q12_RGB_sign")},
    {0x01a5, F("LSC_Q34_RGB_SIGN")},
    {0x01a6, F("LSC_right_left_rgb_b4_sign")},
    {0x01a7, F("LSC_up_down_rgb_b4_sign")},
    {0x01a8, F("LSC_right_up_down_rgb_b22_sign")},
    {0x01a9, F("LSC_left_up_down_rgb_b22_sign")},
    {0x01aa, F("LSC_Q1_red_b1")},
    {0x01ab, F("LSC_Q1_green_b1")},
    {0x01ac, F("LSC_Q1_blue_b1")},
    {0x01ad, F("LSC_Q2_red_b1")},
    {0x01ae, F("LSC_Q2_green_b1")},
    {0x01af, F("LSC_Q2_blue_b1")},
    {0x01b0, F("LSC_Q3_red_b1")},
    {0x01b1, F("LSC_Q3_green_b1")},
    {0x01b2, F("LSC_Q3_blue_b1")},
    {0x01b3, F("LSC_Q4_red_b1")},
    {0x01b4, F("LSC_Q4_green_b1")},
    {0x01b5, F("LSC_Q4_blue_b1")},
    {0x01b6, F("LSC_right_red_b2")},
    {0x01b7, F("LSC_right_green_b2")},
    {0x01b8, F("LSC_right_blue_b2")},
    {0x01b9, F("LSC_right_red_b4")},
    {0x01ba, F("LSC_right_green_b4")},
    {0x01bb, F("LSC_right_blue_b4")},
    {0x01bc, F("LSC_left_red_b2")},
    {0x01bd, F("LSC_left_green_b2")},
    {0x01be, F("LSC_left_blue_b2")},
    {0x01bf, F("LSC_left_red_b4")},
    {0x01c0, F("LSC_left_green_b4")},
    {0x01c1, F("LSC_left_blue_")},
    {0x01c2, F("LSC_up_red_b2")},
    {0x01c3, F("LSC_up_green_b2")},
    {0x01c4, F("LSC_up_blue_b2")},
    {0x01c5, F("LSC_up_red_b4")},
    {0x01c6, F("LSC_up_green_b4")},
    {0x01c7, F("LSC_up_blue_b4")},
    {0x01c8, F("LSC_down_red_b2")},
    {0x01c9, F("LSC_down_green_b2")},
    {0x01ca, F("LSC_down_blue_b2")},
    {0x01cb, F("LSC_down_red_b4")},
    {0x01cc, F("LSC_down_green_b4")},
    {0x01cd, F("LSC_down_blue_b4")},
    {0x01d0, F("LSC_right_up_red_b22")},
    {0x01d1, F("LSC_right_up_green_b22")},
    {0x01d2, F("LSC_right_up_blue_b22")},
    {0x01d3, F("LSC_right_down_red_b22")},
    {0x01d4, F("LSC_right_down_green_b22")},
    {0x01d5, F("LSC_right_down_blue_b22")},
    {0x01d6, F("LSC_left_up_red_b22")},
    {0x01d7, F("LSC_left_up_green_b22")},
    {0x01d8, F("LSC_left_up_blue_b22")},
    {0x01d9, F("LSC_left_down_red_b22")},
    {0x01da, F("LSC_left_down_green_b22")},
    {0x01db, F("LSC_left_down_blue_b22")},
    {0x01dc, F("LSC_Y_dark_th")},
    {0x01dd, F("LSC_Y_dark_slope")},
    {0x01df, F("LSC_U_B2G_stand_plus")},
    {0x0210, F("Gamma_out1")},
    {0x0211, F("Gamma_out2")},
    {0x0212, F("Gamma_out3")},
    {0x0213, F("Gamma_out4")},
    {0x0214, F("Gamma_out5")},
    {0x0215, F("Gamma_out6")},
    {0x0216, F("Gamma_out7")},
    {0x0217, F("Gamma_out8")},
    {0x0218, F("Gamma_out9")},
    {0x0219, F("Gamma_out10")},
    {0x021a, F("Gamma_out11")},
    {0x021b, F("Gamma_out12")},
    {0x021c, F("Gamma_out13")},
    {0x021d, F("Gamma_out14")},
    {0x021e, F("Gamma_out15")},
    {0x021f, F("Gamma_out16")},
    {0x0220, F("Gamma_out17")},
    {0x0221, F("Gamma_out18")},
    {0x0222, F("Gamma_out19")},
    {0x0223, F("Gamma_out20")},
    {0x0224, F("Gamma_out21")},
    {0x0225, F("Gamma_out22")},
    {0x0284, F("DD_dark_th")},
    {0x0285, F("ASDE_DN_B_slope")},
    {0x0289, F("ASDE_low_luma_value_DD_th2")},
    {0x028a, F("ASDE_low_luma_value_DD_th3")},
    {0x028b, F("ASDE_low_luma_value_DD_th4")},
    {0x0290, F("EEINTP")},
    {0x0291, F("EEINTP")},
    {0x0292, F("direction_TH1")},
    {0x0293, F("Direction_TH2")},
    {0x0294, F("diff_HV_mode")},
    {0x0295, F("direction_diff_")},
    {0x0296, F("edge level")},
    {0x0297, F("Edge1 effect")},
    {0x0298, F("Edge_pos_ratio")},
    {0x0299, F("Edge1_max")},
    {0x029a, F("Edge2_max")},
    {0x029b, F("Edge1_th")},
    {0x029c, F("Edge_pos_max")},
    {0x029d, F("Edge_effect_sc")},
    {0x02c0, F("CC_mode")},
    {0x02c1, F("CC_CT1_11")},
    {0x02c2, F("CC_CT1_12")},
    {0x02c3, F("CC_CT1_13")},
    {0x02c4, F("CC_CT1_21")},
    {0x02c5, F("CC_CT1_22")},
    {0x02c6, F("CC_CT1_23")},
    {0x02d0, F("Global")},
    {0x02d1, F("saturation_Cb")},
    {0x02d2, F("saturation_Cr")},
    {0x02d3, F("luma_contrast")},
    {0x02d4, F("Contrast center")},
    {0x02d5, F("Luma_offset")},
    {0x02d6, F("skin_Cb_center")},
    {0x02d7, F("skin_Cr_center")},
    {0x02d9, F("Skin brightnessmode")},
    {0x02da, F("Fixed_Cb")},
    {0x02db, F("Fixed_Cr")},
    {0x02e6, F("CC_R_offset")},
    {0x02e7, F("CC_G_offset")},
    {0x02e8, F("CC_B_offset")},
    {0x0301, F("DPHY_analog_mode1")},
    {0x0302, F("DPHY_analog_mode2")},
    {0x0303, F("DPHY_analog_mode3")},
    {0x0304, F("FIFO_prog_full_level[7:0]")},
    {0x0305, F("FIFO_prog_full_level[11:8]")},
    {0x0306, F("FIFO_mode")},
    {0x0310, F("BUF_CSI2_mode")},
    {0x0311, F("LDI_set")},
    {0x0312, F("LWC_set[7:0]")},
    {0x0313, F("LWC_set[15:8]")},
    {0x0314, F("SYNC_set")},
    {0x0315, F("DPHY_mode")},
    {0x0316, F("LP_set")},
    {0x0317, F("fifo_gate_modeMIPI_wdiv_set")},
    {0x0320, F("T_init_set")},
    {0x0321, F("T_LPX_set")},
    {0x0322, F("T_CLK_HS_PREPARE_set")},
    {0x0323, F("T_CLK_zero_set")},
    {0x0324, F("T_CLK_PRE_set")},
    {0x0325, F("T_CLK_POST_set")},
    {0x0326, F("T_CLK_TRAIL_set")},
    {0x0327, F("T_HS_exit_set")},
    {0x0328, F("T_wakeup_set")},
    {0x0329, F("T_HS_PREPARE_set")},
    {0x032a, F("T_HS_Zero_set")},
    {0x032b, F("T_HS_TRAIL_set")},
    {0x0330, F("MIPI_Test")},
    {0x0331, F("MIPI_Test_data0")},
    {0x0332, F("MIPI_Test_data1")},
    {0x0333, F("MIPI_Test_data2")},
    {0x0334, F("MIPI_Test_data3")},
    {0x033f, F("FIFO_error log")},
    {0x0340, F("output_buf_mode1")},
    {0x0341, F("output_buf_mode2")},
    {0x0342, F("buf_win_width[7:0]")},
    {0x0343, F("buf_win_width[11:8]")},
};

uint16_t gc2145_reg_page = 0;
uint32_t gc2145_registers_set[32] = {0};

#if (0)
void GC2145::printRegisters(bool only_ones_set)
{
    uint8_t reg;
    Serial.println("\n*** Camera Registers ***");
    if (only_ones_set) {
        uint8_t current_page = 0;
        for (uint16_t ii = 3; ii < 1024; ii++) {
            if (gc2145_registers_set[ii >> 5] & (1 << (ii & 0x1f))) {
                uint8_t page = ii >> 8;
                if (page != current_page) {
                    current_page = page;
                    cameraWriteRegister(0xfe, page);
                }
                reg = cameraReadRegister(ii);
                Serial.printf("(%u:0x%x): (0x%x - %d)", page, ii & 0xff, reg, reg);
                for (uint16_t jj=0; jj < (sizeof(GC2145_reg_name_table)/sizeof(GC2145_reg_name_table[0])); jj++) {
                    if (ii == GC2145_reg_name_table[jj].reg) {
                        Serial.print("\t: ");
                        Serial.print(GC2145_reg_name_table[jj].reg_name);
                        break;
                    }
                }
                Serial.println();
            }
        }
        if (current_page != 0) cameraWriteRegister(0xfe, 0);

    } else {
        for (uint16_t ii = 3; ii < 182; ii++) {
            reg = cameraReadRegister(ii);
            Serial.printf("(0x%x): (0x%x - %d)", ii, reg, reg);

            for (uint16_t jj=0; jj < (sizeof(GC2145_reg_name_table)/sizeof(GC2145_reg_name_table[0])); jj++) {
                if (ii == GC2145_reg_name_table[jj].reg) {
                    Serial.print("\t: ");
                    Serial.print(GC2145_reg_name_table[jj].reg_name);
                    break;
                }
            }
            Serial.println();
        }
    }
}
#endif

void GC2145::showRegisters(void) {
  Serial.println("\n*** Camera Registers ***");
  for (uint16_t ii = 0; ii < (sizeof(GC2145_reg_name_table) / sizeof(GC2145_reg_name_table[0])); ii++) {
    uint8_t reg_value = cameraReadRegister(GC2145_reg_name_table[ii].reg);
    Serial.printf("%s(%x): %u(%x)\n", GC2145_reg_name_table[ii].reg_name, GC2145_reg_name_table[ii].reg, reg_value, reg_value);
  }
}
/*******************************************************************/
// Read a single uint8_t from address and return it as a uint8_t
uint8_t GC2145::cameraReadRegister(uint8_t reg) {
  _wire->beginTransmission(camAddress);
  //_wire->write(reg >> 8);
  _wire->write(reg);
  if (_wire->endTransmission(false) != 0) {
    Serial.println("error reading GC2145, address");
    return 0;
  }
  if (_wire->requestFrom(camAddress, 1) < 1) {
    Serial.println("error reading GC2145, data");
    return 0;
  }
  return _wire->read();
}


uint8_t GC2145::cameraWriteRegister(uint8_t reg, uint8_t data) {
#ifdef DEBUG_CAMERA
    if (reg == 0xfe) gc2145_reg_page = (data & 0x7) << 8; 

    Serial.printf("Write Register (%u:0x%x): (0x%x - %d)", gc2145_reg_page >> 8, reg, data, data);
    uint16_t reg_lookup = reg;
    if (reg < 0xf0) reg_lookup += gc2145_reg_page;

    // lets remember all of the registers we wrote something to
    gc2145_registers_set[reg_lookup >> 5] |= 1 << (reg_lookup & 0x1f);


    for (uint16_t jj=0; jj < (sizeof(GC2145_reg_name_table)/sizeof(GC2145_reg_name_table[0])); jj++) {
        if (reg_lookup == GC2145_reg_name_table[jj].reg) {
            Serial.print("\t: ");
            Serial.print(GC2145_reg_name_table[jj].reg_name);
            break;
        }
    }
    Serial.println();

#endif
  _wire->beginTransmission(camAddress);
  //_wire->write(reg >> 8);
  _wire->write(reg);
  _wire->write(data);
  if (_wire->endTransmission() != 0) {
    Serial.println("error writing to GC2145");
  }

  return 0;
}

/***********************************************************************/



#define FLEXIO_USE_DMA
void GC2145::readFrame(void* buffer, bool use_dma){
    if(!_use_gpio) {
        readFrameFlexIO(buffer, use_dma);
    } else {
        readFrameGPIO(buffer);
    }
}

void GC2145::readFrameMultiBuffer(void* buffer1, size_t size1, void* buffer2, size_t size2) {
    if(!_use_gpio) {
        readFrameMultiBufferFlexIO(buffer1, size1, buffer2, size2);
    } else {
        //readFrameGPIO(buffer1);
    }
}


bool GC2145::readContinuous(bool(*callback)(void *frame_buffer), void *fb1, void *fb2) {

	return startReadFlexIO(callback, fb1, fb2);

}

void GC2145::stopReadContinuous() {
	
  stopReadFlexIO();

}
/*
void GC2145::readFrameGPIO(void* buffer)
{

  uint8_t* b = (uint8_t*)buffer;
//  bool _grayscale;  // ????  member variable ?????????????
  int bytesPerRow = _width * _bytesPerPixel;

  // Falling edge indicates start of frame
  //pinMode(PCLK_PIN, INPUT); // make sure back to input pin...
  // lets add our own glitch filter.  Say it must be hig for at least 100us
  elapsedMicros emHigh;
  do {
    while ((*_vsyncPort & _vsyncMask) == 0); // wait for HIGH
    emHigh = 0;
    while ((*_vsyncPort & _vsyncMask) != 0); // wait for LOW
  } while (emHigh < 2);

  for (int i = 0; i < _height; i++) {
    // rising edge indicates start of line
    while ((*_hrefPort & _hrefMask) == 0); // wait for HIGH
    while ((*_pclkPort & _pclkMask) != 0); // wait for LOW
    noInterrupts();

    for (int j = 0; j < bytesPerRow; j++) {
      // rising edges clock each data byte
      while ((*_pclkPort & _pclkMask) == 0); // wait for HIGH 

      //uint32_t in = ((_frame_buffer_pointer)? GPIO1_DR : GPIO6_DR) >> 18; // read all bits in parallel
      uint32_t in =  (GPIO7_PSR >> 4); // read all bits in parallel  

	  //uint32_t in = mmBus;

      if (!(j & 1) || !_grayscale) {
        *b++ = in;
      }
      while (((*_pclkPort & _pclkMask) != 0) && ((*_hrefPort & _hrefMask) != 0)) ; // wait for LOW bail if _href is lost
    }

    while ((*_hrefPort & _hrefMask) != 0) ;  // wait for LOW
    interrupts();
  }

}
*/
void GC2145::readFrameGPIO(void *buffer, size_t cb1, void *buffer2, size_t cb2)
{    
  Serial.printf("$$readFrameGPIO(%p, %u, %p, %u)\n", buffer, cb1, buffer2, cb2);
  uint8_t* b = (uint8_t*)buffer;
  uint32_t cb = (uint32_t)cb1;
//  bool _grayscale;  // ????  member variable ?????????????
  int bytesPerRow = _width * _bytesPerPixel;

  // Falling edge indicates start of frame
  //pinMode(PCLK_PIN, INPUT); // make sure back to input pin...
  // lets add our own glitch filter.  Say it must be hig for at least 100us
  elapsedMicros emHigh;
  do {
    while ((*_vsyncPort & _vsyncMask) == 0); // wait for HIGH
    emHigh = 0;
    while ((*_vsyncPort & _vsyncMask) != 0); // wait for LOW
  } while (emHigh < 2);

  for (int i = 0; i < _height; i++) {
    // rising edge indicates start of line
    while ((*_hrefPort & _hrefMask) == 0); // wait for HIGH
    while ((*_pclkPort & _pclkMask) != 0); // wait for LOW
    noInterrupts();

    for (int j = 0; j < bytesPerRow; j++) {
      // rising edges clock each data byte
      while ((*_pclkPort & _pclkMask) == 0); // wait for HIGH

      //uint32_t in = ((_frame_buffer_pointer)? GPIO1_DR : GPIO6_DR) >> 18; // read all bits in parallel
      uint32_t in =  (GPIO7_PSR >> 4); // read all bits in parallel  

	  //uint32_t in = mmBus;
      // bugbug what happens to the the data if grayscale?
      if (!(j & 1) || !_grayscale) {
        *b++ = in;
        if ( buffer2 && (--cb == 0) ) {
          Serial.printf("\t$$ 2nd buffer: %u %u\n", i, j);
          b = (uint8_t *)buffer2;
          cb = (uint32_t)cb2;
          buffer2 = nullptr;
        }
      }
      while (((*_pclkPort & _pclkMask) != 0) && ((*_hrefPort & _hrefMask) != 0)) ; // wait for LOW bail if _href is lost
    }

    while ((*_hrefPort & _hrefMask) != 0) ;  // wait for LOW
    interrupts();
  }

}



bool GC2145::flexio_configure()
{

    // Going to try this using my FlexIO library.

    // BUGBUG - starting off not going to worry about maybe first pin I choos is on multipl Flex IO controllers (yet)
    uint8_t tpclk_pin; 
    _pflex = FlexIOHandler::mapIOPinToFlexIOHandler(_pclkPin, tpclk_pin);
    if (!_pflex) {
        Serial.printf("GC2145 PCLK(%u) is not a valid Flex IO pin\n", _pclkPin);
        return false;
    }
    _pflexio = &(_pflex->port());

    // Quick and dirty:
    uint8_t thsync_pin = _pflex->mapIOPinToFlexPin(_hrefPin);
    uint8_t tg0 = _pflex->mapIOPinToFlexPin(_dPins[0]);
    uint8_t tg1 = _pflex->mapIOPinToFlexPin(_dPins[1]);
    uint8_t tg2 = _pflex->mapIOPinToFlexPin(_dPins[2]);
    uint8_t tg3 = _pflex->mapIOPinToFlexPin(_dPins[3]);

    // make sure the minimum here is valid: 
    if ((thsync_pin == 0xff) || (tg0 == 0xff) || (tg1 == 0xff) || (tg2 == 0xff) || (tg3 == 0xff)) {
        Serial.printf("GC2145 Some pins did not map to valid Flex IO pin\n");
        Serial.printf("    HSYNC(%u %u) G0(%u %u) G1(%u %u) G2(%u %u) G3(%u %u)", 
            _hrefPin, thsync_pin, _dPins[0], tg0, _dPins[1], tg1, _dPins[2], tg2, _dPins[3], tg3 );
        return false;
    } 
    // Verify that the G numbers are consecutive... Should use arrays!
    if ((tg1 != (tg0+1)) || (tg2 != (tg0+2)) || (tg3 != (tg0+3))) {
        Serial.printf("GC2145 Flex IO pins G0-G3 are not consective\n");
        Serial.printf("    G0(%u %u) G1(%u %u) G2(%u %u) G3(%u %u)", 
            _dPins[0], tg0, _dPins[1], tg1, _dPins[2], tg2, _dPins[3], tg3 );
        return false;
    }
    if (_dPins[4] != 0xff) {
        uint8_t tg4 = _pflex->mapIOPinToFlexPin(_dPins[4]);
        uint8_t tg5 = _pflex->mapIOPinToFlexPin(_dPins[5]);
        uint8_t tg6 = _pflex->mapIOPinToFlexPin(_dPins[6]);
        uint8_t tg7 = _pflex->mapIOPinToFlexPin(_dPins[7]);
        if ((tg4 != (tg0+4)) || (tg5 != (tg0+5)) || (tg6 != (tg0+6)) || (tg7 != (tg0+7))) {
            Serial.printf("GC2145 Flex IO pins G4-G7 are not consective with G0-3\n");
            Serial.printf("    G0(%u %u) G4(%u %u) G5(%u %u) G6(%u %u) G7(%u %u)", 
                _dPins[0], tg0, _dPins[4], tg4, _dPins[5], tg5, _dPins[6], tg6, _dPins[7], tg7 );
            return false;
        }
        Serial.println("Custom - Flexio is 8 bit mode");
    } else {
      // only 8 bit mode supported
      Serial.println("Custom - Flexio 4 bit mode not supported");
      return false;
    }
#if (CNT_SHIFTERS == 1)
    // Needs Shifter 3 (maybe 7 would work as well?)
    if (_pflex->claimShifter(3)) _fshifter = 3;
    else if (_pflex->claimShifter(7)) _fshifter = 7;
    else {
      Serial.printf("GC2145 Flex IO: Could not claim Shifter 3 or 7\n");
      return false;
    }
    _fshifter_mask = 1 << _fshifter;   // 4 channels.
    _dma_source = _pflex->shiftersDMAChannel(_fshifter); // looks like they use 

#elif (CNT_SHIFTERS == 4)
    // lets try to claim for shifters 0-3 or 4-7
    // Needs Shifter 3 (maybe 7 would work as well?)
    for (_fshifter = 0; _fshifter < 4; _fshifter++) {
      if (!_pflex->claimShifter(_fshifter)) break;
    }

    if (_fshifter < CNT_SHIFTERS) {
      // failed on 0-3 - released any we claimed
      Serial.printf("Failed to claim 0-3(%u) shifters trying 4-7\n", _fshifter);
      while (_fshifter > 0) _pflex->freeShifter(--_fshifter);  // release any we grabbed

      for (_fshifter = 4; _fshifter < (4 + CNT_SHIFTERS); _fshifter++) {
        if (!_pflex->claimShifter(_fshifter)) {
          Serial.printf("GC2145 Flex IO: Could not claim Shifter %u\n", _fshifter);
          while (_fshifter > 4) _pflex->freeShifter(--_fshifter);  // release any we grabbed
          return false;
        }
      }
      _fshifter = 4;
    } else {
      _fshifter = 0;
    }


    // ?????????? dma source... 
    _fshifter_mask = 1 << _fshifter;   // 4 channels.
    _dma_source = _pflex->shiftersDMAChannel(_fshifter); // looks like they use 
#else
    // all 8 shifters.
    for (_fshifter = 0; _fshifter < 8; _fshifter++) {
      if (!_pflex->claimShifter(_fshifter)) {
        Serial.printf("GC2145 Flex IO: Could not claim Shifter %u\n", _fshifter);
        while (_fshifter > 4) _pflex->freeShifter(--_fshifter);  // release any we grabbed
        return false;
      }
    }
    _fshifter = 0;
    _fshifter_mask = 1 /*0xff */; // 8 channels << _fshifter;   // 4 channels.
    _dma_source = _pflex->shiftersDMAChannel(_fshifter); // looks like they use 
#endif    
    
    // Now request one timer
    uint8_t _ftimer = _pflex->requestTimers(); // request 1 timer. 
    if (_ftimer == 0xff) {
        Serial.printf("GC2145 Flex IO: failed to request timer\n");
        return false;
    }

    _pflex->setIOPinToFlexMode(_hrefPin);
    _pflex->setIOPinToFlexMode(_pclkPin);
    _pflex->setIOPinToFlexMode(_dPins[0]);
    _pflex->setIOPinToFlexMode(_dPins[1]);
    _pflex->setIOPinToFlexMode(_dPins[2]);
    _pflex->setIOPinToFlexMode(_dPins[3]);
    _pflex->setIOPinToFlexMode(_dPins[4]);
    _pflex->setIOPinToFlexMode(_dPins[5]);
    _pflex->setIOPinToFlexMode(_dPins[6]);
    _pflex->setIOPinToFlexMode(_dPins[7]);



    // We already configured the clock to allow access.
    // Now sure yet aoub configuring the actual colock speed...

/*
    CCM_CSCMR2 |= CCM_CSCMR2__pflex->CLK_SEL(3); // 480 MHz from USB PLL

    CCM_CS1CDR = (CCM_CS1CDR
        & ~(CCM_CS1CDR__pflex->CLK_PRED(7) | CCM_CS1CDR__pflex->CLK_PODF(7)))
        | CCM_CS1CDR__pflex->CLK_PRED(1) | CCM_CS1CDR__pflex->CLK_PODF(1);


    CCM_CCGR3 |= CCM_CCGR3_FLEXIO2(CCM_CCGR_ON);
*/    
    // clksel(0-3PLL4, Pll3 PFD2 PLL5, *PLL3_sw)
    // clk_pred(0, 1, 2, 7) - divide (n+1)
    // clk_podf(0, *7) divide (n+1)
    // So default is 480mhz/16
    // Clock select, pred, podf:
    _pflex->setClockSettings(3, 1, 1);


#ifdef DEBUG_FLEXIO
    Serial.println("FlexIO Configure");
    Serial.printf(" CCM_CSCMR2 = %08X\n", CCM_CSCMR2);
    uint32_t div1 = ((CCM_CS1CDR >> 9) & 7) + 1;
    uint32_t div2 = ((CCM_CS1CDR >> 25) & 7) + 1;
    Serial.printf(" div1 = %u, div2 = %u\n", div1, div2);
    Serial.printf(" FlexIO Frequency = %.2f MHz\n", 480.0 / (float)div1 / (float)div2);
    Serial.printf(" CCM_CCGR3 = %08X\n", CCM_CCGR3);
    Serial.printf(" FlexIO CTRL = %08X\n", _pflexio->CTRL);
    Serial.printf(" FlexIO Config, param=%08X\n", _pflexio->PARAM);
#endif
    
		Serial.println("8Bit FlexIO");
      // SHIFTCFG, page 2927
      //  PWIDTH: number of bits to be shifted on each Shift clock
      //          0 = 1 bit, 1-3 = 4 bit, 4-7 = 8 bit, 8-15 = 16 bit, 16-31 = 32 bit
      //  INSRC: Input Source, 0 = pin, 1 = Shifter N+1 Output
      //  SSTOP: Stop bit, 0 = disabled, 1 = match, 2 = use zero, 3 = use one
      //  SSTART: Start bit, 0 = disabled, 1 = disabled, 2 = use zero, 3 = use one
      // setup the for shifters
      #if (CNT_SHIFTERS == 1)
      _pflexio->SHIFTCFG[_fshifter] = FLEXIO_SHIFTCFG_PWIDTH(7);
      #else
      for (int i = 0; i < (CNT_SHIFTERS - 1); i++) {
        _pflexio->SHIFTCFG[_fshifter + i] = FLEXIO_SHIFTCFG_PWIDTH(7) | FLEXIO_SHIFTCFG_INSRC;
      }
      _pflexio->SHIFTCFG[_fshifter + CNT_SHIFTERS-1] = FLEXIO_SHIFTCFG_PWIDTH(7);
      #endif

      // Timer model, pages 2891-2893
      // TIMCMP, page 2937
      // using 4 shifters
      _pflexio->TIMCMP[_ftimer] = (8U * CNT_SHIFTERS) -1 ;
      
      // TIMCTL, page 2933
      //  TRGSEL: Trigger Select ....
      //          4*N - Pin 2*N input
      //          4*N+1 - Shifter N status flag
      //          4*N+2 - Pin 2*N+1 input
      //          4*N+3 - Timer N trigger output
      //  TRGPOL: 0 = active high, 1 = active low
      //  TRGSRC: 0 = external, 1 = internal
      //  PINCFG: timer pin, 0 = disable, 1 = open drain, 2 = bidir, 3 = output
      //  PINSEL: which pin is used by the Timer input or output
      //  PINPOL: 0 = active high, 1 = active low
      //  TIMOD: mode, 0 = disable, 1 = 8 bit baud rate, 2 = 8 bit PWM, 3 = 16 bit
      #define FLEXIO_TIMER_TRIGGER_SEL_PININPUT(x) ((uint32_t)(x) << 1U)
      _pflexio->TIMCTL[_ftimer] = FLEXIO_TIMCTL_TIMOD(3)
          | FLEXIO_TIMCTL_PINSEL(tpclk_pin) // "Pin" is 16 = PCLK
          //| FLEXIO_TIMCTL_TRGSEL(4 * (thsync_pin/2)) // "Trigger" is 12 = HSYNC
          | FLEXIO_TIMCTL_TRGSEL(FLEXIO_TIMER_TRIGGER_SEL_PININPUT(thsync_pin)) // "Trigger" is 12 = HSYNC
          | FLEXIO_TIMCTL_TRGSRC;
      Serial.printf("TIMCTL: %08X PINSEL: %x THSYNC: %x\n", _pflexio->TIMCTL[_ftimer], tpclk_pin, thsync_pin);

    // SHIFTCTL, page 2926
    //  TIMSEL: which Timer is used for controlling the logic/shift register
    //  TIMPOL: 0 = shift of positive edge, 1 = shift on negative edge
    //  PINCFG: 0 = output disabled, 1 = open drain, 2 = bidir, 3 = output
    //  PINSEL: which pin is used by the Shifter input or output
    //  PINPOL: 0 = active high, 1 = active low
    //  SMOD: 0 = disable, 1 = receive, 2 = transmit, 4 = match store,
    //        5 = match continuous, 6 = state machine, 7 = logic
    // 4 shifters
    uint32_t shiftctl = FLEXIO_SHIFTCTL_TIMSEL(_ftimer) | FLEXIO_SHIFTCTL_SMOD(1)
        | FLEXIO_SHIFTCTL_PINSEL(tg0);    

    for (uint8_t i = 0; i < CNT_SHIFTERS; i++) {
      _pflexio->SHIFTCTL[_fshifter + i] = shiftctl; // 4 = D0
    }

    // TIMCFG, page 2935
    //  TIMOUT: Output
    //          0 = output is logic one when enabled and is not affected by timer reset
    //          1 = output is logic zero when enabled and is not affected by timer reset
    //          2 = output is logic one when enabled and on timer reset
    //          3 = output is logic zero when enabled and on timer reset
    //  TIMDEC: Decrement
    //          0 = on FlexIO clock, Shift clock equals Timer output
    //          1 = on Trigger input (both edges), Shift clock equals Timer output
    //          2 = on Pin input (both edges), Shift clock equals Pin input
    //          3 = on Trigger input (both edges), Shift clock equals Trigger input
    //  TIMRST: Reset
    //          0 = never reset
    //          2 = on Timer Pin equal to Timer Output
    //          3 = on Timer Trigger equal to Timer Output
    //          4 = on Timer Pin rising edge
    //          6 = on Trigger rising edge
    //          7 = on Trigger rising or falling edge
    //  TIMDIS: Disable
    //          0 = never disabled
    //          1 = disabled on Timer N-1 disable
    //          2 = disabled on Timer compare
    //          3 = on Timer compare and Trigger Low
    //          4 = on Pin rising or falling edge
    //          5 = on Pin rising or falling edge provided Trigger is high
    //          6 = on Trigger falling edge
    //  TIMENA
    //          0 = always enabled
    //          1 = enabled on Timer N-1 enable
    //          2 = enabled on Trigger high
    //          3 = enabled on Trigger high and Pin high
    //          4 = enabled on Pin rising edge
    //          5 = enabled on Pin rising edge and Trigger high
    //          6 = enabled on Trigger rising edge
    //          7 = enabled on Trigger rising or falling edge
    //  TSTOP Stop bit, 0 = disabled, 1 = on compare, 2 = on disable, 3 = on either
    //  TSTART: Start bit, 0 = disabled, 1 = enabled
    _pflexio->TIMCFG[_ftimer] = FLEXIO_TIMCFG_TIMOUT(1) | FLEXIO_TIMCFG_TIMDEC(2)
        | FLEXIO_TIMCFG_TIMENA(6) | FLEXIO_TIMCFG_TIMDIS(6);

    // CTRL, page 2916
    _pflexio->CTRL = FLEXIO_CTRL_FLEXEN; // enable after everything configured
    
#ifdef DEBUG_FLEXIO
    Serial.printf(" FLEXIO:%u Shifter:%u Timer:%u\n", _pflex->FlexIOIndex(), _fshifter, _ftimer);
    Serial.print("     SHIFTCFG = ");
    for (uint8_t i = 0; i < CNT_SHIFTERS; i++) Serial.printf(" %08X", _pflexio->SHIFTCFG[_fshifter + i]);
    Serial.print("\n     SHIFTCTL = ");
    for (uint8_t i = 0; i < CNT_SHIFTERS; i++) Serial.printf(" %08X", _pflexio->SHIFTCTL[_fshifter + i]);
    Serial.printf("\n     TIMCMP = %08X\n", _pflexio->TIMCMP[_ftimer]);
    Serial.printf("     TIMCFG = %08X\n", _pflexio->TIMCFG[_ftimer]);
    Serial.printf("     TIMCTL = %08X\n", _pflexio->TIMCTL[_ftimer]);
#endif
return true;
}


void dumpDMA_TCD_GC(DMABaseClass *dmabc, const char *psz_title) {
  if (psz_title)
    Serial.print(psz_title);
  Serial.printf("%x %x: ", (uint32_t)dmabc, (uint32_t)dmabc->TCD);

  Serial.printf(
      "SA:%x SO:%d AT:%x (SM:%x SS:%x DM:%x DS:%x) NB:%x SL:%d DA:%x DO: %d CI:%x DL:%d CS:%x BI:%x\n",
      (uint32_t)dmabc->TCD->SADDR, dmabc->TCD->SOFF, dmabc->TCD->ATTR,
      (dmabc->TCD->ATTR >> 11) & 0x1f, (dmabc->TCD->ATTR >> 8) & 0x7,
      (dmabc->TCD->ATTR >> 3) & 0x1f, (dmabc->TCD->ATTR >> 0) & 0x7,
      dmabc->TCD->NBYTES, dmabc->TCD->SLAST, (uint32_t)dmabc->TCD->DADDR,
      dmabc->TCD->DOFF, dmabc->TCD->CITER, dmabc->TCD->DLASTSGA,
      dmabc->TCD->CSR, dmabc->TCD->BITER);
}

void GC2145::readFrameFlexIO(void* buffer, bool use_dma)
{
    //flexio_configure(); // one-time hardware setup
    // wait for VSYNC to go high and then low with a sort of glitch filter
    elapsedMillis emWaitSOF;
    elapsedMicros emGlitch;
    for (;;) {
      if (emWaitSOF > 2000) {
        Serial.println("Timeout waiting for Start of Frame");
        return;
      }
      while ((*_vsyncPort & _vsyncMask) == 0);
      emGlitch = 0;
      while ((*_vsyncPort & _vsyncMask) != 0);
      if (emGlitch > 5) break;
    }

    _pflexio->SHIFTSTAT = _fshifter_mask; // clear any prior shift status
    _pflexio->SHIFTERR = _fshifter_mask;
    uint32_t *p = (uint32_t *)buffer;

    //----------------------------------------------------------------------
    // Polling FlexIO version
    //----------------------------------------------------------------------
    if (!use_dma) {
    digitalWriteFast(2, HIGH);
      // read FlexIO by polling
      uint32_t *p_end = (uint32_t *)buffer + (_width*_height/4)*_bytesPerPixel;

      while (p < p_end) {
          while ((_pflexio->SHIFTSTAT & _fshifter_mask) == 0) {
              // wait for FlexIO shifter data
          }
          // Lets try to load in multiple shifters
          for (uint8_t i = 0; i < CNT_SHIFTERS; i++) {
            *p++ = _pflexio->SHIFTBUF[_fshifter+i]; // should use DMA...
          }
      }
      digitalWriteFast(2, LOW);
      return;
    }

    //----------------------------------------------------------------------
    // Use DMA FlexIO version
    //----------------------------------------------------------------------
    digitalWriteFast(2, HIGH);

    // Lets try like other implementation.
    const uint32_t frame_size_bytes = _width*_height*_bytesPerPixel;
    //uint32_t length_uint32 = frame_size_bytes / 4;

    _dmachannel.begin();
    _dmachannel.triggerAtHardwareEvent(_dma_source);
    active_dma_camera = this;
    _dmachannel.attachInterrupt(dmaInterruptFlexIO);


#if (CNT_SHIFTERS == 4)
#define FXIO_SHFT_COUNT         4u          /* 4 shifters */
#define DMA_TRSF_SIZE           8u          /* 8 bytes */
#define DMA_MINOR_LOOP_SIZE     16u         /* 16 bytes */
#define DMA_MAJOR_LOOP_SIZE     (frame_size_bytes / DMA_MINOR_LOOP_SIZE)


    uint32_t soff, smod = 0u, size=0u;
    while(1u << size < DMA_TRSF_SIZE) /* size = log2(DMA_TRSF_SIZE) */
    {
        size++;
    }

    if(DMA_TRSF_SIZE == DMA_MINOR_LOOP_SIZE)
    {
        soff = 0u;
    }
    else
    {
        soff = DMA_TRSF_SIZE;
        while(1u << smod < DMA_MINOR_LOOP_SIZE) /* smod = log2(DMA_MINOR_LOOP_SIZE) */
        {
            smod++;
        }
    }
    /* Configure DMA TCD */
    _dmachannel.TCD->SADDR = &_pflexio->SHIFTBUF[_fshifter];
    _dmachannel.TCD->SOFF = soff;
    _dmachannel.TCD->ATTR = DMA_TCD_ATTR_SMOD(smod) |
                            DMA_TCD_ATTR_SSIZE(size) |
                            DMA_TCD_ATTR_DMOD(0u) |
                            DMA_TCD_ATTR_DSIZE(size);
    _dmachannel.TCD->NBYTES_MLNO = DMA_MINOR_LOOP_SIZE;
    _dmachannel.TCD->SLAST = 0u;
    _dmachannel.TCD->DADDR = p;
    _dmachannel.TCD->DOFF = DMA_TRSF_SIZE;
    _dmachannel.TCD->CITER_ELINKNO = DMA_MAJOR_LOOP_SIZE;
    _dmachannel.TCD->DLASTSGA = -frame_size_bytes;
    _dmachannel.TCD->CSR = 0u;
    _dmachannel.TCD->CSR |= DMA_TCD_CSR_DREQ;
    _dmachannel.TCD->BITER_ELINKNO = DMA_MAJOR_LOOP_SIZE;
#elif  (CNT_SHIFTERS == 8)
    // see if I configure for all 8 buffers
    #define SHIFT_BUFFERS_SIZE 32u
    #define SHIFT_BUFFERS_MOD 5u
    _dmachannel.TCD->SADDR = &_pflexio->SHIFTBUF[_fshifter];
    _dmachannel.TCD->SOFF = 0;
    _dmachannel.TCD->ATTR = DMA_TCD_ATTR_SMOD(0u) |
                            DMA_TCD_ATTR_SSIZE(SHIFT_BUFFERS_MOD) |
                            DMA_TCD_ATTR_DMOD(0u) |
                            DMA_TCD_ATTR_DSIZE(SHIFT_BUFFERS_MOD);
    _dmachannel.TCD->NBYTES_MLNO = SHIFT_BUFFERS_SIZE;
    _dmachannel.TCD->SLAST = 0u;
    _dmachannel.TCD->DADDR = p;
    _dmachannel.TCD->DOFF = SHIFT_BUFFERS_SIZE;
    _dmachannel.TCD->CITER_ELINKNO = (frame_size_bytes / SHIFT_BUFFERS_SIZE);
    _dmachannel.TCD->DLASTSGA = -frame_size_bytes;
    _dmachannel.TCD->CSR = 0u;
    _dmachannel.TCD->CSR |= DMA_TCD_CSR_DREQ;
    _dmachannel.TCD->BITER_ELINKNO = (frame_size_bytes / SHIFT_BUFFERS_SIZE);
#endif    
    /* Configure DMA MUX Source */
    //DMAMUX->CHCFG[FLEXIO_CAMERA_DMA_CHN] = DMAMUX->CHCFG[FLEXIO_CAMERA_DMA_CHN] &
    //                                        (~DMAMUX_CHCFG_SOURCE_MASK) | 
    //                                        DMAMUX_CHCFG_SOURCE(FLEXIO_CAMERA_DMA_MUX_SRC);
    /* Enable DMA channel. */
#if (CNT_SHIFTERS > 1)    
    _dmachannel.disableOnCompletion();
    _dmachannel.interruptAtCompletion();
    _dmachannel.clearComplete();

    volatile uint32_t *mux = &DMAMUX_CHCFG0 +  _dmachannel.channel;
    Serial.printf("\nDMA CR: %08X Channel: %u %08X\n", DMA_CR, _dmachannel.channel, *mux);
    dumpDMA_TCD_GC(&_dmachannel,"CH: ");
#else

    // Total length of bytes transfered
    // do it over 2 
    // first pass split into two
    _dmasettings[0].source(_pflexio->SHIFTBUF[_fshifter]);
    _dmasettings[0].destinationBuffer(p, frame_size_bytes / 2);
    _dmasettings[0].replaceSettingsOnCompletion(_dmasettings[1]);

    _dmasettings[1].source(_pflexio->SHIFTBUF[_fshifter]);
    _dmasettings[1].destinationBuffer(&p[frame_size_bytes / 8], frame_size_bytes / 2);
    _dmasettings[1].replaceSettingsOnCompletion(_dmasettings[0]);
    _dmasettings[1].disableOnCompletion();
    _dmasettings[1].interruptAtCompletion();

    _dmachannel = _dmasettings[0];

    _dmachannel.clearComplete();
#ifdef DEBUG_FLEXIO
    dumpDMA_TCD_GC(&_dmachannel," CH: ");
    dumpDMA_TCD_GC(&_dmasettings[0], " 0: ");
    dumpDMA_TCD_GC(&_dmasettings[1], " 1: ");
#endif
#endif


    _dma_state = DMA_STATE_ONE_FRAME;
    _pflexio->SHIFTSDEN = _fshifter_mask;
    _dmachannel.enable();
    
#ifdef DEBUG_FLEXIO
    Serial.printf("Flexio DMA: length: %d\n", frame_size_bytes);
#endif
    
    elapsedMillis timeout = 0;
    //while (!_dmachannel.complete()) {
    while (_dma_state == DMA_STATE_ONE_FRAME) {
        // wait - we should not need to actually do anything during the DMA transfer
        if (_dmachannel.error()) {
            Serial.println("DMA error");
            if (_pflexio->SHIFTSTAT) Serial.printf(" SHIFTSTAT %08X\n", _pflexio->SHIFTSTAT);
            Serial.flush();
            uint32_t i = _pflexio->SHIFTBUF[_fshifter];
            Serial.printf("Result: %x\n", i);


            _dmachannel.clearError();
            break;
        }
        if (timeout > 500) {
            Serial.println("Timeout waiting for DMA");
            if (_pflexio->SHIFTSTAT & _fshifter_mask) Serial.printf(" SHIFTSTAT bit was set (%08X)\n", _pflexio->SHIFTSTAT);
            Serial.printf(" DMA channel #%u\n", _dmachannel.channel);
            Serial.printf(" DMAMUX = %08X\n", *(&DMAMUX_CHCFG0 + _dmachannel.channel));
            Serial.printf(" _pflexio->SHIFTSDEN = %02X\n", _pflexio->SHIFTSDEN);
            Serial.printf(" TCD CITER = %u\n", _dmachannel.TCD->CITER_ELINKNO);
            Serial.printf(" TCD CSR = %08X\n", _dmachannel.TCD->CSR);
            break;
        }
    }
    #ifdef GC2145_USE_DEBUG_PINS
        digitalWriteFast(2, LOW);
    #endif
    arm_dcache_delete(buffer, frame_size_bytes);
#ifdef DEBUG_FLEXIO
    dumpDMA_TCD_GC(&_dmachannel,"CM: ");
#endif
//    dumpDMA_TCD_GC(&_dmasettings[0], " 0: ");
//    dumpDMA_TCD_GC(&_dmasettings[1], " 1: ");
}


void GC2145::readFrameMultiBufferFlexIO(void* buffer1, size_t size1, void* buffer2, size_t size2) 
{
    //flexio_configure(); // one-time hardware setup
    // wait for VSYNC to go high and then low with a sort of glitch filter
    const uint32_t frame_size_bytes = _width*_height*_bytesPerPixel;
    if ((size1 + size2) < frame_size_bytes) return; // not big enough to hold frame.

    elapsedMillis emWaitSOF;
    elapsedMicros emGlitch;
    for (;;) {
      if (emWaitSOF > 2000) {
        Serial.println("Timeout waiting for Start of Frame");
        return;
      }
      while ((*_vsyncPort & _vsyncMask) == 0);
      emGlitch = 0;
      while ((*_vsyncPort & _vsyncMask) != 0);
      if (emGlitch > 5) break;
    }

    _pflexio->SHIFTSTAT = _fshifter_mask; // clear any prior shift status
    _pflexio->SHIFTERR = _fshifter_mask;

    //----------------------------------------------------------------------
    // Use DMA FlexIO version
    //----------------------------------------------------------------------
    digitalWriteFast(2, HIGH);

    // Lets try like other implementation.
    //uint32_t length_uint32 = frame_size_bytes / 4;

    _dmachannel.begin();
    _dmachannel.triggerAtHardwareEvent(_dma_source);
    active_dma_camera = this;
    _dmachannel.attachInterrupt(dmaInterruptFlexIO);


    // Total length of bytes transfered
    // do it over 2 
    // first pass split each buffer into two parts
    uint32_t *p = (uint32_t *)buffer1;
    if (size1 > frame_size_bytes) size1 = frame_size_bytes;
    _dmasettings[0].source(_pflexio->SHIFTBUF[_fshifter]);
    _dmasettings[0].destinationBuffer(p, size1 / 2);
    _dmasettings[0].replaceSettingsOnCompletion(_dmasettings[1]);

    _dmasettings[1].source(_pflexio->SHIFTBUF[_fshifter]);
    _dmasettings[1].destinationBuffer(&p[size1 / 8], size1 / 2);

    uint32_t cb_left = frame_size_bytes - size1;
    if (cb_left) {
        p = (uint32_t *)buffer2;

        _dmasettings[1].TCD->CSR &= ~(DMA_TCD_CSR_DREQ | DMA_TCD_CSR_INTMAJOR); // Don't disable or interrupt on this one
    

        _dmasettings[2].source(_pflexio->SHIFTBUF[_fshifter]);
        _dmasettings[2].destinationBuffer(p, cb_left / 2);
        _dmasettings[2].replaceSettingsOnCompletion(_dmasettings[1]);

        _dmasettings[3].source(_pflexio->SHIFTBUF[_fshifter]);
        _dmasettings[3].destinationBuffer(&p[cb_left / 8], cb_left / 2);
        _dmasettings[3].replaceSettingsOnCompletion(_dmasettings[0]);
        _dmasettings[3].disableOnCompletion();
        _dmasettings[3].interruptAtCompletion();
    } else {
        _dmasettings[1].replaceSettingsOnCompletion(_dmasettings[0]);
        _dmasettings[1].disableOnCompletion();
        _dmasettings[1].interruptAtCompletion();
    }
    _dmachannel = _dmasettings[0];

    _dmachannel.clearComplete();
#ifdef DEBUG_FLEXIO
    dumpDMA_TCD_GC(&_dmachannel," CH: ");
    dumpDMA_TCD_GC(&_dmasettings[0], " 0: ");
    dumpDMA_TCD_GC(&_dmasettings[1], " 1: ");
    dumpDMA_TCD_GC(&_dmasettings[2], " 2: ");
    dumpDMA_TCD_GC(&_dmasettings[3], " 3: ");
#endif


    _dma_state = DMA_STATE_ONE_FRAME;
    _pflexio->SHIFTSDEN = _fshifter_mask;
    _dmachannel.enable();
    
#ifdef DEBUG_FLEXIO
    Serial.printf("Flexio DMA: length: %d\n", frame_size_bytes);
#endif
    
    elapsedMillis timeout = 0;
    //while (!_dmachannel.complete()) {
    while (_dma_state == DMA_STATE_ONE_FRAME) {
        // wait - we should not need to actually do anything during the DMA transfer
        if (_dmachannel.error()) {
            Serial.println("DMA error");
            if (_pflexio->SHIFTSTAT) Serial.printf(" SHIFTSTAT %08X\n", _pflexio->SHIFTSTAT);
            Serial.flush();
            uint32_t i = _pflexio->SHIFTBUF[_fshifter];
            Serial.printf("Result: %x\n", i);


            _dmachannel.clearError();
            break;
        }
        if (timeout > 500) {
            Serial.println("Timeout waiting for DMA");
            if (_pflexio->SHIFTSTAT & _fshifter_mask) Serial.printf(" SHIFTSTAT bit was set (%08X)\n", _pflexio->SHIFTSTAT);
            Serial.printf(" DMA channel #%u\n", _dmachannel.channel);
            Serial.printf(" DMAMUX = %08X\n", *(&DMAMUX_CHCFG0 + _dmachannel.channel));
            Serial.printf(" _pflexio->SHIFTSDEN = %02X\n", _pflexio->SHIFTSDEN);
            Serial.printf(" TCD CITER = %u\n", _dmachannel.TCD->CITER_ELINKNO);
            Serial.printf(" TCD CSR = %08X\n", _dmachannel.TCD->CSR);
            break;
        }
    }
    #ifdef GC2145_USE_DEBUG_PINS
        digitalWriteFast(2, LOW);
    #endif
    arm_dcache_delete(buffer1, size1);
    arm_dcache_delete(buffer2, size2);
#ifdef DEBUG_FLEXIO
    dumpDMA_TCD_GC(&_dmachannel,"CM: ");
#endif
//    dumpDMA_TCD_GC(&_dmasettings[0], " 0: ");
//    dumpDMA_TCD_GC(&_dmasettings[1], " 1: ");
}





bool GC2145::startReadFlexIO(bool(*callback)(void *frame_buffer), void *fb1, void *fb2)
{

#ifdef FLEXIO_USE_DMA
    if (fb1 == nullptr || fb2 == nullptr) return false;
    _frame_buffer_1 = (uint8_t *)fb1;
    _frame_buffer_2 = (uint8_t *)fb2;
    _callback = callback;
    active_dma_camera = this;

    //flexio_configure(); // one-time hardware setup
    _pflexio->SHIFTSTAT = _fshifter_mask; // clear any prior shift status
    _pflexio->SHIFTERR = _fshifter_mask;
    uint32_t *p = (uint32_t *)fb1;

    //----------------------------------------------------------------------
    // Use DMA FlexIO version
    //----------------------------------------------------------------------
    // Currently lets setup for only one shifter
//    digitalWriteFast(2, HIGH);

    const uint32_t frame_size_bytes = _width*_height*_bytesPerPixel;

    _dmachannel.begin();
    _dmachannel.triggerAtHardwareEvent(_dma_source);
    active_dma_camera = this;
    _dmachannel.attachInterrupt(dmaInterruptFlexIO);


    // Total length of bytes transfered
    // do it over 2 
    // first pass split into two
    _dmasettings[0].source(_pflexio->SHIFTBUF[_fshifter]);
    _dmasettings[0].destinationBuffer(p, frame_size_bytes / 2);
    _dmasettings[0].replaceSettingsOnCompletion(_dmasettings[1]);

    _dmasettings[1].source(_pflexio->SHIFTBUF[_fshifter]);
    _dmasettings[1].destinationBuffer(&p[frame_size_bytes / 8], frame_size_bytes / 2);
    _dmasettings[1].replaceSettingsOnCompletion(_dmasettings[2]);
    _dmasettings[1].interruptAtCompletion();

    // lets preset up the dmasettings for second buffer
    p = (uint32_t *)fb2;
    _dmasettings[2].source(_pflexio->SHIFTBUF[_fshifter]);
    _dmasettings[2].destinationBuffer(p, frame_size_bytes / 2);
    _dmasettings[2].replaceSettingsOnCompletion(_dmasettings[3]);

    _dmasettings[3].source(_pflexio->SHIFTBUF[_fshifter]);
    _dmasettings[3].destinationBuffer(&p[frame_size_bytes / 8], frame_size_bytes / 2);
    _dmasettings[3].replaceSettingsOnCompletion(_dmasettings[0]);
    _dmasettings[3].interruptAtCompletion();


    #ifdef USE_VSYNC_PIN_INT
    // disable when we have received a full frame. 
    _dmasettings[1].disableOnCompletion();
    _dmasettings[3].disableOnCompletion();
    #else
    _dmasettings[1].TCD->CSR &= ~(DMA_TCD_CSR_DREQ); // Don't disable on this one
    _dmasettings[3].TCD->CSR &= ~(DMA_TCD_CSR_DREQ); // Don't disable on this one
    #endif

    _dmachannel = _dmasettings[0];

    _dmachannel.clearComplete();
#ifdef DEBUG_FLEXIO

    dumpDMA_TCD_GC(&_dmachannel," CH: ");
    dumpDMA_TCD_GC(&_dmasettings[0], " 0: ");
    dumpDMA_TCD_GC(&_dmasettings[1], " 1: ");
    dumpDMA_TCD_GC(&_dmasettings[2], " 2: ");
    dumpDMA_TCD_GC(&_dmasettings[3], " 3: ");
    Serial.printf("Flexio DMA: length: %d\n", frame_size_bytes);

#endif

    _pflexio->SHIFTSTAT = _fshifter_mask; // clear any prior shift status
    _pflexio->SHIFTERR = _fshifter_mask;


    _dma_last_completed_frame = nullptr;
    _dma_frame_count = 0;

    _dma_state = DMASTATE_RUNNING;

#ifdef USE_VSYNC_PIN_INT
    // Lets use interrupt on interrupt on VSYNC pin to start the capture of a frame
    _dma_active = false;
    _vsync_high_time = 0;
    NVIC_SET_PRIORITY(IRQ_GPIO6789, 102);
    //NVIC_SET_PRIORITY(dma_flexio.channel & 0xf, 102);
    attachInterrupt(_vsyncPin, &frameStartInterruptFlexIO, RISING);
    _pflexio->SHIFTSDEN = _fshifter_mask;
#else    
    // wait for VSYNC to go high and then low with a sort of glitch filter
    elapsedMillis emWaitSOF;
    elapsedMicros emGlitch;
    for (;;) {
      if (emWaitSOF > 2000) {
        Serial.println("Timeout waiting for Start of Frame");
        return false;
      }
      while ((*_vsyncPort & _vsyncMask) == 0);
      emGlitch = 0;
      while ((*_vsyncPort & _vsyncMask) != 0);
      if (emGlitch > 2) break;
    }

    _pflexio->SHIFTSDEN = _fshifter_mask;
    _dmachannel.enable();
#endif    

    return true;
#else
    return false;
#endif
}

#ifdef USE_VSYNC_PIN_INT
void GC2145::frameStartInterruptFlexIO()
{
	active_dma_camera->processFrameStartInterruptFlexIO();
}

void GC2145::processFrameStartInterruptFlexIO()
{
  #ifdef GC2145_USE_DEBUG_PINS
  digitalWriteFast(5, HIGH);
  #endif
  //Serial.println("VSYNC");
  // See if we read the state of it a few times if the pin stays high...
  if (digitalReadFast(_vsyncPin) && digitalReadFast(_vsyncPin) && digitalReadFast(_vsyncPin) 
          && digitalReadFast(_vsyncPin) )  {
    // stop this interrupt.
    #ifdef GC2145_USE_DEBUG_PINS
    //digitalToggleFast(2);
    digitalWriteFast(2, LOW);
    digitalWriteFast(2, HIGH);
    #endif
    detachInterrupt(_vsyncPin);

    // For this pass will leave in longer DMAChain with both buffers.
  	_pflexio->SHIFTSTAT = _fshifter_mask; // clear any prior shift status
  	_pflexio->SHIFTERR = _fshifter_mask;

    _vsync_high_time = 0; // clear out the time.
    _dmachannel.clearComplete();
    _dmachannel.enable();
  }
	asm("DSB");
  #ifdef GC2145_USE_DEBUG_PINS
  digitalWriteFast(5, LOW);
  #endif
}

#endif

void GC2145::dmaInterruptFlexIO()
{
	active_dma_camera->processDMAInterruptFlexIO();
}

void GC2145::processDMAInterruptFlexIO()
{

  _dmachannel.clearInterrupt();
  #ifdef GC2145_USE_DEBUG_PINS
//  digitalToggleFast(2);
  digitalWriteFast(2, HIGH);
  digitalWriteFast(2, LOW);
  #endif
  if (_dma_state == DMA_STATE_ONE_FRAME) {
    _dma_state = DMA_STATE_STOPPED;
    asm("DSB");
    return;

  } else if (_dma_state == DMASTATE_STOP_REQUESTED) {
    _dmachannel.disable();
    _frame_buffer_1 = nullptr;
    _frame_buffer_2 = nullptr;
    _callback = nullptr;
    _dma_state = DMA_STATE_STOPPED;
    asm("DSB");
    return;
  }

#if 0
  static uint8_t debug_print_count = 8;
  if (debug_print_count) {
    debug_print_count--;
    Serial.printf("PDMAIF: %x\n", (uint32_t)_dmachannel.TCD->DADDR);
    dumpDMA_TCD_GC(&_dmachannel," CH: ");

  }
#endif  
	_dmachannel.clearComplete();
  const uint32_t frame_size_bytes = _width*_height*_bytesPerPixel;
  _dma_last_completed_frame = (((uint32_t)_dmachannel.TCD->DADDR) == (uint32_t)_frame_buffer_1)? _frame_buffer_2 : _frame_buffer_1;

	arm_dcache_delete(_dma_last_completed_frame, frame_size_bytes);

	if (_callback) (*_callback)(_dma_last_completed_frame); // TODO: use EventResponder
  _dma_active = false;
  // start up interrupt to look for next start of interrupt.
  _vsync_high_time = 0; // remember the time we were called

  if (_dma_state == DMASTATE_RUNNING) attachInterrupt(_vsyncPin, &frameStartInterruptFlexIO, RISING);

	asm("DSB");
}


bool GC2145::stopReadFlexIO()
{
  #ifdef USE_VSYNC_PIN_INT
  // first disable the vsync interrupt
  detachInterrupt(_vsyncPin);
  if (!_dma_active) {
    _dma_state = DMA_STATE_STOPPED;
  } else {
    cli();
    if (_dma_state != DMA_STATE_STOPPED) _dma_state = DMASTATE_STOP_REQUESTED;
    sei();
  }
  #else
  _dmasettings[1].disableOnCompletion();
  _dmasettings[3].disableOnCompletion();
  _dma_state = DMASTATE_STOP_REQUESTED;
  #endif
	return true;
}


//======================================== DMA JUNK
//================================================================================
// experiment with DMA
//================================================================================
// Define our DMA structure.
DMAChannel GC2145::_dmachannel;
DMASetting GC2145::_dmasettings[4];
uint32_t GC2145::_dmaBuffer1[DMABUFFER_SIZE] __attribute__ ((used, aligned(32)));
uint32_t GC2145::_dmaBuffer2[DMABUFFER_SIZE] __attribute__ ((used, aligned(32)));
extern "C" void xbar_connect(unsigned int input, unsigned int output); // in pwm.c

GC2145 *GC2145::active_dma_camera = nullptr;


//===================================================================
// Start a DMA operation -
//===================================================================
bool GC2145::startReadFrameDMA(bool(*callback)(void *frame_buffer), uint8_t *fb1, uint8_t *fb2)
{
  // First see if we need to allocate frame buffers.
  if (fb1) _frame_buffer_1 = fb1;
  else if (_frame_buffer_1 == nullptr) {
    _frame_buffer_1 = (uint8_t*)malloc(_width * _height );
    if (_frame_buffer_1 == nullptr) return false;
  }
  if (fb2) _frame_buffer_2 = fb2;
  else if (_frame_buffer_2 == nullptr) {
    _frame_buffer_2 = (uint8_t*)malloc(_width * _height);
    if (_frame_buffer_2 == nullptr) return false; // BUGBUG should we 32 byte align?
  }
  // remember the call back if passed in
  _callback = callback;
  active_dma_camera = this;

  Serial.printf("startReadFrameDMA called buffers %x %x\n", (uint32_t)_frame_buffer_1, (uint32_t)_frame_buffer_2);

  //DebugDigitalToggle(GC2145_DEBUG_PIN_1);
  // lets figure out how many bytes we will tranfer per setting...
  //  _dmasettings[0].begin();
  _frame_row_buffer_pointer = _frame_buffer_pointer = (uint8_t *)_frame_buffer_1;

  // configure DMA channels
  _dmachannel.begin();
  _dmasettings[0].source(GPIO2_PSR); // setup source.
  _dmasettings[0].destinationBuffer(_dmaBuffer1, DMABUFFER_SIZE * 4);  // 32 bits per logical byte
  _dmasettings[0].replaceSettingsOnCompletion(_dmasettings[1]);
  _dmasettings[0].interruptAtCompletion();  // we will need an interrupt to process this.
  _dmasettings[0].TCD->CSR &= ~(DMA_TCD_CSR_DREQ); // Don't disable on this one
  //DebugDigitalToggle(GC2145_DEBUG_PIN_1);

  _dmasettings[1].source(GPIO2_PSR); // setup source.
  _dmasettings[1].destinationBuffer(_dmaBuffer2, DMABUFFER_SIZE * 4);  // 32 bits per logical byte
  _dmasettings[1].replaceSettingsOnCompletion(_dmasettings[0]);
  _dmasettings[1].interruptAtCompletion();  // we will need an interrupt to process this.
  _dmasettings[1].TCD->CSR &= ~(DMA_TCD_CSR_DREQ); // Don't disable on this one
  //DebugDigitalToggle(GC2145_DEBUG_PIN_1);

  GPIO2_GDIR = 0; // set all as input...
  GPIO2_DR = 0; // see if I can clear it out...

  _dmachannel = _dmasettings[0];  // setup the first on...
  _dmachannel.attachInterrupt(dmaInterrupt);
  _dmachannel.triggerAtHardwareEvent(DMAMUX_SOURCE_XBAR1_0);
  //DebugDigitalToggle(GC2145_DEBUG_PIN_1);

  // Lets try to setup the DMA setup...
  // first see if we can convert the _pclk to be an XBAR Input pin...
    // GC2145_PLK   4
  // GC2145_PLK   8    //8       B1_00   FlexIO2:16  XBAR IO14

  _save_pclkPin_portConfigRegister = *(portConfigRegister(_pclkPin));
  *(portConfigRegister(_pclkPin)) = 1; // set to XBAR mode 14

  // route the timer outputs through XBAR to edge trigger DMA request
  CCM_CCGR2 |= CCM_CCGR2_XBAR1(CCM_CCGR_ON);
  xbar_connect(XBARA1_IN_IOMUX_XBAR_INOUT14, XBARA1_OUT_DMA_CH_MUX_REQ30);
  //DebugDigitalToggle(GC2145_DEBUG_PIN_1);

  // Tell XBAR to dDMA on Rising
  XBARA1_CTRL0 = XBARA_CTRL_STS0 | XBARA_CTRL_EDGE0(1) | XBARA_CTRL_DEN0/* | XBARA_CTRL_IEN0 */ ;

  IOMUXC_GPR_GPR6 &= ~(IOMUXC_GPR_GPR6_IOMUXC_XBAR_DIR_SEL_14);  // Make sure it is input mode
  IOMUXC_XBAR1_IN14_SELECT_INPUT = 1; // Make sure this signal goes to this pin...


#if defined (ARDUINO_TEENSY_MICROMOD)
  // Need to switch the IO pins back to GPI1 from GPIO6
  _save_IOMUXC_GPR_GPR27 = IOMUXC_GPR_GPR27;  // save away the configuration before we change...
  IOMUXC_GPR_GPR27 &= ~(0x0ff0u);

  // lets also un map the _hrefPin to GPIO1
  IOMUXC_GPR_GPR27 &= ~_hrefMask; //
#else
  // Need to switch the IO pins back to GPI1 from GPIO6
  _save_IOMUXC_GPR_GPR26 = IOMUXC_GPR_GPR26;  // save away the configuration before we change...
  IOMUXC_GPR_GPR26 &= ~(0x0ff0u);

  // lets also un map the _hrefPin to GPIO1
  IOMUXC_GPR_GPR26 &= ~_hrefMask; //
#endif

  // Need to switch the IO pins back to GPI1 from GPIO6
  //_save_IOMUXC_GPR_GPR27 = IOMUXC_GPR_GPR27;  // save away the configuration before we change...
  //IOMUXC_GPR_GPR27 &= ~(0x0ff0u);

  // lets also un map the _hrefPin to GPIO1
  //IOMUXC_GPR_GPR27 &= ~_hrefMask; //


  //DebugDigitalToggle(GC2145_DEBUG_PIN_1);

  // Falling edge indicates start of frame
//  while ((*_vsyncPort & _vsyncMask) == 0); // wait for HIGH
//  while ((*_vsyncPort & _vsyncMask) != 0); // wait for LOW
//  DebugDigitalWrite(GC2145_DEBUG_PIN_2, HIGH);

// Debug stuff for now

  // We have the start of a frame, so lets start the dma.
#ifdef DEBUG_CAMERA
  dumpDMA_TCD_GC(&_dmachannel," CH: ");
  dumpDMA_TCD_GC(&_dmasettings[0], " 0: ");
  dumpDMA_TCD_GC(&_dmasettings[1], " 1: ");

  Serial.printf("pclk pin: %d config:%lx control:%lx\n", _pclkPin, *(portConfigRegister(_pclkPin)), *(portControlRegister(_pclkPin)));
  Serial.printf("IOMUXC_GPR_GPR26-29:%lx %lx %lx %lx\n", IOMUXC_GPR_GPR26, IOMUXC_GPR_GPR27, IOMUXC_GPR_GPR28, IOMUXC_GPR_GPR29);
  Serial.printf("GPIO1: %lx %lx, GPIO6: %lx %lx\n", GPIO1_DR, GPIO1_PSR, GPIO6_DR, GPIO6_PSR);
  Serial.printf("XBAR CTRL0:%x CTRL1:%x\n\n", XBARA1_CTRL0, XBARA1_CTRL1);
#endif
  _dma_state = DMASTATE_RUNNING;
  _dma_last_completed_frame = nullptr;
  _dma_frame_count = 0;

  // Now start an interrupt for start of frame. 
//  attachInterrupt(_vsyncPin, &frameStartInterrupt, RISING);

  //DebugDigitalToggle(GC2145_DEBUG_PIN_1);
  return true;
}

//===================================================================
// stopReadFrameDMA - stop doing the reading and then exit.
//===================================================================
bool GC2145::stopReadFrameDMA()
{

  // hopefully it start here (fingers crossed)
  // for now will hang here to see if completes...
  #ifdef GC2145_USE_DEBUG_PINS
  //DebugDigitalWrite(GC2145_DEBUG_PIN_2, HIGH);
  #endif
  elapsedMillis em = 0;
  // tell the background stuff DMA stuff to exit.
  // Note: for now let it end on on, later could disable the DMA directly.
  _dma_state = DMASTATE_STOP_REQUESTED;

  while ((em < 1000) && (_dma_state == DMASTATE_STOP_REQUESTED)) ; // wait up to a second...
  if (_dma_state != DMA_STATE_STOPPED) {
    Serial.println("*** stopReadFrameDMA DMA did not exit correctly...");
    Serial.printf("  Bytes Left: %u frame buffer:%x Row:%u Col:%u\n", _bytes_left_dma, (uint32_t)_frame_buffer_pointer, _frame_row_index, _frame_col_index);
  }
  #ifdef GC2145_USE_DEBUG_PINS
  //DebugDigitalWrite(GC2145_DEBUG_PIN_2, LOW);
  #endif
#ifdef DEBUG_CAMERA
  dumpDMA_TCD_GC(&_dmachannel, nullptr);
  dumpDMA_TCD_GC(&_dmasettings[0], nullptr);
  dumpDMA_TCD_GC(&_dmasettings[1], nullptr);
  Serial.println();
#endif
  // Lets restore some hardware pieces back to the way we found them.
#if defined (ARDUINO_TEENSY_MICROMOD)
  IOMUXC_GPR_GPR27 = _save_IOMUXC_GPR_GPR27;  // Restore... away the configuration before we change...
#else
  IOMUXC_GPR_GPR26 = _save_IOMUXC_GPR_GPR26;  // Restore... away the configuration before we change...
#endif
  *(portConfigRegister(_pclkPin)) = _save_pclkPin_portConfigRegister;

  return (em < 1000); // did we stop...
}

//===================================================================
// Our Frame Start interrupt.
//===================================================================
#if 0
void  GC2145::frameStartInterrupt() {
  active_dma_camera->processFrameStartInterrupt();  // lets get back to the main object...
}

void  GC2145::processFrameStartInterrupt() {
  _bytes_left_dma = (_width + _frame_ignore_cols) * _height; // for now assuming color 565 image...
  _dma_index = 0;
  _frame_col_index = 0;  // which column we are in a row
  _frame_row_index = 0;  // which row
  _save_lsb = 0xffff;
  // make sure our DMA is setup properly again. 
  _dmasettings[0].transferCount(DMABUFFER_SIZE);
  _dmasettings[0].TCD->CSR &= ~(DMA_TCD_CSR_DREQ); // Don't disable on this one
  _dmasettings[1].transferCount(DMABUFFER_SIZE);
  _dmasettings[1].TCD->CSR &= ~(DMA_TCD_CSR_DREQ); // Don't disable on this one
  _dmachannel = _dmasettings[0];  // setup the first on...
  _dmachannel.enable();
  
  detachInterrupt(_vsyncPin);
}
#endif

//===================================================================
// Our DMA interrupt.
//===================================================================
void GC2145::dmaInterrupt() {
  active_dma_camera->processDMAInterrupt();  // lets get back to the main object...
}


// This version assumes only called when HREF...  as set pixclk to only fire
// when set.
void GC2145::processDMAInterrupt() {
  _dmachannel.clearInterrupt(); // tell system we processed it.
  asm("DSB");
  #ifdef GC2145_USE_DEBUG_PINS
  //DebugDigitalWrite(GC2145_DEBUG_PIN_3, HIGH);
  #endif
  
  if (_dma_state == DMA_STATE_STOPPED) {
    Serial.println("GC2145::dmaInterrupt called when DMA_STATE_STOPPED");
    return; //
  }


  // lets guess which buffer completed.
  uint32_t *buffer;
  uint16_t buffer_size;
  _dma_index++;
  if (_dma_index & 1) {
    buffer = _dmaBuffer1;
    buffer_size = _dmasettings[0].TCD->CITER;

  } else {
    buffer = _dmaBuffer2;
    buffer_size = _dmasettings[1].TCD->CITER;
  }
  // lets try dumping a little data on 1st 2nd and last buffer.
#ifdef DEBUG_CAMERA_VERBOSE
  if ((_dma_index < 3) || (buffer_size  < DMABUFFER_SIZE)) {
    Serial.printf("D(%d, %d, %lu) %u : ", _dma_index, buffer_size, _bytes_left_dma, pixformat);
    for (uint16_t i = 0; i < 8; i++) {
      uint16_t b = buffer[i] >> 4;
      Serial.printf(" %lx(%02x)", buffer[i], b);
    }
    Serial.print("...");
    for (uint16_t i = buffer_size - 8; i < buffer_size; i++) {
      uint16_t b = buffer[i] >> 4;
      Serial.printf(" %lx(%02x)", buffer[i], b);
    }
    Serial.println();
  }
#endif

  for (uint16_t buffer_index = 0; buffer_index < buffer_size; buffer_index++) {
    if (!_bytes_left_dma || (_frame_row_index >= _height)) break;

    // only process if href high...
    uint16_t b = *buffer >> 4;
    *_frame_buffer_pointer++ = b;
    _frame_col_index++;
    if (_frame_col_index == _width) {
        // we just finished a row.
        _frame_row_index++;
        _frame_col_index = 0;
    }
    _bytes_left_dma--; // for now assuming color 565 image...
    buffer++;
  }

  if ((_frame_row_index == _height) || (_bytes_left_dma == 0)) { // We finished a frame lets bail
    _dmachannel.disable();  // disable the DMA now...
    #ifdef GC2145_USE_DEBUG_PINS
    //DebugDigitalWrite(GC2145_DEBUG_PIN_2, LOW);
    #endif
#ifdef DEBUG_CAMERA_VERBOSE
    Serial.println("EOF");
#endif
    _frame_row_index = 0;
    _dma_frame_count++;

    bool swap_buffers = true;

    //DebugDigitalToggle(GC2145_DEBUG_PIN_1);
    _dma_last_completed_frame = _frame_row_buffer_pointer;
    if (_callback) swap_buffers = (*_callback)(_dma_last_completed_frame);

    if (swap_buffers) {
        if (_frame_row_buffer_pointer != _frame_buffer_1) _frame_row_buffer_pointer = _frame_buffer_2;
        else _frame_row_buffer_pointer = _frame_buffer_2;    
    }

    _frame_buffer_pointer = _frame_row_buffer_pointer;

    //DebugDigitalToggle(GC2145_DEBUG_PIN_1);


    if (_dma_state == DMASTATE_STOP_REQUESTED) {
#ifdef DEBUG_CAMERA
      Serial.println("GC2145::dmaInterrupt - Stop requested");
#endif
      _dma_state = DMA_STATE_STOPPED;
    } else {
      // We need to start up our ISR for the next frame. 
#if 1
  // bypass interrupt and just restart DMA... 
  _bytes_left_dma = (_width + _frame_ignore_cols) * _height; // for now assuming color 565 image...
  _dma_index = 0;
  _frame_col_index = 0;  // which column we are in a row
  _frame_row_index = 0;  // which row
  _save_lsb = 0xffff;
  // make sure our DMA is setup properly again. 
  _dmasettings[0].transferCount(DMABUFFER_SIZE);
  _dmasettings[0].TCD->CSR &= ~(DMA_TCD_CSR_DREQ); // Don't disable on this one
  _dmasettings[1].transferCount(DMABUFFER_SIZE);
  _dmasettings[1].TCD->CSR &= ~(DMA_TCD_CSR_DREQ); // Don't disable on this one
  _dmachannel = _dmasettings[0];  // setup the first on...
  _dmachannel.enable();

#else
      attachInterrupt(_vsyncPin, &frameStartInterrupt, RISING);
#endif
    }
  } else {

    if (_bytes_left_dma == (2 * DMABUFFER_SIZE)) {
      if (_dma_index & 1) _dmasettings[0].disableOnCompletion();
      else _dmasettings[1].disableOnCompletion();
    }

  }
  #ifdef GC2145_USE_DEBUG_PINS
  //DebugDigitalWrite(GC2145_DEBUG_PIN_3, LOW);
  #endif
}

typedef struct {
    uint32_t frameTimeMicros;
    uint16_t vsyncStartCycleCount;
    uint16_t vsyncEndCycleCount;
    uint16_t hrefCount;
    uint32_t cycleCount;
    uint16_t pclkCounts[350]; // room to spare.
    uint32_t hrefStartTime[350];
    uint16_t pclkNoHrefCount;
} frameStatics_t;

frameStatics_t fstat_gc;

void GC2145::captureFrameStatistics()
{
   memset((void*)&fstat_gc, 0, sizeof(fstat_gc));

   // lets wait for the vsync to go high;
    while ((*_vsyncPort & _vsyncMask) != 0); // wait for HIGH
    // now lets wait for it to go low    
    while ((*_vsyncPort & _vsyncMask) == 0) fstat_gc.vsyncStartCycleCount ++; // wait for LOW

    while ((*_hrefPort & _hrefMask) == 0); // wait for HIGH
    while ((*_pclkPort & _pclkMask) != 0); // wait for LOW

    uint32_t microsStart = micros();
    fstat_gc.hrefStartTime[0] = microsStart;
    // now loop through until we get the next _vsynd
    // BUGBUG We know that HSYNC and PCLK on same GPIO VSYNC is not...
    uint32_t regs_prev = 0;
    //noInterrupts();
    while ((*_vsyncPort & _vsyncMask) != 0) {

        fstat_gc.cycleCount++;
        uint32_t regs = (*_hrefPort & (_hrefMask | _pclkMask ));
        if (regs != regs_prev) {
            if ((regs & _hrefMask) && ((regs_prev & _hrefMask) ==0)) {
                fstat_gc.hrefCount++;
                fstat_gc.hrefStartTime[fstat_gc.hrefCount] = micros();
            }
            if ((regs & _pclkMask) && ((regs_prev & _pclkMask) ==0)) fstat_gc.pclkCounts[fstat_gc.hrefCount]++;
            if ((regs & _pclkMask) && ((regs_prev & _hrefMask) ==0)) fstat_gc.pclkNoHrefCount++;
            regs_prev = regs;
        }
    }
    while ((*_vsyncPort & _vsyncMask) == 0) fstat_gc.vsyncEndCycleCount++; // wait for LOW
    //interrupts();
    fstat_gc.frameTimeMicros = micros() - microsStart;

    // Maybe return data. print now
    Serial.printf("*** Frame Capture Data: elapsed Micros: %u loops: %u\n", fstat_gc.frameTimeMicros, fstat_gc.cycleCount);
    Serial.printf("   VSync Loops Start: %u end: %u\n", fstat_gc.vsyncStartCycleCount, fstat_gc.vsyncEndCycleCount);
    Serial.printf("   href count: %u pclk ! href count: %u\n    ", fstat_gc.hrefCount,  fstat_gc.pclkNoHrefCount);
    for (uint16_t ii=0; ii < fstat_gc.hrefCount + 1; ii++) {
        Serial.printf("%3u(%u) ", fstat_gc.pclkCounts[ii], (ii==0)? 0 : fstat_gc.hrefStartTime[ii] - fstat_gc.hrefStartTime[ii-1]);
        if (!(ii & 0x0f)) Serial.print("\n    ");
    }
    Serial.println();
}
