#include "GC2145.h"

#include <Wire.h>

#define FLEXIO_TIMER_TRIGGER_SEL_PININPUT(x) ((uint32_t)(x) << 1U)

/*
 * This file is part of the OpenMV project.
 *
 * Copyright (c) 2013-2021 Ibrahim Abdelkader <iabdalkader@openmv.io>
 * Copyright (c) 2013-2021 Kwabena W. Agyeman <kwagyeman@openmv.io>
 *
 * This work is licensed under the MIT license, see the file LICENSE for
 * details.
 *
 * GC2145 driver.
 */

/*
 * This file modified to work with Teensy 4.x with Flexio, GPIO, etc
 * https://github.com/openmv/openmv/blob/47e3a567d560c5340ce582bfe78199102fff5775/src/omv/common/sensor_utils.c#L46
 * some comes from Arduino_dvp
 */

#define debug Serial

// #define DEBUG_CAMERA
// #define DEBUG_CAMERA_VERBOSE
// #define DEBUG_FLEXIO
// #define USE_DEBUG_PINS

#define camAddress 0x3C

#define BLANK_LINES 16
#define DUMMY_LINES 16

#define BLANK_COLUMNS 0
#define DUMMY_COLUMNS 8

#define SENSOR_WIDTH 1616
#define SENSOR_HEIGHT 1248

#define ACTIVE_SENSOR_WIDTH (SENSOR_WIDTH - BLANK_COLUMNS - (2 * DUMMY_COLUMNS))
#define ACTIVE_SENSOR_HEIGHT (SENSOR_HEIGHT - BLANK_LINES - (2 * DUMMY_LINES))

#define DUMMY_WIDTH_BUFFER 16
#define DUMMY_HEIGHT_BUFFER 8

static int16_t readout_x = 0;
static int16_t readout_y = 0;

static uint16_t readout_w = ACTIVE_SENSOR_WIDTH;
static uint16_t readout_h = ACTIVE_SENSOR_HEIGHT;

static bool fov_wide = true;

#define REG_AMODE1 (0x17)
#define REG_AMODE1_DEF (0x14)
#define REG_AMODE1_SET_HMIRROR(r, x) ((r & 0xFE) | ((x & 1) << 0))
#define REG_AMODE1_SET_VMIRROR(r, x) ((r & 0xFD) | ((x & 1) << 1))

#define REG_OUTPUT_FMT (0x84)
#define REG_OUTPUT_FMT_RGB565 (0x06)
#define REG_OUTPUT_FMT_YCBYCR (0x02)
#define REG_OUTPUT_FMT_BAYER (0x17)
#define REG_OUTPUT_SET_FMT(r, x) ((r & 0xE0) | (x))

#define REG_SYNC_MODE (0x86)
#define REG_SYNC_MODE_DEF (0x03)
#define REG_SYNC_MODE_COL_SWITCH (0x10)
#define REG_SYNC_MODE_ROW_SWITCH (0x20)

// Sensor frame size/resolution table.
const int resolution[][2] = {
    {640, 480},   /* VGA       */
    {160, 120},   /* QQVGA     */
    {320, 240},   /* QVGA      */
    {480, 320},   /* ILI9488   */
    {320, 320},   /* 320x320   */
    {320, 240},   /* QVGA      */
    {176, 144},   /* QCIF      */
    {352, 288},   /* CIF       */
    {800, 600},   /* SVGA      */
    {1600, 1200}, /* UXGA      */
    {0, 0},
};

static const uint8_t default_regs[][2] = {
    {0xfe, 0xf0},
    {0xfe, 0xf0},
    {0xfe, 0xf0},

    {0xfc, 0x06},
    {0xf6, 0x00},

    {0xf7, 0x1d}, // 37 //17 //37 //1d//05
    {0xf8, 0x83}, // 87 //83 //82
    {0xfa, 0x00},
    {0xf9, 0xfe}, // ff
    {0xfd, 0x00},
    {0xc2, 0x00},
    {0xf2, 0x0f},
    //////////////////////////////////////////////////////
    ////////////////////  Analog & Cisctl ////////////////
    //////////////////////////////////////////////////////
    {0xfe, 0x00},

    {0x03, 0x04}, // exp time
    {0x04, 0x62}, // exp time

    {0x05, 0x01}, // 00 //hb[11:8]
    {0x06, 0x3b}, // 0b //hb

    {0x09, 0x00}, // row start
    {0x0a, 0x00}, //
    {0x0b, 0x00}, // col start
    {0x0c, 0x00},
    {0x0d, 0x04}, // height
    {0x0e, 0xc0},
    {0x0f, 0x06}, // width
    {0x10, 0x52},

    {0x12, 0x2e}, // sh_delay 太短 YUV出图异常
    {0x17, 0x14}, // CISCTL Mode1 [1:0]mirror flip
    {0x18, 0x22}, // sdark mode
    {0x19, 0x0f}, // AD pipe number
    {0x1a, 0x01}, // AD manual switch mode

    {0x1b, 0x4b}, // 48 restg Width,SH width
    {0x1c, 0x07}, // 06  帧率快后，横条纹 //12 //TX Width,Space Width
    {0x1d, 0x10}, // double reset
    {0x1e, 0x88}, // 90//98 //fix  竖线//Analog Mode1,TX high,Coln_r
    {0x1f, 0x78}, // 78 //38 //18 //Analog Mode2,txlow
    {0x20, 0x03}, // 07 //Analog Mode3,comv,ad_clk mode
    {0x21, 0x40}, // 10//20//40 //fix 灯管横条纹
    {0x22, 0xa0}, // d0//f0 //a2 //Vref vpix  FPN严重
    {0x24, 0x1e},
    {0x25, 0x01}, // col sel
    {0x26, 0x10}, // Analog PGA gain1
    {0x2d, 0x60}, // 40//40 //txl drv mode
    {0x30, 0x01}, // Analog Mode4
    {0x31, 0x90}, // b0//70 // Analog Mode7 [7:5]rsgh_r灯管横条纹[4:3]isp_g
    {0x33, 0x06}, // 03//02//01 //EQ_hstart_width
    {0x34, 0x01},
    //
    ///////////////////////////////////////////////////
    ////////////////////  ISP reg  //////////////////////
    //////////////////////////////////////////////////////
    {0x80, 0xff}, // outdoor gamma_en, GAMMA_en, CC_en, EE_en, INTP_en, DN_en,
                  // DD_en,LSC_en
    {0x81, 0x24}, // 26//24 //BLK dither mode, ll_y_en ,skin_en, edge SA,
                  // new_skin_mode, autogray_en,ll_gamma_en,BFF test image
    {0x82, 0xfa}, // FA //auto_SA, auto_EE, auto_DN, auto_DD, auto_LSC, ABS_en,
                  // AWB_en, NA
    {0x83, 0x00}, // special_effect
    {0x84, 0x02}, // output format
    {0x86, 0x03}, // c2 //46 //c2 //sync mode
    {0x88, 0x03}, //[1]ctl_auto_gating [0]out_auto_gating
    {0x89, 0x03}, // bypass disable
    {0x85, 0x30}, // 60//frame start cut
    {0x8a,
     0x00}, // ISP_quiet_mode,close aaa pclk,BLK gate mode,exception,close first
            // pipe clock,close dndd clock,close intp clock,DIV_gatedclk_en
    {0x8b,
     0x00}, //[7:6]BFF_gate_mode,[5]BLK switch gain,[4]protect exp,[3:2]pipe
            // gate mode,[1]not split sram,[0]dark current update

    {0xb0, 0x55}, // 60 //global gain
    {0xc3, 0x00}, //[7:4]auto_exp_gamma_th1[11:8],[3:0]auto_exp_gamma_th2[11:8]
    {0xc4, 0x80}, // auto_exp_gamma_th1[7:0] into
    {0xc5, 0x90}, // auto_exp_gamma_th2[7:0] out //outdoor gamma
    {0xc6, 0x38}, // auto_gamma_th1
    {0xc7, 0x40}, // auto_gamma_th2

    {0xec, 0x06}, // measure window
    {0xed, 0x04},
    {0xee, 0x60}, // 16  col
    {0xef, 0x90}, // 8  row

    {0xb6, 0x01}, //[0]aec en

    {0x90, 0x01}, // crop
    {0x91, 0x00},
    {0x92, 0x00},
    {0x93, 0x00},
    {0x94, 0x00}, // 08
    {0x95, 0x04},
    {0x96, 0xb0},
    {0x97, 0x06},
    {0x98, 0x40},

    ///////////////////////////////////////////////
    ///////////  BLK ////////////////////////
    ///////////////////////////////////////////////
    {0x18, 0x02},
    {0x40, 0x42}, // 2b //27
    {0x41, 0x00}, // 80 //dark row sel
    {0x43, 0x54}, //[7:4]BLK start not smooth  [3:0]output start frame

    {0x5e, 0x00}, // 00//10 //18
    {0x5f, 0x00}, // 00//10 //18
    {0x60, 0x00}, // 00//10 //18
    {0x61, 0x00}, // 00///10 //18
    {0x62, 0x00}, // 00//10 //18
    {0x63, 0x00}, // 00//10 //18
    {0x64, 0x00}, // 00/10 //18
    {0x65, 0x00}, // 00//10 //18
    {0x66, 0x20}, // 1e
    {0x67, 0x20}, // 1e
    {0x68, 0x20}, // 1e
    {0x69, 0x20}, // 1e

    {0x76, 0x00}, // 0f

    {0x6a, 0x00}, // 06
    {0x6b, 0x00}, // 06
    {0x6c, 0x3e}, // 06
    {0x6d, 0x3e}, // 06
    {0x6e, 0x3f}, // 06
    {0x6f, 0x3f}, // 06
    {0x70, 0x00}, // 06
    {0x71, 0x00}, // 06 //manual offset

    {0x76, 0x00}, // 1f//add offset
    {0x72, 0xf0}, //[7:4]BLK DD th [3:0]BLK various th
    {0x7e, 0x3c}, // ndark
    {0x7f, 0x00},

    {0xfe, 0x02},
    {0x48, 0x15},
    {0x49, 0x00}, // 04//04 //ASDE OFFSET SLOPE
    {0x4b, 0x0b}, // ASDE y OFFSET SLOPE
    {0xfe, 0x00},

    ///////////////////////////////////////////////
    /////////// AEC ////////////////////////
    ///////////////////////////////////////////////
    {0xfe, 0x01},

    {0x01, 0x04}, // AEC X1
    {0x02, 0xc0}, // AEC X2
    {0x03, 0x04}, // AEC Y1
    {0x04, 0x90}, // AEC Y2
    {0x05, 0x30}, // 20 //AEC center X1
    {0x06, 0x90}, // 40 //AEC center X2
    {0x07, 0x20}, // 30 //AEC center Y1
    {0x08, 0x70}, // 60 //AEC center Y2

    {0x09, 0x00}, // AEC show mode
    {0x0a, 0xc2}, //[7]col gain enable
    {0x0b, 0x11}, // AEC every N
    {0x0c, 0x10}, // AEC_mode3 center weight
    {0x13, 0x40}, // 2a //AEC Y target
    {0x17, 0x00}, // AEC ignore mode
    {0x1c, 0x11}, //
    {0x1e, 0x61}, //
    {0x1f, 0x30}, // 40//50 //max pre gain
    {0x20, 0x40}, // 60//40 //max post gain
    {0x22, 0x80}, // AEC outdoor THD
    {0x23, 0x20}, // target_Y_low_limit
    {0xfe, 0x02},
    {0x0f, 0x04}, // 05
    {0xfe, 0x01},

    {0x12, 0x35}, // 35 //[5:4]group_size [3]slope_disable [2]outdoor_enable
                  // [0]histogram_enable
    {0x15, 0x50}, // target_Y_high_limit
    {0x10, 0x31}, // num_thd_high
    {0x3e, 0x28}, // num_thd_low
    {0x3f, 0xe0}, // luma_thd
    {0x40, 0x20}, // luma_slope
    {0x41, 0x0f}, // color_diff

    {0xfe, 0x02},
    {0x0f, 0x05}, // max_col_level
    ///////////////////////////
    ////// INTPEE /////////////
    ///////////////////////////
    {0xfe, 0x02}, // page2
    {0x90, 0x6c}, // ac //eeintp mode1
    {0x91, 0x03}, // 02 ////eeintp mode2
    {0x92, 0xc8}, // 44 //low criteria for direction
    {0x94, 0x66},
    {0x95, 0xb5},
    {0x97, 0x64}, // 78 ////edge effect
    {0xa2, 0x11}, // fix direction
    {0xfe, 0x00},

    /////////////////////////////
    //////// DNDD///////////////
    /////////////////////////////
    {0xfe, 0x02},
    {0x80, 0xc1}, // c1 //[7]share mode [6]skin mode  [5]is 5x5 mode [1:0]noise
                  // value select 0:2  1:2.5  2:3  3:4
    {0x81, 0x08}, //
    {0x82, 0x08}, // signal a 0.6
    {0x83, 0x08}, // 04 //signal b 2.5

    {0x84, 0x0a}, // 10 //05 dark_DD_TH
    {0x86, 0xf0}, // a0 Y_value_dd_th2
    {0x87, 0x50}, // 90 Y_value_dd_th3
    {0x88, 0x15}, // 60 Y_value_dd_th4

    {0x89, 0x50}, // 80  // asde th2
    {0x8a, 0x30}, // 60  // asde th3
    {0x8b, 0x10}, // 30  // asde th4

    /////////////////////////////////////////////////
    ///////////// ASDE ////////////////////////
    /////////////////////////////////////////////////
    {0xfe, 0x01}, // page 1
    {0x21,
     0x14}, // luma_value_div_sel(分频，与0xef呈2倍关系，增大1，0xef的值减小1倍)
    // ff  ef  luma_value read_only

    {0xfe, 0x02}, // page2
    {0xa3, 0x40}, // ASDE_low_luma_value_LSC_th_H
    {0xa4, 0x20}, // ASDE_low_luma_value_LSC_th_L

    {0xa5, 0x40}, // 80 //ASDE_LSC_gain_dec_slope_H
    {0xa6, 0x80}, // 80 //ASDE_LSC_gain_dec_slope_L
                  // ff  a7  ASDE_LSC_gain_dec  //read only

    {0xab, 0x40}, // 50 //ASDE_low_luma_value_OT_th

    {0xae, 0x0c}, //[3]EE1_effect_inc_or_dec_high,[2]EE2_effect_inc_or_dec_high,
    //[1]EE1_effect_inc_or_dec_low,[0]EE2_effect_inc_or_dec_low,  1:inc  0:dec

    {0xb3, 0x34}, // 44 //ASDE_EE1_effect_slope_low,ASDE_EE2_effect_slope_low
    {0xb4, 0x44}, // 12 //ASDE_EE1_effect_slope_high,ASDE_EE2_effect_slope_high

    {0xb6, 0x38}, // 40//40 //ASDE_auto_saturation_dec_slope
    {0xb7, 0x02}, // 04 //ASDE_sub_saturation_slope
    {0xb9, 0x30}, //[7:0]ASDE_auto_saturation_low_limit
    {0x3c, 0x08}, //[3:0]auto gray_dec_slope
    {0x3d, 0x30}, //[7:0]auto gray_dec_th

    {0x4b, 0x0d}, // y offset slope
    {0x4c, 0x20}, // y offset limit

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
    {0xd1, 0x30}, // 32 //
    {0xd2, 0x30}, // 32 //
    {0xd3, 0x45},
    {0xdd, 0x14}, // edge sa
    {0xde, 0x86}, // asde auto gray
    {0xed, 0x01}, //
    {0xee, 0x28},
    {0xef, 0x30},
    {0xd8, 0xd8}, // autogray protecy

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
    {0x4e, 0x02}, // light

    {0x4c, 0x01},
    {0x4d, 0xed},
    {0x4e, 0x33}, // light
    {0x4c, 0x01},
    {0x4d, 0xcd},
    {0x4e, 0x33}, // light
    {0x4c, 0x01},
    {0x4d, 0xec},
    {0x4e, 0x03}, // light

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
    {0x4e, 0x03}, // dark
    {0x4c, 0x01},
    {0x4d, 0xcf},
    {0x4e, 0x03}, // dark

    {0x4c, 0x01},
    {0x4d, 0xca},
    {0x4e, 0x04}, // light
    {0x4c, 0x02},
    {0x4d, 0x0b},
    {0x4e, 0x05}, // light
    {0x4c, 0x02},
    {0x4d, 0xc8},
    {0x4e, 0x06}, // light 100lux
    {0x4c, 0x02},
    {0x4d, 0xa8},

    {0x4e, 0x06}, // light
    {0x4c, 0x02},
    {0x4d, 0xa9},
    {0x4e, 0x06}, // light

    {0x4c, 0x02},
    {0x4d, 0x89},
    {0x4e, 0x06}, // 400lux
    {0x4c, 0x02},
    {0x4d, 0x69},
    {0x4e, 0x06}, // f12
    {0x4c, 0x02},
    {0x4d, 0x6a},
    {0x4e, 0x06}, // f12
    {0x4c, 0x02},
    {0x4d, 0xc7},
    {0x4e, 0x07},
    {0x4c, 0x02},
    {0x4d, 0xe7},
    {0x4e, 0x07}, // 100lux
    {0x4c, 0x03},
    {0x4d, 0x07},
    {0x4e, 0x07}, // light

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
    {0x50, 0x80}, // AWB_PRE_mode
    {0x51, 0xa8}, // AWB_pre_THD_min[7:0]
    {0x52, 0x57}, // AWB_pre_THD_min[15:8] Dominiate luma 0.25=639c 0.22=57a8
    {0x53, 0x38}, // AWB_pre_THD_min_MIX[7:0]
    {0x54, 0xc7}, // AWB_pre_THD_min_MIX[15:8] Mix luma 0.5

    {0x56, 0x0e}, // AWB_tone mode
    {0x58, 0x08}, // AWB_C_num_sel,AWB_D_num_sel
    {0x5b, 0x00}, // AWB_mix_mode

    {0x5c, 0x74}, // green_num0[7:0]
    {0x5d, 0x8b}, // green_num0[15:8] 0.35

    {0x61, 0xd3}, // R2G_stand0
    {0x62, 0xb5}, // B2G_stand0
    {0x63, 0x00}, // 88//a4 //AWB gray mode [7]enable
    {0x65, 0x04}, // AWB margin

    {0x67, 0xb2}, // R2G_stand3[7:0]  FF/CWF
    {0x68, 0xac}, // B2G_stand3[7:0]
    {0x69,
     0x00},       // R2G_stand4[9:8] B2G_stand4[9:8] R2G_stand3[9:8] B2G_stand3[9:8]
    {0x6a, 0xb2}, // R2G_stand4[7:0]  TL84/TL84&CWF
    {0x6b, 0xac}, // B2G_stand4[7:0]
    {0x6c, 0xb2}, // R2G_stand5[7:0]  A
    {0x6d, 0xac}, // B2G_stand5[7:0]
    {0x6e, 0x40}, // AWB_skin_weight R2G_stand5[9:8] B2G_stand5[9:8]
    {0x6f, 0x18}, // AWB_indoor_THD (0x21=17 caculate)
    {0x73, 0x00}, // AWB_indoor_mode

    {0x70, 0x10}, // AWB low luma TH
    {0x71, 0xe8}, // AWB outdoor TH
    {0x72, 0xc0}, // outdoor mode
    {0x74, 0x01}, //[2:0]AWB skip mode 2x2,4x4,4x8,8x8
    {0x75, 0x01}, //[1:0]AWB_every_N
    {0x7f, 0x08}, //[3]gray world frame start

    {0x76, 0x70}, // R limit
    {0x77, 0x58}, // G limit
    {0x78, 0xa0}, // d8 //B limit

    {0xfe, 0x00},
    //
    //////////////////////////////////////////
    ///////////  CC   ////////////////////////
    //////////////////////////////////////////
    {0xfe, 0x02},

    {0xc0, 0x01}, //[5:4] CC mode [0]CCT enable

    {0xC1, 0x50}, // D50/D65
    {0xc2, 0xF9},
    {0xc3, 0x00}, // 0
    {0xc4, 0xe8}, // e0
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

    {0x05, 0x01}, // hb
    {0x06, 0x3b},
    {0x07, 0x01}, // Vb
    {0x08, 0x0b},

    {0xfe, 0x01},
    {0x25, 0x01},
    {0x26, 0x32}, // step
    {0x27, 0x03}, // 8.15fps
    {0x28, 0x96},
    {0x29, 0x03}, // 8.15fps
    {0x2a, 0x96},
    {0x2b, 0x03}, // 8.15fps
    {0x2c, 0x96},
    {0x2d, 0x04}, // 8.15fps
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

    {0x05, 0x01}, // hb
    {0x06, 0x18},
    {0x07, 0x00}, // Vb
    {0x08, 0x2e},

    {0xfe, 0x01},
    {0x25, 0x00},
    {0x26, 0xa2}, // step
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

    {0x09, 0x01}, // row start
    {0x0a, 0xd0}, //
    {0x0b, 0x02}, // col start
    {0x0c, 0x70},
    {0x0d, 0x01}, // height
    {0x0e, 0x00},
    {0x0f, 0x01}, // width
    {0x10, 0x50},

    {0x90, 0x01}, // crop
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
/* 640X480 VGA,30fps*/
static const uint8_t gc2145_setting_vga[][2] = {
    // SENSORDB("GC2145_Sensor_VGA"},
    {0xfe, 0x00},
    {0xb6, 0x01},
    {0xfd, 0x01},
    {0xfa, 0x00},
    /* crop window */
    {0xfe, 0x00},
    {0x90, 0x01},
    {0x91, 0x00},
    {0x92, 0x00},
    {0x93, 0x00},
    {0x94, 0x00},
    {0x95, 0x01},
    {0x96, 0xe0},
    {0x97, 0x02},
    {0x98, 0x80},
    {0x99, 0x55},
    {0x9a, 0x06},
    {0x9b, 0x01},
    {0x9c, 0x23},
    {0x9d, 0x00},
    {0x9e, 0x00},
    {0x9f, 0x01},
    {0xa0, 0x23},
    {0xa1, 0x00},
    {0xa2, 0x00},
    /* Auto White Balance */
    {0xfe, 0x00},
    {0xec, 0x02},
    {0xed, 0x02},
    {0xee, 0x30},
    {0xef, 0x48},
    {0xfe, 0x02},
    {0x9d, 0x08},
    {0xfe, 0x01},
    {0x74, 0x00},
    /* Automatic Exposure Control */
    {0xfe, 0x01},
    {0x01, 0x04},
    {0x02, 0x60},
    {0x03, 0x02},
    {0x04, 0x48},
    {0x05, 0x18},
    {0x06, 0x50},
    {0x07, 0x10},
    {0x08, 0x38},
    // {0x0a, 0xc0},//[1:0]AEC Skip
    {0x0a, 0x80}, //[1:0]AEC Skip
    {0x21, 0x04},
    {0xfe, 0x00},
    {0x20, 0x03},
    {0xfe, 0x00},
    {0x00, 0x00},
};

// Constructor
GC2145::GC2145() : _GC2145(NULL), _frame_buffer_pointer(NULL) {
}

void GC2145::end() {
    endXClk();

    pinMode(_xclkPin, INPUT);

    _wire->end();
}

int GC2145::bitsPerPixel() const {
    if (_grayscale) {
        return 8;
    } else {
        return _bytesPerPixel * 8;
    }
}

int GC2145::bytesPerPixel() const {
    if (_grayscale) {
        return 1;
    } else {
        return _bytesPerPixel;
    }
}

//*****************************************************************************
uint16_t GC2145::getModelid() {
    uint8_t Data;
    uint16_t MID = 0x0000;
    cameraWriteRegister(0xFE, 0x00);
    Data = cameraReadRegister(0XF0);
    MID = (Data << 8);
    Data = cameraReadRegister(0XF1);
    MID |= Data;
    return MID;
}

// bool GC2145::begin(framesize_t resolution, int format, bool use_gpio)
bool GC2145::begin_omnivision(framesize_t resolution, pixformat_t format,
                              int fps, int camera_name, bool use_gpio) {

    _use_gpio = use_gpio;

    // WIP - Need set functions:
    if (_rst != 0xff) {
        if (_rst_init >= 0) {
            pinMode(_rst, OUTPUT);
            digitalWrite(_rst, _rst_init);
        } else if (_rst_init == -1)
            pinMode(_rst, INPUT);
        else if (_rst_init == -2)
            pinMode(_rst, INPUT_PULLUP);
        else if (_rst_init == -3)
            pinMode(_rst, INPUT_PULLDOWN);
        delay(5);
    }

    if (_pwdn != 0xff) {
        if (_pwdn_init >= 0) {
            pinMode(_pwdn, OUTPUT);
            digitalWrite(_pwdn, _pwdn_init);
        } else if (_pwdn_init == -1)
            pinMode(_pwdn, INPUT);
        else if (_pwdn_init == -2)
            pinMode(_pwdn, INPUT_PULLUP);
        else if (_pwdn_init == -3)
            pinMode(_pwdn, INPUT_PULLDOWN);
        delay(5);
    }

// BUGBUG::: see where frame is
#ifdef USE_DEBUG_PINS
    pinMode(49, OUTPUT);
#endif

    _wire->begin();

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
        return false;
    }

    pinMode(_vsyncPin, INPUT_PULLDOWN);
    //  const struct digital_pin_bitband_and_config_table_struct *p;
    //  p = digital_pin_to_info_PGM + _vsyncPin;
    //  *(p->pad) = IOMUXC_PAD_DSE(7) | IOMUXC_PAD_HYS;  // See if I turn on
    //  HYS...
    pinMode(_hrefPin, INPUT);
    pinMode(_pclkPin, INPUT_PULLDOWN);
    pinMode(_xclkPin, OUTPUT);
#ifdef DEBUG_CAMERA
    debug.printf("  VS=%d, HR=%d, PC=%d XC=%d\n", _vsyncPin, _hrefPin, _pclkPin,
                 _xclkPin);
    debug.printf("  RST=%d\n", _rst);

    for (int i = 0; i < 8; i++) {
        pinMode(_dPins[i], INPUT);
        debug.printf("  _dpins(%d)=%d\n", i, _dPins[i]);
    }
#endif

    _vsyncPort = portInputRegister(digitalPinToPort(_vsyncPin));
    _vsyncMask = digitalPinToBitMask(_vsyncPin);
    _hrefPort = portInputRegister(digitalPinToPort(_hrefPin));
    _hrefMask = digitalPinToBitMask(_hrefPin);
    _pclkPort = portInputRegister(digitalPinToPort(_pclkPin));
    _pclkMask = digitalPinToBitMask(_pclkPin);

    beginXClk();
    delay(100);

    if (_rst != 0xFF) {
        pinMode(_rst, OUTPUT);
        digitalWriteFast(_rst, LOW); /* Reset */
        for (volatile uint32_t i = 0; i < 100000; i++) {
        }
        digitalWriteFast(_rst, HIGH); /* Normal mode. */
        for (volatile uint32_t i = 0; i < 100000; i++) {
        }
    }

    if (getModelid() != 0x2145)
        return false;

    reset();

    if (setPixformat(format) != 0) {
        if (_debug)
            debug.println("Error: setPixformat failed");
        return false;
    }
    if (setFramesize(resolution) != 0) {
        if (_debug)
            debug.println("Error: setFramesize failed");
        return false; // failed to set resolution
    }
    if (_debug)
        printRegisters();

    // flexIO/DMA
    if (!_use_gpio) {
        hardware_configure();
        setVSyncISRPriority(102);
        setDMACompleteISRPriority(192);
    } else {
        setVSyncISRPriority(102);
        setDMACompleteISRPriority(192);
    }

    return true;
}

int GC2145::reset() {
    int ret = 0;

    readout_x = 0;
    readout_y = 0;

    readout_w = ACTIVE_SENSOR_WIDTH;
    readout_h = ACTIVE_SENSOR_HEIGHT;

    fov_wide = false;

    for (int i = 0; default_regs[i][0] && ret == 0; i++) {
        ret |= cameraWriteRegister(default_regs[i][0], default_regs[i][1]);
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

int GC2145::setPixformat(pixformat_t pixformat) {
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
        // TODO: There's no support for extracting GS from YUV so we use Bayer
        // for 1BPP for now. ret |= regWrite(GC2145_I2C_ADDR,
        //        REG_OUTPUT_FMT, REG_OUTPUT_SET_FMT(reg,
        //        REG_OUTPUT_FMT_YCBYCR));
        // break;
    case BAYER:
        // There's no BAYER support so it will just look off.
        // Make sure odd/even row are switched to work with our bayer
        // conversion.
        ret |= cameraWriteRegister(REG_SYNC_MODE, REG_SYNC_MODE_DEF |
                                                      REG_SYNC_MODE_ROW_SWITCH);
        ret |= cameraWriteRegister(
            REG_OUTPUT_FMT, REG_OUTPUT_SET_FMT(reg, REG_OUTPUT_FMT_BAYER));
        break;
    default:
        return -1;
    }
    if (_debug)
        debug.printf("Pixel Format: 0x%x, 0x%x, 0x%x\n", reg, REG_OUTPUT_FMT,
                     REG_OUTPUT_SET_FMT(reg, REG_OUTPUT_FMT_RGB565));

    return ret;
}

int GC2145::setWindow(uint16_t reg, uint16_t x, uint16_t y, uint16_t w,
                      uint16_t h) {
    int ret = 0;

    // P0 regs
    ret |= cameraWriteRegister(0xFE, 0x00);

    // Y/row offset
    ret |= cameraWriteRegister(reg++, y >> 8);
    ret |= cameraWriteRegister(reg++, y & 0xff);

    // X/col offset
    ret |= cameraWriteRegister(reg++, x >> 8);
    ret |= cameraWriteRegister(reg++, x & 0xff);
    // Window height
    ret |= cameraWriteRegister(reg++, h >> 8);
    ret |= cameraWriteRegister(reg++, h & 0xff);
    // Window width
    ret |= cameraWriteRegister(reg++, w >> 8);
    ret |= cameraWriteRegister(reg++, w & 0xff);

    return ret;
}

int GC2145::getWindow(uint16_t reg, uint16_t &x, uint16_t &y, uint16_t &w,
                      uint16_t &h) {
    int ret = 0;

    // P0 regs
    ret |= cameraWriteRegister(0xFE, 0x00);

    // Y/row offset
    y = cameraReadRegister(reg++) << 8;
    y |= cameraReadRegister(reg++);

    // X/col offset
    x = cameraReadRegister(reg++) << 8;
    x |= cameraReadRegister(reg++);

    // Window height
    h = cameraReadRegister(reg++) << 8;
    h |= cameraReadRegister(reg++);
    // Window width
    w = cameraReadRegister(reg++) << 8;
    w |= cameraReadRegister(reg++);

    return ret;
}

uint8_t GC2145::setFramesize(framesize_t framesize) {
    if (framesize >= (sizeof(resolution) / sizeof(resolution[0])))
        return 1; // error
    return setFramesize(resolution[framesize][0], resolution[framesize][1]);
}

uint8_t GC2145::setFramesize(int w, int h) {
    uint8_t ret = 0;
    if ((w == 0) || (h == 0))
        return 1; // not valid
    _frame_width = _width = w;
    _frame_height = _height = h;

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
    uint16_t ratio =
        fast_floorf(min(readout_w / ((float)w), readout_h / ((float)h)));

    // Limit the maximum amount of scaling allowed to keep the frame rate up.
    ratio = min(ratio, (fov_wide ? 5 : 3));
    if (_debug)
        debug.printf("\n$$$ ratio:%u, %d %d %d %d\n", ratio, readout_w, w,
                     readout_h, h);
    // if (!(ratio % 2)) {
    //     // camera outputs messed up bayer images at even ratios for some
    //     reason... ratio -= 1;
    // }

    uint16_t sub_readout_w = w * ratio;
    uint16_t sub_readout_h = h * ratio;

    // Step 2: Determine horizontal and vertical start and end points.
    uint16_t sensor_w =
        sub_readout_w +
        DUMMY_WIDTH_BUFFER; // camera hardware needs dummy pixels to sync
    uint16_t sensor_h =
        sub_readout_h +
        DUMMY_HEIGHT_BUFFER; // camera hardware needs dummy lines to sync

    uint16_t sensor_x =
        max(min((((ACTIVE_SENSOR_WIDTH - sensor_w) / 4) - (readout_x / 2)) * 2,
                ACTIVE_SENSOR_WIDTH - sensor_w),
            -(DUMMY_WIDTH_BUFFER / 2)) +
        DUMMY_COLUMNS; // must be multiple of 2

    uint16_t sensor_y =
        max(min((((ACTIVE_SENSOR_HEIGHT - sensor_h) / 4) - (readout_y / 2)) * 2,
                ACTIVE_SENSOR_HEIGHT - sensor_h),
            -(DUMMY_HEIGHT_BUFFER / 2)) +
        DUMMY_LINES; // must be multiple of 2

    // Step 3: Write regs.
    // Set Readout window first.

#if defined(DEBUG_CAMERA)
    debug.println("\nSet Framesize:");
    debug.printf("Setting Resolution\n");
    debug.printf("Resolution (w/h): %d, %d\n", w, h);
    debug.println("Step 0: Clamp readout settings");
    debug.printf("ActSenWidth: %d, ActSenHeight: %d\n", ACTIVE_SENSOR_WIDTH,
                 ACTIVE_SENSOR_HEIGHT);
    debug.printf("Width: %d, Height: %d\n", w, h);
    debug.printf("ReadoutW: %d, ReadoutH: %d\n", readout_w, readout_h);
    debug.printf("ReadoutXmax: %d, ReadoutYmax: %d\n", readout_x_max,
                 readout_y_max);
    debug.printf("ReadoutX: %d, ReadoutY: %d\n", readout_x, readout_y);
    debug.println("\nStep 1: Determine sub-readout window.");
    debug.printf("Ratio: %d\n", ratio);
    debug.printf("Ratio after test: %d\n", ratio);
    debug.printf("sub_readout_w: %d, sub_readout_h: %d\n", sub_readout_w,
                 sub_readout_h);
    debug.println(
        "\nStep 2: Determine horizontal and vertical start and end points");
    debug.printf("sensor_w: %d, sensor_h: %d, sensor_x: %d, sensor_y: %d\n",
                 sensor_w, sensor_h, sensor_x, sensor_y);
    debug.println("\nStep 3: Write Regs - call set window");
    debug.printf("SensorX: %d, SensorY: %d, SensorW: %d, SensorH: %d\n\n",
                 sensor_x, sensor_y, sensor_w, sensor_h);
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

bool GC2145::setZoomWindow(uint16_t x, uint16_t y, uint16_t w, uint16_t h) {
    uint16_t x1, y1, w1, h1;
    if (_debug)
        debug.printf("GC2145::setZoomWindow(%u %u %u %u)\n", x, y, w, h);
    getWindow(0x91, x1, y1, w1, h1);
    if (w == (uint16_t)-1)
        w = w1;
    if (h == (uint16_t)-1)
        h = h1;
    if ((w > frameWidth()) || (h > frameHeight()))
        return false;

    if (x == (uint16_t)-1)
        x = (frameWidth() - w) / 2;
    if (y == (uint16_t)-1)
        y = (frameHeight() - h) / 2;

    if ((x + w) > frameWidth())
        return false;
    if ((y + h) > frameHeight())
        return false;

    if (_debug)
        debug.printf("\tPrev rect(%u %u %u %u)\n", x1, y1, w, h);
    setWindow(0x91, x, y, w, h);

    // remember the width and height
    _width = w;
    _height = h;
    return true;
}

int GC2145::setHmirror(int enable) {
    int ret = 0;
    uint8_t reg;

    // P0 regs
    ret |= cameraWriteRegister(0xFE, 0x00);
    reg = cameraReadRegister(REG_AMODE1);
    ret |= reg;
    ret |= cameraWriteRegister(REG_AMODE1, REG_AMODE1_SET_HMIRROR(reg, enable));
    return ret;
}

int GC2145::setVflip(int enable) {
    int ret = 0;
    uint8_t reg;

    // P0 regs
    ret |= cameraWriteRegister(0xFE, 0x00);
    reg = cameraReadRegister(REG_AMODE1);
    ret |= reg;
    ret |= cameraWriteRegister(REG_AMODE1, REG_AMODE1_SET_VMIRROR(reg, enable));
    return ret;
}

int GC2145::setAutoExposure(int enable, int exposure_us) {
    int ret = 0;
    uint8_t reg;
    ret |= cameraWriteRegister(0xFE, 0x00);
    reg = cameraReadRegister(0xb6);
    ret |= reg;
    ret |= cameraWriteRegister(0xb6, (reg & 0xFE) | (enable & 0x01));
    return ret;
}

int GC2145::setAutoWhitebal(int enable, float r_gain_db, float g_gain_db,
                            float b_gain_db) {
    int ret = 0;
    uint8_t reg;
    ret |= cameraWriteRegister(0xFE, 0x00);
    reg = cameraReadRegister(0x82);
    ret |= reg;
    ret |= cameraWriteRegister(0x82, (reg & 0xFD) | ((enable & 0x01) << 1));
    return ret;
}

int GC2145::setColorbar(int enable) {
    uint8_t ret = 0;
    uint8_t test1 = 0;
    uint8_t test2 = 0x01;
    uint8_t val = enable;

    if (val == GC2145_TEST_PATTERN_VGA_COLOR_BARS)
        test1 = 0x04;
    else if (val == GC2145_TEST_PATTERN_UXGA_COLOR_BARS)
        test1 = 0x44;
    else if (val == GC2145_TEST_PATTERN_SKIN_MAP)
        test1 = 0x10;
    else if (val >= GC2145_TEST_PATTERN_SOLID_COLOR) {
        test1 = 0x04;
        test2 = ((val - GC2145_TEST_PATTERN_SOLID_COLOR) << 4) | 0x8;
    } else if (val != GC2145_TEST_PATTERN_DISABLED) {
        if (_debug)
            debug.println("test pattern out of range\n");
        return 0;
    }

    cameraWriteRegister(0xfe, 0x00);

    ret = cameraWriteRegister(0x8c, test1);
    if (ret)
        return ret;

    return cameraWriteRegister(0x8d, test2);
}

typedef struct {
    uint16_t reg;
    uint8_t combine_with_previous;
    const __FlashStringHelper *reg_name;
} GC2145_TO_NAME_t;

static const GC2145_TO_NAME_t GC2145_reg_name_table[] PROGMEM{
    {0x00f0, 0, F(" chip_ID[15:8]")},
    {0x00f1, 0, F(" chip_ID[7:0]")},
    {0x00f2, 0, F(" pad_vb_hiz_mode data_pad_io sync_pad_io")},
    {0x00f3, 0, F(" I2C_open_en")},
    {0x00f6, 0, F(" Up_dn Pwd_dn")},
    {0x00f7, 0, F(" PLL_mode1")},
    {0x00f8, 0, F(" PLL_mode2")},
    {0x00f9, 0, F(" cm_mode")},
    {0x00fa, 0, F(" clk_div_mode")},
    {0x00fb, 0, F(" I2C_device_ID")},
    {0x00fc, 0, F(" analog_pwc")},
    {0x00fd, 0, F(" Scalar mode")},
    {0x00fe, 0, F(" Reset related")},
    {0x0003, 0, F("Exposure[12:8]")},
    {0x0004, 1, F("Exposure[7:0]")},
    {0x0005, 0, F("capt_hb[11:8]")},
    {0x0006, 1, F("capt_hb[7:0]")},
    {0x0007, 0, F("capt_vb[12:8]")},
    {0x0008, 1, F("capt_vb[7:0]")},
    {0x0009, 0, F("capt_row_start[10:8]")},
    {0x000a, 1, F("capt_row_start[7:0]")},
    {0x000b, 0, F("capt_col_start[10:8 ]")},
    {0x000c, 1, F("capt_col_start[7:1]")},
    {0x000d, 0, F("capt_win_height[10:8]")},
    {0x000e, 1, F("capt_win_height[7:0]")},
    {0x000f, 0, F("capt_win_width[10:8]")},
    {0x0010, 1, F("capt_win_width[7:1]")},
    {0x0011, 0, F("Sh_delay[9:8]")},
    {0x0012, 1, F("Sh_delay[7:0]")},
    {0x0013, 0, F("St -> Start time")},
    {0x0014, 1, F("Et -> End time")},
    {0x0017, 0, F("Analog mode1")},
    {0x0018, 0, F("Analog mode2")},
    {0x0020, 0, F("Analog mode3")},
    {0x0024, 0, F("Driver mode")},
    {0x003f, 0, F("dark_current_st able_th")},
    {0x0040, 0, F("Blk_mode1")},
    {0x0042, 0, F("BLK_limit_value")},
    {0x0043, 0, F("BLK_fame_cnt_TH")},
    {0x005c, 0, F("Exp_rate_darkc")},
    {0x005e, 0, F("current_G1_offset_odd_ratio")},
    {0x005f, 0, F("current_G1_offset_even_ratio")},
    {0x0060, 0, F("current_R1_offset_odd_ratio")},
    {0x0061, 0, F("current_R1_offset_even_ratio")},
    {0x0062, 0, F("current_B1_offset_odd_ratio")},
    {0x0063, 0, F("current_B1_offset_even_ratio")},
    {0x0064, 0, F("current_G2_offset_odd_ratio")},
    {0x0065, 0, F("current_G2_offset_even_ratio")},
    {0x0066, 0, F("Dark_current_G1_ratio")},
    {0x0067, 0, F("Dark_current_R_ratio")},
    {0x0068, 0, F("Dark_current_B_ratio")},
    {0x0069, 0, F("Dark_current_G2_ratio")},
    {0x006a, 0, F("manual_G1_odd_offset")},
    {0x006b, 0, F("manual_G1_even_offset")},
    {0x006c, 0, F("manual_R1_odd_offset")},
    {0x006d, 0, F("manual_R1_even_offset")},
    {0x006e, 0, F("manual_B2_odd_offset")},
    {0x006f, 0, F("manual_B2_even_offset")},
    {0x0070, 0, F("manual_G2_odd_offset")},
    {0x0071, 0, F("manual_G2_even_offset")},
    {0x0072, 0, F("BLK_DD_thBLK_various_th")},
    {0x0080, 0, F("Block_enable1")},
    {0x0081, 0, F("Block_enable2")},
    {0x0082, 0, F("Block enable")},
    {0x0083, 0, F("Special effect")},
    {0x0084, 0, F("Output format")},
    {0x0085, 0, F("Frame start")},
    {0x0086, 0, F("Sync mode")},
    {0x0087, 0, F("block_enable3_buf")},
    {0x0088, 0, F("module_gating")},
    {0x0089, 0, F("bypass_mode")},
    {0x008c, 0, F("debug_mode2")},
    {0x008d, 0, F("Debug_mode3")},
    {0x0090, 0, F("Crop enable")},
    {0x0091, 0, F("out_win_y1[10:8]")},
    {0x0092, 1, F("out_win_y1 [7:0]")},
    {0x0093, 0, F("out_win_x1[10:8]")},
    {0x0094, 1, F("out_win_x1[7:0]")},
    {0x0095, 0, F("out_win_height[10:8]")},
    {0x0096, 1, F("out_win_height[7:0]")},
    {0x0097, 0, F("out_win_width[10:8]")},
    {0x0098, 1, F("out_win_width[7:0]")},
    {0x0099, 0, F("subsample")},
    {0x009a, 0, F("Subsample mode")},
    {0x009b, 0, F("Sub_row_N1")},
    {0x009c, 0, F("Sub_row_N2")},
    {0x009d, 0, F("Sub_row_N3")},
    {0x009e, 0, F("Sub_row_N4")},
    {0x009f, 0, F("Sub_col_N1")},
    {0x00a0, 0, F("Sub_col_N2")},
    {0x00a1, 0, F("Sub_col_N3")},
    {0x00a2, 0, F("Sub_col_N4")},
    {0x00a3, 0, F("channel_gain_G1_odd")},
    {0x00a4, 0, F("channel_gain_G1_even")},
    {0x00a5, 0, F("channel_gain_R1_odd")},
    {0x00a6, 0, F("channel_gain_R1_even")},
    {0x00a7, 0, F("channel_gain_B2_odd")},
    {0x00a8, 0, F("channel_gain_")},
    {0x00a9, 0, F("channel_gain_G2_odd")},
    {0x00aa, 0, F("channel_gain_G2_even")},
    {0x00ad, 0, F("R_ratio")},
    {0x00ae, 0, F("G_ratio")},
    {0x00af, 0, F("B_ratio")},
    {0x00b0, 0, F("Global_gain")},
    {0x00b1, 0, F("Auto_pregain")},
    {0x00b2, 0, F("Auto_postgain")},
    {0x00b3, 0, F("AWB_R_gain")},
    {0x00b4, 0, F("AWB_G_gain")},
    {0x00b5, 0, F("AWB_B_gain")},
    {0x00b6, 0, F("AEC_enable")},
    {0x00c2, 0, F("output_buf_enable_buf")},
    {0x0101, 0, F("AEC_x1")},
    {0x0102, 0, F("AEC_x2")},
    {0x0103, 0, F("AEC_y1")},
    {0x0104, 0, F("AEC_y2")},
    {0x0105, 0, F("AEC_center_x1")},
    {0x0106, 0, F("AEC_center_x2")},
    {0x0107, 0, F("AEC_center_y1")},
    {0x0108, 0, F("AEC_center_y2")},
    {0x010a, 0, F("AEC_mode1")},
    {0x010b, 0, F("AEC_mode2")},
    {0x010c, 0, F("AEC_mode3")},
    {0x010d, 0, F("AEC_mode4")},
    {0x010e, 0, F("AEC_high_range")},
    {0x010f, 0, F("AEC_low_range")},
    {0x0113, 0, F("AEC_target_Y")},
    {0x0114, 0, F("Y_average")},
    {0x0115, 0, F("target_Y_limit_from_histogram")},
    {0x0116, 0, F("AEC_number_limit_high_range")},
    {0x0118, 0, F("AEC mode5")},
    {0x0119, 0, F("AEC mode 6")},
    {0x011a, 0, F("AEC gainmode")},
    {0x011f, 0, F("AEC_max_pre_dg_gain")},
    {0x0120, 0, F("AEC_max_post_dg_gain")},
    {0x0125, 0, F("AEC_anti_flicker_step[12:8]")},
    {0x0126, 0, F("AEC_anti_flicker_step[7:0]")},
    {0x0127, 0, F("AEC_exp_level_1[12:8]")},
    {0x0128, 1, F("AEC_exp_level_1[7:0]")},
    {0x0129, 0, F("AEC_exp_level_2[12:8]")},
    {0x012a, 1, F("AEC_exp_level_2[7:0]")},
    {0x012b, 0, F("AEC_exp_level_3[12:8]")},
    {0x012c, 1, F("AEC_exp_level_3[7:0]")},
    {0x012d, 0, F("AEC_exp_level_4[12:8]")},
    {0x012e, 1, F("AEC_exp_level_4[7:0]")},
    {0x012f, 0, F("AEC_exp_level_5[12:8]")},
    {0x0130, 1, F("AEC_exp_level_5[7:0]")},
    {0x0131, 0, F("AEC_exp_level_6[12:8]")},
    {0x0132, 1, F("AEC_exp_level_6[7:0]")},
    {0x0133, 0, F("AEC_exp_level_7[12:8]")},
    {0x0134, 1, F("AEC_exp_level_7[7:0]")},
    {0x0135, 0, F("AEC_max_dg_gain1")},
    {0x0136, 0, F("AEC_max_dg_gain2")},
    {0x0137, 0, F("AEC_max_dg_gain3")},
    {0x0138, 0, F("AEC_max_dg_gain4")},
    {0x0139, 0, F("AEC_max_dg_gain5")},
    {0x013a, 0, F("AEC_max_dg_gain6")},
    {0x013b, 0, F("AEC_max_dg_gain7")},
    {0x013c, 0, F("AEC_max_exp_level")},
    {0x013d, 0, F("AEC_exp_min_l[7:0]")},
    {0x0150, 0, F("AWB mode 1")},
    {0x0151, 0, F("AWBparameter")},
    {0x0152, 0, F("AWBparameter")},
    {0x0153, 0, F("AWBparameter")},
    {0x0154, 0, F("AWBparameter")},
    {0x0155, 0, F("AWBparameter")},
    {0x0156, 0, F("AWBparameter")},
    {0x0157, 0, F("AWBparameter")},
    {0x0158, 0, F("AWBparameter")},
    {0x0159, 0, F("AWB_PRE_RGB_low")},
    {0x015a, 0, F("AWB_PRE_RGB_high")},
    {0x015b, 0, F("AWBparameter")},
    {0x0175, 0, F("AWB_every_N")},
    {0x0176, 0, F("AWB_R_gain_limit")},
    {0x0177, 0, F("AWB_G_gain_limit")},
    {0x0178, 0, F("AWB_B_gain_limit")},
    {0x0179, 0, F("AWB_R_gain_out_h_limit")},
    {0x017a, 0, F("AWB_G_gain_out_h_limit")},
    {0x017b, 0, F("AWB_B_gain_out_h_limit")},
    {0x017c, 0, F("AWB_R_gain_out_l_limit")},
    {0x017d, 0, F("AWB_G_gain_out_l_limit")},
    {0x017e, 0, F("AWB_B_gain_out_l_limit")},
    {0x019a, 0, F("ABS_range_compesateABS_skip_frame")},
    {0x019b, 0, F("ABS_stop_margin")},
    {0x019c, 0, F("Y_S_compesateABS_manual_K")},
    {0x019d, 0, F("Y_stretch_limit")},
    {0x01a0, 0, F("LSC_row_x2LSC_col_x2LSC_pixel_array_select")},
    {0x01a1, 0, F("LSC_row_center")},
    {0x01a2, 0, F("LSC_col_cente")},
    {0x01a4, 0, F("LSC_Q12_RGB_sign")},
    {0x01a5, 0, F("LSC_Q34_RGB_SIGN")},
    {0x01a6, 0, F("LSC_right_left_rgb_b4_sign")},
    {0x01a7, 0, F("LSC_up_down_rgb_b4_sign")},
    {0x01a8, 0, F("LSC_right_up_down_rgb_b22_sign")},
    {0x01a9, 0, F("LSC_left_up_down_rgb_b22_sign")},
    {0x01aa, 0, F("LSC_Q1_red_b1")},
    {0x01ab, 0, F("LSC_Q1_green_b1")},
    {0x01ac, 0, F("LSC_Q1_blue_b1")},
    {0x01ad, 0, F("LSC_Q2_red_b1")},
    {0x01ae, 0, F("LSC_Q2_green_b1")},
    {0x01af, 0, F("LSC_Q2_blue_b1")},
    {0x01b0, 0, F("LSC_Q3_red_b1")},
    {0x01b1, 0, F("LSC_Q3_green_b1")},
    {0x01b2, 0, F("LSC_Q3_blue_b1")},
    {0x01b3, 0, F("LSC_Q4_red_b1")},
    {0x01b4, 0, F("LSC_Q4_green_b1")},
    {0x01b5, 0, F("LSC_Q4_blue_b1")},
    {0x01b6, 0, F("LSC_right_red_b2")},
    {0x01b7, 0, F("LSC_right_green_b2")},
    {0x01b8, 0, F("LSC_right_blue_b2")},
    {0x01b9, 0, F("LSC_right_red_b4")},
    {0x01ba, 0, F("LSC_right_green_b4")},
    {0x01bb, 0, F("LSC_right_blue_b4")},
    {0x01bc, 0, F("LSC_left_red_b2")},
    {0x01bd, 0, F("LSC_left_green_b2")},
    {0x01be, 0, F("LSC_left_blue_b2")},
    {0x01bf, 0, F("LSC_left_red_b4")},
    {0x01c0, 0, F("LSC_left_green_b4")},
    {0x01c1, 0, F("LSC_left_blue_")},
    {0x01c2, 0, F("LSC_up_red_b2")},
    {0x01c3, 0, F("LSC_up_green_b2")},
    {0x01c4, 0, F("LSC_up_blue_b2")},
    {0x01c5, 0, F("LSC_up_red_b4")},
    {0x01c6, 0, F("LSC_up_green_b4")},
    {0x01c7, 0, F("LSC_up_blue_b4")},
    {0x01c8, 0, F("LSC_down_red_b2")},
    {0x01c9, 0, F("LSC_down_green_b2")},
    {0x01ca, 0, F("LSC_down_blue_b2")},
    {0x01cb, 0, F("LSC_down_red_b4")},
    {0x01cc, 0, F("LSC_down_green_b4")},
    {0x01cd, 0, F("LSC_down_blue_b4")},
    {0x01d0, 0, F("LSC_right_up_red_b22")},
    {0x01d1, 0, F("LSC_right_up_green_b22")},
    {0x01d2, 0, F("LSC_right_up_blue_b22")},
    {0x01d3, 0, F("LSC_right_down_red_b22")},
    {0x01d4, 0, F("LSC_right_down_green_b22")},
    {0x01d5, 0, F("LSC_right_down_blue_b22")},
    {0x01d6, 0, F("LSC_left_up_red_b22")},
    {0x01d7, 0, F("LSC_left_up_green_b22")},
    {0x01d8, 0, F("LSC_left_up_blue_b22")},
    {0x01d9, 0, F("LSC_left_down_red_b22")},
    {0x01da, 0, F("LSC_left_down_green_b22")},
    {0x01db, 0, F("LSC_left_down_blue_b22")},
    {0x01dc, 0, F("LSC_Y_dark_th")},
    {0x01dd, 0, F("LSC_Y_dark_slope")},
    {0x01df, 0, F("LSC_U_B2G_stand_plus")},
    {0x0210, 0, F("Gamma_out1")},
    {0x0211, 0, F("Gamma_out2")},
    {0x0212, 0, F("Gamma_out3")},
    {0x0213, 0, F("Gamma_out4")},
    {0x0214, 0, F("Gamma_out5")},
    {0x0215, 0, F("Gamma_out6")},
    {0x0216, 0, F("Gamma_out7")},
    {0x0217, 0, F("Gamma_out8")},
    {0x0218, 0, F("Gamma_out9")},
    {0x0219, 0, F("Gamma_out10")},
    {0x021a, 0, F("Gamma_out11")},
    {0x021b, 0, F("Gamma_out12")},
    {0x021c, 0, F("Gamma_out13")},
    {0x021d, 0, F("Gamma_out14")},
    {0x021e, 0, F("Gamma_out15")},
    {0x021f, 0, F("Gamma_out16")},
    {0x0220, 0, F("Gamma_out17")},
    {0x0221, 0, F("Gamma_out18")},
    {0x0222, 0, F("Gamma_out19")},
    {0x0223, 0, F("Gamma_out20")},
    {0x0224, 0, F("Gamma_out21")},
    {0x0225, 0, F("Gamma_out22")},
    {0x0284, 0, F("DD_dark_th")},
    {0x0285, 0, F("ASDE_DN_B_slope")},
    {0x0289, 0, F("ASDE_low_luma_value_DD_th2")},
    {0x028a, 0, F("ASDE_low_luma_value_DD_th3")},
    {0x028b, 0, F("ASDE_low_luma_value_DD_th4")},
    {0x0290, 0, F("EEINTP")},
    {0x0291, 0, F("EEINTP")},
    {0x0292, 0, F("direction_TH1")},
    {0x0293, 0, F("Direction_TH2")},
    {0x0294, 0, F("diff_HV_mode")},
    {0x0295, 0, F("direction_diff_")},
    {0x0296, 0, F("edge level")},
    {0x0297, 0, F("Edge1 effect")},
    {0x0298, 0, F("Edge_pos_ratio")},
    {0x0299, 0, F("Edge1_max")},
    {0x029a, 0, F("Edge2_max")},
    {0x029b, 0, F("Edge1_th")},
    {0x029c, 0, F("Edge_pos_max")},
    {0x029d, 0, F("Edge_effect_sc")},
    {0x02c0, 0, F("CC_mode")},
    {0x02c1, 0, F("CC_CT1_11")},
    {0x02c2, 0, F("CC_CT1_12")},
    {0x02c3, 0, F("CC_CT1_13")},
    {0x02c4, 0, F("CC_CT1_21")},
    {0x02c5, 0, F("CC_CT1_22")},
    {0x02c6, 0, F("CC_CT1_23")},
    {0x02d0, 0, F("Global")},
    {0x02d1, 0, F("saturation_Cb")},
    {0x02d2, 0, F("saturation_Cr")},
    {0x02d3, 0, F("luma_contrast")},
    {0x02d4, 0, F("Contrast center")},
    {0x02d5, 0, F("Luma_offset")},
    {0x02d6, 0, F("skin_Cb_center")},
    {0x02d7, 0, F("skin_Cr_center")},
    {0x02d9, 0, F("Skin brightnessmode")},
    {0x02da, 0, F("Fixed_Cb")},
    {0x02db, 0, F("Fixed_Cr")},
    {0x02e6, 0, F("CC_R_offset")},
    {0x02e7, 0, F("CC_G_offset")},
    {0x02e8, 0, F("CC_B_offset")},
    {0x0301, 0, F("DPHY_analog_mode1")},
    {0x0302, 0, F("DPHY_analog_mode2")},
    {0x0303, 0, F("DPHY_analog_mode3")},
    {0x0304, 0, F("FIFO_prog_full_level[7:0]")},
    {0x0305, 0, F("FIFO_prog_full_level[11:8]")},
    {0x0306, 0, F("FIFO_mode")},
    {0x0310, 0, F("BUF_CSI2_mode")},
    {0x0311, 0, F("LDI_set")},
    {0x0312, 0, F("LWC_set[7:0]")},
    {0x0313, 0, F("LWC_set[15:8]")},
    {0x0314, 0, F("SYNC_set")},
    {0x0315, 0, F("DPHY_mode")},
    {0x0316, 0, F("LP_set")},
    {0x0317, 0, F("fifo_gate_modeMIPI_wdiv_set")},
    {0x0320, 0, F("T_init_set")},
    {0x0321, 0, F("T_LPX_set")},
    {0x0322, 0, F("T_CLK_HS_PREPARE_set")},
    {0x0323, 0, F("T_CLK_zero_set")},
    {0x0324, 0, F("T_CLK_PRE_set")},
    {0x0325, 0, F("T_CLK_POST_set")},
    {0x0326, 0, F("T_CLK_TRAIL_set")},
    {0x0327, 0, F("T_HS_exit_set")},
    {0x0328, 0, F("T_wakeup_set")},
    {0x0329, 0, F("T_HS_PREPARE_set")},
    {0x032a, 0, F("T_HS_Zero_set")},
    {0x032b, 0, F("T_HS_TRAIL_set")},
    {0x0330, 0, F("MIPI_Test")},
    {0x0331, 0, F("MIPI_Test_data0")},
    {0x0332, 0, F("MIPI_Test_data1")},
    {0x0333, 0, F("MIPI_Test_data2")},
    {0x0334, 0, F("MIPI_Test_data3")},
    {0x033f, 0, F("FIFO_error log")},
    {0x0340, 0, F("output_buf_mode1")},
    {0x0341, 0, F("output_buf_mode2")},
    {0x0342, 0, F("buf_win_width[7:0]")},
    {0x0343, 0, F("buf_win_width[11:8]")},
};

uint16_t gc2145_reg_page = 0;
uint32_t gc2145_registers_set[32] = {0};

#define USE_PRINTREGISTERS

void GC2145::printRegisters(bool only_ones_set) {
#ifdef USE_PRINTREGISTERS
    uint8_t reg;
    debug.println("\n*** Camera Registers ***");
    if (only_ones_set) {
        uint8_t current_page = 0;
        uint8_t previous_reg_value = 0;
        for (uint16_t ii = 3; ii < 1024; ii++) {
            if (gc2145_registers_set[ii >> 5] & (1 << (ii & 0x1f))) {
                uint8_t page = ii >> 8;
                if (page != current_page) {
                    current_page = page;
                    cameraWriteRegister(0xfe, page);
                }
                reg = cameraReadRegister(ii);
                debug.printf("(%u:0x%x): (0x%x - %d)", page, ii & 0xff, reg,
                             reg);
                for (uint16_t jj = 0; jj < (sizeof(GC2145_reg_name_table) /
                                            sizeof(GC2145_reg_name_table[0]));
                     jj++) {
                    if (ii == GC2145_reg_name_table[jj].reg) {
                        if (GC2145_reg_name_table[jj].combine_with_previous) {
                            uint16_t reg16_value =
                                (previous_reg_value << 8) | reg;
                            debug.printf(" :: %u(%x)", reg16_value,
                                         reg16_value);
                        }
                        debug.print("\t// ");
                        debug.print(GC2145_reg_name_table[jj].reg_name);
                        break;
                    }
                }
                previous_reg_value = reg;
                debug.println();
            }
        }
        if (current_page != 0)
            cameraWriteRegister(0xfe, 0);

    } else {
        for (uint16_t ii = 3; ii < 182; ii++) {
            reg = cameraReadRegister(ii);
            debug.printf("(0x%x): (0x%x - %d)", ii, reg, reg);

            for (uint16_t jj = 0; jj < (sizeof(GC2145_reg_name_table) /
                                        sizeof(GC2145_reg_name_table[0]));
                 jj++) {
                if (ii == GC2145_reg_name_table[jj].reg) {
                    debug.print("\t: ");
                    debug.print(GC2145_reg_name_table[jj].reg_name);
                    break;
                }
            }
            debug.println();
        }
    }
#endif
    uint16_t x, y, w, h;
    getWindow(0x9, x, y, w, h);
    debug.printf("\nCISCTL rect(%u, %u, %u, %u)\n", x, y, w, h);
    getWindow(0x91, x, y, w, h);
    debug.printf("Win rect(%u, %u, %u, %u)\n", x, y, w, h);
    uint8_t ratio = cameraReadRegister(0x99);
    if (_debug)
        debug.printf("Ratio: row:%u col:%u\n", ratio >> 4, ratio & 0xf);
}

void GC2145::showRegisters(void) {
#ifdef USE_PRINTREGISTERS
    printRegisters(true);
#else
    debug.println("\n*** Camera Registers ***");
    uint8_t previous_reg_value = 0;
    for (uint16_t ii = 0; ii < (sizeof(GC2145_reg_name_table) /
                                sizeof(GC2145_reg_name_table[0]));
         ii++) {
        uint8_t reg_value = cameraReadRegister(GC2145_reg_name_table[ii].reg);
        debug.printf("%s(%x): %u(%x)", GC2145_reg_name_table[ii].reg_name,
                     GC2145_reg_name_table[ii].reg, reg_value, reg_value);
        if (GC2145_reg_name_table[ii].combine_with_previous) {
            uint16_t reg16_value = (previous_reg_value << 8) | reg_value;
            debug.printf(" :: %u(%x)\n", reg16_value, reg16_value);
        } else {
            debug.println();
        }
        previous_reg_value = reg_value;
    }
#endif
}

/*******************************************************************/
// Read a single uint8_t from address and return it as a uint8_t
uint8_t GC2145::cameraReadRegister(uint8_t reg) {
    _wire->beginTransmission(camAddress);
    //_wire->write(reg >> 8);
    _wire->write(reg);
    if (_wire->endTransmission(false) != 0) {
        if (_debug)
            debug.println("error reading GC2145, address");
        return 0;
    }
    if (_wire->requestFrom(camAddress, 1) < 1) {
        if (_debug)
            debug.println("error reading GC2145, data");
        return 0;
    }
    return _wire->read();
}

uint8_t GC2145::cameraWriteRegister(uint8_t reg, uint8_t data) {
    if (reg == 0xfe)
        gc2145_reg_page = (data & 0x7) << 8;

    uint16_t reg_lookup = reg;
    if (reg < 0xf0)
        reg_lookup += gc2145_reg_page;

    // lets remember all of the registers we wrote something to
    gc2145_registers_set[reg_lookup >> 5] |= 1 << (reg_lookup & 0x1f);
#ifdef DEBUG_CAMERA
    debug.printf("Write Register (%u:0x%x): (0x%x - %d)", gc2145_reg_page >> 8,
                 reg, data, data);
    if (_debug) {
        for (uint16_t jj = 0; jj < (sizeof(GC2145_reg_name_table) /
                                    sizeof(GC2145_reg_name_table[0]));
             jj++) {
            if (reg_lookup == GC2145_reg_name_table[jj].reg) {
                debug.print("\t: ");
                debug.print(GC2145_reg_name_table[jj].reg_name);
                break;
            }
        }
        debug.println();
    }

#endif
    _wire->beginTransmission(camAddress);
    //_wire->write(reg >> 8);
    _wire->write(reg);
    _wire->write(data);
    if (_wire->endTransmission() != 0) {
        if (_debug)
            debug.println("error writing to GC2145");
    }

    return 0;
}

/***********************************************************************/

#define FLEXIO_USE_DMA

//======================================== DMA JUNK
//================================================================================
// experiment with DMA
//================================================================================
// Define our DMA structure.
// DMAChannel GC2145::_dmachannel;
// DMASetting GC2145::_dmasettings[10];
uint32_t GC2145::_dmaBuffer1[DMABUFFER_SIZE] __attribute__((used, aligned(32)));
uint32_t GC2145::_dmaBuffer2[DMABUFFER_SIZE] __attribute__((used, aligned(32)));
extern "C" void xbar_connect(unsigned int input,
                             unsigned int output); // in pwm.c

// GC2145 *GC2145::active_dma_camera = nullptr;

//===================================================================
// Start a DMA operation -
//===================================================================
bool GC2145::startReadFrameDMA(bool (*callback)(void *frame_buffer),
                               uint8_t *fb1, uint8_t *fb2) {
    // First see if we need to allocate frame buffers.
    if (fb1)
        _frame_buffer_1 = fb1;
    else if (_frame_buffer_1 == nullptr) {
        _frame_buffer_1 = (uint8_t *)malloc(_width * _height);
        if (_frame_buffer_1 == nullptr)
            return false;
    }
    if (fb2)
        _frame_buffer_2 = fb2;
    else if (_frame_buffer_2 == nullptr) {
        _frame_buffer_2 = (uint8_t *)malloc(_width * _height);
        if (_frame_buffer_2 == nullptr)
            return false; // BUGBUG should we 32 byte align?
    }
    // remember the call back if passed in
    _callback = callback;
    active_dma_camera = this;

    debug.printf("startReadFrameDMA called buffers %x %x\n",
                 (uint32_t)_frame_buffer_1, (uint32_t)_frame_buffer_2);

    // DebugDigitalToggle(GC2145_DEBUG_PIN_1);
    // lets figure out how many bytes we will tranfer per setting...
    //  _dmasettings[0].begin();
    _frame_row_buffer_pointer = _frame_buffer_pointer =
        (uint8_t *)_frame_buffer_1;

    // configure DMA channels
    _dmachannel.begin();
    _dmasettings[0].source(GPIO2_PSR); // setup source.
    _dmasettings[0].destinationBuffer(
        _dmaBuffer1, DMABUFFER_SIZE * 4); // 32 bits per logical byte
    _dmasettings[0].replaceSettingsOnCompletion(_dmasettings[1]);
    _dmasettings[0]
        .interruptAtCompletion(); // we will need an interrupt to process this.
    _dmasettings[0].TCD->CSR &=
        ~(DMA_TCD_CSR_DREQ); // Don't disable on this one
    // DebugDigitalToggle(GC2145_DEBUG_PIN_1);

    _dmasettings[1].source(GPIO2_PSR); // setup source.
    _dmasettings[1].destinationBuffer(
        _dmaBuffer2, DMABUFFER_SIZE * 4); // 32 bits per logical byte
    _dmasettings[1].replaceSettingsOnCompletion(_dmasettings[0]);
    _dmasettings[1]
        .interruptAtCompletion(); // we will need an interrupt to process this.
    _dmasettings[1].TCD->CSR &=
        ~(DMA_TCD_CSR_DREQ); // Don't disable on this one
    // DebugDigitalToggle(GC2145_DEBUG_PIN_1);

    GPIO2_GDIR = 0; // set all as input...
    GPIO2_DR = 0;   // see if I can clear it out...

    _dmachannel = _dmasettings[0]; // setup the first on...
    _dmachannel.attachInterrupt(dmaInterrupt);
    _dmachannel.triggerAtHardwareEvent(DMAMUX_SOURCE_XBAR1_0);
    // DebugDigitalToggle(GC2145_DEBUG_PIN_1);

    // Lets try to setup the DMA setup...
    // first see if we can convert the _pclk to be an XBAR Input pin...
    // GC2145_PLK   4
    // GC2145_PLK   8    //8       B1_00   FlexIO2:16  XBAR IO14

    _save_pclkPin_portConfigRegister = *(portConfigRegister(_pclkPin));
    *(portConfigRegister(_pclkPin)) = 1; // set to XBAR mode 14

    // route the timer outputs through XBAR to edge trigger DMA request
    CCM_CCGR2 |= CCM_CCGR2_XBAR1(CCM_CCGR_ON);
    xbar_connect(XBARA1_IN_IOMUX_XBAR_INOUT14, XBARA1_OUT_DMA_CH_MUX_REQ30);
    // DebugDigitalToggle(GC2145_DEBUG_PIN_1);

    // Tell XBAR to dDMA on Rising
    XBARA1_CTRL0 = XBARA_CTRL_STS0 | XBARA_CTRL_EDGE0(1) |
                   XBARA_CTRL_DEN0 /* | XBARA_CTRL_IEN0 */;

    IOMUXC_GPR_GPR6 &=
        ~(IOMUXC_GPR_GPR6_IOMUXC_XBAR_DIR_SEL_14); // Make sure it is input mode
    IOMUXC_XBAR1_IN14_SELECT_INPUT =
        1; // Make sure this signal goes to this pin...

#if defined(ARDUINO_TEENSY_MICROMOD)
    // Need to switch the IO pins back to GPI1 from GPIO6
    _save_IOMUXC_GPR_GPR27 =
        IOMUXC_GPR_GPR27; // save away the configuration before we change...
    IOMUXC_GPR_GPR27 &= ~(0x0ff0u);

    // lets also un map the _hrefPin to GPIO1
    IOMUXC_GPR_GPR27 &= ~_hrefMask; //
#else
    // Need to switch the IO pins back to GPI1 from GPIO6
    _save_IOMUXC_GPR_GPR26 =
        IOMUXC_GPR_GPR26; // save away the configuration before we change...
    IOMUXC_GPR_GPR26 &= ~(0x0ff0u);

    // lets also un map the _hrefPin to GPIO1
    IOMUXC_GPR_GPR26 &= ~_hrefMask; //
#endif

    // Need to switch the IO pins back to GPI1 from GPIO6
    //_save_IOMUXC_GPR_GPR27 = IOMUXC_GPR_GPR27;  // save away the configuration
    // before we change... IOMUXC_GPR_GPR27 &= ~(0x0ff0u);

    // lets also un map the _hrefPin to GPIO1
    // IOMUXC_GPR_GPR27 &= ~_hrefMask; //

    // DebugDigitalToggle(GC2145_DEBUG_PIN_1);

    // Falling edge indicates start of frame
    //  while ((*_vsyncPort & _vsyncMask) == 0); // wait for HIGH
    //  while ((*_vsyncPort & _vsyncMask) != 0); // wait for LOW
    //  DebugDigitalWrite(GC2145_DEBUG_PIN_2, HIGH);

    // Debug stuff for now

    // We have the start of a frame, so lets start the dma.
#ifdef DEBUG_CAMERA
    dumpDMA_TCD_GC(&_dmachannel, " CH: ");
    dumpDMA_TCD_GC(&_dmasettings[0], " 0: ");
    dumpDMA_TCD_GC(&_dmasettings[1], " 1: ");

    debug.printf("pclk pin: %d config:%lx control:%lx\n", _pclkPin,
                 *(portConfigRegister(_pclkPin)),
                 *(portControlRegister(_pclkPin)));
    debug.printf("IOMUXC_GPR_GPR26-29:%lx %lx %lx %lx\n", IOMUXC_GPR_GPR26,
                 IOMUXC_GPR_GPR27, IOMUXC_GPR_GPR28, IOMUXC_GPR_GPR29);
    debug.printf("GPIO1: %lx %lx, GPIO6: %lx %lx\n", GPIO1_DR, GPIO1_PSR,
                 GPIO6_DR, GPIO6_PSR);
    debug.printf("XBAR CTRL0:%x CTRL1:%x\n\n", XBARA1_CTRL0, XBARA1_CTRL1);
#endif
    _dma_state = DMASTATE_RUNNING;
    _dma_last_completed_frame = nullptr;
    _dma_frame_count = 0;

    // Now start an interrupt for start of frame.
    //  attachInterrupt(_vsyncPin, &frameStartInterrupt, RISING);

    // DebugDigitalToggle(GC2145_DEBUG_PIN_1);
    return true;
}

//===================================================================
// stopReadFrameDMA - stop doing the reading and then exit.
//===================================================================
bool GC2145::stopReadFrameDMA() {

// hopefully it start here (fingers crossed)
// for now will hang here to see if completes...
#ifdef USE_DEBUG_PINS
// DebugDigitalWrite(GC2145_DEBUG_PIN_2, HIGH);
#endif
    elapsedMillis em = 0;
    // tell the background stuff DMA stuff to exit.
    // Note: for now let it end on on, later could disable the DMA directly.
    _dma_state = DMASTATE_STOP_REQUESTED;

    while ((em < 1000) && (_dma_state == DMASTATE_STOP_REQUESTED))
        ; // wait up to a second...
    if (_dma_state != DMA_STATE_STOPPED) {
        debug.println("*** stopReadFrameDMA DMA did not exit correctly...");
        debug.printf("  Bytes Left: %u frame buffer:%x Row:%u Col:%u\n",
                     _bytes_left_dma, (uint32_t)_frame_buffer_pointer,
                     _frame_row_index, _frame_col_index);
    }
#ifdef USE_DEBUG_PINS
// DebugDigitalWrite(GC2145_DEBUG_PIN_2, LOW);
#endif
#ifdef DEBUG_CAMERA
    dumpDMA_TCD_GC(&_dmachannel, nullptr);
    dumpDMA_TCD_GC(&_dmasettings[0], nullptr);
    dumpDMA_TCD_GC(&_dmasettings[1], nullptr);
    debug.println();
#endif
    // Lets restore some hardware pieces back to the way we found them.
#if defined(ARDUINO_TEENSY_MICROMOD)
    IOMUXC_GPR_GPR27 =
        _save_IOMUXC_GPR_GPR27; // Restore... away the configuration before we
                                // change...
#else
    IOMUXC_GPR_GPR26 =
        _save_IOMUXC_GPR_GPR26; // Restore... away the configuration before we
                                // change...
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
    active_dma_camera
        ->processDMAInterrupt(); // lets get back to the main object...
}

// This version assumes only called when HREF...  as set pixclk to only fire
// when set.
void GC2145::processDMAInterrupt() {
    _dmachannel.clearInterrupt(); // tell system we processed it.
    asm("DSB");
#ifdef USE_DEBUG_PINS
// DebugDigitalWrite(GC2145_DEBUG_PIN_3, HIGH);
#endif

    if (_dma_state == DMA_STATE_STOPPED) {
        debug.println("GC2145::dmaInterrupt called when DMA_STATE_STOPPED");
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
    if ((_dma_index < 3) || (buffer_size < DMABUFFER_SIZE)) {
        debug.printf("D(%d, %d, %lu) %u : ", _dma_index, buffer_size,
                     _bytes_left_dma, pixformat);
        for (uint16_t i = 0; i < 8; i++) {
            uint16_t b = buffer[i] >> 4;
            debug.printf(" %lx(%02x)", buffer[i], b);
        }
        debug.print("...");
        for (uint16_t i = buffer_size - 8; i < buffer_size; i++) {
            uint16_t b = buffer[i] >> 4;
            debug.printf(" %lx(%02x)", buffer[i], b);
        }
        debug.println();
    }
#endif

    for (uint16_t buffer_index = 0; buffer_index < buffer_size;
         buffer_index++) {
        if (!_bytes_left_dma || (_frame_row_index >= _height))
            break;

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

    if ((_frame_row_index == _height) ||
        (_bytes_left_dma == 0)) { // We finished a frame lets bail
        _dmachannel.disable();    // disable the DMA now...
#ifdef USE_DEBUG_PINS
// DebugDigitalWrite(GC2145_DEBUG_PIN_2, LOW);
#endif
#ifdef DEBUG_CAMERA_VERBOSE
        debug.println("EOF");
#endif
        _frame_row_index = 0;
        _dma_frame_count++;

        bool swap_buffers = true;

        // DebugDigitalToggle(GC2145_DEBUG_PIN_1);
        _dma_last_completed_frame = _frame_row_buffer_pointer;
        if (_callback)
            swap_buffers = (*_callback)(_dma_last_completed_frame);

        if (swap_buffers) {
            if (_frame_row_buffer_pointer != _frame_buffer_1)
                _frame_row_buffer_pointer = _frame_buffer_2;
            else
                _frame_row_buffer_pointer = _frame_buffer_2;
        }

        _frame_buffer_pointer = _frame_row_buffer_pointer;

        // DebugDigitalToggle(GC2145_DEBUG_PIN_1);

        if (_dma_state == DMASTATE_STOP_REQUESTED) {
#ifdef DEBUG_CAMERA
            debug.println("GC2145::dmaInterrupt - Stop requested");
#endif
            _dma_state = DMA_STATE_STOPPED;
        } else {
            // We need to start up our ISR for the next frame.
#if 1
            // bypass interrupt and just restart DMA...
            _bytes_left_dma = (_width + _frame_ignore_cols) *
                              _height; // for now assuming color 565 image...
            _dma_index = 0;
            _frame_col_index = 0; // which column we are in a row
            _frame_row_index = 0; // which row
            _save_lsb = 0xffff;
            // make sure our DMA is setup properly again.
            _dmasettings[0].transferCount(DMABUFFER_SIZE);
            _dmasettings[0].TCD->CSR &=
                ~(DMA_TCD_CSR_DREQ); // Don't disable on this one
            _dmasettings[1].transferCount(DMABUFFER_SIZE);
            _dmasettings[1].TCD->CSR &=
                ~(DMA_TCD_CSR_DREQ);       // Don't disable on this one
            _dmachannel = _dmasettings[0]; // setup the first on...
            _dmachannel.enable();

#else
            attachInterrupt(_vsyncPin, &frameStartInterrupt, RISING);
#endif
        }
    } else {

        if (_bytes_left_dma == (2 * DMABUFFER_SIZE)) {
            if (_dma_index & 1)
                _dmasettings[0].disableOnCompletion();
            else
                _dmasettings[1].disableOnCompletion();
        }
    }
#ifdef USE_DEBUG_PINS
// DebugDigitalWrite(GC2145_DEBUG_PIN_3, LOW);
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

void GC2145::captureFrameStatistics() {
    memset((void *)&fstat_gc, 0, sizeof(fstat_gc));

    // lets wait for the vsync to go high;
    while ((*_vsyncPort & _vsyncMask) != 0)
        ; // wait for HIGH
    // now lets wait for it to go low
    while ((*_vsyncPort & _vsyncMask) == 0)
        fstat_gc.vsyncStartCycleCount++; // wait for LOW

    while ((*_hrefPort & _hrefMask) == 0)
        ; // wait for HIGH
    while ((*_pclkPort & _pclkMask) != 0)
        ; // wait for LOW

    uint32_t microsStart = micros();
    fstat_gc.hrefStartTime[0] = microsStart;
    // now loop through until we get the next _vsynd
    // BUGBUG We know that HSYNC and PCLK on same GPIO VSYNC is not...
    uint32_t regs_prev = 0;
    // noInterrupts();
    while ((*_vsyncPort & _vsyncMask) != 0) {

        fstat_gc.cycleCount++;
        uint32_t regs = (*_hrefPort & (_hrefMask | _pclkMask));
        if (regs != regs_prev) {
            if ((regs & _hrefMask) && ((regs_prev & _hrefMask) == 0)) {
                fstat_gc.hrefCount++;
                fstat_gc.hrefStartTime[fstat_gc.hrefCount] = micros();
            }
            if ((regs & _pclkMask) && ((regs_prev & _pclkMask) == 0))
                fstat_gc.pclkCounts[fstat_gc.hrefCount]++;
            if ((regs & _pclkMask) && ((regs_prev & _hrefMask) == 0))
                fstat_gc.pclkNoHrefCount++;
            regs_prev = regs;
        }
    }
    while ((*_vsyncPort & _vsyncMask) == 0)
        fstat_gc.vsyncEndCycleCount++; // wait for LOW
    // interrupts();
    fstat_gc.frameTimeMicros = micros() - microsStart;

    // Maybe return data. print now
    debug.printf("*** Frame Capture Data: elapsed Micros: %u loops: %u\n",
                 fstat_gc.frameTimeMicros, fstat_gc.cycleCount);
    debug.printf("   VSync Loops Start: %u end: %u\n",
                 fstat_gc.vsyncStartCycleCount, fstat_gc.vsyncEndCycleCount);
    debug.printf("   href count: %u pclk ! href count: %u\n    ",
                 fstat_gc.hrefCount, fstat_gc.pclkNoHrefCount);
    for (uint16_t ii = 0; ii < fstat_gc.hrefCount + 1; ii++) {
        debug.printf("%3u(%u) ", fstat_gc.pclkCounts[ii],
                     (ii == 0) ? 0
                               : fstat_gc.hrefStartTime[ii] -
                                     fstat_gc.hrefStartTime[ii - 1]);
        if (!(ii & 0x0f))
            debug.print("\n    ");
    }
    debug.println();
}
