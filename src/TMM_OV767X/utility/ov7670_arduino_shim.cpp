// SPDX-License-Identifier: GPL-2.0-only
/*
 * This file is part of the Arduino_OX767X library.
 * Copyright (c) 2020 Arduino SA. All rights reserved.
 */
#include <Arduino.h>
#include <Wire.h>

#ifndef OV760_DEBUG
#define OV760_DEBUG
#endif

extern "C" {
  void msleep(unsigned long ms)
  {
    delay(ms);
  }
#ifdef OV760_DEBUG
typedef struct {
  const char *reg_name;
  unsigned char reg;
} OVREG_TO_NAME_t;

static const OVREG_TO_NAME_t ov_reg_name_table[] =
{
    {"GAIN", 0x00},               //< AGC gain bits 7:0 (9:8 in VREF)
    {"BLUE", 0x01},               //< AWB blue channel gain
    {"RED", 0x02},                //< AWB red channel gain
    {"VREF", 0x03},               //< Vert frame control bits
    {"COM1", 0x04},               //< Common control 1
    {"BAVE", 0x05},               //< U/B average level
    {"GbAVE", 0x06},              //< Y/Gb average level
    {"AECHH", 0x07},              //< Exposure value - AEC 15:10 bits
    {"RAVE", 0x08},               //< V/R average level
    {"COM2", 0x09},               //< Common control 2
    {"PID", 0x0A},                //< Product ID MSB (read-only)
    {"VER", 0x0B},                //< Product ID LSB (read-only)
    {"COM3", 0x0C},               //< Common control 3
    {"COM4", 0x0D},               //< Common control 4
    {"COM5", 0x0E},               //< Common control 5
    {"COM6", 0x0F},               //< Common control 6
    {"AECH", 0x10},               //< Exposure value 9:2
    {"CLKRC", 0x11},              //< Internal clock
    {"COM7", 0x12},               //< Common control 7
    {"COM8", 0x13},               //< Common control 8
    {"COM9", 0x14},               //< Common control 9 - max AGC value
    {"COM10", 0x15},              //< Common control 10
    {"*RSVD*", 0x16},
    {"HSTART", 0x17},             //< Horiz frame start high bits
    {"HSTOP", 0x18},              //< Horiz frame end high bits
    {"VSTART", 0x19},             //< Vert frame start high bits
    {"VSTOP", 0x1A},              //< Vert frame end high bits
    {"PSHFT", 0x1B},              //< Pixel delay select
    {"MIDH", 0x1C},               //< Manufacturer ID high byte
    {"MIDL", 0x1D},               //< Manufacturer ID low byte
    {"MVFP", 0x1E},               //< Mirror / vert-flip enable
    {"LAEC", 0x1F},               //< Reserved
    {"ADCCTR0", 0x20},            //< ADC control
    {"ADCCTR1", 0x21},            //< Reserved
    {"ADCCTR2", 0x22},            //< Reserved
    {"ADCCTR3", 0x23},            //< Reserved
    {"AEW", 0x24},                //< AGC/AEC upper limit
    {"AEB", 0x25},                //< AGC/AEC lower limit
    {"VPT", 0x26},                //< AGC/AEC fast mode op region
    {"BBIAS", 0x27},              //< B channel signal output bias
    {"GbBIAS", 0x28},             //< Gb channel signal output bias
    {"*RSVD*", 0x29},
    {"EXHCH", 0x2A},              //< Dummy pixel insert MSB
    {"EXHCL", 0x2B},              //< Dummy pixel insert LSB
    {"RBIAS", 0x2C},              //< R channel signal output bias
    {"ADVFL", 0x2D},              //< Insert dummy lines MSB
    {"ADVFH", 0x2E},              //< Insert dummy lines LSB
    {"YAVE", 0x2F},               //< Y/G channel average value
    {"HSYST", 0x30},              //< HSYNC rising edge delay
    {"HSYEN", 0x31},              //< HSYNC falling edge delay
    {"HREF", 0x32},               //< HREF control
    {"CHLF", 0x33},               //< Array current control
    {"ARBLM", 0x34},              //< Array ref control - reserved
    {"*RSVD*", 0x35},
    {"*RSVD*", 0x36},
    {"ADC", 0x37},                //< ADC control - reserved
    {"ACOM", 0x38},               //< ADC & analog common - reserved
    {"OFON", 0x39},               //< ADC offset control - reserved
    {"TSLB", 0x3A},               //< Line buffer test option
    {"COM11", 0x3B},              //< Common control 11
    {"COM12", 0x3C},              //< Common control 12
    {"COM13", 0x3D},              //< Common control 13
    {"COM14", 0x3E},              //< Common control 14
    {"EDGE", 0x3F},               //< Edge enhancement adjustment
    {"COM15", 0x40},              //< Common control 15
    {"COM16", 0x41},              //< Common control 16
    {"COM17", 0x42},              //< Common control 17
    {"AWBC1", 0x43},              //< Reserved
    {"AWBC2", 0x44},              //< Reserved
    {"AWBC3", 0x45},              //< Reserved
    {"AWBC4", 0x46},              //< Reserved
    {"AWBC5", 0x47},              //< Reserved
    {"AWBC6", 0x48},              //< Reserved
    {"*RSVD*", 0x49},
    {"*RSVD*", 0x4A},
    {"REG4B", 0x4B},              //< UV average enable
    {"DNSTH", 0x4C},              //< De-noise strength
    {"DM_POS", 0x4D},             // Dummy row position
    {"*RSVD*", 0x4E},
    {"MTX1", 0x4F},               //< Matrix coefficient 1
    {"MTX2", 0x50},               //< Matrix coefficient 2
    {"MTX3", 0x51},               //< Matrix coefficient 3
    {"MTX4", 0x52},               //< Matrix coefficient 4
    {"MTX5", 0x53},               //< Matrix coefficient 5
    {"MTX6", 0x54},               //< Matrix coefficient 6
    {"BRIGHT", 0x55},             //< Brightness control
    {"CONTRAS", 0x56},            //< Contrast control
    {"CONTRAS_CENTER", 0x57},     //< Contrast center
    {"MTXS", 0x58},               //< Matrix coefficient sign
    {"AWBC7", 0x59},              //< AWB Control
    {"AWBC8", 0x5A},              //< AWB control
    {"AWBC9", 0x5B},              //< AWB control
    {"AWBC10", 0x5C},             //< AWB control
    {"AWBC11", 0x5D},             //< AWB control
    {"AWBC12", 0x5E},             //< AWB control
    {"B_LMT", 0x5F},              //< AWB Gain control
    {"R_LMT", 0x60},              //< AWB Gain control
    {"G_LMT", 0x61},              //< AWB Gain control
    {"LCC1", 0x62},               //< Lens correction option 1
    {"LCC2", 0x63},               //< Lens correction option 2
    {"LCC3", 0x64},               //< Lens correction option 3
    {"LCC4", 0x65},               //< Lens correction option 4
    {"LCC5", 0x66},               //< Lens correction option 5
    {"MANU", 0x67},               //< Manual U value
    {"MANV", 0x68},               //< Manual V value
    {"GFIX", 0x69},               //< Fix gain control
    {"GGAIN", 0x6A},              //< G channel AWB gain
    {"DBLV", 0x6B},               //< PLL & regulator control
    {"AWBCTR3", 0x6C},            //< AWB control 3
    {"AWBCTR2", 0x6D},            //< AWB control 2
    {"AWBCTR1", 0x6E},            //< AWB control 1
    {"AWBCTR0", 0x6F},            //< AWB control 0
    {"SCALING_XSC", 0x70},        //< Test pattern X scaling
    {"SCALING_YSC", 0x71},        //< Test pattern Y scaling
    {"SCALING_DCWCTR", 0x72},     //< DCW control
    {"SCALING_PCLK_DIV", 0x73},   //< DSP scale control clock divide
    {"REG74", 0x74},              //< Digital gain control
    {"REG75", 0x75},              //< Edge control
    {"REG76", 0x76},              //< Pixel correction
    {"REG77", 0x77},              //< Offset de-noise
    {"*RSVD*", 0x78},
    {"*RSVD*", 0x79},
    {"SLOP", 0x7A},               //< Gamma curve highest seg slope
    {"GAMA1", 0x7B},              //< Gamma register base (1 of 15)
    {"GAMA2", 0x7C},              //< Gamma register base (1 of 15)
    {"GAMA3", 0x7D},              //< Gamma register base (1 of 15)
    {"GAMA4", 0x7E},              //< Gamma register base (1 of 15)
    {"GAMA5", 0x7F},              //< Gamma register base (1 of 15)
    {"GAMA6", 0x80},              //< Gamma register base (1 of 15)
    {"GAMA7", 0x81},              //< Gamma register base (1 of 15)
    {"GAMA8", 0x82},              //< Gamma register base (1 of 15)
    {"GAMA9", 0x83},              //< Gamma register base (1 of 15)
    {"GAMA10", 0x84},              //< Gamma register base (1 of 15)
    {"GAMA11", 0x85},              //< Gamma register base (1 of 15)
    {"GAMA12", 0x86},              //< Gamma register base (1 of 15)
    {"GAMA13", 0x87},              //< Gamma register base (1 of 15)
    {"GAMA14", 0x88},              //< Gamma register base (1 of 15)
    {"GAMA15", 0x89},              //< Gamma register base (1 of 15)
    {"*RSVD*", 0x8A},
    {"*RSVD*", 0x8B},
    {"?RGB444", 0x8C},             //< RGB 444 control#if 0
    {"*RSVD*", 0x8D},
    {"*RSVD*", 0x8E},
    {"*RSVD*", 0x8F},
    {"*RSVD*", 0x90},
    {"*RSVD*", 0x91},
    {"DM_LNL", 0x92},             //< Dummy line LSB
    {"DM_LNH", 0x93},             //< Dummy line LSB
    {"LCC6", 0x94},               //< Lens correction option 6
    {"LCC7", 0x95},               //< Lens correction option 7
    {"*RSVD*", 0x96},
    {"*RSVD*", 0x97},
    {"*RSVD*", 0x98},
    {"*RSVD*", 0x99},
    {"*RSVD*", 0x9A},
    {"*RSVD*", 0x9B},
    {"*RSVD*", 0x9C},
    {"BD50ST", 0x9D},
    {"BD60ST", 0x9E},
    {"HAECC1", 0x9F},             //< Histogram-based AEC/AGC ctrl 1
    {"HAECC2", 0xA0},             //< Histogram-based AEC/AGC ctrl 2
    {"DSPC3", 0xA1},
    {"SCALING_PCLK_DELAY", 0xA2}, //< Scaling pixel clock delay
    {"*RSVD*", 0xA3},
    {"NT_CTRL", 0xA4},
    {"AECGMAX", 0xA5},            //< 50 Hz banding step limit
    {"LPH", 0xA6},             //< Histogram-based AEC/AGC ctrl 3
    {"UPL", 0xA7},             //< Histogram-based AEC/AGC ctrl 4
    {"TPL", 0xA8},             //< Histogram-based AEC/AGC ctrl 5
    {"TPH", 0xA9},             //< Histogram-based AEC/AGC ctrl 6
    {"NALG", 0xAA},             //< Histogram-based AEC/AGC ctrl 7
    {"*RSVD*", 0xAB},            //< 60 Hz banding step limit
    {"STR-OPT", 0xAC},
    {"STR_R", 0xAD},
    {"STR_G", 0xAE},
    {"STR_B", 0xAF},
    {"*RSVD*", 0xB0},
    {"ABLC1", 0xB1},              //< ABLC enable
    {"*RSVD*", 0xB2},
    {"THL_ST", 0xB3},             //< ABLC target
    {"*RSVD*", 0xB4},
    {"THL_DLT", 0xB5},
    {"*RSVD*", 0xB6},
    {"*RSVD*", 0xB7},
    {"*RSVD*", 0xB8},
    {"*RSVD*", 0xB9},
    {"*RSVD*", 0xBA},
    {"*RSVD*", 0xBB},
    {"*RSVD*", 0xBC},
    {"*RSVD*", 0xBD},
    {"AD-CHB", 0xBE},
    {"AD-CHR", 0xBF},
    {"AD-CHGb", 0xC0},
    {"AD-CHRr", 0xC1},
    {"*RSVD*", 0xC2},
    {"*RSVD*", 0xC3},
    {"*RSVD*", 0xC4},
    {"*RSVD*", 0xC5},
    {"*RSVD*", 0xC6},
    {"*RSVD*", 0xC7},
    {"*RSVD*", 0xC8},
    {"SATCTR", 0xC9}              //< Saturation control

};
#endif 



int arduino_i2c_read(unsigned short address, unsigned char reg, unsigned char *value)
{
#ifdef OV760_DEBUG
    Serial.printf("I2C Read: reg: 0x%02x", reg);

    for (uint16_t i=0; i < (sizeof(ov_reg_name_table)/sizeof(ov_reg_name_table[0])); i++) {
      if (ov_reg_name_table[i].reg == reg) {
        Serial.printf("(%s)", ov_reg_name_table[i].reg_name);
        break;
      }
    }
//    Serial.print("arduino_i2c_read: address = 0x");
//    Serial.print(address, HEX);
//    Serial.print(", reg = 0x");
//    Serial.print(reg, HEX);
#endif

    Wire.beginTransmission(address);
    Wire.write(reg);
    if (Wire.endTransmission() != 0) {
#ifdef OV760_DEBUG
    Serial.println();
#endif
      return -1;
    }

    if (Wire.requestFrom(address, 1) != 1) {
#ifdef OV760_DEBUG
      Serial.println();
#endif
      return -1;
    }

    *value = Wire.read();

#ifdef OV760_DEBUG
    Serial.print(", value = 0x");
    Serial.println(*value, HEX);
#endif

    return 0;
  }

void arduino_i2c_printRegs(unsigned short address) {
    Serial.println("\n*** OV767X registers ***");
    unsigned char value;
    for (uint16_t i=0; i < (sizeof(ov_reg_name_table)/sizeof(ov_reg_name_table[0])); i++) {
        arduino_i2c_read(address, i, &value);
    }
}


  int arduino_i2c_write(unsigned short address, unsigned char reg, unsigned char value)
  {
#ifdef OV760_DEBUG
    Serial.printf("I2C Write: reg: 0x%02x", reg);

    for (uint16_t i=0; i < (sizeof(ov_reg_name_table)/sizeof(ov_reg_name_table[0])); i++) {
      if (ov_reg_name_table[i].reg == reg) {
        Serial.printf("(%s)", ov_reg_name_table[i].reg_name);
        break;
      }
    }
    Serial.printf(", value = 0x%02x\n", value);
#endif

    Wire.beginTransmission(address);
    Wire.write(reg);
    Wire.write(value);
   
    if (Wire.endTransmission() != 0) {
      return -1;
    }

    return 0;
  }
};
