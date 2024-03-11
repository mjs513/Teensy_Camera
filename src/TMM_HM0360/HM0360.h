/*
 * This file is part of the OpenMV project.
 *
 * Copyright (c) 2013-2021 Ibrahim Abdelkader <iabdalkader@openmv.io>
 * Copyright (c) 2013-2021 Kwabena W. Agyeman <kwagyeman@openmv.io>
 *
 * This work is licensed under the MIT license, see the file LICENSE for details.
 *
 * HM01B0 driver.
 */
/*
  *  Parts of this library were re-worked from the Sparkfun HB01B0 Library for the Artemis Platform
  */
/*
Copyright (c) 2019 SparkFun Electronics

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifndef __HM0360_H__
#define __HM0360_H__


//#include "../common.h"
#include "Camera.h"
#include <DMAChannel.h>
#include <Wire.h>
#include <FlexIO_t4.h>


#include "HM0360_regs.h"
#include "HM0360_reg_vals.h"

//Do not touch this define
#define SensorMonochrome 1

class HM0360 : public ImageSensor {
public:

  HM0360();

  void setPins(uint8_t mclk_pin, uint8_t pclk_pin, uint8_t vsync_pin, uint8_t hsync_pin, uint8_t en_pin,
               uint8_t g0, uint8_t g1, uint8_t g2, uint8_t g3,
               uint8_t g4 = 0xff, uint8_t g5 = 0xff, uint8_t g6 = 0xff, uint8_t g7 = 0xff, TwoWire &wire = Wire);
  bool begin(framesize_t framesize = FRAMESIZE_QVGA, int framerate = 30, bool use_gpio = false);
  void end();
  int reset();
  void showRegisters(void);
  void debug(bool debug_on) {_debug = debug_on;}
  bool debug() {return _debug;}
  int setPixformat(pixformat_t pfmt);
  uint8_t setFramesize(framesize_t framesize);
  int setFramerate(int framerate);
  int setBrightness(int level);
  int setGainceiling(gainceiling_t gainceiling);
  int setColorbar(int enable);
  int setAutoGain(int enable, float gain_db, float gain_db_ceiling);
  int get_vt_pix_clk(uint32_t *vt_pix_clk);
  int getGain_db(float *gain_db);
  int getCameraClock(uint32_t *vt_pix_clk);
  int setAutoExposure(int enable, int exposure_us);
  int getExposure_us(int *exposure_us);
  int setHmirror(int enable);
  int setVflip(int enable);
  uint8_t setMode(uint8_t Mode, uint8_t FrameCnt);
  uint8_t cmdUpdate();
  uint8_t loadSettings(camera_reg_settings_t settings);
  uint8_t getAE(ae_cfg_t *psAECfg);
  uint8_t calAE(uint8_t CalFrames, uint8_t *Buffer, uint32_t ui32BufferLen, ae_cfg_t *pAECfg);
  uint16_t getModelid();
  void captureFrameStatistics();
  
  // covers ov functions
  bool begin_omnivision(framesize_t resolution = FRAMESIZE_QVGA, pixformat_t format = RGB565, int fps = 30, bool use_gpio = false) { return 0;};
  void setSaturation(int saturation) {}; // 0 - 255
  void setHue(int hue) {}; // -180 - 180
  void setContrast(int contrast) {}; // 0 - 127
  void setGain(int gain) {}; // 0 - 255
  void autoGain(int enable, float gain_db, float gain_db_ceiling) {};
  void setExposure(int exposure) {}; // 0 - 65535
  void autoExposure(int enable) {};
  // unique to GC2145................................
  void printRegisters(bool only_ones_set = true) {} ;
  int setAutoWhitebal(int enable, float r_gain_db, float g_gain_db, float b_gain_db) { return 0;};

  //-------------------------------------------------------
  //Generic Read Frame base on _hw_config
  void readFrame(void *buffer, bool fUseDMA = true);
  void readFrameSplitBuffer(void *buffer1, size_t cb1, void *buffer2, size_t cb2, bool fUseDMA = true);

  //normal Read mode
  void readFrameGPIO(void* buffer, size_t cb1=(uint32_t)-1, void* buffer2=nullptr, size_t cb2=0);
  void readFrame4BitGPIO(void *buffer);
  bool readContinuous(bool (*callback)(void *frame_buffer), void *fb1, void *fb2);
  void stopReadContinuous();

  //FlexIO is default mode for the camera
  void readFrameFlexIO(void *buffer, size_t cb1=(uint32_t)-1, void* buffer2=nullptr, size_t cb2=0, bool fUseDMA=true);
//  void readFrameFlexIO(void *buffer);
//  void readFrameFlexIO(void *buffer, bool fUseDMA);
  bool startReadFlexIO(bool (*callback)(void *frame_buffer), void *fb1, void *fb2);
  bool stopReadFlexIO();

  // Lets try a dma version.  Doing one DMA that is synchronous does not gain anything
  // So lets have a start, stop... Have it allocate 2 frame buffers and it's own DMA
  // buffers, with the option of setting your own buffers if desired.
  bool startReadFrameDMA(bool (*callback)(void *frame_buffer) = nullptr, uint8_t *fb1 = nullptr, uint8_t *fb2 = nullptr);
  void changeFrameBuffer(uint8_t *fbFrom, uint8_t *fbTo) {
    if (_frame_buffer_1 == fbFrom) _frame_buffer_1 = fbTo;
    else if (_frame_buffer_2 == fbFrom) _frame_buffer_2 = fbTo;
  }
  bool stopReadFrameDMA();

  inline uint32_t frameCount() {
    return _dma_frame_count;
  }
  inline void *frameBuffer() {
    return _dma_last_completed_frame;
  }

  void setVSyncISRPriority(uint8_t priority) {
    NVIC_SET_PRIORITY(IRQ_GPIO6789, priority);
  }
  void setDMACompleteISRPriority(uint8_t priority) {
    NVIC_SET_PRIORITY(dma_flexio.channel & 0xf, priority);
  }

  //-------------------------------------------------------
  uint16_t _width, _height;  //width, height
  int16_t width(void) {
    return _width;
  }
  int16_t height(void) {
    return _height;
  }
  int16_t mode(void) {
    return _hw_config;
  }

  framesize_t framesize;
  pixformat_t pixformat;
  camera_reg_settings_t settings;
  hw_config_t _hw_config;
  hw_carrier_t _hw_carrier;

private:
  void beginXClk();
  void endXClk();

  uint8_t cameraReadRegister(uint16_t reg);
  uint8_t cameraWriteRegister(uint16_t reg, uint8_t data);
  bool flexio_configure();

  uint8_t MCLK_PIN, PCLK_PIN, VSYNC_PIN, HSYNC_PIN, EN_PIN;
  uint8_t G0, G1, G2, G3, G4, G5, G6, G7;
  TwoWire *_wire;
  uint32_t _vsyncMask;
  uint32_t _hrefMask;
  uint32_t _pclkMask;
  const volatile uint32_t *_vsyncPort;
  const volatile uint32_t *_hrefPort;
  const volatile uint32_t *_pclkPort;

  uint32_t XCLK_FREQUENCY = 6000000;

  bool _use_gpio = false;


  // DMA STUFF
  enum { DMABUFFER_SIZE = 1296 };  // 640x480  so 640*2*2
  static DMAChannel _dmachannel;
  static DMASetting _dmasettings[4];
  static uint32_t _dmaBuffer1[DMABUFFER_SIZE];
  static uint32_t _dmaBuffer2[DMABUFFER_SIZE];

  bool (*_callback)(void *frame_buffer) = nullptr;
  uint32_t _dma_frame_count;
  uint8_t *_dma_last_completed_frame;
  // TBD Allow user to set all of the buffers...

  uint8_t _camera_name;

  DMAChannel dma_flexio;

  // Added settings for configurable flexio
  FlexIOHandler *_pflex;
  IMXRT_FLEXIO_t *_pflexio;
  uint8_t _fshifter;
  uint8_t _fshifter_mask;
  uint8_t _ftimer;
  uint8_t _dma_source;
  bool _debug = true;

#if defined(ARDUINO_TEENSY_MICROMOD)
  uint32_t _save_IOMUXC_GPR_GPR27;
#else
  uint32_t _save_IOMUXC_GPR_GPR26;
#endif
  uint32_t _save_pclkPin_portConfigRegister;

  uint32_t _bytes_left_dma;
  uint16_t _save_lsb;
  uint16_t _frame_col_index;              // which column we are in a row
  uint16_t _frame_row_index;              // which row
  const uint16_t _frame_ignore_cols = 0;  // how many cols to ignore per row
  uint8_t *_frame_buffer_1 = nullptr;
  uint8_t *_frame_buffer_2 = nullptr;
  uint8_t *_frame_buffer_pointer;
  uint8_t *_frame_row_buffer_pointer;  // start of the row
  uint8_t _dma_index;
  bool _dma_active;
  enum { DMASTATE_INITIAL = 0,
         DMASTATE_RUNNING,
         DMASTATE_STOP_REQUESTED,
         DMA_STATE_STOPPED };
  volatile uint8_t _dma_state;
  static void dmaInterrupt();
  void processDMAInterrupt();
  static void frameStartInterrupt();
  void processFrameStartInterrupt();
  static void dmaInterruptFlexIO();
  void processDMAInterruptFlexIO();
  static void frameStartInterruptFlexIO();
  void processFrameStartInterruptFlexIO();
  static HM0360 *active_dma_camera;


  //OpenMV support functions extracted from imglib.h

  typedef union {
    uint32_t l;
    struct {
      uint32_t m : 20;
      uint32_t e : 11;
      uint32_t s : 1;
    };
  } exp_t;
  inline float fast_log2(float x) {
    union {
      float f;
      uint32_t i;
    } vx = { x };
    union {
      uint32_t i;
      float f;
    } mx = { (vx.i & 0x007FFFFF) | 0x3f000000 };
    float y = vx.i;
    y *= 1.1920928955078125e-7f;

    return y - 124.22551499f - 1.498030302f * mx.f
           - 1.72587999f / (0.3520887068f + mx.f);
  }
  inline float fast_log(float x) {
    return 0.69314718f * fast_log2(x);
  }
  inline int fast_floorf(float x) {
    int i;
    asm volatile(
      "vcvt.S32.f32  %[r], %[x]\n"
      : [r] "=t"(i)
      : [x] "t"(x));
    return i;
  }
  inline int fast_ceilf(float x) {
    int i;
    x += 0.9999f;
    asm volatile(
      "vcvt.S32.f32  %[r], %[x]\n"
      : [r] "=t"(i)
      : [x] "t"(x));
    return i;
  }
  inline int fast_roundf(float x) {
    int i;
    asm volatile(
      "vcvtr.s32.f32  %[r], %[x]\n"
      : [r] "=t"(i)
      : [x] "t"(x));
    return i;
  }
  inline float fast_expf(float x) {
    exp_t e;
    e.l = (uint32_t)(1512775 * x + 1072632447);
    // IEEE binary32 format
    e.e = (e.e - 1023 + 127) & 0xFF;  // rebase

    //uint32_t packed = (e.s << 31) | (e.e << 23) | e.m <<3;
    //return *((float*)&packed);
    union {
      uint32_t ul;
      float f;
    } packed;
    packed.ul = (e.s << 31) | (e.e << 23) | e.m << 3;
    return packed.f;
  }
};
//Rest is TBD.

#endif  // __HIMAX_H__

/*  
Teensy MicroMod pinouts - 8bit
HIMAX pin      pin#    NXP     Usage
----------      ----    ---     -----
FVLD/VSYNC      33      EMC_07  GPIO
LVLD/HSYNC      32      B0_12   FlexIO2:12
MCLK            7       B1_01   PWM
PCLK            8       B1_00   FlexIO2:16
D0              40      B0_04   FlexIO2:4
D1              41      B0_05   FlexIO2:5
D2              42      B0_06   FlexIO2:6
D3              43      B0_07   FlexIO2:7
D4              44      B0_08   FlexIO2:8  - probably not needed, use 4 bit mode
D5              45      B0_09   FlexIO2:9  - probably not needed, use 4 bit mode
D6              6       B0_10   FlexIO2:10 - probably not needed, use 4 bit mode
D7              9       B0_11   FlexIO2:11 - probably not needed, use 4 bit mode
TRIG            5       EMC_08  ???
INT             29      EMC_31  ???
SCL             19      AD_B1_0 I2C
SDA             18      AD_B1_1 I2C
*/