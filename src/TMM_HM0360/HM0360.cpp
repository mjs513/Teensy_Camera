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

#include "HM0360.h"

#define DEBUG_CAMERA
//#define DEBUG_CAMERA_VERBOSE
//#define DEBUG_FLEXIO
//#define _use_original

#define FLEXIO_TIMER_TRIGGER_SEL_PININPUT(x) ((uint32_t)(x) << 1U)

// Constructor
/*
HM0360::HM0360(hw_carrier_t set_hw_carrier)
{
  _wire = &Wire;
  
  if(set_hw_carrier == SPARKFUN_ML_CARRIER) {
	    _hw_config = TEENSY_MICROMOD_FLEXIO_8BIT;
		VSYNC_PIN = 33;
		PCLK_PIN = 8;
		HSYNC_PIN = 32;
		MCLK_PIN = 7;
		EN_PIN = 2;
		G0 = 40; G1 = 41;  G2 = 42;	G3 = 43;
		G4 = 44; G5 = 45;  G6 = 6;
		G7 = 9;
  } else if(set_hw_carrier == PJRC_CARRIER) {
	    _hw_config = TEENSY_MICROMOD_FLEXIO_4BIT;
		VSYNC_PIN = 32;		//33
		PCLK_PIN = 44;		//8
		HSYNC_PIN = 45;		//32
		MCLK_PIN = 33;		//7
		EN_PIN = 28;		//2
		G0 = 40; G1 = 41;  G2 = 42;	G3 = 43;
		G4 = 0xFF;
  }
  
  //init();
	
}


HM0360::HM0360(uint8_t mclk_pin, uint8_t pclk_pin, uint8_t vsync_pin, uint8_t hsync_pin, uint8_t en_pin,
    uint8_t g0, uint8_t g1,uint8_t g2, uint8_t g3, uint8_t g4, uint8_t g5, uint8_t g6,uint8_t g7, TwoWire &wire) : 
        MCLK_PIN(mclk_pin), PCLK_PIN(pclk_pin), VSYNC_PIN(vsync_pin), HSYNC_PIN(hsync_pin), EN_PIN(en_pin), 
        G0(g0), G1(g1), G2(g2), G3(g3), G4(g4), G5(g5), G6(g6), G7(g7) 
{
  _wire = &wire;
  
  if(g4 == 0xff) {
	    _hw_config = TEENSY_MICROMOD_FLEXIO_4BIT;
  } else {
	    _hw_config = TEENSY_MICROMOD_FLEXIO_8BIT;
  }
  
  //init();
}
*/

HM0360::HM0360() {}

void HM0360::setPins(uint8_t mclk_pin, uint8_t pclk_pin, uint8_t vsync_pin, uint8_t hsync_pin, uint8_t en_pin,
                     uint8_t g0, uint8_t g1, uint8_t g2, uint8_t g3, uint8_t g4, uint8_t g5, uint8_t g6, uint8_t g7, TwoWire &wire) {

  _wire = &wire;

  MCLK_PIN = mclk_pin;
  PCLK_PIN = pclk_pin;
  VSYNC_PIN = vsync_pin;
  HSYNC_PIN = hsync_pin;
  EN_PIN = en_pin,

  G0 = g0;
  G1 = g1;
  G2 = g2;
  G3 = g3;
  G4 = g4;
  G5 = g5;
  G6 = g6;
  G7 = g7;

  if (g4 == 0xff) {
    _hw_config = TEENSY_MICROMOD_FLEXIO_4BIT;
  } else {
    _hw_config = TEENSY_MICROMOD_FLEXIO_8BIT;
  }
}

int HM0360::reset() {
  // Reset sensor.
  if (cameraWriteRegister(SW_RESET, HIMAX_RESET) != 0) {  //cambus_writeb2(&sensor->bus, sensor->slv_addr, SW_RESET, HIMAX_RESET
    return -1;
  }

  // Delay for 1ms.
  delay(1);

  // Write default regsiters
  int ret = 0;
  for (int i = 0; himax_default_regs[i][0] && ret == 0; i++) {
    ret |= cameraWriteRegister(himax_default_regs[i][0], himax_default_regs[i][1]);
  }

  // Set mode to streaming
  //ret |= cameraWriteRegister( MODE_SELECT, HIMAX_MODE_STREAMING);

  return ret;
}



// Read a single uint8_t from address and return it as a uint8_t
uint8_t HM0360::cameraReadRegister(uint16_t reg) {
  _wire->beginTransmission(0x24);
  _wire->write(reg >> 8);
  _wire->write(reg);
  if (_wire->endTransmission(false) != 0) {
    Serial.println("error reading HM0360, address");
    return 0;
  }
  if (_wire->requestFrom(0x24, 1) < 1) {
    Serial.println("error reading HM0360, data");
    return 0;
  }
  return _wire->read();
}


uint8_t HM0360::cameraWriteRegister(uint16_t reg, uint8_t data) {
  _wire->beginTransmission(0x24);
  _wire->write(reg >> 8);
  _wire->write(reg);
  _wire->write(data);
  if (_wire->endTransmission() != 0) {
    Serial.println("error writing to HM0360");
  }
  return 0;
}

int HM0360::setPixformat(pixformat_t pfmt) {
  int ret = 0;
  switch (pfmt) {
    case PIXFORMAT_BAYER:
      pixformat = PIXFORMAT_BAYER;
      break;
    case PIXFORMAT_GRAYSCALE:
      pixformat = PIXFORMAT_GRAYSCALE;
      break;
    default:
      pixformat = PIXFORMAT_INVALID;
      return -1;
  }

  return ret;
}

uint8_t HM0360::setFramesize(framesize_t new_framesize) {
  int ret = 0;
  //uint16_t w = resolution[framesize][0];
  //uint16_t h = resolution[framesize][1];
  framesize = new_framesize;

  switch (framesize) {
    case FRAMESIZE_QVGA:
      _width = 320;
      _height = 240;
      for (int i = 0; himax_qvga_regs[i][0] && ret == 0; i++) {
        ret |= cameraWriteRegister(himax_qvga_regs[i][0], himax_qvga_regs[i][1]);
      }
      if(_hw_config == TEENSY_MICROMOD_FLEXIO_4BIT ) 
         ret |= cameraWriteRegister(0x310F, 0x40);
      break;
    case FRAMESIZE_QVGA4BIT:
      _width = 320;
      _height = 240;
      for (int i = 0; himax_qvga4bit_regs[i][0] && ret == 0; i++) {
        ret |= cameraWriteRegister(himax_qvga4bit_regs[i][0], himax_qvga4bit_regs[i][1]);
      }
      break;
    case FRAMESIZE_QQVGA:
      _width = 160;
      _height = 120;
      for (int i = 0; himax_qqvga_regs[i][0] && ret == 0; i++) {
        ret |= cameraWriteRegister(himax_qqvga_regs[i][0], himax_qqvga_regs[i][1]);
      }
      if(_hw_config == TEENSY_MICROMOD_FLEXIO_4BIT ) 
         ret |= cameraWriteRegister(0x310F, 0x40);
      break;
    case FRAMESIZE_VGA:
      _width = 640;
      _height = 480;
      for (int i = 0; himax_vga_regs[i][0] && ret == 0; i++) {
        ret |= cameraWriteRegister(himax_vga_regs[i][0], himax_vga_regs[i][1]);
      }
      if(_hw_config == TEENSY_MICROMOD_FLEXIO_4BIT ) 
         ret |= cameraWriteRegister(0x310F, 0x40);
      break;
    default:
      ret = -1;
  }
  return ret;
}

int HM0360::setFramerate(int framerate) {
  uint8_t pll_cfg = 0;
  uint8_t osc_div = 0;
  bool highres = false;
  //framesize_t framesize;

  if (framesize == FRAMESIZE_VGA) {
    highres = true;
  }

  if (framerate <= 10) {
    osc_div = (highres == true) ? 0x03 : 0x03;
  } else if (framerate <= 15) {
    osc_div = (highres == true) ? 0x02 : 0x03;
  } else if (framerate <= 30) {
    osc_div = (highres == true) ? 0x01 : 0x02;
  } else {
    // Set to the max possible FPS at this resolution.
    osc_div = (highres == true) ? 0x00 : 0x01;
  }

  pll_cfg = cameraReadRegister(PLL1_CONFIG);
  return cameraWriteRegister(PLL1_CONFIG, (pll_cfg & 0xFC) | osc_div);
}

int HM0360::setBrightness(int level) {
  uint8_t ae_mean;
  // Simulate brightness levels by setting AE loop target mean.
  switch (level) {
    case 0:
      ae_mean = 60;
      break;
    case 1:
      ae_mean = 80;
      break;
    case 2:
      ae_mean = 100;
      break;
    case 3:
      ae_mean = 127;
      break;
    default:
      ae_mean = 60;
  }
  return cameraWriteRegister(AE_TARGET_MEAN, ae_mean);
}

int HM0360::setGainceiling(gainceiling_t gainceiling) {
  int gain = 0;

  switch (gainceiling) {
    case GAINCEILING_2X:
      gain = 0x01;
      break;
    case GAINCEILING_4X:
      gain = 0x02;
      break;
    case GAINCEILING_8X:
      gain = 0x03;
      break;
    case GAINCEILING_16X:
      gain = 0x04;
      break;
    default:
      return -1;
  }

  return cameraWriteRegister(MAX_AGAIN, (gain & 0x07));
  ;
}

int HM0360::setColorbar(int enable) {
  return cameraWriteRegister(TEST_PATTERN_MODE, enable & 0x1);
}

int HM0360::setAutoGain(int enable, float gain_db, float gain_db_ceiling) {
  int ret = 0;
  uint8_t ae_ctrl = 0;

  ae_ctrl = cameraReadRegister(AE_CTRL);
  if ((enable == 0) && (!isnanf(gain_db)) && (!isinff(gain_db))) {
    gain_db = max(min(gain_db, 24.0f), 0.0f);
    int gain = fast_ceilf(fast_log2(fast_expf((gain_db / 20.0f) * fast_log(10.0f))));
    ret |= cameraWriteRegister(AE_CTRL, 0);  // Must disable AE
    ret |= cameraWriteRegister(ANALOG_GAIN, ((gain & 0x7) << 4));
  } else if ((enable != 0) && (!isnanf(gain_db_ceiling)) && (!isinff(gain_db_ceiling))) {
    gain_db_ceiling = max(min(gain_db_ceiling, 24.0f), 0.0f);
    int gain = fast_ceilf(fast_log2(fast_expf((gain_db_ceiling / 20.0f) * fast_log(10.0f))));
    ret |= cameraWriteRegister(MAX_AGAIN, (gain & 0x7));
    ret |= cameraWriteRegister(AE_CTRL, (ae_ctrl | 0x01));
  }
  ret |= cameraWriteRegister(COMMAND_UPDATE, 0x01);
  return ret;
}

int HM0360::getCameraClock(uint32_t *vt_pix_clk) {
  return XCLK_FREQUENCY;
}

uint8_t HM0360::getAE(ae_cfg_t *psAECfg) {
  return -1;
}

uint8_t HM0360::calAE(uint8_t CalFrames, uint8_t *Buffer, uint32_t ui32BufferLen, ae_cfg_t *pAECfg) {
  return -1;
}

int HM0360::get_vt_pix_clk(uint32_t *vt_pix_clk) {
  uint8_t reg;
  reg = cameraReadRegister(PLL1_CONFIG);

  // 00 -> MCLK / 8
  // 01 -> MCLK / 4
  // 10 -> MCLK / 2
  // 11 -> MCLK / 1
  uint32_t vt_sys_div = 8 / (1 << (reg & 0x03));

  // vt_pix_clk = MCLK / vt_sys_div
  *vt_pix_clk = XCLK_FREQUENCY / vt_sys_div;
  return 0;
}


int HM0360::getGain_db(float *gain_db) {
  uint8_t gain;
  gain = cameraReadRegister(ANALOG_GAIN);
  if (gain != 0) {
    return -1;
  }
  *gain_db = fast_floorf(fast_log(1 << (gain >> 4)) / fast_log(10.0f) * 20.0f);
  return 0;
}

int HM0360::setAutoExposure(int enable, int exposure_us) {
  int ret = 0;
  uint8_t ae_ctrl = 0;

  ae_ctrl = cameraReadRegister(AE_CTRL);
  if (enable) {
    ret |= cameraWriteRegister(AE_CTRL, (ae_ctrl | 0x01));
  } else {
    uint32_t line_len;
    uint32_t frame_len;
    uint32_t coarse_int;
    uint32_t vt_pix_clk = 0;

    switch (framesize) {
      case FRAMESIZE_320X320:
        line_len = HIMAX_LINE_LEN_PCK_FULL;
        frame_len = HIMAX_FRAME_LENGTH_FULL;
        break;
      case FRAMESIZE_QVGA:
        line_len = HIMAX_LINE_LEN_PCK_QVGA;
        frame_len = HIMAX_FRAME_LENGTH_QVGA;
        break;
      case FRAMESIZE_QQVGA:
        line_len = HIMAX_LINE_LEN_PCK_QQVGA;
        frame_len = HIMAX_FRAME_LENGTH_QQVGA;
        break;
      default:
        return -1;
    }

    ret |= get_vt_pix_clk(&vt_pix_clk);
    coarse_int = fast_roundf(exposure_us * (vt_pix_clk / 1000000.0f) / line_len);

    if (coarse_int < 2) {
      coarse_int = 2;
    } else if (coarse_int > (frame_len - 2)) {
      coarse_int = frame_len - 2;
    }

    ret |= cameraWriteRegister(AE_CTRL, (ae_ctrl & 0xFE));
    ret |= cameraWriteRegister(INTEGRATION_H, coarse_int >> 8);
    ret |= cameraWriteRegister(INTEGRATION_L, coarse_int & 0xff);
    ret |= cameraWriteRegister(COMMAND_UPDATE, 0x01);
  }

  return ret;
}

int HM0360::getExposure_us(int *exposure_us) {
  uint32_t line_len;
  uint32_t coarse_int = 0;
  uint32_t vt_pix_clk = 0;

  if (framesize == FRAMESIZE_VGA) {
    line_len = HIMAX_LINE_LEN_PCK_VGA;
  } else if (framesize == FRAMESIZE_QVGA) {
    line_len = HIMAX_LINE_LEN_PCK_QVGA;
  } else {
    line_len = HIMAX_LINE_LEN_PCK_QQVGA;
  }

  get_vt_pix_clk(&vt_pix_clk);
  ((uint8_t *)&coarse_int)[1] = cameraReadRegister(INTEGRATION_H);
  ((uint8_t *)&coarse_int)[0] = cameraReadRegister(INTEGRATION_L);

  *exposure_us = fast_roundf(coarse_int * line_len / (vt_pix_clk / 1000000.0f));
  return 0;
}

int HM0360::setHmirror(int enable) {
  uint8_t reg;
  reg = cameraReadRegister(IMG_ORIENTATION);
  int ret = reg;
  ret |= cameraWriteRegister(IMG_ORIENTATION, HIMAX_SET_HMIRROR(reg, enable));
  ret |= cameraWriteRegister(COMMAND_UPDATE, 0x01);
  return ret;
}

int HM0360::setVflip(int enable) {
  uint8_t reg;
  reg = cameraReadRegister(IMG_ORIENTATION);
  int ret = reg;
  ret |= cameraWriteRegister(IMG_ORIENTATION, HIMAX_SET_VMIRROR(reg, enable));
  ret |= cameraWriteRegister(COMMAND_UPDATE, 0x01);
  return ret;
}

uint8_t HM0360::setMode(uint8_t Mode, uint8_t FrameCnt) {
  uint8_t Err = 0;

  if (Mode == HIMAX_MODE_STREAMING_NFRAMES) {
    Err = cameraWriteRegister(0x3028, FrameCnt);
  } else {
    Err = cameraWriteRegister(MODE_SELECT, Mode);
  }

  if (Err != 0) {
    Serial.println("Mode Could not be set");
  }

  return Err;
}

uint8_t HM0360::cmdUpdate() {
  uint8_t status = 0;
  status = cameraWriteRegister(COMMAND_UPDATE, 0x01);
  return status;
}


uint8_t HM0360::loadSettings(camera_reg_settings_t settings) {
  uint8_t ret = 0;
  switch (settings) {
    case LOAD_DEFAULT_REGS:
      for (int i = 0; himax_default_regs[i][0] && ret == 0; i++) {
        ret |= cameraWriteRegister(himax_default_regs[i][0], himax_default_regs[i][1]);
      }
      break;
    case LOAD_320x240:
      for (int i = 0; himax_hm0360_320x240[i][0] && ret == 0; i++) {
        ret |= cameraWriteRegister(himax_hm0360_320x240[i][0], himax_hm0360_320x240[i][1]);
      }
      break;
    default:
      ret = -1;
  }
  return ret;
}


//*****************************************************************************
//
//! @brief Get HM01B0 Model ID
//!
//! @param psCfg                - Pointer to HM01B0 configuration structure.
//! @param pui16MID             - Pointer to buffer for the read back model ID.
//!
//! This function reads back HM01B0 model ID.
//!
//! @return Error code.
//
//*****************************************************************************
uint16_t HM0360::getModelid() {
  uint8_t Data;
  uint16_t MID = 0x0000;

  Data = cameraReadRegister(MODEL_ID_H);
  MID = (Data << 8);

  Data = cameraReadRegister(MODEL_ID_L);
  MID |= Data;

  return MID;
}


bool HM0360::begin(framesize_t framesize, int framerate, bool use_gpio) {
  _use_gpio = use_gpio;

  _wire->begin();

  pinMode(VSYNC_PIN, INPUT_PULLDOWN);  // VSYNC Pin
  pinMode(PCLK_PIN, INPUT_PULLDOWN);   //PCLK
  pinMode(HSYNC_PIN, INPUT_PULLDOWN);  //HSYNC
  //pinMode(MCLK_PIN, OUTPUT);

  /*Thanks to @luni for how to read 8bit port	\
	 * https://forum.pjrc.com/threads/66771-MicroMod-Beta-Testing?p=275567&viewfull=1#post275567
	 * This interesting too: https://forum.pjrc.com/threads/57698-Parallel-IO-is-it-possible?p=216501&viewfull=1#post216501
	*/
  if (G4 == 0xff) {
    for (uint8_t pin : { G0, G1, G2, G3 }) {
      pinMode(pin, INPUT_PULLUP);
    }
  } else {
    for (uint8_t pin : { G0, G1, G2, G3, G4, G5, G6, G7 }) {
      pinMode(pin, INPUT_PULLUP);
    }
  }

#ifdef DEBUG_CAMERA
  Serial.printf("  VS=%d, HR=%d, PC=%d XC=%d\n", VSYNC_PIN, HSYNC_PIN, PCLK_PIN, MCLK_PIN);
  Serial.printf("  G0 = %d\n", G0);
  Serial.printf("  G1 = %d\n", G1);
  Serial.printf("  G2 = %d\n", G2);
  Serial.printf("  G3 = %d\n", G3);
  if (G4 != 0xFF) {
    Serial.printf("  G4 = %d\n", G4);
    Serial.printf("  G5 = %d\n", G5);
    Serial.printf("  G6 = %d\n", G6);
    Serial.printf("  G7 = %d\n", G7);
  }

#endif

  _vsyncMask = digitalPinToBitMask(VSYNC_PIN);
  _hrefMask = digitalPinToBitMask(HSYNC_PIN);
  _pclkMask = digitalPinToBitMask(PCLK_PIN);

  _vsyncPort = portInputRegister(digitalPinToPort(VSYNC_PIN));
  _hrefPort = portInputRegister(digitalPinToPort(HSYNC_PIN));
  _pclkPort = portInputRegister(digitalPinToPort(PCLK_PIN));

  // turn on power to camera (1.8V - might be an issue?)
  if (EN_PIN < 255) {
    pinMode(EN_PIN, OUTPUT);
    digitalWrite(EN_PIN, HIGH);
  }
  delay(10);

  // turn on MCLK
  beginXClk();
  
  Serial.printf("SENSOR ID :-) MODEL HM0%X\n", getModelid());

  if (!_use_gpio) {
    flexio_configure();
  }
  setVSyncISRPriority(102);
  setDMACompleteISRPriority(192);

  reset();

  setPixformat(PIXFORMAT_GRAYSCALE);  //Sparkfun camera only supports grayscale
  setFramesize(framesize);
  setFramerate(framerate);

  Serial.printf("GPIO: %d, Sensor Width: %d, Sensor Height: %d\n",use_gpio, _width, _height);

  //set_mode(HIMAX_MODE_STREAMING,0);

  return 1;
}

void HM0360::end()
{
  endXClk();

  pinMode(MCLK_PIN, INPUT);

  Wire.end();

}

void HM0360::beginXClk()
{
  // Generates MCLK clock
  analogWriteFrequency(MCLK_PIN, XCLK_FREQUENCY);
  analogWrite(MCLK_PIN, 128);
  delay(100);

}

void HM0360::endXClk()
{
  analogWrite(XCLK_FREQUENCY, 0);
}

#define FLEXIO_USE_DMA
void HM0360::readFrame(void *buffer, bool fUseDMA) {
  setMode(HIMAX_MODE_STREAMING_NFRAMES, 1);
  if (!_use_gpio) {
#if defined(_use_original)
    readFrameFlexIO(buffer);
#else
    readFrameFlexIO(buffer, (size_t)-1, nullptr, 0, fUseDMA);
#endif
  } else {
    if(_hw_config == TEENSY_MICROMOD_FLEXIO_4BIT) {
        readFrame4BitGPIO(buffer);
    } else {
        readFrameGPIO(buffer);
    }
  }
}

void HM0360::readFrameSplitBuffer(void *buffer1, size_t cb1, void *buffer2, size_t cb2, bool fUseDMA) {
    if(!_use_gpio) {
        readFrameFlexIO(buffer1, cb1, buffer2, cb2, fUseDMA);
    } else {
        readFrameGPIO(buffer1, cb1, buffer2, cb2);
    }

}



bool HM0360::readContinuous(bool (*callback)(void *frame_buffer), void *fb1, void *fb2) {
  setMode(HIMAX_MODE_STREAMING_NFRAMES, 1);

  return startReadFlexIO(callback, fb1, fb2);
}

void HM0360::stopReadContinuous() {

  stopReadFlexIO();
}

void HM0360::readFrameGPIO(void *buffer, size_t cb1, void *buffer2, size_t cb2)
{
  uint8_t *b = (uint8_t *)buffer;
  uint32_t cb = (uint32_t)cb1;
  bool _grayscale;
  int bytesPerRow;
//Change for Monodchrome only Sparkfun HB01b0
#if defined(SensorMonochrome)
  _grayscale = false;
  bytesPerRow = _width;
#else
  _grayscale = (pixformat == PIXFORMAT_GRAYSCALE);
  bytesPerRow = _width * 2;
#endif

  // Falling edge indicates start of frame
  //pinMode(PCLK_PIN, INPUT); // make sure back to input pin...
  // lets add our own glitch filter.  Say it must be hig for at least 100us
  elapsedMicros emHigh;
  do {
    while ((*_vsyncPort & _vsyncMask) == 0)
      ;  // wait for HIGH
    emHigh = 0;
    while ((*_vsyncPort & _vsyncMask) != 0)
      ;  // wait for LOW
  } while (emHigh < 2);

  for (int i = 0; i < _height; i++) {
    // rising edge indicates start of line
    while ((*_hrefPort & _hrefMask) == 0)
      ;  // wait for HIGH
    while ((*_pclkPort & _pclkMask) != 0)
      ;  // wait for LOW
    noInterrupts();

    for (int j = 0; j < bytesPerRow; j++) {
      // rising edges clock each data byte
      while ((*_pclkPort & _pclkMask) == 0)
        ;  // wait for HIGH

      //uint32_t in = ((_frame_buffer_pointer)? GPIO1_DR : GPIO6_DR) >> 18; // read all bits in parallel
      uint32_t in = (GPIO7_PSR >> 4);  // read all bits in parallel
                                       //uint32_t in = mmBus;

      if (!(j & 1) || !_grayscale) {
        *b++ = in;
        if ( buffer2 && (--cb == 0) ) {
          Serial.printf("\t$$ 2nd buffer: %u %u\n", i, j);
          b = (uint8_t *)buffer2;
          cb = (uint32_t)cb2;
          buffer2 = nullptr;
        }
      }
      while (((*_pclkPort & _pclkMask) != 0) && ((*_hrefPort & _hrefMask) != 0))
        ;  // wait for LOW bail if _href is lost
    }

    while ((*_hrefPort & _hrefMask) != 0)
      ;  // wait for LOW
    interrupts();
  }

  setMode(HIMAX_MODE_STREAMING, 0);
}

void HM0360::readFrame4BitGPIO(void *buffer) {

  uint8_t *b = (uint8_t *)buffer;
  bool _grayscale;
  int bytesPerRow;
  uint8_t in0 = 0;

//Change for Monodchrome only Sparkfun HB01b0
#if defined(SensorMonochrome)
  _grayscale = false;
  bytesPerRow = _width * 2;
#else
  _grayscale = (pixformat == PIXFORMAT_GRAYSCALE);
  bytesPerRow = _width * 2 * 2;
#endif

  Serial.printf("readFrame4BitGPIO Gray:%u bpr: %u\n", _grayscale, bytesPerRow);
  // Falling edge indicates start of frame
  //pinMode(PCLK_PIN, INPUT); // make sure back to input pin...
  // lets add our own glitch filter.  Say it must be hig for at least 100us
  elapsedMicros emHigh;
  do {
    while ((*_vsyncPort & _vsyncMask) == 0)
      ;  // wait for HIGH
    emHigh = 0;
    while ((*_vsyncPort & _vsyncMask) != 0)
      ;  // wait for LOW
  } while (emHigh < 1);

  for (int i = 0; i < _height; i++) {
    // rising edge indicates start of line
    while ((*_hrefPort & _hrefMask) == 0)
      ;  // wait for HIGH
    while ((*_pclkPort & _pclkMask) != 0)
      ;  // wait for LOW
    noInterrupts();

    int skip_first_bytes_in_row = bytesPerRow /*/ 2 */;

    for (int j = 0; j < (bytesPerRow + skip_first_bytes_in_row); j++) {
      // rising edges clock each data byte
      while ((*_pclkPort & _pclkMask) == 0)
        ;  // wait for HIGH

      //uint32_t in = ((_frame_buffer_pointer)? GPIO1_DR : GPIO6_DR) >> 18; // read all bits in parallel
      if (j >= skip_first_bytes_in_row) {
        uint8_t in = (GPIO7_PSR >> 4);  // read all bits in parallel
        //uint32_t in = mmBus;
        in &= 0x0F;

        if ((j + 1) % 2) {
          in = (in0 << 4) | (in);
          if (!(j & 1) || !_grayscale) {
            *b++ = in;
          }
        } else {
          in0 = in;
        }
      }
      while (((*_pclkPort & _pclkMask) != 0) && ((*_hrefPort & _hrefMask) != 0))
        ;  // wait for LOW bail if _href is lost
    }

    while ((*_hrefPort & _hrefMask) != 0)
      ;  // wait for LOW
    interrupts();
  }

  setMode(HIMAX_MODE_STREAMING, 0);
}

#if defined(_use_original)
bool HM0360::flexio_configure() {
  // Going to try this using my FlexIO library.

  // BUGBUG - starting off not going to worry about maybe first pin I choos is on multipl Flex IO controllers (yet)
  uint8_t tpclk_pin;
  _pflex = FlexIOHandler::mapIOPinToFlexIOHandler(PCLK_PIN, tpclk_pin);
  if (!_pflex) {
    Serial.printf("HM0360 PCLK(%u) is not a valid Flex IO pin\n", PCLK_PIN);
    return false;
  }
  _pflexio = &(_pflex->port());

  // Quick and dirty:
  uint8_t thsync_pin = _pflex->mapIOPinToFlexPin(HSYNC_PIN);
  uint8_t tg0 = _pflex->mapIOPinToFlexPin(G0);
  uint8_t tg1 = _pflex->mapIOPinToFlexPin(G1);
  uint8_t tg2 = _pflex->mapIOPinToFlexPin(G2);
  uint8_t tg3 = _pflex->mapIOPinToFlexPin(G3);

  // make sure the minimum here is valid:
  if ((thsync_pin == 0xff) || (tg0 == 0xff) || (tg1 == 0xff) || (tg2 == 0xff) || (tg3 == 0xff)) {
    Serial.printf("HM0360 Some pins did not map to valid Flex IO pin\n");
    Serial.printf("    HSYNC(%u %u) G0(%u %u) G1(%u %u) G2(%u %u) G3(%u %u)",
                  HSYNC_PIN, thsync_pin, G0, tg0, G1, tg1, G2, tg2, G3, tg3);
    return false;
  }
  // Verify that the G numbers are consecutive... Should use arrays!
  if ((tg1 != (tg0 + 1)) || (tg2 != (tg0 + 2)) || (tg3 != (tg0 + 3))) {
    Serial.printf("HM0360 Flex IO pins G0-G3 are not consective\n");
    Serial.printf("    G0(%u %u) G1(%u %u) G2(%u %u) G3(%u %u)",
                  G0, tg0, G1, tg1, G2, tg2, G3, tg3);
    return false;
  }
  if (G4 != 0xff) {
    uint8_t tg4 = _pflex->mapIOPinToFlexPin(G4);
    uint8_t tg5 = _pflex->mapIOPinToFlexPin(G5);
    uint8_t tg6 = _pflex->mapIOPinToFlexPin(G6);
    uint8_t tg7 = _pflex->mapIOPinToFlexPin(G7);
    if ((tg4 != (tg0 + 4)) || (tg5 != (tg0 + 5)) || (tg6 != (tg0 + 6)) || (tg7 != (tg0 + 7))) {
      Serial.printf("HM0360 Flex IO pins G4-G7 are not consective with G0-3\n");
      Serial.printf("    G0(%u %u) G4(%u %u) G5(%u %u) G6(%u %u) G7(%u %u)",
                    G0, tg0, G4, tg4, G5, tg5, G6, tg6, G7, tg7);
      return false;
    }
    _hw_config = TEENSY_MICROMOD_FLEXIO_8BIT;
    Serial.println("Custom - Flexio is 8 bit mode");
  } else {
    _hw_config = TEENSY_MICROMOD_FLEXIO_4BIT;
    Serial.println("Custom - Flexio is 4 bit mode");
  }

  // Needs Shifter 3 (maybe 7 would work as well?)
  // Needs Shifter 3 (maybe 7 would work as well?)
  if (_pflex->claimShifter(3)) _fshifter = 3;
  else if (_pflex->claimShifter(7)) _fshifter = 7;
  else {
    Serial.printf("HM0360 Flex IO: Could not claim Shifter 3 or 7\n");
    return false;
  }
  _fshifter_mask = 1 << _fshifter;                      // 4 channels.
  _dma_source = _pflex->shiftersDMAChannel(_fshifter);  // looks like they use

  // Now request one timer
  uint8_t _ftimer = _pflex->requestTimers();  // request 1 timer.
  if (_ftimer == 0xff) {
    Serial.printf("HM01B0 Flex IO: failed to request timer\n");
    return false;
  }

  _pflex->setIOPinToFlexMode(HSYNC_PIN);
  _pflex->setIOPinToFlexMode(PCLK_PIN);
  _pflex->setIOPinToFlexMode(G0);
  _pflex->setIOPinToFlexMode(G1);
  _pflex->setIOPinToFlexMode(G2);
  _pflex->setIOPinToFlexMode(G3);
  if (G4 != 0xff) {
    _pflex->setIOPinToFlexMode(G4);
    _pflex->setIOPinToFlexMode(G5);
    _pflex->setIOPinToFlexMode(G6);
    _pflex->setIOPinToFlexMode(G7);
  }



  // We already configured the clock to allow access.
  // Now sure yet aoub configuring the actual colock speed...

  /*
    CCM_CSCMR2 |= CCM_CSCMR2_FLEXIO2_CLK_SEL(3); // 480 MHz from USB PLL

    CCM_CS1CDR = (CCM_CS1CDR
        & ~(CCM_CS1CDR_FLEXIO2_CLK_PRED(7) | CCM_CS1CDR_FLEXIO2_CLK_PODF(7)))
        | CCM_CS1CDR_FLEXIO2_CLK_PRED(1) | CCM_CS1CDR_FLEXIO2_CLK_PODF(1);


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
  Serial.printf(" FlexIO2 Frequency = %.2f MHz\n", 480.0 / (float)div1 / (float)div2);
  Serial.printf(" CCM_CCGR3 = %08X\n", CCM_CCGR3);
  Serial.printf(" FLEXIO2_CTRL = %08X\n", FLEXIO2_CTRL);
  Serial.printf(" FlexIO2 Config, param=%08X\n", FLEXIO2_PARAM);
#endif

  if (_hw_config == TEENSY_MICROMOD_FLEXIO_8BIT) {
    Serial.println("8Bit FlexIO");
    // SHIFTCFG, page 2927
    //  PWIDTH: number of bits to be shifted on each Shift clock
    //          0 = 1 bit, 1-3 = 4 bit, 4-7 = 8 bit, 8-15 = 16 bit, 16-31 = 32 bit
    //  INSRC: Input Source, 0 = pin, 1 = Shifter N+1 Output
    //  SSTOP: Stop bit, 0 = disabled, 1 = match, 2 = use zero, 3 = use one
    //  SSTART: Start bit, 0 = disabled, 1 = disabled, 2 = use zero, 3 = use one
    _pflexio->SHIFTCFG[_fshifter] = FLEXIO_SHIFTCFG_PWIDTH(7);

    // Timer model, pages 2891-2893
    // TIMCMP, page 2937
    _pflexio->TIMCMP[_ftimer] = 7;

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
    //#define FLEXIO_TIMER_TRIGGER_SEL_PININPUT(x) ((uint32_t)(x) << 1U)
    _pflexio->TIMCTL[_ftimer] = FLEXIO_TIMCTL_TIMOD(3)
                                | FLEXIO_TIMCTL_PINSEL(tpclk_pin)                                      // "Pin" is 16 = PCLK
                                | FLEXIO_TIMCTL_TRGSEL(FLEXIO_TIMER_TRIGGER_SEL_PININPUT(thsync_pin))  // "Trigger" is 12 = HSYNC
                                | FLEXIO_TIMCTL_TRGSRC;
  }

  if (_hw_config == TEENSY_MICROMOD_FLEXIO_4BIT) {

    // SHIFTCFG, page 2927
    //  PWIDTH: number of bits to be shifted on each Shift clock
    //          0 = 1 bit, 1-3 = 4 bit, 4-7 = 8 bit, 8-15 = 16 bit, 16-31 = 32 bit
    //  INSRC: Input Source, 0 = pin, 1 = Shifter N+1 Output
    //  SSTOP: Stop bit, 0 = disabled, 1 = match, 2 = use zero, 3 = use one
    //  SSTART: Start bit, 0 = disabled, 1 = disabled, 2 = use zero, 3 = use one
    _pflexio->SHIFTCFG[_fshifter] = FLEXIO_SHIFTCFG_PWIDTH(3);

    // Timer model, pages 2891-2893
    // TIMCMP, page 2937
    _pflexio->TIMCMP[_ftimer] = 15;

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
    _pflexio->TIMCTL[_ftimer] = FLEXIO_TIMCTL_TIMOD(3)
                                | FLEXIO_TIMCTL_PINSEL(tpclk_pin)                                      // "Pin" is 16 = PCLK
                                | FLEXIO_TIMCTL_TRGSEL(FLEXIO_TIMER_TRIGGER_SEL_PININPUT(thsync_pin))  // "Trigger" is 12 = HSYNC
                                | FLEXIO_TIMCTL_TRGSRC;
  }


  // SHIFTCTL, page 2926
  //  TIMSEL: which Timer is used for controlling the logic/shift register
  //  TIMPOL: 0 = shift of positive edge, 1 = shift on negative edge
  //  PINCFG: 0 = output disabled, 1 = open drain, 2 = bidir, 3 = output
  //  PINSEL: which pin is used by the Shifter input or output
  //  PINPOL: 0 = active high, 1 = active low
  //  SMOD: 0 = disable, 1 = receive, 2 = transmit, 4 = match store,
  //        5 = match continuous, 6 = state machine, 7 = logic
  _pflexio->SHIFTCTL[_fshifter] = FLEXIO_SHIFTCTL_TIMSEL(_ftimer) | FLEXIO_SHIFTCTL_SMOD(1)
                                  | FLEXIO_SHIFTCTL_PINSEL(tg0);  // 4 = D0



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
  _pflexio->CTRL = FLEXIO_CTRL_FLEXEN;  // enable after everything configured

#ifdef DEBUG_FLEXIO
  Serial.printf(" FLEXIO:%u Shifter:%u Timer:%u\n", _pflex->FlexIOIndex(), _fshifter, _ftimer);
  Serial.printf("     SHIFTCFG = %08X\n", _pflexio->SHIFTCFG[_fshifter]);
  Serial.printf("     SHIFTCTL = %08X\n", _pflexio->SHIFTCTL[_fshifter]);
  Serial.printf("     TIMCMP = %08X\n", _pflexio->TIMCMP[_ftimer]);
  Serial.printf("     TIMCFG = %08X\n", _pflexio->TIMCFG[_ftimer]);
  Serial.printf("     TIMCTL = %08X\n", _pflexio->SHIFTCTL[_fshifter]);
#endif
  return true;
}


void HM0360::readFrameFlexIO(void *buffer, size_t cb1, void* buffer2, size_t cb2, bool use_dma)
  //flexio_configure(); // one-time hardware setup
  // wait for VSYNC to be low
  const uint32_t frame_size_bytes = _width*_height*/*_bytesPerPixel*/;
  while ((*_vsyncPort & _vsyncMask) != 0)
    ;

  _pflexio->SHIFTSTAT = _fshifter_mask;  // clear any prior shift status
  _pflexio->SHIFTERR = _fshifter_mask;

  uint32_t *p = (uint32_t *)buffer;
  if (!use_dma) {
    // read FlexIO by polling
    //uint32_t *p_end = (uint32_t *)buffer + _width*_height/4; ???

    uint32_t count_items_left = (_width*_height/4)/**_bytesPerPixel */;
    uint32_t count_items_left_in_buffer = (uint32_t)cb1 / 4;

    while (count_items_left) {

    while (p < p_end) {
      while ((_pflexio->SHIFTSTAT & _fshifter_mask) == 0) {
        // wait for FlexIO shifter data
      }
      
      *p++ = _pflexio->SHIFTBUF[_fshifter];  // should use DMA...
      count_items_left--;
      if (buffer2 && (--count_items_left_in_buffer == 0)) {
        p = (uint32_t*)buffer2;
        count_items_left_in_buffer = (uint32_t)cb2 / 4;
      }
    }
    return;
  }
  // read FlexIO by DMA
  dma_flexio.begin();
  const uint32_t length = _width * _height;
  // Will it fit into one DMA object. 
  if ((_width <= 320) && (buffer2 == nullptr)) {
    dma_flexio.source(_pflexio->SHIFTBUF[_fshifter]);
    dma_flexio.destinationBuffer((uint32_t *)buffer, length);
    dma_flexio.transferSize(4);
    dma_flexio.transferCount(length / 4);
    dma_flexio.disableOnCompletion();
    dma_flexio.clearComplete();
    dma_flexio.triggerAtHardwareEvent(_dma_source);
  } else {
    // We need to use dma settings.
    if (cb1 >= frame_size_bytes) {
      // fit it into 3 _dmasettings
      uint32_t cb_per_setting = ((frame_size_bytes / 3) + 4) & 0xfffffffc; // round up to next multiple of 4.
      _dmasettings[0].TCD->CSR = 0;
      _dmasettings[0].source(_pflexio->SHIFTBUF[_fshifter]);
      _dmasettings[0].destinationBuffer(p, cb_per_setting);
      _dmasettings[0].replaceSettingsOnCompletion(_dmasettings[1]);

      _dmasettings[1].TCD->CSR = 0;
      _dmasettings[1].source(_pflexio->SHIFTBUF[_fshifter]);
      _dmasettings[1].destinationBuffer(&p[cb_per_setting / 4], cb_per_setting);
      _dmasettings[1].replaceSettingsOnCompletion(_dmasettings[2]);

      _dmasettings[2].source(_pflexio->SHIFTBUF[_fshifter]);
      _dmasettings[2].destinationBuffer(&p[cb_per_setting / 2], cb1 - 2 * cb_per_setting);
      _dmasettings[2].replaceSettingsOnCompletion(_dmasettings[0]);
      _dmasettings[2].disableOnCompletion();
      _dmasettings[2].interruptAtCompletion();
    } else {
      if ((cb1 + cb2) < frame_size_bytes) return; // not big enough
      // have two buffers - see how we may need to distribute the dma settings.
      uint8_t count_dma_settings = (cb1 / (32767 * 4)) + 1;
      uint32_t cb_per_setting = ((cb1 / count_dma_settings) + 4) & 0xfffffffc; // round up to next multiple of 4.
      
      uint8_t dmas_index = 0;
      uint8_t cb_left = cb1;
      for (; dmas_index < count_dma_settings; dmas_index++) {
        _dmasettings[dmas_index].TCD->CSR = 0;
        _dmasettings[dmas_index].source(_pflexio->SHIFTBUF[_fshifter]);
        _dmasettings[dmas_index].destinationBuffer(p, min(cb_per_setting, cb_left));
        _dmasettings[dmas_index].replaceSettingsOnCompletion(_dmasettings[dmas_index + 1]);
        p += (cb_per_setting / 4);
        cb_left -= cb_per_setting
      }

      cb_left = frame_size_bytes - cb1;
      count_dma_settings = (cb_left / (32767 * 4)) + 1;
      cb_per_setting = ((cb_left / count_dma_settings) + 4) & 0xfffffffc; // round up to next multiple of 4.
      uint32_t *p = (uint32_t *)buffer2;

      for (uint8_t i = 0; i < count_dma_settings; i++, dmas_index++) {
        _dmasettings[dmas_index].TCD->CSR = 0;
        _dmasettings[dmas_index].source(_pflexio->SHIFTBUF[_fshifter]);
        _dmasettings[dmas_index].destinationBuffer(p, min(cb_per_setting, cb_left));
        _dmasettings[dmas_index].replaceSettingsOnCompletion(_dmasettings[dmas_index + 1]);
        p += (cb_per_setting / 4);
        cb_left -= cb_per_setting
      }
      dmas_index--; // lets point back to the last one
      _dmasettings[dmas_index].replaceSettingsOnCompletion(_dmasettings[0]);
      _dmasettings[dmas_index].disableOnCompletion();
      _dmasettings[dmas_index].interruptAtCompletion();

    }
    dma_flexio = _dmasettings[0];
    dma_flexio.clearComplete();

  }

  dma_flexio.enable();
  _pflexio->SHIFTSDEN = _fshifter_mask;

  elapsedMillis timeout = 0;
  while (!dma_flexio.complete()) {
    // wait - we should not need to actually do anything during the DMA transfer
    if (dma_flexio.error()) {
      Serial.println("DMA error");
      break;
    }
    if (timeout > 500) {
      Serial.println("Timeout waiting for DMA");
      if (_pflexio->SHIFTSTAT & _fshifter_mask) Serial.println(" SHIFTSTAT bit was set");
      Serial.printf(" DMA channel #%u\n", dma_flexio.channel);
      Serial.printf(" DMAMUX = %08X\n", *(&DMAMUX_CHCFG0 + dma_flexio.channel));
      Serial.printf(" FLEXIO2_SHIFTSDEN = %02X\n", FLEXIO2_SHIFTSDEN);
      Serial.printf(" TCD CITER = %u\n", dma_flexio.TCD->CITER_ELINKNO);
      Serial.printf(" TCD CSR = %08X\n", dma_flexio.TCD->CSR);
      break;
    }
  }
  arm_dcache_delete(buffer, length);
}



bool HM0360::startReadFlexIO(bool (*callback)(void *frame_buffer), void *fb1, void *fb2) {
#ifdef FLEXIO_USE_DMA
  if (fb1 == nullptr || fb2 == nullptr) return false;
  _frame_buffer_1 = (uint8_t *)fb1;
  _frame_buffer_2 = (uint8_t *)fb2;
  _callback = callback;
  active_dma_camera = this;
  //Serial.printf("startReadFrameFlexIO called buffers %x %x\n", (uint32_t)fb1, (uint32_t)fb2);

  //flexio_configure(); // one-time hardware setup
  dma_flexio.begin();
  const uint32_t length = _width * _height;
  dma_flexio.source(_pflexio->SHIFTBUF[_fshifter]);
  dma_flexio.destinationBuffer((uint32_t *)fb1, length);
  dma_flexio.transferSize(4);
  dma_flexio.transferCount(length / 4);
  dma_flexio.disableOnCompletion();
  dma_flexio.clearComplete();
  dma_flexio.triggerAtHardwareEvent(_dma_source);
  dma_flexio.interruptAtCompletion();
  dma_flexio.attachInterrupt(dmaInterruptFlexIO);
  FLEXIO2_SHIFTSDEN = _fshifter_mask;
  _dma_frame_count = 0;
  _dma_active = false;

  // wait for VSYNC to be low
  while ((*_vsyncPort & _vsyncMask) != 0)
    ;
  //NVIC_SET_PRIORITY(IRQ_GPIO6789, 102);
  //NVIC_SET_PRIORITY(dma_flexio.channel & 0xf, 102);
  attachInterrupt(VSYNC_PIN, &frameStartInterruptFlexIO, RISING);
  return true;
#else
  return false;
#endif
}


void HM0360::frameStartInterruptFlexIO() {
  active_dma_camera->processFrameStartInterruptFlexIO();
}

void HM0360::processFrameStartInterruptFlexIO() {
  if (!_dma_active) {
    FLEXIO2_SHIFTSTAT = _fshifter_mask;  // clear any prior shift status
    FLEXIO2_SHIFTERR = _fshifter_mask;

    // TODO: could a prior status have a DMA request still be pending?
    void *dest = (_dma_frame_count & 1) ? _frame_buffer_2 : _frame_buffer_1;
    const uint32_t length = _width * _height;
    //dma_flexio.TCD->DADDR = dest;
    dma_flexio.destinationBuffer((uint32_t *)dest, length);
    dma_flexio.transferSize(4);
    dma_flexio.transferCount(length / 4);
    dma_flexio.enable();
    //Serial.println("VSYNC");
    _dma_active = true;
  }
  asm("DSB");
}

void HM0360::dmaInterruptFlexIO() {
  active_dma_camera->processDMAInterruptFlexIO();
}

void HM0360::processDMAInterruptFlexIO() {
  dma_flexio.clearInterrupt();
  if (dma_flexio.error()) return;  // TODO: report or handle error??
  void *dest = (_dma_frame_count & 1) ? _frame_buffer_2 : _frame_buffer_1;
  const uint32_t length = _width * _height;
  _dma_frame_count++;
  arm_dcache_delete(dest, length);
  if (_callback) (*_callback)(dest);  // TODO: use EventResponder
  _dma_active = false;
  asm("DSB");
}


bool HM0360::stopReadFlexIO() {
  detachInterrupt(VSYNC_PIN);
  dma_flexio.disable();
  _frame_buffer_1 = nullptr;
  _frame_buffer_2 = nullptr;
  _callback = nullptr;
  return true;
}


//======================================== DMA JUNK
//================================================================================
// experiment with DMA
//================================================================================
// Define our DMA structure.
DMAChannel HM0360::_dmachannel;
DMASetting HM0360::_dmasettings[4];
uint32_t HM0360::_dmaBuffer1[DMABUFFER_SIZE] __attribute__((used, aligned(32)));
uint32_t HM0360::_dmaBuffer2[DMABUFFER_SIZE] __attribute__((used, aligned(32)));
extern "C" void xbar_connect(unsigned int input, unsigned int output);  // in pwm.c

HM0360 *HM0360::active_dma_camera = nullptr;

void dumpDMA_TCD1(DMABaseClass *dmabc) {
  Serial.printf("%lx %lx:", (uint32_t)dmabc, (uint32_t)dmabc->TCD);

  Serial.printf("SA:%lx SO:%d AT:%x NB:%lx SL:%ld DA:%lx DO:%d CI:%x DL:%ld CS:%x BI:%x\n",
                (uint32_t)dmabc->TCD->SADDR,
                dmabc->TCD->SOFF, dmabc->TCD->ATTR, dmabc->TCD->NBYTES, dmabc->TCD->SLAST, (uint32_t)dmabc->TCD->DADDR,
                dmabc->TCD->DOFF, dmabc->TCD->CITER, dmabc->TCD->DLASTSGA, dmabc->TCD->CSR, dmabc->TCD->BITER);
}


//===================================================================
// Start a DMA operation -
//===================================================================
bool HM0360::startReadFrameDMA(bool (*callback)(void *frame_buffer), uint8_t *fb1, uint8_t *fb2) {
  // First see if we need to allocate frame buffers.
  if (fb1) _frame_buffer_1 = fb1;
  else if (_frame_buffer_1 == nullptr) {
    _frame_buffer_1 = (uint8_t *)malloc(_width * _height);
    if (_frame_buffer_1 == nullptr) return false;
  }
  if (fb2) _frame_buffer_2 = fb2;
  else if (_frame_buffer_2 == nullptr) {
    _frame_buffer_2 = (uint8_t *)malloc(_width * _height);
    if (_frame_buffer_2 == nullptr) return false;  // BUGBUG should we 32 byte align?
  }
  // remember the call back if passed in
  _callback = callback;
  active_dma_camera = this;

  Serial.printf("startReadFrameDMA called buffers %x %x\n", (uint32_t)_frame_buffer_1, (uint32_t)_frame_buffer_2);

  //DebugDigitalToggle(OV7670_DEBUG_PIN_1);
  // lets figure out how many bytes we will tranfer per setting...
  //  _dmasettings[0].begin();
  _frame_row_buffer_pointer = _frame_buffer_pointer = (uint8_t *)_frame_buffer_1;

  // configure DMA channels
  _dmachannel.begin();
  _dmasettings[0].source(GPIO2_PSR);                                   // setup source.
  _dmasettings[0].destinationBuffer(_dmaBuffer1, DMABUFFER_SIZE * 4);  // 32 bits per logical byte
  _dmasettings[0].replaceSettingsOnCompletion(_dmasettings[1]);
  _dmasettings[0].interruptAtCompletion();          // we will need an interrupt to process this.
  _dmasettings[0].TCD->CSR &= ~(DMA_TCD_CSR_DREQ);  // Don't disable on this one
  //DebugDigitalToggle(OV7670_DEBUG_PIN_1);

  _dmasettings[1].source(GPIO2_PSR);                                   // setup source.
  _dmasettings[1].destinationBuffer(_dmaBuffer2, DMABUFFER_SIZE * 4);  // 32 bits per logical byte
  _dmasettings[1].replaceSettingsOnCompletion(_dmasettings[0]);
  _dmasettings[1].interruptAtCompletion();          // we will need an interrupt to process this.
  _dmasettings[1].TCD->CSR &= ~(DMA_TCD_CSR_DREQ);  // Don't disable on this one
  //DebugDigitalToggle(OV7670_DEBUG_PIN_1);

  GPIO2_GDIR = 0;  // set all as input...
  GPIO2_DR = 0;    // see if I can clear it out...

  _dmachannel = _dmasettings[0];  // setup the first on...
  _dmachannel.attachInterrupt(dmaInterrupt);
  _dmachannel.triggerAtHardwareEvent(DMAMUX_SOURCE_XBAR1_0);
  //DebugDigitalToggle(OV7670_DEBUG_PIN_1);

  // Lets try to setup the DMA setup...
  // first see if we can convert the _pclk to be an XBAR Input pin...
  // OV7670_PLK   4
  // OV7670_PLK   8    //8       B1_00   FlexIO2:16  XBAR IO14

  _save_pclkPin_portConfigRegister = *(portConfigRegister(PCLK_PIN));
  *(portConfigRegister(PCLK_PIN)) = 1;  // set to XBAR mode 14

  // route the timer outputs through XBAR to edge trigger DMA request
  CCM_CCGR2 |= CCM_CCGR2_XBAR1(CCM_CCGR_ON);
  xbar_connect(XBARA1_IN_IOMUX_XBAR_INOUT14, XBARA1_OUT_DMA_CH_MUX_REQ30);
  //DebugDigitalToggle(OV7670_DEBUG_PIN_1);

  // Tell XBAR to dDMA on Rising
  XBARA1_CTRL0 = XBARA_CTRL_STS0 | XBARA_CTRL_EDGE0(1) | XBARA_CTRL_DEN0 /* | XBARA_CTRL_IEN0 */;

  IOMUXC_GPR_GPR6 &= ~(IOMUXC_GPR_GPR6_IOMUXC_XBAR_DIR_SEL_14);  // Make sure it is input mode
  IOMUXC_XBAR1_IN14_SELECT_INPUT = 1;                            // Make sure this signal goes to this pin...


#if defined(ARDUINO_TEENSY_MICROMOD)
  // Need to switch the IO pins back to GPI1 from GPIO6
  _save_IOMUXC_GPR_GPR27 = IOMUXC_GPR_GPR27;  // save away the configuration before we change...
  IOMUXC_GPR_GPR27 &= ~(0x0ff0u);

  // lets also un map the _hrefPin to GPIO1
  IOMUXC_GPR_GPR27 &= ~_hrefMask;  //
#else
  // Need to switch the IO pins back to GPI1 from GPIO6
  _save_IOMUXC_GPR_GPR26 = IOMUXC_GPR_GPR26;  // save away the configuration before we change...
  IOMUXC_GPR_GPR26 &= ~(0x0ff0u);

  // lets also un map the _hrefPin to GPIO1
  IOMUXC_GPR_GPR26 &= ~_hrefMask;             //
#endif

  // Need to switch the IO pins back to GPI1 from GPIO6
  //_save_IOMUXC_GPR_GPR27 = IOMUXC_GPR_GPR27;  // save away the configuration before we change...
  //IOMUXC_GPR_GPR27 &= ~(0x0ff0u);

  // lets also un map the _hrefPin to GPIO1
  //IOMUXC_GPR_GPR27 &= ~_hrefMask; //


  //DebugDigitalToggle(OV7670_DEBUG_PIN_1);

  // Falling edge indicates start of frame
  //  while ((*_vsyncPort & _vsyncMask) == 0); // wait for HIGH
  //  while ((*_vsyncPort & _vsyncMask) != 0); // wait for LOW
  //  DebugDigitalWrite(OV7670_DEBUG_PIN_2, HIGH);

  // Debug stuff for now

  // We have the start of a frame, so lets start the dma.
#ifdef DEBUG_CAMERA
  dumpDMA_TCD1(&_dmachannel);
  dumpDMA_TCD1(&_dmasettings[0]);
  dumpDMA_TCD1(&_dmasettings[1]);
  Serial.printf("pclk pin: %d config:%lx control:%lx\n", PCLK_PIN, *(portConfigRegister(PCLK_PIN)), *(portControlRegister(PCLK_PIN)));
  Serial.printf("IOMUXC_GPR_GPR26-29:%lx %lx %lx %lx\n", IOMUXC_GPR_GPR26, IOMUXC_GPR_GPR27, IOMUXC_GPR_GPR28, IOMUXC_GPR_GPR29);
  Serial.printf("GPIO1: %lx %lx, GPIO6: %lx %lx\n", GPIO1_DR, GPIO1_PSR, GPIO6_DR, GPIO6_PSR);
  Serial.printf("XBAR CTRL0:%x CTRL1:%x\n\n", XBARA1_CTRL0, XBARA1_CTRL1);
#endif
  _dma_state = DMASTATE_RUNNING;
  _dma_last_completed_frame = nullptr;
  _dma_frame_count = 0;

  // Now start an interrupt for start of frame.
  attachInterrupt(VSYNC_PIN, &frameStartInterrupt, RISING);

  //DebugDigitalToggle(OV7670_DEBUG_PIN_1);
  return true;
}

//===================================================================
// stopReadFrameDMA - stop doing the reading and then exit.
//===================================================================
bool HM0360::stopReadFrameDMA() {

  // hopefully it start here (fingers crossed)
  // for now will hang here to see if completes...
  //DebugDigitalWrite(OV7670_DEBUG_PIN_2, HIGH);
  elapsedMillis em = 0;
  // tell the background stuff DMA stuff to exit.
  // Note: for now let it end on on, later could disable the DMA directly.
  _dma_state = DMASTATE_STOP_REQUESTED;

  while ((em < 1000) && (_dma_state == DMASTATE_STOP_REQUESTED))
    ;  // wait up to a second...
  if (_dma_state != DMA_STATE_STOPPED) {
    Serial.println("*** stopReadFrameDMA DMA did not exit correctly...");
    Serial.printf("  Bytes Left: %u frame buffer:%x Row:%u Col:%u\n", _bytes_left_dma, (uint32_t)_frame_buffer_pointer, _frame_row_index, _frame_col_index);
  }
  //DebugDigitalWrite(OV7670_DEBUG_PIN_2, LOW);

#ifdef DEBUG_CAMERA
  dumpDMA_TCD1(&_dmachannel);
  dumpDMA_TCD1(&_dmasettings[0]);
  dumpDMA_TCD1(&_dmasettings[1]);
  Serial.println();
#endif
  // Lets restore some hardware pieces back to the way we found them.
#if defined(ARDUINO_TEENSY_MICROMOD)
  IOMUXC_GPR_GPR27 = _save_IOMUXC_GPR_GPR27;  // Restore... away the configuration before we change...
#else
  IOMUXC_GPR_GPR26 = _save_IOMUXC_GPR_GPR26;  // Restore... away the configuration before we change...
#endif
  *(portConfigRegister(PCLK_PIN)) = _save_pclkPin_portConfigRegister;

  return (em < 1000);  // did we stop...
}

//===================================================================
// Our Frame Start interrupt.
//===================================================================
void HM0360::frameStartInterrupt() {
  active_dma_camera->processFrameStartInterrupt();  // lets get back to the main object...
}

void HM0360::processFrameStartInterrupt() {
  _bytes_left_dma = (_width + _frame_ignore_cols) * _height;  // for now assuming color 565 image...
  _dma_index = 0;
  _frame_col_index = 0;  // which column we are in a row
  _frame_row_index = 0;  // which row
  _save_lsb = 0xffff;
  // make sure our DMA is setup properly again.
  _dmasettings[0].transferCount(DMABUFFER_SIZE);
  _dmasettings[0].TCD->CSR &= ~(DMA_TCD_CSR_DREQ);  // Don't disable on this one
  _dmasettings[1].transferCount(DMABUFFER_SIZE);
  _dmasettings[1].TCD->CSR &= ~(DMA_TCD_CSR_DREQ);  // Don't disable on this one
  _dmachannel = _dmasettings[0];                    // setup the first on...
  _dmachannel.enable();

  detachInterrupt(VSYNC_PIN);
}

//===================================================================
// Our DMA interrupt.
//===================================================================
void HM0360::dmaInterrupt() {
  active_dma_camera->processDMAInterrupt();  // lets get back to the main object...
}


// This version assumes only called when HREF...  as set pixclk to only fire
// when set.
void HM0360::processDMAInterrupt() {
  _dmachannel.clearInterrupt();  // tell system we processed it.
  asm("DSB");
  //DebugDigitalWrite(OV7670_DEBUG_PIN_3, HIGH);

  if (_dma_state == DMA_STATE_STOPPED) {
    Serial.println("HM0360::dmaInterrupt called when DMA_STATE_STOPPED");
    return;  //
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
    _bytes_left_dma--;  // for now assuming color 565 image...
    buffer++;
  }

  if ((_frame_row_index == _height) || (_bytes_left_dma == 0)) {  // We finished a frame lets bail
    _dmachannel.disable();                                        // disable the DMA now...
                                                                  //DebugDigitalWrite(OV7670_DEBUG_PIN_2, LOW);
#ifdef DEBUG_CAMERA_VERBOSE
    Serial.println("EOF");
#endif
    _frame_row_index = 0;
    _dma_frame_count++;

    bool swap_buffers = true;

    //DebugDigitalToggle(OV7670_DEBUG_PIN_1);
    _dma_last_completed_frame = _frame_row_buffer_pointer;
    if (_callback) swap_buffers = (*_callback)(_dma_last_completed_frame);

    if (swap_buffers) {
      if (_frame_row_buffer_pointer != _frame_buffer_1) _frame_row_buffer_pointer = _frame_buffer_2;
      else _frame_row_buffer_pointer = _frame_buffer_2;
    }

    _frame_buffer_pointer = _frame_row_buffer_pointer;

    //DebugDigitalToggle(OV7670_DEBUG_PIN_1);


    if (_dma_state == DMASTATE_STOP_REQUESTED) {
#ifdef DEBUG_CAMERA
      Serial.println("HM0360::dmaInterrupt - Stop requested");
#endif
      _dma_state = DMA_STATE_STOPPED;
    } else {
      // We need to start up our ISR for the next frame.
#if 1
      // bypass interrupt and just restart DMA...
      _bytes_left_dma = (_width + _frame_ignore_cols) * _height;  // for now assuming color 565 image...
      _dma_index = 0;
      _frame_col_index = 0;  // which column we are in a row
      _frame_row_index = 0;  // which row
      _save_lsb = 0xffff;
      // make sure our DMA is setup properly again.
      _dmasettings[0].transferCount(DMABUFFER_SIZE);
      _dmasettings[0].TCD->CSR &= ~(DMA_TCD_CSR_DREQ);  // Don't disable on this one
      _dmasettings[1].transferCount(DMABUFFER_SIZE);
      _dmasettings[1].TCD->CSR &= ~(DMA_TCD_CSR_DREQ);  // Don't disable on this one
      _dmachannel = _dmasettings[0];                    // setup the first on...
      _dmachannel.enable();

#else
      attachInterrupt(VSYNC_PIN, &frameStartInterrupt, RISING);
#endif
    }
  } else {

    if (_bytes_left_dma == (2 * DMABUFFER_SIZE)) {
      if (_dma_index & 1) _dmasettings[0].disableOnCompletion();
      else _dmasettings[1].disableOnCompletion();
    }
  }
  //DebugDigitalWrite(OV7670_DEBUG_PIN_3, LOW);
}
#else

bool HM0360::flexio_configure() {
  // Going to try this using my FlexIO library.

  // BUGBUG - starting off not going to worry about maybe first pin I choos is on multipl Flex IO controllers (yet)
  uint8_t tpclk_pin;
  _pflex = FlexIOHandler::mapIOPinToFlexIOHandler(PCLK_PIN, tpclk_pin);
  if (!_pflex) {
    Serial.printf("HM0360 PCLK(%u) is not a valid Flex IO pin\n", PCLK_PIN);
    return false;
  }
  _pflexio = &(_pflex->port());

  // Quick and dirty:
  uint8_t thsync_pin = _pflex->mapIOPinToFlexPin(HSYNC_PIN);
  uint8_t tg0 = _pflex->mapIOPinToFlexPin(G0);
  uint8_t tg1 = _pflex->mapIOPinToFlexPin(G1);
  uint8_t tg2 = _pflex->mapIOPinToFlexPin(G2);
  uint8_t tg3 = _pflex->mapIOPinToFlexPin(G3);

  // make sure the minimum here is valid:
  if ((thsync_pin == 0xff) || (tg0 == 0xff) || (tg1 == 0xff) || (tg2 == 0xff) || (tg3 == 0xff)) {
    Serial.printf("HM0360 Some pins did not map to valid Flex IO pin\n");
    Serial.printf("    HSYNC(%u %u) G0(%u %u) G1(%u %u) G2(%u %u) G3(%u %u)",
                  HSYNC_PIN, thsync_pin, G0, tg0, G1, tg1, G2, tg2, G3, tg3);
    return false;
  }
  // Verify that the G numbers are consecutive... Should use arrays!
  if ((tg1 != (tg0 + 1)) || (tg2 != (tg0 + 2)) || (tg3 != (tg0 + 3))) {
    Serial.printf("HM0360 Flex IO pins G0-G3 are not consective\n");
    Serial.printf("    G0(%u %u) G1(%u %u) G2(%u %u) G3(%u %u)",
                  G0, tg0, G1, tg1, G2, tg2, G3, tg3);
    return false;
  }
  if (G4 != 0xff) {
    uint8_t tg4 = _pflex->mapIOPinToFlexPin(G4);
    uint8_t tg5 = _pflex->mapIOPinToFlexPin(G5);
    uint8_t tg6 = _pflex->mapIOPinToFlexPin(G6);
    uint8_t tg7 = _pflex->mapIOPinToFlexPin(G7);
    if ((tg4 != (tg0 + 4)) || (tg5 != (tg0 + 5)) || (tg6 != (tg0 + 6)) || (tg7 != (tg0 + 7))) {
      Serial.printf("HM0360 Flex IO pins G4-G7 are not consective with G0-3\n");
      Serial.printf("    G0(%u %u) G4(%u %u) G5(%u %u) G6(%u %u) G7(%u %u)",
                    G0, tg0, G4, tg4, G5, tg5, G6, tg6, G7, tg7);
      return false;
    }
    _hw_config = TEENSY_MICROMOD_FLEXIO_8BIT;
    Serial.println("Custom - Flexio is 8 bit mode");
  } else {
    _hw_config = TEENSY_MICROMOD_FLEXIO_4BIT;
    Serial.println("Custom - Flexio is 4 bit mode");
  }

  // Needs Shifter 3 (maybe 7 would work as well?)
  // Needs Shifter 3 (maybe 7 would work as well?)
  if (_pflex->claimShifter(3)) _fshifter = 3;
  else if (_pflex->claimShifter(7)) _fshifter = 7;
  else {
    Serial.printf("HM0360 Flex IO: Could not claim Shifter 3 or 7\n");
    return false;
  }
  _fshifter_mask = 1 << _fshifter;                      // 4 channels.
  _dma_source = _pflex->shiftersDMAChannel(_fshifter);  // looks like they use

  // Now request one timer
  uint8_t _ftimer = _pflex->requestTimers();  // request 1 timer.
  if (_ftimer == 0xff) {
    Serial.printf("HM0360 Flex IO: failed to request timer\n");
    return false;
  }

  _pflex->setIOPinToFlexMode(HSYNC_PIN);
  _pflex->setIOPinToFlexMode(PCLK_PIN);
  _pflex->setIOPinToFlexMode(G0);
  _pflex->setIOPinToFlexMode(G1);
  _pflex->setIOPinToFlexMode(G2);
  _pflex->setIOPinToFlexMode(G3);
  if (G4 != 0xff) {
    _pflex->setIOPinToFlexMode(G4);
    _pflex->setIOPinToFlexMode(G5);
    _pflex->setIOPinToFlexMode(G6);
    _pflex->setIOPinToFlexMode(G7);
  }



  // We already configured the clock to allow access.
  // Now sure yet aoub configuring the actual colock speed...

  /*
    CCM_CSCMR2 |= CCM_CSCMR2_FLEXIO2_CLK_SEL(3); // 480 MHz from USB PLL

    CCM_CS1CDR = (CCM_CS1CDR
        & ~(CCM_CS1CDR_FLEXIO2_CLK_PRED(7) | CCM_CS1CDR_FLEXIO2_CLK_PODF(7)))
        | CCM_CS1CDR_FLEXIO2_CLK_PRED(1) | CCM_CS1CDR_FLEXIO2_CLK_PODF(1);


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
  Serial.printf(" FlexIO2 Frequency = %.2f MHz\n", 480.0 / (float)div1 / (float)div2);
  Serial.printf(" CCM_CCGR3 = %08X\n", CCM_CCGR3);
  Serial.printf(" FLEXIO2_CTRL = %08X\n", FLEXIO2_CTRL);
  Serial.printf(" FlexIO2 Config, param=%08X\n", FLEXIO2_PARAM);
#endif

  if (_hw_config == TEENSY_MICROMOD_FLEXIO_8BIT) {
    Serial.println("8Bit FlexIO");
    // SHIFTCFG, page 2927
    //  PWIDTH: number of bits to be shifted on each Shift clock
    //          0 = 1 bit, 1-3 = 4 bit, 4-7 = 8 bit, 8-15 = 16 bit, 16-31 = 32 bit
    //  INSRC: Input Source, 0 = pin, 1 = Shifter N+1 Output
    //  SSTOP: Stop bit, 0 = disabled, 1 = match, 2 = use zero, 3 = use one
    //  SSTART: Start bit, 0 = disabled, 1 = disabled, 2 = use zero, 3 = use one
    _pflexio->SHIFTCFG[_fshifter] = FLEXIO_SHIFTCFG_PWIDTH(7);

    // Timer model, pages 2891-2893
    // TIMCMP, page 2937
    _pflexio->TIMCMP[_ftimer] = 7;

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
    //#define FLEXIO_TIMER_TRIGGER_SEL_PININPUT(x) ((uint32_t)(x) << 1U)
    _pflexio->TIMCTL[_ftimer] = FLEXIO_TIMCTL_TIMOD(3)
                                | FLEXIO_TIMCTL_PINSEL(tpclk_pin)                                      // "Pin" is 16 = PCLK
                                | FLEXIO_TIMCTL_TRGSEL(FLEXIO_TIMER_TRIGGER_SEL_PININPUT(thsync_pin))  // "Trigger" is 12 = HSYNC
                                | FLEXIO_TIMCTL_TRGSRC;
  }

  if (_hw_config == TEENSY_MICROMOD_FLEXIO_4BIT) {

    // SHIFTCFG, page 2927
    //  PWIDTH: number of bits to be shifted on each Shift clock
    //          0 = 1 bit, 1-3 = 4 bit, 4-7 = 8 bit, 8-15 = 16 bit, 16-31 = 32 bit
    //  INSRC: Input Source, 0 = pin, 1 = Shifter N+1 Output
    //  SSTOP: Stop bit, 0 = disabled, 1 = match, 2 = use zero, 3 = use one
    //  SSTART: Start bit, 0 = disabled, 1 = disabled, 2 = use zero, 3 = use one
    _pflexio->SHIFTCFG[_fshifter] = FLEXIO_SHIFTCFG_PWIDTH(3);

    // Timer model, pages 2891-2893
    // TIMCMP, page 2937
    _pflexio->TIMCMP[_ftimer] = 15;

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
    _pflexio->TIMCTL[_ftimer] = FLEXIO_TIMCTL_TIMOD(3)
                                | FLEXIO_TIMCTL_PINSEL(tpclk_pin)                                      // "Pin" is 16 = PCLK
                                | FLEXIO_TIMCTL_TRGSEL(FLEXIO_TIMER_TRIGGER_SEL_PININPUT(thsync_pin))  // "Trigger" is 12 = HSYNC
                                | FLEXIO_TIMCTL_TRGSRC;
  }


  // SHIFTCTL, page 2926
  //  TIMSEL: which Timer is used for controlling the logic/shift register
  //  TIMPOL: 0 = shift of positive edge, 1 = shift on negative edge
  //  PINCFG: 0 = output disabled, 1 = open drain, 2 = bidir, 3 = output
  //  PINSEL: which pin is used by the Shifter input or output
  //  PINPOL: 0 = active high, 1 = active low
  //  SMOD: 0 = disable, 1 = receive, 2 = transmit, 4 = match store,
  //        5 = match continuous, 6 = state machine, 7 = logic
  _pflexio->SHIFTCTL[_fshifter] = FLEXIO_SHIFTCTL_TIMSEL(_ftimer) | FLEXIO_SHIFTCTL_SMOD(1)
                                  | FLEXIO_SHIFTCTL_PINSEL(tg0);  // 4 = D0



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
  _pflexio->CTRL = FLEXIO_CTRL_FLEXEN;  // enable after everything configured

#ifdef DEBUG_FLEXIO
  Serial.printf(" FLEXIO:%u Shifter:%u Timer:%u\n", _pflex->FlexIOIndex(), _fshifter, _ftimer);
  Serial.printf("     SHIFTCFG = %08X\n", _pflexio->SHIFTCFG[_fshifter]);
  Serial.printf("     SHIFTCTL = %08X\n", _pflexio->SHIFTCTL[_fshifter]);
  Serial.printf("     TIMCMP = %08X\n", _pflexio->TIMCMP[_ftimer]);
  Serial.printf("     TIMCFG = %08X\n", _pflexio->TIMCFG[_ftimer]);
  Serial.printf("     TIMCTL = %08X\n", _pflexio->SHIFTCTL[_fshifter]);
#endif
  return true;
}

extern void dumpDMA_TCD1(DMABaseClass *dmabc, const char *psz_title=nullptr);

void HM0360::readFrameFlexIO(void *buffer, size_t cb1, void* buffer2, size_t cb2, bool use_dma) {
  if (_debug) Serial.printf("$$HM0360::readFrameFlexIO(%p, %u, %p, %u, %u)\n", buffer, cb1, buffer2, cb2, use_dma);
  //flexio_configure(); // one-time hardware setup
  // wait for VSYNC to be low
  //while ((*_vsyncPort & _vsyncMask) != 0);
  // lets wait for a vsync that is high long enough to not be pin noise
  elapsedMillis timeout = 0;
  const uint32_t frame_size_bytes = _width*_height /* * _bytesPerPixel*/;

  for (;;) {
    if (((*_vsyncPort & _vsyncMask) == 0) && ((*_vsyncPort & _vsyncMask) == 0) && ((*_vsyncPort & _vsyncMask) == 0) && ((*_vsyncPort & _vsyncMask) == 0)) break;
    if (timeout > 500) {
      Serial.println("Timeout waiting for VSYNC");
      return;
    }
  }

  digitalToggleFast(5);
  _pflexio->SHIFTSTAT = _fshifter_mask;  // clear any prior shift status
  _pflexio->SHIFTERR = _fshifter_mask;

  //#ifndef FLEXIO_USE_DMA
  // read FlexIO by polling
  uint32_t *p = (uint32_t *)buffer;

  if (!use_dma) {
    _pflexio->SHIFTSDEN = _fshifter_mask;

    uint32_t count_items_left = (_width*_height/4)/**_bytesPerPixel */;
    uint32_t count_items_left_in_buffer = (uint32_t)cb1 / 4;
    if (_debug) Serial.printf("\tleft:%u in_buffer:%u\n", count_items_left, count_items_left_in_buffer);

    if (G4 != 0xff) {
      while (count_items_left) {
        while ((_pflexio->SHIFTSTAT & _fshifter_mask) == 0) {
          // wait for FlexIO shifter data
        }
        *p++ = _pflexio->SHIFTBUF[_fshifter];  // should use DMA...
        count_items_left--;
        if (buffer2 && (--count_items_left_in_buffer == 0)) {
          if (_debug) Serial.println("\tSwitch to second buffer");
          p = (uint32_t*)buffer2;
          count_items_left_in_buffer = (uint32_t)cb2 / 4;
        }
      }
    } else {
      // 4 bit mode:
      bool save_pixels = true;
      uint8_t in_row_count = _width / 4;
      while (count_items_left) {
        // we want to read and save one rows worth of pixels
        // and skipt the second half
        while ((_pflexio->SHIFTSTAT & _fshifter_mask) == 0) {
          // wait for FlexIO shifter data
        }
        uint32_t sbuf = _pflexio->SHIFTBUF[_fshifter];
        if (save_pixels) {
          *p++ = sbuf;  // should use DMA...
          count_items_left--;
          if (buffer2 && (--count_items_left_in_buffer == 0)) {
            p = (uint32_t*)buffer2;
            count_items_left_in_buffer = (uint32_t)cb2 / 4;
          }
        }
        in_row_count--;
        if (in_row_count == 0) {
          save_pixels = !save_pixels;
          in_row_count = _width / 4;
        }
      }
    }
    return;
  }
  //#else
  // read FlexIO by DMA
  dma_flexio.begin();
  const uint32_t length = _width * _height;
  // Will it fit into one DMA object. 
  if ((_width <= 320) && (buffer2 == nullptr)) {
    dma_flexio.source(_pflexio->SHIFTBUF[_fshifter]);
    dma_flexio.destinationBuffer((uint32_t *)buffer, length);
    dma_flexio.transferSize(4);
    dma_flexio.transferCount(length / 4);
    dma_flexio.disableOnCompletion();
    dma_flexio.clearComplete();
    dma_flexio.triggerAtHardwareEvent(_dma_source);
  } else {
    // We need to use dma settings.
    uint8_t dmas_index = 0;
    if (cb1 >= frame_size_bytes) {
      // fit it into 3 _dmasettings
      uint32_t cb_per_setting = ((frame_size_bytes / 3) + 4) & 0xfffffffc; // round up to next multiple of 4.
      _dmasettings[0].TCD->CSR = 0;
      _dmasettings[0].source(_pflexio->SHIFTBUF[_fshifter]);
      _dmasettings[0].destinationBuffer(p, cb_per_setting);
      _dmasettings[0].replaceSettingsOnCompletion(_dmasettings[1]);

      _dmasettings[1].TCD->CSR = 0;
      _dmasettings[1].source(_pflexio->SHIFTBUF[_fshifter]);
      _dmasettings[1].destinationBuffer(&p[cb_per_setting / 4], cb_per_setting);
      _dmasettings[1].replaceSettingsOnCompletion(_dmasettings[2]);

      _dmasettings[2].source(_pflexio->SHIFTBUF[_fshifter]);
      _dmasettings[2].destinationBuffer(&p[cb_per_setting / 2], cb1 - 2 * cb_per_setting);
      _dmasettings[2].replaceSettingsOnCompletion(_dmasettings[0]);
      _dmasettings[2].disableOnCompletion();
      _dmasettings[2].interruptAtCompletion();
      dmas_index = 2;
    } else {
      if ((cb1 + cb2) < frame_size_bytes) return; // not big enough
      // have two buffers - see how we may need to distribute the dma settings.
      uint8_t count_dma_settings = (cb1 / (32767 * 4)) + 1;
      uint32_t cb_per_setting = ((cb1 / count_dma_settings) + 4) & 0xfffffffc; // round up to next multiple of 4.
      
      uint8_t cb_left = cb1;
      for (; dmas_index < count_dma_settings; dmas_index++) {
        _dmasettings[dmas_index].TCD->CSR = 0;
        _dmasettings[dmas_index].source(_pflexio->SHIFTBUF[_fshifter]);
        _dmasettings[dmas_index].destinationBuffer(p, min(cb_per_setting, cb_left));
        _dmasettings[dmas_index].replaceSettingsOnCompletion(_dmasettings[dmas_index + 1]);
        p += (cb_per_setting / 4);
        cb_left -= cb_per_setting;
      }

      cb_left = frame_size_bytes - cb1;
      count_dma_settings = (cb_left / (32767 * 4)) + 1;
      cb_per_setting = ((cb_left / count_dma_settings) + 4) & 0xfffffffc; // round up to next multiple of 4.
      uint32_t *p = (uint32_t *)buffer2;

      for (uint8_t i = 0; i < count_dma_settings; i++, dmas_index++) {
        _dmasettings[dmas_index].TCD->CSR = 0;
        _dmasettings[dmas_index].source(_pflexio->SHIFTBUF[_fshifter]);
        _dmasettings[dmas_index].destinationBuffer(p, min(cb_per_setting, cb_left));
        _dmasettings[dmas_index].replaceSettingsOnCompletion(_dmasettings[dmas_index + 1]);
        p += (cb_per_setting / 4);
        cb_left -= cb_per_setting;
      }
      dmas_index--; // lets point back to the last one
      _dmasettings[dmas_index].replaceSettingsOnCompletion(_dmasettings[0]);
      _dmasettings[dmas_index].disableOnCompletion();
      _dmasettings[dmas_index].interruptAtCompletion();

    }
    dma_flexio.triggerAtHardwareEvent(_dma_source);
    dma_flexio = _dmasettings[0];
    dma_flexio.clearComplete();

    if (_debug) {
      dumpDMA_TCD1(&dma_flexio, "flexio: ");
      for (uint8_t i = 0; i <= dmas_index; i++) {
        Serial.printf("     %u: ", i);
        dumpDMA_TCD1(&_dmasettings[i]);
      }
    }
  }

  dma_flexio.enable();
  _pflexio->SHIFTSDEN = _fshifter_mask;

  timeout = 0;
  while (!dma_flexio.complete()) {
    // wait - we should not need to actually do anything during the DMA transfer
    if (dma_flexio.error()) {
      Serial.println("DMA error");
      break;
    }
    if (timeout > 500) {
      Serial.println("Timeout waiting for DMA");
      if (_pflexio->SHIFTSTAT & _fshifter_mask) Serial.println(" SHIFTSTAT bit was set");
      Serial.printf(" DMA channel #%u\n", dma_flexio.channel);
      Serial.printf(" DMAMUX = %08X\n", *(&DMAMUX_CHCFG0 + dma_flexio.channel));
      Serial.printf(" FLEXIO2_SHIFTSDEN = %02X\n", FLEXIO2_SHIFTSDEN);
      Serial.printf(" TCD CITER = %u\n", dma_flexio.TCD->CITER_ELINKNO);
      Serial.printf(" TCD CSR = %08X\n", dma_flexio.TCD->CSR);
      break;
    }
  }
  if ((uint32_t)buffer >= 0x20200000u) arm_dcache_delete(buffer, min(cb1, frame_size_bytes));
  if (frame_size_bytes > cb1) {
    if ((uint32_t)buffer2 >= 0x20200000u) arm_dcache_delete(buffer2, frame_size_bytes - cb1);
  } 
  //arm_dcache_delete(buffer, length);
  //#endif
}


bool HM0360::startReadFlexIO(bool (*callback)(void *frame_buffer), void *fb1, void *fb2) {
#ifdef FLEXIO_USE_DMA
  if (fb1 == nullptr || fb2 == nullptr) return false;
  _frame_buffer_1 = (uint8_t *)fb1;
  _frame_buffer_2 = (uint8_t *)fb2;
  _callback = callback;
  active_dma_camera = this;
  //Serial.printf("startReadFrameFlexIO called buffers %x %x\n", (uint32_t)fb1, (uint32_t)fb2);

  //flexio_configure(); // one-time hardware setup
  dma_flexio.begin();
  const uint32_t length = _width * _height;
  dma_flexio.source(_pflexio->SHIFTBUF[_fshifter]);
  dma_flexio.destinationBuffer((uint32_t *)fb1, length);
  dma_flexio.transferSize(4);
  dma_flexio.transferCount(length / 4);
  dma_flexio.disableOnCompletion();
  dma_flexio.clearComplete();
  dma_flexio.triggerAtHardwareEvent(_dma_source);
  dma_flexio.interruptAtCompletion();
  dma_flexio.attachInterrupt(dmaInterruptFlexIO);
  FLEXIO2_SHIFTSDEN = _fshifter_mask;
  _dma_frame_count = 0;
  _dma_active = false;

  // wait for VSYNC to be low
  while ((*_vsyncPort & _vsyncMask) != 0)
    ;
  //NVIC_SET_PRIORITY(IRQ_GPIO6789, 102);
  //NVIC_SET_PRIORITY(dma_flexio.channel & 0xf, 102);
  attachInterrupt(VSYNC_PIN, &frameStartInterruptFlexIO, RISING);
  return true;
#else
  return false;
#endif
}


void HM0360::frameStartInterruptFlexIO() {
  active_dma_camera->processFrameStartInterruptFlexIO();
}

void HM0360::processFrameStartInterruptFlexIO() {
  if (!_dma_active) {
    FLEXIO2_SHIFTSTAT = _fshifter_mask;  // clear any prior shift status
    FLEXIO2_SHIFTERR = _fshifter_mask;

    // TODO: could a prior status have a DMA request still be pending?
    void *dest = (_dma_frame_count & 1) ? _frame_buffer_2 : _frame_buffer_1;
    const uint32_t length = _width * _height;
    //dma_flexio.TCD->DADDR = dest;
    dma_flexio.destinationBuffer((uint32_t *)dest, length);
    dma_flexio.transferSize(4);
    dma_flexio.transferCount(length / 4);
    dma_flexio.enable();
    //Serial.println("VSYNC");
    _dma_active = true;
  }
  asm("DSB");
}

void HM0360::dmaInterruptFlexIO() {
  active_dma_camera->processDMAInterruptFlexIO();
}

void HM0360::processDMAInterruptFlexIO() {
  dma_flexio.clearInterrupt();
  if (dma_flexio.error()) return;  // TODO: report or handle error??
  void *dest = (_dma_frame_count & 1) ? _frame_buffer_2 : _frame_buffer_1;
  const uint32_t length = _width * _height;
  _dma_frame_count++;
  arm_dcache_delete(dest, length);
  if (_callback) (*_callback)(dest);  // TODO: use EventResponder
  _dma_active = false;
  asm("DSB");
}


bool HM0360::stopReadFlexIO() {
  detachInterrupt(VSYNC_PIN);
  dma_flexio.disable();
  _frame_buffer_1 = nullptr;
  _frame_buffer_2 = nullptr;
  _callback = nullptr;
  return true;
}


//======================================== DMA JUNK
//================================================================================
// experiment with DMA
//================================================================================
// Define our DMA structure.
DMAChannel HM0360::_dmachannel;
DMASetting HM0360::_dmasettings[4];
uint32_t HM0360::_dmaBuffer1[DMABUFFER_SIZE] __attribute__((used, aligned(32)));
uint32_t HM0360::_dmaBuffer2[DMABUFFER_SIZE] __attribute__((used, aligned(32)));
extern "C" void xbar_connect(unsigned int input, unsigned int output);  // in pwm.c

HM0360 *HM0360::active_dma_camera = nullptr;

void dumpDMA_TCD1(DMABaseClass *dmabc, const char *psz_title=nullptr) {
  if (psz_title)
    Serial.print(psz_title);
  Serial.printf("%x %x: ", (uint32_t)dmabc, (uint32_t)dmabc->TCD);

  Serial.printf(
      "SA:%x SO:%d AT:%x (SM:%x SS:%x DM:%x DS:%x) NB:%x SL:%d DA:%x DO: %d CI:%x DL:%x CS:%x BI:%x\n",
      (uint32_t)dmabc->TCD->SADDR, dmabc->TCD->SOFF, dmabc->TCD->ATTR,
      (dmabc->TCD->ATTR >> 11) & 0x1f, (dmabc->TCD->ATTR >> 8) & 0x7,
      (dmabc->TCD->ATTR >> 3) & 0x1f, (dmabc->TCD->ATTR >> 0) & 0x7,
      dmabc->TCD->NBYTES, dmabc->TCD->SLAST, (uint32_t)dmabc->TCD->DADDR,
      dmabc->TCD->DOFF, dmabc->TCD->CITER, dmabc->TCD->DLASTSGA,
      dmabc->TCD->CSR, dmabc->TCD->BITER);
}

//===================================================================
// Start a DMA operation -
//===================================================================
bool HM0360::startReadFrameDMA(bool (*callback)(void *frame_buffer), uint8_t *fb1, uint8_t *fb2) {
  // First see if we need to allocate frame buffers.
  if (fb1) _frame_buffer_1 = fb1;
  else if (_frame_buffer_1 == nullptr) {
    _frame_buffer_1 = (uint8_t *)malloc(_width * _height);
    if (_frame_buffer_1 == nullptr) return false;
  }
  if (fb2) _frame_buffer_2 = fb2;
  else if (_frame_buffer_2 == nullptr) {
    _frame_buffer_2 = (uint8_t *)malloc(_width * _height);
    if (_frame_buffer_2 == nullptr) return false;  // BUGBUG should we 32 byte align?
  }
  // remember the call back if passed in
  _callback = callback;
  active_dma_camera = this;

  Serial.printf("startReadFrameDMA called buffers %x %x\n", (uint32_t)_frame_buffer_1, (uint32_t)_frame_buffer_2);

  //DebugDigitalToggle(OV7670_DEBUG_PIN_1);
  // lets figure out how many bytes we will tranfer per setting...
  //  _dmasettings[0].begin();
  _frame_row_buffer_pointer = _frame_buffer_pointer = (uint8_t *)_frame_buffer_1;

  // configure DMA channels
  _dmachannel.begin();
  _dmasettings[0].source(GPIO2_PSR);                                   // setup source.
  _dmasettings[0].destinationBuffer(_dmaBuffer1, DMABUFFER_SIZE * 4);  // 32 bits per logical byte
  _dmasettings[0].replaceSettingsOnCompletion(_dmasettings[1]);
  _dmasettings[0].interruptAtCompletion();          // we will need an interrupt to process this.
  _dmasettings[0].TCD->CSR &= ~(DMA_TCD_CSR_DREQ);  // Don't disable on this one
  //DebugDigitalToggle(OV7670_DEBUG_PIN_1);

  _dmasettings[1].source(GPIO2_PSR);                                   // setup source.
  _dmasettings[1].destinationBuffer(_dmaBuffer2, DMABUFFER_SIZE * 4);  // 32 bits per logical byte
  _dmasettings[1].replaceSettingsOnCompletion(_dmasettings[0]);
  _dmasettings[1].interruptAtCompletion();          // we will need an interrupt to process this.
  _dmasettings[1].TCD->CSR &= ~(DMA_TCD_CSR_DREQ);  // Don't disable on this one
  //DebugDigitalToggle(OV7670_DEBUG_PIN_1);

  GPIO2_GDIR = 0;  // set all as input...
  GPIO2_DR = 0;    // see if I can clear it out...

  _dmachannel = _dmasettings[0];  // setup the first on...
  _dmachannel.attachInterrupt(dmaInterrupt);
  _dmachannel.triggerAtHardwareEvent(DMAMUX_SOURCE_XBAR1_0);
  //DebugDigitalToggle(OV7670_DEBUG_PIN_1);

  // Lets try to setup the DMA setup...
  // first see if we can convert the _pclk to be an XBAR Input pin...
  // OV7670_PLK   4
  // OV7670_PLK   8    //8       B1_00   FlexIO2:16  XBAR IO14

  _save_pclkPin_portConfigRegister = *(portConfigRegister(PCLK_PIN));
  *(portConfigRegister(PCLK_PIN)) = 1;  // set to XBAR mode 14

  // route the timer outputs through XBAR to edge trigger DMA request
  CCM_CCGR2 |= CCM_CCGR2_XBAR1(CCM_CCGR_ON);
  xbar_connect(XBARA1_IN_IOMUX_XBAR_INOUT14, XBARA1_OUT_DMA_CH_MUX_REQ30);
  //DebugDigitalToggle(OV7670_DEBUG_PIN_1);

  // Tell XBAR to dDMA on Rising
  XBARA1_CTRL0 = XBARA_CTRL_STS0 | XBARA_CTRL_EDGE0(1) | XBARA_CTRL_DEN0 /* | XBARA_CTRL_IEN0 */;

  IOMUXC_GPR_GPR6 &= ~(IOMUXC_GPR_GPR6_IOMUXC_XBAR_DIR_SEL_14);  // Make sure it is input mode
  IOMUXC_XBAR1_IN14_SELECT_INPUT = 1;                            // Make sure this signal goes to this pin...


#if defined(ARDUINO_TEENSY_MICROMOD)
  // Need to switch the IO pins back to GPI1 from GPIO6
  _save_IOMUXC_GPR_GPR27 = IOMUXC_GPR_GPR27;  // save away the configuration before we change...
  IOMUXC_GPR_GPR27 &= ~(0x0ff0u);

  // lets also un map the _hrefPin to GPIO1
  IOMUXC_GPR_GPR27 &= ~_hrefMask;  //
#else
  // Need to switch the IO pins back to GPI1 from GPIO6
  _save_IOMUXC_GPR_GPR26 = IOMUXC_GPR_GPR26;  // save away the configuration before we change...
  IOMUXC_GPR_GPR26 &= ~(0x0ff0u);

  // lets also un map the _hrefPin to GPIO1
  IOMUXC_GPR_GPR26 &= ~_hrefMask;             //
#endif

  // Need to switch the IO pins back to GPI1 from GPIO6
  //_save_IOMUXC_GPR_GPR27 = IOMUXC_GPR_GPR27;  // save away the configuration before we change...
  //IOMUXC_GPR_GPR27 &= ~(0x0ff0u);

  // lets also un map the _hrefPin to GPIO1
  //IOMUXC_GPR_GPR27 &= ~_hrefMask; //


  //DebugDigitalToggle(OV7670_DEBUG_PIN_1);

  // Falling edge indicates start of frame
  //  while ((*_vsyncPort & _vsyncMask) == 0); // wait for HIGH
  //  while ((*_vsyncPort & _vsyncMask) != 0); // wait for LOW
  //  DebugDigitalWrite(OV7670_DEBUG_PIN_2, HIGH);

  // Debug stuff for now

  // We have the start of a frame, so lets start the dma.
#ifdef DEBUG_CAMERA
  dumpDMA_TCD1(&_dmachannel);
  dumpDMA_TCD1(&_dmasettings[0]);
  dumpDMA_TCD1(&_dmasettings[1]);
  Serial.printf("pclk pin: %d config:%lx control:%lx\n", PCLK_PIN, *(portConfigRegister(PCLK_PIN)), *(portControlRegister(PCLK_PIN)));
  Serial.printf("IOMUXC_GPR_GPR26-29:%lx %lx %lx %lx\n", IOMUXC_GPR_GPR26, IOMUXC_GPR_GPR27, IOMUXC_GPR_GPR28, IOMUXC_GPR_GPR29);
  Serial.printf("GPIO1: %lx %lx, GPIO6: %lx %lx\n", GPIO1_DR, GPIO1_PSR, GPIO6_DR, GPIO6_PSR);
  Serial.printf("XBAR CTRL0:%x CTRL1:%x\n\n", XBARA1_CTRL0, XBARA1_CTRL1);
#endif
  _dma_state = DMASTATE_RUNNING;
  _dma_last_completed_frame = nullptr;
  _dma_frame_count = 0;

  // Now start an interrupt for start of frame.
  attachInterrupt(VSYNC_PIN, &frameStartInterrupt, RISING);

  //DebugDigitalToggle(OV7670_DEBUG_PIN_1);
  return true;
}

//===================================================================
// stopReadFrameDMA - stop doing the reading and then exit.
//===================================================================
bool HM0360::stopReadFrameDMA() {

  // hopefully it start here (fingers crossed)
  // for now will hang here to see if completes...
  //DebugDigitalWrite(OV7670_DEBUG_PIN_2, HIGH);
  elapsedMillis em = 0;
  // tell the background stuff DMA stuff to exit.
  // Note: for now let it end on on, later could disable the DMA directly.
  _dma_state = DMASTATE_STOP_REQUESTED;

  while ((em < 1000) && (_dma_state == DMASTATE_STOP_REQUESTED))
    ;  // wait up to a second...
  if (_dma_state != DMA_STATE_STOPPED) {
    Serial.println("*** stopReadFrameDMA DMA did not exit correctly...");
    Serial.printf("  Bytes Left: %u frame buffer:%x Row:%u Col:%u\n", _bytes_left_dma, (uint32_t)_frame_buffer_pointer, _frame_row_index, _frame_col_index);
  }
  //DebugDigitalWrite(OV7670_DEBUG_PIN_2, LOW);

#ifdef DEBUG_CAMERA
  dumpDMA_TCD1(&_dmachannel);
  dumpDMA_TCD1(&_dmasettings[0]);
  dumpDMA_TCD1(&_dmasettings[1]);
  Serial.println();
#endif
  // Lets restore some hardware pieces back to the way we found them.
#if defined(ARDUINO_TEENSY_MICROMOD)
  IOMUXC_GPR_GPR27 = _save_IOMUXC_GPR_GPR27;  // Restore... away the configuration before we change...
#else
  IOMUXC_GPR_GPR26 = _save_IOMUXC_GPR_GPR26;  // Restore... away the configuration before we change...
#endif
  *(portConfigRegister(PCLK_PIN)) = _save_pclkPin_portConfigRegister;

  return (em < 1000);  // did we stop...
}

//===================================================================
// Our Frame Start interrupt.
//===================================================================
void HM0360::frameStartInterrupt() {
  active_dma_camera->processFrameStartInterrupt();  // lets get back to the main object...
}

void HM0360::processFrameStartInterrupt() {
  _bytes_left_dma = (_width + _frame_ignore_cols) * _height;  // for now assuming color 565 image...
  _dma_index = 0;
  _frame_col_index = 0;  // which column we are in a row
  _frame_row_index = 0;  // which row
  _save_lsb = 0xffff;
  // make sure our DMA is setup properly again.
  _dmasettings[0].transferCount(DMABUFFER_SIZE);
  _dmasettings[0].TCD->CSR &= ~(DMA_TCD_CSR_DREQ);  // Don't disable on this one
  _dmasettings[1].transferCount(DMABUFFER_SIZE);
  _dmasettings[1].TCD->CSR &= ~(DMA_TCD_CSR_DREQ);  // Don't disable on this one
  _dmachannel = _dmasettings[0];                    // setup the first on...
  _dmachannel.enable();

  detachInterrupt(VSYNC_PIN);
}

//===================================================================
// Our DMA interrupt.
//===================================================================
void HM0360::dmaInterrupt() {
  active_dma_camera->processDMAInterrupt();  // lets get back to the main object...
}


// This version assumes only called when HREF...  as set pixclk to only fire
// when set.
void HM0360::processDMAInterrupt() {
  _dmachannel.clearInterrupt();  // tell system we processed it.
  asm("DSB");
  //DebugDigitalWrite(OV7670_DEBUG_PIN_3, HIGH);

  if (_dma_state == DMA_STATE_STOPPED) {
    Serial.println("HM0360::dmaInterrupt called when DMA_STATE_STOPPED");
    return;  //
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
    _bytes_left_dma--;  // for now assuming color 565 image...
    buffer++;
  }

  if ((_frame_row_index == _height) || (_bytes_left_dma == 0)) {  // We finished a frame lets bail
    _dmachannel.disable();                                        // disable the DMA now...
    //DebugDigitalWrite(OV7670_DEBUG_PIN_2, LOW);
#ifdef DEBUG_CAMERA_VERBOSE
    Serial.println("EOF");
#endif
    _frame_row_index = 0;
    _dma_frame_count++;

    bool swap_buffers = true;

    //DebugDigitalToggle(OV7670_DEBUG_PIN_1);
    _dma_last_completed_frame = _frame_row_buffer_pointer;
    if (_callback) swap_buffers = (*_callback)(_dma_last_completed_frame);

    if (swap_buffers) {
      if (_frame_row_buffer_pointer != _frame_buffer_1) _frame_row_buffer_pointer = _frame_buffer_2;
      else _frame_row_buffer_pointer = _frame_buffer_2;
    }

    _frame_buffer_pointer = _frame_row_buffer_pointer;

    //DebugDigitalToggle(OV7670_DEBUG_PIN_1);


    if (_dma_state == DMASTATE_STOP_REQUESTED) {
#ifdef DEBUG_CAMERA
      Serial.println("HM0360::dmaInterrupt - Stop requested");
#endif
      _dma_state = DMA_STATE_STOPPED;
    } else {
      // We need to start up our ISR for the next frame.
#if 1
      // bypass interrupt and just restart DMA...
      _bytes_left_dma = (_width + _frame_ignore_cols) * _height;  // for now assuming color 565 image...
      _dma_index = 0;
      _frame_col_index = 0;  // which column we are in a row
      _frame_row_index = 0;  // which row
      _save_lsb = 0xffff;
      // make sure our DMA is setup properly again.
      _dmasettings[0].transferCount(DMABUFFER_SIZE);
      _dmasettings[0].TCD->CSR &= ~(DMA_TCD_CSR_DREQ);  // Don't disable on this one
      _dmasettings[1].transferCount(DMABUFFER_SIZE);
      _dmasettings[1].TCD->CSR &= ~(DMA_TCD_CSR_DREQ);  // Don't disable on this one
      _dmachannel = _dmasettings[0];                    // setup the first on...
      _dmachannel.enable();

#else
      attachInterrupt(VSYNC_PIN, &frameStartInterrupt, RISING);
#endif
    }
  } else {

    if (_bytes_left_dma == (2 * DMABUFFER_SIZE)) {
      if (_dma_index & 1) _dmasettings[0].disableOnCompletion();
      else _dmasettings[1].disableOnCompletion();
    }
  }
  //DebugDigitalWrite(OV7670_DEBUG_PIN_3, LOW);
}

#endif

typedef struct {
  uint32_t frameTimeMicros;
  uint16_t vsyncStartCycleCount;
  uint16_t vsyncEndCycleCount;
  uint16_t hrefCount;
  uint32_t cycleCount;
  uint16_t pclkCounts[350];  // room to spare.
  uint32_t hrefStartTime[350];
  uint16_t pclkNoHrefCount;
} frameStatics_t;

frameStatics_t fstat1;

void HM0360::captureFrameStatistics() {
  memset((void *)&fstat1, 0, sizeof(fstat1));

  // lets wait for the vsync to go high;
  while ((*_vsyncPort & _vsyncMask) != 0)
    ;  // wait for HIGH
  // now lets wait for it to go low
  while ((*_vsyncPort & _vsyncMask) == 0) fstat1.vsyncStartCycleCount++;  // wait for LOW

  while ((*_hrefPort & _hrefMask) == 0)
    ;  // wait for HIGH
  while ((*_pclkPort & _pclkMask) != 0)
    ;  // wait for LOW

  uint32_t microsStart = micros();
  fstat1.hrefStartTime[0] = microsStart;
  // now loop through until we get the next _vsynd
  // BUGBUG We know that HSYNC and PCLK on same GPIO VSYNC is not...
  uint32_t regs_prev = 0;
  //noInterrupts();
  while ((*_vsyncPort & _vsyncMask) != 0) {

    fstat1.cycleCount++;
    uint32_t regs = (*_hrefPort & (_hrefMask | _pclkMask));
    if (regs != regs_prev) {
      if ((regs & _hrefMask) && ((regs_prev & _hrefMask) == 0)) {
        fstat1.hrefCount++;
        fstat1.hrefStartTime[fstat1.hrefCount] = micros();
      }
      if ((regs & _pclkMask) && ((regs_prev & _pclkMask) == 0)) fstat1.pclkCounts[fstat1.hrefCount]++;
      if ((regs & _pclkMask) && ((regs_prev & _hrefMask) == 0)) fstat1.pclkNoHrefCount++;
      regs_prev = regs;
    }
  }
  while ((*_vsyncPort & _vsyncMask) == 0) fstat1.vsyncEndCycleCount++;  // wait for LOW
  //interrupts();
  fstat1.frameTimeMicros = micros() - microsStart;

  // Maybe return data. print now
  Serial.printf("*** Frame Capture Data: elapsed Micros: %u loops: %u\n", fstat1.frameTimeMicros, fstat1.cycleCount);
  Serial.printf("   VSync Loops Start: %u end: %u\n", fstat1.vsyncStartCycleCount, fstat1.vsyncEndCycleCount);
  Serial.printf("   href count: %u pclk ! href count: %u\n    ", fstat1.hrefCount, fstat1.pclkNoHrefCount);
  for (uint16_t ii = 0; ii < fstat1.hrefCount + 1; ii++) {
    Serial.printf("%3u(%u) ", fstat1.pclkCounts[ii], (ii == 0) ? 0 : fstat1.hrefStartTime[ii] - fstat1.hrefStartTime[ii - 1]);
    if (!(ii & 0x0f)) Serial.print("\n    ");
  }
  Serial.println();
}

typedef struct {
  const __FlashStringHelper *reg_name;
  uint16_t reg;
} HM0360_TO_NAME_t;

static const HM0360_TO_NAME_t hm0360_reg_name_table[] PROGMEM{
  { F("MODEL_ID_H"), 0x0000 },
  { F("MODEL_ID_L"), 0x0001 },
  { F("SILICON_REV"), 0x0002 },
  { F("FRAME_COUNT_H"), 0x0005 },
  { F("FRAME_COUNT_L"), 0x0006 },
  { F("PIXEL_ORDER"), 0x0007 },
  { F("MODE_SELECT"), 0x0100 },
  { F("IMG_ORIENTATION"), 0x0101 },
  { F("EMBEDDED_LINE_EN"), 0x0102 },
  { F("SW_RESET"), 0x0103 },
  { F("COMMAND_UPDATE"), 0x0104 },
  { F("INTEGRATION_H"), 0x0202 },
  { F("INTEGRATION_L"), 0x0203 },
  { F("ANALOG_GAIN"), 0x0205 },
  { F("DIGITAL_GAIN_H"), 0x020E },
  { F("DIGITAL_GAIN_L"), 0x020F },
  { F("PLL1_CONFIG"), 0x0300 },
  { F("PLL2_CONFIG"), 0x0301 },
  { F("PLL3_CONFIG"), 0x0302 },
  { F("FRAME_LEN_LINES_H"), 0x0340 },
  { F("FRAME_LEN_LINES_L"), 0x0341 },
  { F("LINE_LEN_PCK_H"), 0x0342 },
  { F("LINE_LEN_PCK_L"), 0x0343 },
  { F("MONO_MODE"), 0x0370 },
  { F("MONO_MODE_ISP"), 0x0371 },
  { F("MONO_MODE_SEL"), 0x0372 },
  { F("H_SUBSAMPLE"), 0x0380 },
  { F("V_SUBSAMPLE"), 0x0381 },
  { F("BINNING_MODE"), 0x0382 },
  { F("TEST_PATTERN_MODE"), 0x0601 },
  { F("BLC_TGT"), 0x1004 },
  { F("BLC2_TGT"), 0x1009 },
  { F("MONO_CTRL"), 0x100A },
  { F("OPFM_CTRL"), 0x1014 },
  { F("CMPRS_CTRL"), 0x102F },
  { F("CMPRS_01"), 0x1030 },
  { F("CMPRS_02"), 0x1031 },
  { F("CMPRS_03"), 0x1032 },
  { F("CMPRS_04"), 0x1033 },
  { F("CMPRS_05"), 0x1034 },
  { F("CMPRS_06"), 0x1035 },
  { F("CMPRS_07"), 0x1036 },
  { F("CMPRS_08"), 0x1037 },
  { F("CMPRS_09"), 0x1038 },
  { F("CMPRS_10"), 0x1039 },
  { F("CMPRS_11"), 0x103A },
  { F("CMPRS_12"), 0x103B },
  { F("CMPRS_13"), 0x103C },
  { F("CMPRS_14"), 0x103D },
  { F("CMPRS_15"), 0x103E },
  { F("CMPRS_16"), 0x103F },
  { F("AE_CTRL"), 0x2000 },
  { F("AE_CTRL1"), 0x2001 },
  { F("CNT_ORGH_H"), 0x2002 },
  { F("CNT_ORGH_L"), 0x2003 },
  { F("CNT_ORGV_H"), 0x2004 },
  { F("CNT_ORGV_L"), 0x2005 },
  { F("CNT_STH_H"), 0x2006 },
  { F("CNT_STH_L"), 0x2007 },
  { F("CNT_STV_H"), 0x2008 },
  { F("CNT_STV_L"), 0x2009 },
  { F("CTRL_PG_SKIPCNT"), 0x200A },
  { F("BV_WIN_WEIGHT_EN"), 0x200D },
  { F("MAX_INTG_H"), 0x2029 },
  { F("MAX_INTG_L"), 0x202A },
  { F("MAX_AGAIN"), 0x202B },
  { F("MAX_DGAIN_H"), 0x202C },
  { F("MAX_DGAIN_L"), 0x202D },
  { F("MIN_INTG"), 0x202E },
  { F("MIN_AGAIN"), 0x202F },
  { F("MIN_DGAIN"), 0x2030 },
  { F("T_DAMPING"), 0x2031 },
  { F("N_DAMPING"), 0x2032 },
  { F("ALC_TH"), 0x2033 },
  { F("AE_TARGET_MEAN"), 0x2034 },
  { F("AE_MIN_MEAN"), 0x2035 },
  { F("AE_TARGET_ZONE"), 0x2036 },
  { F("CONVERGE_IN_TH"), 0x2037 },
  { F("CONVERGE_OUT_TH"), 0x2038 },
  { F("FS_CTRL"), 0x203B },
  { F("FS_60HZ_H"), 0x203C },
  { F("FS_60HZ_L"), 0x203D },
  { F("FS_50HZ_H"), 0x203E },
  { F("FS_50HZ_L"), 0x203F },
  { F("FRAME_CNT_TH"), 0x205B },
  { F("AE_MEAN"), 0x205D },
  { F("AE_CONVERGE"), 0x2060 },
  { F("AE_BLI_TGT"), 0x2070 },
  { F("PULSE_MODE"), 0x2061 },
  { F("PULSE_TH_H"), 0x2062 },
  { F("PULSE_TH_L"), 0x2063 },
  { F("INT_INDIC"), 0x2064 },
  { F("INT_CLEAR"), 0x2065 },
  { F("MD_CTRL"), 0x2080 },
  { F("ROI_START_END_V"), 0x2081 },
  { F("ROI_START_END_H"), 0x2082 },
  { F("MD_TH_MIN"), 0x2083 },
  { F("MD_TH_STR_L"), 0x2084 },
  { F("MD_TH_STR_H"), 0x2085 },
  { F("MD_LIGHT_COEF"), 0x2099 },
  { F("MD_BLOCK_NUM_TH"), 0x209B },
  { F("MD_LATENCY"), 0x209C },
  { F("MD_LATENCY_TH"), 0x209D },
  { F("MD_CTRL1"), 0x209E },
  { F("PMU_CFG_3"), 0x3024 },
  { F("PMU_CFG_4"), 0x3025 },
  { F("WIN_MODE"), 0x3030 },
  { F("VSYNC_FRONT"), 0x3094 },
  { F("VSYNC_END"), 0x3095 },
  { F("HSYNC_FRONT_H"), 0x3096 },
  { F("HSYNC_FRONT_L"), 0x3097 },
  { F("HSYNC_END_H"), 0x3098 },
  { F("HSYNC_END_L"), 0x3099 },
  { F("READ_PU_FRONT"), 0x309A },
  { F("READ_PU_End"), 0x309B },
  { F("EARLY_INT_EN"), 0x309C },
  { F("PCLKO_GATED_EN"), 0x309E },
  { F("PCLKO_FRAME_FRONT"), 0x309F },
  { F("PCLKO_FRAME_END"), 0x30A0 },
  { F("PCLKO_LINE_FRONT_H"), 0x30A1 },
  { F("PCLKO_LINE_FRONT_L"), 0x30A2 },
  { F("PCLKO_LINE_END_H"), 0x30A3 },
  { F("PCLKO_LINE_END_L"), 0x30A4 },
  { F("OUTPUT_EN"), 0x30A5 },
  { F("FRAME_OUTPUT_EN"), 0x30A8 },
  { F("MULTI_CAMERA_CONFIG"), 0x30A9 },
  { F("MULTI_CAMERA_TUNE_H"), 0x30AA },
  { F("MULTI_CAMERA_TUNE_L"), 0x30AB },
  { F("ANA_REGISTER_03"), 0x310E },
  { F("ANA_REGISTER_04"), 0x310F },
  { F("ANA_REGISTER_05"), 0x3110 },
  { F("ANA_REGISTER_06"), 0x3111 },
  { F("PAD_REGISTER_07"), 0x3112 },
  { F("PLL_POST_DIV_D"), 0x3128 }
};

void HM0360::showRegisters(void) {
  Serial.println("\n*** Camera Registers ***");
  for (uint16_t ii = 0; ii < (sizeof(hm0360_reg_name_table) / sizeof(hm0360_reg_name_table[0])); ii++) {
    uint8_t reg_value = cameraReadRegister(hm0360_reg_name_table[ii].reg);
    Serial.printf("%s(%x): %u(%x)\n", hm0360_reg_name_table[ii].reg_name, hm0360_reg_name_table[ii].reg, reg_value, reg_value);
  }
}
