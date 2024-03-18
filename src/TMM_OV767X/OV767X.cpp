// SPDX-License-Identifier: GPL-2.0-only
/*
 * This file is part of the Arduino_OX767X library.
 * Copyright (c) 2020 Arduino SA. All rights reserved.
 */
#define DEBUG_CAMERA
#define  DEBUG_FLEXIO
#define USE_VSYNC_PIN_INT

#include <Arduino.h>
#include <Wire.h>

#include "OV767X.h"
#include "arm_math.h"

// if not defined in the variant
#ifndef digitalPinToBitMask
#define digitalPinToBitMask(P) (1 << (digitalPinToPinName(P) % 64))
#endif

#ifndef portInputRegister
#define portInputRegister(P) ((P == 0) ? &NRF_P0->IN : &NRF_P1->IN)
#endif

#define CNT_SHIFTERS 1

extern "C" {
  // defined in utility/ov7670.c:
  struct ov7670_fract {
    uint32_t numerator;
    uint32_t denominator;
  };

  void* ov7670_alloc();
  void ov7670_free(void*);

  int ov7670_reset(void*, uint32_t val);
  int ov7670_detect(void*);
  void ov7670_configure(void*, int devtype, int format, int wsize, int clock_speed, int pll_bypass, int pclk_hb_disable);
  int ov7670_s_power(void*, int on);
  int ov7675_set_framerate(void*, struct ov7670_fract *tpf);

  int ov7670_s_sat_hue(void*, int sat, int hue);
  int ov7670_s_brightness(void*, int value);
  int ov7670_s_contrast(void*, int value);
  int ov7670_s_hflip(void*, int value);
  int ov7670_s_vflip(void*, int value);
  int ov7670_s_gain(void*, int value);
  int ov7670_s_autogain(void*, int value);
  int ov7670_s_exp(void*, int value);
  int ov7670_s_autoexp(void*, int value);
  int ov7670_s_test_pattern(void*, int value);
  void ov7670_printRegs();

};

const int OV760_D[8] = {
  OV7670_D0, OV7670_D1, OV7670_D2, OV7670_D3, OV7670_D4, OV7670_D5, OV7670_D6, OV7670_D7
};

OV767X::OV767X() :
  _ov7670(NULL),
  _saturation(128),
  _hue(0),
  _frame_buffer_pointer(NULL)
{
  //setPins(OV7670_VSYNC, OV7670_HREF, OV7670_PLK, OV7670_XCLK, OV7670_RST, OV760_D);
  setPins(OV7670_XCLK, OV7670_PLK, OV7670_VSYNC, OV7670_HREF, OV7670_RST,
                     OV7670_D0, OV7670_D1, OV7670_D2, OV7670_D3, OV7670_D4, OV7670_D5, OV7670_D6, OV7670_D7, Wire);

}
/*
OV767X::~OV767X()
{
  if (_ov7670) {
    ov7670_free(_ov7670);
  }
}
*/
//int OV767X::begin(int resolution, int format, int fps,  int camera_name, bool use_gpio)
bool OV767X::begin_omnivision(framesize_t resolution, pixformat_t format, int fps, int camera_name, bool use_gpio)
{

  int _framesize = 0;
  int _format = 0;
  
  _use_gpio = use_gpio;
  // BUGBUG::: see where frame is
  pinMode(49, OUTPUT);
    
  Serial.println("OV767X::begin");

  switch (resolution) {
  case FRAMESIZE_VGA:
    _width = 640;
    _height = 480;
    _framesize = 0;
    break;

  case FRAMESIZE_CIF:
    _width = 352;
    _height = 240;
    _framesize = 1;
    break;

  case FRAMESIZE_QVGA:
    _width = 320;
    _height = 240;
    _framesize = 2;
    break;

  case FRAMESIZE_QCIF:
    _width = 176;
    _height = 144;
    _framesize = 3;
    break;

  case FRAMESIZE_QQVGA:
    _width = 160;
    _height = 120;
    _framesize = 4;
    break;

  default:
    return 0;
  }

  _grayscale = false;
  switch (format) {
  case YUV422:
    _bytesPerPixel = 2;
    _format = 0;
    break;
  case RGB444:
    _bytesPerPixel = 2;
    _format = 1;
    break;
  case RGB565:
    _bytesPerPixel = 2;
    _format = 2;
    break;

  case GRAYSCALE:
    format = YUV422;    // We use YUV422 but discard U and V bytes
    _bytesPerPixel = 2; // 2 input bytes per pixel of which 1 is discarded
    _grayscale = true;
    _format = 4;
    break;

  default:
    return 0;
  }

  _ov7670 = ov7670_alloc();
  if (!_ov7670) {
    end();

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
  
  if(_rst != 0xFF){
    pinMode(_rst, OUTPUT);
    digitalWriteFast(_rst, LOW);      /* Reset */
    for(volatile uint32_t i=0; i<100000; i++)
    {}
    digitalWriteFast(_rst, HIGH);     /* Normal mode. */
    for(volatile uint32_t i=0; i<100000; i++)
    {}
  }
  
  Wire.begin();

  delay(1000);

  if (ov7670_detect(_ov7670)) {
    end();
    Serial.println("Camera detect failed");

    return 0;
  }
  
  if(camera_name == OV7670) {
      _xclk_freq = 14;  //was 16Mhz
  } else {
      if(fps <= 10){
       _xclk_freq = 14;
      } else {
      _xclk_freq = 16;
      }
  }
  
  Serial.printf("Calling ov7670_configure\n");
  Serial.printf("Cam Name: %d, Format: %d, Resolution: %d, Clock: %d\n", camera_name, _format, _framesize, _xclk_freq);
  Serial.printf("Frame rate: %d\n", fps);
  ov7670_configure(_ov7670, camera_name /*OV7670 = 0, OV7675 = 1*/, _format, _framesize, _xclk_freq /* MHz */,
                   0 /*pll bypass*/, 1 /* pclk_hb_disable */);

  if (ov7670_s_power(_ov7670, 1)) {
    end();
    Serial.println("Camera ov7670_s_power failed");

    return 0;
  }

  struct ov7670_fract tpf;

  tpf.numerator = 1;
  tpf.denominator = fps;
  
//flexIO/DMA
    if(!_use_gpio) {
        flexio_configure();
        setVSyncISRPriority(102);
        setDMACompleteISRPriority(192);
    } else {
        setVSyncISRPriority(102);
        setDMACompleteISRPriority(192);
    }

  ov7675_set_framerate(_ov7670, &tpf);

  return 1;
}

uint16_t OV767X::getModelid()
{
    uint8_t Data;
    uint16_t MID = 0x0000;

    Data = cameraReadRegister(0x0A);
       MID = (Data << 8);

    Data = cameraReadRegister(0x0B);
        MID |= Data;

    return MID;
}

void OV767X::end()
{
  endXClk();

  pinMode(_xclkPin, INPUT);

  Wire.end();

  if (_ov7670) {
    ov7670_free(_ov7670);
  }
}

int16_t OV767X::width()
{
  return _width;
}

int16_t OV767X::height()
{
  return _height;
}

int OV767X::bitsPerPixel() const
{
  if (_grayscale) {
    return 8;
  } else {
    return _bytesPerPixel * 8;
  }
}

int OV767X::bytesPerPixel() const
{
  if (_grayscale) {
    return 1;
  } else {
    return _bytesPerPixel;
  }
}


void OV767X::testPattern(int pattern)
{
  ov7670_s_test_pattern(_ov7670, pattern);
}

void OV767X::noTestPattern()
{
  ov7670_s_test_pattern(_ov7670, 0);
}

void OV767X::setSaturation(int saturation)
{
  _saturation = saturation;

  ov7670_s_sat_hue(_ov7670, _saturation, _hue);
}

void OV767X::setHue(int hue)
{
  _hue = hue;

  ov7670_s_sat_hue(_ov7670, _saturation, _hue);
}

int OV767X::setBrightness(int brightness)
{
  ov7670_s_brightness(_ov7670, brightness);
  return 1;
}

void OV767X::setContrast(int contrast)
{
  ov7670_s_contrast(_ov7670, contrast);
}

int OV767X::setHmirror(int enable)
{
  ov7670_s_hflip(_ov7670, enable);
  delay(10);
  return 1;
}

int OV767X::setVflip(int enable)
{
  ov7670_s_vflip(_ov7670, enable);
  delay(10);
  return 1;
}

void OV767X::setGain(int gain)
{
  ov7670_s_gain(_ov7670, gain);
}

void OV767X::autoGain(int enable, float gain_db, float gain_db_ceiling)
{
  ov7670_s_autogain(_ov7670, enable);
}

void OV767X::setExposure(int exposure)
{
  ov7670_s_exp(_ov7670, exposure);
}

void OV767X::autoExposure(int enable)
{
  if(enable == 1) {
      enable = 0;
  } else if(enable == 0) {
      enable = 1;
  } else {
      enable = 1;
  }
  ov7670_s_autoexp(_ov7670, enable) /* V4L2_EXPOSURE_AUTO */;
  delay(10);
}

//void OV767X::setPins(int vsync, int href, int pclk, int xclk, int rst, const int dpins[8])
void OV767X::setPins(uint8_t mclk_pin, uint8_t pclk_pin, uint8_t vsync_pin, uint8_t hsync_pin, uint8_t en_pin,
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

void OV767X::beginXClk()
{
  // Generates 8 MHz signal using PWM... Will speed up.
#if defined(__IMXRT1062__)  // Teensy 4.x
  analogWriteFrequency(_xclkPin, _xclk_freq * 1000000);
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

void OV767X::endXClk()
{
#if defined(__IMXRT1062__)  // Teensy 4.x
  analogWrite(OV7670_XCLK, 0);
#else
  NRF_I2S->TASKS_STOP = 1;
#endif
}


#define FLEXIO_USE_DMA
void OV767X::readFrame(void* buffer, bool fUseDMA){
    if(!_use_gpio) {
        readFrameFlexIO(buffer, (size_t)-1, nullptr, 0, fUseDMA);
    } else {
        readFrameGPIO(buffer, (size_t)-1, nullptr, 0);
    }
}

void OV767X::readFrameSplitBuffer(void *buffer1, size_t cb1, void *buffer2, size_t cb2, bool fUseDMA) {
    if(!_use_gpio) {
        readFrameFlexIO(buffer1, cb1, buffer2, cb2, fUseDMA);
    } else {
        readFrameGPIO(buffer1, cb1, buffer2, cb2);
    }

}


bool OV767X::readContinuous(bool(*callback)(void *frame_buffer), void *fb1, void *fb2) {

	return startReadFlexIO(callback, fb1, fb2);

}

void OV767X::stopReadContinuous() {
	
  stopReadFlexIO();

}

void OV767X::readFrameGPIO(void *buffer, size_t cb1, void *buffer2, size_t cb2)
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



bool OV767X::flexio_configure()
{

    // Going to try this using my FlexIO library.

    // BUGBUG - starting off not going to worry about maybe first pin I choos is on multipl Flex IO controllers (yet)
    uint8_t tpclk_pin; 
    _pflex = FlexIOHandler::mapIOPinToFlexIOHandler(_pclkPin, tpclk_pin);
    if (!_pflex) {
        Serial.printf("OV767X PCLK(%u) is not a valid Flex IO pin\n", _pclkPin);
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
        Serial.printf("OV767X Some pins did not map to valid Flex IO pin\n");
        Serial.printf("    HSYNC(%u %u) G0(%u %u) G1(%u %u) G2(%u %u) G3(%u %u)", 
            _hrefPin, thsync_pin, _dPins[0], tg0, _dPins[1], tg1, _dPins[2], tg2, _dPins[3], tg3 );
        return false;
    } 
    // Verify that the G numbers are consecutive... Should use arrays!
    if ((tg1 != (tg0+1)) || (tg2 != (tg0+2)) || (tg3 != (tg0+3))) {
        Serial.printf("OV767X Flex IO pins G0-G3 are not consective\n");
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
            Serial.printf("OV767X Flex IO pins G4-G7 are not consective with G0-3\n");
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
      Serial.printf("OV767X Flex IO: Could not claim Shifter 3 or 7\n");
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
          Serial.printf("OV767X Flex IO: Could not claim Shifter %u\n", _fshifter);
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
        Serial.printf("OV767X Flex IO: Could not claim Shifter %u\n", _fshifter);
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
        Serial.printf("OV767X Flex IO: failed to request timer\n");
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


void dumpDMA_TCD(DMABaseClass *dmabc, const char *psz_title) {
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

void OV767X::readFrameFlexIO(void *buffer, size_t cb1, void* buffer2, size_t cb2, bool use_dma)
{
    if (_debug)Serial.printf("$$OV767X::readFrameFlexIO(%p, %u, %p, %u, %u)\n", buffer, cb1, buffer2, cb2, use_dma);
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
      if (_debug)Serial.println("\tNot DMA");
      #ifdef OV7670_USE_DEBUG_PINS
      digitalWriteFast(2, HIGH);
      #endif
      // read FlexIO by polling
      //uint32_t *p_end = (uint32_t *)buffer + (_width*_height/4)*_bytesPerPixel;
      uint32_t count_items_left = (_width*_height/4)*_bytesPerPixel;
      uint32_t count_items_left_in_buffer = (uint32_t)cb1 / 4;

      while (count_items_left) {
          while ((_pflexio->SHIFTSTAT & _fshifter_mask) == 0) {
              // wait for FlexIO shifter data
          }
          // Lets try to load in multiple shifters
          for (uint8_t i = 0; i < CNT_SHIFTERS; i++) {
            *p++ = _pflexio->SHIFTBUF[_fshifter+i]; // should use DMA...
            count_items_left--;
            if (buffer2 && (--count_items_left_in_buffer == 0)) {
              p = (uint32_t*)buffer2;
              count_items_left_in_buffer = (uint32_t)cb2 / 4;
            }
          }
      }
      #ifdef OV7670_USE_DEBUG_PINS
      digitalWriteFast(2, LOW);
      #endif
      return;
    }

    //----------------------------------------------------------------------
    // Use DMA FlexIO version
    //----------------------------------------------------------------------
//    digitalWriteFast(2, HIGH);

    // Lets try like other implementation.
    const uint32_t frame_size_bytes = _width*_height*_bytesPerPixel;
    //uint32_t length_uint32 = frame_size_bytes / 4;

    _dmachannel.begin();
    _dmachannel.triggerAtHardwareEvent(_dma_source);
    active_dma_camera = this;
    _dmachannel.attachInterrupt(dmaInterruptFlexIO);
    /* Configure DMA MUX Source */
    //DMAMUX->CHCFG[FLEXIO_CAMERA_DMA_CHN] = DMAMUX->CHCFG[FLEXIO_CAMERA_DMA_CHN] &
    //                                        (~DMAMUX_CHCFG_SOURCE_MASK) | 
    //                                        DMAMUX_CHCFG_SOURCE(FLEXIO_CAMERA_DMA_MUX_SRC);
    /* Enable DMA channel. */
    // if only one buffer split over the one buffer assuming big enough.
    // Total length of bytes transfered
    // do it over 2 
    // first pass split into two
    uint32_t cb_left = 0;
    if (_debug)Serial.printf("\tframe Size:%u cb1: %u\n", frame_size_bytes, cb1);

    uint8_t dmas_index = 0;
    if ((buffer2 == nullptr) ||  frame_size_bytes < cb1) {
      // quick and dirty test that > buffer can maybe fit into memory...
      if (cb1 == (uint32_t)-1) {
        uint32_t buffer_end = (uint32_t)p + frame_size_bytes; 
        if (_debug) Serial.printf("$$ %p %x\n", p, buffer_end);
        if (((uint32_t)p >= 0x20000000) && ((uint32_t)p < 0x2007FFFF) && (buffer_end >= 0x2007FFFF)) return; // DTCM check
        if (((uint32_t)p >= 0x20200000) && ((uint32_t)p < 0x2027FFFF) && (buffer_end >= 0x2027FFFF)) return; // RAM check
      }

      uint8_t count_dma_settings = (frame_size_bytes / (32767 * 4)) + 1;
      uint32_t cb_per_setting = ((frame_size_bytes / count_dma_settings) + 3) & 0xfffffffc; // round up to next multiple of 4.
      if (_debug) Serial.printf("frame size: %u, cnt dma: %u CB per: %u\n", frame_size_bytes, count_dma_settings, cb_per_setting);

      uint32_t cb_left = frame_size_bytes;
      for (; dmas_index < count_dma_settings; dmas_index++) {
        _dmasettings[dmas_index].TCD->CSR = 0;
        _dmasettings[dmas_index].source(_pflexio->SHIFTBUF[_fshifter]);
        _dmasettings[dmas_index].destinationBuffer(p, cb_per_setting);
        _dmasettings[dmas_index].replaceSettingsOnCompletion(_dmasettings[dmas_index + 1]);
        p += (cb_per_setting / 4);
        cb_left -= cb_per_setting;
        if (cb_left < cb_per_setting) cb_per_setting = cb_left;
      }
      dmas_index--; // lets point back to the last one
      _dmasettings[dmas_index].replaceSettingsOnCompletion(_dmasettings[0]);
      _dmasettings[dmas_index].disableOnCompletion();
      _dmasettings[dmas_index].interruptAtCompletion();
    } else {
      // We will do like above with both buffers, maybe later try to merge the two sections.
      uint32_t cb_left = min(frame_size_bytes, cb1);
      uint8_t count_dma_settings = (cb_left / (32767 * 4)) + 1;
      uint32_t cb_per_setting = ((cb_left / count_dma_settings) + 3) & 0xfffffffc; // round up to next multiple of 4.
      if (_debug) Serial.printf("frame size: %u, cb1:%u cnt dma: %u CB per: %u\n", frame_size_bytes, cb1, count_dma_settings, cb_per_setting);

      for (; dmas_index < count_dma_settings; dmas_index++) {
        _dmasettings[dmas_index].TCD->CSR = 0;
        _dmasettings[dmas_index].source(_pflexio->SHIFTBUF[_fshifter]);
        _dmasettings[dmas_index].destinationBuffer(p, cb_per_setting);
        _dmasettings[dmas_index].replaceSettingsOnCompletion(_dmasettings[dmas_index + 1]);
        p += (cb_per_setting / 4);
        cb_left -= cb_per_setting;
        if (cb_left < cb_per_setting) cb_per_setting = cb_left;
      }
      if (frame_size_bytes > cb1) {
        cb_left = frame_size_bytes - cb1;
        count_dma_settings = (cb_left / (32767 * 4)) + 1;
        cb_per_setting = ((cb_left / count_dma_settings) + 3) & 0xfffffffc; // round up to next multiple of 4.
        if (_debug) Serial.printf("frame size left: %u, cb2:%u cnt dma: %u CB per: %u\n", cb_left, cb2, count_dma_settings, cb_per_setting);
        
        p = (uint32_t *)buffer2;

        for (uint8_t i=0; i < count_dma_settings; i++, dmas_index++) {
          _dmasettings[dmas_index].TCD->CSR = 0;
          _dmasettings[dmas_index].source(_pflexio->SHIFTBUF[_fshifter]);
          _dmasettings[dmas_index].destinationBuffer(p, cb_per_setting);
          _dmasettings[dmas_index].replaceSettingsOnCompletion(_dmasettings[dmas_index + 1]);
          p += (cb_per_setting / 4);
          cb_left -= cb_per_setting;
          if (cb_left < cb_per_setting) cb_per_setting = cb_left;
        }
      }  
      dmas_index--; // lets point back to the last one
      _dmasettings[dmas_index].replaceSettingsOnCompletion(_dmasettings[0]);
      _dmasettings[dmas_index].disableOnCompletion();
      _dmasettings[dmas_index].interruptAtCompletion();
#if 0
      // Note: in this part we are going to use 3 
      // use the first two for the first buffer
      uint32_t cb_per_setting = ((cb1 / 3) + 4) & 0xfffffffc; // round up to next multiple of 4.
      _dmasettings[0].source(_pflexio->SHIFTBUF[_fshifter]);
      _dmasettings[0].destinationBuffer(p, cb_per_setting);
      _dmasettings[0].replaceSettingsOnCompletion(_dmasettings[1]);

      _dmasettings[1].source(_pflexio->SHIFTBUF[_fshifter]);
      _dmasettings[1].destinationBuffer(&p[cb_per_setting / 4], cb_per_setting);
      _dmasettings[1].replaceSettingsOnCompletion(_dmasettings[2]);

      _dmasettings[2].source(_pflexio->SHIFTBUF[_fshifter]);
      _dmasettings[2].destinationBuffer(&p[cb_per_setting / 2], cb1 - 2 * cb_per_setting);
      _dmasettings[2].replaceSettingsOnCompletion(_dmasettings[3]);

      p = (uint32_t *)buffer2;
      cb_left = frame_size_bytes - cb1;
      cb_per_setting = ((cb_left / 3) + 4) & 0xfffffffc; // round up to next multiple of 4.

      _dmasettings[3].source(_pflexio->SHIFTBUF[_fshifter]);
      _dmasettings[3].destinationBuffer(p, cb_per_setting);
      _dmasettings[3].replaceSettingsOnCompletion(_dmasettings[4]);

      _dmasettings[4].source(_pflexio->SHIFTBUF[_fshifter]);
      _dmasettings[4].destinationBuffer(&p[cb_per_setting / 4],cb_per_setting);
      _dmasettings[4].replaceSettingsOnCompletion(_dmasettings[5]);

      _dmasettings[5].source(_pflexio->SHIFTBUF[_fshifter]);
      _dmasettings[5].destinationBuffer(&p[cb_per_setting / 2], cb_left - 2 * cb_per_setting);
      _dmasettings[5].replaceSettingsOnCompletion(_dmasettings[0]);

      _dmasettings[1].TCD->CSR &= ~(DMA_TCD_CSR_DREQ | DMA_TCD_CSR_INTMAJOR); // Don't disable or interrupt on this one
      _dmasettings[5].disableOnCompletion();
      _dmasettings[5].interruptAtCompletion();
      dmas_index = 5;
#endif
    }
    _dmachannel = _dmasettings[0];

    _dmachannel.clearComplete();
#ifdef DEBUG_FLEXIO
    if (_debug) {
      dumpDMA_TCD(&_dmachannel," CH: ");
      for (uint8_t i = 0; i <= dmas_index; i++) {
        Serial.printf(" %u: ", i);
        dumpDMA_TCD(&_dmasettings[i], nullptr);
      }
    }
#endif


    _dma_state = DMA_STATE_ONE_FRAME;
    _pflexio->SHIFTSDEN = _fshifter_mask;
    _dmachannel.enable();
    
#ifdef DEBUG_FLEXIO
    if (_debug) Serial.printf("Flexio DMA: length: %d\n", frame_size_bytes);
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
    #ifdef OV7670_USE_DEBUG_PINS
        digitalWriteFast(2, LOW);
    #endif
    //arm_dcache_delete(buffer, frame_size_bytes);
    if ((uint32_t)buffer >= 0x20200000u) arm_dcache_delete(buffer, min(cb1, frame_size_bytes));
    if (frame_size_bytes > cb1) {
      if ((uint32_t)buffer2 >= 0x20200000u) arm_dcache_delete(buffer2, frame_size_bytes - cb1);
    } 

#ifdef DEBUG_FLEXIO
    if (_debug) dumpDMA_TCD(&_dmachannel,"CM: ");
#endif
//    dumpDMA_TCD(&_dmasettings[0], " 0: ");
//    dumpDMA_TCD(&_dmasettings[1], " 1: ");
}



bool OV767X::startReadFlexIO(bool(*callback)(void *frame_buffer), void *fb1, void *fb2)
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

    dumpDMA_TCD(&_dmachannel," CH: ");
    dumpDMA_TCD(&_dmasettings[0], " 0: ");
    dumpDMA_TCD(&_dmasettings[1], " 1: ");
    dumpDMA_TCD(&_dmasettings[2], " 2: ");
    dumpDMA_TCD(&_dmasettings[3], " 3: ");
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
void OV767X::frameStartInterruptFlexIO()
{
	active_dma_camera->processFrameStartInterruptFlexIO();
}

void OV767X::processFrameStartInterruptFlexIO()
{
  #ifdef OV7670_USE_DEBUG_PINS
  digitalWriteFast(5, HIGH);
  #endif
  //Serial.println("VSYNC");
  // See if we read the state of it a few times if the pin stays high...
  if (digitalReadFast(_vsyncPin) && digitalReadFast(_vsyncPin) && digitalReadFast(_vsyncPin) 
          && digitalReadFast(_vsyncPin) )  {
    // stop this interrupt.
    #ifdef OV7670_USE_DEBUG_PINS
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
  #ifdef OV7670_USE_DEBUG_PINS
  digitalWriteFast(5, LOW);
  #endif
}

#endif

void OV767X::dmaInterruptFlexIO()
{
	active_dma_camera->processDMAInterruptFlexIO();
}

void OV767X::processDMAInterruptFlexIO()
{

  _dmachannel.clearInterrupt();
  #ifdef OV7670_USE_DEBUG_PINS
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
    dumpDMA_TCD(&_dmachannel," CH: ");

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


bool OV767X::stopReadFlexIO()
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
DMAChannel OV767X::_dmachannel;
DMASetting OV767X::_dmasettings[6];
uint32_t OV767X::_dmaBuffer1[DMABUFFER_SIZE] __attribute__ ((used, aligned(32)));
uint32_t OV767X::_dmaBuffer2[DMABUFFER_SIZE] __attribute__ ((used, aligned(32)));
extern "C" void xbar_connect(unsigned int input, unsigned int output); // in pwm.c

OV767X *OV767X::active_dma_camera = nullptr;


//===================================================================
// Start a DMA operation -
//===================================================================
bool OV767X::startReadFrameDMA(bool(*callback)(void *frame_buffer), uint8_t *fb1, uint8_t *fb2)
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

  //DebugDigitalToggle(OV7670_DEBUG_PIN_1);
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
  //DebugDigitalToggle(OV7670_DEBUG_PIN_1);

  _dmasettings[1].source(GPIO2_PSR); // setup source.
  _dmasettings[1].destinationBuffer(_dmaBuffer2, DMABUFFER_SIZE * 4);  // 32 bits per logical byte
  _dmasettings[1].replaceSettingsOnCompletion(_dmasettings[0]);
  _dmasettings[1].interruptAtCompletion();  // we will need an interrupt to process this.
  _dmasettings[1].TCD->CSR &= ~(DMA_TCD_CSR_DREQ); // Don't disable on this one
  //DebugDigitalToggle(OV7670_DEBUG_PIN_1);

  GPIO2_GDIR = 0; // set all as input...
  GPIO2_DR = 0; // see if I can clear it out...

  _dmachannel = _dmasettings[0];  // setup the first on...
  _dmachannel.attachInterrupt(dmaInterrupt);
  _dmachannel.triggerAtHardwareEvent(DMAMUX_SOURCE_XBAR1_0);
  //DebugDigitalToggle(OV7670_DEBUG_PIN_1);

  // Lets try to setup the DMA setup...
  // first see if we can convert the _pclk to be an XBAR Input pin...
    // OV7670_PLK   4
  // OV7670_PLK   8    //8       B1_00   FlexIO2:16  XBAR IO14

  _save_pclkPin_portConfigRegister = *(portConfigRegister(_pclkPin));
  *(portConfigRegister(_pclkPin)) = 1; // set to XBAR mode 14

  // route the timer outputs through XBAR to edge trigger DMA request
  CCM_CCGR2 |= CCM_CCGR2_XBAR1(CCM_CCGR_ON);
  xbar_connect(XBARA1_IN_IOMUX_XBAR_INOUT14, XBARA1_OUT_DMA_CH_MUX_REQ30);
  //DebugDigitalToggle(OV7670_DEBUG_PIN_1);

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


  //DebugDigitalToggle(OV7670_DEBUG_PIN_1);

  // Falling edge indicates start of frame
//  while ((*_vsyncPort & _vsyncMask) == 0); // wait for HIGH
//  while ((*_vsyncPort & _vsyncMask) != 0); // wait for LOW
//  DebugDigitalWrite(OV7670_DEBUG_PIN_2, HIGH);

// Debug stuff for now

  // We have the start of a frame, so lets start the dma.
#ifdef DEBUG_CAMERA
  dumpDMA_TCD(&_dmachannel," CH: ");
  dumpDMA_TCD(&_dmasettings[0], " 0: ");
  dumpDMA_TCD(&_dmasettings[1], " 1: ");

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

  //DebugDigitalToggle(OV7670_DEBUG_PIN_1);
  return true;
}

//===================================================================
// stopReadFrameDMA - stop doing the reading and then exit.
//===================================================================
bool OV767X::stopReadFrameDMA()
{

  // hopefully it start here (fingers crossed)
  // for now will hang here to see if completes...
  #ifdef OV7670_USE_DEBUG_PINS
  //DebugDigitalWrite(OV7670_DEBUG_PIN_2, HIGH);
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
  #ifdef OV7670_USE_DEBUG_PINS
  //DebugDigitalWrite(OV7670_DEBUG_PIN_2, LOW);
  #endif
#ifdef DEBUG_CAMERA
  dumpDMA_TCD(&_dmachannel, nullptr);
  dumpDMA_TCD(&_dmasettings[0], nullptr);
  dumpDMA_TCD(&_dmasettings[1], nullptr);
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
void  OV767X::frameStartInterrupt() {
  active_dma_camera->processFrameStartInterrupt();  // lets get back to the main object...
}

void  OV767X::processFrameStartInterrupt() {
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
void OV767X::dmaInterrupt() {
  active_dma_camera->processDMAInterrupt();  // lets get back to the main object...
}


// This version assumes only called when HREF...  as set pixclk to only fire
// when set.
void OV767X::processDMAInterrupt() {
  _dmachannel.clearInterrupt(); // tell system we processed it.
  asm("DSB");
  #ifdef OV7670_USE_DEBUG_PINS
  //DebugDigitalWrite(OV7670_DEBUG_PIN_3, HIGH);
  #endif
  
  if (_dma_state == DMA_STATE_STOPPED) {
    Serial.println("OV767X::dmaInterrupt called when DMA_STATE_STOPPED");
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
    #ifdef OV7670_USE_DEBUG_PINS
    //DebugDigitalWrite(OV7670_DEBUG_PIN_2, LOW);
    #endif
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
      Serial.println("OV767X::dmaInterrupt - Stop requested");
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
  #ifdef OV7670_USE_DEBUG_PINS
  //DebugDigitalWrite(OV7670_DEBUG_PIN_3, LOW);
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

frameStatics_t fstatOmni;

void OV767X::captureFrameStatistics()
{
   memset((void*)&fstatOmni, 0, sizeof(fstatOmni));

   // lets wait for the vsync to go high;
    while ((*_vsyncPort & _vsyncMask) != 0); // wait for HIGH
    // now lets wait for it to go low    
    while ((*_vsyncPort & _vsyncMask) == 0) fstatOmni.vsyncStartCycleCount ++; // wait for LOW

    while ((*_hrefPort & _hrefMask) == 0); // wait for HIGH
    while ((*_pclkPort & _pclkMask) != 0); // wait for LOW

    uint32_t microsStart = micros();
    fstatOmni.hrefStartTime[0] = microsStart;
    // now loop through until we get the next _vsynd
    // BUGBUG We know that HSYNC and PCLK on same GPIO VSYNC is not...
    uint32_t regs_prev = 0;
    //noInterrupts();
    while ((*_vsyncPort & _vsyncMask) != 0) {

        fstatOmni.cycleCount++;
        uint32_t regs = (*_hrefPort & (_hrefMask | _pclkMask ));
        if (regs != regs_prev) {
            if ((regs & _hrefMask) && ((regs_prev & _hrefMask) ==0)) {
                fstatOmni.hrefCount++;
                fstatOmni.hrefStartTime[fstatOmni.hrefCount] = micros();
            }
            if ((regs & _pclkMask) && ((regs_prev & _pclkMask) ==0)) fstatOmni.pclkCounts[fstatOmni.hrefCount]++;
            if ((regs & _pclkMask) && ((regs_prev & _hrefMask) ==0)) fstatOmni.pclkNoHrefCount++;
            regs_prev = regs;
        }
    }
    while ((*_vsyncPort & _vsyncMask) == 0) fstatOmni.vsyncEndCycleCount++; // wait for LOW
    //interrupts();
    fstatOmni.frameTimeMicros = micros() - microsStart;

    // Maybe return data. print now
    Serial.printf("*** Frame Capture Data: elapsed Micros: %u loops: %u\n", fstatOmni.frameTimeMicros, fstatOmni.cycleCount);
    Serial.printf("   VSync Loops Start: %u end: %u\n", fstatOmni.vsyncStartCycleCount, fstatOmni.vsyncEndCycleCount);
    Serial.printf("   href count: %u pclk ! href count: %u\n    ", fstatOmni.hrefCount,  fstatOmni.pclkNoHrefCount);
    for (uint16_t ii=0; ii < fstatOmni.hrefCount + 1; ii++) {
        Serial.printf("%3u(%u) ", fstatOmni.pclkCounts[ii], (ii==0)? 0 : fstatOmni.hrefStartTime[ii] - fstatOmni.hrefStartTime[ii-1]);
        if (!(ii & 0x0f)) Serial.print("\n    ");
    }
    Serial.println();
}

// Read a single uint8_t from address and return it as a uint8_t
uint8_t OV767X::cameraReadRegister(uint8_t reg) {
  Wire.beginTransmission(0x42>>1);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    Serial.println("error reading OV767X, address");
    return 0;
  }
  if (Wire.requestFrom(0x42>>1, 1) < 1) {
    Serial.println("error reading OV767X, data");
    return 0;
  }
  return Wire.read();
}


void OV767X::showRegisters() {
  ov7670_printRegs();

}

bool OV767X::writeRegister(uint8_t reg, uint8_t data) {
  Wire.beginTransmission(0x42>>1);
  Wire.write(reg);
  Wire.write(data);
 
  if (Wire.endTransmission() != 0) return false;
  return true;
}

