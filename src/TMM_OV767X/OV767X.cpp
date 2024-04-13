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

#define debug     Serial

//#define DEBUG_CAMERA
//#define DEBUG_CAMERA_VERBOSE
#define DEBUG_FLEXIO
//#define USE_DEBUG_PINS

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
  #ifdef USE_DEBUG_PINS
  pinMode(49, OUTPUT);
  #endif
  
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
  debug.printf("  VS=%d, HR=%d, PC=%d XC=%d\n", _vsyncPin, _hrefPin, _pclkPin, _xclkPin);
#endif

  for (int i = 0; i < 8; i++) {
    pinMode(_dPins[i], INPUT);
    debug.printf("  _dpins(%d)=%d\n", i, _dPins[i]);
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
    if(_debug) debug.println("Camera detect failed");
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
  
  #ifdef DEBUG_CAMERA
  debug.printf("Calling ov7670_configure\n");
  debug.printf("Cam Name: %d, Format: %d, Resolution: %d, Clock: %d\n", camera_name, _format, _framesize, _xclk_freq);
  debug.printf("Frame rate: %d\n", fps);
  #endif
  ov7670_configure(_ov7670, camera_name /*OV7670 = 0, OV7675 = 1*/, _format, _framesize, _xclk_freq /* MHz */,
                   0 /*pll bypass*/, 1 /* pclk_hb_disable */);

  if (ov7670_s_power(_ov7670, 1)) {
    end();
    if(_debug) debug.println("Camera ov7670_s_power failed");
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


size_t OV767X::readFrameGPIO(void *buffer, size_t cb1, void *buffer2, size_t cb2)
{    
  debug.printf("$$readFrameGPIO(%p, %u, %p, %u)\n", buffer, cb1, buffer2, cb2);
  const uint32_t frame_size_bytes = _width*_height*_bytesPerPixel;
  if ((cb1+cb2) < frame_size_bytes) return 0; // not enough to hold image

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
          if(_debug) debug.printf("\t$$ 2nd buffer: %u %u\n", i, j);
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
  return frame_size_bytes;
}





//======================================== DMA JUNK
//================================================================================
// experiment with DMA
//================================================================================
// Define our DMA structure.
DMAChannel OV767X::_dmachannel;
DMASetting OV767X::_dmasettings[10];
uint32_t OV767X::_dmaBuffer1[DMABUFFER_SIZE] __attribute__ ((used, aligned(32)));
uint32_t OV767X::_dmaBuffer2[DMABUFFER_SIZE] __attribute__ ((used, aligned(32)));
extern "C" void xbar_connect(unsigned int input, unsigned int output); // in pwm.c

//OV767X *OV767X::active_dma_camera = nullptr;


//===================================================================
// Start a DMA operation -
//===================================================================
#if 0 //def later
bool OV767X::startReadFrameDMA(bool(*callback)(void *frame_buffer), uint8_t *fb1, uint8_t *fb2) {return false;}
bool OV767X::stopReadFrameDMA() {return false;}

#else
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

  if(_debug) debug.printf("startReadFrameDMA called buffers %x %x\n", (uint32_t)_frame_buffer_1, (uint32_t)_frame_buffer_2);

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

  debug.printf("pclk pin: %d config:%lx control:%lx\n", _pclkPin, *(portConfigRegister(_pclkPin)), *(portControlRegister(_pclkPin)));
  debug.printf("IOMUXC_GPR_GPR26-29:%lx %lx %lx %lx\n", IOMUXC_GPR_GPR26, IOMUXC_GPR_GPR27, IOMUXC_GPR_GPR28, IOMUXC_GPR_GPR29);
  debug.printf("GPIO1: %lx %lx, GPIO6: %lx %lx\n", GPIO1_DR, GPIO1_PSR, GPIO6_DR, GPIO6_PSR);
  debug.printf("XBAR CTRL0:%x CTRL1:%x\n\n", XBARA1_CTRL0, XBARA1_CTRL1);
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
    debug.println("*** stopReadFrameDMA DMA did not exit correctly...");
    debug.printf("  Bytes Left: %u frame buffer:%x Row:%u Col:%u\n", _bytes_left_dma, (uint32_t)_frame_buffer_pointer, _frame_row_index, _frame_col_index);
  }
  #ifdef OV7670_USE_DEBUG_PINS
  //DebugDigitalWrite(OV7670_DEBUG_PIN_2, LOW);
  #endif
#ifdef DEBUG_CAMERA
  dumpDMA_TCD(&_dmachannel, nullptr);
  dumpDMA_TCD(&_dmasettings[0], nullptr);
  dumpDMA_TCD(&_dmasettings[1], nullptr);
  debug.println();
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
  #ifdef USE_DEBUG_PINS
  //DebugDigitalWrite(OV7670_DEBUG_PIN_3, HIGH);
  #endif
  
  if (_dma_state == DMA_STATE_STOPPED) {
    debug.println("OV767X::dmaInterrupt called when DMA_STATE_STOPPED");
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
    debug.printf("D(%d, %d, %lu) %u : ", _dma_index, buffer_size, _bytes_left_dma, pixformat);
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
    #ifdef USE_DEBUG_PINS
    //DebugDigitalWrite(OV7670_DEBUG_PIN_2, LOW);
    #endif
#ifdef DEBUG_CAMERA_VERBOSE
    debug.println("EOF");
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
      debug.println("OV767X::dmaInterrupt - Stop requested");
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

#endif // LATER

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
    debug.printf("*** Frame Capture Data: elapsed Micros: %u loops: %u\n", fstatOmni.frameTimeMicros, fstatOmni.cycleCount);
    debug.printf("   VSync Loops Start: %u end: %u\n", fstatOmni.vsyncStartCycleCount, fstatOmni.vsyncEndCycleCount);
    debug.printf("   href count: %u pclk ! href count: %u\n    ", fstatOmni.hrefCount,  fstatOmni.pclkNoHrefCount);
    for (uint16_t ii=0; ii < fstatOmni.hrefCount + 1; ii++) {
        debug.printf("%3u(%u) ", fstatOmni.pclkCounts[ii], (ii==0)? 0 : fstatOmni.hrefStartTime[ii] - fstatOmni.hrefStartTime[ii-1]);
        if (!(ii & 0x0f)) debug.print("\n    ");
    }
    debug.println();
}

// Read a single uint8_t from address and return it as a uint8_t
uint8_t OV767X::cameraReadRegister(uint8_t reg) {
  Wire.beginTransmission(0x42>>1);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    if(_debug) debug.println("error reading OV767X, address");
    return 0;
  }
  if (Wire.requestFrom(0x42>>1, 1) < 1) {
    if(_debug) debug.println("error reading OV767X, data");
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

