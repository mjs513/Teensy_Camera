//=============================================================================
// Default implementations for some of the ImageSensor class
//=============================================================================

#include "camera.h"

#define debug     Serial

//#define DEBUG_CAMERA
//#define DEBUG_CAMERA_VERBOSE
//#define DEBUG_FLEXIO
//#define USE_DEBUG_PINS

ImageSensor *ImageSensor::active_dma_camera = nullptr;
DMAChannel ImageSensor::_dmachannel;
DMASetting ImageSensor::_dmasettings[10];

//#define USE_DEBUG_PINS_TIMING

#ifdef USE_DEBUG_PINS_TIMING
#define DBGdigitalWriteFast digitalWriteFast
#define DBGdigitalToggleFast digitalToggleFast
#else
static inline void DBGdigitalWriteFast(uint8_t pin, uint8_t val) __attribute__((always_inline, unused));
static inline void DBGdigitalWriteFast(uint8_t pin, uint8_t val) {}
static inline void DBGdigitalToggleFast(uint8_t pin) __attribute__((always_inline, unused));
static inline void DBGdigitalToggleFast(uint8_t pin) {}
#endif


void ImageSensor::setPins(uint8_t mclk_pin, uint8_t pclk_pin, uint8_t vsync_pin, uint8_t hsync_pin, uint8_t en_pin,
                     uint8_t g0, uint8_t g1, uint8_t g2, uint8_t g3, uint8_t g4, uint8_t g5, uint8_t g6, uint8_t g7, TwoWire &wire)
{
  #ifdef DEBUG_CAMERA
  debug.printf("void ImageSensor::setPins(MC:%u PC:%u vs:%u hr:%u en:%u, DP: %u %u %u %u %u %u %u %u W:%p)\n", mclk_pin, pclk_pin, vsync_pin, hsync_pin, en_pin,
                     g0, g1, g2, g3, g4, g5, g6, g7, &wire);
  #endif
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
              
  if (g4 == 0xff) {
    _hw_config = TEENSY_MICROMOD_FLEXIO_4BIT;
  } else {
    _hw_config = TEENSY_MICROMOD_FLEXIO_8BIT;
  }
}

size_t ImageSensor::readFrame(void *buffer1, size_t cb1, void *buffer2, size_t cb2) {
    if(!_use_gpio) {
      return readFrameFlexIO(buffer1, cb1, buffer2, cb2);
    } else {
      if(_hw_config == TEENSY_MICROMOD_FLEXIO_4BIT) {
          readFrame4BitGPIO(buffer1);
          return cb1; // not sure what to return here yet...
      }
      return readFrameGPIO(buffer1, cb1, buffer2, cb2);
    }
}


// The read and stop continuous typically just call off to the FlexIO
bool ImageSensor::readContinuous(bool(*callback)(void *frame_buffer), void *fb1, size_t cb1, void *fb2, size_t cb2) {

  return startReadFlexIO(callback, fb1, cb1, fb2, cb2);

}

void ImageSensor::stopReadContinuous() {
  
  stopReadFlexIO();

}

//=============================================================================
// FlexIO support
//=============================================================================

size_t ImageSensor::readFrameFlexIO(void *buffer, size_t cb1, void* buffer2, size_t cb2)
{
    if (_debug)debug.printf("$$ImageSensor::readFrameFlexIO(%p, %u, %p, %u, %u, %u)\n", buffer, cb1, buffer2, cb2, _fuse_dma, _hw_config);
    DBGdigitalWriteFast(0, HIGH);

    uint32_t frame_size_bytes = _width*_height*_bytesPerPixel;
    if(_format == 8){
      frame_size_bytes = frame_size_bytes / 5;
    }
    if ((cb1+cb2) < frame_size_bytes) return 0; // not enough to hold image

    //flexio_configure(); // one-time hardware setup
    // wait for VSYNC to go high and then low with a sort of glitch filter
    elapsedMillis emWaitSOF;
    elapsedMicros emGlitch;
    DBGdigitalWriteFast(0, LOW);

    for (;;) {
      if (emWaitSOF > _timeout) {
        if(_debug) debug.println("Timeout waiting for Start of Frame");
        return 0;
      }
      while ((*_vsyncPort & _vsyncMask) == 0) {
        if (emWaitSOF > _timeout) {
          if(_debug) debug.println("Timeout waiting for rising edge Start of Frame");
          return 0;
        }
      }
      emGlitch = 0;
      while ((*_vsyncPort & _vsyncMask) != 0) {
        if (emWaitSOF > _timeout) {
          if(_debug) debug.println("Timeout waiting for falling edge Start of Frame");
          return 0;
        }
      }
      if (emGlitch > 5) break;
    }
    _pflexio->SHIFTSTAT = _fshifter_mask; // clear any prior shift status
    _pflexio->SHIFTERR = _fshifter_mask;
    uint32_t *p = (uint32_t *)buffer;
    DBGdigitalWriteFast(0, HIGH);

    elapsedMillis timeout = 0;

    //----------------------------------------------------------------------
    // Polling FlexIO version
    //----------------------------------------------------------------------
    if (!_fuse_dma) {
      if (_debug)debug.println("\tNot DMA");
      // lets try another version of this to see if cleaner.
      uint32_t *p_end = (uint32_t*)((uint8_t *)p + cb1);
      uint32_t max_time_to_wait = _timeout;

      uint32_t frame_bytes_received = 0;
      while (frame_bytes_received < frame_size_bytes) {
          while ((_pflexio->SHIFTSTAT & _fshifter_mask) == 0) {
            if (timeout > max_time_to_wait) {              
              DBGdigitalWriteFast(0, LOW);

              if(_debug) debug.printf("Timeout between characters: received: %u bytes\n", frame_bytes_received);
              // wait for FlexIO shifter data
              DBGdigitalWriteFast(0, HIGH);
              break;
            }
          }
          // Lets simplify back to single shifter for now
          uint8_t *pu8 = (uint8_t *)p;
          *p++ = _pflexio->SHIFTBUF[_fshifter]; 
          if((_format == 8) && (frame_bytes_received > 0)){
            for (int i = 0; i < 4; i++) {
              if ((pu8[i-1] == 0xff) && (pu8[i] == 0xd9)) {
                if (_debug)Serial.printf("JPEG - found end marker at %u\n", frame_bytes_received + i);
                DBGdigitalWriteFast(0, LOW);
                return frame_bytes_received + i + 1;
              }

            }
          }
            // jpeg check for 
          if (p >= p_end) {
            p = (uint32_t*)buffer2;
            p_end = (uint32_t*)((uint8_t *)p + cb2);
          }
          frame_bytes_received += sizeof(uint32_t);
          max_time_to_wait = 50; // maybe second setting.
          timeout = 0; // reset timeout. 
      }

    DBGdigitalWriteFast(0, LOW);
    return frame_bytes_received;
    }

    //----------------------------------------------------------------------
    // Use DMA FlexIO version
    //----------------------------------------------------------------------

    _dmachannel.begin();
    _dmachannel.triggerAtHardwareEvent(_dma_source);
    active_dma_camera = this;
    _dmachannel.attachInterrupt(dmaInterruptFlexIO);

    // Some cameras or sizes we cans imply do within the DMAChannel object, so lets
    // see if that helps with the HM01B0


    // first pass split into two
    uint8_t dmas_index = 0;
    // We will do like above with both buffers, maybe later try to merge the two sections.
    uint32_t cb_left = min(frame_size_bytes, cb1);
    uint8_t count_dma_settings = (cb_left / (32767 * 4)) + 1;
    uint32_t cb_per_setting = ((cb_left / count_dma_settings) + 3) & 0xfffffffc; // round up to next multiple of 4.
    if (_debug) debug.printf("frame size: %u, cb1:%u cnt dma: %u CB per: %u\n", frame_size_bytes, cb1, count_dma_settings, cb_per_setting);
    if ((cb1 >= frame_size_bytes) && (count_dma_settings == 1)) {
      _dmachannel.source(_pflexio->SHIFTBUF[_fshifter]);
      _dmachannel.destinationBuffer((uint32_t *)buffer, frame_size_bytes);
      _dmachannel.transferSize(4);
      _dmachannel.transferCount(frame_size_bytes / 4);
      _dmachannel.disableOnCompletion();
      _dmachannel.interruptAtCompletion();
      _dmachannel.clearComplete();
      #ifdef DEBUG_FLEXIO
      if (_debug) {
        dumpDMA_TCD(&_dmachannel,     "CH: ");
      }
      #endif
    } else {
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
        if (_debug) debug.printf("frame size left: %u, cb2:%u cnt dma: %u CB per: %u\n", cb_left, cb2, count_dma_settings, cb_per_setting);
        
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
      _dmachannel = _dmasettings[0];
      _dmachannel.clearComplete();
  #ifdef DEBUG_FLEXIO
      if (_debug) {
        dumpDMA_TCD(&_dmachannel," CH: ");
        for (uint8_t i = 0; i <= dmas_index; i++) {
          debug.printf(" %u: ", i);
          dumpDMA_TCD(&_dmasettings[i], nullptr);
        }
      }
  #endif
    }

    _dma_state = DMA_STATE_ONE_FRAME;
    _pflexio->SHIFTSDEN = _fshifter_mask;
    _dmachannel.enable();
    
#ifdef DEBUG_FLEXIO
    if (_debug) debug.printf("Flexio DMA: length: %d\n", frame_size_bytes);
#endif
    
    timeout = 0;  // reset the timeout
    //while (!_dmachannel.complete()) {
    while (_dma_state == DMA_STATE_ONE_FRAME) {
        // wait - we should not need to actually do anything during the DMA transfer
        if (_dmachannel.error()) {
            debug.println("DMA error");
            if (_pflexio->SHIFTSTAT) debug.printf(" SHIFTSTAT %08X\n", _pflexio->SHIFTSTAT);
            debug.flush();
            uint32_t i = _pflexio->SHIFTBUF[_fshifter];
            debug.printf("Result: %x\n", i);


            _dmachannel.clearError();
            break;
        }
        if (timeout > _timeout) {
            if (_debug) debug.println("Timeout waiting for DMA");
            if (_pflexio->SHIFTSTAT & _fshifter_mask) debug.printf(" SHIFTSTAT bit was set (%08X)\n", _pflexio->SHIFTSTAT);
            #ifdef DEBUG_CAMERA
            debug.printf(" DMA channel #%u\n", _dmachannel.channel);
            debug.printf(" DMAMUX = %08X\n", *(&DMAMUX_CHCFG0 + _dmachannel.channel));
            debug.printf(" _pflexio->SHIFTSDEN = %02X\n", _pflexio->SHIFTSDEN);
            debug.printf(" TCD CITER = %u\n", _dmachannel.TCD->CITER_ELINKNO);
            debug.printf(" TCD CSR = %08X\n", _dmachannel.TCD->CSR);
            #endif
            break;
        }
    }
    #ifdef USE_DEBUG_PINS
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
  DBGdigitalWriteFast(0, LOW);

    return frame_size_bytes;
}

void ImageSensor::frameStartInterruptFlexIO()
{
  active_dma_camera->processFrameStartInterruptFlexIO();
}

void ImageSensor::processFrameStartInterruptFlexIO()
{
  #ifdef USE_DEBUG_PINS
  digitalWriteFast(5, HIGH);
  #endif
  //debug.println("VSYNC");
  // See if we read the state of it a few times if the pin stays high...
  if (digitalReadFast(_vsyncPin) && digitalReadFast(_vsyncPin) && digitalReadFast(_vsyncPin) 
          && digitalReadFast(_vsyncPin) )  {
    // stop this interrupt.
    #ifdef USE_DEBUG_PINS
    //digitalToggleFast(2);
    digitalWriteFast(2, LOW);
    digitalWriteFast(2, HIGH);
    #endif
    detachInterrupt(_vsyncPin);

    // For this pass will leave in longer DMAChain with both buffers.
    _pflexio->SHIFTSTAT = _fshifter_mask; // clear any prior shift status
    _pflexio->SHIFTERR = _fshifter_mask;

    _dmachannel.clearComplete();
    _dmachannel.enable();
  }
  asm("DSB");
  #ifdef USE_DEBUG_PINS
  digitalWriteFast(5, LOW);
  #endif
}


void ImageSensor::dmaInterruptFlexIO()
{
  active_dma_camera->processDMAInterruptFlexIO();
}

void ImageSensor::processDMAInterruptFlexIO()
{

  _dmachannel.clearInterrupt();
  #ifdef USE_DEBUG_PINS
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
    debug.printf("PDMAIF: %x\n", (uint32_t)_dmachannel.TCD->DADDR);
    dumpDMA_TCD(&_dmachannel," CH: ");

  }
#endif  
  _dmachannel.clearComplete();
  const uint32_t frame_size_bytes = _width*_height*_bytesPerPixel;
  if (((uint32_t)_dmachannel.TCD->DADDR) == (uint32_t)_frame_buffer_1) {
     _dma_last_completed_frame = _frame_buffer_2;
    if ((uint32_t)_frame_buffer_2 >= 0x20200000u) arm_dcache_delete(_frame_buffer_2, min(_frame_buffer_2_size, frame_size_bytes));
  } else {
     _dma_last_completed_frame = _frame_buffer_1;
    if ((uint32_t)_frame_buffer_1 >= 0x20200000u) arm_dcache_delete(_frame_buffer_1, min(_frame_buffer_1_size, frame_size_bytes));    
  }

  if (_callback) (*_callback)(_dma_last_completed_frame); // TODO: use EventResponder
  // if we disabled the DMA, then setup to wait for vsyncpin...
  if ((_dma_last_completed_frame == _frame_buffer_2) || (_frame_buffer_1_size >= frame_size_bytes)) {
    _dma_active = false;
    // start up interrupt to look for next start of interrupt.
    
    if (_dma_state == DMASTATE_RUNNING) attachInterrupt(_vsyncPin, &frameStartInterruptFlexIO, RISING);
  }

  asm("DSB");
}


bool ImageSensor::startReadFlexIO(bool(*callback)(void *frame_buffer), void *fb1, size_t cb1, void *fb2, size_t cb2)
{

    const uint32_t frame_size_bytes = _width*_height*_bytesPerPixel;
    // lets handle a few cases.
    if (fb1 == nullptr) return false;
    if (cb1 < frame_size_bytes) {
      if ((fb2 == nullptr) || ((cb1+cb2) < frame_size_bytes)) return false;
    }

    _frame_buffer_1 = (uint8_t *)fb1;
    _frame_buffer_1_size = cb1;
    _frame_buffer_2 = (uint8_t *)fb2;
    _frame_buffer_2_size = cb2;
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


    _dmachannel.begin();
    _dmachannel.triggerAtHardwareEvent(_dma_source);
    active_dma_camera = this;
    _dmachannel.attachInterrupt(dmaInterruptFlexIO);


    // Two versions.  If one buffer is large enough, we will use the two buffers to 
    // read in two frames.  If the combined in large enough for one frame, we will 
    // setup to read one frame but still interrupt on each buffer filled completion

    uint8_t dmas_index = 0;
    
    uint32_t cb_left = min(frame_size_bytes, cb1);
    uint8_t count_dma_settings = (cb_left / (32767 * 4)) + 1;
    uint32_t cb_per_setting = ((cb_left / count_dma_settings) + 3) & 0xfffffffc; // round up to next multiple of 4.
    if (_debug) debug.printf("frame size: %u, cb1:%u cnt dma: %u CB per: %u\n", frame_size_bytes, cb1, count_dma_settings, cb_per_setting);

    for (; dmas_index < count_dma_settings; dmas_index++) {
      _dmasettings[dmas_index].TCD->CSR = 0;
      _dmasettings[dmas_index].source(_pflexio->SHIFTBUF[_fshifter]);
      _dmasettings[dmas_index].destinationBuffer(p, cb_per_setting);
      _dmasettings[dmas_index].replaceSettingsOnCompletion(_dmasettings[dmas_index + 1]);
      p += (cb_per_setting / 4);
      cb_left -= cb_per_setting;
      if (cb_left < cb_per_setting) cb_per_setting = cb_left;
    }
    // Interrupt after each buffer is filled.
    _dmasettings[dmas_index-1].interruptAtCompletion();
    if (cb1 >= frame_size_bytes) {
      _dmasettings[dmas_index-1].disableOnCompletion();  // full frame
      if (fb2 && (cb2 >= frame_size_bytes)) cb_left = min(frame_size_bytes, cb2);
    } else cb_left = frame_size_bytes - cb1;  // need second buffer to complete one frame.

    if (cb_left) {
      count_dma_settings = (cb_left / (32767 * 4)) + 1;
      cb_per_setting = ((cb_left / count_dma_settings) + 3) & 0xfffffffc; // round up to next multiple of 4.
      if (_debug) debug.printf("frame size left: %u, cb2:%u cnt dma: %u CB per: %u\n", cb_left, cb2, count_dma_settings, cb_per_setting);
      
      p = (uint32_t *)fb2;

      for (uint8_t i=0; i < count_dma_settings; i++, dmas_index++) {
        _dmasettings[dmas_index].TCD->CSR = 0;
        _dmasettings[dmas_index].source(_pflexio->SHIFTBUF[_fshifter]);
        _dmasettings[dmas_index].destinationBuffer(p, cb_per_setting);
        _dmasettings[dmas_index].replaceSettingsOnCompletion(_dmasettings[dmas_index + 1]);
        p += (cb_per_setting / 4);
        cb_left -= cb_per_setting;
        if (cb_left < cb_per_setting) cb_per_setting = cb_left;
      }
      _dmasettings[dmas_index-1].disableOnCompletion();
      _dmasettings[dmas_index-1].interruptAtCompletion();
    }  
    dmas_index--; // lets point back to the last one
    _dmasettings[dmas_index].replaceSettingsOnCompletion(_dmasettings[0]);
    _dmachannel = _dmasettings[0];
    _dmachannel.clearComplete();

#ifdef DEBUG_FLEXIO
    if (_debug) {
      dumpDMA_TCD(&_dmachannel," CH: ");
      for (uint8_t i = 0; i <= dmas_index; i++) {
        debug.printf(" %u: ", i);
        dumpDMA_TCD(&_dmasettings[i], nullptr);
      }
    }
    debug.printf("Flexio DMA: length: %d\n", frame_size_bytes);

#endif

    _pflexio->SHIFTSTAT = _fshifter_mask; // clear any prior shift status
    _pflexio->SHIFTERR = _fshifter_mask;


    _dma_last_completed_frame = nullptr;
    _dma_frame_count = 0;

    _dma_state = DMASTATE_RUNNING;

    // Lets use interrupt on interrupt on VSYNC pin to start the capture of a frame
    _dma_active = false;
    NVIC_SET_PRIORITY(IRQ_GPIO6789, 102);
    //NVIC_SET_PRIORITY(dma_flexio.channel & 0xf, 102);
    attachInterrupt(_vsyncPin, &frameStartInterruptFlexIO, RISING);
    _pflexio->SHIFTSDEN = _fshifter_mask;

    return true;
}



bool ImageSensor::stopReadFlexIO()
{
  // first disable the vsync interrupt
  detachInterrupt(_vsyncPin);
  if (!_dma_active) {
    _dma_state = DMA_STATE_STOPPED;
  } else {
    cli();
    if (_dma_state != DMA_STATE_STOPPED) _dma_state = DMASTATE_STOP_REQUESTED;
    sei();
  }
  return true;
}

void ImageSensor::dumpDMA_TCD(DMABaseClass *dmabc, const char *psz_title) {
  if (psz_title)
    debug.print(psz_title);
  debug.printf("%x %x: ", (uint32_t)dmabc, (uint32_t)dmabc->TCD);

  debug.printf(
      "SA:%x SO:%d AT:%x (SM:%x SS:%x DM:%x DS:%x) NB:%x SL:%d DA:%x DO: %d CI:%x DL:%x CS:%x BI:%x\n",
      (uint32_t)dmabc->TCD->SADDR, dmabc->TCD->SOFF, dmabc->TCD->ATTR,
      (dmabc->TCD->ATTR >> 11) & 0x1f, (dmabc->TCD->ATTR >> 8) & 0x7,
      (dmabc->TCD->ATTR >> 3) & 0x1f, (dmabc->TCD->ATTR >> 0) & 0x7,
      dmabc->TCD->NBYTES, dmabc->TCD->SLAST, (uint32_t)dmabc->TCD->DADDR,
      dmabc->TCD->DOFF, dmabc->TCD->CITER, dmabc->TCD->DLASTSGA,
      dmabc->TCD->CSR, dmabc->TCD->BITER);
}


bool ImageSensor::flexio_configure()
{
#ifdef DEBUG_CAMERA
  debug.println("ImageSensor::flexio_configure() called");
#endif    
    uint8_t tpclk_pin; 
    _pflex = FlexIOHandler::mapIOPinToFlexIOHandler(_pclkPin, tpclk_pin);
    if (!_pflex) {
        debug.printf("ImageSensor PCLK(%u) is not a valid Flex IO pin\n", _pclkPin);
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
        debug.printf("OV767X Some pins did not map to valid Flex IO pin\n");
        if(_debug) debug.printf("    HSYNC(%u %u) G0(%u %u) G1(%u %u) G2(%u %u) G3(%u %u)", 
            _hrefPin, thsync_pin, _dPins[0], tg0, _dPins[1], tg1, _dPins[2], tg2, _dPins[3], tg3 );
        return false;
    } 
    // Verify that the G numbers are consecutive... Should use arrays!
    if ((tg1 != (tg0+1)) || (tg2 != (tg0+2)) || (tg3 != (tg0+3))) {
        debug.printf("OV767X Flex IO pins G0-G3 are not consective\n");
        if(_debug) debug.printf("    G0(%u %u) G1(%u %u) G2(%u %u) G3(%u %u)", 
            _dPins[0], tg0, _dPins[1], tg1, _dPins[2], tg2, _dPins[3], tg3 );
        return false;
    }
    if (_dPins[4] != 0xff) {
        uint8_t tg4 = _pflex->mapIOPinToFlexPin(_dPins[4]);
        uint8_t tg5 = _pflex->mapIOPinToFlexPin(_dPins[5]);
        uint8_t tg6 = _pflex->mapIOPinToFlexPin(_dPins[6]);
        uint8_t tg7 = _pflex->mapIOPinToFlexPin(_dPins[7]);
        if ((tg4 != (tg0+4)) || (tg5 != (tg0+5)) || (tg6 != (tg0+6)) || (tg7 != (tg0+7))) {
            debug.printf("OV767X Flex IO pins G4-G7 are not consective with G0-3\n");
            if(_debug) debug.printf("    G0(%u %u) G4(%u %u) G5(%u %u) G6(%u %u) G7(%u %u)", 
                _dPins[0], tg0, _dPins[4], tg4, _dPins[5], tg5, _dPins[6], tg6, _dPins[7], tg7 );
            return false;
        }
        if(_debug) debug.println("Custom - Flexio is 8 bit mode");
    } else {
      // only 8 bit mode supported
        _hw_config = TEENSY_MICROMOD_FLEXIO_4BIT;
        if(_debug) debug.println("Custom - Flexio is 4 bit mode");
    }
    // Needs Shifter 3 (maybe 7 would work as well?)
    if (_pflex->claimShifter(3)) _fshifter = 3;
    else if (_pflex->claimShifter(7)) _fshifter = 7;
    else {
      if(_debug) debug.printf("OV767X Flex IO: Could not claim Shifter 3 or 7\n");
      return false;
    }
    _fshifter_mask = 1 << _fshifter;   // 4 channels.
    _dma_source = _pflex->shiftersDMAChannel(_fshifter); // looks like they use 
    
    // Now request one timer
    uint8_t _ftimer = _pflex->requestTimers(); // request 1 timer. 
    if (_ftimer == 0xff) {
        if(_debug) debug.printf("ImageSensor Flex IO: failed to request timer\n");
        return false;
    }

    _pflex->setIOPinToFlexMode(_hrefPin);
    _pflex->setIOPinToFlexMode(_pclkPin);
    _pflex->setIOPinToFlexMode(_dPins[0]);
    _pflex->setIOPinToFlexMode(_dPins[1]);
    _pflex->setIOPinToFlexMode(_dPins[2]);
    _pflex->setIOPinToFlexMode(_dPins[3]);
    if (_dPins[4] != 0xff) {
      _pflex->setIOPinToFlexMode(_dPins[4]);
      _pflex->setIOPinToFlexMode(_dPins[5]);
      _pflex->setIOPinToFlexMode(_dPins[6]);
      _pflex->setIOPinToFlexMode(_dPins[7]);
    }


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
    debug.println("FlexIO Configure");
    debug.printf(" CCM_CSCMR2 = %08X\n", CCM_CSCMR2);
    uint32_t div1 = ((CCM_CS1CDR >> 9) & 7) + 1;
    uint32_t div2 = ((CCM_CS1CDR >> 25) & 7) + 1;
    debug.printf(" div1 = %u, div2 = %u\n", div1, div2);
    debug.printf(" FlexIO Frequency = %.2f MHz\n", 480.0 / (float)div1 / (float)div2);
    debug.printf(" CCM_CCGR3 = %08X\n", CCM_CCGR3);
    debug.printf(" FlexIO CTRL = %08X\n", _pflexio->CTRL);
    debug.printf(" FlexIO Config, param=%08X\n", _pflexio->PARAM);
#endif


      #define FLEXIO_TIMER_TRIGGER_SEL_PININPUT(x) ((uint32_t)(x) << 1U)
    if(_hw_config == TEENSY_MICROMOD_FLEXIO_8BIT) {
#ifdef DEBUG_FLEXIO
      debug.println("8Bit FlexIO");
#endif
      // SHIFTCFG, page 2927
      //  PWIDTH: number of bits to be shifted on each Shift clock
      //          0 = 1 bit, 1-3 = 4 bit, 4-7 = 8 bit, 8-15 = 16 bit, 16-31 = 32 bit
      //  INSRC: Input Source, 0 = pin, 1 = Shifter N+1 Output
      //  SSTOP: Stop bit, 0 = disabled, 1 = match, 2 = use zero, 3 = use one
      //  SSTART: Start bit, 0 = disabled, 1 = disabled, 2 = use zero, 3 = use one
      // setup the for shifters
      _pflexio->SHIFTCFG[_fshifter] = FLEXIO_SHIFTCFG_PWIDTH(7);

      // Timer model, pages 2891-2893
      // TIMCMP, page 2937
      // using 1 shifters
      _pflexio->TIMCMP[_ftimer] = (8U * 1) -1 ;

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
          | FLEXIO_TIMCTL_PINSEL(tpclk_pin) // "Pin" is 16 = PCLK
          //| FLEXIO_TIMCTL_TRGSEL(4 * (thsync_pin/2)) // "Trigger" is 12 = HSYNC
          | FLEXIO_TIMCTL_TRGSEL(FLEXIO_TIMER_TRIGGER_SEL_PININPUT(thsync_pin)) // "Trigger" is 12 = HSYNC
          | FLEXIO_TIMCTL_TRGSRC;
    } else {

#ifdef DEBUG_FLEXIO
      debug.println("4Bit FlexIO");
#endif
      if (!supports4BitMode()) {
        debug.println("Error: Camera does not support 4 FlexIO pin mode");
        return false;
      }
      // registers fields shown in 8 bit mode.
      _pflexio->SHIFTCFG[_fshifter] = FLEXIO_SHIFTCFG_PWIDTH(3);  // 4 pins
      _pflexio->TIMCMP[_ftimer] = 15 ;
      _pflexio->TIMCTL[_ftimer] = FLEXIO_TIMCTL_TIMOD(3)
          | FLEXIO_TIMCTL_PINSEL(tpclk_pin) // "Pin" is 16 = PCLK
          | FLEXIO_TIMCTL_TRGSEL(FLEXIO_TIMER_TRIGGER_SEL_PININPUT(thsync_pin)) // "Trigger" is 12 = HSYNC
          | FLEXIO_TIMCTL_TRGSRC;
    }      
      
    #ifdef DEBUG_FLEXIO
      debug.printf("TIMCTL: %08X PINSEL: %x THSYNC: %x\n", _pflexio->TIMCTL[_ftimer], tpclk_pin, thsync_pin);
    #endif
    
    // SHIFTCTL, page 2926
    //  TIMSEL: which Timer is used for controlling the logic/shift register
    //  TIMPOL: 0 = shift of positive edge, 1 = shift on negative edge
    //  PINCFG: 0 = output disabled, 1 = open drain, 2 = bidir, 3 = output
    //  PINSEL: which pin is used by the Shifter input or output
    //  PINPOL: 0 = active high, 1 = active low
    //  SMOD: 0 = disable, 1 = receive, 2 = transmit, 4 = match store,
    //        5 = match continuous, 6 = state machine, 7 = logic
    // 4 shifters
    _pflexio->SHIFTCTL[_fshifter] = FLEXIO_SHIFTCTL_TIMSEL(_ftimer) | FLEXIO_SHIFTCTL_SMOD(1)
        | FLEXIO_SHIFTCTL_PINSEL(tg0);    

    

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
    debug.printf(" FLEXIO:%u Shifter:%u Timer:%u\n", _pflex->FlexIOIndex(), _fshifter, _ftimer);
    debug.print("     SHIFTCFG = ");
    debug.printf(" %08X", _pflexio->SHIFTCFG[_fshifter]);
    debug.print("\n     SHIFTCTL = ");
    debug.printf(" %08X", _pflexio->SHIFTCTL[_fshifter]);
    debug.printf("\n     TIMCMP = %08X\n", _pflexio->TIMCMP[_ftimer]);
    debug.printf("     TIMCFG = %08X\n", _pflexio->TIMCFG[_ftimer]);
    debug.printf("     TIMCTL = %08X\n", _pflexio->TIMCTL[_ftimer]);
#endif
return true;
}

//=============================================================================
// Reading using GPIO support
//=============================================================================
size_t ImageSensor::readFrameGPIO(void *buffer, size_t cb1, void *buffer2, size_t cb2)
{    
  debug.printf("$$ImageSensor::readFrameGPIO(%p, %u, %p, %u)\n", buffer, cb1, buffer2, cb2);
  const uint32_t frame_size_bytes = _width*_height*_bytesPerPixel;
  
  if ((cb1+cb2) < frame_size_bytes) return 0; // not enough to hold image
  DBGdigitalWriteFast(0, HIGH);

  uint8_t* b = (uint8_t*)buffer;
  uint32_t cb = (uint32_t)cb1;
//  bool _grayscale;  // ????  member variable ?????????????
  int bytesPerRow = _width * _bytesPerPixel;

  // Falling edge indicates start of frame
  //pinMode(PCLK_PIN, INPUT); // make sure back to input pin...
  // lets add our own glitch filter.  Say it must be hig for at least 100us

  delayMicroseconds(5);  // debug for digitalWrite
  DBGdigitalWriteFast(0, LOW);
  elapsedMicros emHigh;
  do {
    while ((*_vsyncPort & _vsyncMask) == 0); // wait for HIGH
    emHigh = 0;
    while ((*_vsyncPort & _vsyncMask) != 0); // wait for LOW
  } while (emHigh < 2);
  DBGdigitalWriteFast(0, HIGH);

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
  DBGdigitalWriteFast(0, LOW);
  return frame_size_bytes;
}


