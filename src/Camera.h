#ifndef __CAMERA_H
#define __CAMERA_H

#include <Arduino.h>
#include <DMAChannel.h>
#include <Wire.h>
#include <FlexIO_t4.h>
#include "common.h"

class ImageSensor {
public:
  virtual ~ImageSensor() {}

  // must be called before Camera.begin()
  virtual void setPins(uint8_t mclk_pin, uint8_t pclk_pin, uint8_t vsync_pin, uint8_t hsync_pin, uint8_t en_pin,
                       uint8_t g0, uint8_t g1, uint8_t g2, uint8_t g3,
                       uint8_t g4 = 0xff, uint8_t g5 = 0xff, uint8_t g6 = 0xff, uint8_t g7 = 0xff, TwoWire &wire = Wire) = 0;
  virtual bool begin(framesize_t framesize = FRAMESIZE_QVGA, int framerate = 30, bool use_gpio = false) = 0;
  virtual void end() = 0;
  virtual int reset() = 0;
  virtual void showRegisters(void);
  virtual int setPixformat(pixformat_t pfmt) = 0;
  virtual uint8_t setFramesize(framesize_t framesize) = 0;
  virtual int setFramerate(int framerate) = 0;
  virtual int setBrightness(int level) = 0;
  virtual int setGainceiling(gainceiling_t gainceiling) = 0;
  virtual int setColorbar(int enable) = 0;
  virtual int setAutoGain(int enable, float gain_db, float gain_db_ceiling) = 0;
  virtual int get_vt_pix_clk(uint32_t *vt_pix_clk) = 0;
  virtual int getGain_db(float *gain_db) = 0;
  virtual int getCameraClock(uint32_t *vt_pix_clk) = 0;
  virtual int setAutoExposure(int enable, int exposure_us) = 0;
  virtual int getExposure_us(int *exposure_us) = 0;
  virtual int setHmirror(int enable) = 0;
  virtual int setVflip(int enable) = 0;
  virtual uint8_t setMode(uint8_t Mode, uint8_t FrameCnt) = 0;
  virtual uint8_t cmdUpdate() = 0;
  virtual uint8_t loadSettings(camera_reg_settings_t settings) = 0;
  // HM01B0 only so if you use should return -1)
  virtual uint8_t getAE(ae_cfg_t *psAECfg) = 0;
  virtual uint8_t calAE(uint8_t CalFrames, uint8_t *Buffer, uint32_t ui32BufferLen, ae_cfg_t *pAECfg) = 0;
  virtual uint16_t getModelid() = 0;

  //from 0V767x library
  //virtual void end();  // TODO: need to add + endCLK function + set pins
  virtual void captureFrameStatistics() = 0;

  // grab Frame functions
  //-------------------------------------------------------
  //Generic Read Frame base on _hw_config
  virtual void readFrame(void *buffer) = 0;
  //normal Read mode
  virtual void readFrameGPIO(void *buffer) = 0;
  virtual void readFrame4BitGPIO(void *buffer) = 0;
  virtual bool readContinuous(bool (*callback)(void *frame_buffer), void *fb1, void *fb2) = 0;
  virtual void stopReadContinuous() = 0;

  //FlexIO is default mode for the camera
  //virtual void readFrameFlexIO(void* buffer);
  virtual void readFrameFlexIO(void *buffer, bool fUseDMA) = 0;

  virtual bool startReadFlexIO(bool (*callback)(void *frame_buffer), void *fb1, void *fb2) = 0;
  virtual bool stopReadFlexIO() = 0;

  // Lets try a dma version.  Doing one DMA that is synchronous does not gain anything
  // So lets have a start, stop... Have it allocate 2 frame buffers and it's own DMA
  // buffers, with the option of setting your own buffers if desired.
  virtual bool startReadFrameDMA(bool (*callback)(void *frame_buffer) = nullptr, uint8_t *fb1 = nullptr, uint8_t *fb2 = nullptr) = 0;

  virtual bool stopReadFrameDMA() = 0;

  virtual void setVSyncISRPriority(uint8_t priority) = 0;
  virtual void setDMACompleteISRPriority(uint8_t priority) = 0;

  virtual uint32_t frameCount() = 0;  //{return _dma_frame_count;}

  virtual int16_t width(void) = 0;
  virtual int16_t height(void) = 0;
  virtual int16_t mode(void) = 0;

  framesize_t framesize;
  pixformat_t pixformat;
  camera_reg_settings_t settings;
  hw_config_t _hw_config;
  hw_carrier_t _hw_carrier;
private:
};

class Camera {
public:
  Camera(ImageSensor &sensor);

  // must be called before Camera.begin()
  void setPins(uint8_t mclk_pin, uint8_t pclk_pin, uint8_t vsync_pin, uint8_t hsync_pin, uint8_t en_pin,
               uint8_t g0, uint8_t g1, uint8_t g2, uint8_t g3,
               uint8_t g4 = 0xff, uint8_t g5 = 0xff, uint8_t g6 = 0xff, uint8_t g7 = 0xff, TwoWire &wire = Wire);
  bool begin(framesize_t framesize = FRAMESIZE_QVGA, int framerate = 30, bool use_gpio = false);
  void end();
  int reset();
  void showRegisters(void);
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
  // HM01B0 only so if you use should return -1)
  uint8_t getAE(ae_cfg_t *psAECfg);
  uint8_t calAE(uint8_t CalFrames, uint8_t *Buffer, uint32_t ui32BufferLen, ae_cfg_t *pAECfg);
  uint16_t getModelid();

  // grab Frame functions
  //-------------------------------------------------------
  //Generic Read Frame base on _hw_config
  void readFrame(void *buffer);

  //normal Read mode
  void readFrameGPIO(void *buffer);
  void readFrame4BitGPIO(void *buffer);

  bool readContinuous(bool (*callback)(void *frame_buffer), void *fb1, void *fb2);
  void stopReadContinuous();

  //FlexIO is default mode for the camera
  //void readFrameFlexIO(void* buffer);
  void readFrameFlexIO(void *buffer, bool fUseDMA);

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

  void captureFrameStatistics();

  void setVSyncISRPriority(uint8_t priority);
  void setDMACompleteISRPriority(uint8_t priority);

  uint32_t frameCount();  //{return _dma_frame_count;}

  int16_t width(void);
  int16_t height(void);
  int16_t mode(void);

  framesize_t framesize;
  pixformat_t pixformat;
  camera_reg_settings_t settings;
  hw_config_t _hw_config;
  hw_carrier_t _hw_carrier;

private:
  ImageSensor *sensor;  /// Pointer to the camera sensor

  uint8_t *_frame_buffer_1 = nullptr;
  uint8_t *_frame_buffer_2 = nullptr;


};
#endif
