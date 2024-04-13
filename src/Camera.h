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
                       uint8_t g4 = 0xff, uint8_t g5 = 0xff, uint8_t g6 = 0xff, uint8_t g7 = 0xff, TwoWire &wire = Wire);
  virtual bool begin(framesize_t framesize = FRAMESIZE_QVGA, int framerate = 30, bool use_gpio = false) = 0;
  virtual void end() = 0;
  virtual int reset() = 0;
  virtual void showRegisters(void) = 0;
  virtual void debug(bool debug_on) {_debug = debug_on;}
  virtual bool debug() {return _debug;}
  bool usingGPIO() {return _use_gpio;}
  // debug and experimenting support
  virtual uint8_t readRegister(uint8_t reg) {return (uint8_t)-1;}
  virtual bool writeRegister(uint8_t reg, uint8_t data) {return false;}
  virtual int setPixformat(pixformat_t pfmt) = 0;
  virtual uint8_t setFramesize(framesize_t framesize) = 0;
  virtual uint8_t setFramesize(int w, int h) {return 0;}  // some cameras don't support
  virtual bool setZoomWindow(uint16_t x, uint16_t y, uint16_t w, uint16_t h) {return false;}

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
  virtual void captureFrameStatistics() = 0;
  
  virtual bool begin_omnivision(framesize_t resolution = FRAMESIZE_QVGA, pixformat_t format = RGB565, int fps = 30, int camera_name = OV7670, bool use_gpio = false); // Supported FPS: 1, 5, 10, 15, 30
  virtual void setSaturation(int saturation) = 0; // 0 - 255
  virtual void setHue(int hue) = 0; // -180 - 180
  virtual void setContrast(int contrast) = 0; // 0 - 127
  virtual void setGain(int gain) = 0; // 0 - 255
  virtual void autoGain(int enable, float gain_db, float gain_db_ceiling) = 0;
  virtual void setExposure(int exposure) = 0; // 0 - 65535
  virtual void autoExposure(int enable) = 0;
  
  virtual int setAutoWhitebal(int enable, float r_gain_db, float g_gain_db, float b_gain_db);

  
  // grab Frame functions
  //-------------------------------------------------------
  //Generic Read Frame base on _hw_config
  virtual bool readFrame(void *buffer1, size_t cb1, void *buffer2=nullptr, size_t cb2=0); // give default one for now

  virtual void useDMA(bool f) {_fuse_dma = f;}
  virtual bool useDMA() {return _fuse_dma; }

  //normal Read mode
  virtual bool readFrameGPIO(void* buffer, size_t cb1=(uint32_t)-1, void* buffer2=nullptr, size_t cb2=0) = 0;
  virtual void readFrame4BitGPIO(void *buffer) = 0;

  // Have default implementations that simply call off to flexio or GPIO...
  virtual bool readContinuous(bool (*callback)(void *frame_buffer), void *fb1, size_t cb1, void *fb2, size_t cb2);
  virtual void stopReadContinuous();

  //FlexIO is default mode for the camera
  //virtual void readFrameFlexIO(void* buffer);
  virtual bool readFrameFlexIO(void *buffer, size_t cb1=(uint32_t)-1, void* buffer2=nullptr, size_t cb2=0);

  virtual bool startReadFlexIO(bool (*callback)(void *frame_buffer), void *fb1, size_t cb1, void *fb2, size_t cb2);
  virtual bool stopReadFlexIO();

  // Lets try a dma version.  Doing one DMA that is synchronous does not gain anything
  // So lets have a start, stop... Have it allocate 2 frame buffers and it's own DMA
  // buffers, with the option of setting your own buffers if desired.
  virtual bool startReadFrameDMA(bool (*callback)(void *frame_buffer) = nullptr, uint8_t *fb1 = nullptr, uint8_t *fb2 = nullptr) = 0;

  virtual void changeFrameBuffer(uint8_t *fbFrom, uint8_t *fbTo) = 0;

  virtual bool stopReadFrameDMA() = 0;

  virtual void setVSyncISRPriority(uint8_t priority) = 0;
  virtual void setDMACompleteISRPriority(uint8_t priority) = 0;

  virtual uint32_t frameCount() = 0;  //{return _dma_frame_count;}

  // Set and retrieve read timeout
  uint32_t timeout() { return _timeout;}
  void timeout(uint32_t timeout_ms) {_timeout = timeout_ms;}

  // The width and height are the sizes of the data returned when you do a frameRead.
  // initialially it is the size of the resolution that was passed into setFramesize
  // however if you set a zoom window this is the size of that window. 
  virtual int16_t width(void) {return _width;}
  virtual int16_t height(void) {return _height;}
  // Save of the frame in width and height set by setFramesize
  virtual int16_t frameWidth(void) {return _frame_width;}
  virtual int16_t frameHeight(void) {return _frame_height;}
  virtual int16_t mode(void) = 0;

  // See which of these are used by which cameras. 
  framesize_t framesize;
  pixformat_t pixformat;
  camera_reg_settings_t settings;
  hw_config_t _hw_config;
  hw_carrier_t _hw_carrier;


  // FlexIO transfer variables and methods:
  static void dmaInterruptFlexIO();
  virtual void processDMAInterruptFlexIO();
  static void frameStartInterruptFlexIO();
  virtual void processFrameStartInterruptFlexIO();
  virtual void processDMAInterrupt() {}
  virtual bool flexio_configure();
  virtual void processFrameStartInterrupt() {};
  virtual bool supports4BitMode() {return false;}

  void dumpDMA_TCD(DMABaseClass *dmabc, const char *psz_title);

protected:
  bool _debug = true;     // Should the camera code print out debug information?
  bool _fuse_dma = true;  // in some cameras should we use DMA or do the Io directly
  bool _use_gpio = false; // set in the begin of some cameras
  uint32_t _timeout = 2000; // timeout in ms for a read

  int _vsyncPin;
  int _hrefPin;
  int _pclkPin;
  int _xclkPin;
  int _rst;
  int _dPins[8];


  int16_t _width;
  int16_t _height;
  int16_t _frame_width;
  int16_t _frame_height;
  int _bytesPerPixel;
  int _format;

  TwoWire *_wire;

  
  static DMAChannel _dmachannel;
  static DMASetting _dmasettings[10];  // For now lets have enough for two full size buffers...
  volatile bool _dma_active = false;
  uint8_t *_frame_buffer_1 = nullptr;
  size_t  _frame_buffer_1_size = 0;
  uint8_t *_frame_buffer_2 = nullptr;
  size_t  _frame_buffer_2_size = 0;
  FlexIOHandler *_pflex;
  IMXRT_FLEXIO_t *_pflexio;
  uint8_t _fshifter;
  uint8_t _fshifter_mask;
  uint8_t _ftimer;
  uint8_t _dma_source;

  volatile uint32_t* _vsyncPort;
  uint32_t _vsyncMask;
  volatile uint32_t* _hrefPort;
  uint32_t _hrefMask;
  volatile uint32_t* _pclkPort;
  uint32_t _pclkMask;

  bool (*_callback)(void *frame_buffer) = nullptr ;
  uint32_t  _dma_frame_count;
  uint8_t *_dma_last_completed_frame;


  enum {DMASTATE_INITIAL=0, DMASTATE_RUNNING, DMASTATE_STOP_REQUESTED, DMA_STATE_STOPPED, DMA_STATE_ONE_FRAME};
  volatile uint8_t _dma_state = DMASTATE_INITIAL;
  static ImageSensor *active_dma_camera;


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
  void debug(bool debug_on);
  bool debug();
  bool usingGPIO(); // Is the camera configured to use GPIO instead of flexio?
  int setPixformat(pixformat_t pfmt);
  uint8_t setFramesize(framesize_t framesize);
  uint8_t setFramesize(int w, int h);
  bool setZoomWindow(uint16_t x=-1, uint16_t y=-1, uint16_t w=-1, uint16_t h=-1);
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

  // debug and experimenting support
  uint8_t readRegister(uint8_t reg);
  bool writeRegister(uint8_t reg, uint8_t data);

  
  /***********  OV specific ************************/
  bool begin(framesize_t resolution = FRAMESIZE_QVGA, pixformat_t format = RGB565, int fps = 30, int camera_name = OV7670, bool use_gpio = false); // Supported FPS: 1, 5, 10, 15, 30
  void setSaturation(int saturation); // 0 - 255
  void setHue(int hue); // -180 - 180
  void setContrast(int contrast); // 0 - 127
  void setGain(int gain); // 0 - 255
  void autoGain(int enable, float gain_db, float gain_db_ceiling);
  void setExposure(int exposure); // 0 - 65535
  void autoExposure(int enable);
  /***********  GC2145 specific ************************/
  int setAutoWhitebal(int enable, float r_gain_db, float g_gain_db, float b_gain_db);

  // grab Frame functions
  //-------------------------------------------------------
  //Generic Read Frame base on _hw_config
  bool readFrame(void *buffer1, size_t cb1, void *buffer2 = nullptr, size_t cb2=0);

  // enable/disable DMA
  void useDMA(bool f);
  bool useDMA();

  //normal Read mode
  bool readFrameGPIO(void* buffer, size_t cb1=(uint32_t)-1, void* buffer2=nullptr, size_t cb2=0);
  void readFrame4BitGPIO(void *buffer);

  bool readContinuous(bool (*callback)(void *frame_buffer), void *fb1, size_t cb1, void *fb2, size_t cb2);
  void stopReadContinuous();

  //FlexIO is default mode for the camera
  //void readFrameFlexIO(void* buffer);
  bool readFrameFlexIO(void *buffer, size_t cb1=(uint32_t)-1, void* buffer2=nullptr, size_t cb2=0);

  bool startReadFlexIO(bool (*callback)(void *frame_buffer), void *fb1, size_t cb1, void *fb2, size_t cb2);
  bool stopReadFlexIO();

  // Lets try a dma version.  Doing one DMA that is synchronous does not gain anything
  // So lets have a start, stop... Have it allocate 2 frame buffers and it's own DMA
  // buffers, with the option of setting your own buffers if desired.
  bool startReadFrameDMA(bool (*callback)(void *frame_buffer) = nullptr, uint8_t *fb1 = nullptr, uint8_t *fb2 = nullptr);

  void changeFrameBuffer(uint8_t *fbFrom, uint8_t *fbTo);

  bool stopReadFrameDMA();

  void captureFrameStatistics();

  void setVSyncISRPriority(uint8_t priority);
  void setDMACompleteISRPriority(uint8_t priority);

  uint32_t frameCount();  //{return _dma_frame_count;}

  int16_t width(void);
  int16_t height(void);
  int16_t frameWidth(void);
  int16_t frameHeight(void);
  int16_t mode(void);

  // set and retrieve read timeout
  uint32_t timeout();
  void timeout(uint32_t timeout_ms);


private:
  ImageSensor *sensor;  /// Pointer to the camera sensor


};
#endif
