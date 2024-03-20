

#ifndef _GC2145_H_
#define _GC2145_H_

#include <Arduino.h>
#if defined(__IMXRT1062__)  // Teensy 4.x
#include <DMAChannel.h>
#include <Wire.h>
#include <FlexIO_t4.h>


//#define DEBUG_CAMERA
#define USE_VSYNC_PIN_INT

#include <Arduino.h>
#include <Wire.h>
#include "Camera.h"

#include "arm_math.h"
//#define  DEBUG_FLEXIO
// if not defined in the variant
#ifndef digitalPinToBitMask
#define digitalPinToBitMask(P) (1 << (digitalPinToPinName(P) % 64))
#endif

#ifndef portInputRegister
#define portInputRegister(P) ((P == 0) ? &NRF_P0->IN : &NRF_P1->IN)
#endif

#define CNT_SHIFTERS 1


#ifdef ARDUINO_TEENSY_MICROMOD
/*
HM01B0 pin      pin#    NXP     Usage
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

#define GC2145_PLK   8    //8       B1_00   FlexIO2:16
#define GC2145_XCLK  7    //7       B1_01   PWM
#define GC2145_HREF  32   //32      B0_12   FlexIO2:12, pin 46 on sdram board
#define GC2145_VSYNC 33   //33      EMC_07  GPIO, 21 pon sdram board
#define GC2145_RST   17  // reset pin 

#define GC2145_D0    40   //40      B0_04   FlexIO2:4
#define GC2145_D1    41   //41      B0_05   FlexIO2:5
#define GC2145_D2    42   //42      B0_06   FlexIO2:6
#define GC2145_D3    43   //43      B0_07   FlexIO2:7
#define GC2145_D4    44   //44      B0_08   FlexIO2:8  - probably not needed, use 4 bit mode
#define GC2145_D5    45   //45      B0_09   FlexIO2:9  - probably not needed, use 4 bit mode
#define GC2145_D6    6    //6       B0_10   FlexIO2:10 - probably not needed, use 4 bit mode
#define GC2145_D7    9    //9       B0_11   FlexIO2:11 - probably not needed, use 4 bit mode

#elif defined USE_CSI_PINS
#define GC2145_PLK   40 //40 // AD_B1_04 CSI_PIXCLK
#define GC2145_XCLK_JUMPER 41 // BUGBUG CSI 41 is NOT a PWM pin so we jumper to it...
#define GC2145_XCLK  37  //41 // AD_B1_05 CSI_MCLK
#define GC2145_HREF  16 // AD_B1_07 CSI_HSYNC
#define GC2145_VSYNC 17 // AD_B1_06 CSI_VSYNC

#define GC2145_D0    27 // AD_B1_15 CSI_D2
#define GC2145_D1    26 // AD_B1_14 CSI_D3
#define GC2145_D2    39 // AD_B1_13 CSI_D4
#define GC2145_D3    38 // AD_B1_12 CSI_D5
#define GC2145_D4    21 // AD_B1_11 CSI_D6
#define GC2145_D5    20 // AD_B1_10 CSI_D7
#define GC2145_D6    23 // AD_B1_09 CSI_D8
#define GC2145_D7    22 // AD_B1_08 CSI_D9
#elif 1
#define GC2145_PLK   4 //40 // AD_B1_04 CSI_PIXCLK
#define GC2145_XCLK  5  //41 // AD_B1_05 CSI_MCLK
#define GC2145_HREF  40 // AD_B1_07 CSI_HSYNC
#define GC2145_VSYNC 41 // AD_B1_06 CSI_VSYNC

#define GC2145_D0    27 // AD_B1_02 1.18
#define GC2145_D1    15 // AD_B1_03 1.19
#define GC2145_D2    17 // AD_B1_06 1.22
#define GC2145_D3    16 // AD_B1_07 1.23
#define GC2145_D4    22 // AD_B1_08 1.24
#define GC2145_D5    23 // AD_B1_09 1.25
#define GC2145_D6    20 // AD_B1_10 1.26
#define GC2145_D7    21 // AD_B1_11 1.27

#else
// For T4.1 can choose same or could choose a contiguous set of pins only one shift required.
// Like:  Note was going to try GPI pins 1.24-21 but save SPI1 pins 26,27 as no ...
#define GC2145_PLK   4
#define GC2145_XCLK  5
#define GC2145_HREF  40 // AD_B1_04 1.20 T4.1...
#define GC2145_VSYNC 41 // AD_B1_05 1.21 T4.1...

#define GC2145_D0    17 // AD_B1_06 1.22
#define GC2145_D1    16 // AD_B1_07 1.23
#define GC2145_D2    22 // AD_B1_08 1.24
#define GC2145_D3    23 // AD_B1_09 1.25
#define GC2145_D4    20 // AD_B1_10 1.26
#define GC2145_D5    21 // AD_B1_11 1.27
#define GC2145_D6    38 // AD_B1_12 1.28
#define GC2145_D7    39 // AD_B1_13 1.29
#endif
//      #define GC2145_D6    26 // AD_B1_14 1.30
//      #define GC2145_D7    27 // AD_B1_15 1.31
#endif

// dummy defines for camera class
#define         HIMAX_MODE_STREAMING            0x01     // I2C triggered streaming enable
#define         HIMAX_MODE_STREAMING_NFRAMES    0x03     


enum {
	GC2145_TEST_PATTERN_DISABLED,
	GC2145_TEST_PATTERN_VGA_COLOR_BARS,
	GC2145_TEST_PATTERN_UXGA_COLOR_BARS,
	GC2145_TEST_PATTERN_SKIN_MAP,
	GC2145_TEST_PATTERN_SOLID_COLOR,
};

enum {
    GC2145_DISABLED = 0,
    GC2145_COLOR_BARS,
    GC2145_UXGA_COLOR_BARS,
    GC2145_SKIN_MAP,
    GC2145_SOLID_BLACK,
    GC2145_SOLID_LIGHT_GRAY,
    GC2145_SOLID_GRAY,
    GC2145_SOLID_DARK_GRAY,
    GC2145_SOLID_WHITE,
    GC2145_SOLID_RED,
    GC2145_SOLID_GREEN,
    GC2145_SOLID_BLUE,
    GC2145_SOLID_YELLOW,
    GC2145_SOLID_CYAN,
    GC2145_SOLID_MAGENTA,
};

/*
typedef enum {
    FRAMESIZE_INVALID = 0,
    // C/SIF Resolutions
    FRAMESIZE_QQCIF,    // 88x72  0
    FRAMESIZE_QCIF,     // 176x144  1
    FRAMESIZE_CIF,      // 352x288  2
    FRAMESIZE_QQSIF,    // 88x60  3
    FRAMESIZE_QSIF,     // 176x120  4
    FRAMESIZE_SIF,      // 352x240  5
    // VGA Resolutions
    FRAMESIZE_QQQQVGA,  // 40x30
    FRAMESIZE_QQQVGA,   // 80x60
    FRAMESIZE_QQVGA,    // 160x120  8
    FRAMESIZE_QVGA,     // 320x240  9
    FRAMESIZE_VGA,      // 640x480 11
    FRAMESIZE_HQQQQVGA, // 30x20
    FRAMESIZE_HQQQVGA,  // 60x40
    FRAMESIZE_HQQVGA,   // 120x80
    FRAMESIZE_HQVGA,    // 240x160
    FRAMESIZE_HVGA,     // 480x320
    // FFT Resolutions
    FRAMESIZE_64X32,    // 64x32
    FRAMESIZE_64X64,    // 64x64
    FRAMESIZE_128X64,   // 128x64
    FRAMESIZE_128X128,  // 128x128
    // Himax Resolutions
    FRAMESIZE_160X160,  // 160x160
    FRAMESIZE_320X320,  // 320x320
    // Other
    FRAMESIZE_LCD,      // 128x160
    FRAMESIZE_QQVGA2,   // 128x160
    FRAMESIZE_WVGA,     // 720x480
    FRAMESIZE_WVGA2,    // 752x480
    FRAMESIZE_SVGA,     // 800x600
    FRAMESIZE_XGA,      // 1024x768
    FRAMESIZE_WXGA,     // 1280x768
    FRAMESIZE_SXGA,     // 1280x1024
    FRAMESIZE_SXGAM,    // 1280x960
    FRAMESIZE_UXGA,     // 1600x1200
    FRAMESIZE_HD,       // 1280x720
    FRAMESIZE_FHD,      // 1920x1080
    FRAMESIZE_QHD,      // 2560x1440
    FRAMESIZE_QXGA,     // 2048x1536
    FRAMESIZE_WQXGA,    // 2560x1600
    FRAMESIZE_WQXGA2,   // 2592x1944
} framesize_t;

enum
{
  YUV422 = 0,
  RGB565,
  BAYER,
  GRAYSCALE
};
*/

class GC2145 : public ImageSensor
{
public:
  GC2145();

  void setPins(uint8_t mclk_pin, uint8_t pclk_pin, uint8_t vsync_pin, uint8_t hsync_pin, uint8_t en_pin,
                     uint8_t g0, uint8_t g1, uint8_t g2, uint8_t g3, uint8_t g4, uint8_t g5, uint8_t g6, uint8_t g7, TwoWire &wire);

  //bool begin(framesize_t resolution, int format, bool use_gpio = false); // Supported FPS: 1, 5, 10, 15, 30
  bool begin_omnivision(framesize_t resolution = FRAMESIZE_QVGA, pixformat_t format = RGB565, int fps = 30, int camera_name = OV7670, bool use_gpio = false); 
  void end();
  uint16_t getModelid();

  // must be called after Camera.begin():
  int16_t width();
  int16_t height();
  int bitsPerPixel() const;
  int bytesPerPixel() const;
  
  int reset();
  int sleep(int enable);
  int setPixelFormat(pixformat_t pixformat);
  uint8_t setFramesize(framesize_t framesize);
  uint8_t setFramesize(int w, int h);
  int setHmirror(int enable);
  int setVflip(int enable);
  int setAutoExposure(int enable, int exposure_us);
  int setAutoWhitebal(int enable, float r_gain_db, float g_gain_db, float b_gain_db);
  void printRegisters(bool only_ones_set = true);
  void debug(bool debug_on) {_debug = debug_on;}
  bool debug() {return _debug;}
  void showRegisters();
  int setColorbar(int enable);
  
  
  /**********************************************************/
    //other camera functions - non-operational
  int setGainceiling(gainceiling_t gainceiling) {return 0; };
  //int setColorbar(int enable) {return 0; };
  int setAutoGain(int enable, float gain_db, float gain_db_ceiling) {return 0; };
  int get_vt_pix_clk(uint32_t *vt_pix_clk) {return 0; };
  int getGain_db(float *gain_db) {return 0; };
  int getCameraClock(uint32_t *vt_pix_clk) {return 0; };
  int getExposure_us(int *exposure_us) {return 0; };
  uint8_t cmdUpdate() {return 0; };
  uint8_t loadSettings(camera_reg_settings_t settings) {return 0; };
  uint8_t getAE( ae_cfg_t *psAECfg) {return 0; };
  uint8_t calAE( uint8_t CalFrames, uint8_t* Buffer, uint32_t ui32BufferLen, ae_cfg_t* pAECfg) {return 0; };
  void readFrame4BitGPIO(void* buffer) {Serial.println("4 Bit mode not supported ..... !!!!!"); }
  int16_t mode(void) { return 0; }
  int setPixformat( pixformat_t pfmt) {return 0; };
  int setFramerate(int framerate) {return 0; };
  uint8_t setMode(uint8_t Mode, uint8_t FrameCnt) { return 0; }
  void setSaturation(int saturation) {  } // 0 - 255
  void setHue(int hue) {  } // -180 - 180
  int setBrightness(int brightness) { return 0; } // 0 - 255
  void setContrast(int contrast) { } // 0 - 127
  void setGain(int gain) {  } // 0 - 255
  void autoGain(int enable, float gain_db, float gain_db_ceiling) {  }
  void setExposure(int exposure) {  } // 0 - 65535
  void autoExposure(int enable) {}
  bool begin(framesize_t resolution, int format, bool use_gpio = false){return 0;}
/********************************************************************************************/
	//-------------------------------------------------------
	//Generic Read Frame base on _hw_config
  bool readFrame(void *buffer1, size_t cb1, void* buffer2=nullptr, size_t cb2=0); // give default one for now

	// quick and dirty attempt to read in images larger than can fit into one region of memory...
//	void readFrameMultiBuffer(void* buffer1, size_t size1, void* buffer2, size_t size2);

  void useDMA(bool f) {_fuse_dma = f;}
  bool useDMA() {return _fuse_dma; }

	
	//normal Read mode
	//void readFrameGPIO(void* buffer);
  bool readFrameGPIO(void* buffer, size_t cb1=(uint32_t)-1, void* buffer2=nullptr, size_t cb2=0);

	bool readContinuous(bool(*callback)(void *frame_buffer), void *fb1, void *fb2);
	void stopReadContinuous();

	//FlexIO is default mode for the camera
	//void readFrameFlexIO(void* buffer, bool use_dma=true);
  bool readFrameFlexIO(void *buffer, size_t cb1, void* buffer2=nullptr, size_t cb2=0);
	void readFrameMultiBufferFlexIO(void* buffer1, size_t size1, void* buffer2, size_t size2);
	bool startReadFlexIO(bool (*callback)(void *frame_buffer), void *fb1, void *fb2);
	bool stopReadFlexIO();

	// Lets try a dma version.  Doing one DMA that is synchronous does not gain anything
	// So lets have a start, stop... Have it allocate 2 frame buffers and it's own DMA 
	// buffers, with the option of setting your own buffers if desired.
	bool startReadFrameDMA(bool (*callback)(void *frame_buffer)=nullptr, uint8_t *fb1=nullptr, uint8_t *fb2=nullptr);
	void changeFrameBuffer(uint8_t *fbFrom, uint8_t *fbTo) {
		if (_frame_buffer_1 == fbFrom) _frame_buffer_1 = fbTo;
		else if (_frame_buffer_2 == fbFrom) _frame_buffer_2 = fbTo;
	}
	bool stopReadFrameDMA();	
	inline uint32_t frameCount() {return _dma_frame_count;}
	inline void *frameBuffer() {return _dma_last_completed_frame;}
	void captureFrameStatistics();
	
	void setVSyncISRPriority(uint8_t priority) {NVIC_SET_PRIORITY(IRQ_GPIO6789, priority); }
	void setDMACompleteISRPriority(uint8_t priority) {NVIC_SET_PRIORITY(_dmachannel.channel & 0xf, priority); }
    /********************************************************************************************/

  // TBD Allow user to set all of the buffers...
  void readFrameDMA(void* buffer);
    
   
private:
  void beginXClk();
  void endXClk();

  uint8_t cameraReadRegister(uint8_t reg);
  uint8_t cameraWriteRegister(uint8_t reg, uint8_t data);

  int setWindow(uint16_t reg, uint16_t x, uint16_t y, uint16_t w, uint16_t h);

private:
    int _vsyncPin;
    int _hrefPin;
    int _pclkPin;
    int _xclkPin;
    int _rst;
    int _dPins[8];

    bool _use_gpio = false;
    bool _debug = true;
    bool _fuse_dma = true;
	  TwoWire *_wire;

    int _width;
    int _height;
    int _bytesPerPixel;
    bool _grayscale;
    
    void* _GC2145;

    volatile uint32_t* _vsyncPort;
    uint32_t _vsyncMask;
    volatile uint32_t* _hrefPort;
    uint32_t _hrefMask;
    volatile uint32_t* _pclkPort;
    uint32_t _pclkMask;

    uint32_t _xclk_freq	= 12000000;
    

	bool flexio_configure();

	// DMA STUFF
	enum {DMABUFFER_SIZE=1296};  // 640x480  so 640*2*2
	static DMAChannel _dmachannel;
	static DMASetting _dmasettings[8];  // maybe handle up to 800x600
	static uint32_t _dmaBuffer1[DMABUFFER_SIZE];
	static uint32_t _dmaBuffer2[DMABUFFER_SIZE];

	bool (*_callback)(void *frame_buffer) = nullptr ;
	uint32_t  _dma_frame_count;
	uint8_t *_dma_last_completed_frame;
	// TBD Allow user to set all of the buffers...


	// Added settings for configurable flexio
	FlexIOHandler *_pflex;
    IMXRT_FLEXIO_t *_pflexio;
	uint8_t _fshifter;
	uint8_t _fshifter_mask;
    uint8_t _ftimer;
    uint8_t _dma_source;


	#if defined (ARDUINO_TEENSY_MICROMOD)
	uint32_t _save_IOMUXC_GPR_GPR27;
	#else
	uint32_t _save_IOMUXC_GPR_GPR26;
	#endif      
	uint32_t _save_pclkPin_portConfigRegister;

	uint32_t _bytes_left_dma;
	uint16_t  _save_lsb;
	uint16_t  _frame_col_index;  // which column we are in a row
	uint16_t  _frame_row_index;  // which row
	const uint16_t  _frame_ignore_cols = 0; // how many cols to ignore per row
	uint8_t *_frame_buffer_1 = nullptr;
	uint8_t *_frame_buffer_2 = nullptr;
	uint8_t *_frame_buffer_pointer;
	uint8_t *_frame_row_buffer_pointer; // start of the row
	uint8_t _dma_index;
	volatile bool	_dma_active;
	volatile uint32_t _vsync_high_time = 0;
	enum {DMASTATE_INITIAL=0, DMASTATE_RUNNING, DMASTATE_STOP_REQUESTED, DMA_STATE_STOPPED, DMA_STATE_ONE_FRAME};
	volatile uint8_t _dma_state;
	static void dmaInterrupt(); 
	void processDMAInterrupt();
#if 0
	static void frameStartInterrupt();
	void processFrameStartInterrupt();
#endif	
	static void dmaInterruptFlexIO();
	void processDMAInterruptFlexIO();
	static void frameStartInterruptFlexIO();
	void processFrameStartInterruptFlexIO();
	static GC2145 *active_dma_camera;

	inline int fast_floorf(float x)
	{
		int i;
		asm volatile (
				"vcvt.S32.f32  %[r], %[x]\n"
				: [r] "=t" (i)
				: [x] "t"  (x));
		return i;
	}


};

#endif