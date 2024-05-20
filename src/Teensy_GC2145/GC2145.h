

#ifndef _GC2145_H_
#define _GC2145_H_

#include <Arduino.h>
// Teensy 4.1 default to CSI pisn
#ifdef ARDUINO_TEENSY41
#define USE_CSI_PINS
// #warning "Use CSI Pins"
#endif
#include "teensy_csi_support.h"
#include <Teensy_Camera.h>
#if defined(__IMXRT1062__) // Teensy 4.x
#include <DMAChannel.h>
#include <FlexIO_t4.h>
#include <Wire.h>

// #define DEBUG_CAMERA
#define USE_VSYNC_PIN_INT

#include "Teensy_Camera.h"
#include <Arduino.h>
#include <Wire.h>

#include "arm_math.h"
// #define  DEBUG_FLEXIO
//  if not defined in the variant
#ifndef digitalPinToBitMask
#define digitalPinToBitMask(P) (1 << (digitalPinToPinName(P) % 64))
#endif

#ifndef portInputRegister
#define portInputRegister(P) ((P == 0) ? &NRF_P0->IN : &NRF_P1->IN)
#endif

#define CNT_SHIFTERS 1

#endif

// dummy defines for camera class
#define HIMAX_MODE_STREAMING 0x01 // I2C triggered streaming enable
#define HIMAX_MODE_STREAMING_NFRAMES 0x03

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

class GC2145 : public ImageSensor {
  public:
    GC2145();

    // bool begin(framesize_t resolution, int format, bool use_gpio = false); //
    // Supported FPS: 1, 5, 10, 15, 30
    bool begin_omnivision(framesize_t resolution = FRAMESIZE_QVGA,
                          pixformat_t format = RGB565, int fps = 30,
                          int camera_name = OV7670, bool use_gpio = false);
    void end();
    uint16_t getModelid();

    // must be called after Camera.begin():
    int bitsPerPixel() const;
    int bytesPerPixel() const;

    int reset();
    int sleep(int enable);
    int setPixformat(pixformat_t pixformat);
    uint8_t setFramesize(framesize_t framesize);
    uint8_t setFramesize(int w, int h);
    bool setZoomWindow(uint16_t x, uint16_t y, uint16_t w, uint16_t h);
    int setHmirror(int enable);
    int setVflip(int enable);
    int setAutoExposure(int enable, int exposure_us);
    int setAutoWhitebal(int enable, float r_gain_db, float g_gain_db,
                        float b_gain_db);
    void printRegisters(bool only_ones_set = true);
    void showRegisters();
    int setColorbar(int enable);

    /**********************************************************/
    // other camera functions - non-operational
    int setGainceiling(gainceiling_t gainceiling) { return 0; };
    // int setColorbar(int enable) {return 0; };
    int setAutoGain(int enable, float gain_db, float gain_db_ceiling) {
        return 0;
    };
    int get_vt_pix_clk(uint32_t *vt_pix_clk) { return 0; };
    int getGain_db(float *gain_db) { return 0; };
    int getCameraClock(uint32_t *vt_pix_clk) { return 0; };
    int getExposure_us(int *exposure_us) { return 0; };
    uint8_t cmdUpdate() { return 0; };
    uint8_t loadSettings(camera_reg_settings_t settings) { return 0; };
    uint8_t getAE(ae_cfg_t *psAECfg) { return 0; };
    uint8_t calAE(uint8_t CalFrames, uint8_t *Buffer, uint32_t ui32BufferLen,
                  ae_cfg_t *pAECfg) {
        return 0;
    };
    void readFrame4BitGPIO(void *buffer) {
        Serial.println("4 Bit mode not supported ..... !!!!!");
    }
    int16_t mode(void) { return 0; }
    int setFramerate(int framerate) { return 0; };
    uint8_t setMode(uint8_t Mode, uint8_t FrameCnt) { return 0; }
    void setSaturation(int saturation) {}           // 0 - 255
    void setHue(int hue) {}                         // -180 - 180
    int setBrightness(int brightness) { return 0; } // 0 - 255
    void setContrast(int contrast) {}               // 0 - 127
    void setGain(int gain) {}                       // 0 - 255
    void autoGain(int enable, float gain_db, float gain_db_ceiling) {}
    void setExposure(int exposure) {} // 0 - 65535
    void autoExposure(int enable) {}
    bool begin(framesize_t resolution, int format, bool use_gpio = false) {
        return 0;
    }
    /********************************************************************************************/
    //-------------------------------------------------------

    // quick and dirty attempt to read in images larger than can fit into one
    // region of memory...
    //  void readFrameMultiBuffer(void* buffer1, size_t size1, void* buffer2,
    // size_t size2);

    // normal Read mode
    // void readFrameGPIO(void* buffer);
    // size_t readFrameGPIO(void* buffer, size_t cb1=(uint32_t)-1, void*
    // buffer2=nullptr, size_t cb2=0);

    // FlexIO is default mode for the camera
    // void readFrameFlexIO(void* buffer, bool use_dma=true);
    // The code is in the base class.
    // bool readFrameFlexIO(void *buffer, size_t cb1, void* buffer2=nullptr,
    // size_t cb2=0); void readFrameMultiBufferFlexIO(void* buffer1, size_t
    // size1, void* buffer2, size_t size2); bool startReadFlexIO(bool
    // (*callback)(void *frame_buffer), void *fb1, size_t cb1, void *fb2, size_t
    // cb2); bool stopReadFlexIO();

    // Lets try a dma version.  Doing one DMA that is synchronous does not gain
    // anything So lets have a start, stop... Have it allocate 2 frame buffers
    // and it's own DMA buffers, with the option of setting your own buffers if
    // desired.
    bool startReadFrameDMA(bool (*callback)(void *frame_buffer) = nullptr,
                           uint8_t *fb1 = nullptr, uint8_t *fb2 = nullptr);
    void changeFrameBuffer(uint8_t *fbFrom, uint8_t *fbTo) {
        if (_frame_buffer_1 == fbFrom)
            _frame_buffer_1 = fbTo;
        else if (_frame_buffer_2 == fbFrom)
            _frame_buffer_2 = fbTo;
    }
    bool stopReadFrameDMA();
    inline uint32_t frameCount() { return _dma_frame_count; }
    inline void *frameBuffer() { return _dma_last_completed_frame; }
    void captureFrameStatistics();

    void setVSyncISRPriority(uint8_t priority) {
        NVIC_SET_PRIORITY(IRQ_GPIO6789, priority);
    }
    void setDMACompleteISRPriority(uint8_t priority) {
        NVIC_SET_PRIORITY(_dmachannel.channel & 0xf, priority);
    }
    /********************************************************************************************/

    // TBD Allow user to set all of the buffers...
    void readFrameDMA(void *buffer);

  private:
    uint8_t cameraReadRegister(uint8_t reg);
    uint8_t cameraWriteRegister(uint8_t reg, uint8_t data);

    int setWindow(uint16_t reg, uint16_t x, uint16_t y, uint16_t w, uint16_t h);
    int getWindow(uint16_t reg, uint16_t &x, uint16_t &y, uint16_t &w,
                  uint16_t &h);

  private:
    void *_GC2145;

    uint32_t _xclk_freq = 12000000;

    // DMA STUFF
    enum {
        DMABUFFER_SIZE = 1296
    }; // 640x480  so 640*2*2
       //   static DMAChannel _dmachannel;
    //  static DMASetting _dmasettings[10];  // maybe handle up to 800x600
    static uint32_t _dmaBuffer1[DMABUFFER_SIZE];
    static uint32_t _dmaBuffer2[DMABUFFER_SIZE];

    //  bool (*_callback)(void *frame_buffer) = nullptr ;
    //  uint32_t  _dma_frame_count;
    //  uint8_t *_dma_last_completed_frame;
    // TBD Allow user to set all of the buffers...

#if defined(ARDUINO_TEENSY_MICROMOD)
    uint32_t _save_IOMUXC_GPR_GPR27;
#else
    uint32_t _save_IOMUXC_GPR_GPR26;
#endif
    uint32_t _save_pclkPin_portConfigRegister;

    uint32_t _bytes_left_dma;
    uint16_t _save_lsb;
    uint16_t _frame_col_index;             // which column we are in a row
    uint16_t _frame_row_index;             // which row
    const uint16_t _frame_ignore_cols = 0; // how many cols to ignore per row
    uint8_t *_frame_buffer_1 = nullptr;
    size_t _frame_buffer_1_size = 0;
    uint8_t *_frame_buffer_2 = nullptr;
    size_t _frame_buffer_2_size = 0;
    uint8_t *_frame_buffer_pointer;
    uint8_t *_frame_row_buffer_pointer; // start of the row
    uint8_t _dma_index;
    volatile bool _dma_active;
    volatile uint32_t _vsync_high_time = 0;
    enum {
        DMASTATE_INITIAL = 0,
        DMASTATE_RUNNING,
        DMASTATE_STOP_REQUESTED,
        DMA_STATE_STOPPED,
        DMA_STATE_ONE_FRAME
    };
    volatile uint8_t _dma_state;
    static void dmaInterrupt();
    void processDMAInterrupt();
#if 0
    static void frameStartInterrupt();
    void processFrameStartInterrupt();
#endif

    inline int fast_floorf(float x) {
        int i;
        asm volatile("vcvt.S32.f32  %[r], %[x]\n" : [r] "=t"(i) : [x] "t"(x));
        return i;
    }
};

#endif