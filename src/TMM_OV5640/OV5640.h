
#ifndef _OV5640_H_
#define _OV5640_H_

#include "OV5640_regs.h"
#include <Arduino.h>
#include <Camera.h>

#if defined(__IMXRT1062__) // Teensy 4.x
#include <DMAChannel.h>
#include <FlexIO_t4.h>
#include <Wire.h>

// #define OV5640_VSYNC 2    // Lets setup for T4.1 CSI pins
// #define USE_CSI_PINS

// #define OV5640_USE_DEBUG_PINS
#ifdef OV5640_USE_DEBUG_PINS
#define OV5640_DEBUG_PIN_1 14
#define OV5640_DEBUG_PIN_2 15
#define OV5640_DEBUG_PIN_3 3
#define DebugDigitalWrite(pin, val) digitalWriteFast(pin, val)
#define DebugDigitalToggle(pin) digitalToggleFast(pin)
#else
#define DebugDigitalWrite(pin, val)
#define DebugDigitalToggle(pin)
#endif

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

#define OV5640_PLK 8    // 8       B1_00   FlexIO2:16
#define OV5640_XCLK 7   // 7       B1_01   PWM
#define OV5640_HREF 32  // 32      B0_12   FlexIO2:12, pin 46 on sdram board
#define OV5640_VSYNC 33 // 33      EMC_07  GPIO, 21 pon sdram board
#define OV5640_RST 17   // reset pin

#define OV5640_D0 40 // 40      B0_04   FlexIO2:4
#define OV5640_D1 41 // 41      B0_05   FlexIO2:5
#define OV5640_D2 42 // 42      B0_06   FlexIO2:6
#define OV5640_D3 43 // 43      B0_07   FlexIO2:7
#define OV5640_D4 44 // 44      B0_08   FlexIO2:8
#define OV5640_D5 45 // 45      B0_09   FlexIO2:9
#define OV5640_D6 6  // 6       B0_10   FlexIO2:10
#define OV5640_D7 9  // 9       B0_11   FlexIO2:11

#elif defined USE_CSI_PINS
#define OV5640_PLK 40 // 40 // AD_B1_04 CSI_PIXCLK
#define OV5640_XCLK_JUMPER \
    41                  // BUGBUG CSI 41 is NOT a PWM pin so we jumper to it...
#define OV5640_XCLK 37  // 41 // AD_B1_05 CSI_MCLK
#define OV5640_HREF 16  // AD_B1_07 CSI_HSYNC
#define OV5640_VSYNC 17 // AD_B1_06 CSI_VSYNC

#define OV5640_D0 27 // AD_B1_15 CSI_D2
#define OV5640_D1 26 // AD_B1_14 CSI_D3
#define OV5640_D2 39 // AD_B1_13 CSI_D4
#define OV5640_D3 38 // AD_B1_12 CSI_D5
#define OV5640_D4 21 // AD_B1_11 CSI_D6
#define OV5640_D5 20 // AD_B1_10 CSI_D7
#define OV5640_D6 23 // AD_B1_09 CSI_D8
#define OV5640_D7 22 // AD_B1_08 CSI_D9
#elif 1
#define OV5640_PLK 4    // 40 // AD_B1_04 CSI_PIXCLK
#define OV5640_XCLK 5   // 41 // AD_B1_05 CSI_MCLK
#define OV5640_HREF 40  // AD_B1_07 CSI_HSYNC
#define OV5640_VSYNC 41 // AD_B1_06 CSI_VSYNC

#define OV5640_D0 27 // AD_B1_02 1.18
#define OV5640_D1 15 // AD_B1_03 1.19
#define OV5640_D2 17 // AD_B1_06 1.22
#define OV5640_D3 16 // AD_B1_07 1.23
#define OV5640_D4 22 // AD_B1_08 1.24
#define OV5640_D5 23 // AD_B1_09 1.25
#define OV5640_D6 20 // AD_B1_10 1.26
#define OV5640_D7 21 // AD_B1_11 1.27

#else
// For T4.1 can choose same or could choose a contiguous set of pins only one
// shift required. Like:  Note was going to try GPI pins 1.24-21 but save SPI1
// pins 26,27 as no ...
#define OV5640_PLK 4
#define OV5640_XCLK 5
#define OV5640_HREF 40  // AD_B1_04 1.20 T4.1...
#define OV5640_VSYNC 41 // AD_B1_05 1.21 T4.1...

#define OV5640_D0 17 // AD_B1_06 1.22
#define OV5640_D1 16 // AD_B1_07 1.23
#define OV5640_D2 22 // AD_B1_08 1.24
#define OV5640_D3 23 // AD_B1_09 1.25
#define OV5640_D4 20 // AD_B1_10 1.26
#define OV5640_D5 21 // AD_B1_11 1.27
#define OV5640_D6 38 // AD_B1_12 1.28
#define OV5640_D7 39 // AD_B1_13 1.29
#endif
//      #define OV5640_D6    26 // AD_B1_14 1.30
//      #define OV5640_D7    27 // AD_B1_15 1.31

#endif

// dummy defines for camera class
#define HIMAX_MODE_STREAMING 0x01         // I2C triggered streaming enable
#define HIMAX_MODE_STREAMING_NFRAMES 0x03 // Output N frames

class OV5640 : public ImageSensor {
  public:
    OV5640();

    /********************************************************************************************/
    //-------------------------------------------------------
    // normal Read mode
    // size_t readFrameGPIO(void* buffer, size_t cb1=(uint32_t)-1, void*
    // buffer2=nullptr, size_t cb2=0);
    size_t readFrameGPIO_JPEG(void *buffer, size_t cb1 = (uint32_t)-1,
                              void *buffer2 = nullptr, size_t cb2 = 0);

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

    void end();

    int bitsPerPixel() { return 16; }
    int bytesPerPixel() { return 2; }

    bool begin_omnivision(framesize_t resolution, pixformat_t format, int fps,
                          int camera_name, bool use_gpio);

    int reset();
    uint16_t getModelid();
    int setPixformat(pixformat_t pixformat);
    uint8_t setFramesize(framesize_t framesize);
    uint8_t setFramesize(int w, int h);
    bool setZoomWindow(uint16_t x, uint16_t y, uint16_t w, uint16_t h) {
        return false;
    };
    void setContrast(int level);
    int setBrightness(int level);
    void setSaturation(int level);
    int setGainceiling(gainceiling_t gainceiling);
    int setQuality(int qs);
    uint8_t getQuality();
    int setColorbar(int enable);
    int setAutoGain(int enable, float gain_db, float gain_db_ceiling);
    void setGain(int gain) {};
    int getGain_db(float *gain_db);
    void autoExposure(int enable) {};
    int setAutoExposure(int enable, int exposure_us);
    int getExposure_us(int *exposure_us);
    int setAutoWhitebal(int enable, float r_gain_db, float g_gain_db,
                        float b_gain_db);
    int getRGB_Gain_db(float *r_gain_db, float *g_gain_db, float *b_gain_db);
    int setHmirror(int enable);
    int setVflip(int enable);
    void setHue(int hue);

    /**
     * Sets OV5640 Image Special Effects.
     *
     * Input: Enumerated
     * 0. NOEFFECT.
     * 1. NEGATIVE.
     * 2. BW.
     * 3. REDDISH.
     * 4. GREENISH.
     * 5. BLUEISH.
     * 6. RETRO.
     * 7. OVEREXPOSURE (5640 only).
     * 8. SOLARIZE (5640 only).
     * RETURNS:  Non-zero if it fails.
     */
    int setSpecialEffect(sde_t sde);

    /**
     * Sets Whitebalance mode for OV5640 camera Only.
     *
     * INPUT: integer.
     *   0 - Auto white balance.
     *   1 - Sunny.
     *   2 - Cloudy.
     *   3 - Office.
     *   4 - Home.
     *
     * RETURNS:   Non-zero if it fails.
     */
    int setWBmode(int mode);
    int setAutoBlc(int enable, int *regs);
    int getBlcRegs(int *regs);
    int setLensCorrection(int enable);
    int setNightMode(int enable);
    int setSharpness(int level);
    int setAutoSharpness(int enable);

    void showRegisters();

    uint8_t readRegister(uint8_t reg) { return (uint8_t)-1; }
    bool writeRegister(uint8_t reg, uint8_t data) { return false; }

    /*********************************************************/
    void setExposure(int exposure) {}

    bool begin(framesize_t framesize = FRAMESIZE_QVGA, int framerate = 30,
               bool use_gpio = false) {
        return false;
    };
    uint8_t setMode(uint8_t Mode, uint8_t FrameCnt) {
        return 0;
    }; // covers and extra
    int setFramerate(int framerate) { return 0; };
    int get_vt_pix_clk(uint32_t *vt_pix_clk) { return 0; };
    int getCameraClock(uint32_t *vt_pix_clk) { return 0; };
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
    void printRegisters(bool only_ones_set = true) {};
    void autoGain(int enable, float gain_db, float gain_db_ceiling) {}

  private:
    void beginXClk();
    void endXClk();
    uint8_t cameraReadRegister(uint16_t reg_addr, uint8_t &reg_data);
    uint8_t cameraWriteRegister(uint16_t reg, uint8_t data);
    int calculate_vts(uint16_t readout_height);
    int calculate_hts(uint16_t width);
    int calc_pclk_freq(uint8_t sc_pll_ctrl_0, uint8_t sc_pll_ctrl_1,
                       uint8_t sc_pll_ctrl_2, uint8_t sc_pll_ctrl_3,
                       uint8_t sys_root_div);

  private:
    int _xclk_freq = 8;

    bool _grayscale;
    int _framesize = FRAMESIZE_QVGA;
    uint8_t aecCtrl00_old = 0x78;

    void *_OV5640;

    int _saturation;
    int _hue;

    // bool flexio_configure();

    // DMA STUFF
    enum { DMABUFFER_SIZE = 1296 }; // 640x480  so 640*2*2
    static DMAChannel _dmachannel;
    static DMASetting _dmasettings[10]; // For now lets have enough for two full
                                        // size buffers...
    static uint32_t _dmaBuffer1[DMABUFFER_SIZE];
    static uint32_t _dmaBuffer2[DMABUFFER_SIZE];

    bool (*_callback)(void *frame_buffer) = nullptr;
    uint32_t _dma_frame_count;
    uint8_t *_dma_last_completed_frame;

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

    uint8_t *_frame_buffer_pointer;
    uint8_t *_frame_row_buffer_pointer; // start of the row
    uint8_t _dma_index;
    volatile bool _dma_active;
    volatile uint32_t _vsync_high_time = 0;

    static void dmaInterrupt();
    void processDMAInterrupt();
#if 0
	static void frameStartInterrupt();
	void processFrameStartInterrupt();
#endif

    // OpenMV support functions extracted from imglib.h

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
        } vx = {x};
        union {
            uint32_t i;
            float f;
        } mx = {(vx.i & 0x007FFFFF) | 0x3f000000};
        float y = vx.i;
        y *= 1.1920928955078125e-7f;

        return y - 124.22551499f - 1.498030302f * mx.f -
               1.72587999f / (0.3520887068f + mx.f);
    }
    inline float fast_log(float x) { return 0.69314718f * fast_log2(x); }
    inline int fast_floorf(float x) {
        int i;
        asm volatile("vcvt.S32.f32  %[r], %[x]\n" : [r] "=t"(i) : [x] "t"(x));
        return i;
    }
    inline int fast_ceilf(float x) {
        int i;
        x += 0.9999f;
        asm volatile("vcvt.S32.f32  %[r], %[x]\n" : [r] "=t"(i) : [x] "t"(x));
        return i;
    }
    inline int fast_roundf(float x) {
        int i;
        asm volatile("vcvtr.s32.f32  %[r], %[x]\n" : [r] "=t"(i) : [x] "t"(x));
        return i;
    }
    inline float fast_expf(float x) {
        exp_t e;
        e.l = (uint32_t)(1512775 * x + 1072632447);
        // IEEE binary32 format
        e.e = (e.e - 1023 + 127) & 0xFF; // rebase

        // uint32_t packed = (e.s << 31) | (e.e << 23) | e.m <<3;
        // return *((float*)&packed);
        union {
            uint32_t ul;
            float f;
        } packed;
        packed.ul = (e.s << 31) | (e.e << 23) | e.m << 3;
        return packed.f;
    }
};

#endif
