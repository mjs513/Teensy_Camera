// SPDX-License-Identifier: GPL-2.0-only
/*
 * This file is part of the Arduino_OX767X library.
 * Copyright (c) 2020 Arduino SA. All rights reserved.
 */

#ifndef _OV767X_H_
#define _OV767X_H_

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

// #define OV7670_VSYNC 2    // Lets setup for T4.1 CSI pins
// #define USE_CSI_PINS

// #define OV7670_USE_DEBUG_PINS
#ifdef OV7670_USE_DEBUG_PINS
#define OV7670_DEBUG_PIN_1 14
#define OV7670_DEBUG_PIN_2 15
#define OV7670_DEBUG_PIN_3 3
#define DebugDigitalWrite(pin, val) digitalWriteFast(pin, val)
#define DebugDigitalToggle(pin) digitalToggleFast(pin)
#else
#define DebugDigitalWrite(pin, val)
#define DebugDigitalToggle(pin)
#endif

#endif

// dummy defines for camera class
#define HIMAX_MODE_STREAMING 0x01         // I2C triggered streaming enable
#define HIMAX_MODE_STREAMING_NFRAMES 0x03 // Output N frames
/*
enum
{
  YUV422 = 0,
  RGB444 = 1,
  RGB565 = 2,
  // SBGGR8 = 3
  GRAYSCALE = 4
};
*/
/*
enum
{
  OV7670 = 0,
  OV7675 = 1
};
*/
/*
enum
{
  VGA = 0,  // 640x480
  CIF = 1,  // 352x240
  QVGA = 2, // 320x240
  QCIF = 3,  // 176x144
  QQVGA = 4,  // 160x120
};
*/
class OV767X : public ImageSensor {
  public:
    OV767X();

    // int begin(int resolution, int format, int fps,  int camera_name = OV7670,
    // bool use_gpio = false); // Supported FPS: 1, 5, 10, 15, 30
    bool
    begin_omnivision(framesize_t resolution = FRAMESIZE_QVGA,
                     pixformat_t format = RGB565, int fps = 30,
                     int camera_name = OV7670,
                     bool use_gpio = false); // Supported FPS: 1, 5, 10, 15, 30
    void end();

    // must be called after Camera.begin():
    int bitsPerPixel() const;
    int bytesPerPixel() const;
    /********************************************************************************************/
    //-------------------------------------------------------
    // normal Read mode
    //	size_t readFrameGPIO(void* buffer, size_t cb1=(uint32_t)-1, void*
    // buffer2=nullptr, size_t cb2=0);

    // FlexIO is default mode for the camera

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

    void setDMACompleteISRPriority(uint8_t priority) {
        NVIC_SET_PRIORITY(_dmachannel.channel & 0xf, priority);
    }
    /********************************************************************************************/

    // TBD Allow user to set all of the buffers...
    void readFrameDMA(void *buffer);

    void testPattern(int pattern = 2);
    void noTestPattern();

    uint16_t getModelid();
    void setSaturation(int saturation); // 0 - 255
    void setHue(int hue);               // -180 - 180
    int setBrightness(int brightness);  // 0 - 255
    void setContrast(int contrast);     // 0 - 127
    int setHmirror(int enable);
    int setVflip(int enable);
    void setGain(int gain); // 0 - 255
    void autoGain(int enable, float gain_db, float gain_db_ceiling);
    void setExposure(int exposure); // 0 - 65535
    void autoExposure(int enable);
    void showRegisters();

    uint8_t readRegister(uint8_t reg) { return cameraReadRegister(reg); }
    bool writeRegister(uint8_t reg, uint8_t data);

    // If you are using different pin numbers other than those defined
    // in default_camera_pins.h - you can call Camera.setPins(...)
    // before calling Camera.begin()
    /****************** covers non supported virtual funcs in OV class
     * *****************/
    bool begin(framesize_t framesize = FRAMESIZE_QVGA, int framerate = 30,
               bool use_gpio = false) {
        return 0;
    };
    uint8_t setMode(uint8_t Mode, uint8_t FrameCnt) {
        return 0;
    }; // covers and extra
    int reset() { return 0; };
    int setPixformat(pixformat_t pfmt) { return 0; };
    uint8_t setFramesize(framesize_t framesize) { return 0; };
    int setFramerate(int framerate) { return 0; };
    int setGainceiling(gainceiling_t gainceiling) { return 0; };
    int setColorbar(int enable) { return 0; };
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
    int setAutoExposure(int enable, int exposure_us) { return 0; };

    // unique to GC2145................................
    void printRegisters(bool only_ones_set = true) {};
    int setAutoWhitebal(int enable, float r_gain_db, float g_gain_db,
                        float b_gain_db) {
        return 0;
    };

    // Experiment add some of the register names here:
    enum {
        REG_GAIN = 0x00,     /* Gain lower 8 bits (rest in vref) */
        REG_BLUE = 0x01,     /* blue gain */
        REG_RED = 0x02,      /* red gain */
        REG_VREF = 0x03,     /* Pieces of GAIN, VSTART, VSTOP */
        REG_COM1 = 0x04,     /* Control 1 */
        COM1_CCIR656 = 0x40, /* CCIR656 enable */
        REG_BAVE = 0x05,     /* U/B Average level */
        REG_GbAVE = 0x06,    /* Y/Gb Average level */
        REG_AECHH = 0x07,    /* AEC MS 5 bits */
        REG_RAVE = 0x08,     /* V/R Average level */
        REG_COM2 = 0x09,     /* Control 2 */
        COM2_SSLEEP = 0x10,  /* Soft sleep mode */
        REG_PID = 0x0a,      /* Product ID MSB */
        REG_VER = 0x0b,      /* Product ID LSB */
        REG_COM3 = 0x0c,     /* Control 3 */
        COM3_SWAP = 0x40,    /* Byte swap */
        COM3_SCALEEN = 0x08, /* Enable scaling */
        COM3_DCWEN = 0x04,   /* Enable downsamp/crop/window */
        REG_COM4 = 0x0d,     /* Control 4 */
        REG_COM5 = 0x0e,     /* All "reserved" */
        REG_COM6 = 0x0f,     /* Control 6 */
        REG_AECH = 0x10,     /* More bits of AEC value */
        REG_CLKRC = 0x11,    /* Clocl control */
        CLK_EXT = 0x40,      /* Use external clock directly */
        CLK_SCALE = 0x3f,    /* Mask for internal clock scale */
        REG_COM7 = 0x12,     /* Control 7 */
        COM7_RESET = 0x80,   /* Register reset */
        COM7_FMT_MASK = 0x38,
        COM7_FMT_VGA = 0x00,
        COM7_FMT_CIF = 0x20,   /* CIF format */
        COM7_FMT_QVGA = 0x10,  /* QVGA format */
        COM7_FMT_QCIF = 0x08,  /* QCIF format */
        COM7_RGB = 0x04,       /* bits 0 and 2 - RGB format */
        COM7_YUV = 0x00,       /* YUV */
        COM7_BAYER = 0x01,     /* Bayer format */
        COM7_PBAYER = 0x05,    /* "Processed bayer" */
        REG_COM8 = 0x13,       /* Control 8 */
        COM8_FASTAEC = 0x80,   /* Enable fast AGC/AEC */
        COM8_AECSTEP = 0x40,   /* Unlimited AEC step size */
        COM8_BFILT = 0x20,     /* Band filter enable */
        COM8_AGC = 0x04,       /* Auto gain enable */
        COM8_AWB = 0x02,       /* White balance enable */
        COM8_AEC = 0x01,       /* Auto exposure enable */
        REG_COM9 = 0x14,       /* Control 9  - gain ceiling */
        REG_COM10 = 0x15,      /* Control 10 */
        COM10_HSYNC = 0x40,    /* HSYNC instead of HREF */
        COM10_PCLK_HB = 0x20,  /* Suppress PCLK on horiz blank */
        COM10_HREF_REV = 0x08, /* Reverse HREF */
        COM10_VS_LEAD = 0x04,  /* VSYNC on clock leading edge */
        COM10_VS_NEG = 0x02,   /* VSYNC negative */
        COM10_HS_NEG = 0x01,   /* HSYNC negative */
        REG_HSTART = 0x17,     /* Horiz start high bits */
        REG_HSTOP = 0x18,      /* Horiz stop high bits */
        REG_VSTART = 0x19,     /* Vert start high bits */
        REG_VSTOP = 0x1a,      /* Vert stop high bits */
        REG_PSHFT = 0x1b,      /* Pixel delay after HREF */
        REG_MIDH = 0x1c,       /* Manuf. ID high */
        REG_MIDL = 0x1d,       /* Manuf. ID low */
        REG_MVFP = 0x1e,       /* Mirror / vflip */
        MVFP_MIRROR = 0x20,    /* Mirror image */
        MVFP_FLIP = 0x10,      /* Vertical flip */
        REG_AEW = 0x24,        /* AGC upper limit */
        REG_AEB = 0x25,        /* AGC lower limit */
        REG_VPT = 0x26,        /* AGC/AEC fast mode op region */
        REG_HSYST = 0x3,       /* HSYNC rising edge delay */
        REG_HSYEN = 0x31,      /* HSYNC falling edge delay */
        REG_HREF = 0x32        /* HREF pieces */
    };

  private:
    uint8_t cameraReadRegister(uint8_t reg);

  private:
    int _xclk_freq = 14;

    void *_ov7670;

    int _saturation;
    int _hue;

    //	bool flexio_configure(); // moved to ImageSensor

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
    // TBD Allow user to set all of the buffers...

    // Added settings for configurable flexio

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
};

#endif
