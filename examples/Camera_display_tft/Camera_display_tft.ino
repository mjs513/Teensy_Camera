#include <SD.h>
#include <SPI.h>
#include <stdint.h>

#include "Camera.h"

#define USE_MMOD_ATP_ADAPTER

#define TFT_ROTATION 3
 #define USE_SDCARD

//#define use9488
//#define ARDUCAM_CAMERA_HM01B0
//#define ARDUCAM_CAMERA_HM0360
//#define ARDUCAM_CAMERA_OV2640
// #define ARDUCAM_CAMERA_OV7670
//#define ARDUCAM_CAMERA_OV7675
//#define ARDUCAM_CAMERA_GC2145
#define ARDUCAM_CAMERA_OV5640

//set cam configuration - need to remember when saving jpeg
framesize_t camera_framesize = FRAMESIZE_QVGA;
pixformat_t camera_format = RGB565;
bool useGPIO = false;


#if defined(ARDUCAM_CAMERA_HM0360)
#define CAMERA_USES_MONO_PALETTE
#include "TMM_HM0360/HM0360.h"
HM0360 himax;
Camera camera(himax);
#define CameraID 0x0360
#define MIRROR_FLIP_CAMERA

#elif defined(ARDUCAM_CAMERA_HM01B0)
#define CAMERA_USES_MONO_PALETTE
#include "TMM_HM01B0/HM01B0.h"
HM01B0 himax;
Camera camera(himax);
#define CameraID 0x01B0
#define MIRROR_FLIP_CAMERA

#elif defined(ARDUCAM_CAMERA_OV2640)
#include "TMM_OV2640/OV2640.h"
OV2640 omni;
Camera camera(omni);
#define CameraID OV2640a
#define MIRROR_FLIP_CAMERA

#elif defined(ARDUCAM_CAMERA_OV7670)
#include "TMM_OV767X/OV767X.h"
OV767X omni;
Camera camera(omni);
#define CameraID OV7670

#elif defined(ARDUCAM_CAMERA_OV7675)
#include "TMM_OV767X/OV767X.h"
OV767X omni;
Camera camera(omni);
#define CameraID OV7675

#elif defined(ARDUCAM_CAMERA_GC2145)
#include "TMM_GC2145/GC2145.h"
GC2145 galaxycore;
Camera camera(galaxycore);
#define CameraID GC2145a

#elif defined(ARDUCAM_CAMERA_OV5640)
#include "TMM_OV5640/OV5640.h"
OV5640 omni;
Camera camera(omni);
#define CameraID OV5640a
#endif

#if defined(ARDUCAM_CAMERA_OV2640)
#define skipFrames 1
#else
#define skipFrames 1
#endif

File file;

/*****************************************************
 * If using the HM01B0 Arduino breakout 8bit mode    *
 * does not work.  Arduino breakout only brings out  *
 * the lower 4 bits.                                 *
 ****************************************************/
#if defined(ARDUCAM_CAMERA_HM01B0)
#define _hmConfig 1 // note HM0360 can operater in 4 bit mode as well
#else
#define _hmConfig 0 // select mode string below
#endif

PROGMEM const char hmConfig[][48] = {"FLEXIO_CUSTOM_LIKE_8_BIT",
                                     "FLEXIO_CUSTOM_LIKE_4_BIT"};

// Define Palette for Himax Cameras
#define MCP(m) (uint16_t)(((m & 0xF8) << 8) | ((m & 0xFC) << 3) | (m >> 3))

static const uint16_t mono_palette[256] PROGMEM = {
    MCP(0x00), MCP(0x01), MCP(0x02), MCP(0x03), MCP(0x04), MCP(0x05), MCP(0x06),
    MCP(0x07), MCP(0x08), MCP(0x09), MCP(0x0a), MCP(0x0b), MCP(0x0c), MCP(0x0d),
    MCP(0x0e), MCP(0x0f), MCP(0x10), MCP(0x11), MCP(0x12), MCP(0x13), MCP(0x14),
    MCP(0x15), MCP(0x16), MCP(0x17), MCP(0x18), MCP(0x19), MCP(0x1a), MCP(0x1b),
    MCP(0x1c), MCP(0x1d), MCP(0x1e), MCP(0x1f), MCP(0x20), MCP(0x21), MCP(0x22),
    MCP(0x23), MCP(0x24), MCP(0x25), MCP(0x26), MCP(0x27), MCP(0x28), MCP(0x29),
    MCP(0x2a), MCP(0x2b), MCP(0x2c), MCP(0x2d), MCP(0x2e), MCP(0x2f), MCP(0x30),
    MCP(0x31), MCP(0x32), MCP(0x33), MCP(0x34), MCP(0x35), MCP(0x36), MCP(0x37),
    MCP(0x38), MCP(0x39), MCP(0x3a), MCP(0x3b), MCP(0x3c), MCP(0x3d), MCP(0x3e),
    MCP(0x3f), MCP(0x40), MCP(0x41), MCP(0x42), MCP(0x43), MCP(0x44), MCP(0x45),
    MCP(0x46), MCP(0x47), MCP(0x48), MCP(0x49), MCP(0x4a), MCP(0x4b), MCP(0x4c),
    MCP(0x4d), MCP(0x4e), MCP(0x4f), MCP(0x50), MCP(0x51), MCP(0x52), MCP(0x53),
    MCP(0x54), MCP(0x55), MCP(0x56), MCP(0x57), MCP(0x58), MCP(0x59), MCP(0x5a),
    MCP(0x5b), MCP(0x5c), MCP(0x5d), MCP(0x5e), MCP(0x5f), MCP(0x60), MCP(0x61),
    MCP(0x62), MCP(0x63), MCP(0x64), MCP(0x65), MCP(0x66), MCP(0x67), MCP(0x68),
    MCP(0x69), MCP(0x6a), MCP(0x6b), MCP(0x6c), MCP(0x6d), MCP(0x6e), MCP(0x6f),
    MCP(0x70), MCP(0x71), MCP(0x72), MCP(0x73), MCP(0x74), MCP(0x75), MCP(0x76),
    MCP(0x77), MCP(0x78), MCP(0x79), MCP(0x7a), MCP(0x7b), MCP(0x7c), MCP(0x7d),
    MCP(0x7e), MCP(0x7f), MCP(0x80), MCP(0x81), MCP(0x82), MCP(0x83), MCP(0x84),
    MCP(0x85), MCP(0x86), MCP(0x87), MCP(0x88), MCP(0x89), MCP(0x8a), MCP(0x8b),
    MCP(0x8c), MCP(0x8d), MCP(0x8e), MCP(0x8f), MCP(0x90), MCP(0x91), MCP(0x92),
    MCP(0x93), MCP(0x94), MCP(0x95), MCP(0x96), MCP(0x97), MCP(0x98), MCP(0x99),
    MCP(0x9a), MCP(0x9b), MCP(0x9c), MCP(0x9d), MCP(0x9e), MCP(0x9f), MCP(0xa0),
    MCP(0xa1), MCP(0xa2), MCP(0xa3), MCP(0xa4), MCP(0xa5), MCP(0xa6), MCP(0xa7),
    MCP(0xa8), MCP(0xa9), MCP(0xaa), MCP(0xab), MCP(0xac), MCP(0xad), MCP(0xae),
    MCP(0xaf), MCP(0xb0), MCP(0xb1), MCP(0xb2), MCP(0xb3), MCP(0xb4), MCP(0xb5),
    MCP(0xb6), MCP(0xb7), MCP(0xb8), MCP(0xb9), MCP(0xba), MCP(0xbb), MCP(0xbc),
    MCP(0xbd), MCP(0xbe), MCP(0xbf), MCP(0xc0), MCP(0xc1), MCP(0xc2), MCP(0xc3),
    MCP(0xc4), MCP(0xc5), MCP(0xc6), MCP(0xc7), MCP(0xc8), MCP(0xc9), MCP(0xca),
    MCP(0xcb), MCP(0xcc), MCP(0xcd), MCP(0xce), MCP(0xcf), MCP(0xd0), MCP(0xd1),
    MCP(0xd2), MCP(0xd3), MCP(0xd4), MCP(0xd5), MCP(0xd6), MCP(0xd7), MCP(0xd8),
    MCP(0xd9), MCP(0xda), MCP(0xdb), MCP(0xdc), MCP(0xdd), MCP(0xde), MCP(0xdf),
    MCP(0xe0), MCP(0xe1), MCP(0xe2), MCP(0xe3), MCP(0xe4), MCP(0xe5), MCP(0xe6),
    MCP(0xe7), MCP(0xe8), MCP(0xe9), MCP(0xea), MCP(0xeb), MCP(0xec), MCP(0xed),
    MCP(0xee), MCP(0xef), MCP(0xf0), MCP(0xf1), MCP(0xf2), MCP(0xf3), MCP(0xf4),
    MCP(0xf5), MCP(0xf6), MCP(0xf7), MCP(0xf8), MCP(0xf9), MCP(0xfa), MCP(0xfb),
    MCP(0xfc), MCP(0xfd), MCP(0xfe), MCP(0xff)};

static const uint16_t mono_palette_4bit_rev[256] PROGMEM = {
	MCP(0x00), MCP(0x08), MCP(0x04), MCP(0x0C), MCP(0x02), MCP(0x0A), MCP(0x06), MCP(0x0E), 
	MCP(0x01), MCP(0x09), MCP(0x05), MCP(0x0D), MCP(0x03), MCP(0x0B), MCP(0x07), MCP(0x0F), 
	MCP(0x80), MCP(0x88), MCP(0x84), MCP(0x8C), MCP(0x82), MCP(0x8A), MCP(0x86), MCP(0x8E), 
	MCP(0x81), MCP(0x89), MCP(0x85), MCP(0x8D), MCP(0x83), MCP(0x8B), MCP(0x87), MCP(0x8F), 
	MCP(0x40), MCP(0x48), MCP(0x44), MCP(0x4C), MCP(0x42), MCP(0x4A), MCP(0x46), MCP(0x4E), 
	MCP(0x41), MCP(0x49), MCP(0x45), MCP(0x4D), MCP(0x43), MCP(0x4B), MCP(0x47), MCP(0x4F), 
	MCP(0xC0), MCP(0xC8), MCP(0xC4), MCP(0xCC), MCP(0xC2), MCP(0xCA), MCP(0xC6), MCP(0xCE), 
	MCP(0xC1), MCP(0xC9), MCP(0xC5), MCP(0xCD), MCP(0xC3), MCP(0xCB), MCP(0xC7), MCP(0xCF), 
	MCP(0x20), MCP(0x28), MCP(0x24), MCP(0x2C), MCP(0x22), MCP(0x2A), MCP(0x26), MCP(0x2E), 
	MCP(0x21), MCP(0x29), MCP(0x25), MCP(0x2D), MCP(0x23), MCP(0x2B), MCP(0x27), MCP(0x2F), 
	MCP(0xA0), MCP(0xA8), MCP(0xA4), MCP(0xAC), MCP(0xA2), MCP(0xAA), MCP(0xA6), MCP(0xAE), 
	MCP(0xA1), MCP(0xA9), MCP(0xA5), MCP(0xAD), MCP(0xA3), MCP(0xAB), MCP(0xA7), MCP(0xAF), 
	MCP(0x60), MCP(0x68), MCP(0x64), MCP(0x6C), MCP(0x62), MCP(0x6A), MCP(0x66), MCP(0x6E), 
	MCP(0x61), MCP(0x69), MCP(0x65), MCP(0x6D), MCP(0x63), MCP(0x6B), MCP(0x67), MCP(0x6F), 
	MCP(0xE0), MCP(0xE8), MCP(0xE4), MCP(0xEC), MCP(0xE2), MCP(0xEA), MCP(0xE6), MCP(0xEE), 
	MCP(0xE1), MCP(0xE9), MCP(0xE5), MCP(0xED), MCP(0xE3), MCP(0xEB), MCP(0xE7), MCP(0xEF), 
	MCP(0x10), MCP(0x18), MCP(0x14), MCP(0x1C), MCP(0x12), MCP(0x1A), MCP(0x16), MCP(0x1E), 
	MCP(0x11), MCP(0x19), MCP(0x15), MCP(0x1D), MCP(0x13), MCP(0x1B), MCP(0x17), MCP(0x1F), 
	MCP(0x90), MCP(0x98), MCP(0x94), MCP(0x9C), MCP(0x92), MCP(0x9A), MCP(0x96), MCP(0x9E), 
	MCP(0x91), MCP(0x99), MCP(0x95), MCP(0x9D), MCP(0x93), MCP(0x9B), MCP(0x97), MCP(0x9F), 
	MCP(0x50), MCP(0x58), MCP(0x54), MCP(0x5C), MCP(0x52), MCP(0x5A), MCP(0x56), MCP(0x5E), 
	MCP(0x51), MCP(0x59), MCP(0x55), MCP(0x5D), MCP(0x53), MCP(0x5B), MCP(0x57), MCP(0x5F), 
	MCP(0xD0), MCP(0xD8), MCP(0xD4), MCP(0xDC), MCP(0xD2), MCP(0xDA), MCP(0xD6), MCP(0xDE), 
	MCP(0xD1), MCP(0xD9), MCP(0xD5), MCP(0xDD), MCP(0xD3), MCP(0xDB), MCP(0xD7), MCP(0xDF), 
	MCP(0x30), MCP(0x38), MCP(0x34), MCP(0x3C), MCP(0x32), MCP(0x3A), MCP(0x36), MCP(0x3E), 
	MCP(0x31), MCP(0x39), MCP(0x35), MCP(0x3D), MCP(0x33), MCP(0x3B), MCP(0x37), MCP(0x3F), 
	MCP(0xB0), MCP(0xB8), MCP(0xB4), MCP(0xBC), MCP(0xB2), MCP(0xBA), MCP(0xB6), MCP(0xBE), 
	MCP(0xB1), MCP(0xB9), MCP(0xB5), MCP(0xBD), MCP(0xB3), MCP(0xBB), MCP(0xB7), MCP(0xBF), 
	MCP(0x70), MCP(0x78), MCP(0x74), MCP(0x7C), MCP(0x72), MCP(0x7A), MCP(0x76), MCP(0x7E), 
	MCP(0x71), MCP(0x79), MCP(0x75), MCP(0x7D), MCP(0x73), MCP(0x7B), MCP(0x77), MCP(0x7F), 
	MCP(0xF0), MCP(0xF8), MCP(0xF4), MCP(0xFC), MCP(0xF2), MCP(0xFA), MCP(0xF6), MCP(0xFE), 
	MCP(0xF1), MCP(0xF9), MCP(0xF5), MCP(0xFD), MCP(0xF3), MCP(0xFB), MCP(0xF7), MCP(0xFF)
	};    
const uint16_t *current_mono_palette = mono_palette;

#ifdef ARDUINO_TEENSY_DEVBRD4
// Set up ILI9341
#undef USE_MMOD_ATP_ADAPTER

#define TFT_CS 10 // AD_B0_02
#define TFT_DC 25 // AD_B0_03
#define TFT_RST 24

#elif defined(ARDUINO_TEENSY41)
#undef USE_MMOD_ATP_ADAPTER

// My T4.1 Camera board
#undef TFT_ROTATION
#define TFT_ROTATION 1
#define TFT_DC 9
#define TFT_CS 7
#define TFT_RST 8 

#elif defined(USE_MMOD_ATP_ADAPTER)
#define TFT_DC 4  // 0   // "TX1" on left side of Sparkfun ML Carrier
#define TFT_CS 5  // 4   // "CS" on left side of Sparkfun ML Carrier
#define TFT_RST 2 // 1  // "RX1" on left side of Sparkfun ML Carrier

#else
#define TFT_DC 0  // 20   // "TX1" on left side of Sparkfun ML Carrier
#define TFT_CS 4  // 5, 4   // "CS" on left side of Sparkfun ML Carrier
#define TFT_RST 1 // 2, 1  // "RX1" on left side of Sparkfun ML Carrier
#endif

#if defined(use9488)
#include <ILI9488_t3.h>
ILI9488_t3 tft = ILI9488_t3(TFT_CS, TFT_DC, TFT_RST);
#define TFT_BLACK ILI9488_BLACK
#define TFT_YELLOW ILI9488_YELLOW
#define TFT_RED ILI9488_RED
#define TFT_GREEN ILI9488_GREEN
#define TFT_BLUE ILI9488_BLUE
#define CENTER ILI9488_t3::CENTER

#else
#include <ILI9341_t3n.h>
ILI9341_t3n tft = ILI9341_t3n(TFT_CS, TFT_DC, TFT_RST);
#define TFT_BLACK ILI9341_BLACK
#define TFT_YELLOW ILI9341_YELLOW
#define TFT_RED ILI9341_RED
#define TFT_GREEN ILI9341_GREEN
#define TFT_BLUE ILI9341_BLUE
#define CENTER ILI9341_t3n::CENTER
#endif

// Setup framebuffers
DMAMEM uint16_t FRAME_WIDTH, FRAME_HEIGHT;
#ifdef ARDUINO_TEENSY_DEVBRD4
//#include "SDRAM_t4.h"
//SDRAM_t4 sdram;
uint16_t *frameBuffer = nullptr;
uint16_t *frameBuffer2 = nullptr;
uint16_t *frameBufferSDRAM = nullptr;
uint16_t *frameBufferSDRAM2 = nullptr;
DMAMEM uint16_t frameBufferM[640 * 240] __attribute__((aligned(32)));
uint16_t frameBufferM2[640 * 240] __attribute__((aligned(32)));
uint32_t sizeof_framebuffer = 0;
uint32_t sizeof_framebuffer2 = 0;
uint32_t sizeof_framebufferSDRAM = 0;
#elif defined(ARDUINO_TEENSY41)
// CSI - lets try to setup for PSRAM (EXTMEM)
// only half buffer will fit in each of the two main memory regions
// split into two parts, part dmamem and part fast mememory to fit 640x480x2
  #if (defined(ARDUCAM_CAMERA_OV7675) || defined(ARDUCAM_CAMERA_OV7670) ||        \
    defined(ARDUCAM_CAMERA_OV2640)  || defined(ARDUCAM_CAMERA_GC2145) ||        \
    defined(ARDUCAM_CAMERA_OV5640))
    EXTMEM uint16_t frameBuffer[800 * 600] __attribute__((aligned(32)));
    EXTMEM uint16_t frameBuffer2[800 * 600] __attribute__((aligned(32)));
    uint16_t *frameBufferRead;
  #else
    EXTMEM uint8_t frameBuffer[800 * 600] __attribute__((aligned(32)));
    EXTMEM uint8_t frameBuffer2[800 * 600] __attribute__((aligned(32)));
    uint8_t *frameBufferRead;
  #endif
const uint32_t sizeof_framebuffer = sizeof(frameBuffer);
const uint32_t sizeof_framebuffer2 = sizeof(frameBuffer2);
#else
#if defined(USE_SDCARD)
DMAMEM uint16_t frameBuffer[700 * 320] __attribute__((aligned(32)));
uint16_t frameBuffer2[480 * 240] __attribute__((aligned(32)));
#else
DMAMEM uint16_t frameBuffer[640 * 240] __attribute__((aligned(32)));
uint16_t frameBuffer2[640 * 240] __attribute__((aligned(32)));
#endif
//#define SCREEN_ROTATION 1
uint16_t *frameBufferRead;
const uint32_t sizeof_framebuffer = sizeof(frameBuffer);
const uint32_t sizeof_framebuffer2 = sizeof(frameBuffer2);
#endif

// Setup display modes frame / video
bool g_continuous_flex_mode = false;
bool g_flex_dual_buffer_per_frame = false;
void *volatile g_new_flexio_data = nullptr;
void *volatile g_last_flexio_data = nullptr;
uint32_t g_flexio_capture_count = 0;
uint32_t g_flexio_redraw_count = 0;
elapsedMillis g_flexio_runtime;
bool g_dma_mode = false;

ae_cfg_t aecfg;

void setup() {
    Serial.begin(921600);
    while (!Serial && millis() < 5000) {
    }
#if defined(USB_DUAL_SERIAL) || defined(USB_TRIPLE_SERIAL)
    SerialUSB1.begin(921600);
#endif

    if (CrashReport) {
        Serial.print(CrashReport);
        while (1)
            ;
    }

    Serial.printf("Start Display CS:%u DC:%u RST:%u\n", TFT_CS, TFT_DC, TFT_RST);
    tft.begin(15000000);

    tft.setRotation(TFT_ROTATION);
    tft.fillScreen(TFT_RED);
    delay(500);
    tft.fillScreen(TFT_GREEN);
    delay(500);
    tft.fillScreen(TFT_BLUE);
    delay(500);
    tft.fillScreen(TFT_BLACK);
    delay(500);

    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_YELLOW);
    tft.setTextSize(2);
    tft.println("Waiting for Arduino Serial Monitor...");

#if defined(USE_SDCARD)
    Serial.println("Using SDCARD - Initializing");
#if MMOD_ML == 1
    if (!SD.begin(10)) {
#else
    if (!SD.begin(BUILTIN_SDCARD)) {
#endif
    }
    Serial.println("initialization failed!");
    // while (1){
    //    LEDON; delay(100);
    //    LEDOFF; delay(100);
    //  }
    Serial.println("initialization done.");
    delay(100);
#endif

    while (!Serial)
        ;
    Serial.println("hm0360 Camera Test");
    Serial.println(hmConfig[_hmConfig]);
    Serial.println("------------------");

    delay(500);

    tft.fillScreen(TFT_BLACK);

/***************************************************************/
//    setPins(uint8_t mclk_pin, uint8_t pclk_pin, uint8_t vsync_pin, uint8_t
//    hsync_pin, en_pin, uint8_t g0, uint8_t g1,uint8_t g2, uint8_t g3, uint8_t
//    g4=0xff, uint8_t g5=0xff,uint8_t g6=0xff,uint8_t g7=0xff);
  uint8_t reset_pin = 31;
  uint8_t powdwn_pin = 30;
#ifdef USE_MMOD_ATP_ADAPTER
  pinMode(0, OUTPUT);
  pinMode(3, OUTPUT);
  Serial.println("Using Micromod ATP Adapter");
#elif defined(ARDUINO_TEENSY41)
  // CSI support
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
#endif

    uint8_t status = 0;
#if (defined(ARDUCAM_CAMERA_OV7675) || defined(ARDUCAM_CAMERA_OV7670) ||      \
     defined(ARDUCAM_CAMERA_OV2640) || defined(ARDUCAM_CAMERA_GC2145) ||      \
     defined(ARDUCAM_CAMERA_OV5640)) 
    status = camera.begin(camera_framesize, camera_format, 15, CameraID, useGPIO);
#else
    // HM0360(4pin) 15/30 @6mhz, 60 works but get 4 pics on one screen :)
    // HM0360(8pin) 15/30/60/120 works :)
    #if defined(ARDUCAM_CAMERA_HM01B0)
    camera.data4BitMode(true);
    #endif
    status = camera.begin(camera_framesize, 15, useGPIO);
#endif


  Serial.printf("Begin status: %d\n", status);
  if (!status) {
    Serial.println("Camera failed to start - try reset!!!");
    //Serial.printf("\tPin 30:%u 31:%u\n", digitalReadFast(30), digitalReadFast(31));
    Serial.printf("\tPin rst(%u):%u PowDn(%u):%u\n", reset_pin, digitalRead(reset_pin), powdwn_pin, digitalRead(powdwn_pin));
    pinMode(reset_pin, OUTPUT);
    digitalWriteFast(reset_pin, LOW);
    delay(500);
    pinMode(reset_pin, INPUT_PULLUP);
    delay(500);
#if (defined(ARDUCAM_CAMERA_OV7675) || defined(ARDUCAM_CAMERA_OV7670) ||      \
     defined(ARDUCAM_CAMERA_OV2640) || defined(ARDUCAM_CAMERA_GC2145) ||      \
     defined(ARDUCAM_CAMERA_OV5640)) 
    status = camera.begin(FRAMESIZE_QVGA, RGB565, 12, CameraID, false);
#else
    // HM0360(4pin) 15/30 @6mhz, 60 works but get 4 pics on one screen :)
    // HM0360(8pin) 15/30/60/120 works :)
    #if defined(ARDUCAM_CAMERA_HM01B0)
    camera.data4BitMode(true);
    #endif
    status = camera.begin(camera_framesize, 15, false);
#endif
    if (!status) {
      Serial.println("Camera failed to start again program halted");
      while (1) {}
    }
  }

  if (camera.dataPinsReversed() && camera.data4BitMode()) {
    current_mono_palette = mono_palette_4bit_rev; //mono_palette;
  }


#ifdef MIRROR_FLIP_CAMERA
    camera.setHmirror(true);
    camera.setVflip(true);
#endif


#if defined(ARDUINO_TEENSY_DEVBRD4)
    // we need to allocate bufers
    // if (!sdram.begin()) {
    //  Serial.printf("SDRAM Init Failed!!!\n");
    //  while (1)
    //    ;
    //};
#if defined(ARDUCAM_CAMERA_OV7675) || defined(ARDUCAM_CAMERA_OV7670) ||        \
    defined(ARDUCAM_CAMERA_OV2640) || defined(ARDUCAM_CAMERA_GC2145) ||         \
    defined(ARDUCAM_CAMERA_OV5640)) 

    frameBuffer = (uint16_t *)((
        ((uint32_t)(sdram_malloc(camera.width() * camera.height() * 2 + 32)) +
         32) &
        0xffffffe0));
    frameBuffer2 = (uint16_t *)((
        ((uint32_t)(sdram_malloc(camera.width() * camera.height() * 2 + 32)) +
         32) &
        0xffffffe0));
#else
    frameBuffer = (uint8_t *)((
        ((uint32_t)(sdram_malloc(camera.width() * camera.height() + 32)) + 32) &
        0xffffffe0));
    frameBuffer2 = (uint8_t *)((
        ((uint32_t)(sdram_malloc(camera.width() * camera.height() + 32)) + 32) &
        0xffffffe0));
#endif
    Serial.printf("Camera Buffers: %p %p\n", frameBuffer, frameBuffer2);
#endif

#if (defined(ARDUCAM_CAMERA_OV7675) || defined(ARDUCAM_CAMERA_OV7670))
    camera.setContrast(0x30);
    camera.setBrightness(0x80);
    camera.autoExposure(1);
#elif defined(ARDUCAM_CAMERA_OV2640)
    // camera.setBrightness(0);          // -2 to +2
    // camera.setContrast(0);            // -2 to +2
    // camera.setSaturation(0);          // -2 to +2
    // omni.setSpecialEffect(RETRO);  // NOEFFECT, NEGATIVE, BW, REDDISH,
    // GREEISH, BLUEISH, RETRO
    omni.setWBmode(0); // AWB ON, 1 - Sunny, 2 - Cloudy, 3 - Office, 4 - Home
#elif !defined(ARDUCAM_CAMERA_OV5640)
    camera.setGainceiling(GAINCEILING_2X);
    camera.setBrightness(3);
    camera.setAutoExposure(
        true, 1500); // higher the setting the less saturaturation of whiteness
    camera.cmdUpdate(); // only need after changing auto exposure settings
    camera.setMode(HIMAX_MODE_STREAMING,
                   0); // turn on, continuous streaming mode
#endif

    Serial.println("Camera settings:");
    Serial.print("\twidth = ");
    Serial.println(camera.width());
    Serial.print("\theight = ");
    Serial.println(camera.height());
    // Serial.print("\tbits per pixel = NA");
    // Serial.println(camera.bitsPerPixel());
    Serial.println();
    Serial.printf("TFT Width = %u Height = %u\n\n", tft.width(), tft.height());

    FRAME_HEIGHT = camera.height();
    FRAME_WIDTH = camera.width();
    Serial.printf("ImageSize (w,h): %d, %d\n", FRAME_WIDTH, FRAME_HEIGHT);

    // Lets setup camera interrupt priorities:
    // camera.setVSyncISRPriority(102); // higher priority than default
    camera.setDMACompleteISRPriority(192); // lower than default

#if defined(ARDUCAM_CAMERA_GC2145)
    /***************note for the GC2145 the following is supported
    ************** GC2145_DISABLED = 0, GC2145_COLOR_BARS,
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
      GC2145_SOLID_MAGENTA
    ****************************************************************************/
    camera.setColorbar(0);
#else
    /**************************************************************************
      0 = disabled
      1 = enabled
     *************************************************************************/
    camera.setColorbar(0);
#endif

    camera.showRegisters();

    showCommandList();
}

bool hm0360_flexio_callback(void *pfb) {
    // Serial.println("Flexio callback");
    g_new_flexio_data = pfb;
    return true;
}

// Quick and Dirty
#define UPDATE_ON_CAMERA_FRAMES

inline uint16_t HTONS(uint16_t x) {
#if defined(ARDUCAM_CAMERA_OV2640) || defined(ARDUCAM_CAMERA_OV5640)
  return x;
#else  //byte reverse
  return ((x >> 8) & 0x00FF) | ((x << 8) & 0xFF00);
#endif
}

#if defined(ARDUCAM_CAMERA_OV7675) || defined(ARDUCAM_CAMERA_OV7670) ||        \
    defined(ARDUCAM_CAMERA_OV2640) || defined(ARDUCAM_CAMERA_GC2145) ||         \
    defined(ARDUCAM_CAMERA_OV5640)

volatile uint16_t *pfb_last_frame_returned = nullptr;

bool camera_flexio_callback_video(void *pfb) {
    pfb_last_frame_returned = (uint16_t *)pfb;
#ifdef UPDATE_ON_CAMERA_FRAMES
    if ((uint32_t)pfb_last_frame_returned >= 0x20200000u)
        arm_dcache_delete((void *)pfb_last_frame_returned,
                          FRAME_WIDTH * FRAME_HEIGHT * 2);
    int numPixels = camera.width() * camera.height();

    for (int i = 0; i < numPixels; i++)
        pfb_last_frame_returned[i] = HTONS(pfb_last_frame_returned[i]);

    tft.writeRect(0, 0, FRAME_WIDTH, FRAME_HEIGHT,
                  (uint16_t *)pfb_last_frame_returned);
    pfb_last_frame_returned = nullptr;
    tft.setOrigin(0, 0);
    uint16_t *pframebuf = tft.getFrameBuffer();
    if ((uint32_t)pframebuf >= 0x20200000u)
        arm_dcache_flush(pframebuf, FRAME_WIDTH * FRAME_HEIGHT);
#endif
    // Serial.print("#");
    return true;
}

void frame_complete_cb() {
    // Serial.print("@");
#ifndef UPDATE_ON_CAMERA_FRAMES
    if (!pfb_last_frame_returned)
        return;
    if ((uint32_t)pfb_last_frame_returned >= 0x20200000u)
        arm_dcache_delete(pfb_last_frame_returned,
                          FRAME_WIDTH * FRAME_HEIGHT * 2);
    tft.writeSubImageRectBytesReversed(0, 0, FRAME_WIDTH, FRAME_HEIGHT, 0, 0,
                                       FRAME_WIDTH, FRAME_HEIGHT,
                                       pfb_last_frame_returned);
    pfb_last_frame_returned = nullptr;
    uint16_t *pfb = tft.getFrameBuffer();
    if ((uint32_t)pfb >= 0x20200000u)
        arm_dcache_flush(pfb, FRAME_WIDTH * FRAME_HEIGHT);
#endif
}

#else
uint8_t *pfb_last_frame_returned = nullptr;

bool camera_flexio_callback_video(void *pfb) {
    pfb_last_frame_returned = (uint8_t *)pfb;
#ifdef UPDATE_ON_CAMERA_FRAMES
    tft.setOrigin(-2, -2);
    if ((uint32_t)pfb_last_frame_returned >= 0x20200000u)
        arm_dcache_delete(pfb_last_frame_returned, FRAME_WIDTH * FRAME_HEIGHT);

    tft.writeRect8BPP(0, 0, FRAME_WIDTH, FRAME_HEIGHT,
                      (uint8_t *)pfb_last_frame_returned, current_mono_palette);
    pfb_last_frame_returned = nullptr;
    tft.setOrigin(0, 0);
    uint16_t *pframebuf = tft.getFrameBuffer();
    if ((uint32_t)pframebuf >= 0x20200000u)
        arm_dcache_flush(pframebuf, FRAME_WIDTH * FRAME_HEIGHT);
#endif
    // Serial.print("#");
    return true;
}

void frame_complete_cb() {
    // Serial.print("@");
#ifndef UPDATE_ON_CAMERA_FRAMES
    if (!pfb_last_frame_returned)
        return;
    tft.setOrigin(-2, -2);
    if ((uint32_t)pfb_last_frame_returned >= 0x20200000u)
        arm_dcache_delete(pfb_last_frame_returned, FRAME_WIDTH * FRAME_HEIGHT);

    tft.writeRect8BPP(0, 0, FRAME_WIDTH, FRAME_HEIGHT,
                      (uint8_t *)pfb_last_frame_returned, current_mono_palette);
    pfb_last_frame_returned = nullptr;
    tft.setOrigin(0, 0);
    uint16_t *pfb = tft.getFrameBuffer();
    if ((uint32_t)pfb >= 0x20200000u)
        arm_dcache_flush(pfb, FRAME_WIDTH * FRAME_HEIGHT);
#endif
}
#endif

void loop() {
    char ch;
#if defined(USB_DUAL_SERIAL) || defined(USB_TRIPLE_SERIAL)
    while (SerialUSB1.available()) {
        ch = SerialUSB1.read();
        Serial.println(ch, HEX);
        if (0x30 == ch) {
            Serial.print(F("ACK CMD CAM start single shoot ... "));
            send_image(&SerialUSB1);
            Serial.println(F("READY. END"));
        }
    }
#endif
    if (Serial.available()) {
        ch = Serial.read();
        switch (ch) {
        case 'p': {
#if defined(USB_DUAL_SERIAL) || defined(USB_TRIPLE_SERIAL)
            send_raw();
            Serial.println("Image Sent!");
            ch = ' ';
#else
            Serial.println(
                "*** Only works in USB Dual or Triple Serial Mode ***");
#endif
            break;
        }
        case 'z': {
#if defined(USE_SDCARD)
            save_image_SD();
#endif
            break;
        }
        case 'j': {
#if (defined(USE_SDCARD) && (defined(ARDUCAM_CAMERA_OV2640) || defined(ARDUCAM_CAMERA_OV5640)))
            bool error = false;
            error = save_jpg_SD();
            if (!error)
                Serial.println("ERROR reading JPEG.  Try again....");
#endif
            break;
        }
        case 'b': {
#if defined(USE_SDCARD)
            memset((uint8_t *)frameBuffer, 0, sizeof(frameBuffer));
            camera.setMode(HIMAX_MODE_STREAMING_NFRAMES, 1);
            camera.readFrame(frameBuffer, sizeof(frameBuffer));
#if defined(ARDUCAM_CAMERA_OV7675) || defined(ARDUCAM_CAMERA_OV7670) ||        \
    defined(ARDUCAM_CAMERA_OV2640) || defined(ARDUCAM_CAMERA_GC2145) ||         \
    defined(ARDUCAM_CAMERA_OV5640) 
            int numPixels = camera.width() * camera.height();
            for (int i = 0; i < numPixels; i++)
                frameBuffer[i] = HTONS(frameBuffer[i]);
#endif
            save_image_SD();
            ch = ' ';
#endif
            break;
        }
        case 'm':
            read_display_multiple_frames(false);
            break;

        case 'M':
            read_display_multiple_frames(true);
            break;
        case 't':
            tft.fillScreen(TFT_RED);
            delay(500);
            tft.fillScreen(TFT_GREEN);
            delay(500);
            tft.fillScreen(TFT_BLUE);
            delay(500);
            tft.fillScreen(TFT_BLACK);
            delay(500);
            break;
        case 'd':
            camera.debug(!camera.debug());
            if (camera.debug())
                Serial.println("Camera Debug turned on");
            else
                Serial.println("Camera debug turned off");
            break;

        case 'f': {
            memset((uint8_t *)frameBuffer, 0, sizeof(frameBuffer));
            camera.setMode(HIMAX_MODE_STREAMING_NFRAMES, 1);
            tft.useFrameBuffer(false);
            tft.fillScreen(TFT_BLACK);
            Serial.println("Reading frame");
            Serial.printf("Buffer: %p halfway: %p end:%p\n", frameBuffer,
                          &frameBuffer[camera.width() * camera.height() / 2],
                          &frameBuffer[camera.width() * camera.height()]);
            memset((uint8_t *)frameBuffer, 0, sizeof(frameBuffer));
            for (uint8_t i = 0; i < skipFrames; i++) {
                camera.readFrame(frameBuffer, sizeof(frameBuffer), frameBuffer2, sizeof(frameBuffer2));
            }

            Serial.println("Finished reading frame");
            Serial.flush();
#if (defined(ARDUCAM_CAMERA_OV7675) || defined(ARDUCAM_CAMERA_OV7670) ||        \
    defined(ARDUCAM_CAMERA_OV2640)  || defined(ARDUCAM_CAMERA_GC2145) ||        \
    defined(ARDUCAM_CAMERA_OV5640))
            #ifdef ARDUINO_TEENSY41
            // See what which camera buffer was used.
            frameBufferRead = (uint16_t*)camera.readFrameReturnBuffer();
            #else 
            frameBufferRead = frameBuffer;
            #endif
            for (volatile uint16_t *pfb = frameBufferRead;
                 pfb < (frameBufferRead + 4 * camera.width());
                 pfb += camera.width()) {
#else
            #ifdef ARDUINO_TEENSY41
            // See what which camera buffer was used.
            frameBufferRead = (uint8_t*)camera.readFrameReturnBuffer();
            if (frameBufferRead == nullptr) frameBufferRead = frameBuffer;
            #else 
            frameBufferRead = frameBuffer;
            #endif
            for (volatile uint8_t *pfb = frameBufferRead;
                 pfb < (frameBufferRead + 4 * camera.width());
                 pfb += camera.width()) {
#endif
                Serial.printf("\n%08x: ", (uint32_t)pfb);
                for (uint16_t i = 0; i < 8; i++)
                    Serial.printf("%02x ", pfb[i]);
                Serial.print("..");
                Serial.print("..");
                for (uint16_t i = camera.width() - 8; i < camera.width(); i++)
                    Serial.printf("%04x ", pfb[i]);
            }
            Serial.println("\n");

            // Lets dump out some of center of image.
            Serial.println("Show Center pixels\n");
#if defined(ARDUCAM_CAMERA_OV7675) || defined(ARDUCAM_CAMERA_OV7670) ||        \
    defined(ARDUCAM_CAMERA_OV2640) || defined(ARDUCAM_CAMERA_GC2145) ||         \
     defined(ARDUCAM_CAMERA_OV5640)
            for (volatile uint16_t *pfb =
                     frameBufferRead + camera.width() * ((camera.height() / 2) - 8);
                 pfb <
                 (frameBufferRead + camera.width() * (camera.height() / 2 + 8));
                 pfb += camera.width()) {
#else
            for (volatile uint8_t *pfb =
                     frameBufferRead + camera.width() * ((camera.height() / 2) - 8);
                 pfb <
                 (frameBufferRead + camera.width() * (camera.height() / 2 + 8));
                 pfb += camera.width()) {
#endif
                Serial.printf("\n%08x: ", (uint32_t)pfb);
                for (uint16_t i = 0; i < 8; i++)
                    Serial.printf("%02x ", pfb[i]);
                Serial.print("..");
                for (uint16_t i = (camera.width() / 2) - 4;
                     i < (camera.width() / 2) + 4; i++)
                    Serial.printf("%02x ", pfb[i]);
                Serial.print("..");
                for (uint16_t i = camera.width() - 8; i < camera.width(); i++)
                    Serial.printf("%02x ", pfb[i]);
            }
            Serial.println("\n...");

            // int numPixels = camera.width() * camera.height();
            Serial.printf("TFT(%u, %u) Camera(%u, %u)\n", tft.width(),
                          tft.height(), camera.width(), camera.height());
            // int camera_width = Camera.width();

            tft.setOrigin(-2, -2);
#if defined(ARDUCAM_CAMERA_OV7675) || defined(ARDUCAM_CAMERA_OV7670) ||        \
    defined(ARDUCAM_CAMERA_OV2640) || defined(ARDUCAM_CAMERA_GC2145) ||         \
     defined(ARDUCAM_CAMERA_OV5640)
            int numPixels = camera.width() * camera.height();
            Serial.printf("TFT(%u, %u) Camera(%u, %u)\n", tft.width(),
                          tft.height(), camera.width(), camera.height());

// int camera_width = Camera.width();
#if 1
            // byte swap
            // for (int i = 0; i < numPixels; i++) frameBufferRead[i] =
            // (frameBufferRead[i] >> 8) | (((frameBufferRead[i] & 0xff) << 8));
            for (int i = 0; i < numPixels; i++)
                frameBufferRead[i] = HTONS(frameBufferRead[i]);

            if ((camera.width() <= tft.width()) &&
                (camera.height() <= tft.height())) {
                if ((camera.width() != tft.width()) ||
                    (camera.height() != tft.height()))
                    tft.fillScreen(TFT_BLACK);
                tft.writeRect(CENTER, CENTER, camera.width(), camera.height(),
                              frameBufferRead);
            } else {
                Serial.println("sub image");
                tft.writeSubImageRect(0, 0, tft.width(), tft.height(),
                                      (camera.width() - tft.width()) / 2,
                                      (camera.height() - tft.height()),
                                      camera.width(), camera.height(),
                                      frameBufferRead);
            }
#else
            Serial.println("sub image1");
            tft.writeSubImageRect(0, 0, tft.width(), tft.height(), 0, 0,
                                  camera.width(), camera.height(), pixels);
#endif
#else
            tft.writeRect8BPP(0, 0, FRAME_WIDTH, FRAME_HEIGHT, frameBufferRead,
                              current_mono_palette);
#endif
            tft.setOrigin(0, 0);
            ch = ' ';
            g_continuous_flex_mode = false;
            break;
        }
        case 'F': {
            if (!g_continuous_flex_mode) {
                if (camera.readContinuous(&hm0360_flexio_callback, frameBuffer,
                                          sizeof(frameBuffer), frameBuffer2,
                                          sizeof(frameBuffer2))) {
                    Serial.println("* continuous mode started");
                    g_flexio_capture_count = 0;
                    g_flexio_redraw_count = 0;
                    g_continuous_flex_mode = true;
                } else {
                    Serial.println("* error, could not start continuous mode");
                }
            } else {
                camera.stopReadContinuous();
                g_continuous_flex_mode = false;
                Serial.println("* continuous mode stopped");
            }
            break;
        }
        case 'V': {
            if (!g_continuous_flex_mode) {
                if (camera.readContinuous(&camera_flexio_callback_video,
                                          frameBuffer, sizeof(frameBuffer),
                                          frameBuffer2, sizeof(frameBuffer2))) {

                    Serial.println("Before Set frame complete CB");
                    if (!tft.useFrameBuffer(true))
                        Serial.println("Failed call to useFrameBuffer");
                    tft.setFrameCompleteCB(&frame_complete_cb, false);
                    Serial.println("Before UPdateScreen Async");
                    tft.updateScreenAsync(true);
                    Serial.println("* continuous mode (Video) started");
                    g_flexio_capture_count = 0;
                    g_flexio_redraw_count = 0;
                    g_continuous_flex_mode = 2;
                } else {
                    Serial.println("* error, could not start continuous mode");
                }
            } else {
                camera.stopReadContinuous();
                tft.endUpdateAsync();
                tft.useFrameBuffer(false);
                g_continuous_flex_mode = 0;
                Serial.println("* continuous mode stopped");
            }
            ch = ' ';
            break;
        }
        case '1': {
            tft.fillScreen(TFT_BLACK);
            break;
        }
        case 0x30: {
#if defined(USB_DUAL_SERIAL) || defined(USB_TRIPLE_SERIAL)
            SerialUSB1.println(F("ACK CMD CAM start single shoot. END"));
            send_image(&SerialUSB1);
            SerialUSB1.println(F("READY. END"));
#else
            Serial.println(
                "*** Only works in USB Dual or Triple Serial Mode ***");
#endif
            break;
        }
        case '?': {
            showCommandList();
            ch = ' ';
            break;
        }
        default:
            break;
        }
        while (Serial.read() != -1)
            ; // lets strip the rest out
    }

    if (g_continuous_flex_mode) {
        if (g_new_flexio_data) {
            // Serial.println("new FlexIO data");
#ifndef CAMERA_USES_MONO_PALETTE
            uint16_t *pframe = (uint16_t *)g_new_flexio_data;
            for (int i = 0; i < (FRAME_WIDTH * FRAME_HEIGHT); i++)
                pframe[i] = HTONS(pframe[i]);
            tft.writeRect(CENTER, CENTER, camera.width(), camera.height(),
                          pframe);

#else
            tft.setOrigin(-2, -2);
            tft.writeRect8BPP(0, 0, FRAME_WIDTH, FRAME_HEIGHT,
                              (uint8_t *)g_new_flexio_data, current_mono_palette);
            tft.setOrigin(0, 0);
#endif
            tft.updateScreenAsync();
            g_new_flexio_data = nullptr;
            g_flexio_redraw_count++;
            if (g_flexio_runtime > 10000) {
                // print some stats on actual speed, but not too much
                // printing too quickly to be considered "spew"
                float redraw_rate = (float)g_flexio_redraw_count /
                                    (float)g_flexio_runtime * 1000.0f;
                g_flexio_runtime = 0;
                g_flexio_redraw_count = 0;
                Serial.printf("redraw rate = %.2f Hz\n", redraw_rate);
            }
        }
    }
}

// Pass 8-bit (each) R,G,B, get back 16-bit packed color
uint16_t color565(uint8_t r, uint8_t g, uint8_t b) {
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

#if defined(USB_DUAL_SERIAL) || defined(USB_TRIPLE_SERIAL)
void send_image(Stream *imgSerial) {
    memset((uint8_t *)frameBuffer, 0, sizeof(frameBuffer));
    camera.setHmirror(1);
    camera.readFrame(frameBuffer, sizeof(frameBuffer));

    imgSerial->write(0xFF);
    imgSerial->write(0xAA);

    // BUGBUG:: maybe combine with the save to SD card code
    unsigned char bmpFileHeader[14] = {'B', 'M', 0, 0,  0, 0, 0,
                                       0,   0,   0, 54, 0, 0, 0};
    unsigned char bmpInfoHeader[40] = {40, 0, 0, 0, 0, 0, 0,  0,
                                       0,  0, 0, 0, 1, 0, 24, 0};

    int rowSize = 4 * ((3 * FRAME_WIDTH + 3) /
                       4); // how many bytes in the row (used to create padding)
    int fileSize =
        54 + FRAME_HEIGHT * rowSize; // headers (54 bytes) + pixel data

    bmpFileHeader[2] = (unsigned char)(fileSize);
    bmpFileHeader[3] = (unsigned char)(fileSize >> 8);
    bmpFileHeader[4] = (unsigned char)(fileSize >> 16);
    bmpFileHeader[5] = (unsigned char)(fileSize >> 24);

    bmpInfoHeader[4] = (unsigned char)(FRAME_WIDTH);
    bmpInfoHeader[5] = (unsigned char)(FRAME_WIDTH >> 8);
    bmpInfoHeader[6] = (unsigned char)(FRAME_WIDTH >> 16);
    bmpInfoHeader[7] = (unsigned char)(FRAME_WIDTH >> 24);
    bmpInfoHeader[8] = (unsigned char)(FRAME_HEIGHT);
    bmpInfoHeader[9] = (unsigned char)(FRAME_HEIGHT >> 8);
    bmpInfoHeader[10] = (unsigned char)(FRAME_HEIGHT >> 16);
    bmpInfoHeader[11] = (unsigned char)(FRAME_HEIGHT >> 24);

    imgSerial->write(bmpFileHeader, sizeof(bmpFileHeader)); // write file header
    imgSerial->write(bmpInfoHeader, sizeof(bmpInfoHeader)); // " info header

    unsigned char bmpPad[rowSize - 3 * FRAME_WIDTH];
    for (int i = 0; i < (int)(sizeof(bmpPad)); i++) { // fill with 0s
        bmpPad[i] = 0;
    }

    uint32_t idx = 0;
#ifdef CAMERA_USES_MONO_PALETTE
    uint8_t *pfb = frameBuffer;
    uint32_t count_y_first_buffer = sizeof(frameBuffer) / FRAME_WIDTH;
    uint8_t img[3];
    for (int y = FRAME_HEIGHT - 1; y >= 0; y--) { // iterate image array
        if (y < (int)count_y_first_buffer)
            pfb = &frameBuffer[y * FRAME_WIDTH];
        for (int x = 0; x < FRAME_WIDTH; x++) {
            // r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3
            img[2] = *pfb; // r
            img[1] = *pfb; // g
            img[0] = *pfb; // b
            imgSerial->write(img, 3);
            delayMicroseconds(8);
            pfb++;
        }
        imgSerial->write(bmpPad,
                         (4 - (FRAME_WIDTH * 3) % 4) %
                             4); // and padding as needed
    }
#else
    uint16_t *pfb = frameBuffer;
    uint8_t img[3];
    uint32_t count_y_first_buffer = sizeof(frameBuffer) / (FRAME_WIDTH * 2);
    for (int y = FRAME_HEIGHT - 1; y >= 0; y--) { // iterate image array
        if (y < (int)count_y_first_buffer)
            pfb = &frameBuffer[y * FRAME_WIDTH];
        for (int x = 0; x < FRAME_WIDTH; x++) {
            // r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3
            uint16_t pixel = HTONS(*pfb++);
            img[2] = (pixel >> 8) & 0xf8; // r
            img[1] = (pixel >> 3) & 0xfc; // g
            img[0] = (pixel << 3);        // b
            imgSerial->write(img, 3);
            delayMicroseconds(8);
        }
        imgSerial->write(bmpPad,
                         (4 - (FRAME_WIDTH * 3) % 4) %
                             4); // and padding as needed
    }
#endif

    imgSerial->write(0xBB);
    imgSerial->write(0xCC);

    imgSerial->println(F("ACK CMD CAM Capture Done. END"));
    camera.setHmirror(0);

    delay(50);
}

void send_raw() {
    memset((uint8_t *)frameBuffer, 0, sizeof(frameBuffer));
    camera.readFrame(frameBuffer, sizeof(frameBuffer));
    uint32_t idx = 0;
#ifdef CAMERA_USES_MONO_PALETTE
    uint32_t image_idx = 0;
    uint32_t frame_idx = 0;
    for (uint32_t row = 0; row < FRAME_HEIGHT; row++) {
        for (uint32_t col = 0; col < FRAME_WIDTH; col++) {
            frame_idx = ((FRAME_WIDTH + 4) * (row + 2)) + col + 2;
            uint16_t framePixel =
                color565(frameBuffer[frame_idx], frameBuffer[frame_idx],
                         frameBuffer[frame_idx]);
            SerialUSB1.write((framePixel) & 0xFF);
            SerialUSB1.write((framePixel >> 8) & 0xFF);
            delayMicroseconds(8);
        }
    }
#else
    for (int i = 0; i < FRAME_HEIGHT * FRAME_WIDTH; i++) {
        idx = i * 2;
        SerialUSB1.write((frameBuffer[i] >> 8) & 0xFF);
        SerialUSB1.write((frameBuffer[i]) & 0xFF);
    }
#endif
}
#endif

#if defined(USE_SDCARD)
char name[] = "9px_0000.bmp"; // filename convention (will auto-increment)
                              // can probably reuse framebuffer2...

// DMAMEM unsigned char img[3 * 320*240];
void save_image_SD() {
    // uint8_t r, g, b;
    // uint32_t x, y;

    Serial.print("Writing BMP to SD CARD File: ");

    // if name exists, create new filename, SD.exists(filename)
    for (int i = 0; i < 10000; i++) {
        name[4] = (i / 1000) % 10 + '0'; // thousands place
        name[5] = (i / 100) % 10 + '0';  // hundreds
        name[6] = (i / 10) % 10 + '0';   // tens
        name[7] = i % 10 + '0';          // ones
        if (!SD.exists(name)) {
            Serial.println(name);
            file = SD.open(name, FILE_WRITE);
            break;
        }
    }

    uint16_t w = FRAME_WIDTH;
    uint16_t h = FRAME_HEIGHT;

    // unsigned char *img = NULL;
    // set fileSize (used in bmp header)
    int rowSize = 4 * ((3 * w + 3) /
                       4); // how many bytes in the row (used to create padding)
    int fileSize = 54 + h * rowSize; // headers (54 bytes) + pixel data

    //  img = (unsigned char *)malloc(3 * w * h);

    // create padding (based on the number of pixels in a row
    unsigned char bmpPad[rowSize - 3 * w];
    for (int i = 0; i < (int)(sizeof(bmpPad)); i++) { // fill with 0s
        bmpPad[i] = 0;
    }

    unsigned char bmpFileHeader[14] = {'B', 'M', 0, 0,  0, 0, 0,
                                       0,   0,   0, 54, 0, 0, 0};
    unsigned char bmpInfoHeader[40] = {40, 0, 0, 0, 0, 0, 0,  0,
                                       0,  0, 0, 0, 1, 0, 24, 0};

    bmpFileHeader[2] = (unsigned char)(fileSize);
    bmpFileHeader[3] = (unsigned char)(fileSize >> 8);
    bmpFileHeader[4] = (unsigned char)(fileSize >> 16);
    bmpFileHeader[5] = (unsigned char)(fileSize >> 24);

    bmpInfoHeader[4] = (unsigned char)(w);
    bmpInfoHeader[5] = (unsigned char)(w >> 8);
    bmpInfoHeader[6] = (unsigned char)(w >> 16);
    bmpInfoHeader[7] = (unsigned char)(w >> 24);
    bmpInfoHeader[8] = (unsigned char)(h);
    bmpInfoHeader[9] = (unsigned char)(h >> 8);
    bmpInfoHeader[10] = (unsigned char)(h >> 16);
    bmpInfoHeader[11] = (unsigned char)(h >> 24);

    // write the file (thanks forum!)
    file.write(bmpFileHeader, sizeof(bmpFileHeader)); // write file header
    file.write(bmpInfoHeader, sizeof(bmpInfoHeader)); // " info header

// try to compute and output one row at a time.
#ifdef CAMERA_USES_MONO_PALETTE
    uint8_t *pfb = frameBuffer;
    uint8_t img[3];
    for (int y = h - 1; y >= 0; y--) { // iterate image array
        pfb = &frameBuffer[y * w];
        for (int x = 0; x < w; x++) {
            // r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3
            img[2] = *pfb; // r
            img[1] = *pfb; // g
            img[0] = *pfb; // b
            file.write(img, 3);
            pfb++;
        }
#else
    uint16_t *pfb = frameBuffer;
    uint8_t img[3];
    for (int y = h - 1; y >= 0; y--) { // iterate image array
        pfb = &frameBuffer[y * w];
        for (int x = 0; x < w; x++) {
            // r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3
            img[2] = (*pfb >> 8) & 0xf8; // r
            img[1] = (*pfb >> 3) & 0xfc; // g
            img[0] = (*pfb << 3);        // b
            file.write(img, 3);
            pfb++;
        }
#endif
        file.write(bmpPad,
                   (4 - (FRAME_WIDTH * 3) % 4) % 4); // and padding as needed
    }
    file.close(); // close file when done writing
    Serial.println("Done Writing BMP");
}

char name_jpg[] = "9px_0000.jpg"; // filename convention (will auto-increment)
                                  // can probably reuse framebuffer2...

bool save_jpg_SD() {

    // omni.setQuality(11);
    camera.useDMA(false);

    memset((uint8_t *)frameBuffer, 0, sizeof(frameBuffer));
    if (camera.usingGPIO()) {
        omni.readFrameGPIO_JPEG(frameBuffer, sizeof(frameBuffer));
        delay(100);
        omni.readFrameGPIO_JPEG(frameBuffer, sizeof(frameBuffer));
    } else {
        camera.readFrame(frameBuffer, sizeof(frameBuffer));
        delay(100);
        camera.readFrame(frameBuffer, sizeof(frameBuffer));
    }

    uint16_t w = FRAME_WIDTH;
    uint16_t h = FRAME_HEIGHT;
    uint16_t eop = 0;
    uint8_t eoi = 0;

    uint16_t *pfb = frameBuffer;
    Serial.printf("jpeg size: %d\n", (w * h / 5));

    for (uint32_t i = 0; i < (w * h / 5); i++) {
        // Serial.println(pfb[i], HEX);
        if ((i == 0) && (pfb[0] == 0xD8FF)) {
            eoi = 1;
            Serial.printf("Found begining of frame at position %d\n", i);
        }

        if (pfb[i] == 0xD9FF) {
            eop = i;
            Serial.printf("Found ending of frame at position %d\n", i);
        }
    }

    if (eop == 0 || eoi == 0)
        return false;

    Serial.print("Writing jpg to SD CARD File: ");

    // if name exists, create new filename, SD.exists(filename)
    for (int i = 0; i < 10000; i++) {
        name_jpg[4] = (i / 1000) % 10 + '0'; // thousands place
        name_jpg[5] = (i / 100) % 10 + '0';  // hundreds
        name_jpg[6] = (i / 10) % 10 + '0';   // tens
        name_jpg[7] = i % 10 + '0';          // ones
        if (!SD.exists(name_jpg)) {
            Serial.println(name_jpg);
            file = SD.open(name_jpg, FILE_WRITE);
            break;
        }
    }

    Serial.println(" Writing to SD");
    for (uint32_t i = 0; i < (w * h / 5); i++) {
        // file.write(pfb[i]);
        file.write(pfb[i] & 0xFF);
        file.write((pfb[i] >> 8) & 0xFF);
    }

    file.close(); // close file when done writing
    Serial.println("Done Writing JPG");

    Serial.printf("%x, %x\n", pfb[0], pfb[eop]);

    return true;
}
#endif

void showCommandList() {
    if (camera.usingGPIO()) {
        Serial.println("Send the 'f' character to read a frame using GPIO");
        Serial.println("Send the 'F' to start/stop continuous using GPIO");
    } else {
        Serial.println("Send the 'f' character to read a frame using FlexIO "
                       "(changes hardware setup!)");
        Serial.println("Send the 'F' to start/stop continuous using FlexIO "
                       "(changes hardware setup!)");
    }
    Serial.println(
        "Send the 'm' character to read and display multiple frames");
    Serial.println("Send the 'M' character to read and display multiple frames "
                   "use Frame buffer");
    Serial.println("Send the 'V' character DMA to TFT async continueous  ...");
    Serial.println("Send the 'p' character to snapshot to PC on USB1");
    Serial.println("Send the 'b' character to save snapshot (BMP) to SD Card");
    Serial.println("Send the 'j' character to save snapshot (JPEG) to SD Card");
    Serial.println("Send the '1' character to blank the display");
    Serial.println("Send the 'z' character to send current screen BMP to SD");
    Serial.println("Send the 't' character to send Check the display");
    Serial.println("Send the 'd' character to toggle camera debug on and off");
    Serial.println();
}

//=============================================================================
void read_display_multiple_frames(bool use_frame_buffer) {
    if (use_frame_buffer) {
        Serial.println(
            "\n*** Read and display multiple frames (using async screen "
            "updates), press any key to stop ***");
        tft.useFrameBuffer(true);
    } else {
        Serial.println("\n*** Read and display multiple frames, press any key "
                       "to stop ***");
    }

    while (Serial.read() != -1) {
    }

    elapsedMicros em = 0;
    int frame_count = 0;

    for (;;) {

        for (uint8_t i = 0; i < skipFrames; i++)
            camera.readFrame(frameBuffer, sizeof(frameBuffer), frameBuffer2, sizeof(frameBuffer2));

        int numPixels = camera.width() * camera.height();

// byte swap
// for (int i = 0; i < numPixels; i++) frameBuffer[i] = (frameBuffer[i] >> 8) |
// (((frameBuffer[i] & 0xff) << 8));
#ifdef CAMERA_USES_MONO_PALETTE
        uint8_t *frameBufferRead = (uint8_t*)camera.readFrameReturnBuffer();
        if (frameBufferRead == nullptr) frameBufferRead = frameBuffer;
        Serial.printf("NP:%d W:%u H:%u %p\n", numPixels, FRAME_WIDTH,
                      FRAME_HEIGHT, frameBufferRead);
        if (use_frame_buffer)
            tft.waitUpdateAsyncComplete();
        tft.setOrigin(-2, -2);
        tft.writeRect8BPP(0, 0, FRAME_WIDTH, FRAME_HEIGHT, frameBufferRead,
                          current_mono_palette);
        tft.setOrigin(0, 0);
#else
        uint16_t *frameBufferRead = (uint16_t*)camera.readFrameReturnBuffer();
        if (frameBufferRead == nullptr) frameBufferRead = frameBuffer;

        for (int i = 0; i < numPixels; i++)
            frameBufferRead[i] = HTONS(frameBufferRead[i]);

        if (use_frame_buffer)
            tft.waitUpdateAsyncComplete();

        if ((camera.width() <= tft.width()) &&
            (camera.height() <= tft.height())) {
            if ((camera.width() != tft.width()) ||
                (camera.height() != tft.height()))
                tft.fillScreen(TFT_BLACK);
            tft.writeRect(CENTER, CENTER, camera.width(), camera.height(),
                          frameBufferRead);
        } else {
            tft.writeSubImageRect(0, 0, tft.width(), tft.height(),
                                  (camera.width() - tft.width()) / 2,
                                  (camera.height() - tft.height()),
                                  camera.width(), camera.height(), frameBufferRead);
        }
#endif

        if (use_frame_buffer)
            tft.updateScreenAsync();

        frame_count++;
        if ((frame_count & 0x7) == 0) {
            Serial.printf("Elapsed: %u frames: %d fps: %.2f\n", (uint32_t)em,
                          frame_count,
                          (float)(1000000.0 / em) * (float)frame_count);
        }
        if (Serial.available())
            break;
    }
    // turn off frame buffer
    tft.useFrameBuffer(false);
}
