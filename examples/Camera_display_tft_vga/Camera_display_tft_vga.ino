#include <SD.h>
#include <SPI.h>
#include <stdint.h>

#include "Camera.h"

#define USE_MMOD_ATP_ADAPTER
#define USE_SDCARD

// #define ARDUCAM_CAMERA_HM01B0
// #define ARDUCAM_CAMERA_HM0360
#define ARDUCAM_CAMERA_OV2640
// #define ARDUCAM_CAMERA_OV7670
// #define ARDUCAM_CAMERA_OV7675
// #define ARDUCAM_CAMERA_GC2145

#if defined(ARDUCAM_CAMERA_HM0360)
#include "TMM_HM0360/HM0360.h"
HM0360 himax;
Camera camera(himax);
#define CameraID 0x0360
#define SCREEN_ROTATION 3
#define MIRROR_FLIP_CAMERA

#elif defined(ARDUCAM_CAMERA_HM01B0)
#include "TMM_HM01B0/HM01B0.h"
HM01B0 himax;
Camera camera(himax);
#define CameraID 0x01B0
#define SCREEN_ROTATION 3
#define MIRROR_FLIP_CAMERA

#elif defined(ARDUCAM_CAMERA_OV2640)
#include "TMM_OV2640/OV2640.h"
OV2640 omni;
Camera camera(omni);
#define CameraID 0x2642
#define MIRROR_FLIP_CAMERA
#define SCREEN_ROTATION 3
pixformat_t camera_format = RGB565;

#elif defined(ARDUCAM_CAMERA_OV7670)
#include "TMM_OV767X/OV767X.h"
OV767X omni;
Camera camera(omni);
#define CameraID 0x7676
#define SCREEN_ROTATION 3

#elif defined(ARDUCAM_CAMERA_OV7675)
#include "TMM_OV767X/OV767X.h"
OV767X omni;
Camera camera(omni);
#define CameraID 0x7673
#define SCREEN_ROTATION 3

#elif defined(ARDUCAM_CAMERA_GC2145)
#include "TMM_GC2145/GC2145.h"
GC2145 galaxycore;
Camera camera(galaxycore);
#define CameraID 0x2145
#define SCREEN_ROTATION 3
pixformat_t camera_format = RGB565;

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
#define _hmConfig 3 // select mode string below
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

// Set up ILI9488
#ifdef ARDUINO_TEENSY_DEVBRD4
#undef USE_MMOD_ATP_ADAPTER
#define SD_CS 27
#define TFT_CS 10 // AD_B0_02
#define TFT_DC 25 // AD_B0_03
#define TFT_RST 24
#define VSYNC_PIN 21

#elif defined(USE_MMOD_ATP_ADAPTER)
#define VSYNC_PIN 33
#define TFT_DC 4  // 0   // "TX1" on left side of Sparkfun ML Carrier
#define TFT_CS 5  // 4   // "CS" on left side of Sparkfun ML Carrier
#define TFT_RST 2 // 1  // "RX1" on left side of Sparkfun ML Carrier
#else
#define VSYNC_PIN 33
#define TFT_DC 0  // 20   // "TX1" on left side of Sparkfun ML Carrier
#define TFT_CS 4  // 5, 4   // "CS" on left side of Sparkfun ML Carrier
#define TFT_RST 1 // 2, 1  // "RX1" on left side of Sparkfun ML Carrier
#endif

#include <ILI9488_t3.h>
ILI9488_t3 tft = ILI9488_t3(TFT_CS, TFT_DC, TFT_RST);
#define TFT_BLACK ILI9488_BLACK
#define TFT_YELLOW ILI9488_YELLOW
#define TFT_RED ILI9488_RED
#define TFT_GREEN ILI9488_GREEN
#define TFT_BLUE ILI9488_BLUE
#define CENTER ILI9488_t3::CENTER

// Setup framebuffers
DMAMEM uint16_t FRAME_WIDTH, FRAME_HEIGHT;
#ifdef ARDUINO_TEENSY_DEVBRD4
#if defined(ARDUCAM_CAMERA_OV7675) || defined(ARDUCAM_CAMERA_OV7670) ||        \
    defined(ARDUCAM_CAMERA_OV2640) || defined(ARDUCAM_CAMERA_GC2145)
uint16_t *frameBuffer = nullptr;
uint16_t *frameBuffer2 = nullptr;
uint16_t *frameBufferSDRAM = nullptr;
uint16_t *frameBufferSDRAM2 = nullptr;
DMAMEM uint16_t frameBufferM[640 * 240] __attribute__((aligned(32)));
uint16_t frameBufferM2[640 * 240] __attribute__((aligned(32)));

// #define SCREEN_ROTATION 3
#else
uint8_t *frameBuffer = nullptr;
uint8_t *frameBuffer2 = nullptr;
DMAMEM uint8_t frameBufferM[640 * 480] __attribute__((aligned(32)));
// mono can fit one in each.
uint8_t frameBufferM2[640 * 480] __attribute__((aligned(32)));
// #define SCREEN_ROTATION 1
#define CAMERA_USES_MONO_PALETTE
#endif
uint32_t sizeof_framebuffer = 0;
uint32_t sizeof_framebuffer2 = 0;
uint32_t sizeof_framebufferSDRAM = 0;
#else
#if defined(ARDUCAM_CAMERA_OV7675) || defined(ARDUCAM_CAMERA_OV7670) ||        \
    defined(ARDUCAM_CAMERA_OV2640) || defined(ARDUCAM_CAMERA_GC2145)
// only half buffer will fit in each of the two main memory regions
// split into two parts, part dmamem and part fast mememory to fit 640x480x2
DMAMEM uint16_t frameBuffer[640 * 240] __attribute__((aligned(32)));
uint16_t frameBuffer2[640 * 240] __attribute__((aligned(32)));
#else
// split into two parts, part dmamem and part fast mememory to fit 640x480x2
// mono can fit one in each.
#if 1 // Test two full size buffers
DMAMEM uint8_t frameBuffer[640 * 480] __attribute__((aligned(32)));
uint8_t frameBuffer2[640 * 480] __attribute__((aligned(32)));
#else
// Try putting both in main memory as half size again, to test both probably
// needed to test V in full screen mode
uint8_t frameBuffer[640 * 240] __attribute__((aligned(32)));
uint8_t frameBuffer2[640 * 240] __attribute__((aligned(32)));
#endif

#define CAMERA_USES_MONO_PALETTE
// #define SCREEN_ROTATION 1
#endif
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
  pinMode(0, OUTPUT);
  digitalWriteFast(0, LOW);
  while (!Serial && millis() < 5000) {
  }
  Serial.begin(921600);
#if defined(USB_DUAL_SERIAL) || defined(USB_TRIPLE_SERIAL)
  SerialUSB1.begin(921600);
#endif
  if (CrashReport) {
    Serial.print(CrashReport);
    Serial.println("Press any key to continue");
    while (Serial.read() != -1) {
    }
    while (Serial.read() == -1) {
    }
    while (Serial.read() != -1) {
    }
  }
#ifdef SD_CS
  pinMode(SD_CS, OUTPUT);
  digitalWriteFast(SD_CS, HIGH);
  pinMode(TFT_CS, OUTPUT);
  digitalWriteFast(TFT_CS, HIGH);
#endif
#if defined(USE_SDCARD)
  Serial.println("Using SDCARD - Initializing");
#ifdef SD_CS
  // #if MMOD_ML == 1
  Serial.printf("Start SD pin: %u\n", SD_CS);
  // if (!SD.begin(SD_CS)) {
  if (!SD.sdfs.begin(SdSpiConfig(SD_CS, SHARED_SPI, SD_SCK_MHZ(12)))) {
    Serial.println("initialization failed!");
  }

#else
  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("initialization failed!");
  }
#endif
  // while (1){
  //    LEDON; delay(100);
  //    LEDOFF; delay(100);
  //  }
  Serial.println("initialization done.");
  delay(100);
#endif

  tft.begin(15000000);

  tft.setRotation(SCREEN_ROTATION);
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

  while (!Serial)
    ;
  Serial.println("TeensyMM Camera Test");
  Serial.println(hmConfig[_hmConfig]);
  Serial.println("------------------");

  delay(500);

  tft.fillScreen(TFT_BLACK);

/***************************************************************/
//    setPins(uint8_t mclk_pin, uint8_t pclk_pin, uint8_t vsync_pin, uint8_t
//    hsync_pin, en_pin, uint8_t g0, uint8_t g1,uint8_t g2, uint8_t g3, uint8_t
//    g4=0xff, uint8_t g5=0xff,uint8_t g6=0xff,uint8_t g7=0xff);
#ifdef USE_MMOD_ATP_ADAPTER
  pinMode(30, INPUT);
  pinMode(31, INPUT_PULLUP);

  if ((_hmConfig == 0) || (_hmConfig == 2)) {
    camera.setPins(29, 10, 33, 32, 31, 40, 41, 42, 43, 44, 45, 6, 9);
  } else if ((_hmConfig == 1) || (_hmConfig == 3)) {
    // camera.setPins(7, 8, 33, 32, 17, 40, 41, 42, 43);
    camera.setPins(29, 10, 33, 32, 31, 40, 41, 42, 43);
  }
#elif defined(ARDUINO_TEENSY_DEVBRD4)
  pinMode(23, INPUT_PULLUP);
  if ((_hmConfig == 0) || (_hmConfig == 2)) {
    camera.setPins(7, 8, 21, 46, 23, 40, 41, 42, 43, 44, 45, 6, 9);
  } else if (_hmConfig == 1) {
    // camera.setPins(7, 8, 33, 32, 17, 40, 41, 42, 43);
    camera.setPins(7, 8, 21, 46, 31, 40, 41, 42, 43);
  }

#else
  if (_hmConfig == 0) {
    // camera.setPins(29, 10, 33, 32, 31, 40, 41, 42, 43, 44, 45, 6, 9);
    camera.setPins(7, 8, 33, 32, 17, 40, 41, 42, 43, 44, 45, 6, 9);
  } else if (_hmConfig == 1) {
    // camera.setPins(7, 8, 33, 32, 17, 40, 41, 42, 43);
    camera.setPins(29, 10, 33, 32, 31, 40, 41, 42, 43);
  }
#endif

  //  FRAMESIZE_VGA = 0,
  //  FRAMESIZE_QQVGA,    // 160x120
  //  FRAMESIZE_QVGA,     // 320x240
  //  FRAMESIZE_480X320,
  //  FRAMESIZE_320X320,  // 320x320
  //  FRAMESIZE_QVGA4BIT,
  //  FRAMESIZE_QCIF,
  //  FRAMESIZE_CIF,
  //  FRAMESIZE_SVGA, //800, 600
  //  FRAMESIZE_UXGA, //1500, 1200

  uint8_t status = 0;
#if defined(ARDUCAM_CAMERA_OV7675) || defined(ARDUCAM_CAMERA_OV7670) ||        \
    defined(ARDUCAM_CAMERA_OV2640) || defined(ARDUCAM_CAMERA_GC2145)
  // VGA mode
#if defined(ARDUCAM_CAMERA_GC2145) || defined(ARDUCAM_CAMERA_OV2640)
  status = camera.begin(FRAMESIZE_SVGA, camera_format, 15, false);
  camera.setZoomWindow(-1, -1, 480, 320);
#else
  status = camera.begin(FRAMESIZE_VGA, RGB565, 15, false);
#endif
#elif defined(ARDUCAM_CAMERA_HM0360)
  status = camera.begin(FRAMESIZE_VGA, 5);
#else
  // 1b0 defaults to qvga 4 bit
  // HM0360(4pin) 15/30 @6mhz, 60 works but get 4 pics on one screen :)
  // HM0360(8pin) 15/30/60/120 works :)
  // HM01B0(4pin only) 15/30/60 works, 120 not supported
  // camera.begin(FRAMESIZE_QVGA, 30);
  status = camera.begin(FRAMESIZE_QVGA4BIT, 15, false);
#endif

  Serial.printf("Begin status: %d\n", status);
  if (!status) {
    Serial.println("Camera failed to start!!!");
    while (1) {
    }
  }

#ifdef MIRROR_FLIP_CAMERA
  camera.setHmirror(true);
  camera.setVflip(true);
#endif
  // galaxycore.setFramesize(800, 600);

#if defined(ARDUINO_TEENSY_DEVBRD4)
#if defined(ARDUCAM_CAMERA_OV7675) || defined(ARDUCAM_CAMERA_OV7670) ||        \
    defined(ARDUCAM_CAMERA_OV2640) || defined(ARDUCAM_CAMERA_GC2145)

  sizeof_framebufferSDRAM = sizeof_framebuffer = sizeof_framebuffer2 =
      camera.width() * camera.height() * 2;
  frameBufferSDRAM = frameBuffer = (uint16_t *)((
      ((uint32_t)(sdram_malloc(camera.width() * camera.height() * 2 + 32)) +
       32) &
      0xffffffe0));
  frameBufferSDRAM2 = frameBuffer2 = (uint16_t *)((
      ((uint32_t)(sdram_malloc(camera.width() * camera.height() * 2 + 32)) +
       32) &
      0xffffffe0));
#else
  sizeof_framebufferSDRAM = sizeof_framebuffer = sizeof_framebuffer2 =
      camera.width() * camera.height();
  frameBufferSDRAM = frameBuffer = (uint8_t *)((
      ((uint32_t)(sdram_malloc(camera.width() * camera.height() + 32)) + 32) &
      0xffffffe0));
  frameBufferSDRAM2 = frameBuffer2 = (uint8_t *)((
      ((uint32_t)(sdram_malloc(camera.width() * camera.height() + 32)) + 32) &
      0xffffffe0));
#endif
  Serial.printf("Camera Buffers: %p %p\n", frameBuffer, frameBuffer2);
#endif

#ifndef CAMERA_USES_MONO_PALETTE
// #if (defined(ARDUCAM_CAMERA_OV7675) || defined(ARDUCAM_CAMERA_OV7670))
#if defined(ARDUCAM_CAMERA_OV2640)
// camera.setBrightness(0);          // -2 to +2
// camera.setContrast(0);            // -2 to +2
// camera.setSaturation(0);          // -2 to +2
// omni.setSpecialEffect(RETRO);  // NOEFFECT, NEGATIVE, BW, REDDISH, GREEISH,
// BLUEISH, RETRO omni.setWBmode(0);                  // AWB ON, 1 - Sunny, 2 -
// Cloudy, 3 - Office, 4 - Home
#else
  camera.setContrast(0x30);
  camera.setBrightness(0x80);
  camera.autoExposure(1);
#endif
#else
  camera.setGainceiling(GAINCEILING_2X);
  camera.setBrightness(3);
  camera.setAutoExposure(
      true, 1500);    // higher the setting the less saturaturation of whiteness
  camera.cmdUpdate(); // only need after changing auto exposure settings
  camera.setMode(HIMAX_MODE_STREAMING, 0); // turn on, continuous streaming mode
#endif

  Serial.println("Camera settings:");
  Serial.print("\twidth = ");
  Serial.println(camera.width());
  Serial.print("\theight = ");
  Serial.println(camera.height());
  Serial.printf("Camera Frame Width = %u Height = %u\n", camera.frameWidth(),
                camera.frameHeight());

#if (defined(ARDUCAM_CAMERA_OV7675) || defined(ARDUCAM_CAMERA_OV7670))
  uint8_t href = camera.readRegister(OV767X::REG_HREF);
  uint8_t vref = camera.readRegister(OV767X::REG_VREF);
  Serial.printf("\tHorizontal Frame START:%u STOP:%u\n",
                (camera.readRegister(OV767X::REG_HSTART) << 3) | (href & 0x7),
                (camera.readRegister(OV767X::REG_HSTOP) << 3) |
                    ((href >> 3) & 0x7));
  Serial.printf("\tVertical Frame START:%u STOP:%u\n",
                (camera.readRegister(OV767X::REG_VSTART) << 2) | (vref & 0x3),
                (camera.readRegister(OV767X::REG_VSTOP) << 2) |
                    ((vref >> 2) & 0x3));

#endif
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

  showCommandList();

#if defined(ARDUCAM_CAMERA_GC2145)
  /***************note for the GC2145 the following is supported **************
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
    GC2145_SOLID_MAGENTA
  ****************************************************************************/
  camera.setColorbar(GC2145_DISABLED);
#else
  /**************************************************************************
    0 = disabled
    1 = enabled
   *************************************************************************/
//  camera.setColorbar(GC2145_COLOR_BARS);
#endif
}

bool hm0360_flexio_callback(void *pfb) {
  // Serial.println("Flexio callback");
  //  Serial.printf("FCB:%p\n", pfb);
  g_new_flexio_data = pfb;
  return true;
}

// Quick and Dirty
#define UPDATE_ON_CAMERA_FRAMES

inline uint16_t HTONS(uint16_t x) {
#if defined(ARDUCAM_CAMERA_OV2640)
  return x;
#else
  return ((x >> 8) & 0x00FF) | ((x << 8) & 0xFF00);
#endif
}

#if defined(ARDUCAM_CAMERA_OV7675) || defined(ARDUCAM_CAMERA_OV7670) ||        \
    defined(ARDUCAM_CAMERA_OV2640) || defined(ARDUCAM_CAMERA_GC2145)

volatile uint16_t *pfb_last_frame_returned = nullptr;

bool camera_flexio_callback_video(void *pfb) {
  pfb_last_frame_returned = (uint16_t *)pfb;
#ifdef UPDATE_ON_CAMERA_FRAMES
  int numPixels = FRAME_WIDTH * FRAME_HEIGHT;
  if (g_flex_dual_buffer_per_frame) {
    int numPixels1 = min((int)(sizeof_framebuffer / 2), numPixels);

    if (g_new_flexio_data == frameBuffer) {
      for (int i = 0; i < numPixels1; i++)
        frameBuffer[i] = HTONS(frameBuffer[i]);
      int start_x = (tft.width() - camera.width()) / 2;
      int start_y = (tft.height() - camera.height()) / 2;
      tft.writeRect(start_x, start_y, camera.width(),
                    numPixels1 / camera.width(), frameBuffer);

    } else {
      int numPixels2 =
          min((int)(sizeof_framebuffer2 / 2), numPixels - numPixels1);
      for (int i = 0; i < numPixels2; i++)
        frameBuffer2[i] = HTONS(frameBuffer2[i]);
      int start_x = (tft.width() - camera.width()) / 2;
      int start_y = ((tft.height() - camera.height()) / 2) +
                    (numPixels1 / camera.width());
      tft.writeRect(start_x, start_y, camera.width(),
                    numPixels2 / camera.width(), frameBuffer2);
    }

  } else {
    for (int i = 0; i < numPixels; i++)
      frameBuffer[i] = HTONS(frameBuffer[i]);
    tft.writeRect(CENTER, CENTER, camera.width(), camera.height(),
                  (uint16_t *)g_new_flexio_data);
  }
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
    arm_dcache_delete(pfb_last_frame_returned, FRAME_WIDTH * FRAME_HEIGHT * 2);
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
#ifdef UPDATE_ON_CAMERA_FRAMES
  if (g_flex_dual_buffer_per_frame && (pfb == g_last_flexio_data))
    return true; // same frame as previous punt
  g_last_flexio_data = (uint8_t *)pfb;

  int numPixels = camera.width() * camera.height();
  if (g_flex_dual_buffer_per_frame) {
    int numPixels1 = min((int)(sizeof_framebuffer), numPixels);

    if (pfb == frameBuffer) {
      int start_x = (tft.width() - camera.width()) / 2;
      int start_y = (tft.height() - camera.height()) / 2;
      tft.writeRect8BPP(start_x, start_y, camera.width(),
                        numPixels1 / camera.width(), frameBuffer, mono_palette);

    } else {
      int numPixels2 = min((int)sizeof_framebuffer2, numPixels - numPixels1);
      int start_x = (tft.width() - camera.width()) / 2;
      int start_y = ((tft.height() - camera.height()) / 2) +
                    (numPixels1 / camera.width());
      tft.writeRect8BPP(start_x, start_y, camera.width(),
                        numPixels2 / camera.width(), frameBuffer2,
                        mono_palette);
    }

  } else {
    int start_x = (tft.width() - camera.width()) / 2;
    int start_y = (tft.height() - camera.height()) / 2;
    tft.writeRect8BPP(start_x, start_y, camera.width(), camera.height(),
                      (uint8_t *)pfb, mono_palette);
  }
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
                    (uint8_t *)pfb_last_frame_returned, mono_palette);
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
    Serial.printf("USB1 CH: %x\n", ch);
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
      Serial.println("*** Only works in USB Dual or Triple Serial Mode ***");
#endif
      break;
    }
    case 'z': {
#if defined(USE_SDCARD)
      save_image_SD();
#endif
      break;
    }
    case 'b': {
#if defined(USE_SDCARD)
      memset((uint8_t *)frameBuffer, 0, sizeof_framebuffer);
      camera.setMode(HIMAX_MODE_STREAMING_NFRAMES, 1);
      // camera.readFrame(frameBuffer);
      camera.readFrame(frameBuffer, sizeof_framebuffer, frameBuffer2,
                       sizeof_framebuffer2);

#ifndef CAMERA_USES_MONO_PALETTE
      // #if defined(ARDUCAM_CAMERA_OV7675) || defined(ARDUCAM_CAMERA_OV7670) ||
      // defined(ARDUCAM_CAMERA_GC2145)
      int numPixels = camera.width() * camera.height();
      // for (int i = 0; i < numPixels; i++) frameBuffer[i] =
      // HTONS(frameBuffer[i]);
      int numPixels1 = min((int)(sizeof_framebuffer / 2), numPixels);
      int numPixels2 =
          min((int)(sizeof_framebuffer2 / 2), numPixels - numPixels1);
      for (int i = 0; i < numPixels1; i++)
        frameBuffer[i] = HTONS(frameBuffer[i]);
      for (int i = 0; i < numPixels2; i++)
        frameBuffer2[i] = HTONS(frameBuffer2[i]);

#endif
      save_image_SD();
      ch = ' ';
#endif
      break;
    }
    case 'j': {
#if (defined(USE_SDCARD) && defined(ARDUCAM_CAMERA_OV2640))
      bool error = false;
      error = save_jpg_SD();
      if (!error)
        Serial.println("ERROR reading JPEG.  Try again....");
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
    case 'r':
      camera.showRegisters();
      break;
    case 'w':
      changeCameraWindow();
      break;
    case 'W':
      panCameraWindow();
      break;
    case 'c':
      print_vsync_timings();
      break;
    case 'd':
      camera.debug(!camera.debug());
      if (camera.debug())
        Serial.println("Camera Debug turned on");
      else
        Serial.println("Camera debug turned off");
      break;
#ifdef ARDUINO_TEENSY_DEVBRD4
    case 's': {
      if (frameBuffer == frameBufferSDRAM) {
        Serial.print("Switching to NON SDRam buffers: ");
        frameBuffer = frameBufferM;
        frameBuffer2 = frameBufferM2;
        sizeof_framebuffer = sizeof(frameBufferM);
        sizeof_framebuffer2 = sizeof(frameBufferM2);
      } else {
        Serial.print("Switching to SDRam buffers: ");
        frameBuffer = frameBufferSDRAM;
        frameBuffer2 = frameBufferSDRAM2;
        sizeof_framebuffer = sizeof_framebufferSDRAM;
        sizeof_framebuffer2 = sizeof_framebufferSDRAM;
      }
      Serial.printf("%p(%u) %p(%u)\n", frameBuffer, sizeof_framebuffer,
                    frameBuffer2, sizeof_framebuffer2);
    } break;
#endif
    case 'f':
      tft.useFrameBuffer(false);
      tft.fillScreen(TFT_BLACK);
      read_display_one_frame(true, true);
      ch = ' ';
      g_continuous_flex_mode = false;
      break;
    case 'n':
      tft.useFrameBuffer(false);
      tft.fillScreen(TFT_BLACK);
      read_display_one_frame(false, true);
      ch = ' ';
      g_continuous_flex_mode = false;
      break;
    case 'F': {
      if (!g_continuous_flex_mode) {
        if (camera.readContinuous(&hm0360_flexio_callback, frameBuffer,
                                  sizeof_framebuffer, frameBuffer2,
                                  sizeof_framebuffer2)) {
          Serial.println("* continuous mode started");
          g_flex_dual_buffer_per_frame =
              sizeof_framebuffer <
              (FRAME_WIDTH * FRAME_HEIGHT * sizeof(frameBuffer[0]));
          g_flexio_capture_count = 0;
          g_flexio_redraw_count = 0;
          g_continuous_flex_mode = true;
          g_last_flexio_data = nullptr;
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
        if (!tft.useFrameBuffer(true)) {
          Serial.println("Failed call to useFrameBuffer");
        } else if (camera.readContinuous(&camera_flexio_callback_video,
                                         frameBuffer, sizeof_framebuffer,
                                         frameBuffer2, sizeof_framebuffer2)) {

          Serial.println("Before Set frame complete CB");
          tft.setFrameCompleteCB(&frame_complete_cb, false);
          Serial.println("Before UPdateScreen Async");
          if (!tft.updateScreenAsync(true)) {
            Serial.println("* error, could not start Update Screen Async\n");
          }
          Serial.println("* continuous mode (Video) started");
          g_flexio_capture_count = 0;
          g_flexio_redraw_count = 0;
          g_continuous_flex_mode = 2;
          g_flex_dual_buffer_per_frame =
              sizeof_framebuffer <
              (FRAME_WIDTH * FRAME_HEIGHT * sizeof(frameBuffer[0]));
        } else {
          Serial.println("* error, could not start continuous mode");
        }
      } else {
        camera.stopReadContinuous();
        tft.endUpdateAsync();
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
      Serial.println("*** Only works in USB Dual or Triple Serial Mode ***");
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
    // try to make sure we alternate.
    if (g_new_flexio_data && (!g_flex_dual_buffer_per_frame ||
                              (g_new_flexio_data != g_last_flexio_data))) {
      // Serial.println("new FlexIO data");
      int numPixels = camera.width() * camera.height();
      g_last_flexio_data = g_new_flexio_data;

#ifndef CAMERA_USES_MONO_PALETTE
      if (g_flex_dual_buffer_per_frame) {
        int numPixels1 = min((int)(sizeof_framebuffer / 2), numPixels);

        if (g_new_flexio_data == frameBuffer) {
          for (int i = 0; i < numPixels1; i++)
            frameBuffer[i] = HTONS(frameBuffer[i]);
          int start_x = (tft.width() - camera.width()) / 2;
          int start_y = (tft.height() - camera.height()) / 2;
          tft.writeRect(start_x, start_y, camera.width(),
                        numPixels1 / camera.width(), frameBuffer);
          // Serial.println("1");

        } else {
          int numPixels2 =
              min((int)(sizeof_framebuffer2 / 2), numPixels - numPixels1);
          for (int i = 0; i < numPixels2; i++)
            frameBuffer2[i] = HTONS(frameBuffer2[i]);
          int start_x = (tft.width() - camera.width()) / 2;
          int start_y = ((tft.height() - camera.height()) / 2) +
                        (numPixels1 / camera.width());
          tft.writeRect(start_x, start_y, camera.width(),
                        numPixels2 / camera.width(), frameBuffer2);
          // Serial.println("2");
        }

      } else {
        for (int i = 0; i < numPixels; i++)
          frameBuffer[i] = HTONS(frameBuffer[i]);
        tft.writeRect(CENTER, CENTER, camera.width(), camera.height(),
                      (uint16_t *)g_new_flexio_data);
        // Serial.printf("T: %p\n", g_new_flexio_data);
      }
#else // Monochrome
      if (g_flex_dual_buffer_per_frame) {
        int numPixels1 = min((int)(sizeof_framebuffer), numPixels);

        if (g_new_flexio_data == frameBuffer) {
          int start_x = (tft.width() - camera.width()) / 2;
          int start_y = (tft.height() - camera.height()) / 2;
          tft.writeRect8BPP(start_x, start_y, camera.width(),
                            numPixels1 / camera.width(), frameBuffer,
                            mono_palette);
          Serial.println("1");

        } else {
          int numPixels2 =
              min((int)sizeof_framebuffer2, numPixels - numPixels1);
          int start_x = (tft.width() - camera.width()) / 2;
          int start_y = ((tft.height() - camera.height()) / 2) +
                        (numPixels1 / camera.width());
          tft.writeRect8BPP(start_x, start_y, camera.width(),
                            numPixels2 / camera.width(), frameBuffer2,
                            mono_palette);
          Serial.println("2");
        }

      } else {
        tft.writeRect8BPP(0, 0, camera.width(), camera.height(),
                          (uint8_t *)g_new_flexio_data, mono_palette);
        Serial.printf("T: %p\n", g_new_flexio_data);
      }
#endif
      tft.updateScreenAsync();
      g_new_flexio_data = nullptr;
      g_flexio_redraw_count++;
      if (g_flexio_runtime > 10000) {
        // print some stats on actual speed, but not too much
        // printing too quickly to be considered "spew"
        float redraw_rate =
            (float)g_flexio_redraw_count / (float)g_flexio_runtime * 1000.0f;
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
  memset((uint8_t *)frameBuffer, 0, sizeof_framebuffer);
  //  camera.setHmirror(1);
  camera.readFrame(frameBuffer, sizeof_framebuffer, frameBuffer2,
                   sizeof_framebuffer2);

  imgSerial->write(0xFF);
  imgSerial->write(0xAA);

  // BUGBUG:: maybe combine with the save to SD card code
  unsigned char bmpFileHeader[14] = {'B', 'M', 0, 0,  0, 0, 0,
                                     0,   0,   0, 54, 0, 0, 0};
  unsigned char bmpInfoHeader[40] = {40, 0, 0, 0, 0, 0, 0,  0,
                                     0,  0, 0, 0, 1, 0, 24, 0};

  int rowSize = 4 * ((3 * FRAME_WIDTH + 3) /
                     4); // how many bytes in the row (used to create padding)
  int fileSize = 54 + FRAME_HEIGHT * rowSize; // headers (54 bytes) + pixel data

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

#ifdef CAMERA_USES_MONO_PALETTE
  uint8_t *pfb = frameBuffer;
  uint32_t count_y_first_buffer = sizeof_framebuffer / FRAME_WIDTH;
  uint8_t img[3];
  for (int y = FRAME_HEIGHT - 1; y >= 0; y--) { // iterate image array
    if (y < (int)count_y_first_buffer)
      pfb = &frameBuffer[y * FRAME_WIDTH];
    else
      pfb = &frameBuffer2[(y - count_y_first_buffer) * FRAME_WIDTH];
    for (int x = 0; x < FRAME_WIDTH; x++) {
      // r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3
      img[2] = *pfb; // r
      img[1] = *pfb; // g
      img[0] = *pfb; // b
      imgSerial->write(img, 3);
      delayMicroseconds(8);
      pfb++;
    }
#else
  uint16_t *pfb = frameBuffer;
  uint8_t img[3];
  uint32_t count_y_first_buffer = sizeof_framebuffer / (FRAME_WIDTH * 2);
  for (int y = FRAME_HEIGHT - 1; y >= 0; y--) { // iterate image array
    if (y < (int)count_y_first_buffer)
      pfb = &frameBuffer[y * FRAME_WIDTH];
    else
      pfb = &frameBuffer2[(y - count_y_first_buffer) * FRAME_WIDTH];
    for (int x = 0; x < FRAME_WIDTH; x++) {
      // r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3
      uint16_t pixel = HTONS(*pfb++);
      img[2] = (pixel >> 8) & 0xf8; // r
      img[1] = (pixel >> 3) & 0xfc; // g
      img[0] = (pixel << 3);        // b
      imgSerial->write(img, 3);
      delayMicroseconds(8);
    }
#endif
    file.write(bmpPad,
               (4 - (FRAME_WIDTH * 3) % 4) % 4); // and padding as needed
  }
  imgSerial->write(0xBB);
  imgSerial->write(0xCC);

  imgSerial->println(F("ACK CMD CAM Capture Done. END"));
  // camera.setHmirror(0);

  delay(50);
}

// #if defined(USB_DUAL_SERIAL) || defined(USB_TRIPLE_SERIAL)
#ifdef CAMERA_USES_MONO_PALETTE
void send_raw() {
  Serial.println("Send Raw MONO");
  memset((uint8_t *)frameBuffer, 0, sizeof_framebuffer);
  memset((uint8_t *)frameBuffer2, 0, sizeof_framebuffer2);
  camera.readFrame(frameBuffer, sizeof_framebuffer, frameBuffer2,
                   sizeof_framebuffer2);
  uint8_t *p = frameBuffer;
  uint32_t cnt = sizeof_framebuffer;
  for (int i = 0; i < FRAME_HEIGHT * FRAME_WIDTH; i++) {
    uint16_t rgb = mono_palette[*p];
    SerialUSB1.write(rgb & 0xFF);
    SerialUSB1.write((rgb >> 8) & 0xFF);
    cnt--;
    if (cnt == 0) {
      cnt = sizeof_framebuffer2;
      p = frameBuffer2;
    } else {
      p++;
    }
  }
}
#else
void send_raw() {
  Serial.println("Send Raw RGB");
  memset((uint8_t *)frameBuffer, 0, sizeof_framebuffer);
  memset((uint8_t *)frameBuffer2, 0, sizeof_framebuffer2);
  camera.readFrame(frameBuffer, sizeof_framebuffer, frameBuffer2,
                   sizeof_framebuffer2);
  uint16_t *p = frameBuffer;
  uint32_t cnt = sizeof_framebuffer / 2;
  for (int i = 0; i < FRAME_HEIGHT * FRAME_WIDTH; i++) {
    SerialUSB1.write((*p >> 8) & 0xFF);
    SerialUSB1.write((*p) & 0xFF);
    cnt--;
    if (cnt == 0) {
      cnt = sizeof_framebuffer2 / 2;
      p = frameBuffer2;
    } else {
      p++;
    }
  }
}
#endif
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
// Handle possible split buffer.
#ifdef CAMERA_USES_MONO_PALETTE
  uint8_t *pfb = frameBuffer;
  uint32_t count_y_first_buffer = sizeof_framebuffer / w;
  uint8_t img[3];
  for (int y = h - 1; y >= 0; y--) { // iterate image array
    if ((int)y < count_y_first_buffer)
      pfb = &frameBuffer[y * w];
    else
      pfb = &frameBuffer2[(y - count_y_first_buffer) * w];
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
  uint32_t count_y_first_buffer = sizeof_framebuffer / (w * 2);
  for (int y = h - 1; y >= 0; y--) { // iterate image array
    if (y < (int)count_y_first_buffer)
      pfb = &frameBuffer[y * w];
    else
      pfb = &frameBuffer2[(y - count_y_first_buffer) * w];
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
  camera.setPixformat(JPEG);
  delay(100);
  // omni.setQuality(12);

  camera.setMode(HIMAX_MODE_STREAMING_NFRAMES, 1);
  uint32_t count_pixels_in_buffer = sizeof_framebuffer / sizeof(frameBuffer[0]);

  if (0) {
    Serial.println("Reading frame");
    Serial.printf("Buffer1: %p(%u) halfway: %p end:%p\n", frameBuffer,
                  sizeof_framebuffer, &frameBuffer[count_pixels_in_buffer / 2],
                  &frameBuffer[count_pixels_in_buffer]);
    count_pixels_in_buffer = sizeof_framebuffer2 / sizeof(frameBuffer2[0]);
    Serial.printf("Buffer2: %p(%u) halfway: %p end:%p\n", frameBuffer2,
                  sizeof_framebuffer2,
                  &frameBuffer2[count_pixels_in_buffer / 2],
                  &frameBuffer2[count_pixels_in_buffer]);
    memset((uint8_t *)frameBuffer, 0, sizeof_framebuffer);
    memset((uint8_t *)frameBuffer2, 0, sizeof_framebuffer2);
  }

  //  digitalWriteFast(24, HIGH);
  camera.useDMA(false);
  camera.readFrame(frameBuffer, sizeof_framebuffer, frameBuffer2,
                   sizeof_framebuffer2);
  delay(10);
  camera.readFrame(frameBuffer, sizeof_framebuffer, frameBuffer2,
                   sizeof_framebuffer2);

  //  digitalWriteFast(24, LOW);

  uint16_t w = FRAME_WIDTH;
  uint16_t h = FRAME_HEIGHT;
  uint16_t eop = 0;
  uint8_t eoi = 0;

  uint32_t numPixels = w * h;
  uint32_t jpegSize = (w * h) / 5;
  Serial.printf("Width: %d, Height: %d\n", w, h);
  Serial.printf("jpeg size: %d\n", jpegSize);

  // byte swap
  // for (int i = 0; i < numPixels; i++) frameBuffer[i] = (frameBuffer[i] >> 8)
  // | (((frameBuffer[i] & 0xff) << 8));
  // now see if all fit into one buffer or part in second...
  uint32_t numPixels1 = min((int)(sizeof_framebuffer / 2), numPixels);
  uint32_t numPixels2 =
      min((int)(sizeof_framebuffer2 / 2), numPixels - numPixels1);
  if (0)
    Serial.printf("\tBuffers:%p(%u) %p(%u)\n", frameBuffer, sizeof_framebuffer,
                  frameBuffer2, sizeof_framebuffer2);
  if (1)
    Serial.printf("\tnumpixels %u %u %u\n", numPixels, numPixels1, numPixels2);
  // for (int i = 0; i < numPixels1; i++) frameBuffer[i] =
  // HTONS(frameBuffer[i]); for (int i = 0; i < numPixels2; i++) frameBuffer2[i]
  // = HTONS(frameBuffer2[i]);

  if (numPixels2 == 0) {
    uint16_t *pfb = frameBuffer;
    for (uint32_t i = 0; i < numPixels1; i++) {
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
    Serial.printf("%x, %x\n", pfb[0], pfb[eop]);

  } else {
    for (uint32_t i = 0; i < numPixels1; i++) {
      // Serial.println(pfb[i], HEX);
      if ((i == 0) && (frameBuffer[0] == 0xD8FF)) {
        eoi = 1;
        Serial.printf("Found begining of frame at position %d\n", i);
        break;
      }
    }
    for (uint32_t i = 0; i < numPixels2; i++) {
      if (frameBuffer2[i] == 0xD9FF) {
        eop = i;
        Serial.printf("(2)Found ending of frame at position %d\n", i);
      }
    }
    Serial.printf("%x, %x\n", frameBuffer[0], frameBuffer2[eop]);
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
  if (numPixels2 == 0) {
    for (uint32_t i = 0; i < jpegSize; i++) {
      file.write(frameBuffer[i] & 0xFF);
      file.write((frameBuffer[i] >> 8) & 0xFF);
    }
  } else {
    for (uint32_t i = 0; i < numPixels1; i++) {
      // file.write(pfb[i]);
      file.write(frameBuffer[i] & 0xFF);
      file.write((frameBuffer[i] >> 8) & 0xFF);
    }
    for (uint32_t i = 0; i < eop; i++) {
      file.write(frameBuffer2[i] & 0xFF);
      file.write((frameBuffer2[i] >> 8) & 0xFF);
    }
  }

  file.close(); // close file when done writing
  Serial.println("Done Writing JPG");

  camera.setPixformat(camera_format);
  camera.useDMA(true);
  delay(50);
  return true;
}

#endif

void showCommandList() {
  if (camera.usingGPIO()) {
    Serial.println("Send the 'f' character to read a frame using GPIO");
    Serial.println("Send the 'F' to start/stop continuous using GPIO");
    Serial.println(
        "Send the 'n' character to read a frame using GPIO without DMA?");

  } else {
    Serial.println("Send the 'f' character to read a frame using FlexIO "
                   "(changes hardware setup!)");
    Serial.println("Send the 'F' to start/stop continuous using FlexIO "
                   "(changes hardware setup!)");
    Serial.println("Send the 'n' character to read a frame using FlexIO "
                   "without DMA (changes hardware setup!)");
  }
  Serial.println("Send the 'm' character to read and display multiple frames");
  Serial.println("Send the 'M' character to read and display multiple frames "
                 "use Frame buffer");
  Serial.println("Send the 'V' character DMA to TFT async continueous  ...");
  Serial.println("Send the 'p' character to snapshot to PC on USB1");
  Serial.println("Send the 'b' character to save snapshot (BMP) to SD Card");
  Serial.println("Send the '1' character to blank the display");
  Serial.println("Send the 'z' character to send current screen BMP to SD");
  Serial.println("Send the 't' character to send Check the display");
  Serial.println("Send the 'c' character to debug clock - print vsync timing");
  Serial.println("Send the 'd' character to toggle camera debug on and off");
  Serial.println("Send the 'r' character to show the current camera registers");
  Serial.println("Send the 'w <row> <col>' to set the start window x, y");
  Serial.println("Send the 'W' to pan through range of windows");
#ifdef ARDUINO_TEENSY_DEVBRD4
  Serial.println("Send the 's' to change if using SDRAM or other memory");
#endif
  Serial.println();
}

//=============================================================================
void read_display_one_frame(bool use_dma, bool show_debug_info) {
  camera.setMode(HIMAX_MODE_STREAMING_NFRAMES, 1);
  uint32_t count_pixels_in_buffer = sizeof_framebuffer / sizeof(frameBuffer[0]);
  if (show_debug_info) {
    Serial.println("Reading frame");
    Serial.printf("Buffer1: %p(%u) halfway: %p end:%p\n", frameBuffer,
                  sizeof_framebuffer, &frameBuffer[count_pixels_in_buffer / 2],
                  &frameBuffer[count_pixels_in_buffer]);
    count_pixels_in_buffer = sizeof_framebuffer2 / sizeof(frameBuffer2[0]);
    Serial.printf("Buffer2: %p(%u) halfway: %p end:%p\n", frameBuffer2,
                  sizeof_framebuffer2,
                  &frameBuffer2[count_pixels_in_buffer / 2],
                  &frameBuffer2[count_pixels_in_buffer]);
    memset((uint8_t *)frameBuffer, 0, sizeof_framebuffer);
    memset((uint8_t *)frameBuffer2, 0, sizeof_framebuffer2);
  }
  //  digitalWriteFast(24, HIGH);
  camera.useDMA(use_dma);
  camera.readFrame(frameBuffer, sizeof_framebuffer, frameBuffer2,
                   sizeof_framebuffer2);
  //  digitalWriteFast(24, LOW);

  if (show_debug_info) {
    Serial.println("Finished reading frame");
    Serial.flush();
#if defined(ARDUCAM_CAMERA_OV7675) || defined(ARDUCAM_CAMERA_OV7670) ||        \
    defined(ARDUCAM_CAMERA_OV2640) || defined(ARDUCAM_CAMERA_GC2145)
    for (volatile uint16_t *pfb = frameBuffer;
         pfb < (frameBuffer + 4 * camera.width()); pfb += camera.width()) {
#else
    for (volatile uint8_t *pfb = frameBuffer;
         pfb < (frameBuffer + 4 * camera.width()); pfb += camera.width()) {
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
#if 0 // Figure this out later...
      // Lets dump out some of center of image.
            Serial.println("Show Center pixels\n");
#if defined(ARDUCAM_CAMERA_OV7675) || defined(ARDUCAM_CAMERA_OV7670)
            for (volatile uint16_t *pfb = frameBuffer + camera.width() * ((camera.height() / 2) - 8); pfb < (frameBuffer + camera.width() * (camera.height() / 2 + 8)); pfb += camera.width()) {
#else
            for (volatile uint8_t *pfb = frameBuffer + camera.width() * ((camera.height() / 2) - 8); pfb < (frameBuffer + camera.width() * (camera.height() / 2 + 8)); pfb += camera.width()) {
#endif
              Serial.printf("\n%08x: ", (uint32_t)pfb);
              for (uint16_t i = 0; i < 8; i++) Serial.printf("%02x ", pfb[i]);
              Serial.print("..");
              for (uint16_t i = (camera.width() / 2) - 4; i < (camera.width() / 2) + 4; i++) Serial.printf("%02x ", pfb[i]);
              Serial.print("..");
              for (uint16_t i = camera.width() - 8; i < camera.width(); i++) Serial.printf("%02x ", pfb[i]);
            }
            Serial.println("\n...");

            //int numPixels = camera.width() * camera.height();
            Serial.printf("TFT(%u, %u) Camera(%u, %u)\n", tft.width(), tft.height(), camera.width(), camera.height());
            //int camera_width = Camera.width();
#endif
    Serial.printf("TFT(%u, %u) Camera(%u, %u)\n", tft.width(), tft.height(),
                  camera.width(), camera.height());
  }
  // tft.setOrigin(-2, -2);
  int numPixels = camera.width() * camera.height();
#ifndef CAMERA_USES_MONO_PALETTE
// #if defined(ARDUCAM_CAMERA_OV7675) || defined(ARDUCAM_CAMERA_OV7670)

// int camera_width = Camera.width();
#if 0 // def ARDUINO_TEENSY_DEVBRD4
  for (int i = 0; i < numPixels; i++) frameBuffer[i] = HTONS(frameBuffer[i]);
  tft.writeRect(CENTER, CENTER, camera.width(), camera.height(), frameBuffer);
  return;
#endif

#if 1
  // byte swap
  // for (int i = 0; i < numPixels; i++) frameBuffer[i] = (frameBuffer[i] >> 8)
  // | (((frameBuffer[i] & 0xff) << 8));
  // now see if all fit into one buffer or part in second...
  int numPixels1 = min((int)(sizeof_framebuffer / 2), numPixels);
  int numPixels2 = min((int)(sizeof_framebuffer2 / 2), numPixels - numPixels1);
  if (show_debug_info)
    Serial.printf("\tBuffers:%p(%u) %p(%u)\n", frameBuffer, sizeof_framebuffer,
                  frameBuffer2, sizeof_framebuffer2);
  if (show_debug_info)
    Serial.printf("\tnumpixels %u %u %u\n", numPixels, numPixels1, numPixels2);
  for (int i = 0; i < numPixels1; i++)
    frameBuffer[i] = HTONS(frameBuffer[i]);
  for (int i = 0; i < numPixels2; i++)
    frameBuffer2[i] = HTONS(frameBuffer2[i]);

  if (numPixels2 == 0) {
    if ((camera.width() <= tft.width()) && (camera.height() <= tft.height())) {
      // if ((camera.width() != tft.width()) || (camera.height() !=
      // tft.height())) tft.fillScreen(TFT_BLACK);
      tft.writeRect(CENTER, CENTER, camera.width(), camera.height(),
                    frameBuffer);
    } else {
      Serial.println("sub image");
      tft.writeSubImageRect(0, 0, tft.width(), tft.height(),
                            (camera.width() - tft.width()) / 2,
                            (camera.height() - tft.height()), camera.width(),
                            camera.height(), frameBuffer);
    }
  } else {
    // We used both buffers.  Assume for now each buffer is in multples of Width
    //    tft.writeRect(CENTER, CENTER, camera.width(), camera.height(),
    //    frameBuffer);
    int start_x = (tft.width() - camera.width()) / 2;
    int start_y = (tft.height() - camera.height()) / 2;
    tft.writeRect(start_x, start_y, camera.width(), numPixels1 / camera.width(),
                  frameBuffer);

    // now try second part
    start_y += numPixels1 / camera.width();
    tft.writeRect(start_x, start_y, camera.width(), numPixels2 / camera.width(),
                  frameBuffer2);
  }

#else
  Serial.println("sub image1");
  tft.writeSubImageRect(0, 0, tft.width(), tft.height(), 0, 0, camera.width(),
                        camera.height(), pixels);
#endif
#else
  // tft.writeRect8BPP(0, 0, FRAME_WIDTH, FRAME_HEIGHT, frameBuffer,
  // mono_palette);
  int numPixels1 = min((int)(sizeof_framebuffer), numPixels);
  int start_x = (tft.width() - camera.width()) / 2;
  int start_y = (tft.height() - camera.height()) / 2;
  int num_rows = numPixels1 / camera.width();
  Serial.printf("start(%d, %d, %d, %d) cnt:%d\n", start_x, start_y,
                camera.width(), num_rows, numPixels1);
  tft.writeRect8BPP(start_x, start_y, camera.width(), num_rows, frameBuffer,
                    mono_palette);

  if (numPixels1 < numPixels) {
    // now try second part
    start_y += num_rows;
    int numPixels2 = min((int)(sizeof_framebuffer2), numPixels - numPixels1);
    num_rows = numPixels2 / camera.width();

    Serial.printf("start(%d, %d, %d, %d): cnt:%d %02x %02x %02x %02x\n",
                  start_x, start_y, camera.width(), num_rows, numPixels2,
                  frameBuffer2[0], frameBuffer2[1], frameBuffer2[2],
                  frameBuffer2[3]);
    tft.writeRect8BPP(start_x, start_y, camera.width(), num_rows, frameBuffer2,
                      mono_palette);
  }
#endif
}

//=============================================================================
void read_display_multiple_frames(bool use_frame_buffer) {
  if (use_frame_buffer) {
    Serial.println("\n*** Read and display multiple frames (using async screen "
                   "updates), press any key to stop ***");
    tft.useFrameBuffer(true);
  } else {
    Serial.println(
        "\n*** Read and display multiple frames, press any key to stop ***");
  }

  while (Serial.read() != -1) {
  }

  elapsedMicros em = 0;
  int frame_count = 0;

  for (;;) {

#if 1
    if (use_frame_buffer)
      tft.waitUpdateAsyncComplete();
    read_display_one_frame(true, false);
#else
    camera.readFrame(frameBuffer, sizeof_framebuffer);

    int numPixels = camera.width() * camera.height();

// byte swap
// for (int i = 0; i < numPixels; i++) frameBuffer[i] = (frameBuffer[i] >> 8) |
// (((frameBuffer[i] & 0xff) << 8));
#ifdef CAMERA_USES_MONO_PALETTE
    tft.writeRect8BPP(0, 0, FRAME_WIDTH, FRAME_HEIGHT, frameBuffer,
                      mono_palette);
#else
    for (int i = 0; i < numPixels; i++)
      frameBuffer[i] = HTONS(frameBuffer[i]);

    if (use_frame_buffer)
      tft.waitUpdateAsyncComplete();

    if ((camera.width() <= tft.width()) && (camera.height() <= tft.height())) {
      // if ((camera.width() != tft.width()) || (camera.height() !=
      // tft.height())) tft.fillScreen(TFT_BLACK);
      tft.writeRect(CENTER, CENTER, camera.width(), camera.height(),
                    frameBuffer);
    } else {
      tft.writeSubImageRect(0, 0, tft.width(), tft.height(),
                            (camera.width() - tft.width()) / 2,
                            (camera.height() - tft.height()), camera.width(),
                            camera.height(), frameBuffer);
    }
#endif
#endif // 1
    if (use_frame_buffer)
      tft.updateScreenAsync();

    frame_count++;
    if ((frame_count & 0x7) == 0) {
      Serial.printf("Elapsed: %u frames: %d fps: %.2f\n", (uint32_t)em,
                    frame_count, (float)(1000000.0 / em) * (float)frame_count);
    }
    if (Serial.available())
      break;
  }
  // turn off frame buffer
  tft.useFrameBuffer(false);
}

uint16_t get_next_value(uint8_t &b) {
  uint16_t val = 0;

  for (;;) {
    if (b == (uint8_t)-1)
      return 0;
    if (b != ' ')
      break;
    b = Serial.read();
  }
  while ((b >= '0') && (b <= '9')) {
    val = val * 10 + b - '0';
    b = Serial.read();
  }
  return val;
}

void changeCameraWindow() {

  uint8_t b = Serial.read();
  if (b < ' ') {
    Serial.printf("Set Camera origin to center\n");
    camera.setZoomWindow(-1, -1);
  } else {
    uint16_t start_x = get_next_value(b);
    uint16_t start_y = get_next_value(b);
    Serial.printf("Set Camera origin row: %u col:%u\n", start_y, start_x);
    camera.setZoomWindow(start_x, start_y);
  }
}

void panCameraWindow() {
  while (Serial.read() != -1) {
  }
  for (;;) {
    for (uint16_t start_y = 0;
         start_y < (camera.frameHeight() - camera.height()); start_y += 100) {
      for (uint16_t start_x = 0;
           start_x < (camera.frameWidth() - camera.width()); start_x += 100) {
        camera.setZoomWindow(start_x, start_y);
        read_display_one_frame(true, false);
        delay(250);
        if (Serial.available())
          return;
      }
    }
  }
}

void print_vsync_timings() {
  uint8_t frame_count = 0;
  while (Serial.read() != -1) {
  } // remove any extra characters

  uint32_t falling_edge_time = micros();
  uint32_t rising_edge_time = micros();
  while (!Serial.available() && (frame_count < 5)) {
    elapsedMillis emTimeout = 0;
    while (!digitalReadFast(VSYNC_PIN)) {
      if (emTimeout > 5000) {
        Serial.printf("Vsync Timing timeout rising edge FC:%u\n", frame_count);
        return;
      }
    }
    rising_edge_time = micros();
    emTimeout = 0;
    while (digitalReadFast(VSYNC_PIN)) {
      if (emTimeout > 5000) {
        Serial.printf("Vsync Timing timeout falling edge FC:%u\n", frame_count);
        return;
      }
    }
    uint32_t new_falling_edge_time = micros();
    frame_count++;
    Serial.printf("Frame %u Time: %u High:%u Low:%u fps:%0.2f\n", frame_count,
                  (uint32_t)(new_falling_edge_time - falling_edge_time),
                  (uint32_t)(new_falling_edge_time - rising_edge_time),
                  (uint32_t)(rising_edge_time - falling_edge_time),
                  1000000.0 / (float)(uint32_t)(new_falling_edge_time -
                                                falling_edge_time));
    falling_edge_time = new_falling_edge_time;
  }
}
