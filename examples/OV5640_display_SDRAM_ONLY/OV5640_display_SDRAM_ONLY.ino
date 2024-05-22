#include <stdint.h>

#include <SPI.h>
#include <JPEGDEC.h>
#include <MemoryHexDump.h>

#include "Teensy_Camera.h"

#define USE_MMOD_ATP_ADAPTER
//#define useILI9341

#define DVP_CAMERA_OV5640

#include "Teensy_OV5640/OV5640.h"
OV5640 omni;
Camera camera(omni);
#define CameraID OV5640a
#define MIRROR_FLIP_CAMERA


//set cam configuration - need to remember when saving jpeg
framesize_t camera_framesize = FRAMESIZE_VGA;
pixformat_t camera_format = RGB565;
bool useGPIO = false;
#define use_sdram

#define skipFrames 1



/*****************************************************
 * If using the HM01B0 Arduino breakout 8bit mode    *
 * does not work.  Arduino breakout only brings out  *
 * the lower 4 bits.                                 *
 ****************************************************/
#define _hmConfig 0  // select mode string below

PROGMEM const char hmConfig[][48] = {
  "FLEXIO_CUSTOM_LIKE_8_BIT",
  "FLEXIO_CUSTOM_LIKE_4_BIT"
};


//Set up Display
#ifdef ARDUINO_TEENSY_DEVBRD4
#undef USE_MMOD_ATP_ADAPTER
#define TFT_CS 10  // AD_B0_02
#define TFT_DC 25  // AD_B0_03
#define TFT_RST 24
#define VSYNC_PIN 21

#elif defined(USE_MMOD_ATP_ADAPTER)
#define VSYNC_PIN 33
#define TFT_DC 4   //0   // "TX1" on left side of Sparkfun ML Carrier
#define TFT_CS 5   //4   // "CS" on left side of Sparkfun ML Carrier
#define TFT_RST 2  //1  // "RX1" on left side of Sparkfun ML Carrier
#else
#define VSYNC_PIN 33
#define TFT_DC 0   //20   // "TX1" on left side of Sparkfun ML Carrier
#define TFT_CS 4   //5, 4   // "CS" on left side of Sparkfun ML Carrier
#define TFT_RST 1  //2, 1  // "RX1" on left side of Sparkfun ML Carrier
#endif

#if defined(useILI9341)
#include "ILI9341_t3n.h"  // https://github.com/KurtE/ILI9341_t3n
ILI9341_t3n tft = ILI9341_t3n(TFT_CS, TFT_DC, TFT_RST);
#define TFT_BLACK ILI9341_BLACK
#define TFT_YELLOW ILI9341_YELLOW
#define TFT_RED ILI9341_RED
#define TFT_GREEN ILI9341_GREEN
#define TFT_BLUE ILI9341_BLUE
#define CENTER ILI9341_t3n::CENTER
#else
#include <ILI9488_t3.h>
ILI9488_t3 tft = ILI9488_t3(TFT_CS, TFT_DC, TFT_RST);
#define TFT_BLACK ILI9488_BLACK
#define TFT_YELLOW ILI9488_YELLOW
#define TFT_RED ILI9488_RED
#define TFT_GREEN ILI9488_GREEN
#define TFT_BLUE ILI9488_BLUE
#define CENTER ILI9488_t3::CENTER
#endif

// Setup framebuffers
DMAMEM uint16_t FRAME_WIDTH, FRAME_HEIGHT;

#if defined(use_sdram)
//#include "SDRAM_t4.h"
//SDRAM_t4 sdram;
uint16_t *frameBuffer = nullptr;
uint16_t *frameBuffer2 = nullptr;
#else
DMAMEM uint16_t frameBuffer[640 * 240] __attribute__((aligned(32)));
uint16_t frameBuffer2[640 * 240] __attribute__((aligned(32)));
#endif

uint32_t sizeof_framebuffer = 0;
uint32_t sizeof_framebuffer2 = 0;

// Setup display modes frame / video
bool g_continuous_flex_mode = false;
bool g_flex_dual_buffer_per_frame = false;
void *volatile g_new_flexio_data = nullptr;
void *volatile g_last_flexio_data = nullptr;
uint32_t g_flexio_capture_count = 0;
uint32_t g_flexio_redraw_count = 0;
elapsedMillis g_flexio_runtime;
bool g_dma_mode = false;

#include "jpeg_viewer.h"


void setup() {
  Serial.begin(921600);
  while (!Serial && millis() < 5000) {}
#if defined(USB_DUAL_SERIAL) || defined(USB_TRIPLE_SERIAL)
  SerialUSB1.begin(921600);
#endif

  if (CrashReport) {
    Serial.print(CrashReport);
    Serial.println("Press any key to continue");
    while (Serial.read() != -1) {}
    while (Serial.read() == -1) {}
    while (Serial.read() != -1) {}
  }


  tft.begin(15000000);
  test_display();

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
  //  If: not using default pins you may need to setup which pins are used using:
  //    camera.setPins(...) See the header file Teensy_Camera.h for details
  //          for the different pins
uint8_t reset_pin = 31;
#ifdef USE_MMOD_ATP_ADAPTER
  pinMode(30, INPUT);
  pinMode(31, INPUT_PULLUP);
  pinMode(0, OUTPUT);
  if ((_hmConfig == 0) || (_hmConfig == 2)) {
    camera.setPins(29, 10, 33, 32, 31, 40, 41, 42, 43, 44, 45, 6, 9);
  } else if (_hmConfig == 1) {
    //camera.setPins(7, 8, 33, 32, 17, 40, 41, 42, 43);
    camera.setPins(29, 10, 33, 32, 31, 40, 41, 42, 43);
  }
#elif defined(ARDUINO_TEENSY_DEVBRD4)
  pinMode(23, INPUT_PULLUP);
  if ((_hmConfig == 0) || (_hmConfig == 2)) {
    camera.setPins(7, 8, 21, 46, 23, 40, 41, 42, 43, 44, 45, 6, 9);
  } else if (_hmConfig == 1) {
    //camera.setPins(7, 8, 33, 32, 17, 40, 41, 42, 43);
    camera.setPins(7, 8, 21, 46, 31, 40, 41, 42, 43);
  }
  reset_pin = 23;

#else
  if (_hmConfig == 0) {
    //camera.setPins(29, 10, 33, 32, 31, 40, 41, 42, 43, 44, 45, 6, 9);
    camera.setPins(7, 8, 33, 32, 17, 40, 41, 42, 43, 44, 45, 6, 9);
  } else if (_hmConfig == 1) {
    camera.setPins(7, 8, 33, 32, 17, 40, 41, 42, 43);
  }
  reset_pin = 17;
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
  status = camera.begin(camera_framesize, camera_format, 15, CameraID, useGPIO);

  Serial.printf("Begin status: %d\n", status);
  if (!status) {
    Serial.println("Camera failed to start - try reset!!!");
    Serial.printf("\tPin 30:%u 31:%u\n", digitalReadFast(30), digitalReadFast(31));
    pinMode(reset_pin, OUTPUT);
    digitalWriteFast(reset_pin, LOW);
    delay(500);
    pinMode(reset_pin, INPUT_PULLUP);
    delay(500);
    status = camera.begin(camera_framesize, camera_format, 15, CameraID, useGPIO);
    if (!status) {
      Serial.println("Camera failed to start again program halted");
      while (1) {}
    }
  }


#ifdef MIRROR_FLIP_CAMERA
  camera.setHmirror(true);
  camera.setVflip(true);
#endif

#if defined(use_sdram)
  sizeof_framebuffer = sizeof_framebuffer2 = camera.width() * camera.height() * 2;
  frameBuffer  = (uint16_t *)((((uint32_t)(sdram_malloc(sizeof_framebuffer+ 32)) + 32) & 0xffffffe0));
  frameBuffer2 = (uint16_t *)((((uint32_t)(sdram_malloc(sizeof_framebuffer2 + 32)) + 32) & 0xffffffe0));
#else
  sizeof_framebuffer = sizeof(frameBuffer);
  sizeof_framebuffer2 = sizeof(frameBuffer2);
#endif
  Serial.printf("Camera Buffers: %p %p\n", frameBuffer, frameBuffer2);


  Serial.println("Camera settings:");
  Serial.print("\twidth = ");
  Serial.println(camera.width());
  Serial.print("\theight = ");
  Serial.println(camera.height());
  //Serial.print("\tbits per pixel = NA");
  //Serial.println(camera.bitsPerPixel());
  Serial.println();
  Serial.printf("TFT Width = %u Height = %u\n\n", tft.width(), tft.height());

  FRAME_HEIGHT = camera.height();
  FRAME_WIDTH = camera.width();
  Serial.printf("ImageSize (w,h): %d, %d\n", FRAME_WIDTH, FRAME_HEIGHT);

  ShowAllOptionValues();
  /**********************************************************/

}


inline uint16_t HTONS(uint16_t x) {
  return x;
}
// Pass 8-bit (each) R,G,B, get back 16-bit packed color
uint16_t color565(uint8_t r, uint8_t g, uint8_t b) {
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}


void loop() {
  int ch;
  if (Serial.available()) {
    uint8_t command = Serial.read();
    switch (command) {
      case 'x':
        processJPGFile(true);
        break;
      case 'R':
        ch = Serial.read();
        change_camera_resolution(ch);
        break;
      case 'f':
        {
          tft.useFrameBuffer(false);
          tft.fillScreen(TFT_BLACK);
          read_display_one_frame(true, true);
          ch = ' ';
          g_continuous_flex_mode = false;
          break;
        }
      case 'n':
        tft.useFrameBuffer(false);
        tft.fillScreen(TFT_BLACK);
        read_display_one_frame(false, true);
        ch = ' ';
        g_continuous_flex_mode = false;
        break;
      case 't':
        test_display();
        break;
      default:
        break;
    }
    while (Serial.read() != -1)
      ;  // lets strip the rest out
  }
  
}

//=============================================================================
void read_display_one_frame(bool use_dma, bool show_debug_info) {

  uint32_t count_pixels_in_buffer = sizeof_framebuffer / sizeof(frameBuffer[0]);
  if (show_debug_info) {
    Serial.println("Reading frame");
    Serial.printf("Buffer1: %p(%u) halfway: %p end:%p\n", frameBuffer, sizeof_framebuffer, &frameBuffer[count_pixels_in_buffer / 2], &frameBuffer[count_pixels_in_buffer]);
    count_pixels_in_buffer = sizeof_framebuffer2 / sizeof(frameBuffer2[0]);
    Serial.printf("Buffer2: %p(%u) halfway: %p end:%p\n", frameBuffer2, sizeof_framebuffer2, &frameBuffer2[count_pixels_in_buffer / 2], &frameBuffer2[count_pixels_in_buffer]);
    memset((uint8_t *)frameBuffer, 0, sizeof_framebuffer);
    memset((uint8_t *)frameBuffer2, 0, sizeof_framebuffer2);
  }
  //  digitalWriteFast(24, HIGH);
  camera.useDMA(use_dma);
  camera.readFrame(frameBuffer, sizeof_framebuffer, frameBuffer2, sizeof_framebuffer2);
  //  digitalWriteFast(24, LOW);

  if (show_debug_info) {
    Serial.println("Finished reading frame");
    Serial.flush();
#if (defined(DVP_CAMERA_OV7675) || defined(DVP_CAMERA_OV7670) || defined(DVP_CAMERA_OV2640) || defined(DVP_CAMERA_GC2145) || defined(DVP_CAMERA_OV5640))
    for (volatile uint16_t *pfb = frameBuffer; pfb < (frameBuffer + 4 * camera.width()); pfb += camera.width()) {
#else
    for (volatile uint8_t *pfb = frameBuffer; pfb < (frameBuffer + 4 * camera.width()); pfb += camera.width()) {
#endif
      Serial.printf("\n%08x: ", (uint32_t)pfb);
      for (uint16_t i = 0; i < 8; i++) Serial.printf("%02x ", pfb[i]);
      Serial.print("..");
      Serial.print("..");
      for (uint16_t i = camera.width() - 8; i < camera.width(); i++) Serial.printf("%04x ", pfb[i]);
    }
    Serial.println("\n");
#if 0  // Figure this out later...
       // Lets dump out some of center of image.
            Serial.println("Show Center pixels\n");
#if defined(DVP_CAMERA_OV7675) || defined(DVP_CAMERA_OV7670)
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
    Serial.printf("TFT(%u, %u) Camera(%u, %u)\n", tft.width(), tft.height(), camera.width(), camera.height());
  }
  //tft.setOrigin(-2, -2);
  int numPixels = camera.width() * camera.height();
#ifndef CAMERA_USES_MONO_PALETTE
//#if defined(DVP_CAMERA_OV7675) || defined(DVP_CAMERA_OV7670)

//int camera_width = Camera.width();
#if 0  //def ARDUINO_TEENSY_DEVBRD4
  for (int i = 0; i < numPixels; i++) frameBuffer[i] = HTONS(frameBuffer[i]);
  tft.writeRect(CENTER, CENTER, camera.width(), camera.height(), frameBuffer);
  return;
#endif

#if 1
  //byte swap
  //for (int i = 0; i < numPixels; i++) frameBuffer[i] = (frameBuffer[i] >> 8) | (((frameBuffer[i] & 0xff) << 8));
  // now see if all fit into one buffer or part in second...
  int numPixels1 = min((int)(sizeof_framebuffer / 2), numPixels);
  int numPixels2 = min((int)(sizeof_framebuffer2 / 2), numPixels - numPixels1);
  if (show_debug_info) Serial.printf("\tBuffers:%p(%u) %p(%u)\n", frameBuffer, sizeof_framebuffer, frameBuffer2, sizeof_framebuffer2);
  if (show_debug_info) Serial.printf("\tnumpixels %u %u %u\n", numPixels, numPixels1, numPixels2);
  for (int i = 0; i < numPixels1; i++) frameBuffer[i] = HTONS(frameBuffer[i]);
  for (int i = 0; i < numPixels2; i++) frameBuffer2[i] = HTONS(frameBuffer2[i]);

  if (numPixels2 == 0) {
    if ((camera.width() <= tft.width()) && (camera.height() <= tft.height())) {
      //if ((camera.width() != tft.width()) || (camera.height() != tft.height())) tft.fillScreen(TFT_BLACK);
      tft.writeRect(CENTER, CENTER, camera.width(), camera.height(), frameBuffer);
    } else {
      Serial.println("sub image");
      tft.writeSubImageRect(0, 0, tft.width(), tft.height(), (camera.width() - tft.width()) / 2, (camera.height() - tft.height()),
                            camera.width(), camera.height(), frameBuffer);
    }
  } else {
    // We used both buffers.  Assume for now each buffer is in multples of Width
    //    tft.writeRect(CENTER, CENTER, camera.width(), camera.height(), frameBuffer);
    int start_x = (tft.width() - camera.width()) / 2;
    int start_y = (tft.height() - camera.height()) / 2;
    tft.writeRect(start_x, start_y, camera.width(), numPixels1 / camera.width(), frameBuffer);

    // now try second part
    start_y += numPixels1 / camera.width();
    tft.writeRect(start_x, start_y, camera.width(), numPixels2 / camera.width(), frameBuffer2);
  }

#else
  Serial.println("sub image1");
  tft.writeSubImageRect(0, 0, tft.width(), tft.height(), 0, 0, camera.width(), camera.height(), pixels);
#endif
#else
  //tft.writeRect8BPP(0, 0, FRAME_WIDTH, FRAME_HEIGHT, frameBuffer, mono_palette);
  int numPixels1 = min((int)(sizeof_framebuffer), numPixels);
  int start_x = (tft.width() - camera.width()) / 2;
  int start_y = (tft.height() - camera.height()) / 2;
  int num_rows = numPixels1 / camera.width();
  Serial.printf("start(%d, %d, %d, %d) cnt:%d\n", start_x, start_y, camera.width(), num_rows, numPixels1);
  tft.writeRect8BPP(start_x, start_y, camera.width(), num_rows, frameBuffer, mono_palette);

  if (numPixels1 < numPixels) {
    // now try second part
    start_y += num_rows;
    int numPixels2 = min((int)(sizeof_framebuffer2), numPixels - numPixels1);
    num_rows = numPixels2 / camera.width();

    Serial.printf("start(%d, %d, %d, %d): cnt:%d %02x %02x %02x %02x\n", start_x, start_y, camera.width(), num_rows, numPixels2, frameBuffer2[0], frameBuffer2[1], frameBuffer2[2], frameBuffer2[3]);
    tft.writeRect8BPP(start_x, start_y, camera.width(), num_rows, frameBuffer2, mono_palette);
  }
#endif
}

void change_camera_resolution(int ch) {
  while (ch == ' ') ch = Serial.read();
  framesize_t fs = FRAMESIZE_INVALID;
  switch (ch) {
    case 'q':
    case 'Q':
      Serial.println("Switching to QVGA mode");
      fs = FRAMESIZE_QVGA;
      break;
    case 'v':
    case 'V':
      Serial.println("Switching to VGA mode");
      fs = FRAMESIZE_VGA;
      break;
    case 's':
    case 'S':
      Serial.println("Switching to SVGA mode");
      fs = FRAMESIZE_SVGA;
      break;
    case 'u':
    case 'U':
      Serial.println("Switching to UXGA mode");
      fs = FRAMESIZE_UXGA;
      break;
    default: 
      Serial.println("Unknown size option");
  }
  if (fs != FRAMESIZE_INVALID) {
    camera.setFramesize(fs);
    camera.setPixformat(camera_format);
  }
}


uint32_t readJPG(uint8_t &eoi_jpg, uint32_t &eop_jpg, bool debug_on) {
  if (camera_format != JPEG) {
    camera.setPixformat(JPEG);
    delay(1000);
  }
  //omni.setQuality(12);

  camera.setMode(HIMAX_MODE_STREAMING_NFRAMES, 1);
  uint32_t count_pixels_in_buffer = sizeof_framebuffer / sizeof(frameBuffer[0]);

  if (debug_on) {
    Serial.println("Reading frame");
    Serial.printf("Buffer1: %p(%u) halfway: %p end:%p\n", frameBuffer, sizeof_framebuffer, &frameBuffer[count_pixels_in_buffer / 2], &frameBuffer[count_pixels_in_buffer]);
    count_pixels_in_buffer = sizeof_framebuffer2 / sizeof(frameBuffer2[0]);
    Serial.printf("Buffer2: %p(%u) halfway: %p end:%p\n", frameBuffer2, sizeof_framebuffer2, &frameBuffer2[count_pixels_in_buffer / 2], &frameBuffer2[count_pixels_in_buffer]);
    memset((uint8_t *)frameBuffer, 0, sizeof_framebuffer);
    memset((uint8_t *)frameBuffer2, 0, sizeof_framebuffer2);
  }

  uint16_t w = FRAME_WIDTH;
  uint16_t h = FRAME_HEIGHT;

  eop_jpg = 0;
  eoi_jpg = 0;

  uint32_t numPixels = w * h;
  uint32_t jpegSize = (w * h) / 5;
  if (debug_on) {
    Serial.printf("Width: %d, Height: %d\n", w, h);
    Serial.printf("jpeg size: %d\n", jpegSize);
  }

  //byte swap
  //for (int i = 0; i < numPixels; i++) frameBuffer[i] = (frameBuffer[i] >> 8) | (((frameBuffer[i] & 0xff) << 8));
  // now see if all fit into one buffer or part in second...
  uint32_t numPixels1 = min((sizeof_framebuffer / 2), numPixels);
  uint32_t numPixels2 = min((sizeof_framebuffer2 / 2), numPixels - numPixels1);
  if (debug_on) Serial.printf("\tBuffers:%p(%u) %p(%u)\n", frameBuffer, sizeof_framebuffer, frameBuffer2, sizeof_framebuffer2);
  if (debug_on) Serial.printf("\tnumpixels %u %u %u\n", numPixels, numPixels1, numPixels2);
  //for (int i = 0; i < numPixels1; i++) frameBuffer[i] = HTONS(frameBuffer[i]);
  //for (int i = 0; i < numPixels2; i++) frameBuffer2[i] = HTONS(frameBuffer2[i]);

  if (jpegSize > (sizeof_framebuffer + sizeof_framebuffer2)) return false;


  //  digitalWriteFast(24, HIGH);
  uint32_t bytes_read = 0;
  camera.useDMA(false);
  if (camera.usingGPIO()) {
      bytes_read = omni.readFrameGPIO_JPEG(frameBuffer, sizeof_framebuffer, frameBuffer2, sizeof_framebuffer2);
    //delay(1000);
    //omni.readFrameGPIO_JPEG(frameBuffer, sizeof_framebuffer);
  } else {
    bytes_read = omni.readFrame(frameBuffer, sizeof_framebuffer, frameBuffer2, sizeof_framebuffer2);
    //delay(1000);
    //camera.readFrame(frameBuffer, sizeof_framebuffer, frameBuffer2, sizeof_framebuffer2);
  }
  //delay(1000);
  //  digitalWriteFast(24, LOW);

  if (debug_on) Serial.printf("Bytes returned from camera: %u\n", bytes_read);

  if (bytes_read == 0) {
    if (debug_on) Serial.printf("Error: No bytes returned from camera\n");
    return 0;
  }

  // verify that the start of data returned has valid marker... assumes always in first buffer
  uint8_t *pfb = (uint8_t *)frameBuffer;
  if (debug_on) MemoryHexDump(Serial, pfb, 128, true, "SOF:\n");
  if ((pfb[0] != 0xff) || (pfb[1] != 0xd8) || (pfb[2] != 0xff)) {
    if (debug_on) Serial.printf("begining of frame not found at position 0\n");
    return 0;
  }
  eoi_jpg = 0;

  // Now lets try to verify the returned size for the end marker.
  if (bytes_read < sizeof_framebuffer) {
    if (debug_on) MemoryHexDump(Serial, pfb + bytes_read - 63, 64, true, "\nEOF:\n");
    if ((pfb[bytes_read - 2] != 0xFF) || (pfb[bytes_read - 1] != 0xd9)) {
      if (debug_on) Serial.printf("Invalid frame ending: %02x %02x\n", pfb[bytes_read - 2], pfb[bytes_read - 1]);
      return 0;
    }
  } else {
    uint8_t *pfb2 = (uint8_t *)frameBuffer2;
    uint32_t bytes_read_in_2 = bytes_read - sizeof_framebuffer;
    uint32_t bytes_dump = min(bytes_read_in_2, 64);
    if (debug_on) MemoryHexDump(Serial, pfb2 + bytes_read_in_2 - bytes_dump + 1, bytes_dump, true);
    if ((pfb2[bytes_read_in_2 - 2] != 0xFF) || (pfb2[bytes_read_in_2 - 1] != 0xd9)) {
      if (debug_on) Serial.printf("Invalid frame ending(2): %02x %02x\n", pfb[bytes_read_in_2 - 2], pfb[bytes_read_in_2 - 1]);
      return 0;
    }
  }

  eop_jpg = bytes_read;

  if (camera_format != JPEG) {
    camera.setPixformat(camera_format);
    delay(500);
  }

  camera.useDMA(true);
  return bytes_read;
}


void test_display() {
  tft.setRotation(3);
  tft.fillScreen(TFT_RED);
  delay(500);
  tft.fillScreen(TFT_GREEN);
  delay(500);
  tft.fillScreen(TFT_BLUE);
  delay(500);
  tft.fillScreen(TFT_BLACK);
  delay(500);
}
