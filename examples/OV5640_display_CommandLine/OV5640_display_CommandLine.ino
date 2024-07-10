#include <stdint.h>

#include <SPI.h>
#include <JPEGDEC.h>
#include <MemoryHexDump.h>

#include "Teensy_Camera.h"

//#define USE_MMOD_ATP_ADAPTER
//#define USE_SDCARD
//#define useILI9341

#define DVP_CAMERA_OV5640

#include "Teensy_OV5640/OV5640.h"
OV5640 omni;
Camera camera(omni);
#define CameraID OV5640a
#define MIRROR_FLIP_CAMERA


//set cam configuration - need to remember when saving jpeg
framesize_t camera_framesize = FRAMESIZE_QVGA;
pixformat_t camera_format = RGB565;
bool useGPIO = false;
bool useAutoFocus = false;

#define skipFrames 1

/************** Set up MTP Disk *************/
#if defined(USE_SDCARD)
#include <SD.h>
#include <MTP_Teensy.h>
#include <LittleFS.h>

File file;

//Used a store of index file
LittleFS_Program lfsProg;  // Used to create FS on the Flash memory of the chip
FS *myfs = &lfsProg;       // current default FS...
static const uint32_t file_system_size = 1024 * 512;
uint8_t current_store = 0;
uint8_t storage_index = '0';

#define SPI_SPEED SD_SCK_MHZ(50)  // adjust to sd card
elapsedMillis elapsed_millis_since_last_sd_check = 0;
#define TIME_BETWEEN_SD_CHECKS_MS 1000
bool sdio_previously_present;

const char *sd_str[] = { "SD1" };  // edit to reflect your configuration
#if MMOD_ML == 1
const int cs[] = { 10 };  // edit to reflect your configuration
#else
//const char *sd_str[]={"BUILTIN"}; // edit to reflect your configuration
const int cs[] = { BUILTIN_SDCARD };  // edit to reflect your configuration
#endif
const int cdPin[] = { 0xff };
const int nsd = sizeof(sd_str) / sizeof(const char *);
bool sd_media_present_prev[nsd];

SDClass sdx[nsd];
#endif

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

#elif defined(ARDUINO_TEENSY_DEVBRD5)
#undef USE_MMOD_ATP_ADAPTER
#define TFT_CS 63  // AD_B0_02
#define TFT_DC 61  // AD_B0_03
#define TFT_RST 62
#define DB5_USE_CSI

#elif defined(ARDUINO_TEENSY41)
#undef USE_MMOD_ATP_ADAPTER
// My T4.1 Camera board
#define TFT_DC 9
#define TFT_CS 7
#define TFT_RST 8

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
#if defined(ARDUINO_TEENSY_DEVBRD4) || defined(ARDUINO_TEENSY_DEVBRD5)
//#include "SDRAM_t4.h"
//SDRAM_t4 sdram;
extern "C" bool sdram_begin(uint8_t external_sdram_size, uint8_t clock, uint8_t useDQS);
uint16_t *frameBuffer = nullptr;
uint16_t *frameBuffer2 = nullptr;
uint16_t *frameBufferSDRAM = nullptr;
uint16_t *frameBufferSDRAM2 = nullptr;
DMAMEM uint16_t frameBufferM[640 * 240] __attribute__((aligned(32)));
uint16_t frameBufferM2[640 * 240] __attribute__((aligned(32)));
uint32_t sizeof_framebuffer = 0;
uint32_t sizeof_framebuffer2 = 0;
uint32_t sizeof_framebufferSDRAM = 0;
#else
#if defined(USE_SDCARD)
DMAMEM uint16_t frameBuffer[640 * 240] __attribute__((aligned(32)));
uint16_t frameBuffer2[480 * 240] __attribute__((aligned(32)));
#else
DMAMEM uint16_t frameBuffer[640 * 240] __attribute__((aligned(32)));
uint16_t frameBuffer2[640 * 240] __attribute__((aligned(32)));
#endif
//#define SCREEN_ROTATION 1
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

//This is mandatory to begin the d session.
#if defined(USE_SDCARD)
  storage_configure();
  MTP.begin();
#endif

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
  if (useAutoFocus)
    omni.enableAutoFocus(true);

#if defined(DB5_USE_CSI)
    // try using the CSI pins on devboard 5
    reset_pin = 57;
    powdwn_pin = 58;
    camera.setPins(65,  64, 17, 16, 57, 27, 26, 67, 66, 21, 20, 23, 22, 58);
 #endif
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

#if defined(ARDUINO_TEENSY_DEVBRD4) || defined(ARDUINO_TEENSY_DEVBRD5)
  sdram_begin(32, 221, 1);
  sizeof_framebufferSDRAM = sizeof_framebuffer = sizeof_framebuffer2 = camera.width() * camera.height() * 2;
  frameBufferSDRAM = frameBuffer = (uint16_t *)((((uint32_t)(sdram_malloc(camera.width() * camera.height() * 2 + 32)) + 32) & 0xffffffe0));
  frameBufferSDRAM2 = frameBuffer2 = (uint16_t *)((((uint32_t)(sdram_malloc(camera.width() * camera.height() * 2 + 32)) + 32) & 0xffffffe0));
  Serial.printf("Camera Buffers: %p %p\n", frameBuffer, frameBuffer2);
#endif

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


#if defined(USE_SDCARD)
  File optionsFile = SD.open(options_file_name);

  if (optionsFile) {
    ProcessOptionsFile(optionsFile);
    optionsFile.close();
  }
#endif

  ShowAllOptionValues();
  /**********************************************************/
  camera.showRegisters();
  showCommandList();
}

bool hm0360_flexio_callback(void *pfb) {
  //Serial.println("Flexio callback");
  g_new_flexio_data = pfb;
  return true;
}


// Quick and Dirty
#define UPDATE_ON_CAMERA_FRAMES

inline uint16_t HTONS(uint16_t x) {
  return x;
}

volatile uint16_t *pfb_last_frame_returned = nullptr;

bool camera_flexio_callback_video(void *pfb) {
  pfb_last_frame_returned = (uint16_t *)pfb;
#ifdef UPDATE_ON_CAMERA_FRAMES
  if ((uint32_t)pfb_last_frame_returned >= 0x20200000u)
    arm_dcache_delete((void *)pfb_last_frame_returned, FRAME_WIDTH * FRAME_HEIGHT * 2);
  int numPixels = camera.width() * camera.height();

  for (int i = 0; i < numPixels; i++) pfb_last_frame_returned[i] = HTONS(pfb_last_frame_returned[i]);

  tft.writeRect(0, 0, FRAME_WIDTH, FRAME_HEIGHT, (uint16_t *)pfb_last_frame_returned);
  pfb_last_frame_returned = nullptr;
  tft.setOrigin(0, 0);
  uint16_t *pframebuf = tft.getFrameBuffer();
  if ((uint32_t)pframebuf >= 0x20200000u) arm_dcache_flush(pframebuf, FRAME_WIDTH * FRAME_HEIGHT);
#endif
  //Serial.print("#");
  return true;
}

void frame_complete_cb() {
  //Serial.print("@");
#ifndef UPDATE_ON_CAMERA_FRAMES
  if (!pfb_last_frame_returned) return;
  if ((uint32_t)pfb_last_frame_returned >= 0x20200000u)
    arm_dcache_delete(pfb_last_frame_returned, FRAME_WIDTH * FRAME_HEIGHT * 2);
  tft.writeSubImageRectBytesReversed(0, 0, FRAME_WIDTH, FRAME_HEIGHT, 0, 0, FRAME_WIDTH, FRAME_HEIGHT, pfb_last_frame_returned);
  pfb_last_frame_returned = nullptr;
  uint16_t *pfb = tft.getFrameBuffer();
  if ((uint32_t)pfb >= 0x20200000u) arm_dcache_flush(pfb, FRAME_WIDTH * FRAME_HEIGHT);
#endif
}


void loop() {
  int ch;
  if (Serial.available()) {
    uint8_t command = Serial.read();
#if defined(USE_SDCARD)
    if ('2' == command) storage_index = CommandLineReadNextNumber(getValue(), 0);
#endif
    switch (command) {
      case 'C':
        changeContrast();
        break;
      case 'B':
        changeBrightness();
        break;
      case 'S':
        changeSaturation();
        break;
      case 'E':
        changeSpecialEffects();
        break;
      case 'W':
        changeLight();
        break;
      case 'H':
        changeHue();
        break;
      case 'N':
        changeNightMode();
        break;
      case 'A':
        changeAutoSharpness();
        break;
      case 'P':
        changeSharpness();
        break;
      case 'L':
        changeLensCorrection();
        break;
      case 'z':
        {
#if defined(USE_SDCARD)
          save_image_SD();
#endif
          break;
        }
      case 'j':
        {
#if (defined(USE_SDCARD) && (defined(DVP_CAMERA_OV2640) || defined(DVP_CAMERA_OV5640)))
          bool error = false;
          error = save_jpg_SD();
          if (!error) Serial.println("ERROR reading JPEG.  Try again....");
#else
          Serial.println("Error USE_SDCARD - option not enabled...");
#endif
          break;
        }
      case 'b':
        {
#if defined(USE_SDCARD)
          memset((uint8_t *)frameBuffer, 0, sizeof_framebuffer);
          camera.setMode(HIMAX_MODE_STREAMING_NFRAMES, 1);
          //camera.readFrame(frameBuffer);
          camera.readFrame(frameBuffer, sizeof_framebuffer, frameBuffer2, sizeof_framebuffer2);

          int numPixels = camera.width() * camera.height();
          //for (int i = 0; i < numPixels; i++) frameBuffer[i] = HTONS(frameBuffer[i]);
          int numPixels1 = min((int)(sizeof_framebuffer / 2), numPixels);
          int numPixels2 = min((int)(sizeof_framebuffer2 / 2), numPixels - numPixels1);
          for (int i = 0; i < numPixels1; i++) frameBuffer[i] = HTONS(frameBuffer[i]);
          for (int i = 0; i < numPixels2; i++) frameBuffer2[i] = HTONS(frameBuffer2[i]);
          save_image_SD();
#endif
          ch = ' ';
          break;
        }
      case 'm':
        read_display_multiple_frames(false);
        break;

      case 'M':
        read_display_multiple_frames(true);
        break;
      case 'x':
        processJPGFile(true);
        break;
      case 't':
        test_display();
        break;
      case 'r':
        camera.showRegisters();
        break;
      case 'R':
        ch = Serial.read();
        change_camera_resolution(ch);
        break;
      case 'd':
        camera.debug(!camera.debug());
        if (camera.debug()) Serial.println("Camera Debug turned on");
        else Serial.println("Camera debug turned off");
        break;
#if defined(ARDUINO_TEENSY_DEVBRD4) || defined(ARDUINO_TEENSY_DEVBRD5)
      case 's':
        {
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
          Serial.printf("%p(%u) %p(%u)\n", frameBuffer, sizeof_framebuffer, frameBuffer2, sizeof_framebuffer2);
        }
        break;
#endif
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
      case 'F':
        {
          if (!g_continuous_flex_mode) {
            if (camera.readContinuous(&hm0360_flexio_callback, frameBuffer, sizeof_framebuffer, frameBuffer2, sizeof_framebuffer2)) {
              Serial.println("* continuous mode started");
              g_flex_dual_buffer_per_frame = sizeof_framebuffer < (FRAME_WIDTH * FRAME_HEIGHT * sizeof(frameBuffer[0]));
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
      case 'V':
        {
          if (!g_continuous_flex_mode) {
            if (!tft.useFrameBuffer(true)) {
              Serial.println("Failed call to useFrameBuffer");
            } else if (camera.readContinuous(&camera_flexio_callback_video, frameBuffer, sizeof_framebuffer, frameBuffer2, sizeof_framebuffer2)) {

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
              g_flex_dual_buffer_per_frame = sizeof_framebuffer < (FRAME_WIDTH * FRAME_HEIGHT * sizeof(frameBuffer[0]));
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
#if defined(USE_SDCARD)
        uint32_t fsCount;
      case '1':
        // first dump list of storages:
        fsCount = MTP.getFilesystemCount();
        Serial.printf("\nDump Storage list(%u)\n", fsCount);
        for (uint32_t ii = 0; ii < fsCount; ii++) {
          Serial.printf("store:%u storage:%x name:%s fs:%x pn:\n", ii,
                        MTP.Store2Storage(ii), MTP.getFilesystemNameByIndex(ii),
                        (uint32_t)MTP.getFilesystemByIndex(ii));
          /*    char dest[12];     // Destination string
          char key[] = "MSC";
          strncpy(dest, MTP.getFilesystemNameByIndex(ii), 3);
          if(strcmp(key, dest) == 0) {
            DBGSerial.println(getFSPN(ii));
            DBGSerial.println("USB Storage");
          } else {
            DBGSerial.println(getFSPN(ii));
          }
       */
          //Serial.println(getFSPN(ii));
        }
        //DBGSerial.println("\nDump Index List");
        //MTP.storage()->dumpIndexList();
        break;
      case '2':
        if (storage_index < MTP.getFilesystemCount()) {
          Serial.printf("Storage Index %u Name: %s Selected\n", storage_index,
                        MTP.getFilesystemNameByIndex(storage_index));
          myfs = MTP.getFilesystemByIndex(storage_index);
          current_store = storage_index;
        } else {
          Serial.printf("Storage Index %u out of range\n", storage_index);
        }
        break;
      case '3':
        Serial.println("MTP Reset");
        MTP.send_DeviceResetEvent();
        break;
      case 'l': listFiles(); break;
      case 'e': eraseFiles(); break;
#endif

      case '?':
        {
          showCommandList();
          ch = ' ';
          break;
        }
      default:
        break;
    }
    while (Serial.read() != -1)
      ;  // lets strip the rest out
  }

  if (g_continuous_flex_mode) {
    // try to make sure we alternate.
    if (g_new_flexio_data && (!g_flex_dual_buffer_per_frame || (g_new_flexio_data != g_last_flexio_data))) {
      //Serial.println("new FlexIO data");
      int numPixels = camera.width() * camera.height();
      g_last_flexio_data = g_new_flexio_data;

      if (g_flex_dual_buffer_per_frame) {
        int numPixels1 = min((int)(sizeof_framebuffer / 2), numPixels);

        if (g_new_flexio_data == frameBuffer) {
          for (int i = 0; i < numPixels1; i++) frameBuffer[i] = HTONS(frameBuffer[i]);
          int start_x = (tft.width() - camera.width()) / 2;
          int start_y = (tft.height() - camera.height()) / 2;
          tft.writeRect(start_x, start_y, camera.width(), numPixels1 / camera.width(), frameBuffer);
          //Serial.println("1");

        } else {
          int numPixels2 = min((int)(sizeof_framebuffer2 / 2), numPixels - numPixels1);
          for (int i = 0; i < numPixels2; i++) frameBuffer2[i] = HTONS(frameBuffer2[i]);
          int start_x = (tft.width() - camera.width()) / 2;
          int start_y = ((tft.height() - camera.height()) / 2) + (numPixels1 / camera.width());
          tft.writeRect(start_x, start_y, camera.width(), numPixels2 / camera.width(), frameBuffer2);
          //Serial.println("2");
        }

      } else {
        for (int i = 0; i < numPixels; i++) frameBuffer[i] = HTONS(frameBuffer[i]);
        tft.writeRect(CENTER, CENTER, camera.width(), camera.height(), (uint16_t *)g_new_flexio_data);
        //Serial.printf("T: %p\n", g_new_flexio_data);
      }

      tft.updateScreenAsync();
      g_new_flexio_data = nullptr;
      g_flexio_redraw_count++;
      if (g_flexio_runtime > 10000) {
        // print some stats on actual speed, but not too much
        // printing too quickly to be considered "spew"
        float redraw_rate = (float)g_flexio_redraw_count / (float)g_flexio_runtime * 1000.0f;
        g_flexio_runtime = 0;
        g_flexio_redraw_count = 0;
        Serial.printf("redraw rate = %.2f Hz\n", redraw_rate);
      }
    }
  }

#if defined(USE_SDCARD)
  MTP.loop();
#endif
}

// Pass 8-bit (each) R,G,B, get back 16-bit packed color
uint16_t color565(uint8_t r, uint8_t g, uint8_t b) {
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}


#if defined(USE_SDCARD)
char name[] = "9px_0000.bmp";  // filename convention (will auto-increment)
  // can probably reuse framebuffer2...

//DMAMEM unsigned char img[3 * 320*240];
void save_image_SD() {
  //uint8_t r, g, b;
  //uint32_t x, y;

  Serial.print("Writing BMP to SD CARD File: ");

  // if name exists, create new filename, SD.exists(filename)
  for (int i = 0; i < 10000; i++) {
    name[4] = (i / 1000) % 10 + '0';  // thousands place
    name[5] = (i / 100) % 10 + '0';   // hundreds
    name[6] = (i / 10) % 10 + '0';    // tens
    name[7] = i % 10 + '0';           // ones
    if (!myfs->exists(name)) {
      Serial.println(name);
      file = myfs->open(name, FILE_WRITE);
      break;
    }
  }

  uint16_t w = FRAME_WIDTH;
  uint16_t h = FRAME_HEIGHT;

  //unsigned char *img = NULL;
  // set fileSize (used in bmp header)
  int rowSize = 4 * ((3 * w + 3) / 4);  // how many bytes in the row (used to create padding)
  int fileSize = 54 + h * rowSize;      // headers (54 bytes) + pixel data

  //  img = (unsigned char *)malloc(3 * w * h);

  // create padding (based on the number of pixels in a row
  unsigned char bmpPad[rowSize - 3 * w];
  for (int i = 0; i < (int)(sizeof(bmpPad)); i++) {  // fill with 0s
    bmpPad[i] = 0;
  }

  unsigned char bmpFileHeader[14] = { 'B', 'M', 0, 0, 0, 0, 0, 0, 0, 0, 54, 0, 0, 0 };
  unsigned char bmpInfoHeader[40] = { 40, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 24, 0 };

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
  file.write(bmpFileHeader, sizeof(bmpFileHeader));  // write file header
  file.write(bmpInfoHeader, sizeof(bmpInfoHeader));  // " info header

  // try to compute and output one row at a time.
  uint16_t *pfb = frameBuffer;
  uint8_t img[3];
  uint32_t count_y_first_buffer = sizeof_framebuffer / (w * 2);
  for (int y = h - 1; y >= 0; y--) {  // iterate image array
    if (y < (int)count_y_first_buffer) pfb = &frameBuffer[y * w];
    else pfb = &frameBuffer2[(y - count_y_first_buffer) * w];
    for (int x = 0; x < w; x++) {
      //r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3
      img[2] = (*pfb >> 8) & 0xf8;  // r
      img[1] = (*pfb >> 3) & 0xfc;  // g
      img[0] = (*pfb << 3);         // b
      file.write(img, 3);
      pfb++;
    }
    file.write(bmpPad, (4 - (FRAME_WIDTH * 3) % 4) % 4);  // and padding as needed
  }
  file.close();  // close file when done writing
  Serial.println("Done Writing BMP");
}

char name_jpg[] = "9px_0000.jpg";  // filename convention (will auto-increment)
  // can probably reuse framebuffer2...

bool save_jpg_SD() {

  // if name exists, create new filename, SD.exists(filename)
  for (int i = 0; i < 10000; i++) {
    name_jpg[4] = (i / 1000) % 10 + '0';  // thousands place
    name_jpg[5] = (i / 100) % 10 + '0';   // hundreds
    name_jpg[6] = (i / 10) % 10 + '0';    // tens
    name_jpg[7] = i % 10 + '0';           // ones
    if (!myfs->exists(name_jpg)) {
      Serial.println(name_jpg);
      file = myfs->open(name_jpg, FILE_WRITE);
      break;
    }
  }

  uint8_t eoi = 0;
  uint32_t eop = 0;

  uint32_t status = 0;
  status = readJPG(eoi, eop, false);
  if (status == 0) return false;

  uint16_t *pfb = frameBuffer;
  Serial.printf(F("Writing JPEG to SD: %s\n"), name_jpg);

  if (eop <= sizeof_framebuffer) {
    // only used first buffer.
    file.write((uint8_t *)frameBuffer, eop);
  } else {
    // used both buffers.
    file.write((uint8_t *)frameBuffer, sizeof_framebuffer);
    file.write((uint8_t *)frameBuffer2, eop - sizeof_framebuffer);
  }

  file.close();  // close file when done writing
  Serial.println("Done Writing JPG");

  delay(50);
  return true;
}

#endif

void showCommandList() {
  Serial.println("************ Camera Settings **************/");
  Serial.println("Send the 'C[-3 to +3]' To set Contrast");
  Serial.println("Send the 'S[-4 to +4]' To set Saturation");
  Serial.println("Send the 'B[-4 to +4]' To set Brightness");
  Serial.println("Send the 'E[0-9]' To set Special Effects");
  Serial.println("Send the 'W[0-4]' To set Light Mode");
  Serial.println("Send the 'H[-6 to +5]' To set Hue");
  Serial.println("Send the 'N[on/Off]' To turn on Nightmode");
  Serial.println("Send the 'A[on/Off]' To turn on AutoSharpness");
  Serial.println("Send the 'P[0-9]' To set Sharpness");
  Serial.println("Send the 'L[on/Off]' To turn on LensCorrection");
  Serial.println("Send the 'R[QVSU]' To set image size QVGA, VGA, SVGA UXGA");
  Serial.println("************ Single Frame **************/");
  if (camera.usingGPIO()) {
    Serial.println("Send the 'f' character to read a frame using GPIO");
    Serial.println("Send the 'F' to start/stop continuous using GPIO");
    Serial.println("Send the 'n' character to read a frame using GPIO without DMA?");
  } else {
    Serial.println("Send the 'f' character to read a frame using FlexIO (changes hardware setup!)");
    Serial.println("Send the 'F' to start/stop continuous using FlexIO (changes hardware setup!)");
    Serial.println("Send the 'n' character to read a frame using FlexIO without DMA (changes hardware setup!)");
  }
  Serial.println("Send the 'x' send snapshot (JPEG) to display");

  Serial.println("************ Multi Frame **************/");
  Serial.println("Send the 'm' character to read and display multiple frames");
  Serial.println("Send the 'M' character to read and display multiple frames use Frame buffer");
  Serial.println("Send the 'V' character DMA to TFT async continueous  ...");
  Serial.println("************ Save Frames **************/");
  Serial.println("Send the 'b' character to save snapshot (BMP) to SD Card");
  Serial.println("Send the 'j' character to save snapshot (JPEG) to SD Card");
  Serial.println("Send the 'z' character to send current screen BMP to SD");
  Serial.println("************ INFO **************/");

  Serial.println("Send the 't' character to send Check the display");
  Serial.println("Send the 'd' character to toggle camera debug on and off");
  Serial.println("Send the 'r' character to show the current camera registers");

#if defined(ARDUINO_TEENSY_DEVBRD4) || defined(ARDUINO_TEENSY_DEVBRD5)
  Serial.println("************ SDRAM if available **************/");
  Serial.println("Send the 's' to change if using SDRAM or other memory");
#endif
#if defined(USE_SDCARD)
  Serial.println("/**************  MTP FUNCTIONS: ********************/");
  Serial.println("\t1 - List Drives (Step 1)");
  Serial.println("\t2# - Select Drive # for Logging (Step 2)");
  Serial.println("\tl - List files on disk");
  Serial.println("\te - Erase files on disk with Format");
  Serial.println();
#endif
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
  omni.useDMA(use_dma);
  camera.readFrame(frameBuffer, sizeof_framebuffer, frameBuffer2, sizeof_framebuffer2);
  //digitalWriteFast(24, LOW);

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
#if 0  // Figure this out later... \
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

//=============================================================================
void read_display_multiple_frames(bool use_frame_buffer) {
  if (use_frame_buffer) {
    Serial.println("\n*** Read and display multiple frames (using async screen updates), press any key to stop ***");
    tft.useFrameBuffer(true);
  } else {
    Serial.println("\n*** Read and display multiple frames, press any key to stop ***");
  }

  while (Serial.read() != -1) {}

  elapsedMicros em = 0;
  int frame_count = 0;

  for (;;) {

#if 1
    if (use_frame_buffer) tft.waitUpdateAsyncComplete();
    read_display_one_frame(true, false);
#else
    camera.readFrame(frameBuffer, sizeof_framebuffer);

    int numPixels = camera.width() * camera.height();

//byte swap
//for (int i = 0; i < numPixels; i++) frameBuffer[i] = (frameBuffer[i] >> 8) | (((frameBuffer[i] & 0xff) << 8));
#ifdef CAMERA_USES_MONO_PALETTE
    tft.writeRect8BPP(0, 0, FRAME_WIDTH, FRAME_HEIGHT, frameBuffer, mono_palette);
#else
    for (int i = 0; i < numPixels; i++) frameBuffer[i] = HTONS(frameBuffer[i]);

    if (use_frame_buffer) tft.waitUpdateAsyncComplete();

    if ((camera.width() <= tft.width()) && (camera.height() <= tft.height())) {
      //if ((camera.width() != tft.width()) || (camera.height() != tft.height())) tft.fillScreen(TFT_BLACK);
      tft.writeRect(CENTER, CENTER, camera.width(), camera.height(), frameBuffer);
    } else {
      tft.writeSubImageRect(0, 0, tft.width(), tft.height(), (camera.width() - tft.width()) / 2, (camera.height() - tft.height()),
                            camera.width(), camera.height(), frameBuffer);
    }
#endif
#endif  // 1
    if (use_frame_buffer) tft.updateScreenAsync();

    frame_count++;
    if ((frame_count & 0x7) == 0) {
      Serial.printf("Elapsed: %u frames: %d fps: %.2f\n", (uint32_t)em, frame_count, (float)(1000000.0 / em) * (float)frame_count);
    }
    if (Serial.available()) break;
  }
  // turn off frame buffer
  tft.useFrameBuffer(false);
}

/***********************************************************/

void test_display() {
#if defined(USE_MMOD_ATP_ADAPTER)
  tft.setRotation(1);
#else
  tft.setRotation(3);
#endif
  tft.fillScreen(TFT_RED);
  delay(500);
  tft.fillScreen(TFT_GREEN);
  delay(500);
  tft.fillScreen(TFT_BLUE);
  delay(500);
  tft.fillScreen(TFT_BLACK);
  delay(500);
}

void dump_partial_FB() {
  for (volatile uint16_t *pfb = frameBuffer; pfb < (frameBuffer + 4 * camera.width()); pfb += camera.width()) {
    Serial.printf("\n%08x: ", (uint32_t)pfb);
    for (uint16_t i = 0; i < 8; i++) Serial.printf("%02x ", pfb[i]);
    Serial.print("..");
    Serial.print("..");
    for (uint16_t i = camera.width() - 8; i < camera.width(); i++) Serial.printf("%04x ", pfb[i]);
  }
  Serial.println("\n");

  // Lets dump out some of center of image.
  Serial.println("Show Center pixels\n");
  for (volatile uint16_t *pfb = frameBuffer + camera.width() * ((camera.height() / 2) - 8); pfb < (frameBuffer + camera.width() * (camera.height() / 2 + 8)); pfb += camera.width()) {
    Serial.printf("\n%08x: ", (uint32_t)pfb);
    for (uint16_t i = 0; i < 8; i++) Serial.printf("%02x ", pfb[i]);
    Serial.print("..");
    for (uint16_t i = (camera.width() / 2) - 4; i < (camera.width() / 2) + 4; i++) Serial.printf("%02x ", pfb[i]);
    Serial.print("..");
    for (uint16_t i = camera.width() - 8; i < camera.width(); i++) Serial.printf("%02x ", pfb[i]);
  }
  Serial.println("\n...");
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



int getValue() {
  int level = 0;
  if (Serial.available()) {
    int val = Serial.parseInt();  //get the secondary code
    if (Serial.read() == '\n') {
      level = val;
    }
  }
  Serial.println(level);
  return level;
}


void changeContrast() {
  /* -3 to +3 */
  camera.setContrast(getValue());
}

void changeSaturation() {
  /* -4 to +4 */
  camera.setSaturation(getValue());
}

void changeBrightness() {
  /* -4 to +4 */
  camera.setBrightness(getValue());
}

void changeSpecialEffects() {
  /* 0 to 9 */
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
  switch (getValue()) {
    case 0:
      omni.setSpecialEffect(NOEFFECT);
      break;
    case 1:
      omni.setSpecialEffect(NEGATIVE);
      break;
    case 2:
      omni.setSpecialEffect(BW);
      break;
    case 3:
      omni.setSpecialEffect(REDDISH);
      break;
    case 4:
      omni.setSpecialEffect(GREENISH);
      break;
    case 5:
      omni.setSpecialEffect(BLUEISH);
      break;
    case 6:
      omni.setSpecialEffect(RETRO);
      break;
    case 7:
      omni.setSpecialEffect(OVEREXPOSURE);
      break;
    case 8:
      omni.setSpecialEffect(SOLARIZE);
      break;
    default:
      omni.setSpecialEffect(NOEFFECT);
      break;
  }
}

void changeLight() {
  /* 0 to 4 */
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
  omni.setWBmode(getValue());
}

void changeHue() {
  /* -6 to +5 */
  camera.setHue(getValue());
}

void changeNightMode() {
  /* 0 - disable, 1 - enable */
  omni.setNightMode(getValue());
}

void changeAutoSharpness() {
  /* 0 - disable, 1 - enable */
  omni.setAutoSharpness(getValue());
}

void changeSharpness() {
  /* 0 to 9 */
  omni.setAutoSharpness(getValue());
}

void changeLensCorrection() {
  /* 0 - disable, 1 - enable */
  omni.setLensCorrection(getValue());
}