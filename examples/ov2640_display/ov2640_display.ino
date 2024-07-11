#include <stdint.h>

#include <SPI.h>
#include <JPEGDEC.h>
#include <MemoryHexDump.h>

#include "Teensy_Camera.h"

#define USE_MMOD_ATP_ADAPTER
#define USE_SDCARD
//#define useILI9341

#define DVP_CAMERA_OV2640
//#define DVP_CAMERA_OV5640

#if defined(DVP_CAMERA_OV2640)
#include "Teensy_OV2640/OV2640.h"
OV2640 omni;
Camera camera(omni);
#define CameraID OV2640a
#define MIRROR_FLIP_CAMERA
#elif defined(DVP_CAMERA_OV5640)
#include "Teensy_OV5640/OV5640.h"
OV5640 omni;
Camera camera(omni);
#define CameraID OV5640a
#define MIRROR_FLIP_CAMERA
#else
#error "Camera not selected"
#endif

//set cam configuration - need to remember when saving jpeg
framesize_t camera_framesize = FRAMESIZE_VGA;
pixformat_t camera_format = JPEG;  //RGB565;
bool useGPIO = false;

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
#define VSYNC_PIN 21
#define DB5_USE_CSI

#elif defined(ARDUINO_TEENSY41)
// My T4.1 Camera board
#undef USE_MMOD_ATP_ADAPTER
#undef TFT_ROTATION
#define TFT_ROTATION 1
#define TFT_DC 9
#define TFT_CS 7
#define TFT_RST 8
#define VSYNC_PIN 33


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
#elif defined(ARDUINO_TEENSY41)
// CSI - lets try to setup for PSRAM (EXTMEM)
// only half buffer will fit in each of the two main memory regions
// split into two parts, part dmamem and part fast mememory to fit 640x480x2
EXTMEM uint16_t frameBuffer[800 * 600] __attribute__((aligned(32)));
EXTMEM uint16_t frameBuffer2[800 * 600] __attribute__((aligned(32)));
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
const uint32_t sizeof_framebuffer = sizeof(frameBuffer);
const uint32_t sizeof_framebuffer2 = sizeof(frameBuffer2);
#endif

// Setup display modes frame / video
bool g_continuous_flex_mode = false;
bool g_flex_dual_buffer_per_frame = false;
void *volatile g_new_flexio_data = nullptr;
size_t g_new_flexio_data_length = 0;
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
    //  If: not using default pins you may need to setup which pins are used using:
    //    camera.setPins(...) See the header file Teensy_Camera.h for details
    //          for the different pins
    
    uint8_t reset_pin = 31;
    uint8_t powdwn_pin = 30;
#if defined(ARDUINO_TEENSY_DEVBRD5)
    reset_pin = 57;
    powdwn_pin = 58;
#endif

#ifdef USE_MMOD_ATP_ADAPTER
    pinMode(0, OUTPUT);
    pinMode(3, OUTPUT);
    digitalWriteFast(3, HIGH);
    delayMicroseconds(25);
    digitalWriteFast(3, LOW);
    delayMicroseconds(25);
    digitalWriteFast(3, HIGH);
    delayMicroseconds(25);
    digitalWriteFast(3, LOW);
    delayMicroseconds(25);
    digitalWriteFast(3, HIGH);
    delayMicroseconds(25);
    digitalWriteFast(3, LOW);
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
#if defined(DB5_USE_CSI)
    // try using the CSI pins on devboard 5
    reset_pin = 57;
    powdwn_pin = 58;
    camera.setPins(65,  64, 17, 16, 57, 27, 26, 67, 66, 21, 20, 23, 22, 58);
 #endif
    status = camera.begin(camera_framesize, camera_format, 5, CameraID, useGPIO);

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

    //camera.setBrightness(0);          // -2 to +2
    //camera.setContrast(0);            // -2 to +2
    //camera.setSaturation(0);          // -2 to +2
    //omni.setSpecialEffect(RETRO);  // NOEFFECT, NEGATIVE, BW, REDDISH, GREEISH, BLUEISH, RETRO
    //omni.setWBmode(0);                  // AWB ON, 1 - Sunny, 2 - Cloudy, 3 - Office, 4 - Home

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

    // Lets setup camera interrupt priorities:
    //camera.setVSyncISRPriority(102); // higher priority than default
    camera.setDMACompleteISRPriority(192);  // lower than default


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

bool camera_flexio_callback(void *pfb) {
    g_new_flexio_data = pfb;
    g_new_flexio_data_length = camera.readImageSizeBytes();
    Serial.printf("Flexio callback %p %u\n", pfb, g_new_flexio_data_length);
    return true;
}


// Quick and Dirty
#define UPDATE_ON_CAMERA_FRAMES

inline uint16_t HTONS(uint16_t x) {
#if defined(DVP_CAMERA_OV2640) || defined(DVP_CAMERA_OV5640)
    return x;
#else  //byte reverse
    return ((x >> 8) & 0x00FF) | ((x << 8) & 0xFF00);
#endif
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
#if defined(USB_DUAL_SERIAL) || defined(USB_TRIPLE_SERIAL)
    while (SerialUSB1.available()) {
        ch = SerialUSB1.read();
        //Serial.println(ch, HEX);
        switch (ch) {
            case 0x10:
                {
#if defined(USB_DUAL_SERIAL) || defined(USB_TRIPLE_SERIAL)
                    SerialUSB1.println(F("ACK CMD CAM start jpg single shoot. END"));
                    send_jpeg();
                    SerialUSB1.println(F("READY. END"));
#else
                    Serial.println("*** Only works in USB Dual or Triple Serial Mode ***");
#endif
                }
                break;
            case 0x30:
                {
                    Serial.print(F("ACK CMD CAM start single shoot ... "));
                    send_image(&SerialUSB1);
                    Serial.println(F("READY. END"));
                }
                break;
            case 0x40:
                omni.setWBmode(0);
                break;
            case 0x41:
                omni.setWBmode(1);
                break;
            case 0x42:
                omni.setWBmode(2);
                break;
            case 0x43:
                omni.setWBmode(3);
                break;
            case 0x44:
                omni.setWBmode(4);
                break;
            case 0x87:  //Normal
                omni.setSpecialEffect(NOEFFECT);
                break;
            case 0x85:
                omni.setSpecialEffect(NEGATIVE);
                break;
            case 0x84:
                omni.setSpecialEffect(BW);
                break;
            case 0x83:
                omni.setSpecialEffect(REDDISH);
                break;
            case 0x82:
                omni.setSpecialEffect(GREENISH);
                break;
            case 0x81:
                omni.setSpecialEffect(BLUEISH);
                break;
            case 0x80:
                omni.setSpecialEffect(RETRO);
                break;
            case 0x50:
                camera.setSaturation(2);
                break;
            case 0x51:
                camera.setSaturation(1);
                break;
            case 0x52:
                camera.setSaturation(0);
                break;
            case 0x53:
                camera.setSaturation(-1);
                break;
            case 0x54:
                camera.setSaturation(-2);
                break;
            case 0x60:
                camera.setBrightness(2);
                break;
            case 0x61:
                camera.setBrightness(1);
                break;
            case 0x62:
                camera.setBrightness(0);
                break;
            case 0x63:
                camera.setBrightness(-1);
                break;
            case 0x64:
                camera.setBrightness(-2);
                break;
            case 0x70:
                camera.setContrast(2);
                break;
            case 0x71:
                camera.setContrast(1);
                break;
            case 0x72:
                camera.setContrast(0);
                break;
            case 0x73:
                camera.setContrast(-1);
                break;
            case 0x74:
                camera.setContrast(-2);
                break;
            default:
                break;
        }
    }
#endif
    if (Serial.available()) {
        uint8_t command = Serial.read();
        ch = Serial.read();
#if defined(USE_SDCARD)
        if ('2' == command) storage_index = CommandLineReadNextNumber(ch, 0);
#endif
        switch (command) {
            case 'p':
                {
#if defined(USB_DUAL_SERIAL) || defined(USB_TRIPLE_SERIAL)
                    send_raw();
                    Serial.println("Image Sent!");
                    ch = ' ';
#else
                    Serial.println("*** Only works in USB Dual or Triple Serial Mode ***");
#endif
                    break;
                }
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
                processJPGFile(nullptr, 0, true);  // nullptr says to do the actual read from the camera
                break;
            case 't':
                test_display();
                break;
            case 'r':
                camera.showRegisters();
                break;
            case 'R':
                change_camera_resolution(ch);
                break;
            case 'Q':
                show_change_jpeq_quality(ch);
                break;
            case 'w':
                changeCameraWindow();
                break;
            case 'W':
                panCameraWindow();
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
                        if (camera.readContinuous(&camera_flexio_callback, frameBuffer, sizeof_framebuffer, frameBuffer2, sizeof_framebuffer2)) {
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
            case 'C':
                {
                    tft.fillScreen(TFT_BLACK);
                    break;
                }
            case 0x30:
                {
#if defined(USB_DUAL_SERIAL) || defined(USB_TRIPLE_SERIAL)
                    SerialUSB1.println(F("ACK CMD CAM start single shoot. END"));
                    send_image(&SerialUSB1);
                    SerialUSB1.println(F("READY. END"));
#else
                    Serial.println("*** Only works in USB Dual or Triple Serial Mode ***");
#endif
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


    if (g_new_flexio_data && g_continuous_flex_mode && !tft.asyncUpdateActive()) {
        pixformat_t fmt = camera.getPixformat();
        Serial.printf("FMT: %u\n", fmt);
        if (fmt == pixformat_t::JPEG) {
            static uint8_t debug_count = 5;
            uint32_t image_size = g_new_flexio_data_length;
            memset(frameBuffer2, 0, sizeof_framebuffer2);
            memcpy(frameBuffer2, frameBuffer, min(sizeof_framebuffer2, image_size));

            if (debug_count) {
                debug_count--;
                MemoryHexDump(Serial, frameBuffer2, 256, true, "Start\n");
                MemoryHexDump(Serial, (void *)((uint32_t)frameBuffer2 + image_size - 128), 256, true, "end\n");
            }

            processJPGFile((uint8_t *)frameBuffer2, sizeof_framebuffer2, true);
            tft.updateScreenAsync();

            g_new_flexio_data = nullptr;

        } else
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

#if defined(USB_DUAL_SERIAL) || defined(USB_TRIPLE_SERIAL)
void send_image(Stream *imgSerial) {
    memset((uint8_t *)frameBuffer, 0, sizeof(frameBuffer));
    camera.setHmirror(1);
    camera.readFrame(frameBuffer, sizeof_framebuffer, frameBuffer2, sizeof_framebuffer2);

    imgSerial->write(0xFF);
    imgSerial->write(0xAA);

    // BUGBUG:: maybe combine with the save to SD card code
    unsigned char bmpFileHeader[14] = { 'B', 'M', 0, 0, 0, 0, 0, 0, 0, 0, 54, 0, 0, 0 };
    unsigned char bmpInfoHeader[40] = { 40, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 24, 0 };

    int rowSize = 4 * ((3 * FRAME_WIDTH + 3) / 4);  // how many bytes in the row (used to create padding)
    int fileSize = 54 + FRAME_HEIGHT * rowSize;     // headers (54 bytes) + pixel data

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


    imgSerial->write(bmpFileHeader, sizeof(bmpFileHeader));  // write file header
    imgSerial->write(bmpInfoHeader, sizeof(bmpInfoHeader));  // " info header

    unsigned char bmpPad[rowSize - 3 * FRAME_WIDTH];
    for (int i = 0; i < (int)(sizeof(bmpPad)); i++) {  // fill with 0s
        bmpPad[i] = 0;
    }

    uint32_t idx = 0;

    uint16_t *pfb = frameBuffer;
    uint8_t img[3];
    uint32_t count_y_first_buffer = sizeof_framebuffer / (FRAME_WIDTH * 2);
    for (int y = FRAME_HEIGHT - 1; y >= 0; y--) {  // iterate image array
        if (y < (int)count_y_first_buffer) pfb = &frameBuffer[y * FRAME_WIDTH];
        else pfb = &frameBuffer2[(y - count_y_first_buffer) * FRAME_WIDTH];
        for (int x = 0; x < FRAME_WIDTH; x++) {
            //r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3
            uint16_t pixel = HTONS(*pfb++);
            img[2] = (pixel >> 8) & 0xf8;  // r
            img[1] = (pixel >> 3) & 0xfc;  // g
            img[0] = (pixel << 3);         // b
            imgSerial->write(img, 3);
            delayMicroseconds(8);
        }
        imgSerial->write(bmpPad, (4 - (FRAME_WIDTH * 3) % 4) % 4);  // and padding as needed
    }

    imgSerial->write(0xBB);
    imgSerial->write(0xCC);

    imgSerial->println(F("ACK CMD CAM Capture Done. END"));
    camera.setHmirror(0);

    delay(50);
}

void send_raw() {
    Serial.println("Send Raw RGB");
    memset((uint8_t *)frameBuffer, 0, sizeof_framebuffer);
    memset((uint8_t *)frameBuffer2, 0, sizeof_framebuffer2);
    camera.readFrame(frameBuffer, sizeof_framebuffer, frameBuffer2, sizeof_framebuffer2);
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

bool send_jpeg() {

    uint16_t w = FRAME_WIDTH;
    uint16_t h = FRAME_HEIGHT;
    uint8_t eoi = 0;
    uint32_t eop = 0;

    uint32_t status = 0;
    status = readJPG(eoi, eop, false);
    if (status == 0) return false;

    uint16_t *pfb = frameBuffer;
    Serial.println(F("ACK BEGIN JPEG XFER"));

    uint32_t numPixels = w * h;
    uint32_t jpegSize = (w * h) / 5;

    //byte swap
    //for (int i = 0; i < numPixels; i++) frameBuffer[i] = (frameBuffer[i] >> 8) | (((frameBuffer[i] & 0xff) << 8));
    // now see if all fit into one buffer or part in second...
    uint32_t numPixels1 = min((int)(sizeof_framebuffer / 2), numPixels);
    uint32_t numPixels2 = min((int)(sizeof_framebuffer2 / 2), numPixels - numPixels1);
    if (0) Serial.printf("\tBuffers:%p(%u) %p(%u)\n", frameBuffer, sizeof_framebuffer, frameBuffer2, sizeof_framebuffer2);
    if (0) Serial.printf("\tnumpixels %u %u %u\n", numPixels, numPixels1, numPixels2);
    //for (int i = 0; i < numPixels1; i++) frameBuffer[i] = HTONS(frameBuffer[i]);
    //for (int i = 0; i < numPixels2; i++) frameBuffer2[i] = HTONS(frameBuffer2[i]);


    if ((numPixels2 == 0) || ((sizeof_framebuffer - jpegSize) < 0)) {
        for (uint32_t i = 0; i < jpegSize; i++) {
            SerialUSB1.write(frameBuffer[i] & 0xFF);
            SerialUSB1.write((frameBuffer[i] >> 8) & 0xFF);
            delayMicroseconds(2);
        }
    } else {
        for (uint32_t i = 0; i < numPixels1; i++) {
            //file.write(pfb[i]);
            SerialUSB1.write(frameBuffer[i] & 0xFF);
            SerialUSB1.write((frameBuffer[i] >> 8) & 0xFF);
            delayMicroseconds(2);
        }
        for (uint32_t i = 0; i < eop; i++) {
            SerialUSB1.write(frameBuffer2[i] & 0xFF);
            SerialUSB1.write((frameBuffer2[i] >> 8) & 0xFF);
            delayMicroseconds(2);
        }
    }
    SerialUSB1.flush();
    Serial.println(F("ACK IMG END"));

    return true;
}
#endif


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

    //uint16_t *pfb = frameBuffer;
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
    if (camera.usingGPIO()) {
        Serial.println("Send the 'f' character to read a frame using GPIO");
        Serial.println("Send the 'F' to start/stop continuous using GPIO");
        Serial.println("Send the 'n' character to read a frame using GPIO without DMA?");

    } else {
        Serial.println("Send the 'f' character to read a frame using FlexIO (changes hardware setup!)");
        Serial.println("Send the 'F' to start/stop continuous using FlexIO (changes hardware setup!)");
        Serial.println("Send the 'n' character to read a frame using FlexIO without DMA (changes hardware setup!)");
    }
    Serial.println("Send the 'm' character to read and display multiple frames");
    Serial.println("Send the 'M' character to read and display multiple frames use Frame buffer");
    Serial.println("Send the 'V' character DMA to TFT async continueous  ...");
    Serial.println("Send the 'p' character to snapshot to PC on USB1");
    Serial.println("Send the 'b' character to save snapshot (BMP) to SD Card");
    Serial.println("Send the 'j' character to save snapshot (JPEG) to SD Card");
    Serial.println("Send the 'x' send snapshot (JPEG) to display");
    Serial.println("Send the '0' character to blank the display");
    Serial.println("Send the 'z' character to send current screen BMP to SD");
    Serial.println("Send the 't' character to send Check the display");
    Serial.println("Send the 'c' character to debug clock - print vsync timing");
    Serial.println("Send the 'd' character to toggle camera debug on and off");
    Serial.println("Send the 'r' character to show the current camera registers");
    Serial.println("Send the 'Q' character to print current JPEG Quality value");
    Serial.println("Send the 'Q<value>' to change the quality 10-63 lower is higher");
    Serial.println("Send the 'R[QVSU]' To set image size QVGA, VGA, SVGA UXGA");
    Serial.println("Send the 'w <row> <col>' to set the start window x, y");
    Serial.println("Send the 'W' to pan through range of windows");
#if defined(ARDUINO_TEENSY_DEVBRD4) || defined(ARDUINO_TEENSY_DEVBRD5)
    Serial.println("Send the 's' to change if using SDRAM or other memory");
#endif
#if defined(USE_SDCARD)
    Serial.println("MTP FUNCTIONS:");
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
    camera.useDMA(use_dma);
    uint32_t cbRead = camera.readFrame(frameBuffer, sizeof_framebuffer, frameBuffer2, sizeof_framebuffer2);
    //  digitalWriteFast(24, LOW);

    if (show_debug_info) {
        Serial.printf("Finished reading frame(%u)\n", cbRead);
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
#ifdef ARDUINO_TEENSY41
    // See what which camera buffer was used.
    uint16_t *pbuffer = (uint16_t *)camera.readFrameReturnBuffer();

    if (pbuffer) {
        for (int i = 0; i < numPixels; i++) pbuffer[i] = HTONS(pbuffer[i]);
        tft.writeRect(CENTER, CENTER, camera.width(), camera.height(),
                      pbuffer);
    }

#else

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
#ifdef ARDUINO_TEENSY41
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

uint16_t get_next_value(uint8_t &b) {
    uint16_t val = 0;

    for (;;) {
        if (b == (uint8_t)-1) return 0;
        if (b != ' ') break;
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

    camera.setPixformat(camera_format);
}

void panCameraWindow() {
    while (Serial.read() != -1) {}
    for (;;) {
        for (uint16_t start_y = 0; start_y < (camera.frameHeight() - camera.height()); start_y += 100) {
            for (uint16_t start_x = 0; start_x < (camera.frameWidth() - camera.width()); start_x += 100) {
                camera.setZoomWindow(start_x, start_y);
                read_display_one_frame(true, false);
                delay(250);
                if (Serial.available()) return;
            }
        }
    }
}

void print_vsync_timings() {
    uint8_t frame_count = 0;
    while (Serial.read() != -1) {}  // remove any extra characters

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
                      1000000.0 / (float)(uint32_t)(new_falling_edge_time - falling_edge_time));
        falling_edge_time = new_falling_edge_time;
    }
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

void show_change_jpeq_quality(int ch) {
    uint8_t quality = 0;
    while (ch == ' ') ch = Serial.read();
    while ((ch >= '0') && (ch <= '9')) {
        quality = quality * 10 + (ch - '0');
        ch = Serial.read();
    }

    if (quality > 0) {
        Serial.printf("Setting quality to: %u\n", quality);
        omni.setQuality(quality);
    } else {
        Serial.printf("Current JPEQ quality is: %u\n", omni.getQuality());
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
        if (debug_on) {
            Serial.printf("Error: No bytes returned from camera\n");
            MemoryHexDump(Serial, frameBuffer, 256, true, "FB1\n", -1, 0);
            MemoryHexDump(Serial, frameBuffer2, 256, true, "FB2\n", -1, 0);

            // Curious to see if it maybe did retrieve the data...
            uint8_t *pfb = (uint8_t *)frameBuffer;
            if ((pfb[0] == 0xff) && (pfb[1] == 0xd8) && (pfb[2] == 0xff)) {
                // looks like first buffer has start of a JPEG...
                Serial.println("Data in buffer appears to be start with JPEG data");
                for (int i = 3; i < (int)(sizeof_framebuffer - 1); i++) {
                    if ((pfb[i] == 0xff) && (pfb[i + 1] == 0xd9)) {
                        Serial.println("Data in buffer appears to be JPEG data");
                        bytes_read = i + 2;
                        break;
                    }
                }
            } else {
                pfb = (uint8_t *)frameBuffer2;
                if ((pfb[0] == 0xff) && (pfb[1] == 0xd8) && (pfb[2] == 0xff)) {
                    Serial.println("Data in buffer2 appears to be start with JPEG data");
                    // looks like first buffer has start of a JPEG...
                    for (int i = 3; i < (int)(sizeof_framebuffer2 - 1); i++) {
                        if ((pfb[i] == 0xff) && (pfb[i + 1] == 0xd9)) {
                            bytes_read = i + 2;
                            // hack it wants it in the first fuffer...
                            Serial.println("Data in buffer2 appears to be JPEG data");
                            memcpy(frameBuffer, frameBuffer2, bytes_read);
                            break;
                        }
                    }
                }
            }
            // If we found start marker but not end marker, lets look at what is at the end of both frames.
            if ((bytes_read == 0) && (pfb[0] == 0xff) && (pfb[1] == 0xd8) && (pfb[2] == 0xff)) {
                pfb = (uint8_t *)frameBuffer;
                int i;
                for (i = sizeof_framebuffer - 1; i > 128; i--) {
                    if (pfb[i] != 0) break;
                }
                if (i > 128) {
                    i &= 0xfffffff0;
                    MemoryHexDump(Serial, &pfb[i], 128, true, "End FB1\n", -1, (uint32_t)i);
                }

                pfb = (uint8_t *)frameBuffer2;
                for (i = sizeof_framebuffer2 - 1; i > 128; i--) {
                    if (pfb[i] != 0) break;
                }
                if (i > 128) {
                    i &= 0xfffffff0;
                    MemoryHexDump(Serial, &pfb[i], 128, true, "End FB2\n", -1, (uint32_t)i);
                }
            }
        }

        if (bytes_read == 0) return false;
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
