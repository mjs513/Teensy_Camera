#include <stdint.h>

#define use_spi

#include <MemoryHexDump.h>
#include <Adafruit_GFX.h>  // Core graphics library
#if defined(use_spi)
#include "SPI.h"
#include <RA8876_t3.h>
#else
#include <RA8876_t41_p.h>
#endif

#include <SPI.h>
#include <JPEGDEC.h>
#include <MemoryHexDump.h>

#include "Teensy_Camera.h"

#if defined(USE_MMOD_ATP_ADAPTER
#define useRA9976
//#define USE_SDCARD

#ifdef DVP_CAMERA_OV3640
#include "Teensy_OV2640/OV2640.h"
OV2640 omni;
Camera camera(omni);
#define CameraID OV2640a
#define MIRROR_FLIP_CAMERA
#else
//define DVP_CAMERA_OV5640
#include "Teensy_OV5640/OV5640.h"
OV5640 omni;
Camera camera(omni);
#define CameraID OV5640a
#define MIRROR_FLIP_CAMERA
#endif

#if defined(USE_SDCARD)
#include <SD.h>
File file;
#endif

//set cam configuration - need to remember when saving jpeg
framesize_t camera_framesize = FRAMESIZE_VGA;
pixformat_t camera_format = RGB565;
bool useGPIO = false;

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

#elif defined(ARDUINO_TEENSY_DEVBRD5)
#undef USE_MMOD_ATP_ADAPTER
#if defined(use_spi)
#define TFT_CS 63
#define TFT_RST 62
//#define TFT_BL 29
#define DB5_USE_CSI

#else
#define TFT_CS 63  // AD_B0_02
#define TFT_DC 61  // AD_B0_03
#define TFT_RST 62
#define VSYNC_PIN 21
#endif

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

#if defined(use_spi)
RA8876_t3 tft = RA8876_t3(TFT_CS, TFT_RST);
#else
uint8_t dc = 13;
uint8_t cs = 11;
uint8_t rst = 12;
RA8876_t41_p tft = RA8876_t41_p(dc,cs,rst); //(dc, cs, rst)
#endif

#if defined(ARDUINO_TEENSY_DEVBRD4) || defined(ARDUINO_TEENSY_DEVBRD5)
extern "C" bool sdram_begin(uint8_t external_sdram_size, uint8_t clock, uint8_t useDQS);
#endif

#define TFT_BLACK BLACK
#define TFT_YELLOW YELLOW
#define TFT_RED RED
#define TFT_GREEN GREEN
#define TFT_BLUE BLUE
//#define CENTER RA8876_t3::CENTER

// Setup framebuffers
DMAMEM uint16_t FRAME_WIDTH, FRAME_HEIGHT;
#if defined(ARDUINO_TEENSY_DEVBRD4) || defined(ARDUINO_TEENSY_DEVBRD5)
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
#else
#if defined(USE_SDCARD)
DMAMEM uint16_t frameBuffer[700 * 240] __attribute__((aligned(32)));
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

    tft.begin(34000000);
    test_display();

    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_YELLOW);
    tft.setTextSize(0);
    tft.println("Waiting for Arduino Serial Monitor...");

    while (!Serial)
        ;

#if defined(USE_SDCARD)
    Serial.println("Using SDCARD - Initializing");
#if MMOD_ML == 1
    if (!SD.begin(10)) {
#else
    if (!SD.begin(BUILTIN_SDCARD)) {
#endif
        Serial.println("initialization failed!");
        //while (1){
        //    LEDON; delay(100);
        //    LEDOFF; delay(100);
        //  }
    }
    Serial.println("initialization done.");
    delay(100);
#endif

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

#elif  defined(DB5_USE_CSI)
    // try using the CSI pins on devboard 5
    camera.setPins(65,  64, 17, 16, 57, 27, 26, 67, 66, 21, 20, 23, 22, 58);

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

#if defined(ARDUINO_TEENSY_DEVBRD4) || defined(ARDUINO_TEENSY_DEVBRD5)
    //sdram_begin(32, 216, 1);
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

    /**********************************************************/
    tft.graphicMode(true);
    tft.setRotation(0);

    showCommandList();
}

// Pass 8-bit (each) R,G,B, get back 16-bit packed color
uint16_t color565(uint8_t r, uint8_t g, uint8_t b) {
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

inline uint16_t HTONS(uint16_t x) {
#if defined(DVP_CAMERA_OV5640)
    return x;
#else  //byte reverse
    return ((x >> 8) & 0x00FF) | ((x << 8) & 0xFF00);
#endif
}

bool hm0360_flexio_callback(void *pfb) {
    //Serial.println("Flexio callback");
    g_new_flexio_data = pfb;
    return true;
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
#if (defined(USE_SDCARD) && (defined(DVP_CAMERA_OV2640) || defined(DVP_CAMERA_OV5640))
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
                }
            case 'd':
                camera.debug(!camera.debug());
                if (camera.debug()) Serial.println("Camera Debug turned on");
                else Serial.println("Camera debug turned off");
                break;
            case 'f':
                {
                    //tft.useFrameBuffer(false);
                    tft.fillScreen(TFT_BLACK);
                    tft.displayOn(false);
                    read_display_one_frame(true, true);
                    tft.displayOn(true);
                    ch = ' ';
                    g_continuous_flex_mode = false;
                    break;
                }
            case 'n':
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
            case 'm':
                read_display_multiple_frames(false);
                break;

            case 'M':
                read_display_multiple_frames(true);
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
            case 't':
                test_display();
                break;
            case 'C':
                {
                    tft.fillScreen(TFT_BLACK);
                    break;
                }
            case 'x':
                processJPGFile(true);
                break;
            case 'r':
                camera.showRegisters();
                break;
            case 'R':
                change_camera_resolution(ch);
                break;
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
            //tft.updateScreenAsync();
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
    camera.useDMA(true);
    camera.readFrame(frameBuffer, sizeof_framebuffer, frameBuffer2, sizeof_framebuffer2);
    //  digitalWriteFast(24, LOW);

    int numPixels = camera.width() * camera.height();

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
            //tft.writeSubImageRect(0, 0, tft.width(), tft.height(), (camera.width() - tft.width()) / 2, (camera.height() - tft.height()),
            //                      camera.width(), camera.height(), frameBuffer);
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
}


//=============================================================================
void read_display_multiple_frames(bool use_frame_buffer) {
    if (use_frame_buffer) {
        Serial.println("\n*** Read and display multiple frames (using async screen updates), press any key to stop ***");
        tft.displayOn(false);
        tft.useCanvas(true);
    } else {
        Serial.println("\n*** Read and display multiple frames, press any key to stop ***");
    }

    while (Serial.read() != -1) {}

    elapsedMicros em = 0;
    int frame_count = 0;

    for (;;) {

        //if (use_frame_buffer) tft.waitUpdateAsyncComplete();
        read_display_one_frame(true, false);

        if (use_frame_buffer) tft.displayOn(true);

        frame_count++;
        if ((frame_count & 0x7) == 0) {
            Serial.printf("Elapsed: %u frames: %d fps: %.2f\n", (uint32_t)em, frame_count, (float)(1000000.0 / em) * (float)frame_count);
        }
        if (Serial.available()) break;
    }
    // turn off frame buffer
    tft.updateScreen();
    tft.useCanvas(false);
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
        camera.setPixformat(camera_format);
        camera.setFramesize(fs);
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

    uint8_t status = 0;
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
        if (!SD.exists(name)) {
            Serial.println(name);
            file = SD.open(name, FILE_WRITE);
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
        if (!SD.exists(name_jpg)) {
            Serial.println(name_jpg);
            file = SD.open(name_jpg, FILE_WRITE);
            break;
        }
    }

    uint8_t eoi = 0;
    uint32_t eop = 0;

    uint8_t status = 0;
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
    Serial.println("Send the 'p' character to snapshot to PC on USB1");
    Serial.println("Send the 'b' character to save snapshot (BMP) to SD Card");
    Serial.println("Send the 'j' character to save snapshot (JPEG) to SD Card");
    Serial.println("Send the 'x' send snapshot (JPEG) to display");
    Serial.println("Send the 'C' character to blank the display");
    Serial.println("Send the 'z' character to send current screen BMP to SD");
    Serial.println("Send the 't' character to send Check the display");
    Serial.println("Send the 'd' character to toggle camera debug on and off");
    Serial.println("Send the 'r' character to show the current camera registers");
    Serial.println("Send the 'Q' character to print current JPEG Quality value");
    Serial.println("Send the 'Q<value>' to change the quality 10-63 lower is higher");
    Serial.println("Send the 'R[QVSU]' To set image size QVGA, VGA, SVGA UXGA");
#ifdef ARDUINO_TEENSY_DEVBRD4
    Serial.println("Send the 's' to change if using SDRAM or other memory");
#endif
}