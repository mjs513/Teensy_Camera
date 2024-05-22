#include <stdint.h>

#include <JPEGDEC.h>
#include <MemoryHexDump.h>
#include <SPI.h>

#include "Teensy_Camera.h"

#define USE_MMOD_ATP_ADAPTER
#define USE_T4_PXP

// just a test
// #define USE_HX8357D

// #define DVP_CAMERA_OV2640
#define DVP_CAMERA_OV5640

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
// #define MIRROR_FLIP_CAMERA
#else
#error "Camera not selected"
#endif

// set cam configuration - need to remember when saving jpeg
framesize_t camera_framesize = FRAMESIZE_480X320;
pixformat_t camera_format = RGB565;
bool useGPIO = false;

// Set up Display
#if defined(ARDUINO_TEENSY41)

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
#define TFT_DC 4  // 0   // "TX1" on left side of Sparkfun ML Carrier
#define TFT_CS 5  // 4   // "CS" on left side of Sparkfun ML Carrier
#define TFT_RST 2 // 1  // "RX1" on left side of Sparkfun ML Carrier
#else
#define VSYNC_PIN 33
#define TFT_DC 0  // 20   // "TX1" on left side of Sparkfun ML Carrier
#define TFT_CS 4  // 5, 4   // "CS" on left side of Sparkfun ML Carrier
#define TFT_RST 1 // 2, 1  // "RX1" on left side of Sparkfun ML Carrier
#endif

#ifdef ARDUINO_TEENSY41
#define TFT_ROTATION 1
#else
#define TFT_ROTATION 3
#endif

#ifdef USE_HX8357D
#include <HX8357_t3n.h>
HX8357_t3n tft = HX8357_t3n(TFT_CS, TFT_DC, TFT_RST);
#define TFT_BLACK HX8357_BLACK
#define TFT_YELLOW HX8357_YELLOW
#define TFT_RED HX8357_RED
#define TFT_GREEN HX8357_GREEN
#define TFT_BLUE HX8357_BLUE
#define CENTER ILI9488_t3::CENTER
#define RAFB uint16_t

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
#if defined(ARDUINO_TEENSY41)
// CSI - lets try to setup for PSRAM (EXTMEM)
// only half buffer will fit in each of the two main memory regions
// split into two parts, part dmamem and part fast mememory to fit 640x480x2
EXTMEM uint16_t frameBuffer[480 * 320] __attribute__((aligned(32)));
EXTMEM uint16_t frameBuffer2[480 * 320] __attribute__((aligned(32)));
const uint32_t sizeof_framebuffer = sizeof(frameBuffer);
const uint32_t sizeof_framebuffer2 = sizeof(frameBuffer2);
#else

DMAMEM uint16_t frameBuffer[480 * 320] __attribute__((aligned(32)));
uint16_t frameBuffer2[480 * 320] __attribute__((aligned(32)));

#ifdef USE_T4_PXP
#include <T4_PXP.h>
#endif

// #define SCREEN_ROTATION 1
const uint32_t sizeof_framebuffer = sizeof(frameBuffer);
const uint32_t sizeof_framebuffer2 = sizeof(frameBuffer2);
#endif

volatile uint32_t frame_count_camera = 0;
volatile uint32_t frame_count_tft = 0;

void setup() {
    Serial.begin(921600);
    while (!Serial && millis() < 5000) {
    }

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

    tft.begin(20000000);
#ifdef USE_T4_PXP
    Serial.println("Starting T4 PXP library");
    PXP_init();
    tft.setRotation(0);
#else
    tft.setRotation(TFT_ROTATION);
#endif
    test_display();

    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_YELLOW);
    tft.setTextSize(2);
    tft.println("Waiting for Arduino Serial Monitor...");

    while (!Serial && (millis() < 5000)) {
    }
    Serial.println("TeensyMM Camera Test");

    delay(500);

    tft.fillScreen(TFT_BLACK);

    /***************************************************************/
    //  If: not using default pins you may need to setup which pins are used using:
    //    camera.setPins(...) See the header file Teensy_Camera.h for details
    //          for the different pins
    uint8_t reset_pin = 31;
    uint8_t powdwn_pin = 30;
#ifdef USE_MMOD_ATP_ADAPTER
    pinMode(0, OUTPUT);
    pinMode(3, OUTPUT);
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
    status = camera.begin(camera_framesize, camera_format, 5, CameraID, useGPIO);

    Serial.printf("Begin status: %d\n", status);
    if (!status) {
        Serial.println("Camera failed to start - try floating both reset and powndn!!!");
        // Serial.printf("\tPin 30:%u 31:%u\n", digitalReadFast(30), digitalReadFast(31));
        Serial.printf("\tPin rst(%u):%u PowDn(%u):%u\n", reset_pin, digitalRead(reset_pin), powdwn_pin, digitalRead(powdwn_pin));
        pinMode(reset_pin, INPUT);
        pinMode(powdwn_pin, INPUT);

        status = camera.begin(camera_framesize, camera_format, 15, CameraID, useGPIO);
    }
    if (!status) {
// lets try to tell the camera code to disregard the enable and shutdown pin
#define NOCHANGE 0xfe
        camera.setPins(NOCHANGE, NOCHANGE, NOCHANGE, NOCHANGE, 0xff,
                       NOCHANGE, NOCHANGE, NOCHANGE, NOCHANGE, NOCHANGE, NOCHANGE, NOCHANGE, NOCHANGE,
                       0xff);
        Serial.println("Camera failed to start - try reset!!!");
        // Serial.printf("\tPin 30:%u 31:%u\n", digitalReadFast(30), digitalReadFast(31));
        Serial.printf("\tPin rst(%u):%u PowDn(%u):%u\n", reset_pin, digitalRead(reset_pin), powdwn_pin, digitalRead(powdwn_pin));
        pinMode(reset_pin, OUTPUT);
        digitalWriteFast(reset_pin, LOW);
        delay(500);
        digitalWriteFast(reset_pin, HIGH);
        delay(500);
        status = camera.begin(camera_framesize, camera_format, 15, CameraID, useGPIO);
    }
    if (!status) {
        // Lets try setting the power down pin
        Serial.println("Try Powdn line HIGH");
        pinMode(powdwn_pin, OUTPUT);
        digitalWrite(powdwn_pin, HIGH);

        digitalWriteFast(reset_pin, LOW);
        delay(100);
        digitalWriteFast(reset_pin, HIGH);
        delay(100);
        status = camera.begin(camera_framesize, camera_format, 15, CameraID, useGPIO);
    }
    if (!status) {
        Serial.println("Try powdn line Low");
        digitalWrite(powdwn_pin, LOW);

        digitalWriteFast(reset_pin, LOW);
        delay(100);
        digitalWriteFast(reset_pin, HIGH);
        delay(100);
        status = camera.begin(camera_framesize, camera_format, 15, CameraID, useGPIO);
    }
    if (!status) {
        Serial.println("Camera failed to start again program halted");
        while (1) {
        }
    }

    //
    //  if (!camera.setZoomWindow(80, 80, 480, 320)) Serial.println("$$$camera.setZoomWindow failed");

#ifdef MIRROR_FLIP_CAMERA
    camera.setHmirror(true);
    camera.setVflip(true);
#endif

    // camera.setBrightness(0);          // -2 to +2
    // camera.setContrast(0);            // -2 to +2
    // camera.setSaturation(0);          // -2 to +2
    // omni.setSpecialEffect(RETRO);  // NOEFFECT, NEGATIVE, BW, REDDISH, GREEISH, BLUEISH, RETRO
    // omni.setWBmode(0);                  // AWB ON, 1 - Sunny, 2 - Cloudy, 3 - Office, 4 - Home

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

    // Lets try reading in one frame
    for (uint8_t i = 0; i < 5; i++) {
        Serial.printf("Couple of Quick frames: %u\n", i);
        camera.readFrame(frameBuffer, sizeof(frameBuffer));
#ifdef USE_T4_PXP
        uint16_t outputWidth, outputHeight;
        PXP_ps_output(tft.width(), tft.height(),       /* Display width and height */
                      camera.width(), camera.height(), /* Image width and height */
                      frameBuffer, PXP_RGB565, 2, 0,   /* Input buffer configuration */
                      frameBuffer2, PXP_RGB565, 2, 0,  /* Output buffer configuration */
                      TFT_ROTATION, 0, 0.0,            /* Rotation, flip, scaling */
                      &outputWidth, &outputHeight);    /* Frame Out size for drawing */

        tft.writeRect(0, 0, outputWidth, outputHeight, frameBuffer2);
#else
        tft.writeRect(0, 0, camera.width(), camera.height(), frameBuffer);
#endif

        delay(500);
    }

    Serial.println("Now try to async ");
    delay(1000);
#ifdef USE_T4_PXP
    uint16_t outputWidth, outputHeight;
    camera.readFrame(frameBuffer, sizeof(frameBuffer));
    PXP_ps_output(tft.width(), tft.height(),       /* Display width and height */
                  camera.width(), camera.height(), /* Image width and height */
                  frameBuffer, PXP_RGB565, 2, 0,   /* Input buffer configuration */
                  frameBuffer2, PXP_RGB565, 2, 0,  /* Output buffer configuration */
                  TFT_ROTATION, 0, 0.0,            /* Rotation, flip, scaling */
                  &outputWidth, &outputHeight);    /* Frame Out size for drawing */
#else
    camera.readFrame(frameBuffer2, sizeof(frameBuffer2));
#endif
    tft.setFrameBuffer((RAFB *)frameBuffer2);
    tft.useFrameBuffer(true);
    tft.updateScreenAsync();

    // now lets start up continuious reading and displaying...
    Serial.println("Press any key to enter continuous mode");
    while (Serial.read() == -1) {
    };
    while (Serial.read() != -1) {
    };
    camera.debug(false);
    camera.readContinuous(&camera_read_callback, frameBuffer, sizeof(frameBuffer), nullptr, 0);
    tft.setFrameBuffer((RAFB *)frameBuffer2);
}

bool camera_read_callback(void *pfb) {
    frame_count_camera++;
    if (tft.asyncUpdateActive())
        return true;
    frame_count_tft++;

#ifdef USE_T4_PXP
    uint16_t outputWidth, outputHeight;
    PXP_ps_output(tft.width(), tft.height(),       /* Display width and height */
                  camera.width(), camera.height(), /* Image width and height */
                  frameBuffer, PXP_RGB565, 2, 0,   /* Input buffer configuration */
                  frameBuffer2, PXP_RGB565, 2, 0,  /* Output buffer configuration */
                  TFT_ROTATION, 0, 0.0,            /* Rotation, flip, scaling */
                  &outputWidth, &outputHeight);    /* Frame Out size for drawing */
#else
    // not sure if I can update the CSI buffers without stopping camera and restarting the dma..
    // so first try copy memory
    memcpy(frameBuffer2, frameBuffer, sizeof(frameBuffer));
#endif
    tft.updateScreenAsync();
    return true;
}

void loop() {
    Serial.printf("%u %u\n", frame_count_camera, frame_count_tft);
    delay(1000);
}

/***********************************************************/

void test_display() {
    tft.fillScreen(TFT_RED);
    delay(500);
    tft.fillScreen(TFT_GREEN);
    delay(500);
    tft.fillScreen(TFT_BLUE);
    delay(500);
    tft.fillScreen(TFT_BLACK);
    delay(500);
}
