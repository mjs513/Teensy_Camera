#include <stdint.h>

#include <SPI.h>

#include "Teensy_Camera.h"

#define USE_MMOD_ATP_ADAPTER
#define USE_T4_PXP

// just a test
// #define USE_HX8357D

//#define DVP_CAMERA_OV2640
//#define DVP_CAMERA_OV5640
#define DVP_CAMERA_HM0360

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
#elif defined(DVP_CAMERA_HM0360)
#define CAMERA_USES_MONO_PALETTE
#include "Teensy_HM0360/HM0360.h"
HM0360 himax;
Camera camera(himax);
#define CameraID 0x0360
#define MIRROR_FLIP_CAMERA


#else
#error "Camera not selected"
#endif

// set cam configuration - need to remember when saving jpeg
#if defined(CAMERA_USES_MONO_PALETTE)
framesize_t camera_framesize = FRAMESIZE_VGA;
pixformat_t camera_format = RGB565;
#else
framesize_t camera_framesize = FRAMESIZE_480X320;
pixformat_t camera_format = RGB565;
#endif

bool useGPIO = false;
bool continuous_mode = false;

uint8_t reset_pin = 31;
uint8_t powdwn_pin = 30;

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
#define TFT_DC 4   // 0   // "TX1" on left side of Sparkfun ML Carrier
#define TFT_CS 5   // 4   // "CS" on left side of Sparkfun ML Carrier
#define TFT_RST 2  // 1  // "RX1" on left side of Sparkfun ML Carrier
#else
#define VSYNC_PIN 33
#define TFT_DC 0   // 20   // "TX1" on left side of Sparkfun ML Carrier
#define TFT_CS 4   // 5, 4   // "CS" on left side of Sparkfun ML Carrier
#define TFT_RST 1  // 2, 1  // "RX1" on left side of Sparkfun ML Carrier
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

// Later we will play with memory locations...
#if defined(CAMERA_USES_MONO_PALETTE)
// we will read in full VGA size
DMAMEM uint8_t camera_buffer[640 * 480] __attribute__((aligned(32)));
#else
DMAMEM uint16_t camera_buffer[480 * 320] __attribute__((aligned(32)));
#endif
uint16_t screen_buffer[480 * 320] __attribute__((aligned(32)));

#ifdef USE_T4_PXP
#include <T4_PXP.h>
#endif

volatile uint32_t frame_count_camera = 0;
volatile uint32_t frame_count_tft = 0;

inline void do_pxp_conversion(uint16_t &outputWidth, uint16_t &outputHeight) {

#if defined(CAMERA_USES_MONO_PALETTE)
    PXP_ps_output(tft.width(), tft.height(),       /* Display width and height */
                  camera.width(), camera.height(), /* Image width and height */
                  camera_buffer, PXP_Y8, 1, 0,     /* Input buffer configuration */
                  screen_buffer, PXP_RGB565, 2, 0, /* Output buffer configuration */
                  TFT_ROTATION, 0, 480.0 / 320.0,  /* Rotation, flip, scaling */
                  &outputWidth, &outputHeight);    /* Frame Out size for drawing */
#else
    PXP_ps_output(tft.width(), tft.height(),       /* Display width and height */
                  camera.width(), camera.height(), /* Image width and height */
                  camera_buffer, PXP_RGB565, 2, 0, /* Input buffer configuration */
                  screen_buffer, PXP_RGB565, 2, 0, /* Output buffer configuration */
                  TFT_ROTATION, 0, 0.0,            /* Rotation, flip, scaling */
                  &outputWidth, &outputHeight);    /* Frame Out size for drawing */
#endif
}


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

    tft.begin();
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
#ifdef USE_MMOD_ATP_ADAPTER
    pinMode(0, OUTPUT);
    pinMode(3, OUTPUT);
#elif defined(ARDUINO_TEENSY41)
    // CSI support
    pinMode(2, OUTPUT);
    pinMode(3, OUTPUT);
#endif

    // Start up the camera, with error recovery code
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
    start_camera();

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
    camera.setDMACompleteISRPriority(192);  // lower than default

    // Lets try reading in one frame
    delay(250);
    memset(camera_buffer, 0, sizeof(camera_buffer));
    memset(screen_buffer, 0, sizeof(screen_buffer));

    uint16_t outputWidth, outputHeight;
    for (uint8_t i = 0; i < 5; i++) {
        Serial.printf("Couple of Quick frames: %u\n", i);
        camera.setMode(HIMAX_MODE_STREAMING_NFRAMES, 1);
        camera.readFrame(camera_buffer, sizeof(camera_buffer));
        arm_dcache_flush((uint8_t *)camera_buffer, sizeof(camera_buffer));  // always flush cache after writing to DMAMEM variable that will be accessed by DMA
        Serial.println("After camera.read");
        Serial.flush();
#ifdef USE_T4_PXP
        do_pxp_conversion(outputWidth, outputHeight);
        Serial.println("After pxp conversion");
        Serial.flush();
        tft.writeRect(0, 0, outputWidth, outputHeight, screen_buffer);
#else
        tft.writeRect(0, 0, camera.width(), camera.height(), camera_buffer);
#endif

        delay(500);
    }

    Serial.println("Now try to async ");
    delay(1000);
#ifdef USE_T4_PXP
    camera.setMode(HIMAX_MODE_STREAMING_NFRAMES, 1);
    camera.readFrame(camera_buffer, sizeof(camera_buffer));
    do_pxp_conversion(outputWidth, outputHeight);
#else
    camera.readFrame(screen_buffer, sizeof(screen_buffer));
#endif
    tft.setFrameBuffer((RAFB *)screen_buffer);
    tft.useFrameBuffer(true);
    tft.updateScreenAsync();

    // now lets start up continuious reading and displaying...
    Serial.println("Press any key to enter continuous mode");
    while (Serial.read() == -1) {
    };
    while (Serial.read() != -1) {
    };
    camera.setMode(HIMAX_MODE_STREAMING, 0);  // turn on, continuous streaming mode
    if (!camera.readContinuous(&camera_read_callback, camera_buffer, sizeof(camera_buffer), nullptr, 0)) {
        Serial.print("\n*** Failed to enter Continuous mode ***");
    }
    camera.debug(false);
    tft.setFrameBuffer((RAFB *)screen_buffer);
    continuous_mode = true;
}

bool camera_read_callback(void *pfb) {
    frame_count_camera++;
    if (tft.asyncUpdateActive())
        return true;
    frame_count_tft++;

#ifdef USE_T4_PXP
    uint16_t outputWidth, outputHeight;
    do_pxp_conversion(outputWidth, outputHeight);
#else
    // not sure if I can update the CSI buffers without stopping camera and restarting the dma..
    // so first try copy memory
    memcpy(screen_buffer, camera_buffer, sizeof(camera_buffer));
#endif
    tft.updateScreenAsync();
    return true;
}

void loop() {
    if (Serial.available()) {
        while (Serial.read() != -1) {
            continuous_mode = !continuous_mode;
            if (continuous_mode) {
                Serial.println("Restart Continuous mode");
                camera.setMode(HIMAX_MODE_STREAMING, 0);  // turn on, continuous streaming mode
                if (!camera.readContinuous(&camera_read_callback, camera_buffer, sizeof(camera_buffer), nullptr, 0)) {
                    Serial.print("\n*** Failed to enter Continuous mode ***");
                }
            } else {
                Serial.println("Exit continuous mode");
                camera.stopReadContinuous();
            }
        }
    }
    if (continuous_mode) Serial.printf("%u %u\n", frame_count_camera, frame_count_tft);
    delay(1000);
}

/***********************************************************/
#if 1
void start_camera() {
    // Setup for OV5640 Camera
    // CSI support
    //  pinMode(2, OUTPUT);
    //  pinMode(3, OUTPUT);
    uint8_t reset_pin = 14;
    uint8_t powdwn_pin = 15;
    pinMode(powdwn_pin, INPUT);
    pinMode(reset_pin, INPUT_PULLUP);

    uint8_t status = 0;
#if defined(ARDUINO_TEENSY_DEVBRD4)
    camera.setPins(7, 8, 21, 46, 23, 40, 41, 42, 43, 44, 45, 6, 9);
#endif

#if defined(CAMERA_USES_MONO_PALETTE)
    status = camera.begin(camera_framesize, 15, useGPIO);
    camera.setPixformat(YUV422);
#else
    status = camera.begin(camera_framesize, camera_format, 15, CameraID, useGPIO);
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
        status = camera.begin(camera_framesize, 15, useGPIO);
        if (!status) {
            Serial.println("Camera failed to start again program halted");
            while (1) {}
        }
    }
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
    camera.setVSyncISRPriority(102);  // higher priority than default
    //camera.setDMACompleteISRPriority(192); // lower than default

    camera.setMode(HIMAX_MODE_STREAMING, 0);  // turn on, continuous streaming mode
}

#else
inline uint8_t camera_begin() {
#if defined(CAMERA_USES_MONO_PALETTE)
    if (!camera.begin(camera_framesize, 15, useGPIO)) return false;
    camera.setPixformat(YUV422);
    return true;
#else
    return camera.begin(camera_framesize, camera_format, 5, CameraID, useGPIO);
#endif
}

void start_camera() {
    uint8_t status = camera_begin();
    Serial.printf("Begin status: %d\n", status);
    if (!status) {
        Serial.println("Camera failed to start - try floating both reset and powndn!!!");
        // Serial.printf("\tPin 30:%u 31:%u\n", digitalReadFast(30), digitalReadFast(31));
        Serial.printf("\tPin rst(%u):%u PowDn(%u):%u\n", reset_pin, digitalRead(reset_pin), powdwn_pin, digitalRead(powdwn_pin));
        pinMode(reset_pin, INPUT);
        pinMode(powdwn_pin, INPUT);

        status = camera_begin();
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
        status = camera_begin();
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
        status = camera_begin();
    }
    if (!status) {
        Serial.println("Try powdn line Low");
        digitalWrite(powdwn_pin, LOW);

        digitalWriteFast(reset_pin, LOW);
        delay(100);
        digitalWriteFast(reset_pin, HIGH);
        delay(100);
        status = camera_begin();
    }
    if (!status) {
        Serial.println("Camera failed to start again program halted");
        while (1) {
        }
    }
}
#endif

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
