#include <stdint.h>

#include <SPI.h>

#include "Teensy_Camera.h"

#define USE_MMOD_ATP_ADAPTER
#define USE_T4_PXP
#define PXP_LET_SCREEN_DO_ROTATE  //
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
//#define MIRROR_FLIP_CAMERA
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

float pxp_scale_factor = 1.3333333333333;
bool pxp_next_initialized = false;



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
#if defined(CAMERA_USES_MONO_PALETTE)
#define TFT_ROTATION 1
#else
#define TFT_ROTATION 3
#endif
#else
#define TFT_ROTATION 1
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

#ifdef USE_T4_PXP

//=================================================================================================
// PXP_ps_window_output
//=================================================================================================

void PXP_ps_window_output(uint16_t disp_width, uint16_t disp_height, uint16_t image_width, uint16_t image_height,
                          void *buf_in, uint8_t format_in, uint8_t bpp_in, uint8_t byte_swap_in,
                          void *buf_out, uint8_t format_out, uint8_t bpp_out, uint8_t byte_swap_out,
                          uint8_t rotation, bool flip, float scaling,
                          uint16_t *scr_width, uint16_t *scr_height) {

    // Lets see if we can setup a window into the camera data
    Serial.printf("PXP_ps_window_output(%u, %u, %u, %u.... %u, %u. %f)\n", disp_width, disp_height, image_width, image_height,
                  rotation, flip, scaling);

    // we may bypass some of the helper functions and output the data directly.
    volatile IMXRT_NEXT_PXP_t *next_pxp = PXP_next_pxp();


    // lets reduce down to basic.
    PXP_input_background_color(0, 0, 153);

    // Initial input stuff.
    PXP_input_buffer(buf_in, bpp_in, image_width, image_height);
    // VYUY1P422, PXP_UYVY1P422
    PXP_input_format(format_in, 0, 0, byte_swap_in);
    if (format_in == PXP_Y8 || format_in == PXP_Y4)
        PXP_set_csc_y8_to_rgb();

    // Output stuff
    uint16_t out_width, out_height, output_Width, output_Height;
    if (rotation == 1 || rotation == 3) {
        out_width = disp_height;
        out_height = disp_width;
    } else {
        out_width = disp_width;
        out_height = disp_height;
    }

    PXP_output_buffer(buf_out, bpp_out, disp_width, disp_height);
    PXP_output_format(format_out, 0, 0, byte_swap_out);

    // Set the clip rect.
    PXP_output_clip(disp_width - 1, disp_height - 1);

    // Rotation
    /* Setting this bit to 1'b0 will place the rotationre sources at  *
   * the output stage of the PXP data path. Image compositing will  *
   * occur before pixels are processed for rotation.                *
   * Setting this bit to a 1'b1 will place the rotation resources   *
   * before image composition.                                      *
   */
    PXP_rotate_position(0);
    //Serial.println("Rotating");
    // Performs the actual rotation specified
    PXP_rotate(rotation);

    // flip - pretty straight forward
    PXP_flip(flip);

    // lets setup the scaling.
    uint16_t pxp_scale = scaling * 4096;
    next_pxp->PS_SCALE = PXP_XSCALE(pxp_scale) | PXP_YSCALE(pxp_scale);

    // now if the input image and the scaled output image are not the same size, we may want to center either
    // the input or the output...
    uint32_t scaled_image_width = image_width / scaling;
    uint32_t scaled_image_height = image_height / scaling;

    uint16_t ul_x = 0;
    uint16_t ul_y = 0;
    uint16_t lr_x = disp_width - 1;
    uint16_t lr_y = disp_height - 1;

    if (scaled_image_width < disp_width) {
        ul_x = (disp_width - scaled_image_width) / 2;
        lr_x = ul_x + scaled_image_width - 1;
    }
    if (scaled_image_height < disp_height) {
        ul_y = (disp_height - scaled_image_height) / 2;
        lr_y = ul_y + scaled_image_height - 1;
    }
    PXP_input_position(ul_x, ul_y, lr_x, lr_y);  // need this to override the setup in pxp_input_buffer

    // See if we should input clip the image...  That is center the large image onto the screen.  later allow for panning.
    uint8_t *buf_in_clipped = (uint8_t *)buf_in;

    if (scaled_image_width > disp_width) {
        buf_in_clipped += ((scaled_image_width - disp_width) / 2) * bpp_in;
    }

    if (scaled_image_height > disp_height) {
        buf_in_clipped += ((scaled_image_height - disp_height) / 2) * bpp_in * image_width;
    }

    if (buf_in_clipped != (uint8_t*)buf_in) {
        Serial.printf("Input clip Buffer(%p -> %p)\n", buf_in, buf_in_clipped);
        next_pxp->PS_BUF = buf_in_clipped;
    }

    *scr_width = disp_width;
    *scr_height = disp_height;



    PXP_process();
}





inline void do_pxp_conversion(uint16_t &outputWidth, uint16_t &outputHeight) {

    if (pxp_next_initialized) {
        PXP_process();
        return;
    }
    pxp_next_initialized = true;
#if defined(CAMERA_USES_MONO_PALETTE)
#ifdef PXP_LET_SCREEN_DO_ROTATE
    uint8_t rotate = 0;
#else
    uint8_t rotate = TFT_ROTATION;
#endif
#if 1
    // lets try to unwind the PXP_ps_output code and instead of clip
    // se if we can setup window into the source with the output size.
    PXP_ps_window_output(tft.width(), tft.height(),       /* Display width and height */
                         camera.width(), camera.height(), /* Image width and height */
                         camera_buffer, PXP_Y8, 1, 0,     /* Input buffer configuration */
                         screen_buffer, PXP_RGB565, 2, 0, /* Output buffer configuration */
                         rotate, 0, pxp_scale_factor,                    /* Rotation, flip, scaling */
                         &outputWidth, &outputHeight);    /* Frame Out size for drawing */

#else
    PXP_ps_output(tft.width(), tft.height(),       /* Display width and height */
                  camera.width(), camera.height(), /* Image width and height */
                  camera_buffer, PXP_Y8, 1, 0,     /* Input buffer configuration */
                  screen_buffer, PXP_RGB565, 2, 0, /* Output buffer configuration */
                  rotate, 0, 640.0 / 480.0,        /* Rotation, flip, scaling */
                  &outputWidth, &outputHeight);    /* Frame Out size for drawing */
#endif
#else
    PXP_ps_output(tft.width(), tft.height(),       /* Display width and height */
                  camera.width(), camera.height(), /* Image width and height */
                  camera_buffer, PXP_RGB565, 2, 0, /* Input buffer configuration */
                  screen_buffer, PXP_RGB565, 2, 0, /* Output buffer configuration */
                  rotate, true, 0.0,               /* Rotation, flip, scaling */
                  &outputWidth, &outputHeight);    /* Frame Out size for drawing */
#endif
    PXPShowNext();
    Serial.flush();
}
#endif

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

#ifdef PXP_LET_SCREEN_DO_ROTATE
    tft.setRotation(TFT_ROTATION);
#else
    tft.setRotation(0);
#endif
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
    tft.setFrameCompleteCB(&TFTFrameComplete);

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
    pinMode(1, OUTPUT);

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
#ifndef CAMERA_USES_MONO_PALETTE
    omni.setWBmode(true);
#endif
    camera.setPixformat(camera_format);

    delay(1000);
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


#ifdef USE_T4_PXP
    PXPShow();
#endif
    // now lets start up continuious reading and displaying...
    Serial.println("Press any key to enter continuous mode");
    while (Serial.read() == -1) {
    };
    while (Serial.read() != -1) {
    };

#ifdef PXP_CALLBACK_SUPPORTED
    Serial.println("*** Setting PXP Callback function ***");
    PXP_setPXPDoneCB(&pxpCompleteCB);
#endif

    camera.setMode(HIMAX_MODE_STREAMING, 0);  // turn on, continuous streaming mode
    if (!camera.readContinuous(&camera_read_callback, camera_buffer, sizeof(camera_buffer), nullptr, 0)) {
        Serial.print("\n*** Failed to enter Continuous mode ***");
    }
    camera.debug(false);
    tft.setFrameBuffer((RAFB *)screen_buffer);
    continuous_mode = true;
}

#ifdef PXP_CALLBACK_SUPPORTED
// lets try to use CB from PXP to start the update screen...
void pxpCompleteCB() {
    tft.updateScreenAsync();
    digitalWriteFast(1, HIGH);
}

#endif

bool camera_read_callback(void *pfb) {
    digitalWriteFast(3, HIGH);
    frame_count_camera++;
    if (tft.asyncUpdateActive()) {
        digitalWriteFast(3, LOW);
        return true;
    }
    frame_count_tft++;

#ifdef USE_T4_PXP
    uint16_t outputWidth, outputHeight;
    do_pxp_conversion(outputWidth, outputHeight);
#else
    // not sure if I can update the CSI buffers without stopping camera and restarting the dma..
    // so first try copy memory
    memcpy(screen_buffer, camera_buffer, sizeof(camera_buffer));
#endif
#ifndef PXP_CALLBACK_SUPPORTED
    tft.updateScreenAsync();
    digitalWriteFast(1, HIGH);
#endif
    digitalWriteFast(3, LOW);
    return true;
}

void TFTFrameComplete() {
    digitalWriteFast(1, LOW);
}


void loop() {
    if (Serial.available()) {
        float next_scale = 0;
        next_scale = Serial.parseFloat();
        while (Serial.read() != -1) {}
        continuous_mode = !continuous_mode;
        if (continuous_mode) {
            Serial.printf("Restart Continuous mode(%f)\n", next_scale);
            pxp_scale_factor = next_scale;
            pxp_next_initialized = false;  // have it rebuild it...
            memset(screen_buffer, 0, sizeof(screen_buffer));

            camera.setMode(HIMAX_MODE_STREAMING, 0);  // turn on, continuous streaming mode
            if (!camera.readContinuous(&camera_read_callback, camera_buffer, sizeof(camera_buffer), nullptr, 0)) {
                Serial.print("\n*** Failed to enter Continuous mode ***");
            }
        } else {
            Serial.println("Exit continuous mode");
            camera.stopReadContinuous();
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
#if defined(CAMERA_USES_MONO_PALETTE)
        status = camera.begin(camera_framesize, 15, useGPIO);
        camera.setPixformat(YUV422);
#else
        status = camera.begin(camera_framesize, camera_format, 15, CameraID, useGPIO);
#endif
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
void PXPShow(void) {
    Serial.printf("CTRL:         %08X       STAT:         %08X\n", PXP_CTRL, PXP_STAT);
    Serial.printf("OUT_CTRL:     %08X       OUT_BUF:      %08X    \nOUT_BUF2:     %08X\n", PXP_OUT_CTRL, PXP_OUT_BUF, PXP_OUT_BUF2);
    Serial.printf("OUT_PITCH:    %8lu       OUT_LRC:       %3u,%3u\n", PXP_OUT_PITCH, PXP_OUT_LRC >> 16, PXP_OUT_LRC & 0xFFFF);

    Serial.printf("OUT_PS_ULC:    %3u,%3u       OUT_PS_LRC:    %3u,%3u\n", PXP_OUT_PS_ULC >> 16, PXP_OUT_PS_ULC & 0xFFFF,
                  PXP_OUT_PS_LRC >> 16, PXP_OUT_PS_LRC & 0xFFFF);
    Serial.printf("OUT_AS_ULC:    %3u,%3u       OUT_AS_LRC:    %3u,%3u\n", PXP_OUT_AS_ULC >> 16, PXP_OUT_AS_ULC & 0xFFFF,
                  PXP_OUT_AS_LRC >> 16, PXP_OUT_AS_LRC & 0xFFFF);
    Serial.println();  // section separator
    Serial.printf("PS_CTRL:      %08X       PS_BUF:       %08X\n", PXP_PS_CTRL, PXP_PS_BUF);
    Serial.printf("PS_UBUF:      %08X       PS_VBUF:      %08X\n", PXP_PS_UBUF, PXP_PS_VBUF);
    Serial.printf("PS_PITCH:     %8lu       PS_BKGND:     %08X\n", PXP_PS_PITCH, PXP_PS_BACKGROUND_0);
    Serial.printf("PS_SCALE:     %08X       PS_OFFSET:    %08X\n", PXP_PS_SCALE, PXP_PS_OFFSET);
    Serial.printf("PS_CLRKEYLOW: %08X       PS_CLRKEYLHI: %08X\n", PXP_PS_CLRKEYLOW_0, PXP_PS_CLRKEYHIGH_0);
    Serial.println();
    Serial.printf("AS_CTRL:      %08X       AS_BUF:       %08X    AS_PITCH: %6u\n", PXP_AS_CTRL, PXP_AS_BUF, PXP_AS_PITCH & 0xFFFF);
    Serial.printf("AS_CLRKEYLOW: %08X       AS_CLRKEYLHI: %08X\n", PXP_AS_CLRKEYLOW_0, PXP_AS_CLRKEYHIGH_0);
    Serial.println();
    Serial.printf("CSC1_COEF0:   %08X       CSC1_COEF1:   %08X    \nCSC1_COEF2:   %08X\n",
                  PXP_CSC1_COEF0, PXP_CSC1_COEF1, PXP_CSC1_COEF2);
    Serial.println();  // section separator
    Serial.printf("POWER:        %08X       NEXT:         %08X\n", PXP_POWER, PXP_NEXT);
    Serial.printf("PORTER_DUFF:  %08X\n", PXP_PORTER_DUFF_CTRL);
}

void PXPShowNext(void) {
    volatile IMXRT_NEXT_PXP_t *next_pxp = PXP_next_pxp();
    Serial.printf("Show next PXP data(%p)\n", next_pxp);
    Serial.printf("CTRL:         %08X       STAT:         %08X\n", next_pxp->CTRL, next_pxp->STAT);
    Serial.printf("OUT_CTRL:     %08X       OUT_BUF:      %08X    \nOUT_BUF2:     %08X\n", next_pxp->OUT_CTRL, next_pxp->OUT_BUF, next_pxp->OUT_BUF2);
    Serial.printf("OUT_PITCH:    %8lu       OUT_LRC:       %3u,%3u\n", next_pxp->OUT_PITCH, next_pxp->OUT_LRC >> 16, next_pxp->OUT_LRC & 0xFFFF);

    Serial.printf("OUT_PS_ULC:    %3u,%3u       OUT_PS_LRC:    %3u,%3u\n", next_pxp->OUT_PS_ULC >> 16, next_pxp->OUT_PS_ULC & 0xFFFF,
                  next_pxp->OUT_PS_LRC >> 16, next_pxp->OUT_PS_LRC & 0xFFFF);
    Serial.printf("OUT_AS_ULC:    %3u,%3u       OUT_AS_LRC:    %3u,%3u\n", next_pxp->OUT_AS_ULC >> 16, next_pxp->OUT_AS_ULC & 0xFFFF,
                  next_pxp->OUT_AS_LRC >> 16, next_pxp->OUT_AS_LRC & 0xFFFF);
    Serial.println();  // section separator
    Serial.printf("PS_CTRL:      %08X       PS_BUF:       %08X\n", next_pxp->PS_CTRL, next_pxp->PS_BUF);
    Serial.printf("PS_UBUF:      %08X       PS_VBUF:      %08X\n", next_pxp->PS_UBUF, next_pxp->PS_VBUF);
    Serial.printf("PS_PITCH:     %8lu       PS_BKGND:     %08X\n", next_pxp->PS_PITCH, next_pxp->PS_BACKGROUND);
    float x_ps_scale = (next_pxp->PS_SCALE & 0xffff) / 4096.0;
    float y_ps_scale = (next_pxp->PS_SCALE >> 16) / 4096.0;
    Serial.printf("PS_SCALE:  %5.3f,%5.3f       PS_OFFSET:    %08X\n", x_ps_scale, y_ps_scale, next_pxp->PS_OFFSET);
    Serial.printf("PS_CLRKEYLOW: %08X       PS_CLRKEYLHI: %08X\n", next_pxp->PS_CLRKEYLOW, next_pxp->PS_CLRKEYHIGH);
    Serial.println();
    Serial.printf("AS_CTRL:      %08X       AS_BUF:       %08X    AS_PITCH: %6u\n", next_pxp->AS_CTRL, next_pxp->AS_BUF, next_pxp->AS_PITCH & 0xFFFF);
    Serial.printf("AS_CLRKEYLOW: %08X       AS_CLRKEYLHI: %08X\n", next_pxp->AS_CLRKEYLOW, next_pxp->AS_CLRKEYHIGH);
    Serial.println();
    Serial.printf("CSC1_COEF0:   %08X       CSC1_COEF1:   %08X    \nCSC1_COEF2:   %08X\n",
                  next_pxp->CSC1_COEF0, next_pxp->CSC1_COEF1, next_pxp->CSC1_COEF2);
    Serial.println();  // section separator
    Serial.printf("POWER:        %08X       NEXT:         %08X\n", next_pxp->POWER, next_pxp->NEXT);
    Serial.printf("PORTER_DUFF:  %08X\n", next_pxp->PORTER_DUFF_CTRL);
}
