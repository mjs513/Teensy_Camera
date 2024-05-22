JPEGDEC jpeg;

//****************************************************************************
// forward function definitions.
//****************************************************************************
inline void writeClippedRect(int16_t x, int16_t y, int16_t cx, int16_t cy, uint16_t *pixels, bool waitForWRC = true) {
    tft.writeRect(x + g_image_offset_x, y + g_image_offset_y, cx, cy, pixels);
}

inline void Color565ToRGB(uint16_t color, uint8_t &r, uint8_t &g, uint8_t &b) {
    tft.color565toRGB(color, r, g, b);
}

inline void FillScreen(uint16_t color) {
    tft.fillScreen(color);
}


bool processJPGFile(bool fErase) {
    //-----------------------------------------------------------------------------
    // Initialize options and then read optional config file
    //-----------------------------------------------------------------------------

    g_tft_width = tft.width();
    g_tft_height = tft.height();
    g_jpg_scale_x_above[0] = (g_tft_width * 3) / 2;
    g_jpg_scale_x_above[1] = g_tft_width * 3;
    g_jpg_scale_x_above[2] = g_tft_width * 6;
    g_jpg_scale_x_above[3] = g_tft_width * 12;

    g_jpg_scale_y_above[0] = (g_tft_height * 3) / 2;
    g_jpg_scale_y_above[1] = g_tft_height * 3;
    g_jpg_scale_y_above[2] = g_tft_height * 6;
    g_jpg_scale_y_above[3] = g_tft_height * 12;

    Serial.println();

    Serial.print(F("Loading JPG image from Framebuffer'"));
    Serial.println('\'');

    uint8_t eoi = 0;
    uint32_t eop = 0;


    uint32_t bytesRead = readJPG(eoi, eop, true);
    if (bytesRead == 0) return 0;

    uint8_t scale = 1;
    if (jpeg.openRAM((uint8_t *)frameBuffer, sizeof(frameBuffer), JPEGDraw)) {
        //if (jpeg.open(name, myOpen, myClose, myReadJPG, mySeekJPG, JPEGDraw)) {
        int image_width = jpeg.getWidth();
        int image_height = jpeg.getHeight();
        int decode_options = 0;
        Serial.printf("Image size: %dx%d", image_width, image_height);
        switch (g_JPGScale) {
            case 1:
                scale = 1;
                decode_options = 0;
                break;
            case 2:
                scale = 2;
                decode_options = JPEG_SCALE_HALF;
                break;
            case 4:
                scale = 4;
                decode_options = JPEG_SCALE_QUARTER;
                break;
            case 8:
                scale = 8;
                decode_options = JPEG_SCALE_EIGHTH;
                break;
            default:
                {
                    if ((image_width > g_jpg_scale_x_above[SCL_16TH]) || (image_height > g_jpg_scale_y_above[SCL_16TH])) {
                        decode_options = JPEG_SCALE_EIGHTH | JPEG_SCALE_HALF;
                        scale = 16;
                    } else if ((image_width > g_jpg_scale_x_above[SCL_EIGHTH]) || (image_height > g_jpg_scale_y_above[SCL_EIGHTH])) {
                        decode_options = JPEG_SCALE_EIGHTH;
                        scale = 8;
                    } else if ((image_width > g_jpg_scale_x_above[SCL_QUARTER]) || (image_height > g_jpg_scale_y_above[SCL_QUARTER])) {
                        decode_options = JPEG_SCALE_QUARTER;
                        scale = 4;
                    } else if ((image_width > g_jpg_scale_x_above[SCL_HALF]) || (image_height > g_jpg_scale_y_above[SCL_HALF])) {
                        decode_options = JPEG_SCALE_HALF;
                        scale = 2;
                    }
                }
        }
        if (fErase && ((image_width / scale < g_tft_width) || (image_height / scale < g_tft_height))) {
            FillScreen((uint16_t)g_background_color);
        }

        if (g_center_image) {
            g_image_offset_x = (g_tft_width - image_width / scale) / 2;
            g_image_offset_y = (g_tft_height - image_height / scale) / 2;
        } else {
            g_image_offset_x = 0;
            g_image_offset_y = 0;
        }
        g_image_scale = scale;
        Serial.printf("Scale: 1/%d Image Offsets (%d, %d)\n", g_image_scale, g_image_offset_x, g_image_offset_y);
        Serial.printf("Scale: 1/%d tft dim (%d, %d)\n", g_image_scale, g_tft_width, g_tft_height);

        jpeg.decode(0, 0, decode_options);
        jpeg.close();
    } else {
        Serial.println("Was not a valid jpeg file");
        return 0;
    }
    return 1;
}

int JPEGDraw(JPEGDRAW *pDraw) {
    if (g_debug_output) Serial.printf("jpeg draw: x,y=%d,%d, cx,cy = %d,%d\n",
                                      pDraw->x, pDraw->y, pDraw->iWidth, pDraw->iHeight);

    writeClippedRect(pDraw->x, pDraw->y, pDraw->iWidth, pDraw->iHeight, pDraw->pPixels);
    delay(5);
    return 1;
}



//=============================================================================
// TFT Helper functions to work on ILI9341_t3
// which doe snot have offset/clipping support
//=============================================================================

// Function to draw pixels to the display
void ScaleUpWriteClippedRect(int row, int image_width, uint16_t *usPixels) {
    //--------------------------------------------------------------------
    // experiment scale up...
    //--------------------------------------------------------------------
    uint16_t *usRowOut = usPixels + 2 * image_width;  //
    uint8_t r_cur, r_prev, g_cur, g_prev, b_cur, b_prev;
    int red, green, blue;
    int image_width_out = (image_width - 1) * g_image_scale_up + 1;  // we don't fill out the last one.
    // if this is not our first row, then we need to compute the fill in row
    // first...
    // Our buffer has g_image_scale_up rows of data to send in one chunk
    uint16_t *puCol = usRowOut;
    uint16_t *puCurRow;
    uint16_t *puPrevRow;
    if (row & 1) {
        puCurRow = usPixels + image_width;
        puPrevRow = usPixels;
    } else {
        puCurRow = usPixels;
        puPrevRow = usPixels + image_width;
    }

    // First lets generate the one for the actual row;
    uint16_t *p = usRowOut + image_width_out * (g_image_scale_up - 1);
    uint16_t *ppixIn = puCurRow;
    for (int col = 0; col < image_width; col++) {
        // bug bug.. could be faster
        *p = *ppixIn++;  // copy the pixel in to pixel out
        if (col) {
            // Now lets fill in the columns between the prev and new...
            Color565ToRGB(*p, r_cur, g_cur, b_cur);
            Color565ToRGB(*(p - g_image_scale_up), r_prev, g_prev, b_prev);
            for (int j = 1; j < g_image_scale_up; j++) {
                red = (int)r_prev + (((int)r_cur - (int)r_prev) * j) / g_image_scale_up;
                green = (int)g_prev + (((int)g_cur - (int)g_prev) * j) / g_image_scale_up;
                blue = (int)b_prev + (((int)b_cur - (int)b_prev) * j) / g_image_scale_up;
                *(p - g_image_scale_up + j) = color565(red, green, blue);
            }
        }
        p += g_image_scale_up;
    }

    // except for the first row we now need to fill in the extra rows from the previous one
    if (row) {
        for (int col = 0; col < image_width; col++) {
            Color565ToRGB(*puCurRow++, r_cur, g_cur, b_cur);
            Color565ToRGB(*puPrevRow++, r_prev, g_prev, b_prev);
            for (int i = 1; i < g_image_scale_up; i++) {
                uint16_t *p = puCol + (i - 1) * image_width_out;  // so location for this item
                int red = (int)r_prev + (((int)r_cur - (int)r_prev) * i) / g_image_scale_up;
                int green = (int)g_prev + (((int)g_cur - (int)g_prev) * i) / g_image_scale_up;
                int blue = (int)b_prev + (((int)b_cur - (int)b_prev) * i) / g_image_scale_up;
                *p = color565(red, green, blue);
                // need to compute middle ones as well.
                if (col) {
                    // Now lets fill in the columns between the prev and new...
                    Color565ToRGB(*p, r_cur, g_cur, b_cur);
                    Color565ToRGB(*(p - g_image_scale_up), r_prev, g_prev, b_prev);
                    for (int j = 1; j < g_image_scale_up; j++) {
                        red = (int)r_prev + (((int)r_cur - (int)r_prev) * j) / g_image_scale_up;
                        green = (int)g_prev + (((int)g_cur - (int)g_prev) * j) / g_image_scale_up;
                        blue = (int)b_prev + (((int)b_cur - (int)b_prev) * j) / g_image_scale_up;
                        *(p - g_image_scale_up + j) = color565(red, green, blue);
                    }
                }
            }
            puCol += g_image_scale_up;
        }
        writeClippedRect(0, 1 + (row - 1) * g_image_scale_up, image_width_out, g_image_scale_up, usRowOut);
    } else {
        // first row just output it's own data.
        writeClippedRect(0, 0, image_width_out, 1, usRowOut + image_width_out * (g_image_scale_up - 1));
    }
}

void ScaleDownWriteClippedRect(int row, int image_width, uint16_t *usPixels) {
    if ((row % g_image_scale) == (g_image_scale - 1)) {
        //--------------------------------------------------------------------
        // else scale down
        //--------------------------------------------------------------------
        uint16_t newx = 0;
        for (uint16_t pix_cnt = 0; pix_cnt < image_width; pix_cnt += g_image_scale) {
            uint8_t red = 0;
            uint8_t green = 0;
            uint8_t blue = 0;
            float r = 0;
            float g = 0;
            float b = 0;
            for (uint8_t i = 0; i < g_image_scale; i++) {
                for (uint8_t j = 0; j < g_image_scale; j++) {
                    Color565ToRGB(usPixels[pix_cnt + i + (j * image_width)], red, green, blue);
                    // Sum the squares of components instead
                    r += red * red;
                    g += green * green;
                    b += blue * blue;
                }
            }
            // overwrite the start of our buffer with
            usPixels[newx++] = color565((uint8_t)sqrt(r / (g_image_scale * g_image_scale)), (uint8_t)sqrt(g / (g_image_scale * g_image_scale)), (uint8_t)sqrt(b / (g_image_scale * g_image_scale)));
        }
        writeClippedRect(0, row / g_image_scale, image_width / g_image_scale, 1, usPixels);
    }
}




//=============================================================================
// Options file support - process only if file changed dates (Or first time)
//    example looking for update file.
// This is a real simple parser x=y where x is string y is int...
//=============================================================================
DateTimeFields g_dtf_optFileLast = { 99 };  // not valid so change first time...
#define MAX_KEY_NAME 20
typedef struct {
    const char key_name[MAX_KEY_NAME];
    int *key_value_addr;
} key_name_value_t;

static const PROGMEM key_name_value_t keyNameValues[] = {
    { "Background", &g_background_color },
    { "debug", &g_debug_output },
    { "Step", &g_stepMode },
    { "BMPScale", &g_BMPScale },
    { "JPGScale", &g_JPGScale },
    { "PNGScale", &g_PNGScale },
    { "ScaleXAbove2", &g_jpg_scale_x_above[SCL_HALF] },
    { "ScaleXAbove4", &g_jpg_scale_x_above[SCL_QUARTER] },
    { "ScaleXAbove8", &g_jpg_scale_x_above[SCL_EIGHTH] },
    { "ScaleXAbove16", &g_jpg_scale_x_above[SCL_16TH] },
    { "ScaleYAbove2", &g_jpg_scale_y_above[SCL_HALF] },
    { "ScaleYAbove4", &g_jpg_scale_y_above[SCL_QUARTER] },
    { "ScaleYAbove8", &g_jpg_scale_y_above[SCL_EIGHTH] },
    { "ScaleYAbove16", &g_jpg_scale_y_above[SCL_16TH] },
    { "Center", &g_center_image },
    { "MaxScaleUp", &g_max_scale_up },
    { "ImageTimeMS", &g_display_image_time }
};

bool ReadOptionsLine(File &optFile, char *key_name, uint8_t sizeof_key, int &key_value) {
    int ch;

    key_value = 0;
    // first lets get key name ignore all whitespace...
    while ((ch = optFile.read()) <= ' ') {
        if (ch < 0) return false;
    }

    uint8_t ich = 0;
    while (ich < (sizeof_key - 1)) {
        if (ch == '=') {
            ch = optFile.read();
            break;
        }
        key_name[ich++] = ch;
        ch = optFile.read();
        if (ch < 0) return false;  //
    }
    key_name[ich] = '\0';

    int sign_value = 1;
    if (ch == '-') {
        sign_value = -1;
        ch = optFile.read();
        if (ch == -1) return false;
    }

    while ((ch >= '0') && (ch <= '9')) {
        key_value = key_value * 10 + ch - '0';
        ch = optFile.read();
    }
    // should probably check for other stuff, but...
    key_value *= sign_value;

    // total hacky but allow hex value
    if ((key_value == 0) && ((ch == 'x') || (ch == 'X'))) {
        ch = optFile.read();
        for (;;) {
            if ((ch >= '0') && (ch <= '9')) key_value = key_value * 16 + ch - '0';
            else if ((ch >= 'a') && (ch <= 'f'))
                key_value = key_value * 16 + 10 + ch - 'a';
            else if ((ch >= 'A') && (ch <= 'F'))
                key_value = key_value * 16 + 10 + ch - 'A';
            else
                break;
            ch = optFile.read();
        }
    }

    return true;
}


bool ProcessOptionsFile(File &optfile) {
    tft.setRotation(0);

    DateTimeFields dtf;
    int key_value;
    char key_name[20];
    if (!optfile) return false;
    if (!optfile.getModifyTime(dtf)) return false;
    if (memcmp(&dtf, &g_dtf_optFileLast, sizeof(dtf)) == 0) return false;
    g_dtf_optFileLast = dtf;
    Serial.printf("Updated Options file found date: M: %02u/%02u/%04u %02u:%02u\n",
                  dtf.mon + 1, dtf.mday, dtf.year + 1900, dtf.hour, dtf.min);

    // do simple scan through file
    bool found = false;
    while (ReadOptionsLine(optfile, key_name, sizeof(key_name), key_value)) {
        Serial.printf("\t>>%s=%d", key_name, key_value);
        for (uint8_t key_index = 0; key_index < (sizeof(keyNameValues) / sizeof(keyNameValues[0])); key_index++) {
            if (stricmp(key_name, keyNameValues[key_index].key_name) == 0) {
                Serial.printf(" was: %d\n", *(keyNameValues[key_index].key_value_addr));
                *(keyNameValues[key_index].key_value_addr) = key_value;
                found = true;
                break;
            }
        }
        if (!found) Serial.println(" ** Unknown Key **");
    }
    tft.setRotation(0);
    return true;
}

void ShowAllOptionValues() {
    Serial.println("\n----------------------------------");
    Serial.printf("Sketch uses Option file: %s at the root of SD Card\n", options_file_name);
    Serial.println("\t<All key names>=<current key value");
    for (uint8_t key_index = 0; key_index < (sizeof(keyNameValues) / sizeof(keyNameValues[0])); key_index++) {
        Serial.printf("\t%s=%d\n", keyNameValues[key_index].key_name, *(keyNameValues[key_index].key_value_addr));
    }
    Serial.println("----------------------------------\n");
}
