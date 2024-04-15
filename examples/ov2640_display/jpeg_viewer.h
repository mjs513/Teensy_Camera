// Options file information
static const PROGMEM char options_file_name[] = "PictureViewOptions.ini";
int g_debug_output = 0;
int g_stepMode = 0;
int g_BMPScale = -1;
int g_JPGScale = 0;
int g_PNGScale = 1;
int g_center_image = 1;
int g_display_image_time = 2500;
int g_background_color = TFT_BLACK;
int g_max_scale_up = 4;

// scale boundaries {2, 4, 8, 16<maybe>}
enum { SCL_HALF = 0, SCL_QUARTER, SCL_EIGHTH, SCL_16TH };
int g_jpg_scale_x_above[4];
int g_jpg_scale_y_above[4];

// variables used in some of the display output functions
int g_tft_width = 0;
int g_tft_height = 0;
int g_image_offset_x = 0;
int g_image_offset_y = 0;
uint8_t g_image_scale = 1;
uint8_t g_image_scale_up = 0;
uint32_t g_WRCount = 0; // debug count how many time writeRect called
