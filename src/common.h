typedef enum {
    PIXFORMAT_INVALID = 0,
    PIXFORMAT_GRAYSCALE, // 1BPP/GRAYSCALE
    PIXFORMAT_BAYER,     // 1BPP/RAW
    YUV422,
    RGB444,
    RGB565,
    BAYER,
    GRAYSCALE,
} pixformat_t;

typedef enum {
    // VGA Resolutions
    FRAMESIZE_VGA = 0,
    FRAMESIZE_QQVGA,    // 160x120
    FRAMESIZE_QVGA,     // 320x240
    FRAMESIZE_480X320,
	FRAMESIZE_320X320,  // 320x320
	FRAMESIZE_QVGA4BIT,
    FRAMESIZE_QCIF,
    FRAMESIZE_CIF,
    FRAMESIZE_INVALID,
} framesize_t;

typedef enum {
    GAINCEILING_2X,
    GAINCEILING_4X,
    GAINCEILING_8X,
    GAINCEILING_16X,
} gainceiling_t;

typedef enum {
	LOAD_DEFAULT_REGS,
	LOAD_WALKING1S_REG,
	LOAD_SHM01B0INIT_REGS,
    LOAD_320x240,
} camera_reg_settings_t;


typedef enum {
	TEENSY_MICROMOD_FLEXIO_8BIT = 0,
	TEENSY_MICROMOD_FLEXIO_4BIT,
} hw_config_t;

typedef enum {
	SPARKFUN_ML_CARRIER = 0,
	PJRC_CARRIER,
} hw_carrier_t;

typedef struct
{
    uint8_t                 ui8AETargetMean;
    uint8_t                 ui8AEMinMean;
    uint8_t                 ui8ConvergeInTh;
    uint8_t                 ui8ConvergeOutTh;
    uint8_t                 ui8AEMean;
} ae_cfg_t;

typedef struct
{
    uint8_t                 ui8IntegrationH;
    uint8_t                 ui8IntegrationL;
    uint8_t                 ui8AGain;
    uint8_t                 ui8DGain_H;
    uint8_t                 ui8DGain_L;
} hm01b0_snr_expo_gain_ctrl_t;


typedef enum {
    HIMAX_ERR_OK               = 0x00,
    HIMAX_ERR_AE_NOT_CONVERGED,
    HIMAX_ERR_PARAMS,
    HIMAX_NUM_ERR
} status_e;

enum
{
  OV7670 = 0,
  OV7675 = 1,
  GC2145a = 2
};