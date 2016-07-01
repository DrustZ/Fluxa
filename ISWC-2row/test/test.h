// Don't edit this file!  It's software-generated.
// See convert.py script instead.

#define PALETTE1  0
#define PALETTE4  1
#define PALETTE8  2
#define TRUECOLOR 3

#define NUM_LEDS 16

// my.png ------------------------------------------------------------------

const uint8_t PROGMEM palette00[][3] = {
  {   0,   0,   0 },
  { 175, 175, 175 } };

const uint8_t PROGMEM pixels00[] = {
  0X03, 0X00, 0X03, 0X00, 0X0C, 0X00, 0X0C, 0X00,
  0X30, 0X00, 0X30, 0X00, 0XC0, 0X00, 0XC0, 0X00,
  0X00, 0X03, 0X00, 0X03, 0X00, 0X0C, 0X00, 0X0C,
  0X00, 0X30, 0X00, 0X30, 0X00, 0XC0, 0X00, 0XC0 };

typedef struct {
  uint8_t        type;    // PALETTE[1,4,8] or TRUECOLOR
  line_t         lines;   // Length of image (in scanlines)
  const uint8_t *palette; // -> PROGMEM color table (NULL if truecolor)
  const uint8_t *pixels;  // -> Pixel data in PROGMEM
} image;

const image PROGMEM images[] = {
  { PALETTE1 ,   16, (const uint8_t *)palette00, pixels00 }
};

#define NUM_IMAGES (sizeof(images) / sizeof(images[0]))
