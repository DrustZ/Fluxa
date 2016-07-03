#include <Arduino.h>
#include <Adafruit_DotStar.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS0.h>

typedef uint16_t line_t;


// CONFIGURABLE STUFF ------------------------------------------------------

#include "newtest.h" // Graphics data is contained in this header file.

Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(1000);  // Use I2C, ID #1000

// Ideally you use hardware SPI as it's much faster, though limited to
// specific pins.  If you really need to bitbang DotStar data & clock on
// different pins, optionally define those here:
#define LED_DATA_PIN  4
#define LED_CLOCK_PIN 5

//#define LED_DATA_PIN  2
//#define LED_CLOCK_PIN 3
int flag = 0;

// Select from multiple images using tactile button (#1489) between pin and
// ground.  Requires suitably-built graphics.h file w/more than one image.
#define SELECT_PIN 3

#define SLEEP_TIME 2000  // Not-spinning time before sleep, in milliseconds


boolean autoCycle = false; // Set to true to cycle images by default
#define CYCLE_TIME 3      // Time, in seconds, between auto-cycle images

// -------------------------------------------------------------------------

#if defined(LED_DATA_PIN) && defined(LED_CLOCK_PIN)
// Older DotStar LEDs use GBR order.  If colors are wrong, edit here.
// originally was DOTSTAR_BRG -> I changed to BGR
Adafruit_DotStar strip = Adafruit_DotStar(NUM_LEDS,
                         LED_DATA_PIN, LED_CLOCK_PIN, DOTSTAR_BGR);
#else
Adafruit_DotStar strip = Adafruit_DotStar(NUM_LEDS, DOTSTAR_BGR);
#endif

void     imageInit(void);
uint16_t readVoltage(void);
#ifdef MOTION_PIN
void     sleep(void);
#endif

int brightness = 0;
int frameSkip = 5;
int ct = 0;
void setup() {

  strip.begin();                // Allocate DotStar buffer, init SPI
  strip.clear();                // Make sure strip is clear
  strip.show();                 // before measuring battery
  strip.setBrightness(70);
  brightness = 0;
  imageInit(); // Initialize pointers for default image

#ifndef ESP8266
  while (!Serial);     // will pause Zero, Leonardo, etc until serial console opens
#endif
  Serial.begin(9600);
  /* Initialise the sensor */
  if (!lsm.begin())
  {
    /* There was a problem detecting the LSM9DS0 ... check your connections */
    Serial.print(F("Ooops, no LSM9DS0 detected ... Check your wiring or I2C ADDR!"));
    while (1);
  }
  Serial.println(F("Found LSM9DS0 9DOF"));
  configureSensor();

}

// GLOBAL STATE STUFF ------------------------------------------------------

uint32_t lastImageTime = 0L; // Time of last image change
#ifdef MOTION_PIN
uint32_t prev          = 0L; // Used for sleep timing
#endif
// change image number here fastfind
uint8_t  imageNumber   = 0,  // Current image being displayed
         imageType,          // Image type: PALETTE[1,4,8] or TRUECOLOR
         *imagePalette,       // -> palette data in PROGMEM
         *imagePixels,        // -> pixel data in PROGMEM
         palette[16][3];     // RAM-based color table for 1- or 4-bit images
int8_t   imageLines,         // Number of lines in active image
         imageLine;          // Current line number in image
#ifdef SELECT_PIN
uint8_t  debounce      = 0;  // Debounce counter for image select pin
#endif

void imageInit() { // Initialize global image state for current imageNumber
  imageType    = pgm_read_byte(&images[imageNumber].type);

  imageLines   = pgm_read_word(&images[imageNumber].lines);

  imageLine    = 0;
  imagePalette = (uint8_t *)pgm_read_word(&images[imageNumber].palette);
  imagePixels  = (uint8_t *)pgm_read_word(&images[imageNumber].pixels);

  // 1- and 4-bit images have their color palette loaded into RAM both for
  // faster access and to allow dynamic color changing.  Not done w/8-bit
  // because that would require inordinate RAM (328P could handle it, but
  // I'd rather keep the RAM free for other features in the future).
  if (imageType == PALETTE1)      memcpy_P(palette, imagePalette,  2 * 3);
  else if (imageType == PALETTE4) memcpy_P(palette, imagePalette, 16 * 3);
  lastImageTime = millis(); // Save time of image init for next auto-cycle
}

void nextImage(void) {
  if (++imageNumber >= NUM_IMAGES) imageNumber = 0;
  imageInit();
}

float lastGyro = 0;
float alpha = 0.8;
bool start = false;
int8_t cnt = 0;

// MAIN LOOP ---------------------------------------------------------------

void loop() {
  sensors_event_t accel, mag, gyro, temp;
  lsm.getEvent(&accel, &mag, &gyro, &temp);

  if (Serial.available() > 0) {
    imageNumber = int (Serial.read()) - 48;
//    Serial.println(imageNumber);
    imageInit();
  }
  //Serial.println(brightness);
  lastGyro = lastGyro * alpha + (1-alpha)*gyro.gyro.z;
  float gz = gyro.gyro.z - lastGyro;
  
//  Serial.println(gz);

//  Serial.println(imageLine);
  if (fabs(gz) < 1){
    cnt ++;
    if (cnt > 10){
      start = false;
      Serial.println("stopped!");
      cnt = 0;
    }
  } else cnt = 0;
  
  if (gz > 1  && flag == 0) {
    start = true;
    imageLine = 0;
    flag = 1;
    Serial.println("!!!!");
  }
  if (gz < -1 && flag == 1) {
    start = true;
    Serial.println('?');
    imageLine = imageLine-2 >= 0 ? imageLine-2 : imageLines-1;
    flag = 0;
  }
  else {
    strip.clear();
  }
//  Serial.print("after");
  Serial.println(imageLine);
  uint32_t t = millis();               // Current time, milliseconds
  
  if (autoCycle) {
    if ((t - lastImageTime) >= (CYCLE_TIME * 1000L)) nextImage();
    // CPU clocks vary slightly; multiple poi won't stay in perfect sync.
    // Keep this in mind when using auto-cycle mode, you may want to cull
    // the image selection to avoid unintentional regrettable combinations.
  }
/*
  switch (imageNumber) {
    case 0:
      brightness = 10;
      strip.setBrightness(10);
      break;
    case 15:
      if (ct % frameSkip == (frameSkip - 1))
        brightness ++;
      strip.setBrightness(brightness);
      break;
    case 16:
      if (ct % frameSkip == (frameSkip - 1))
        brightness -= 1;
      strip.setBrightness(brightness);
      break;
    case 18:
      if (ct % frameSkip == (frameSkip - 1))
        brightness ++;
      strip.setBrightness(brightness);
      break;
    case 19:
      if (ct % frameSkip == (frameSkip - 1))
        brightness -= 1;
      strip.setBrightness(brightness);
      break;
    case 20:
      if (ct % frameSkip == (frameSkip - 1))
        brightness ++;
        if(brightness >40)
          brightness == 40;
      strip.setBrightness(brightness);
      break;
    case 14:
      brightness = 0;
      strip.setBrightness(brightness);
      break;
  }
  */
  switch (imageType) {

    case PALETTE1: { // 1-bit (2 color) palette-based image
        uint8_t  pixelNum = 0, byteNum, bitNum, pixels, idx,
                 *ptr = (uint8_t *)&imagePixels[imageLine * NUM_LEDS / 8];
        for (byteNum = NUM_LEDS / 8; byteNum--; ) { // Always padded to next byte, each byte has 8 pixels
          pixels = pgm_read_byte(ptr++);  // 8 pixels of data (pixel 0 = LSB)
          for (bitNum = 8; bitNum--; pixels >>= 1) {
            idx = pixels & 1; // Color table index for pixel (0 or 1)
            strip.setPixelColor(pixelNum++,
                                palette[idx][0], palette[idx][1], palette[idx][2]);
          }
        }

        break;
      }

    case PALETTE4: { // 4-bit (16 color) palette-based image
        uint8_t  pixelNum, p1, p2,
                 *ptr = (uint8_t *)&imagePixels[imageLine * NUM_LEDS / 2];

        for (pixelNum = 0; pixelNum < NUM_LEDS; ) {
          p2  = pgm_read_byte(ptr++); // Data for two pixels...
          p1  = p2 >> 4;              // Shift down 4 bits for first pixel
          p2 &= 0x0F;                 // Mask out low 4 bits for second pixel
          if (imageNumber != 20) {
            strip.setPixelColor(pixelNum++,
                                palette[p1][0], palette[p1][1], palette[p1][2]);
            strip.setPixelColor(pixelNum++,
                                palette[p2][0], palette[p2][1], palette[p2][2]);
          } else {
            strip.setPixelColor(pixelNum++,
                                rand255(), rand255(), rand255());
            strip.setPixelColor(pixelNum++,
                                rand255(), rand255(), rand255());
          }
        }

        break;
      }

#if 0 // Yep, demo images need ALL THE SPACE (see comment above)
    case PALETTE8: { // 8-bit (256 color) PROGMEM-palette-based image
        uint16_t  o;
        uint8_t   pixelNum,
                  *ptr = (uint8_t *)&imagePixels[imageLine * NUM_LEDS];
        for (pixelNum = 0; pixelNum < NUM_LEDS; pixelNum++) {
          o = pgm_read_byte(ptr++) * 3; // Offset into imagePalette
          strip.setPixelColor(pixelNum,
                              pgm_read_byte(&imagePalette[o]),
                              pgm_read_byte(&imagePalette[o + 1]),
                              pgm_read_byte(&imagePalette[o + 2]));
        }
        break;
      }

    case TRUECOLOR: { // 24-bit ('truecolor') image (no palette)
        uint8_t  pixelNum, r, g, b,
                 *ptr = (uint8_t *)&imagePixels[imageLine * NUM_LEDS * 3];
        for (pixelNum = 0; pixelNum < NUM_LEDS; pixelNum++) {
          r = pgm_read_byte(ptr++);
          g = pgm_read_byte(ptr++);
          b = pgm_read_byte(ptr++);
          strip.setPixelColor(pixelNum, r, g, b);
        }
        break;
      }
#endif
  }

  strip.show(); // Refresh LEDs

  ct++; // time progress

#if !defined(LED_DATA_PIN) && !defined(LED_CLOCK_PIN)
  delayMicroseconds(900);  // Because hardware SPI is ludicrously fast
#endif
    
  if (start && flag == 1) {
    imageLine++;
    if (imageLine >= imageLines) imageLine = 0; // Next scanline, wrap around
    flag = 1;
  }
  else if (start && flag == 0) {
    imageLine--;
    if (imageLine <= -1) imageLine = imageLines-1;
    flag = 0;
  }
  /*else {
    Serial.println("oh not change");
  }*/
}


void configureSensor(void)
{
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_6G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_16G);

  // 2.) Set the magnetometer sensitivity
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_12GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_2000DPS);
}

int rand255() {
  return (int)random(255);
}
