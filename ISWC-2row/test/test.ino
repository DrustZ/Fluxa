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

#include "test.h" // Graphics data is contained in this header file.

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

// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
#define GyroMeasError PI * (40.0f / 180.0f)       // gyroscope measurement error in rads/s (shown as 3 deg/s)
#define GyroMeasDrift PI * (0.0f / 180.0f)      // gyroscope measurement drift in rad/s/s (shown as 0.0 deg/s/s)
#define beta sqrt(3.0f / 4.0f) * GyroMeasError   // compute beta
#define zeta sqrt(3.0f / 4.0f) * GyroMeasDrift   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
float pitch, yaw, roll, heading;
float deltat = 0.0f;        // integration interval for both filter schemes
uint16_t lastUpdate = 0;    // used to calculate integration interval
uint16_t now = 0;           // used to calculate integration interval
float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values 
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
float ix, iy, iz, iw, vx, vy, vz;
float interval = 0;
uint8_t cnt,timeelapse = 0;
float lastx[3] = {0,0,0};
float lasty[3] = {0,0,0};
float lastz[3] = {0,0,0};
float gs[3] = {0,0,0};
float alpha = 0.8;

float constantV = 0.8;
uint8_t frameSkip = 0; 

int brightness = 0;
// int frameSkip = 5;
int ct = 0;
void setup() {

  strip.begin();                // Allocate DotStar buffer, init SPI
  strip.clear();                // Make sure strip is clear
  strip.show();                 // before measuring battery
  strip.setBrightness(50);
  brightness = 50;
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
line_t   imageLines,         // Number of lines in active image
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

void transform(float x, float y, float z, float w, int Step){
  float qx,qy,qz,qw;
  if (Step == 0){
    qx = q[1], qy = q[2], qz=q[3], qw = q[0];
    ix = qy*z - qz*y + qw*x + w*qx;
    iy = qz*x - qx*z + qw*y + w*qy;
    iz = qx*y - qy*x + qw*z + w*qz;
    iw = qw*w - qx*x - qy*y - qz*z;
  } else {
    float norm = sqrt(q[1]*q[1]+q[2]*q[2]+q[3]*q[3]+q[0]*q[0]);
    qx = -q[1]/norm, qy = -q[2]/norm, qz=-q[3]/norm, qw = q[0]/norm;
    
    ax = y*qz - z*qy + w*qx + qw*x;
    ay = z*qx - x*qz + qw*y + w*qy;
    az = x*qy - y*qx + qw*z + w*qz;
  }
}

void calculateVelocity(){
  transform(ax, ay, az, 0, 0); 
  transform(ix, iy, iz, iw, 1);
  
  gs[0] = gs[0] * alpha + (1-alpha)*ax;
  gs[1] = gs[1] * alpha + (1-alpha)*ay;
  gs[2] = gs[2] * alpha + (1-alpha)*az;
  ax -= gs[0], ay -= gs[1], az -= gs[2];

  if (abs(ax) < 0.5) ax = 0;
  if (abs(ay) < 0.5) ay = 0;
  if (abs(az) < 0.5) az = 0;

  if (ax == 0 && ay ==0 && az == 0)
    timeelapse += 1;
  else timeelapse = 0;
  if (timeelapse >= 4){
    timeelapse = 0;
    vx = vy = vz = 0;
  }
  
  lastx[cnt] = ax;
  lasty[cnt] = ay;
  lastz[cnt] = az;
  
  cnt++;
  interval += deltat;

  if (cnt < 3){
    return;
  } else {
    vx += (lastx[0] + lastx[1]*4 + lastx[2]) / 6 * interval;
    vy += (lasty[0] + lasty[1]*4 + lasty[2]) / 6 * interval;
    vz += (lastz[0] + lastz[1]*4 + lastz[2]) / 6 * interval;
    interval = 0;
    cnt = 0;
  }
}

void assignLights(int imageLine, int mul){
  uint8_t r,g,b;
   switch (imageType) {
    case PALETTE1: { // 1-bit (2 color) palette-based image
        uint8_t  pixelNum = 0, byteNum, bitNum, pixels, idx,
                 *ptr = (uint8_t *)&imagePixels[imageLine * NUM_LEDS / 8];
        for (byteNum = NUM_LEDS / 8; byteNum--; ) { // Always padded to next byte, each byte has 8 pixels
          pixels = pgm_read_byte(ptr++);  // 8 pixels of data (pixel 0 = LSB)
          for (bitNum = 8; bitNum--; pixels >>= 1) {
            idx = pixels & 1; // Color table index for pixel (0 or 1)
            r = palette[idx][0], g = palette[idx][1], b = palette[idx][2];
            r *= mul, g *= mul, b *= mul;
            strip.setPixelColor(pixelNum++,
                                r, g, b);
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
            r = palette[p1][0], g = palette[p1][1], b = palette[p1][2];
            r *= mul, g *= mul, b *= mul;
            strip.setPixelColor(pixelNum++,
                                r, g, b);
            r = palette[p2][0], g = palette[p2][1], b = palette[p2][2];
            r *= mul, g *= mul, b *= mul;
            strip.setPixelColor(pixelNum++,
                                r, g, b);
          } else {
            strip.setPixelColor(pixelNum++,
                                rand255(), rand255(), rand255());
            strip.setPixelColor(pixelNum++,
                                rand255(), rand255(), rand255());
          }
        }

        break;
      }
  }
  strip.show(); // Refresh LEDs
}

// MAIN LOOP ---------------------------------------------------------------

void loop() {
  sensors_event_t accel, mag, gyro, temp;
  lsm.getEvent(&accel, &mag, &gyro, &temp);

  now = micros();
  deltat = ((now - lastUpdate)/1000000.0f); // set integration time by time elapsed since last filter update
  lastUpdate = now;
  
  ax = accel.acceleration.x; ay = accel.acceleration.y; az = accel.acceleration.z;
  gx = gyro.gyro.x; gy = gyro.gyro.y; gz = gyro.gyro.z;
  mx = mag.magnetic.x; my = mag.magnetic.y; mz = mag.magnetic.z; 
  MadgwickQuaternionUpdate(ax/9.8, ay/9.8, az/9.8, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, mx, my, mz);
  calculateVelocity();
  float v = sqrt(vx*vx+vy*vy+vz*vz);

//  Serial.print(v);Serial.print('\n');
  
  if (Serial.available() > 0) {
    imageNumber = int (Serial.read()) - 48;
//    Serial.println(imageNumber);
    imageInit();
  }
  //Serial.println(brightness);
//  Serial.println(gyro.gyro.z);
  if (gyro.gyro.z > 25 && flag == 0) {
    imageLine = 0;
    flag = 1;
    frameSkip = 0;
  }
  if (gyro.gyro.z < -25 && flag == 1) {
//    imageLine = imageLines-1;
    frameSkip = 0;
    flag = 0;
  }

  else {
    strip.clear();
  }
  uint32_t t = millis();               // Current time, milliseconds
  
  if (autoCycle) {
    if ((t - lastImageTime) >= (CYCLE_TIME * 1000L)) nextImage();
    // CPU clocks vary slightly; multiple poi won't stay in perfect sync.
    // Keep this in mind when using auto-cycle mode, you may want to cull
    // the image selection to avoid unintentional regrettable combinations.
  }
  
  if (ct){
    assignLights(imageLine-1, 0.5);
    ct = 0;
  }
  if (v/constantV > 1.3){
    uint8_t tmpl = imageLine+1 >= imageLines ? imageLines-1 : imageLine+1;
    assignLights(tmpl, 0.5);
    assignLights(imageLine, 1);
    imageLine = tmpl;
    ct = 1;
    Serial.println("yes");
  } else {
    assignLights(imageLine, 1);
    ct = 0;
  }
  // ct++; // time progress

#if !defined(LED_DATA_PIN) && !defined(LED_CLOCK_PIN)
  delayMicroseconds(900);  // Because hardware SPI is ludicrously fast
#endif

  if (frameSkip <= 0){
    frameSkip = (int)(constantV/v);
    Serial.print("Skip:");Serial.println(frameSkip);
  }
  else {
    frameSkip--;
    return;
  }

  if (gyro.gyro.z > 25 && flag == 1) {
    imageLine++;
    if (imageLine == imageLines) imageLine = imageLines-1; // Next scanline, wrap around
  }
  if (gyro.gyro.z < -25 && flag == 0) {
    imageLine--;
    if (imageLine == 0) imageLine = 1;
  }
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

// Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
// (see http://www.x-io.co.uk/category/open-source/ for examples and more details)
// which fuses acceleration, rotation rate, and magnetic moments to produce a quaternion-based estimate of absolute
// device orientation -- which can be converted to yaw, pitch, and roll. Useful for stabilizing quadcopters, etc.
// The performance of the orientation filter is at least as good as conventional Kalman-based filtering algorithms
// but is much less computationally intensive---it can be performed on a 3.3 V Pro Mini operating at 8 MHz!
void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
{
  float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
  float norm;
  float hx, hy, _2bx, _2bz;
  float s1, s2, s3, s4;
  float qDot1, qDot2, qDot3, qDot4;

  // Auxiliary variables to avoid repeated arithmetic
  float _2q1mx;
  float _2q1my;
  float _2q1mz;
  float _2q2mx;
  float _4bx;
  float _4bz;
  float _2q1 = 2.0f * q1;
  float _2q2 = 2.0f * q2;
  float _2q3 = 2.0f * q3;
  float _2q4 = 2.0f * q4;
  float _2q1q3 = 2.0f * q1 * q3;
  float _2q3q4 = 2.0f * q3 * q4;
  float q1q1 = q1 * q1;
  float q1q2 = q1 * q2;
  float q1q3 = q1 * q3;
  float q1q4 = q1 * q4;
  float q2q2 = q2 * q2;
  float q2q3 = q2 * q3;
  float q2q4 = q2 * q4;
  float q3q3 = q3 * q3;
  float q3q4 = q3 * q4;
  float q4q4 = q4 * q4;

  // Normalise accelerometer measurement
  norm = sqrt(ax * ax + ay * ay + az * az);
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f/norm;
  ax *= norm;
  ay *= norm;
  az *= norm;

  // Normalise magnetometer measurement
  norm = sqrt(mx * mx + my * my + mz * mz);
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f/norm;
  mx *= norm;
  my *= norm;
  mz *= norm;

  // Reference direction of Earth's magnetic field
  _2q1mx = 2.0f * q1 * mx;
  _2q1my = 2.0f * q1 * my;
  _2q1mz = 2.0f * q1 * mz;
  _2q2mx = 2.0f * q2 * mx;
  hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
  hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
  _2bx = sqrt(hx * hx + hy * hy);
  _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
  _4bx = 2.0f * _2bx;
  _4bz = 2.0f * _2bz;

  // Gradient decent algorithm corrective step
  s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
  norm = 1.0f/norm;
  s1 *= norm;
  s2 *= norm;
  s3 *= norm;
  s4 *= norm;

  // Compute rate of change of quaternion
  qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
  qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
  qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
  qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

  // Integrate to yield quaternion
  q1 += qDot1 * deltat;
  q2 += qDot2 * deltat;
  q3 += qDot3 * deltat;
  q4 += qDot4 * deltat;
  norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
  norm = 1.0f/norm;
  q[0] = q1 * norm;
  q[1] = q2 * norm;
  q[2] = q3 * norm;
  q[3] = q4 * norm;
}
