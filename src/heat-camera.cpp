/***************************************************************************
 * This sketch is based on the Adafruit AMG88xx example
 * https://github.com/adafruit/Adafruit_AMG88xx/blob/master/examples/thermal_cam_interpolate/thermal_cam_interpolate.ino
 * Davy Landman has added a few functionality, but the license remains the same.
 * 
  This is a library for the AMG88xx GridEYE 8x8 IR camera
  This sketch makes an inetrpolated pixel thermal camera with the 
  GridEYE sensor and a 2.4" tft featherwing:
	 https://www.adafruit.com/product/3315
  Designed specifically to work with the Adafruit AMG8833 Featherwing
          https://www.adafruit.com/product/3622
  These sensors use I2C to communicate. The device's I2C address is 0x69
  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!
  Written by Dean Miller, James DeVito & ladyada for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/

#include <Arduino.h>

#include <TFT_eSPI.h>
#include <SPI.h>
#include <Adafruit_AMG88xx.h>

TFT_eSPI tft = TFT_eSPI();
Adafruit_AMG88xx amg;

/*
connectors:

heat cam:
SDA => D2
SCL => D1
GND => GND
VIN => 3.3V

screen:
blk => resistor(brown-black-brown) => vcc
cs => d8
dc => d3
res => d4
sda => d7
scl => d5
vcc => 3.3v
gnd => gnd
*/


//the colors we will be using
const uint16_t camColors[] = {0x480F,
0x400F,0x400F,0x400F,0x4010,0x3810,0x3810,0x3810,0x3810,0x3010,0x3010,
0x3010,0x2810,0x2810,0x2810,0x2810,0x2010,0x2010,0x2010,0x1810,0x1810,
0x1811,0x1811,0x1011,0x1011,0x1011,0x0811,0x0811,0x0811,0x0011,0x0011,
0x0011,0x0011,0x0011,0x0031,0x0031,0x0051,0x0072,0x0072,0x0092,0x00B2,
0x00B2,0x00D2,0x00F2,0x00F2,0x0112,0x0132,0x0152,0x0152,0x0172,0x0192,
0x0192,0x01B2,0x01D2,0x01F3,0x01F3,0x0213,0x0233,0x0253,0x0253,0x0273,
0x0293,0x02B3,0x02D3,0x02D3,0x02F3,0x0313,0x0333,0x0333,0x0353,0x0373,
0x0394,0x03B4,0x03D4,0x03D4,0x03F4,0x0414,0x0434,0x0454,0x0474,0x0474,
0x0494,0x04B4,0x04D4,0x04F4,0x0514,0x0534,0x0534,0x0554,0x0554,0x0574,
0x0574,0x0573,0x0573,0x0573,0x0572,0x0572,0x0572,0x0571,0x0591,0x0591,
0x0590,0x0590,0x058F,0x058F,0x058F,0x058E,0x05AE,0x05AE,0x05AD,0x05AD,
0x05AD,0x05AC,0x05AC,0x05AB,0x05CB,0x05CB,0x05CA,0x05CA,0x05CA,0x05C9,
0x05C9,0x05C8,0x05E8,0x05E8,0x05E7,0x05E7,0x05E6,0x05E6,0x05E6,0x05E5,
0x05E5,0x0604,0x0604,0x0604,0x0603,0x0603,0x0602,0x0602,0x0601,0x0621,
0x0621,0x0620,0x0620,0x0620,0x0620,0x0E20,0x0E20,0x0E40,0x1640,0x1640,
0x1E40,0x1E40,0x2640,0x2640,0x2E40,0x2E60,0x3660,0x3660,0x3E60,0x3E60,
0x3E60,0x4660,0x4660,0x4E60,0x4E80,0x5680,0x5680,0x5E80,0x5E80,0x6680,
0x6680,0x6E80,0x6EA0,0x76A0,0x76A0,0x7EA0,0x7EA0,0x86A0,0x86A0,0x8EA0,
0x8EC0,0x96C0,0x96C0,0x9EC0,0x9EC0,0xA6C0,0xAEC0,0xAEC0,0xB6E0,0xB6E0,
0xBEE0,0xBEE0,0xC6E0,0xC6E0,0xCEE0,0xCEE0,0xD6E0,0xD700,0xDF00,0xDEE0,
0xDEC0,0xDEA0,0xDE80,0xDE80,0xE660,0xE640,0xE620,0xE600,0xE5E0,0xE5C0,
0xE5A0,0xE580,0xE560,0xE540,0xE520,0xE500,0xE4E0,0xE4C0,0xE4A0,0xE480,
0xE460,0xEC40,0xEC20,0xEC00,0xEBE0,0xEBC0,0xEBA0,0xEB80,0xEB60,0xEB40,
0xEB20,0xEB00,0xEAE0,0xEAC0,0xEAA0,0xEA80,0xEA60,0xEA40,0xF220,0xF200,
0xF1E0,0xF1C0,0xF1A0,0xF180,0xF160,0xF140,0xF100,0xF0E0,0xF0C0,0xF0A0,
0xF080,0xF060,0xF040,0xF020,0xF800,};

unsigned long delayTime;

#define AMG_COLS 8
#define AMG_ROWS 8
float pixels[AMG_COLS * AMG_ROWS];

#define INTERPOLATED_COLS 24
#define INTERPOLATED_ROWS 24



float get_point(float *p, uint8_t rows, uint8_t cols, int8_t x, int8_t y);
void set_point(float *p, uint8_t rows, uint8_t cols, int8_t x, int8_t y, float f);
void get_adjacents_1d(float *src, float *dest, uint8_t rows, uint8_t cols, int8_t x, int8_t y);
void get_adjacents_2d(float *src, float *dest, uint8_t rows, uint8_t cols, int8_t x, int8_t y);
float cubicInterpolate(float p[], float x);
float bicubicInterpolate(float p[], float x, float y);
void interpolate_image(float *src, uint8_t src_rows, uint8_t src_cols, 
                       float *dest, uint8_t dest_rows, uint8_t dest_cols);


static uint16_t boxsize;
void setup() {
    Serial.begin(74880);
    tft.init();
    tft.setRotation(1);
    if (!amg.begin()) {
        Serial.println("Could not find a valid AMG88xx sensor, check wiring!");
        while (1);
    }
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_WHITE);
    tft.setTextSize(1);
    tft.setCursor(0,0);
    tft.print("Temp");
    delay(100); // give sensor time to start

    boxsize = min(tft.width() / INTERPOLATED_COLS, tft.height() / INTERPOLATED_COLS);
}

void drawpixels(float *p, uint8_t rows, uint8_t cols, uint8_t boxWidth, uint8_t boxHeight, float min, float max);


constexpr int SPATIAL_SMOOTHING_SIZE = 4;
static size_t frame_counter = 0;
static float previous_frames[SPATIAL_SMOOTHING_SIZE][INTERPOLATED_ROWS * INTERPOLATED_COLS];

unsigned long next_tick = 0;
constexpr unsigned long TICK_RATE = 200;

constexpr int TEMP_RANGE_SMOOTH = 10;
static size_t range_counter = 0;
static float minimal_temps[TEMP_RANGE_SMOOTH] = {10};
static float maximum_temps[TEMP_RANGE_SMOOTH] = {30};

void loop() {
  auto till_next_tick = next_tick -  millis();
  if (till_next_tick > 0 && till_next_tick < TICK_RATE) {
    Serial.printf("sleeping: %ul\n", till_next_tick);

    delay(till_next_tick);
  }
  next_tick += TICK_RATE;
  
  //read all the pixels
  amg.readPixels(pixels);

/*
  Serial.print("[");
  for(int i=1; i<=AMG88xx_PIXEL_ARRAY_SIZE; i++){
    Serial.print(pixels[i-1]);
    Serial.print(", ");
    if( i%8 == 0 ) Serial.println();
  }
  Serial.println("]");
  Serial.println();
  */


  unsigned long t = millis();
  interpolate_image(pixels, AMG_ROWS, AMG_COLS, previous_frames[(frame_counter++ % SPATIAL_SMOOTHING_SIZE)], INTERPOLATED_ROWS, INTERPOLATED_COLS);
  Serial.print("Interpolation: "); Serial.print(millis()-t); Serial.print(" ms");



  t = millis();
  float smoothed_frames[INTERPOLATED_ROWS * INTERPOLATED_COLS];
  for (size_t i = 0; i < INTERPOLATED_ROWS * INTERPOLATED_COLS; i++) {
    float sum = 0;
    for (int j = 0; j < SPATIAL_SMOOTHING_SIZE; j++) {
      sum += previous_frames[j][i];
    }
    smoothed_frames[i] = sum / SPATIAL_SMOOTHING_SIZE;
  }
  Serial.print("Smoothing: "); Serial.print(millis()-t); Serial.print(" ms");
  
  auto min = smoothed_frames[0];
  auto max = smoothed_frames[0];
  for (size_t i = 1; i < INTERPOLATED_ROWS * INTERPOLATED_COLS; i++) {
    if (smoothed_frames[i] < min) {
      min = smoothed_frames[i];
    }
    else if (smoothed_frames[i] > max) {
      max = smoothed_frames[i];
    }
  }

  minimal_temps[range_counter % TEMP_RANGE_SMOOTH] = min;
  maximum_temps[range_counter % TEMP_RANGE_SMOOTH] = max;
  range_counter++;

  for (int i = 0; i < TEMP_RANGE_SMOOTH; i++) {
    if (minimal_temps[i] < min) {
      min = minimal_temps[i];
    }
    if (maximum_temps[i] > max) {
      max = maximum_temps[i];
    }
  }

  
  t = millis();
  drawpixels(smoothed_frames, INTERPOLATED_ROWS, INTERPOLATED_COLS, boxsize, boxsize, min, max);
  Serial.print("Rendering: "); Serial.print(millis()-t); Serial.println(" ms");

  //delay(50);
}

void drawpixels(float *p, uint8_t rows, uint8_t cols, uint8_t boxWidth, uint8_t boxHeight, float min, float max) {
  min *= 1000;
  max *= 1000;
  for (int y=0; y<rows; y++) {
    for (int x=0; x<cols; x++) {
      float val = get_point(p, rows, cols, x, y);
      float colorTemp = val * 1000;
      
      uint8_t colorIndex = map(colorTemp, min, max, 0, 255);  // move decimal point of colorTemp 2 places to the right, otherwise colors are only mapped to full degrees and not points
      colorIndex = constrain(colorIndex, 0, 255);
      //draw the pixels!
      uint16_t color;
      color = val * 2;
      tft.fillRect((tft.width() - (boxWidth * cols))+boxWidth * x, boxHeight * y, boxWidth, boxHeight, camColors[colorIndex]);
    } 
  }
  tft.fillRect(0,0,tft.width() / 2, tft.height(), TFT_BLACK);
  tft.setCursor(0,0);
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(1);
  tft.printf("Min: %.1f\nMax: %.1f",min / 1000, max / 1000);
}