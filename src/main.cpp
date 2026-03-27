#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <Adafruit_MAX1704X.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_TestBed.h>
#include <Fonts/Org_01.h>
#include <lvgl.h>
#include <time.h>

// Old IMU
// #include <ICM45605.h>
// New IMU
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_Sensor.h>

// ESP32-S3 SPI Pins
// SPI2
//#define HSPI_SS 10
//#define HSPI_MOSI 11
//#define HSPI_SCLK 12
//#define HSPI_MISO 13

// SPI3
//#define VSPI_SS 42
//#define VSPI_MOSI 35
//#define VSPI_SCLK 36
//#define VSPI_MISO 37
//#define I2Caddress 1101000

// ICM456xx IMU — I2C settings (tune here)
#define IMU_I2C_ADDR_LSB true    // false → 0x68, true → 0x69
#define IMU_I2C_FREQ     100000  // Bus clock in Hz (100000–1000000)

extern Adafruit_TestBed TB;
Adafruit_MAX17048 lipo;
Adafruit_ST7789 display = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);     // Global variable

// For LVGL
static lv_disp_draw_buf_t draw_buf;         // LVGL draw buffer descriptor
static lv_color_t draw_buffer[240 * 20];    // Actual memory LVGL will draw into

// Different types of screens
static lv_obj_t *battery_screen;
static lv_obj_t *i2c_screen;
static lv_obj_t *spi_screen;

// Battery Screen
static lv_obj_t *title_label;
static lv_obj_t *buttons_label;

// i2c screen
static lv_obj_t *i2c_title_label;
static lv_obj_t *i2c_battery_nav_label;
static lv_obj_t *i2c_spi_nav_label;
static lv_obj_t *i2c_data_label;
static lv_obj_t *spi_test_label;

// spi screen
static lv_obj_t *spi_title_label;
static lv_obj_t *spi_battery_nav_label;
static lv_obj_t *spi_i2c_nav_label;
static lv_obj_t *spi_devices_label;
static lv_obj_t *imu_label;

// Menu options
static lv_obj_t *battery_label;
static lv_obj_t *i2c_nav_label;
static lv_obj_t *spi_nav_label;

bool valid_i2c[128];                          // Array for I2C Scan
bool currentScreen[] = {true, false, false};  // Array for determining currently shown screen
int currentPosition = 0;                      // Determines current screen being shown

int j = 0;
bool valid_spi_devices[2];
int spi_device_count = 0;

// OLD IMU
// int IMUresult = 1;
// ICM456xx IMU(Wire, IMU_I2C_ADDR_LSB);
// inv_imu_sensor_data_t imu_data;
// New IMU
Adafruit_LSM6DS lsm6dso;
bool imu_ok = false;

sensors_event_t accel;
sensors_event_t gyro;
sensors_event_t temp;

// Used for debouncing
int last_pressed = 0;
const int debounce_ms = 250;
volatile bool button_event = false;

void IRAM_ATTR updateScreen()
{
  button_event = true;
}

// Sends the pixels to the display for LVGL, updates only region instead of the whole canvas
static void my_flush_cb(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p)
{
  uint32_t w = area->x2 - area->x1 + 1;
  uint32_t h = area->y2 - area->y1 + 1;

  display.startWrite();
  display.setAddrWindow(area->x1, area->y1, w, h);
  display.writePixels((uint16_t *) &color_p->full, w*h, true);
  display.endWrite();

  lv_disp_flush_ready(disp_drv);
}

void create_battery_screen()
{
  battery_screen = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(battery_screen, lv_color_black(), 0);
  lv_obj_set_style_bg_opa(battery_screen, LV_OPA_COVER, 0);
  lv_obj_set_style_text_color(battery_screen, lv_color_white(), 0);

  title_label = lv_label_create(battery_screen);
  lv_label_set_text(title_label, "Battery");
  lv_obj_set_style_text_color(title_label, lv_color_hex(0x097ad6), 0);
  lv_obj_align(title_label, LV_ALIGN_TOP_LEFT, 5, 5);

  i2c_nav_label = lv_label_create(battery_screen);
  lv_label_set_text(i2c_nav_label, "I2C Data");
  lv_obj_align(i2c_nav_label, LV_ALIGN_LEFT_MID, 5, 0);

  spi_nav_label = lv_label_create(battery_screen);
  lv_label_set_text(spi_nav_label, "SPI Data");
  lv_obj_align(spi_nav_label, LV_ALIGN_BOTTOM_LEFT, 5, -5);

  battery_label = lv_label_create(battery_screen);
  lv_label_set_text(battery_label, "Battery: --.-V / --%");
  lv_obj_align(battery_label, LV_ALIGN_TOP_LEFT, 120, 5);
  
  buttons_label = lv_label_create(battery_screen);
  lv_label_set_text(buttons_label, "Buttons: ");
  lv_obj_align(buttons_label, LV_ALIGN_TOP_LEFT, 120, 25);
}

void update_battery_screen()
{
  static char batt_buf[64];
  static char btn_buf[64];

  snprintf(batt_buf, sizeof(batt_buf), "Battery: %.1fV / %.0f%%",
            lipo.cellVoltage(), lipo.cellPercent());
  lv_label_set_text(battery_label, batt_buf);

  snprintf(btn_buf, sizeof(btn_buf), "Buttons: %s%s%s",
           !digitalRead(0) ? "D0 " : "",
           digitalRead(1) ? "D1 " : "",
           digitalRead(2) ? "D2" : "");
  lv_label_set_text(buttons_label, btn_buf);
}

void create_i2c_screen()
{
  i2c_screen = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(i2c_screen, lv_color_black(), 0);
  lv_obj_set_style_bg_opa(i2c_screen, LV_OPA_COVER, 0);
  lv_obj_set_style_text_color(i2c_screen, lv_color_white(), 0);

  i2c_title_label = lv_label_create(i2c_screen);
  lv_label_set_text(i2c_title_label, "I2C Data");
  lv_obj_set_style_text_color(i2c_title_label, lv_color_hex(0x097ad6), 0);
  lv_obj_align(i2c_title_label, LV_ALIGN_LEFT_MID, 5, 0);

  i2c_battery_nav_label = lv_label_create(i2c_screen);
  lv_label_set_text(i2c_battery_nav_label, "Battery");
  lv_obj_align(i2c_battery_nav_label, LV_ALIGN_TOP_LEFT, 5, 5);

  i2c_spi_nav_label = lv_label_create(i2c_screen);
  lv_label_set_text(i2c_spi_nav_label, "SPI Data");
  lv_obj_align(i2c_spi_nav_label, LV_ALIGN_BOTTOM_LEFT, 5, -5);

  i2c_data_label = lv_label_create(i2c_screen);
  lv_label_set_text(i2c_data_label, "I2C: ");
  lv_obj_set_width(i2c_data_label, 135);
  lv_label_set_long_mode(i2c_data_label, LV_LABEL_LONG_WRAP);
  lv_obj_align(i2c_data_label, LV_ALIGN_TOP_LEFT, 95, 5);

  spi_test_label = lv_label_create(i2c_screen);
  lv_label_set_text(spi_test_label, "SPI Test: ");
  lv_obj_set_width(spi_test_label, 135);
  lv_label_set_long_mode(spi_test_label, LV_LABEL_LONG_WRAP);
  lv_obj_align(spi_test_label, LV_ALIGN_TOP_LEFT, 95, 45);
}

void update_i2c_screen()
{
  static char i2c_buf[256];
  static char spi_buf[64];

  strcpy(i2c_buf, "I2C: ");
  for (uint8_t a = 0x01; a <= 0x7F; a++)
  {
    if (valid_i2c[a])
    {
      char temp[8];
      snprintf(temp, sizeof(temp), "0x%02X ", a);
      strncat(i2c_buf, temp, sizeof(i2c_buf) - strlen(i2c_buf) - 1);
    }
  }
  lv_label_set_text(i2c_data_label, i2c_buf);
  lv_label_set_text(spi_test_label, "LSM6DSO uses I2C");
}

void create_spi_screen()
{
  spi_screen = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(spi_screen, lv_color_black(), 0);
  lv_obj_set_style_bg_opa(spi_screen, LV_OPA_COVER, 0);
  lv_obj_set_style_text_color(spi_screen, lv_color_white(), 0);
  lv_obj_set_style_text_font(spi_screen, &lv_font_montserrat_10, 0);

  spi_title_label = lv_label_create(spi_screen);
  lv_label_set_text(spi_title_label, "SPI Data");
  lv_obj_set_style_text_color(spi_title_label, lv_color_hex(0x097ad6), 0);
  lv_obj_align(spi_title_label, LV_ALIGN_BOTTOM_LEFT, 5, -5);

  spi_battery_nav_label = lv_label_create(spi_screen);
  lv_label_set_text(spi_battery_nav_label, "Battery");
  lv_obj_align(spi_battery_nav_label, LV_ALIGN_TOP_LEFT, 5, 5);

  spi_i2c_nav_label = lv_label_create(spi_screen);
  lv_label_set_text(spi_i2c_nav_label, "I2C Data");
  lv_obj_align(spi_i2c_nav_label, LV_ALIGN_LEFT_MID, 5, 0);

  spi_devices_label = lv_label_create(spi_screen);
  lv_label_set_text(spi_devices_label, "SPI: ");
  lv_obj_set_width(spi_devices_label, 135);
  lv_label_set_long_mode(spi_devices_label, LV_LABEL_LONG_WRAP);
  lv_obj_align(spi_devices_label, LV_ALIGN_TOP_LEFT, 95, 5);

  imu_label = lv_label_create(spi_screen);
  lv_label_set_text(imu_label, "AccelX:\nAccelY:\nAccelZ:\nGyroX:\nGyroY:\nGyroZ:\nTemp:");
  lv_obj_set_width(imu_label, 140);
  lv_label_set_long_mode(imu_label, LV_LABEL_LONG_CLIP);
  lv_obj_align(imu_label, LV_ALIGN_TOP_LEFT, 95, 28);
}

void update_spi_screen()
{
  static char spi_buf[128];
  static char imu_buf[256];

  strcpy(spi_buf, "SPI: ");
  if (valid_spi_devices[0]) strncat(spi_buf, "ST7789 TFT ", sizeof(spi_buf) - strlen(spi_buf) - 1);
  if (valid_spi_devices[1]) strncat(spi_buf, "LSM6DSO", sizeof(spi_buf) - strlen(spi_buf) - 1);
  lv_label_set_text(spi_devices_label, spi_buf);

  if (imu_ok)
  {
    lsm6dso.getEvent(&accel, &gyro, &temp);

    snprintf(
      imu_buf, sizeof(imu_buf),
      "AccelX: %.2f\n"
      "AccelY: %.2f\n"
      "AccelZ: %.2f\n"
      "GyroX:  %.2f\n"
      "GyroY:  %.2f\n"
      "GyroZ:  %.2f\n"
      "Temp:   %.2f",
      accel.acceleration.x,
      accel.acceleration.y,
      accel.acceleration.z,
      gyro.gyro.x,
      gyro.gyro.y,
      gyro.gyro.z,
      temp.temperature
    );
  }
  else
  {
    snprintf(imu_buf, sizeof(imu_buf), "LSM6DSO not detected");
  }

  lv_label_set_text(imu_label, imu_buf);
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(100);
  //delay(1000);
  
  TB.neopixelPin = PIN_NEOPIXEL;
  TB.neopixelNum = 1;
  TB.begin();
  TB.setColor(WHITE);

  display.init(135, 240);           // Init ST7789 240x135
  display.setRotation(3);

  // Turns on screen backlight
  pinMode(TFT_BACKLITE, OUTPUT);
  digitalWrite(TFT_BACKLITE, HIGH);

  // Turns on the display
  pinMode(TFT_I2C_POWER, OUTPUT);
  digitalWrite(TFT_I2C_POWER, HIGH);

  // Initializes and tells lvgl what memory it can draw into 
  lv_init();
  lv_disp_draw_buf_init(&draw_buf, draw_buffer, NULL, 240*20);

  // Settings for the LVGL display driver
  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = 240;
  disp_drv.ver_res = 135;
  disp_drv.flush_cb = my_flush_cb;
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register(&disp_drv);

  // Creates the screens for the watch
  create_battery_screen();
  create_i2c_screen();
  create_spi_screen();
  lv_scr_load(battery_screen);

  if (!lipo.begin()) {
   Serial.println(F("Couldnt find Adafruit MAX17048?\nMake sure a battery is plugged in!"));
   while (1) delay(10);
  }
    
  Serial.print(F("Found MAX17048"));
  Serial.print(F(" with Chip ID: 0x"));
  Serial.println(lipo.getChipID(), HEX);

  //pinMode(3, OUTPUT_OPEN_DRAIN);
  //pinMode(4, OUTPUT_OPEN_DRAIN);
  pinMode(0, INPUT_PULLUP);
  pinMode(1, INPUT_PULLDOWN);
  pinMode(2, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(0), updateScreen, FALLING);
  attachInterrupt(digitalPinToInterrupt(1), updateScreen, RISING);
  attachInterrupt(digitalPinToInterrupt(2), updateScreen, RISING);

  // New IMU set up, used to start up
  Wire.begin();
  Wire.setClock(IMU_I2C_FREQ);

  Serial.println("Starting LSM6DSO...");

  imu_ok = lsm6dso.begin_I2C();   // default I2C address chosen by board wiring / SA0

  if (!imu_ok) {
    Serial.println("LSM6DSO not found");
  } else {
    Serial.println("LSM6DSO found");

    lsm6dso.setAccelRange(LSM6DS_ACCEL_RANGE_16_G);
    lsm6dso.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);

    lsm6dso.setAccelDataRate(LSM6DS_RATE_104_HZ);
    lsm6dso.setGyroDataRate(LSM6DS_RATE_104_HZ);
  }
  Serial.println("SETUP DONE");
}

// Function to scan SPI devices
void scanSPIDevices()
{
  Serial.print("Devices: ");
  spi_device_count = 0;

  Serial.print("ST7789 TFT");
  valid_spi_devices[0] = true;
  spi_device_count++;

  if (imu_ok) {
    Serial.print(", LSM6DSO");
    valid_spi_devices[1] = true;
    spi_device_count++;
  } else {
    valid_spi_devices[1] = false;
  }

  Serial.println();
}

void scanI2CDevices( bool (&valid_i2c)[128] )
{
  Serial.print("I2C scan: ");
  for (int i=0; i< 128; i++)
    {
      if (TB.scanI2CBus(i, 0))
      {
        Serial.print("0x");
        Serial.print(i, HEX);
        Serial.print(", ");
        valid_i2c[i] = true;
      }
      else
        valid_i2c[i] = false;
    }
  Serial.println();
}


void loop() {
  if (j == 0)
  {
    scanI2CDevices(valid_i2c);
    scanSPIDevices();
    Serial.println("HAHA");
  }

  if (currentPosition == 0)
  {
    update_battery_screen();
  }
  else if (currentPosition == 1)
  {
    update_i2c_screen();
  }
  else if (currentPosition == 2)
  {
    update_spi_screen();
  }

  // Changes screens
  if (button_event)
  {
    button_event = false;

    int now = millis();
    if (now - last_pressed > debounce_ms)
    {
      last_pressed = now;

      if (!digitalRead(0)) {
        currentPosition = 0;
        lv_scr_load(battery_screen);
      }
      else if (digitalRead(1)) 
      {
        currentPosition = 1;
        lv_scr_load(i2c_screen);
      }
      else 
      {
        currentPosition = 2;
        lv_scr_load(spi_screen);
      }
    }
  }
  
  lv_timer_handler();
  TB.setColor(TB.Wheel(j++));
  delay(5);
  return;
}