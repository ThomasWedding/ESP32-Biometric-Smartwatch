#include <Arduino.h>
#include <SPI.h>
#include <ICM45605.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <Adafruit_MAX1704X.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_TestBed.h>
#include <Fonts/Org_01.h>
#include <lvgl.h>
#include <time.h>

// ESP32-S3 SPI Pins
// SPI2
#define HSPI_SS 10
#define HSPI_MOSI 11
#define HSPI_SCLK 12
#define HSPI_MISO 13

// SPI3 (Used by display)
#define VSPI_SS 39
#define VSPI_MOSI 35
#define VSPI_SCLK 36
#define VSPI_MISO 37

extern Adafruit_TestBed TB;
Adafruit_MAX17048 lipo;
Adafruit_ST7789 display = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
GFXcanvas16 canvas(240, 135);

bool valid_i2c[128];                          // Array for I2C Scan
bool currentScreen[] = {true, false, false};  // Array for determining currently shown screen
int currentPosition = 0;                      // Determines current screen being shown

int j = 0;
bool valid_spi_devices[2];
int spi_device_count = 0;
int IMUresult = 1;

SPIClass *vAccel = new SPIClass(HSPI);
ICM456xx IMU( *vAccel, HSPI_SS );
inv_imu_sensor_data_t imu_data;

// Used for debouncing
int last_pressed = 0;
const int debounce_ms = 250;
volatile bool button_event = false;

void IRAM_ATTR updateScreen()
{
  button_event = true;
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
  canvas.setTextColor(ST77XX_WHITE);

  if (!lipo.begin()) {
    Serial.println(F("Couldnt find Adafruit MAX17048?\nMake sure a battery is plugged in!"));
    while (1) delay(10);
  }
    
  Serial.print(F("Found MAX17048"));
  Serial.print(F(" with Chip ID: 0x"));
  Serial.println(lipo.getChipID(), HEX);

  vAccel->begin(HSPI_SCLK, HSPI_MISO, HSPI_MOSI, HSPI_SS);
  pinMode(vAccel->pinSS(), OUTPUT);
  IMUresult = IMU.begin();

  pinMode(0, INPUT_PULLUP);
  pinMode(1, INPUT_PULLDOWN);
  pinMode(2, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(0), updateScreen, FALLING);
  attachInterrupt(digitalPinToInterrupt(1), updateScreen, RISING);
  attachInterrupt(digitalPinToInterrupt(2), updateScreen, RISING);

  //IMU.startAccel(100, 16);
  //IMU.startGyro(100, 2000);
  Serial.println("SETUP DONE");
}

// Function to scan SPI devices
void scanSPIDevices() 
{
  Serial.print("SPI scan: ");
  spi_device_count = 0;

  // Report known SPI devices on this board
  Serial.print("ST7789 TFT (CS:");
  Serial.print(TFT_CS);
  Serial.print(")");
  valid_spi_devices[0] = true;
  spi_device_count++;

  if( !IMUresult )
  {
    Serial.print( ", ICM45605" );
    valid_spi_devices[1] = true;
    spi_device_count++;
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
    //IMU.getDataFromRegisters(imu_data);
  }

  if (j % 2 == 0)
  {
    canvas.fillScreen(ST77XX_BLACK);

    if(currentScreen[0] == true)
    {
      canvas.setTextColor(ST77XX_BLUE);
      canvas.setCursor(0, 0);
      canvas.print("Battery");
      canvas.setTextColor(ST77XX_WHITE);
      canvas.setCursor(0, 67);
      canvas.print("I2C Data");
      canvas.setCursor(0, 125);
      canvas.print("SPI Data");
      canvas.setCursor(120, 0);
      canvas.setTextColor(ST77XX_GREEN); 
      canvas.print("Battery: ");
      canvas.setTextColor(ST77XX_WHITE);
      canvas.print(lipo.cellVoltage(), 1);
      canvas.print("V / ");
      canvas.print(lipo.cellPercent(), 0);
      canvas.println("%");
      canvas.setCursor(120, 8);
      canvas.setTextColor(ST77XX_CYAN);
      canvas.print("Buttons: ");
      if (!digitalRead(0)) 
      {
        canvas.setTextColor(ST77XX_RED);
        canvas.print("D0, ");
      }
      if (digitalRead(1)) 
      {
        canvas.setTextColor(ST77XX_GREEN);
        canvas.print("D1, ");
      }
      if (digitalRead(2)) 
      {
        canvas.setTextColor(ST77XX_BLUE);
        canvas.print("D2");
      }
      // Serial.println("Screen #1!");
    }
    else if(currentScreen[1] == true)
    {
      canvas.setTextColor(ST77XX_BLUE);
      canvas.setCursor(0, 67);
      canvas.print("I2C Data");
      canvas.setTextColor(ST77XX_WHITE);
      canvas.setCursor(0, 0);
      canvas.print("Battery");
      canvas.setCursor(0, 125);
      canvas.print("SPI Data");
      canvas.setCursor(120, 0);
      canvas.setTextColor(ST77XX_ORANGE);
      canvas.print("I2C: ");
      canvas.setTextColor(ST77XX_WHITE);
      for (uint8_t a=0x01; a<=0x7F; a++) 
      {
        if (valid_i2c[a])  
        {
          canvas.print("0x");
          canvas.print(a, HEX);
          canvas.print(", ");
        }
      }
      canvas.println("");
      canvas.setCursor(120, 8);
      canvas.print("SPI Test: ");
      canvas.setTextColor(ST77XX_WHITE);
      vAccel->beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
      digitalWrite(vAccel->pinSS(), LOW);
      canvas.println(vAccel->transfer(0b11110000));
      digitalWrite(vAccel->pinSS(), HIGH);
      vAccel->endTransaction();
      //Serial.println("Screen #2!");
    }
    else if(currentScreen[2] == true)
    {
      canvas.setTextColor(ST77XX_BLUE);
      canvas.setCursor(0, 125);
      canvas.print("SPI Data");
      canvas.setTextColor(ST77XX_WHITE);
      canvas.setCursor(0, 0);
      canvas.print("Battery");
      canvas.setCursor(0, 65);
      canvas.print("I2C Data");
      canvas.setCursor(120, 0);
      canvas.setTextColor(ST77XX_MAGENTA);
      canvas.print("SPI: ");
      canvas.setTextColor(ST77XX_WHITE);
      if( valid_spi_devices[0] )
        canvas.println("ST7789 TFT");
      canvas.setCursor(120, 8);
      if( valid_spi_devices[1] )
        canvas.println(", ICM45605");
      canvas.setCursor(120, 8*2);
      canvas.setTextColor(ST77XX_RED);
      canvas.print("AccelX: ");
      canvas.setTextColor(ST77XX_WHITE);
      canvas.println(imu_data.accel_data[0]);
      canvas.setCursor(120, 8*3);
      canvas.setTextColor(ST77XX_RED);
      canvas.print("AccelY: ");
      canvas.setTextColor(ST77XX_WHITE);
      canvas.println(imu_data.accel_data[1]);
      canvas.setCursor(120, 8*4);
      canvas.setTextColor(ST77XX_RED);
      canvas.print("AccelZ: ");
      canvas.setTextColor(ST77XX_WHITE);
      canvas.println(imu_data.accel_data[2]);
      canvas.setCursor(120, 8*5);
      canvas.setTextColor(ST77XX_RED);
      canvas.print("GyroX: ");
      canvas.setTextColor(ST77XX_WHITE);
      canvas.println(imu_data.gyro_data[0]);
      canvas.setCursor(120, 8*6);
      canvas.setTextColor(ST77XX_RED);
      canvas.print("GyroY: ");
      canvas.setTextColor(ST77XX_WHITE);
      canvas.println(imu_data.gyro_data[1]);
      canvas.setCursor(120, 8*7);
      canvas.setTextColor(ST77XX_RED);
      canvas.print("GyroZ: ");
      canvas.setTextColor(ST77XX_WHITE);
      canvas.println(imu_data.gyro_data[2]);
      canvas.setCursor(120, 8*8);
      canvas.setTextColor(ST77XX_RED);
      canvas.print("Temperature: ");
      canvas.setTextColor(ST77XX_WHITE);
      canvas.println(imu_data.temp_data);
      //Serial.println("Screen #3!");
    }
    else
    {
      Serial.println("Something is broken!");
      //Serial.println(currentPosition);
    }
      
    display.drawRGBBitmap(0, 0, canvas.getBuffer(), 240, 135);
    pinMode(TFT_BACKLITE, OUTPUT);
    digitalWrite(TFT_BACKLITE, HIGH);
  }
  
  // Software debouncer, for when switching screens
  if (button_event)
  {
    button_event = false;

    int now = millis();
    if (now - last_pressed > debounce_ms)
    {
      last_pressed = now;
      
      if (!digitalRead(0)) 
      {
        currentPosition = 0;
      }
      else if (digitalRead(1)) 
      {
        currentPosition = 1;
      }
      else 
      {
        currentPosition = 2;
      }
      for (int index = 0; index < 3; ++index)
      {
        currentScreen[index] = (index == currentPosition);
      }
    }
  }

  TB.setColor(TB.Wheel(j++));
  delay(10);
  return;
}