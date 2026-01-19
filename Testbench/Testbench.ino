// Modified version of demo program found here:
// https://github.com/adafruit/Adafruit_Learning_System_Guides/blob/main/Factory_Tests/Feather_ESP32S3_ReverseTFT_FactoryTest/Feather_ESP32S3_ReverseTFT_FactoryTest.ino
#include "declarations.h"

//#define SPI_MOSI 23
//#define SPI_MISO 19
//#define SPI_SCK 18
//#define SPI_CS 5

// ESP32-S3 SPI2/3 Ports 
#define HSPI_SS 10
#define HSPI_MOSI 11
#define HSPI_SCLK 12
#define HSPI_MISO 13

#define VSPI_SS 39
#define VSPI_MOSI 37
#define VSPI_SCLK 36
#define VSPI_MISO 35

Adafruit_BME280 bme; // I2C
bool bmefound = false;
extern Adafruit_TestBed TB;

Adafruit_MAX17048 lipo;
Adafruit_ST7789 display = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
ICM456xx IMU( SPI, HSPI_SCLK );

GFXcanvas16 canvas(240, 135);

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
  canvas.setFont(&FreeSans9pt7b);
  canvas.setTextColor(ST77XX_WHITE);

  if (!lipo.begin()) {
    Serial.println(F("Couldnt find Adafruit MAX17048?\nMake sure a battery is plugged in!"));
    while (1) delay(10);
  }
    
  Serial.print(F("Found MAX17048"));
  Serial.print(F(" with Chip ID: 0x")); 
  Serial.println(lipo.getChipID(), HEX);

  if (TB.scanI2CBus(0x77)) {
    Serial.println("BME280 address");

    unsigned status = bme.begin();  
    if (!status) {
      Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
      Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(),16);
      Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
      Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
      Serial.print("        ID of 0x60 represents a BME 280.\n");
      Serial.print("        ID of 0x61 represents a BME 680.\n");
      return;
    }
    Serial.println("BME280 found OK");
    bmefound = true;
  }

  pinMode(0, INPUT_PULLUP);
  pinMode(1, INPUT_PULLDOWN);
  pinMode(2, INPUT_PULLDOWN);
}

int j = 0;
bool valid_spi_devices[2];
int spi_device_count = 0;

// Function to scan SPI devices
void scanSPIDevices() {
  Serial.print("SPI scan: ");
  spi_device_count = 0;

  // Report known SPI devices on this board
  Serial.print("ST7789 TFT (CS:");
  Serial.print(TFT_CS);
  Serial.print(")");
  valid_spi_devices[0] = true;
  spi_device_count++;

  int IMUresult = IMU.begin();
  if( !IMUresult )
  {
    Serial.print( ", ICM45605" );
    valid_spi_devices[1] = true;
    spi_device_count++;
  }
  Serial.println();
}

void loop() {
  bool valid_i2c[128];
  //Serial.println("**********************");

  if (j == 0)
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

    // Scan for SPI devices
    scanSPIDevices();
  }

  if (j % 2 == 0) {
    canvas.fillScreen(ST77XX_BLACK);
    canvas.setCursor(0, 17);
    canvas.setTextColor(ST77XX_RED);
    canvas.println("Adafruit Feather");
    canvas.setTextColor(ST77XX_YELLOW);
    canvas.println("ESP32-S3 TFT Demo");
    canvas.setTextColor(ST77XX_GREEN); 
    canvas.print("Battery: ");
    canvas.setTextColor(ST77XX_WHITE);
    canvas.print(lipo.cellVoltage(), 1);
    canvas.print(" V  /  ");
    canvas.print(lipo.cellPercent(), 0);
    canvas.println("%");
    canvas.setTextColor(ST77XX_BLUE);
    canvas.print("I2C: ");
    canvas.setTextColor(ST77XX_WHITE);
    for (uint8_t a=0x01; a<=0x7F; a++) {
      if (valid_i2c[a])  {
        canvas.print("0x");
        canvas.print(a, HEX);
        canvas.print(", ");
      }
    }
    canvas.println("");
    canvas.setTextColor(ST77XX_MAGENTA);
    canvas.print("SPI: ");
    canvas.setTextColor(ST77XX_WHITE);
    if( valid_spi_devices[0] )
      canvas.println("ST7789 TFT");
    if( valid_spi_devices[1] )
      canvas.println(", ICM45605");
    canvas.print("Buttons: ");
    if( !digitalRead(0) )
      Serial.println("D0 Pressed!");

    if( digitalRead(1) )
      Serial.println("D1 Pressed!");

    if( digitalRead(2) )
      Serial.println("D2 Pressed!");
    //Serial.println(digitalRead(0));
    //Serial.println(digitalRead(1));
    //Serial.println(digitalRead(2));
    if (!digitalRead(0)) {
      canvas.setTextColor(ST77XX_RED);
      canvas.print("D0, ");
    }
    if (digitalRead(1)) {
      canvas.setTextColor(ST77XX_GREEN);
      canvas.print("D1, ");
    }
    if (digitalRead(2)) {
      canvas.setTextColor(ST77XX_BLUE);
      canvas.print("D2, ");
    }
    display.drawRGBBitmap(0, 0, canvas.getBuffer(), 240, 135);
    pinMode(TFT_BACKLITE, OUTPUT);
    digitalWrite(TFT_BACKLITE, HIGH);
  }
  
  TB.setColor(TB.Wheel(j++));
  delay(10);
  return;
}