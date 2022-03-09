#include <Arduino.h>
#include <driver/i2s.h>
#include "Wire.h"			  
#include "Config.h"
#include "MPU6886.h"
#include "AXP192.h"
#include "RTC.h"
#include "SPI.h"
#include "ILI9341_Defines.h"

#define CONFIG_I2S_BCK_PIN 12
#define CONFIG_I2S_LRCK_PIN 0
#define CONFIG_I2S_DATA_PIN 2
#define CONFIG_I2S_DATA_IN_PIN 34

#define CST_DEVICE_ADDR 0x38
#define CST_INT 39

#define DEFAULT_INTERVAL 13
#define SPI_FREQUENCY  40000000
#define TFT_SPI_MODE SPI_MODE0

#define TFT_MISO 38
#define TFT_MOSI 23
#define TFT_SCLK 18
#define TFT_CS   5      // Chip select control pin
#define TFT_DC   15     // Data Command control pin

#define TFT_SWRST   0x01
#define TFT_INVON   0x21

#define CS_L GPIO.out_w1tc = (1 << TFT_CS);GPIO.out_w1tc = (1 << TFT_CS)
#define CS_H GPIO.out_w1ts = (1 << TFT_CS)//;GPIO.out_w1ts = (1 << TFT_CS)

#define DC_C GPIO.out_w1tc = (1 << TFT_DC)//;GPIO.out_w1tc = (1 << TFT_DC)
#define DC_D GPIO.out_w1ts = (1 << TFT_DC)//;GPIO.out_w1ts = (1 << TFT_DC)

void ft6336_fw_updater(void);

typedef enum {
    kPOWER_EXTERNAL = 0,
    kPOWER_INTERNAL,
    kPOWER_MAX
}system_power_t;

// Power
AXP192 Axp;

MPU6886 IMU;

RTC  Rtc;

SPIClass& spi = SPI;


void spi_begin(void)
{
 spi.beginTransaction(SPISettings(SPI_FREQUENCY, MSBFIRST, TFT_SPI_MODE)); 
 CS_L;
}

void spi_end(void)
{   
    CS_H;
    spi.endTransaction();
}

void writecommand(uint8_t c)
{
  spi_begin(); // CS_L;
  DC_C;  
  spi.transfer(c);
  DC_D;
  spi_end();  // CS_H;
}

void writedata(uint8_t d)
{
  spi_begin(); // CS_L;
  DC_D;        // Play safe, but should already be in data mode  
  spi.transfer(d);
  CS_L;        // Allow more hold time for low VDI rail
  spi_end();   // CS_H;
}



void TFT_eSPI__init(uint8_t tc)
{  
  spi.begin(TFT_SCLK, TFT_MISO, TFT_MOSI, -1);

  // Set to output once again in case D6 (MISO) is used for CS
  digitalWrite(TFT_CS, HIGH); // Chip select high (inactive)
  pinMode(TFT_CS, OUTPUT);
 
  digitalWrite(TFT_DC, HIGH); // Data/Command high = data mode
  pinMode(TFT_DC, OUTPUT);
  spi_end();


  // Toggle RST low to reset
  writecommand(TFT_SWRST); // Software reset
 
  delay(150); // Wait for reset to complete

  // This loads the driver specific initialisation code  <<<<<<<<<<<<<<<<<<<<< ADD NEW DRIVERS TO THE LIST HERE <<<<<<<<<<<<<<<<<<<<<<<

  #include "ILI9341_Init.h"
  
  writecommand(TFT_INVON);
}

uint8_t ft6336(uint8_t reg) 
{
  Wire1.beginTransmission((uint8_t)CST_DEVICE_ADDR);
  Wire1.write(reg);
  Wire1.endTransmission();
  Wire1.requestFrom((uint8_t)CST_DEVICE_ADDR, uint8_t(1));
  return Wire1.read();
}

void ft6336(uint8_t reg, uint8_t value) 
{
  Wire1.beginTransmission(CST_DEVICE_ADDR);
  Wire1.write(reg);
  Wire1.write((uint8_t)value);
  Wire1.endTransmission();
}

uint8_t interval(uint8_t ivl) 
{
  ft6336(0x88, ivl);
  return ft6336(0x88);
}

void M5Touch__begin() 
{
  Wire1.begin(21, 22);
  pinMode(CST_INT, INPUT);

  // By default, the FT6336 will pulse the INT line for every touch
  // event. But because it shares the Wire1 TwoWire/I2C with other
  // devices, we cannot easily create an interrupt service routine to
  // handle these events. So instead, we set the INT wire to polled mode,
  // so it simply goes low as long as there is at least one valid touch.
  ft6336(0xA4, 0x00);

  Serial.print("touch: ");
  if (interval(DEFAULT_INTERVAL) == DEFAULT_INTERVAL) 
  {
    Serial.printf("FT6336 ready (fw id 0x%02X rel %d, lib 0x%02X%02X)\n",
                  ft6336(0xA6), ft6336(0xAF), ft6336(0xA1), ft6336(0xA2));
  } 
  else 
  {
    Serial.println("ERROR - FT6336 not responding");
  }
}


void M5Core2_begin() 
{  
  // UART
  Serial.begin(115200);
  Serial.flush();
  delay(50);
  Serial.print("M5Core2 initializing...");

  // I2C init
  Wire.begin(32, 33);

  Axp.begin(kMBusModeOutput);

  ft6336_fw_updater();  

  // LCD INIT
  TFT_eSPI__init(0);

  // Touch init
  M5Touch__begin(); // Touch begin after AXP begin. (Reset at the start of AXP)

  Rtc.begin();
}

void setup()
{   
    M5Core2_begin();    
}

void loop()
{
    delay(1000);
    Serial.println("read device id: ");
    Serial.println(ft6336(0xA3));
}