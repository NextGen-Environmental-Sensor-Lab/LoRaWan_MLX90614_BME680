/*
   () PROJECT
   NGENS ASRC CUNY, R TOLEDO-CROW, V CHING, M BELLAMY JAN 2025

   SENSORS:
   -SEN0263 (IR TEMP, I2C, DFROBOT OR EQUIVALENT)
   -BME680 (TEMP. PRESS. RH, I2C)
   ADDITIONALLY USES:
   -HTCC-AB02 (HELTEC)

/*   
   Based on CubeCell example libraries

   Heltec HTCC-AB02 and a BME680 TPRH sensor for ALLAN FREI at HUNTER COLLEGE 
   Uses Adafruit's BME680 library 

   rtoledocrow@gc.cuny.edu, Oct 2022
   Two Sensors Edit by jasoncayetano01@gc.cuny.edu, Nov 2022
*/

#include "LoRaWan_APP.h"
#include "Arduino.h"
#include <Adafruit_Sensor.h>  //Addition of Adafruit Sensor Library
#include "Adafruit_BME680.h" //Constexpr error resolved from http://community.heltec.cn/t/htcc-ab01-v2-and-ads1115-error/12112
#include <Adafruit_MLX90614.h>

#include <Wire.h>

#define SEALEVELPRESSURE_HPA (1013.25)
#define Vext P3_2 

/*
   set LoraWan_RGB to Active,the RGB active in loraWan
   RGB red means sending;
   RGB purple means joined done;
   RGB blue means RxWindow1;
   RGB yellow means RxWindow2;
   RGB green means received done;
*/

/* OTAA para*/
// For devEui the last two bytes are random.
// For the rest see https://lora-developers.semtech.com/documentation/tech-papers-and-guides/the-book/deveui/
uint8_t devEui[] = { 0xFE, 0xFF, 0xFF, 0xFF, 0xFD, 0xFF, 0x92, 0x3E };  // rm-0004 on 01.10.25
// aka joinEUI Use all 0x00 as described here: https://lora-developers.semtech.com/documentation/tech-papers-and-guides/the-book/joineui
uint8_t appEui[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
// random from TTI website random
uint8_t appKey[] = { 0x03, 0x2A, 0x01, 0x2E, 0x8B, 0x93, 0x9C, 0x3D, 0x37, 0xA0, 0x6A, 0x02, 0xDB, 0xBF, 0x07, 0x1F };
/* ABP para (WE DONT USE)*/
uint8_t nwkSKey[] = { 0x15, 0xb1, 0xd0, 0xef, 0xa4, 0x63, 0xdf, 0xbe, 0x3d, 0x11, 0x18, 0x1e, 0x1e, 0xc7, 0xda, 0x85 };
uint8_t appSKey[] = { 0xd7, 0x2c, 0x78, 0x75, 0x8c, 0xdc, 0xca, 0xbf, 0x55, 0xee, 0x4a, 0x77, 0x8d, 0x16, 0xef, 0x67 };
uint32_t devAddr = (uint32_t)0x007e6ae1;
/*LoraWan channelsmask, default channels 0-7*/  // Helium use 0xffff
uint16_t userChannelsMask[6] = { 0xFF00, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 };

/*LoraWan region, select in arduino IDE tools*/
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;
/*LoraWan Class, Class A and Class C are supported*/
DeviceClass_t loraWanClass = LORAWAN_CLASS;
/*the application data transmission duty cycle.  value in [ms].*/
uint32_t appTxDutyCycle = 90000;  // 3 min cycle
/*OTAA or ABP*/
bool overTheAirActivation = LORAWAN_NETMODE;
/*ADR enable*/
bool loraWanAdr = LORAWAN_ADR;
/* set LORAWAN_Net_Reserve ON, the node could save the network info to flash, when node reset not need to join again */
bool keepNet = LORAWAN_NET_RESERVE;
/* Indicates if the node is sending confirmed or unconfirmed messages */
bool isTxConfirmed = LORAWAN_UPLINKMODE;
/* Application port */
uint8_t appPort = 2;
/*!
  Number of trials to transmit the frame, if the LoRaMAC layer did not
  receive an acknowledgment. The MAC performs a datarate adaptation,
  according to the LoRaWAN Specification V1.0.2, chapter 18.4, according
  to the following table:

  Transmission nb | Data Rate
  ----------------|-----------
  1 (first)       | DR
  2               | DR
  3               | max(DR-1,0)
  4               | max(DR-1,0)
  5               | max(DR-2,0)
  6               | max(DR-2,0)
  7               | max(DR-3,0)
  8               | max(DR-3,0)

  Note, that if NbTrials is set to 1 or 2, the MAC will not decrease
  the datarate, in case the LoRaMAC layer did not receive an acknowledgment
*/
uint8_t confirmedNbTrials = 4;

/* Prepares the payload of the frame */

Adafruit_BME680 bme(&Wire); 
Adafruit_MLX90614 mlx = Adafruit_MLX90614();

static void prepareTxFrame(uint8_t port) {
  /*appData size is LORAWAN_APP_DATA_MAX_SIZE which is defined in "commissioning.h".
    appDataSize max value is LORAWAN_APP_DATA_MAX_SIZE.
    if enabled AT, don't modify LORAWAN_APP_DATA_MAX_SIZE, it may cause system hanging or failure.
    if disabled AT, LORAWAN_APP_DATA_MAX_SIZE can be modified, the max value is reference to lorawan region and SF.
    for example, if use REGION_CN470,
    the max value for different DR can be found in MaxPayloadOfDatarateCN470 refer to DataratesCN470 and BandwidthsCN470 in "RegionCN470.h".
  */

  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, LOW);  // turn Vext on for BME680 and MLX90614

  // very important for the BME280 if being powered on and off by a fet switch
  // http://community.heltec.cn/t/bme280-fails-when-running-on-battery/2469
  Wire.end();
  Wire.begin();

  int stime = millis();
  while (!bme.begin() & (millis() - stime <= 10000)) {
    Serial.println("Problem starting BME680 rebooting.");
    digitalWrite(Vext, HIGH);
    delay(450);
    digitalWrite(Vext, LOW);
    delay(250);
  };
  if (millis() - stime > 10000) {
    Serial.println("BME680 not initialized");
  }

  //Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150);  // 320*C for 150 ms


  if (!mlx.begin()) {
    Serial.println("Error connecting to MLX sensor. Check wiring.");
    while (1);
  };

  float temperature = bme.temperature;
  float emissivity = mlx.readEmissivity();
  float ambientTemperature = mlx.readAmbientTempC();
  float objectTemperature = mlx.readObjectTempC();  
  float pressure = bme.pressure / 100.00;
  float humidity = bme.humidity;
  float altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
  float batteryVoltage = getBatteryVoltage() / 1000.00;
  delay(500);

  Wire.end(); //End I2C communication 
  digitalWrite(Vext, HIGH); //Navigate voltage away from sensors

  unsigned char *puc;
  puc = (unsigned char *)(&temperature);
  appDataSize = 28;  // 28 instead of 16, as three more unsigned chars have been added
  appData[0] = puc[0];
  appData[1] = puc[1];
  appData[2] = puc[2];
  appData[3] = puc[3];

  puc = (unsigned char *)(&emissivity);
  appData[4] = puc[0];
  appData[5] = puc[1];
  appData[6] = puc[2];
  appData[7] = puc[3];

  puc = (unsigned char *)(&ambientTemperature);
  appData[8] = puc[0];
  appData[9] = puc[1];
  appData[10] = puc[2];
  appData[11] = puc[3];

  puc = (unsigned char *)(&objectTemperature);
  appData[12] = puc[0];
  appData[13] = puc[1];
  appData[14] = puc[2];
  appData[15] = puc[3];

  puc = (unsigned char *)(&pressure);
  appData[16] = puc[0];
  appData[17] = puc[1];
  appData[18] = puc[2];
  appData[19] = puc[3];

  puc = (unsigned char *)(&humidity);
  appData[20] = puc[0];
  appData[21] = puc[1];
  appData[22] = puc[2];
  appData[23] = puc[3];
  
  puc = (unsigned char *)(&batteryVoltage);
  appData[24] = puc[0];
  appData[25] = puc[1];
  appData[26] = puc[2];
  appData[27] = puc[3];

  Serial.print("Temp,MLX Accuracy,MLX Temp,MLX TargetTemp,Press,Rh,V: ");
  Serial.print(temperature);
  Serial.print(", ");
  Serial.print(emissivity);
  Serial.print(", ");
  Serial.print(ambientTemperature);
  Serial.print(", ");
  Serial.print(objectTemperature);
  Serial.print(", ");
  Serial.print(pressure);
  Serial.print(", ");
  Serial.println(humidity);
  Serial.print(", ");
  Serial.println(batteryVoltage);
}

void setup() {
  Serial.begin(115200);
  delay(8000);
  Serial.println(__FILE__);
    Serial.println("Changed1");

#if (AT_SUPPORT)
  enableAt();
#endif
  deviceState = DEVICE_STATE_INIT;
  LoRaWAN.ifskipjoin();
}

void loop() {
  switch (deviceState) {
    case DEVICE_STATE_INIT:
      {
#if (LORAWAN_DEVEUI_AUTO)
        LoRaWAN.generateDeveuiByChipID();
#endif
#if (AT_SUPPORT)
        getDevParam();
#endif
        printDevParam();
        LoRaWAN.init(loraWanClass, loraWanRegion);
        deviceState = DEVICE_STATE_JOIN;
        break;
      }
    case DEVICE_STATE_JOIN:
      {
        LoRaWAN.join();
        break;
      }
    case DEVICE_STATE_SEND:
      {
        prepareTxFrame(appPort);
        LoRaWAN.send();
        deviceState = DEVICE_STATE_CYCLE;
        break;
      }
    case DEVICE_STATE_CYCLE:
      {
        // Schedule next packet transmission
        txDutyCycleTime = appTxDutyCycle + randr(0, APP_TX_DUTYCYCLE_RND);
        LoRaWAN.cycle(txDutyCycleTime);
        deviceState = DEVICE_STATE_SLEEP;
        break;
      }
    case DEVICE_STATE_SLEEP:
      {
        LoRaWAN.sleep();
        break;
      }
    default:
      {
        deviceState = DEVICE_STATE_INIT;
        break;
      }
  }
}
