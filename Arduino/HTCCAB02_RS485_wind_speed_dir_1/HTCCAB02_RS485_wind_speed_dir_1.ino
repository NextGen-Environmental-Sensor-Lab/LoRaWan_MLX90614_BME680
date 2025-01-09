/*
   PSC JAMAICA BAY WIND PROJECT
   NGENS ASRC CUNY, R TOLEDO-CROW, V CHING, OCT 2024

   SENSORS:
   -SEN0482 (WIND DIRECTION RS485, DFROBOT OR EQUIVALENT)
   -SEN0483 (WIND SPEED RS485, DFROBOT OR EQUIVALENT)
   -BME280 (TEMP. PRESS. RH, I2C)
   ADDITIONALLY USES:
   -HTCC-AB02 (HELTEC)
   -PS61023 (3.3V->5V, ADAFRUIT)
   -RS485->UART (SOLDERED) */
/*   
   Based on CubeCell example libraries

   Heltec HTCC-AB02 and a BME280 TPRH sensor for Zihao Zhang at CCNY Architecture
   Uses Adafruit's BME280 library as Seeed no good

   rtoledocrow@gc.cuny.edu, Oct 2022
   Two Sensors Edit by jasoncayetano01@gc.cuny.edu, Nov 2022
*/

#include "LoRaWan_APP.h"
#include "Arduino.h"
#include <Adafruit_Sensor.h>  //Addition of Adafruit Sensor Library
#include <Adafruit_BME280.h>
#include <Wire.h>

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
uint8_t devEui[] = { 0xFE, 0xFF, 0xFF, 0xFF, 0xFD, 0xFF, 0xEB, 0x89 };  // rm-0003 on 10.27.24
// aka joinEUI Use all 0x00 as described here: https://lora-developers.semtech.com/documentation/tech-papers-and-guides/the-book/joineui
uint8_t appEui[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
// random from TTI website random
uint8_t appKey[] = { 0x07, 0x98, 0x56, 0x28, 0x6B, 0x50, 0xA0, 0x22, 0x3E, 0xE7, 0xC1, 0x66, 0x83, 0xC0, 0x55, 0x69 };
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

Adafruit_BME280 bme280_1;  //FIRST SENSOR Creation of object of type bme280 and calling the object bme280

// Adafruit_BME280 bme280_2; //SECOND SENSOR

// unsigned long DelayTime;

/* FOR second UART for cubecell AB02, AB02S, AB02A*/
#define RE_DE GPIO8  // RS485
#define TSTPT GPIO9
#define TRANSMIT HIGH
#define RECEIVE LOW

static void prepareTxFrame(uint8_t port) {
  /*appData size is LORAWAN_APP_DATA_MAX_SIZE which is defined in "commissioning.h".
    appDataSize max value is LORAWAN_APP_DATA_MAX_SIZE.
    if enabled AT, don't modify LORAWAN_APP_DATA_MAX_SIZE, it may cause system hanging or failure.
    if disabled AT, LORAWAN_APP_DATA_MAX_SIZE can be modified, the max value is reference to lorawan region and SF.
    for example, if use REGION_CN470,
    the max value for different DR can be found in MaxPayloadOfDatarateCN470 refer to DataratesCN470 and BandwidthsCN470 in "RegionCN470.h".
  */

  // from : https://wiki.dfrobot.com/RS485_Wind_Speed_Transmitter_SKU_SEN0483
  uint8_t getWindSpeed[8] = { 0x03, 0x03, 0x00, 0x00, 0x00, 0x01, 0x85, 0xE8 };
  // from : https://wiki.dfrobot.com/RS485_Wind_Direction_Transmitter_SKU_SEN0482
  uint8_t getWindDir[8] = { 0x02, 0x03, 0x00, 0x00, 0x00, 0x01, 0x84, 0x39 };

  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, LOW);  // turn Vext on for bme280 and RS485

  Serial1.begin(9600);  // for rs485 translator
  delay(100);

  pinMode(RE_DE, OUTPUT);  // rx/tx pin
  digitalWrite(RE_DE, TRANSMIT);
  delay(500);

  // very important for the BME280 if being powered on and off by a fet switch
  // http://community.heltec.cn/t/bme280-fails-when-running-on-battery/2469
  Wire.end();
  Wire.begin();  

  // test pin for timing diagrams
  // pinMode(TSTPT, OUTPUT);
  // digitalWrite(TSTPT, LOW);

  while (Serial1.available()) {  // flush
    Serial1.read();
  }

  transmitRS485data(getWindDir, 8);
  int dir = receiveRS485data();
  float fdir = (float)dir / 10;
  delay(500);

  transmitRS485data(getWindSpeed, 8);
  int speed = receiveRS485data();
  float fspeed = (float)speed / 10;
  delay(500);

  int stime = millis();
  while (!bme280_1.begin() & (millis() - stime <=10000)) {
    Serial.println("Problem starting bme280 rebooting.");
    digitalWrite(Vext, HIGH);
    delay(450);
    digitalWrite(Vext,LOW);
    delay(250);
  };
  if (millis() - stime > 10000) {
    Serial.println("bme280 not initialized");
  }

  float temperature = bme280_1.readTemperature();
  float pressure = bme280_1.readPressure() / 100.00;
  float humidity = bme280_1.readHumidity();
  // float temperature = 0.0;
  // float pressure = 0.0;
  // float humidity = 0.0;

  float batteryVoltage = getBatteryVoltage() / 1000.00;
  delay(500);

  // digitalWrite(Vext, HIGH);  // Vext off

  unsigned char *puc;
  puc = (unsigned char *)(&fspeed);
  appDataSize = 28;  // 28 instead of 16, as three more unsigned chars have been added
  appData[0] = puc[0];
  appData[1] = puc[1];
  appData[2] = puc[2];
  appData[3] = puc[3];

  puc = (unsigned char *)(&fdir);
  appData[4] = puc[0];
  appData[5] = puc[1];
  appData[6] = puc[2];
  appData[7] = puc[3];

  puc = (unsigned char *)(&temperature);
  appData[8] = puc[0];
  appData[9] = puc[1];
  appData[10] = puc[2];
  appData[11] = puc[3];

  puc = (unsigned char *)(&pressure);
  appData[12] = puc[0];
  appData[13] = puc[1];
  appData[14] = puc[2];
  appData[15] = puc[3];

  puc = (unsigned char *)(&humidity);
  appData[16] = puc[0];
  appData[17] = puc[1];
  appData[18] = puc[2];
  appData[19] = puc[3];

  puc = (unsigned char *)(&batteryVoltage);  //just one is needed, as there is one power source at the moment
  appData[20] = puc[0];
  appData[21] = puc[1];
  appData[22] = puc[2];
  appData[23] = puc[3];

  Serial.print("speed,dir,temp,press,rh,V: ");
  Serial.print(fspeed);
  Serial.print(", ");
  Serial.print(fdir);
  Serial.print(", ");
  Serial.print(temperature);
  Serial.print(", ");
  Serial.print(pressure);
  Serial.print(", ");
  Serial.print(humidity);
  Serial.print(", ");
  Serial.println(batteryVoltage);
}

void setup() {
  Serial.begin(115200);
  delay(8000);
  Serial.println(__FILE__);

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

void transmitRS485data(uint8_t *tbuffer, uint16_t bufSize) {
  while (Serial1.available())  // clear the buffer
    Serial1.read();

  digitalWrite(RE_DE, TRANSMIT);  // transmit mode
  delay(100);                     // settle

  for (int i = 0; i < bufSize; i++) {
    //digitalWrite(TSTPT, HIGH);
    // Serial.print(tbuffer[i], HEX);
    // Serial.print(" ");
    Serial1.write(tbuffer[i]);
    Serial1.flush();
    delayMicroseconds(1200);
    //digitalWrite(TSTPT, LOW);
  }
  //Serial.println();
  digitalWrite(RE_DE, RECEIVE);  // receive mode
  delay(100);                    // let settle
}

uint16_t receiveRS485data() {
  uint8_t serialBuffer[256];
  digitalWrite(RE_DE, RECEIVE);  // receive mode
  delay(100);

  int stime = millis();
  while (!Serial1.available() & (millis() - stime <= 3000)) {
    Serial.print(".");
    delay(100);
  }
  if (millis() - stime > 3000) {
    Serial.println("RS485 not receiving anything.");
  }

  int inx = 0;
  while (Serial1.available()) {
    serialBuffer[inx++] = Serial1.read();
  }
  // can check crc here
  uint16_t valu = (serialBuffer[inx - 4] << 8) | serialBuffer[inx - 3];
  return valu;
}