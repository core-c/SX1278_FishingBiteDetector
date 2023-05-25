/*
  Dennis Beetmelder

  This program is using an SX1278 transceiver to receive signals from a fishing bite-detector.
  The signal is FSK @ 433.96 MHz. There is no preamble, no syncword, no address,..
  A whole packet is just a 32-bit value (and a special extra long high-bit as an end-of-packet marker).
*/

// include the libraries
#include <RadioLib.h>
#include <Adafruit_SSD1306.h>
#include <Ticker.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>


//=== SERIAL MONITOR ===============================================================
#define SERIAL_DEBUG 1
#ifdef SERIAL_DEBUG
#define DEBUG_PRINT(x) Serial.print(x)
#define DEBUG_PRINTLN(x) Serial.println(x)
#define DEBUG_PRINTHEX(x) Serial.print(x, HEX)
#define DEBUG_PRINTLNHEX(x) Serial.println(x, HEX)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#define DEBUG_PRINTHEX(x)
#define DEBUG_PRINTLNHEX(x)
#endif


//=== SX1278 =======================================================================
// http://www.lilygo.cn/prod_view.aspx?TypeId=50060&Id=1271&FId=t3:50060:3
#define SX1278_SCK 5
#define SX1278_MISO 19
#define SX1278_MOSI 27
#define SX1278_SS 18
#define SX1278_RST 23
#define SX1278_DIO0 26
//#define SX1278_DIO1 33
//#define SX1278_DIO2 32
#define SX1278_DIO1 35
#define SX1278_DIO2 34
#define SX1278_LED 25
SX1278 radio = new Module(SX1278_SS, SX1278_DIO0, SX1278_RST, SX1278_DIO1);


//=== OLED =========================================================================
#define OLED_SDA 21
#define OLED_SCL 22 
#define OLED_RST -1  // the T3_1.6.1 needs this be set at -1 (not 16)
#define OLED_ADDR 0x3C // the I2C address
#define OLED_SCREEN_WIDTH 128 // OLED display width, in pixels
#define OLED_SCREEN_HEIGHT 64 // OLED display height, in pixels
Adafruit_SSD1306 oled = Adafruit_SSD1306(OLED_SCREEN_WIDTH, OLED_SCREEN_HEIGHT, &Wire, OLED_RST);

#define FISH_HEIGHT   8
#define FISH_WIDTH    16
static const unsigned char PROGMEM fish_bmp[] =
{ 0b00000011, 0b00000000,
  0b00011111, 0b11110001,
  0b01110011, 0b11111011,
  0b11110011, 0b11111111,
  0b11111111, 0b11111111,
  0b01111111, 0b11111011,
  0b00011111, 0b11110001,
  0b00000011, 0b00000000 };

Ticker OLEDdisplayer;

float oled_rssi = -1000.0f;
uint32_t oled_packet = 0x0000;
uint16_t oled_packetcount = 0;
String oled_setting = "";


void OLED_show(uint32_t packet, float rssi, uint16_t packetcount, String setting_string) {
  // we display the packet bits on 2 rows because it'll not fit on one row of the OLED
  String packetstring1 = "";
  String packetstring2 = "";
  if (packet != 0x0000) {
    uint32_t mask = 0x80008000;
    for (int b=0; b<16; b++) {
      uint32_t pm = (packet & mask);
      packetstring1 += ((pm & 0xFFFF0000) != 0)? '1':'0';
      packetstring2 += ((pm & 0x0000FFFF) != 0)? '1':'0';
      mask >>= 1;
    }
  }
  //
  oled.clearDisplay();
  oled.fillRoundRect(0, 0, 128, 10, 3, SSD1306_WHITE);
  oled.setTextColor(SSD1306_BLACK);
  oled.setTextSize(1);
  oled.setCursor(12,1);
  oled.print(F("Dennis Beetmelder"));
  oled.setTextColor(SSD1306_WHITE);
  if (packet != 0x0000) {
    oled.setCursor(0,12);
    oled.print(packetstring1);
    oled.setCursor(0,22);
    oled.print(packetstring2);
    oled.setCursor(0,32);
    oled.print(packet,HEX);
    // perfect hit?
    if ((packet & 0x00FF0000) == 0x00790000) {
      oled.drawBitmap(104, 17, fish_bmp, FISH_WIDTH, FISH_HEIGHT, SSD1306_WHITE);
      oled.setCursor(104,32);
      oled.print(packetcount);
    }
  } else {
    oled.setTextSize(2);
    oled.setCursor(0,22);
    oled.print(setting_string);
  }
  oled.setTextSize(1);
  oled.setCursor(0,53);
  oled.print("RSSI: ");
  oled.setCursor(36,53);
  oled.print(rssi);
  oled.setCursor(86,53);
  oled.print("dBm");
  int16_t w = 128 + rssi;
  oled.drawFastHLine(0, 62, w, SSD1306_WHITE); // oled.drawFastHLine(w, 62, 128-w, SSD1306_BLACK);
  oled.drawFastHLine(0, 63, w, SSD1306_WHITE); // oled.drawFastHLine(w, 63, 128-w, SSD1306_BLACK);
  oled.display();
}


// OLED static callback routine
void OLED_display_callback() {
  OLED_show(oled_packet, oled_rssi, oled_packetcount, oled_setting);
  oled_packet = 0x0000;
  oled_rssi = -1000.0f;
}


//=== ONBOARD LED ==================================================================
#define LED_PIN 25
Ticker LEDflasher;

// LED static callback routine
void LED_off_callback() {
  digitalWrite(LED_PIN, LOW);
}

void LED_flash(int ms) {
  digitalWrite(LED_PIN, HIGH);
  LEDflasher.once_ms(ms, LED_off_callback);
}


//=== ISD1820 SOUND MODULE =========================================================
#define SOUND_PLAY_PIN 14
Ticker SoundPlayer;

// Sound Module static callback routine
void SOUND_off_callback() {
  digitalWrite(SOUND_PLAY_PIN, LOW);
}

void SOUND_play(int ms) {
  digitalWrite(SOUND_PLAY_PIN, LOW); // make sure the trigger is released first..
  digitalWrite(SOUND_PLAY_PIN, HIGH);
  SoundPlayer.once_ms(ms, SOUND_off_callback);
}


//=== BLUETOOTH ====================================================================
BLEServer *pServer = NULL;
BLECharacteristic * pTxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;
uint8_t txValue = 0;

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"


class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };
    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();
      if (rxValue.length() > 0) {
        DEBUG_PRINTLN(F(""));
        DEBUG_PRINT(F("BLE Received Value: "));
        for (int i = 0; i < rxValue.length(); i++) DEBUG_PRINT(rxValue[i]);
        //DEBUG_PRINT(rxValue);
        DEBUG_PRINTLN(F(""));
        // change the setting
        String value = rxValue.c_str();
        EditValue(value);
      }
    }
};

void BT_server_start() {
  // Create the BLE Device
  BLEDevice::init("Beetmelder");
  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);
  // Create a BLE TX Characteristic
  pTxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_TX, BLECharacteristic::PROPERTY_NOTIFY);       
  pTxCharacteristic->addDescriptor(new BLE2902());
  // Create a BLE RX Characteristic
  BLECharacteristic * pRxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_RX, BLECharacteristic::PROPERTY_WRITE);
  // callback for processing the received data
  pRxCharacteristic->setCallbacks(new MyCallbacks());
  // Start the service
  pService->start();
  // Start advertising
  pServer->getAdvertising()->start();
}


//=== Radio settings ===============================================================
#define RADIO_FREQUENCY_MHZ 433.96
#define RADIO_BITRATE 0.666
#define RADIO_DEVIATION_KHZ 3.5
#define RADIO_RXBANDWIDTH_KHZ 25.0
#define RADIO_AFCBANDWIDTH_KHZ 25.0
// signal properties
#define NOISE_FLOOR (-87.0)
#define SIGNAL_US_IGNORE 150
#define SIGNAL_US_BIT_0 700
#define SIGNAL_US_BIT_1 1300
#define LOW_US_EOP 4800
// timeout values (in seconds)
#define TIMEOUT_SERIAL_INPUT 1
#define TIMEOUT_CLEAR 10

float prev_rssi = -1000.0f, rssi = -1000.0f, signal_rssi = -1000.0f;
uint32_t signal_start, signal_end, signal_us;
uint32_t low_start = 0, low_us;
uint32_t packet = 0x0000;
int packetbits = -1;
uint16_t packetcount = 0; // how many valid packets are received before the next timeout occurs
float noisefloor = NOISE_FLOOR;
int state;


#define CHECK_FOR_ERROR(x,y) \
  DEBUG_PRINT(x); \
  if (state == RADIOLIB_ERR_NONE) { \
    DEBUG_PRINTLN(F(y"*")); \
  } else { \
    DEBUG_PRINT(F(y"error:")); \
    DEBUG_PRINTLN(state); \
    while (true); \
  }


void EditValue(String value) {
  // save the value for display during signal inactivity
  oled_setting = value;
  // process
  if (value.substring(0,1) == "d") { // freq deviation
    float freqDeviation = value.substring(1).toFloat();
    radio.setFrequencyDeviation(freqDeviation);
    DEBUG_PRINT(F("FREQUENCY DEVIATION: "));
    DEBUG_PRINTLN(freqDeviation);
  } else
  if (value.substring(0,1) == "w") { // rx bandwidth
    float rxBandwidth = value.substring(1).toFloat();
    radio.setRxBandwidth(rxBandwidth);
    DEBUG_PRINT(F("RX BANDWIDTH: "));
    DEBUG_PRINTLN(rxBandwidth);
  }
  if (value.substring(0,1) == "r") { // bitrate
    float bitrate = value.substring(1).toFloat();
    radio.setBitRate(bitrate);
    DEBUG_PRINT(F("BITRATE: "));
    DEBUG_PRINTLN(bitrate);
  }
  if (value.substring(0,1) == "s") { // rssi smoothing
    uint8_t smooth = value.substring(1).toInt();
    radio.setRSSIConfig(smooth); // 0=2 samples, 1=4 samples, 2=8 samples, 3=16...
    DEBUG_PRINT(F("RSSI SMOOTHING: "));
    DEBUG_PRINTLN(smooth);
  }
  if (value.substring(0,1) == "n") { // noisefloor
    float noise = value.substring(1).toFloat();
    radio.setBitRate(noise);
    DEBUG_PRINT(F("NOISEFLOOR: "));
    DEBUG_PRINTLN(noise);
  }

  if (value.substring(0,1) == "b") { // BLE enable/disable
    uint8_t ble_enabled = value.substring(1).toInt();
    if (ble_enabled == 0) {
      BLEDevice::stopAdvertising();
      BLEDevice::deinit(true);
    } /*else if (ble_enabled == 1) {
      BT_server_start();
    }*/
    DEBUG_PRINT(F("BLE: "));
    DEBUG_PRINTLN(ble_enabled);
  }

}


//=== SETUP FUNCTION ===============================================================
void setup() {
#ifdef SERIAL_DEBUG
  Serial.begin(115200);
#endif
  DEBUG_PRINTLN(F(""));

  // LED
  pinMode(LED_PIN, OUTPUT);
  LED_flash(1000);

  // sound module
  pinMode(SOUND_PLAY_PIN, OUTPUT);

  // start a BLE server
  DEBUG_PRINTLN(F("BLE Initialize"));
  BT_server_start();

  // ESP32 information
  uint32_t mhz = getCpuFrequencyMhz(); // cpu_hal_get_cycle_count()  clockCyclesPerMicrosecond()  microsecondsToClockCycles()
  DEBUG_PRINT(F("ESP32 @ "));
  DEBUG_PRINT(mhz);
  DEBUG_PRINTLN(F(" MHz"));

  //initialize OLED
  Wire.begin(OLED_SDA, OLED_SCL);
  if (!oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR, false, false)) { // Address 0x3C for 128x32
    DEBUG_PRINTLN(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  DEBUG_PRINTLN(F("SSD1306 Initialize\t\t\t*"));
  // setup OLED screen
  oled.clearDisplay();
  oled.setTextColor(WHITE);
  oled.setTextSize(1);
  oled.setCursor(0,0);
  oled.print(F("core_c.. codings"));
  oled.display();

  // initialize SX1278 with default settings
  state = radio.begin();
  CHECK_FOR_ERROR(F("SX1278 Initialize"),"\t\t\t");

  state = radio.beginFSK(); //433.96, 0.666, 3.5, 25.0, 10, 0, false);
  CHECK_FOR_ERROR(F("SX1278 Starting FSK mode"),"\t\t");

  state = radio.setFrequency(RADIO_FREQUENCY_MHZ); // 433.96
  CHECK_FOR_ERROR(F("SX1278 Set frequency"),"\t\t\t");

  state = radio.setBitRate(RADIO_BITRATE); // 0.666
  CHECK_FOR_ERROR(F("SX1278 Set bitrate"),"\t\t\t");

  state = radio.setFrequencyDeviation(RADIO_DEVIATION_KHZ); // 3.5
  CHECK_FOR_ERROR(F("SX1278 Set frequency deviation"),"\t\t");

  state = radio.setRxBandwidth(RADIO_RXBANDWIDTH_KHZ); // 25.0
  CHECK_FOR_ERROR(F("SX1278 Set RX bandwidth"),"\t\t\t");

  state = radio.disableAddressFiltering();
  CHECK_FOR_ERROR(F("SX1278 Disable address filtering"),"\t");

  state = radio.setPreambleLength(0); // 0
  CHECK_FOR_ERROR(F("SX1278 Disable preamble"),"\t\t\t");

  state = radio.setCRC(false); // false
  CHECK_FOR_ERROR(F("SX1278 Disable CRC"),"\t\t\t");

  state = radio.setEncoding(RADIOLIB_ENCODING_NRZ);
  CHECK_FOR_ERROR(F("SX1278 Set encoding"),"\t\t\t");

  state = radio.setRSSIConfig(1); // 0=2 samples, 1=4 samples, 2=8 samples, 3=16...
  CHECK_FOR_ERROR(F("SX1278 Set RSSI smoothing"),"\t\t");

  state = radio.setRSSIThreshold(noisefloor); // dBm
  CHECK_FOR_ERROR(F("SX1278 Set RSSI threshold"),"\t\t");

  state = radio.setGain(1); // 0=auto, 1=highest gain
  CHECK_FOR_ERROR(F("SX1278 Set gain"),"\t\t\t\t");

  state = radio.setAFCBandwidth(RADIO_AFCBANDWIDTH_KHZ); // 25.0
  CHECK_FOR_ERROR(F("SX1278 Set AFC bandwidth"),"\t\t");

  state = radio.setAFCAGCTrigger(RADIOLIB_SX127X_RX_TRIGGER_RSSI_INTERRUPT);
  CHECK_FOR_ERROR(F("SX1278 Set AFC/AGC trigger"),"\t\t");

  state = radio.setAFC(true);
  CHECK_FOR_ERROR(F("SX1278 Enable AFC"),"\t\t\t");

//  float AFCerror = radio.getAFCError();
//  radio.setDataShaping(RADIOLIB_SHAPING_1_0); //RADIOLIB_SHAPING_NONE

  state = radio.setOOK(false);
  CHECK_FOR_ERROR(F("SX1278 Disable OOK"),"\t\t\t");

  // start listening for FSK packets
  state = radio.startReceive();
  CHECK_FOR_ERROR(F("SX1278 Listening"),"\t\t\t");

/*
  uint32_t test_start = esp_timer_get_time();
  uint32_t test_value = 0;
  while (esp_timer_get_time()-1000000 < test_start) {
    test_value++;
  }
  DEBUG_PRINT(F("esp_timer_get_time() calls per second: "));
  DEBUG_PRINTLN(test_value);


  test_start = esp_timer_get_time();
  test_value = 0;
  while (esp_timer_get_time()-1000000 < test_start) {
    float rssi = radio.getRSSI(false,true); // if put into receiver mode, it will trigger a reset and the AFC/AGC will adjust
    test_value++;
  }
  DEBUG_PRINT(F("esp_timer_get_time()+getRSSI() calls per second: "));
  DEBUG_PRINTLN(test_value);
*/
  low_start = esp_timer_get_time() - LOW_US_EOP;
}


//=== LOOP FUNCTION ================================================================
void loop() {

  prev_rssi = rssi;
  rssi = radio.getRSSI(false,true); // if put into receiver mode, it will trigger a reset and the AFC/AGC will adjust

  // noise or signal?
  if (rssi > noisefloor) { // now there's a signal
    if (prev_rssi < noisefloor) { //========================================================== transition noise->signal

      signal_start = esp_timer_get_time(); //signal_start = cpu_hal_get_cycle_count();
      low_us = signal_start - low_start;

      if (low_us > LOW_US_EOP) { // end of packet
        float freqError = radio.getFrequencyError(true);
        float afcError = radio.getAFCError();
#ifdef SERIAL_DEBUG
        if ((freqError != 0.0) || (afcError != 0.0)) {
          DEBUG_PRINTLN(F(""));
          DEBUG_PRINT(F("ERROR "));
          if (freqError != 0.0) {
            DEBUG_PRINT(F("\tFREQ:"));
            DEBUG_PRINT(freqError);
          }
          if (afcError != 0.0) {
            DEBUG_PRINT(F("\tAFC:"));
            DEBUG_PRINT(afcError);
          }
        }
#endif
        DEBUG_PRINTLN(F(""));
        DEBUG_PRINT(F("_  "));
        // end-of-packet received. Start a new packet..
        packet = 0x0000;
        packetbits = -1;
        signal_rssi = -1000.0f;
      }

      // blink the LED
      LED_flash(1);

    } else { //=============================================================================== the signal continues
      if (rssi > signal_rssi) signal_rssi = rssi; // hold the strongest signal strength (for display)
    }

  } else { // now it's noise
    if (prev_rssi > noisefloor) { //========================================================== transition signal->noise

      signal_end = esp_timer_get_time(); //signal_end = cpu_hal_get_cycle_count();
      signal_us = signal_end - signal_start; //signal_us = clockCyclesToMicroseconds(signal_end - signal_start);
      low_start = signal_end;

      // 0 or 1 bit?
      if (signal_us < SIGNAL_US_IGNORE) { // too short for a signal pulse..
        // ignore
      } else {
        // LSL packet
        packet <<= 1;
        packetbits++;
        if (signal_us < SIGNAL_US_BIT_0) { // 0-bit
          DEBUG_PRINT(F("0"));
        } else /*if (signal_us < SIGNAL_US_BIT_1)*/ { // 1-bit
          packet |= 0x0001;
          DEBUG_PRINT(F("1"));
        }
      }

      // valid packet?
      if (packetbits == 32) {
        // All bits of the packet have been received. Now there's some time to display data..
        // bits[23..12] are constant (for my tested beetmelder): 0x79B
        // 01100001001111001101110101111010_  C279BAF4
//@        if ((packet & 0x00FFF000) == 0x0079B000) {
        if ((packet & 0x00FF0000) == 0x00790000) { // more chance for a match..
          DEBUG_PRINT(F("\tYEAH!\t"));
          // trigger a sound
          SOUND_play(3000);
          // copy values for display on the OLED
          oled_packet = packet;
          if (signal_rssi > oled_rssi) oled_rssi = signal_rssi;
          oled_packetcount = packetcount;
          OLEDdisplayer.once_ms(1, OLED_display_callback);
        } else { // mismatch
          DEBUG_PRINT(F("\t\t"));
        }
        //
        DEBUG_PRINTHEX(packet);
        //
        packet = 0x0000;
        packetbits = -1;
        signal_rssi = -1000.0f;
        packetcount++;
      }

    } else { //=============================================================================== no signal for a while

      // reset the packetcount, and clear the OLED display
      if ((esp_timer_get_time() - signal_end) / 1000000 > TIMEOUT_CLEAR) { // no signal for 10s
        packetcount = 0;
        OLED_show(0x00000000, rssi, 0x0000, oled_setting);
      }

      // Bluetooth
      if ((esp_timer_get_time() - signal_end) / 1000000 > TIMEOUT_CLEAR) { // no signal for 10s
        // disconnecting
        if (!deviceConnected && oldDeviceConnected) {
            delay(500); // give the bluetooth stack the chance to get things ready
            pServer->startAdvertising(); // restart advertising
            DEBUG_PRINTLN(F(""));
            DEBUG_PRINTLN(F("BLE Start advertising"));
            oldDeviceConnected = deviceConnected;
        }
        // connecting
        if (deviceConnected && !oldDeviceConnected) {
        // do stuff here on connecting
            oldDeviceConnected = deviceConnected;
        }
      }

#ifdef SERIAL_DEBUG
      // check for serial input
      if ((esp_timer_get_time() - signal_end) / 1000000 > TIMEOUT_SERIAL_INPUT) { // no signal for 1s
        if (Serial.available()) {
          String value = Serial.readString();
          DEBUG_PRINTLN(value);
          value.trim();
          EditValue(value);
          while (Serial.available()) Serial.read();
          radio.startReceive();
        }
      }
#endif

    }
  }

}