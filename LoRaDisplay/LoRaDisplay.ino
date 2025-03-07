/*
  LoRa Simple receiver
  Based on the example "LoRa Simple Gateway/Node Exemple"

  Hardware: either
  * Arduino Pro Mini, or
  * ESP32 DEVKIT according to https://randomnerdtutorials.com/esp32-lora-rfm95-transceiver-arduino-ide/ or
  * WeAct ESP32C3 

  Watchdog concept explained: https://forum.arduino.cc/t/esp32-und-watchdog-callback/1072863/23 but this is for the old
  espressif API. The differences between the old and the new API are explained here: https://esp32.com/viewtopic.php?t=40261
  and https://iotassistant.io/esp32/fixing-error-hardware-wdt-arduino-esp32/
  But: The watchdog hits, but the software does not restart. Reason unclear. So we do not use the watchdog at the moment.
*/

//#define USE_ESP32DEVKIT
#define USE_WEACT_ESP32C3
#define USE_WIFI
#define USE_SERIAL

#include <SPI.h>              // include libraries
#include <LoRa.h> /* Lib: LoRa by Sandeep Mistry */

//#define USE_WATCHDOG

#ifdef USE_WIFI
  #include <WiFi.h> /* Wifi connection for the ESP32 */
  #include "privatesettings.h" /* wifi credentials, server name etc */
  #ifdef USE_WATCHDOG
    #include <esp_task_wdt.h>

    #define MY_WDT_TIMEOUT 3    /* 3 seconds watchdog timeout */
    int lastWatchdogTriggerTime;
    int watchdogTriggerCount;
    #define CONFIG_FREERTOS_NUMBER_OF_CORES 2 /* ESP32 with 2 cores */
  #endif
  #define STATUS_LOG_CYCLE_TIME_MS 60000
  uint32_t lastStatusLogTime=-15000; /* trick: first log after 15s */
  uint32_t last100msTime=0;
#endif

const long frequency = 868E6;  // LoRa Frequency

#ifdef USE_WEACT_ESP32C3
  /* board schematic: https://github.com/WeActStudio/WeActStudio.ESP32C3CoreBoard/blob/master/Hardware/WeAct-ESP32C3CoreBoard_V10_SchDoc.pdf */
  //define the pins used by the transceiver module
  //#define PIN_CLK 4  SPI assignment according to https://www.esp32.com/viewtopic.php?t=25680
  //#define PIN_MISO 5  and http://tamanegi.digick.jp/computer-embedded/mcuboa/wa-esp32c3/
  //#define PIN_MOSI 6
  const int csPin = 7;     // LoRa radio chip select
  const int resetPin = 10; // LoRa radio reset
  const int irqPin = 2;    // LoRa hardware interrupt pin, DIO0
  #define LED_PORT 8
#else
 #ifdef USE_ESP32DEVKIT
  //define the pins used by the transceiver module
  const int csPin = 5;     // LoRa radio chip select
  const int resetPin = 14; // LoRa radio reset
  const int irqPin = 2;    // LoRa hardware interrupt pin, DIO0
 #else
  /* pins on Arduino Pro Mini */
  const int csPin = 10;          // LoRa radio chip select
  const int resetPin = 9;        // LoRa radio reset
  const int irqPin = 2;          // LoRa hardware interrupt pin, DIO0
 #endif
#endif

#define MYRXBUFFER_SIZE 64
uint8_t myRxBuffer[MYRXBUFFER_SIZE];
uint8_t myRxBufferLen;
uint8_t rxDataAvailable;
int rxPacketSizeOriginal;
int rxRssi;
float rxSnr;
uint16_t LED_divider;

uint16_t nTransmitCounts, txpower_dBm, u_batt_mV, u_aux_mV, sum;

#ifdef USE_WIFI
// Use WiFiClient class to create TCP connections
WiFiClient myHttpClient;

void myWifiConnect(void) {
   // We start by connecting to a WiFi network
   if (1) {
        WiFi.mode(WIFI_STA);
        #ifdef USE_SERIAL
          Serial.print("Connecting to "); Serial.println(ssid);
        #endif
        WiFi.begin(ssid, password);
        while (WiFi.status() != WL_CONNECTED) {
            delay(500);
            #ifdef USE_SERIAL
              Serial.print(".");
            #endif
        }
        #ifdef USE_SERIAL
          Serial.println("");
          Serial.println("WiFi connected");
          Serial.println("IP address: ");
          Serial.println(WiFi.localIP());
        #endif
   } else {
      #ifdef USE_SERIAL
        Serial.println("---WILL NOT CONNECT TO WIFI -----");
      #endif
   }
}

void makeSureThatWifiIsConnected(void) {
   if (WiFi.status() == WL_CONNECTED) {
      #ifdef USE_SERIAL
        Serial.println("makeSureThatWifiIsConnected: WiFi is already connected.");
      #endif
   } else {
      #ifdef USE_SERIAL
        Serial.println("makeSureThatWifiIsConnected: Will try to connect.");
      #endif
      myWifiConnect();
   }
}

void addValueToWebTrace(char *dataname, char *datavalue) {
  #ifdef USE_SERIAL
    Serial.print("addValueToWebTrace ");
    //Serial.print(tracename);Serial.print(" ");
    Serial.print(dataname);Serial.print(" ");
    Serial.println(datavalue);
  #endif
  makeSureThatWifiIsConnected();
  #ifdef USE_SERIAL
    Serial.print("connecting to ");
    Serial.println(httphost);
  #endif
  int r = myHttpClient.connect(httphost, httpPort);
  #ifdef USE_SERIAL
    Serial.println(r);
  #endif
  if (!r) {
      #ifdef USE_SERIAL
        Serial.println("http connection failed");
        Serial.println("we try a restart of the ESP."); Serial.flush();
      #endif
      ESP.restart();
      return;
    }
          // We now create a URI for the request
        String url;
        url = "/";
        url += MY_TRACE_NAME;
        url += "/add.php";
        url += "?u=";
        url += myApiKey;
        url += "&x=";
        url += dataname;
        url += ",";
        url += datavalue;
        //url += uptime_s;
        //url += "s";
        #ifdef USE_SERIAL
          Serial.print("Requesting URL: ");
          Serial.println(url);
        #endif

        // This will send the request to the server
        myHttpClient.print(String("GET ") + url + " HTTP/1.1\r\n" +
                     "Host: " + httphost + "\r\n" +
                     "Connection: close\r\n\r\n");
        unsigned long timeout = millis();
        while (myHttpClient.available() == 0) {
            if (millis() - timeout > 5000) {
                #ifdef USE_SERIAL
                  Serial.println(">>> Client Timeout !");
                #endif
                myHttpClient.stop();
                return;
            }
        }

        // Read all the lines of the reply from server and print them to Serial
        while(myHttpClient.available()) {
            String line = myHttpClient.readStringUntil('\r');
            unsigned int L;
            char charBuf[500];
            line.toCharArray(charBuf, 500) ;
            #ifdef USE_SERIAL
              Serial.print(line);
            #endif
        }
        /* Todo: do we need to disconnect here?
           https://reference.arduino.cc/reference/en/libraries/wifi/ */
        #ifdef USE_SERIAL
          Serial.println("disconnecting the http");
        #endif
        myHttpClient.stop();
}

void sendDataViaHttp(void) {
  char s[100];
  float f;
  f=u_batt_mV;
  f/=1000;
  snprintf(s, 80,  "%1.3f", f); /* attention: Do not use spaces here, because spaces are not allowed in the URL. */
  addValueToWebTrace("lora25_ubatt", s);
  f=u_aux_mV;
  f/=1000;
  snprintf(s, 80,  "%1.3f", f); /* attention: Do not use spaces here, because spaces are not allowed in the URL. */
  addValueToWebTrace("lora25_uaux", s);
}

void sendStatusDataViaHttp(void) {
  char s[100];
  float f;
  f=millis();
  f/=1000;
  snprintf(s, 80,  "%1.2f", f); /* attention: Do not use spaces here, because spaces are not allowed in the URL. */
  addValueToWebTrace("loragw3_uptime", s);
}
#endif /* USE_WIFI */

void checkForReceivedData(void) {
  uint16_t calculatedSum, i;
  uint16_t nTransmitCounts2, txpower_dBm2, u_batt_mV2, u_aux_mV2, sum2;
  if (rxDataAvailable) {
    rxDataAvailable=0;
    /* data consistency risk: we are using variables which are shared between interrupt context and task context.
       If we would get an receive interrupt during the evaluation, the data could be broken.
       Since the messages have a air time of some 10ms, the risk is low that this case really happens. */
    #ifdef USE_SERIAL
      Serial.print("RX: rssi "+ String(rxRssi) + "  snr " + String(rxSnr) + " len " + String(myRxBufferLen) + " ");
      Serial.print("rxPacketSizeOriginal:" + String(rxPacketSizeOriginal) + " ");

      for (i=0; i<8; i++) {
        Serial.print(String(myRxBuffer[i]) + " ");
      }
      Serial.println();
    #endif
    nTransmitCounts = myRxBuffer[0]; nTransmitCounts<<=8; nTransmitCounts+=myRxBuffer[1];
    txpower_dBm = myRxBuffer[2]; txpower_dBm<<=8; txpower_dBm+=myRxBuffer[3];
    u_batt_mV = myRxBuffer[4]; u_batt_mV<<=8; u_batt_mV+=myRxBuffer[5];
    u_aux_mV = myRxBuffer[6]; u_aux_mV<<=8; u_aux_mV+=myRxBuffer[7];
    sum = myRxBuffer[8]; sum<<=8; sum+=myRxBuffer[9];
    nTransmitCounts2 = myRxBuffer[16]; nTransmitCounts2<<=8; nTransmitCounts2+=myRxBuffer[17];
    txpower_dBm2 = myRxBuffer[18]; txpower_dBm2<<=8; txpower_dBm2+=myRxBuffer[19];
    u_batt_mV2 = myRxBuffer[20]; u_batt_mV2<<=8; u_batt_mV2+=myRxBuffer[21];
    u_aux_mV2 = myRxBuffer[22]; u_aux_mV2<<=8; u_aux_mV2+=myRxBuffer[23];
    sum2 = myRxBuffer[24]; sum2<<=8; sum2+=myRxBuffer[25]; /* sum2 is wrongly filled by the transmitter. Ignore it for the moment. */
    calculatedSum = nTransmitCounts+txpower_dBm+u_batt_mV+u_aux_mV;
    #ifdef USE_SERIAL
     Serial.print("nTransmitCounts " + String(nTransmitCounts)
      + ", txpower_dBm " + String(txpower_dBm)
      + ", u_batt_mV " + String(u_batt_mV)
      + ", u_aux_mV " + String(u_aux_mV)
      + ", sum " + String(sum));
     Serial.println();
     Serial.print("nTransmitCounts2 " + String(nTransmitCounts2)
      + ", txpower_dBm2 " + String(txpower_dBm2)
      + ", u_batt_mV2 " + String(u_batt_mV2)
      + ", u_aux_mV2 " + String(u_aux_mV2)
      + ", sum2 " + String(sum2));
     Serial.println();
    #endif
    if (calculatedSum==sum) {
      #ifdef USE_SERIAL
        Serial.println(" checksum ok");
      #endif
      if ((nTransmitCounts==nTransmitCounts2) && (txpower_dBm==txpower_dBm2) && (u_batt_mV==u_batt_mV2) && (u_aux_mV==u_aux_mV2)) {
        #ifdef USE_SERIAL
          Serial.println(" plausibility ok");
        #endif
        sendDataViaHttp();
      } else {
        #ifdef USE_SERIAL
          Serial.println(" plausibility ERROR");
        #endif 
      }
    } else {
      #ifdef USE_SERIAL
        Serial.println(" checksum ERROR");
      #endif
    }
  }
}

void blinkN(uint8_t n) {
  uint8_t i;
  for (i=1; i<=n; i++) {
    digitalWrite(LED_PORT, 0); /* low active, ON */
    delay(250);
    digitalWrite(LED_PORT, 1); /* off */
    delay(250);
  }
  delay(800);
}

void setup() {
  pinMode(LED_PORT, OUTPUT);
  blinkN(1);
  Serial.begin(57600);                   // initialize serial
  blinkN(2);
  delay(500); /* The while (!Serial); cannot be used here, because the ESP32C3 hangs forever if no USB is connected. */
  blinkN(3);
  makeSureThatWifiIsConnected();
  blinkN(4);
  #ifdef USE_SERIAL
    Serial.println("----------- WiFi Info ------------");
    WiFi.printDiag(Serial);
    Serial.println();
  #endif
  LoRa.setPins(csPin, resetPin, irqPin);
  if (!LoRa.begin(frequency)) {
    #ifdef USE_SERIAL
      Serial.println("LoRa init failed. Check your connections.");
    #endif
    while (true) {
        blinkN(9);
    };                       // if failed, do nothing
  }
  blinkN(5);

  #ifdef USE_SERIAL
    Serial.println("LoRa init succeeded.");
    Serial.println();
    Serial.println("LoRa Simple Receiver");
    Serial.println("Only receive messages from nodes");
    Serial.println("Tx: invertIQ enable");
    Serial.println("Rx: invertIQ disable");
    Serial.println();
  #endif

  LoRa.onReceive(onReceive);
  LoRa.onTxDone(onTxDone);
  LoRa_rxMode();
  #ifdef USE_WATCHDOG
    #ifdef USE_SERIAL
      Serial.println("configuring the watchdog");
    #endif
          // v3 board manager detected
          // Create and initialize the watchdog timer(WDT) configuration structure
            esp_task_wdt_config_t wdt_config = {
                .timeout_ms = MY_WDT_TIMEOUT * 1000, // Convert seconds to milliseconds
                .idle_core_mask = 1 << 0,         // Monitor core 1 only
                .trigger_panic = 1             // Enable panic
            };
          // Initialize the WDT with the configuration structure
          esp_task_wdt_deinit(); //wdt is enabled by default, so we need to deinit it first
          esp_task_wdt_init(&wdt_config);       // Pass the pointer to the configuration structure
          esp_task_wdt_add(NULL);               // Add current thread to WDT watch    
          esp_task_wdt_reset();                 // reset timer
  #ifdef USE_SERIAL
    Serial.println("done");
  #endif
  #endif
}

void task100ms(void) {
  LED_divider++;
  if (LED_divider>=10) {
    LED_divider=0;
  }
  if (LED_divider==0) digitalWrite(LED_PORT, 0); else digitalWrite(LED_PORT, 1);
}

void loop() {
  checkForReceivedData();
  delay(10); /* 10ms sleeping */

  #ifdef USE_WATCHDOG
  // resetting WDT every 2s, but 5 times only, to test that the watchdog works
  if ((millis() - lastWatchdogTriggerTime >= 2000) && (watchdogTriggerCount < 5)) {
      #ifdef USE_SERIAL
        Serial.println("Resetting WDT...");
      #endif
      esp_task_wdt_reset();
      lastWatchdogTriggerTime = millis();
      watchdogTriggerCount++;
      if (watchdogTriggerCount == 5) {
        #ifdef USE_SERIAL
         Serial.println("Stopping WDT reset. CPU should reboot in 3s");
        #endif
      }
  }
  #endif
  if ((millis() - lastStatusLogTime >= STATUS_LOG_CYCLE_TIME_MS)) {
      #ifdef USE_SERIAL
        Serial.println("Logging status...");
      #endif
      lastStatusLogTime +=STATUS_LOG_CYCLE_TIME_MS;
      sendStatusDataViaHttp();
  }
  if ((millis() - last100msTime >= 100)) {
      last100msTime +=100;
      task100ms();
  }
  

}

void LoRa_rxMode(){
  LoRa.disableInvertIQ();               // normal mode
  LoRa.receive();                       // set receive mode
}

void LoRa_txMode(){
  LoRa.idle();                          // set standby mode
  LoRa.enableInvertIQ();                // active invert I and Q signals
}

void LoRa_sendMessage(String message) {
  LoRa_txMode();                        // set tx mode
  LoRa.beginPacket();                   // start packet
  LoRa.print(message);                  // add payload
  LoRa.endPacket(true);                 // finish packet and send it
}

void onReceive(int packetSize) {
  /* This runs in interrupt context. It's not a good idea to do longer things here like serial print. */
  uint8_t rxByte;
  rxPacketSizeOriginal = packetSize;
  myRxBufferLen=0;
  while (LoRa.available()) {
    rxByte = LoRa.read();
    if (myRxBufferLen<MYRXBUFFER_SIZE) {
      myRxBuffer[myRxBufferLen]=rxByte;
      myRxBufferLen++;
    }
  }
  rxRssi = LoRa.packetRssi(); /* Returns the averaged RSSI of the last received packet (dBm). */
  rxSnr = LoRa.packetSnr(); /* Returns the estimated SNR of the received packet in dB. */
  rxDataAvailable=1;
}


void onTxDone() {
  Serial.println("TxDone");
  LoRa_rxMode();
}



