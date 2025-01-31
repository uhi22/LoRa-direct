/*
  LoRa Simple receiver
  Based on the example "LoRa Simple Gateway/Node Exemple"

  Hardware: either
  * Arduino Pro Mini, or
  * ESP32 DEVKIT according to https://randomnerdtutorials.com/esp32-lora-rfm95-transceiver-arduino-ide/

  Watchdog concept explained: https://forum.arduino.cc/t/esp32-und-watchdog-callback/1072863/23 but this is for the old
  espressif API. The differences between the old and the new API are explained here: https://esp32.com/viewtopic.php?t=40261
  and https://iotassistant.io/esp32/fixing-error-hardware-wdt-arduino-esp32/
  But: The watchdog hits, but the software does not restart. Reason unclear. So we do not use the watchdog at the moment.
*/

#define USE_ESP32DEVKIT


#include <SPI.h>              // include libraries
#include <LoRa.h> /* Lib: LoRa by Sandeep Mistry */

//#define USE_WATCHDOG

#ifdef USE_ESP32DEVKIT
  #include <WiFi.h> /* Wifi connection for the ESP32 */
  #include "privatesettings.h" /* wifi credentials, server name etc */
  #ifdef USE_WATCHDOG
    #include <esp_task_wdt.h>

    #define MY_WDT_TIMEOUT 3    /* 3 seconds watchdog timeout */
    int lastWatchdogTriggerTime;
    int watchdogTriggerCount;
    #define CONFIG_FREERTOS_NUMBER_OF_CORES 2 /* ESP32 with 2 cores */
  #endif
#endif

const long frequency = 868E6;  // LoRa Frequency

#ifdef USE_ESP32DEVKIT
  //define the pins used by the transceiver module
  const int csPin = 5;     // LoRa radio chip select
  const int resetPin = 14; // LoRa radio reset
  const int irqPin = 2;    // LoRa hardware interrupt pin
#else
  /* pins on Arduino Pro Mini */
  const int csPin = 10;          // LoRa radio chip select
  const int resetPin = 9;        // LoRa radio reset
  const int irqPin = 2;          // LoRa hardware interrupt pin
#endif

uint8_t myRxBuffer[64];
uint8_t myRxBufferLen;
uint8_t rxDataAvailable;
int rxRssi;
float rxSnr;

uint16_t nTransmitCounts, txpower_dBm, u_batt_mV, u_aux_mV, sum;

#ifdef USE_ESP32DEVKIT
// Use WiFiClient class to create TCP connections
WiFiClient myHttpClient;

void myWifiConnect(void) {
   // We start by connecting to a WiFi network
   if (1) {
        WiFi.mode(WIFI_STA);
        Serial.print("Connecting to ");
        Serial.println(ssid);
        WiFi.begin(ssid, password);
        while (WiFi.status() != WL_CONNECTED) {
            delay(500);
            Serial.print(".");
        }
        Serial.println("");
        Serial.println("WiFi connected");
        Serial.println("IP address: ");
        Serial.println(WiFi.localIP());
   } else {
        Serial.println("---WILL NOT CONNECT TO WIFI -----");
   }
}

void makeSureThatWifiIsConnected(void) {
   if (WiFi.status() == WL_CONNECTED) {
      Serial.println("makeSureThatWifiIsConnected: WiFi is already connected.");
   } else {
      Serial.println("makeSureThatWifiIsConnected: Will try to connect.");
      myWifiConnect();
   }
}

void addValueToWebTrace(char *dataname, char *datavalue) {
    Serial.print("addValueToWebTrace ");
    //Serial.print(tracename);Serial.print(" ");
    Serial.print(dataname);Serial.print(" ");
    Serial.println(datavalue);
    makeSureThatWifiIsConnected();
    Serial.print("connecting to ");
    Serial.println(httphost);
    int r = myHttpClient.connect(httphost, httpPort);
    Serial.println(r);
    if (!r) {
        Serial.println("http connection failed");
        Serial.println("we try a restart of the ESP."); Serial.flush();
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
        Serial.print("Requesting URL: ");
        Serial.println(url);

        // This will send the request to the server
        myHttpClient.print(String("GET ") + url + " HTTP/1.1\r\n" +
                     "Host: " + httphost + "\r\n" +
                     "Connection: close\r\n\r\n");
        unsigned long timeout = millis();
        while (myHttpClient.available() == 0) {
            if (millis() - timeout > 5000) {
                Serial.println(">>> Client Timeout !");
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
            Serial.print(line);
        }
        /* Todo: do we need to disconnect here?
           https://reference.arduino.cc/reference/en/libraries/wifi/ */
        Serial.println("disconnecting the http");
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
#endif /* USE_ESP32DEVKIT */

void checkForReceivedData(void) {
  uint16_t calculatedSum, i;
  if (rxDataAvailable) {
    rxDataAvailable=0;
    /* data consistency risk: we are using variables which are shared between interrupt context and task context.
       If we would get an receive interrupt during the evaluation, the data could be broken.
       Since the messages have a air time of some 10ms, the risk is low that this case really happens. */
    Serial.print("RX: rssi "+ String(rxRssi) + "  snr " + String(rxSnr) + " len " + String(myRxBufferLen) + " ");
    //for (i=0; i<8; i++) {
    //  Serial.print(String(myRxBuffer[i]) + " ");
    //}
    //Serial.println();
    nTransmitCounts = myRxBuffer[0]; nTransmitCounts<<=8; nTransmitCounts+=myRxBuffer[1];
    txpower_dBm = myRxBuffer[2]; txpower_dBm<<=8; txpower_dBm+=myRxBuffer[3];
    u_batt_mV = myRxBuffer[4]; u_batt_mV<<=8; u_batt_mV+=myRxBuffer[5];
    u_aux_mV = myRxBuffer[6]; u_aux_mV<<=8; u_aux_mV+=myRxBuffer[7];
    sum = myRxBuffer[8]; sum<<=8; sum+=myRxBuffer[9];
    calculatedSum = nTransmitCounts+txpower_dBm+u_batt_mV+u_aux_mV;
    Serial.print("nTransmitCounts " + String(nTransmitCounts)
      + ", txpower_dBm " + String(txpower_dBm)
      + ", u_batt_mV " + String(u_batt_mV)
      + ", u_aux_mV " + String(u_aux_mV));
    if (calculatedSum==sum) {
      Serial.println(" checksum ok");
      sendDataViaHttp();
    } else {
      Serial.println(" checksum ERROR");
    }
  }
}



void setup() {
  Serial.begin(57600);                   // initialize serial
  while (!Serial);
  makeSureThatWifiIsConnected();
  Serial.println("----------- WiFi Info ------------");
  WiFi.printDiag(Serial);
  Serial.println();

  LoRa.setPins(csPin, resetPin, irqPin);
  if (!LoRa.begin(frequency)) {
    Serial.println("LoRa init failed. Check your connections.");
    while (true);                       // if failed, do nothing
  }

  Serial.println("LoRa init succeeded.");
  Serial.println();
  Serial.println("LoRa Simple Receiver");
  Serial.println("Only receive messages from nodes");
  Serial.println("Tx: invertIQ enable");
  Serial.println("Rx: invertIQ disable");
  Serial.println();

  LoRa.onReceive(onReceive);
  LoRa.onTxDone(onTxDone);
  LoRa_rxMode();
  #ifdef USE_WATCHDOG
  Serial.println("configuring the watchdog");
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
  Serial.println("done");
  #endif
}

void loop() {
  checkForReceivedData();
  if (runEvery(5000)) { // repeat every 5000 millis

    //String message = "HeLoRa World! ";
    //message += "I'm a Gateway! ";
    //message += millis();

    //LoRa_sendMessage(message); // send a message

    //Serial.println("Send Message!");
  }

  #ifdef USE_WATCHDOG
  // resetting WDT every 2s, but 5 times only, to test that the watchdog works
  if ((millis() - lastWatchdogTriggerTime >= 2000) && (watchdogTriggerCount < 5)) {
      Serial.println("Resetting WDT...");
      esp_task_wdt_reset();
      lastWatchdogTriggerTime = millis();
      watchdogTriggerCount++;
      if (watchdogTriggerCount == 5) {
        Serial.println("Stopping WDT reset. CPU should reboot in 3s");
      }
  }
  #endif
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
  myRxBufferLen=0;
  while (LoRa.available()) {
    rxByte = LoRa.read();
    myRxBuffer[myRxBufferLen]=rxByte;
    myRxBufferLen++;
  }
  rxRssi = LoRa.packetRssi(); /* Returns the averaged RSSI of the last received packet (dBm). */
  rxSnr = LoRa.packetSnr(); /* Returns the estimated SNR of the received packet in dB. */
  rxDataAvailable=1;
}


void onTxDone() {
  Serial.println("TxDone");
  LoRa_rxMode();
}

boolean runEvery(unsigned long interval)
{
  static unsigned long previousMillis = 0;
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval)
  {
    previousMillis = currentMillis;
    return true;
  }
  return false;
}

