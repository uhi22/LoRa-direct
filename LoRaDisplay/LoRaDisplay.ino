/*
  LoRa Simple receiver
  Based on the example "LoRa Simple Gateway/Node Exemple"

  Hardware: either
  * Arduino Pro Mini, or
  * ESP32 DEVKIT according to https://randomnerdtutorials.com/esp32-lora-rfm95-transceiver-arduino-ide/

*/

#define USE_ESP32DEVKIT


#include <SPI.h>              // include libraries
#include <LoRa.h> /* Lib: LoRa by Sandeep Mistry */

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

uint16_t nTransmitCounts, txpower_dBm, u_batt_mV, u_aux_mV, sum;

void setup() {
  Serial.begin(57600);                   // initialize serial
  while (!Serial);

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
}

void loop() {
  if (runEvery(5000)) { // repeat every 5000 millis

    //String message = "HeLoRa World! ";
    //message += "I'm a Gateway! ";
    //message += millis();

    //LoRa_sendMessage(message); // send a message

    //Serial.println("Send Message!");
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
  //String message = "";
  uint8_t rxByte;
  uint8_t i;
  //uint16_t messageLen = 0;
  uint16_t calculatedSum;
  myRxBufferLen=0;
  while (LoRa.available()) {
    rxByte = LoRa.read();
    //message += (char)rxByte;
    //messageLen++;
    myRxBuffer[myRxBufferLen]=rxByte;
    myRxBufferLen++;
  }

  int rssi = LoRa.packetRssi(); /* Returns the averaged RSSI of the last received packet (dBm). */
  float snr = LoRa.packetSnr(); /* Returns the estimated SNR of the received packet in dB. */
  Serial.print("RX: rssi "+ String(rssi) + "  snr " + String(snr) + " len " + String(myRxBufferLen) + " ");
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
  } else {
    Serial.println(" checksum ERROR");
  }
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

