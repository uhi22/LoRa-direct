/*
  LoRa Simple Sensor

  Based on the example "LoRa Simple Gateway/Node Exemple"

*/

#include <SPI.h>              // include libraries
#include <LoRa.h>

#define WITH_SERIAL_DEBUG

/* Analog Pins */
#define AdcVccPin  A0
#define AdcUAuxPin A1

/* Voltage divider 150k und 33k (and 10nF in parallel), with URef=1.1V, gives 5.962mV/Digit */
#define VCC_MEASUREMENT_RESOLUTION_UV 5995

/* Voltage divider 470k und 15k (and 10nF in parallel), with URef=1.1V, gives 34.767 mV/Digit */
#define UAUX_MEASUREMENT_RESOLUTION_UV 35180


const long frequency = 868E6;  // LoRa Frequency. In Europe we are at 868 MHz. */

const int csPin = 10;          // LoRa radio chip select
const int resetPin = 9;        // LoRa radio reset
const int irqPin = 2;          // change for your board; must be a hardware interrupt pin

uint16_t u_batt_mV, u_aux_mV;
int8_t txpower_dBm;

void readTheSensors(uint8_t blShow) {
    #define WAIT_TIME_AFTER_REFERENCE_CHANGE_MS 10 /* bewährte 10ms */
    uint32_t tmp32;
    analogReference(INTERNAL); /* interne 1.1V Refererenzspannung beim ATMega 328 */
    delay(WAIT_TIME_AFTER_REFERENCE_CHANGE_MS);
    tmp32 = analogRead(AdcVccPin);
    tmp32 += analogRead(AdcVccPin);
    tmp32 += analogRead(AdcVccPin);
    tmp32 += analogRead(AdcVccPin);
    tmp32>>=2;
    tmp32*=VCC_MEASUREMENT_RESOLUTION_UV;
    tmp32/=1000; /* 1mV Auflösung */
    u_batt_mV=tmp32;

    tmp32 = analogRead(AdcUAuxPin);
    tmp32 += analogRead(AdcUAuxPin);
    tmp32 += analogRead(AdcUAuxPin);
    tmp32 += analogRead(AdcUAuxPin);
    tmp32>>=2;
    tmp32*=UAUX_MEASUREMENT_RESOLUTION_UV;
    tmp32/=1000; /* 1mV Auflösung */
    u_aux_mV=tmp32;

    analogReference(DEFAULT); /* VCC vom Controller als Referenz, weil NTC-Pullup an VCC hängt */
    /* Das Umschalten von interner Referenz auf VCC-Referenz scheint scnnell zu gehen. Nur
     *  10ms warten:
     */
    delay(WAIT_TIME_AFTER_REFERENCE_CHANGE_MS);

    /* here we could do other measurments, like NTC, humidity, ... */

    /* gleich wieder auf intern umschalten, so dass während des Raussendens
     *  die Spannung von 3 auf 1.1V abklingen kann. Aber warten alleine reicht nicht zum
     *  Abklingen. Erst durch aktive Wandlung scheint die Spannung runter zu gehen. Daher
     *  ein paar Wandlungen starten:
     */
    analogReference(INTERNAL); /* interne 1.1V Refererenzspannung beim ATMega 328 */
    for (int w=0; w<10; w++) {
       tmp32 += analogRead(AdcVccPin);
    }

    #ifdef WITH_SERIAL_DEBUG
      if (blShow) {
      Serial.print(F("mV:"));
      Serial.print(u_batt_mV);
      Serial.println();
      }
    #endif
}



void setup() {
  Serial.begin(9600);                   // initialize serial
  while (!Serial);

  LoRa.setPins(csPin, resetPin, irqPin);

  if (!LoRa.begin(frequency)) {
    Serial.println("LoRa init failed. Check your connections.");
    while (true);                       // if failed, do nothing
  }

  Serial.println("LoRa init succeeded.");
  Serial.println();
  Serial.println("LoRa Sensor");

  LoRa.onReceive(onReceive);
  LoRa.onTxDone(onTxDone);
  LoRa_rxMode();
}

uint16_t oneSecondCounter;

void loop() {
  if (runEvery(1000)) { // repeat every 1000 millis
    readTheSensors(0);
    switch (oneSecondCounter & 3) {
       case 0: txpower_dBm=17; break;
       case 1: txpower_dBm=11; break;
       case 2: txpower_dBm=5; break;
       case 3: txpower_dBm=0; break;
    }
    LoRa.setTxPower(txpower_dBm);
    oneSecondCounter++;
    String message = "HeLoRa World! ";
    message += "up_s=" + String(millis()/1000);
    message += ",txp_dBm="+String(txpower_dBm);
    message += ",ubatt_mV=" + String(u_batt_mV);
    message += ",u_aux_mV=" + String(u_aux_mV);
    
    LoRa_sendMessage(message); // send a message

    Serial.println("Sent Message " + message);
  }
}

void LoRa_rxMode(){
  LoRa.enableInvertIQ();                // active invert I and Q signals
  LoRa.receive();                       // set receive mode
}

void LoRa_txMode(){
  LoRa.idle();                          // set standby mode
  LoRa.disableInvertIQ();               // normal mode
}

void LoRa_sendMessage(String message) {
  LoRa_txMode();                        // set tx mode
  LoRa.beginPacket();                   // start packet
  LoRa.print(message);                  // add payload
  LoRa.endPacket(true);                 // finish packet and send it
}

void onReceive(int packetSize) {
  String message = "";

  while (LoRa.available()) {
    message += (char)LoRa.read();
  }

  Serial.print("Node Receive: ");
  Serial.println(message);
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

