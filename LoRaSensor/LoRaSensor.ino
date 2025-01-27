/*
  LoRa Simple Sensor

  Based on the example "LoRa Simple Gateway/Node Exemple"

*/

#include <SPI.h>              // include libraries
#include <LoRa.h> /* Lib: LoRa by Sandeep Mistry */
#include <LowPower.h> /* Lib: LowPower_LowPowerLab by LowPowerLab 2.2 */

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
uint8_t nFastSleep = 0;

#define WKREASON_RESET 1
#define WKREASON_INT0 2
#define WKREASON_INT1 3
#define WKREASON_8S 4

uint8_t myWakeupReason;
uint16_t nWakeCycles;
uint16_t nTransmitCounts;

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
  myWakeupReason=WKREASON_RESET; /* Reset */
  Serial.begin(57600);                   // initialize serial
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



void loop() {
  if (myWakeupReason!=WKREASON_8S) {
    nFastSleep=0; /* react immediately in case of Power-On or interrupt. */
  }
  if (nFastSleep>0) { /* count the "nothing-to-do" cycles */
    nFastSleep--;
  }
  if (nFastSleep==0) {
    /* There was a timer wakeup and the number of fast sleep cycles elapsed, or
     * we had in interrupt wakeup, then transmit immediately.
     */
    readTheSensors(0);
    switch (nTransmitCounts & 3) {
       case 0: txpower_dBm=17; break;
       case 1: txpower_dBm=11; break;
       case 2: txpower_dBm=5; break;
       case 3: txpower_dBm=0; break;
    }
    LoRa.setTxPower(txpower_dBm);
    nTransmitCounts++;
    String message = "HeLoRa World! ";
    message += "up_s=" + String(millis()/1000);
    message += ",txp_dBm="+String(txpower_dBm);
    message += ",ubatt_mV=" + String(u_batt_mV);
    message += ",u_aux_mV=" + String(u_aux_mV);
    Serial.println("will send " + message); Serial.flush();
    LoRa_sendMessage(message); // send a message
    /* Timing constraint: We need to wait here before going to sleep, because the modem
       is still transmitting (visible on the power consumption), even if it already
       fired the "transmit complete" */
    delay(150); /* while the modem is transmitting, stay awake. */
    Serial.println(".."); Serial.flush();

    nFastSleep = 3; /* 3*8s=24s */
  } else {
    /* this was a timer wakeup, but in this cycle we do not want to do anything and
     * go to sleep as fast as possible. */
    #ifdef WITH_SERIAL_DEBUG
      Serial.println(F("Q")); Serial.flush(); /* Q like "Quick go to sleep". And wait until to serial transmission finished before going to sleep */
    #endif
  }
  myWakeupReason=WKREASON_8S;
  //attachInterrupt(0, wakeUpFunction0, FALLING); /* Allow wake up pin to trigger interrupt on edge */
  //attachInterrupt(1, wakeUpFunction1, FALLING); /* Allow wake up pin to trigger interrupt on edge */
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF); /* go to sleep */
  /* we reach this part when we have been woken up. */
  /* Seems we get an wakeup from the LoRa interrupt here, too. Not generally, but depending on the
     timing between transmission and sleep. So after first going to sleep,
     we immediate wake up, and so the count of intended 8s cycles is wrong by one. */
  nWakeCycles++;
  //detachInterrupt(0); /* disable external interrupt on wakeup pin */
  //detachInterrupt(1); /* disable external interrupt on wakeup pin */
  #ifdef WITH_SERIAL_DEBUG
    Serial.println(F("W")); /* W like "Woke up" */
  #endif
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
  //Serial.println("TxDone"); /* the onTxDone is called in interrupt context. So serial.print is not a good idea. */
  LoRa_rxMode();
}


