/* SPDX-FileCopyrightText: 2025 Casijn Broerse <casijn@protonmail.com> */
/* SPDX-License-Identifier: GPL-3.0-or-later */


/*
 * E-rubab sockketIO client v0.8.7
 * By Casijn Broerse, 21 July 2025
 * Adapted from WebSocketClientSocketIOack.ino, AdaFruit and randomnerdtutorials.com examples
 * 
 * Works on laptop + node.js socketio server + ESP32
 * Sends notes as string (chat) message for Unreal Engine
 * Senses Touch pad via MPR121 breakout, connected over I2C
 * Snares are IR Proximity (line following) sensors RPR220 with threshold and min. interval set
 * Also tested with Sparkfun QRE1113 "QA" IR breakouts
 * Additional (debug) note inputs: 3 digital touch buttons to test UnReal VR interaction
 * Refactored functions to fix code duplication, stores data in arrays
 * Calibrated for 3x RPR220 on custom PCB
 * Removed deep_sleep mode, FSR code, MPU6050 I2C gyroscope
 * Added SPDX header, see https://reuse.software/faq/
 */

#include <Arduino.h>

#include <WiFi.h>

#include <Wire.h>
#include <Adafruit_MPR121.h> // lib for touch sensor breakout

#include "driver/rtc_io.h"

#include <ArduinoJson.h>

#include <SocketIOclient.h>

#include "arduino_secrets.h"

WiFiClient client;

const char credits[] = "ESP32 E-rubab by Casijn Broerse, 2025";
const char sio_host[] = "172.19.65.69"; // set for the laptop/PC running socket.io server

// sensitive data in the Secret tab/arduino_secrets.h
const char ssid[] = SECRET_SSID;      // network SSID (name)
const char password[] = SECRET_PASS;  // network wifi wachtwoord (use for WPA, or use as key for WEP)


// Global variable definitions
int status = WL_IDLE_STATUS;

// SocketIO server details
bool connected; // socketIO server status
// IP address set at top of code
const uint16_t sio_port = 3000;
SocketIOclient socketIO;

unsigned long messageTimestamp = 0;
int msgId = 0;  // to filter out duplicates already sent

bool sendData(char eventName[],  int data1, int data2=0, int data3=0); // in header set default values for optional parameters
char PLAYSTR[] = "rb_notestring";
// char MENUSTR[] = "rb_menustring";
char CHAT[] = "chat message";
int maxMessageLength = 1;

#define USE_SERIAL Serial
#define onboard_LED 2

// for comparing MPR121 touch states
// from https://github.com/adafruit/Adafruit_MPR121/
#ifndef _BV
#define _BV(bit) (1 << (bit)) 
#endif

//===== I/O definitions

// FRETS:
// capacitive touch threshold value on capacitive touch areas
// single value, applied to all fret touch sensors
// settable in lib, but MPR121 defaults work fine

// MPR121 breakout board I2C setup, use default SCL=22 & SDA=21 pins
Adafruit_MPR121 cap = Adafruit_MPR121(); // requires cap.autoconfig() in setup()

// Keep track of the last pads touched, returned as 12 bit integer
// every bit is a pad state: 0000 0000 0010 0000 Pad #5 touched, 0000 0000 0000 0001 Pad #0 touched.
// so defined as uint16_t, not default int = uint8_t
uint16_t currTouched = 0;
uint16_t lastTouched = 0; // check to enter sleep mode
const int bitPositions[3][4] = { { 0, 1, 2, 3 },
                       { 4, 5, 6, 7 },
                       { 8, 9, 10, 11 } };
// Used to fetch corresponding bit from MPR121 byte
// Equivalent to the lowest bit position for the index


// SNARES aka "musical strings":
// Add 30mm piece of yellow plastic cable insulation over sensor around transparent snares
#define NUM_SNARES 3 // the number of snares, starting count at 1

const int snare1IrPin = 34;  // IR sensor1 ADC on D34 - top left (with USB @ bottom)
const int snare2IrPin = 35;  // IR sensor2 ADC on D35 - same row left, a bit down
const int snare3IrPin = 32;  // IR sensor3 ADC on D32 - same row left, a bit down

// IR sensor on 3.3V with:
// 10k Ohm extra pulldown resistor for Sparkfun QA pin "S"
// 68k Ohm pullup resistor for RPR220 pin 4, and 330 Ohm resister for LED pin 1-2

// for RPR220, adjust if distance to IR sensor > 2mm
// use dynamic setting during startup
int SNARE_THRESHOLD1 = 1000; // base value
int SNARE_THRESHOLD2 = 1000;
int SNARE_THRESHOLD3 = 1000;
int snareThresholds[NUM_SNARES] = {SNARE_THRESHOLD1, SNARE_THRESHOLD2, SNARE_THRESHOLD3};
const int THRESHOLD_MARGIN = 200; // sensitivity for IR snare sensor margin above base (rest)
// Base value for Sparkfun QA breakout: 1323. For RPR220: 300-511

const int START_COUNTDOWN = 100; // Minimum wait cycles between playing the same snare

// Arrays to hold the pin numbers, states etc.
int snarePins[NUM_SNARES] = {snare1IrPin, snare2IrPin, snare3IrPin};

int prevSnareIrStates[NUM_SNARES] = {HIGH, HIGH, HIGH}; // Initial states for IR snare sensors
int snareValues[NUM_SNARES] = {0, 0, 0};
int snareCounters[NUM_SNARES] = {0, 0, 0};
bool snareHighs[NUM_SNARES] = {false, false, false}; // these are defined above


// end of variable definitions


void setup() {
    USE_SERIAL.begin(115200);  // 921600); // lower as advised
    USE_SERIAL.setDebugOutput(true);
    
    // initialize ESP32 built in LED as an output.
    pinMode(onboard_LED, OUTPUT); // ESP32 WROOM v1
    // temporary snare buttons

    for (uint8_t t = 4; t > 0; t--) {
        USE_SERIAL.printf("[SETUP] BOOT WAIT %d...\n", t);
        USE_SERIAL.flush();
        delay(1000); // required for WiFi connect succes
    }

    USE_SERIAL.println(credits); USE_SERIAL.println();


    // INPUTS
    
    // Frets, 4 per snare (="string") 
    // pads 0-3 under Snare 1, pads 4-7 under Snare 2, pads 8-11 under Snare 3

    // Fret inputs use touch/analog input via MPR121 breakout over I2C, no config
    USE_SERIAL.print("MPR121 Capacitive Touch Sensor breakout board "); 
    // use default MPR121 address: 0x5A
      Wire.begin(); // already running?
    if (!cap.begin(0x5A, &Wire)) {
      USE_SERIAL.println("not found. Check the frets wiring");
      while (1);
    }
    USE_SERIAL.println("found"); USE_SERIAL.println();
    cap.setAutoconfig(true); // adjust threshold per touch area

    // Snares, 1 = top snare (low), 3 = bottom snare (high)
    // uses analog input on GPIO32/35/34
    USE_SERIAL.println("IR snare sensors active.");
    for (int i = 0; i < NUM_SNARES; i++) {
        snareValues[i] = analogRead(snarePins[i]);
        snareThresholds[i] = max(snareValues[i] + THRESHOLD_MARGIN, 200); // prevent setting threshold to just 200
    }
    USE_SERIAL.print("Snare 1-3 IR base values: 1="); // print initial PSR value
    USE_SERIAL.print(snareValues[0]);
    USE_SERIAL.print(" 2=");
    USE_SERIAL.print(snareValues[1]);
    USE_SERIAL.print(" 3=");
    USE_SERIAL.println(snareValues[2]);

    // Auto-calibrate: set threshold per snare sensor a margin above base value
    USE_SERIAL.print("Assigned snare threshold: S1:");
    USE_SERIAL.print(snareThresholds[0]); USE_SERIAL.print(" S2:");
    USE_SERIAL.print(snareThresholds[1]); USE_SERIAL.print(" S3:");
    USE_SERIAL.println(snareThresholds[2]);
    if (max(max(SNARE_THRESHOLD1, SNARE_THRESHOLD2), SNARE_THRESHOLD3) > 4000) {
      USE_SERIAL.println("A Snare sensor seems to be not working. Can't play, sorry");
    }

    // CONNECT to wifi
    digitalWrite(onboard_LED, HIGH);
    USE_SERIAL.println();
    USE_SERIAL.println("*********************************");
    USE_SERIAL.print("Connecting to ");
    USE_SERIAL.println(ssid);

    // Set WiFi to station mode
    WiFi.mode(WIFI_STA);
    // WiFi.disconnect(); // disconnect from an AP if it was previously connected.
    WiFi.begin(ssid, password);

    int j = 0;
    while (WiFi.status() != WL_CONNECTED && j < 40) {
      j++;
      delay(500);
      USE_SERIAL.print(".");
    }
    if (WiFi.status() == WL_CONNECTED) {
      USE_SERIAL.println("\n Connected to WiFi");
      USE_SERIAL.print("ESP32 IP address: ");
      USE_SERIAL.println(WiFi.localIP());
    } else {
      USE_SERIAL.print("\n Sorry, could not connect to WiFi "); USE_SERIAL.println(ssid);
    }
    digitalWrite(onboard_LED, LOW);
  
    // Connect to socket.io server
    digitalWrite(onboard_LED, HIGH);
    // server address, port and URL
    socketIO.begin(sio_host, sio_port, "/socket.io/?EIO=4");
    // this format tells the Arduino lib it wants socketIO, so must include =4
    
    // use HTTP Basic Authorization (optional - for extra security as suggested)
    // webSocket.setAuthorization("username", "password");
    
    digitalWrite(onboard_LED, LOW);
    
    // event handler
    socketIO.onEvent(socketIOEvent);
  }


void loop() {
    socketIO.loop(); // reads incoming, keeps SocketIO active


    // repeat static message - testing
    if (msgId >= 18000){
        msgId = 0;
        // sendString(CHAT, "Hello from esp32"); // causes callback() crash on socketio server
    }
      
    // CHECK THE INPUTS

    // FRETS
    currTouched = cap.touched();
    // will CRASH here if no MPR121, so must DEFINE to activate FRETS, see #else next
    
    // Get the currently touched fret pads from MPR121 cap
    // as a 2 byte int value containing 12 bits 0-11 for 'touched' state of each pad
    // fret state is parsed only when a snare is actually played, see checkSnareIr()

    // Debug MPR121 touch pads to Serial Console
    // prints out the bits from the MPR121 return value nicely formatted
    // for (uint8_t i=0; i<12; i++) {
    //   // if it *is* touched and *wasnt* touched before, mark:
    //   if ((currTouched & _BV(i)) && !(lastTouched & _BV(i)) ) {
    //     // Debug: print out as as bits
    //     USE_SERIAL.print("Pads byte from MPR121: ");
    //     //printBits(currTouched); USE_SERIAL.print(" Pad #");
    //     USE_SERIAL.print(i); Serial.println(" touched.");
    //   }
    //   // if it *was* touched and now *isnt*, mark too:
    //   if (!(currTouched & _BV(i)) && (lastTouched & _BV(i)) ) {
    //     // Debug: print out as as bits
    //     USE_SERIAL.print("Pads byte from MPR121: ");
    //     //printBits(currTouched); USE_SERIAL.print(" Pad #");
    //     USE_SERIAL.print(i); Serial.println(" released.");
    //   }
    // }
    // store touched state for comparison in next loop (debugging only)
    if (currTouched != lastTouched) { // activity/change
        lastTouched = currTouched;    // store for next looop
    }

    // SNARES
    // Check state of snares 1-3 
    for (int i = 0; i < NUM_SNARES; i++) {
        checkSnareIr(i);
    }

    delay(50); // add delay for stability. DEBUG, reset to 10 normal?
}


// SUPPORTING FUNCTIONS

void checkSnareIr(int index) {
    // Determine which pin and variables to use, based on index. Index 0 = snare 1 etc.
    int snareCounter = max(snareCounters[index] - 10, 0); // decrease countdown timer for snare i
    bool snareIsHigh = snareHighs[index];
    // read IR value
    int currentValue = analogRead(snarePins[index]);
    //USE_SERIAL.print(index + 1); USE_SERIAL.print(" "); USE_SERIAL.println(currentValue); // DEBUG ONLY, full Monitor!
    
    // Perform IR snare played logic
    if (currentValue >= snareThresholds[index]) { // no reflection, snare pulled aside
        snareHighs[index] = true;
    } else if ((currentValue < snareThresholds[index]) && (snareCounter == 0)) {
        if (snareHighs[index]) {
            // snare goes from HIGH/OFF to LOW/rest: reset and send note to socketIO 1x
            if (currentValue > 10 && currentValue < 4095) { // don't send from an E-rubab without sensors
                sendNote(index + 1, getFretState(index)); // frets under snare[index] get parsed, send Note
                snareCounter = START_COUNTDOWN;
            } 
            snareHighs[index] = false;
        }
    }
    // Update global variable
    snareCounters[index] = snareCounter;
}


int getFretState(int index) {
    // Parse global currTouched for index (= index=0 for frets under snare 1 etc.)
    // Compressed logic, aided by Mistral lechat AI
    // Same logic for index == 1 in a more readable form:
    //    fret1State = 10; // bitRead() checks a fret state by its index
    //    if (bitRead(currTouched, 3)) { // only highest fret is picked up
    //        fret1State = 14;
    //    } else if (bitRead(currTouched, 2)) {
    //        fret1State = 13;
    //    } else if (bitRead(currTouched, 1)) {
    //        fret1State = 12;
    //    } else if (bitRead(currTouched, 0)) {
    //        fret1State = 11;
    //    }
    
    int baseState = (index + 1) * 10; // e.g. 20 for snare 2 (note: snare2 index = 1 !)
    int fretState = baseState;  // default state with no frets touched

    // Calculate the bit positions for the current index: defined in global array

    // Check each bit position to determine the fret state
    if (bitRead(currTouched, bitPositions[index][3])) {
        fretState = baseState + 4;
    } else if (bitRead(currTouched, bitPositions[index][2])) {
        fretState = baseState + 3;
    } else if (bitRead(currTouched, bitPositions[index][1])) {
        fretState = baseState + 2;
    } else if (bitRead(currTouched, bitPositions[index][0])) {
        fretState = baseState + 1;
    }

    // Return the calculated state
    return fretState;
}


/*  SocketIO functions to send an receive */

/* Send note depending on snare and fret pressed */
void sendNote(int snareNum, int fretPressed) {
    USE_SERIAL.println(" sendNote(S" + String(snareNum) + ",F" + String(fretPressed) + ")");
    // To send note to socketIO as JSON, uncomment next line
    // sendData(PLAYED, snareNum, fretPressed); 

    // Send note to socketIO as TXT
    char payload[] = "S";
    char snr[2];
    itoa(snareNum, snr, 10);
    strcat(payload, snr);
    strcat(payload, "F");
    char frt[2];
    itoa(fretPressed, frt, 10);
    strcat(payload, frt);
    sendString(PLAYSTR, payload);  
    // sendString(MENUSTR, payload); // send same payload, but sent as a user menu selection
    // not required on client as a copy is emitted by the socketIO server
}

/* Send a note as String to the socketIO Server + callback (counter) */
bool sendString(char eventName[],  char textString[]) {
    // Parameter defaults are set in header

    // Build object tree in memory to store the data you want to send in the request
    
    digitalWrite(onboard_LED, HIGH); // turn on onboard LED
    
    // create JSON message for Socket.IO
    JsonDocument docOut;
    JsonArray array = docOut.to<JsonArray>();

    // add event name
    // Hint: socket.on(eventName, ....
    array.add(eventName); // element[0] = event_name
    
    // add payload
    array.add(textString);
    int clientOffset = msgId;
    array.add(clientOffset);
    
    // JSON to String (serialization)
    String outputString;
    serializeJson(docOut, outputString);
  
    // Send event
    socketIO.sendEVENT(outputString);
    
    digitalWrite(onboard_LED, LOW); // turn off the LED
    msgId++;
    return true;
}

/* Send a note as data elements to the socketIO Server.
Not used in Unreal plugin. Leave in code for TODO info */
bool sendData(char eventName[],  int data1, int data2, int data3) {
    // Parameter defaults are set in header

    // Build object tree in memory to store the data you want to send in the request
    
    digitalWrite(onboard_LED, HIGH); // turn on onboard LED
    
    // create JSON message for Socket.IO
    JsonDocument docOut;
    JsonArray array = docOut.to<JsonArray>();

    // add event name
    // Hint: socket.on(eventName, ....
    array.add(eventName); // element[0] = event_name
    
    // add payload
    JsonObject param1 = array.createNestedObject();
    param1["now"] = millis();
    param1["data"][0] = data1;
    param1["data"][1] = data2;
    param1["data"][2] = data3;

    // JSON to String (serialization)
    String outputString;
    outputString += msgId;
    serializeJson(docOut, outputString);
  
    // Send event
    socketIO.sendEVENT(outputString);

    // Print JSON for debugging
    // USE_SERIAL.println(outputString);

    digitalWrite(onboard_LED, LOW); // turn off the LED
    msgId++;
    return true;
}

/*  Print all received event messages to Monitor */
void event(const char * payload, size_t length) {
    USE_SERIAL.printf("got message: %s\n", payload);
}

void socketIOEvent(socketIOmessageType_t type, uint8_t * payload, size_t length) {
    switch(type) {
        case sIOtype_DISCONNECT:
            if (connected) {
              USE_SERIAL.printf("[IOc] Not connected!\n");
              connected = false;
            }
            break;
        case sIOtype_CONNECT:
            USE_SERIAL.printf("[IOc] Connected to url: %s\n", payload);
            connected = true;
            // join default namespace "/"
            socketIO.send(sIOtype_CONNECT, "/");  // TODO use namespace /rubab
            break;
        case sIOtype_EVENT: {
            char * sptr = NULL;
            int id = strtol((char *)payload, &sptr, 10);
            USE_SERIAL.printf("[IOc] got event: %s id: %d\n", payload, id);
            if (id) {
                payload = (uint8_t *)sptr;
            }
            JsonDocument doc;
            DeserializationError error = deserializeJson(doc, payload, length);
            if (error) {
                USE_SERIAL.print(F("deserializeJson() failed: "));
                USE_SERIAL.println(error.c_str());
                return;
            }

            String eventName = doc[0];
            // USE_SERIAL.printf("[IOc] event name: %s\n", eventName.c_str());

            // Message Includes an ID for an ACK (callback)
            if (id) {
                // create JSON message for Socket.IO (ack)
                JsonDocument docOut;
                JsonArray array = docOut.to<JsonArray>();

                // add payload (parameters) for the ack (callback function)
                JsonObject param1 = array.createNestedObject();
                param1["now"] = millis();

                // JSON to String (serialization)
                String output;
                output += id;
                serializeJson(docOut, output);

                // Send event
                socketIO.send(sIOtype_ACK, output);
            }
        }
            break;
        case sIOtype_ACK:
            USE_SERIAL.printf("[IOc] got ack: %u\n", length);
            break;
        case sIOtype_ERROR:
            USE_SERIAL.printf("[IOc] got error: %u\n", length);
            break;
        case sIOtype_BINARY_EVENT:
            USE_SERIAL.printf("[IOc] got binary: %u\n", length);
            break;
        case sIOtype_BINARY_ACK:
            USE_SERIAL.printf("[IOc] got binary ack: %u\n", length);
            break;
        default:
            USE_SERIAL.printf("[IOc] got unknown: %u\n", length);
    }
}

/* UTIL */
void printBits(long int n) {
    // Util method to print an N-bit integer in the form: 0000 0000 0000 0000
    // works for 4 - 32 bits. accepts signed numbers
    // only used for debugging MPR121 frets in loop()
    byte numBits = 16;  // 2^numBits must be big enough to include the number n
    char b;
    char c = ' ';   // delimiter character
    for (byte i = 0; i < numBits; i++) {
        // shift 1 and mask to identify each bit value
        b = (n & (1 << (numBits - 1 - i))) > 0 ? '1' : '0'; // slightly faster to print chars than ints (saves conversion)
        USE_SERIAL.print(b);
        if (i < (numBits - 1) && ((numBits-i - 1) % 4 == 0 )) USE_SERIAL.print(c); // print a separator at every 4 bits
    }
}
