#include <Arduino.h>
#include <LiquidCrystal.h>
#include <SoftwareSerial.h>
#include <stdio.h>

#define MINUTE 60000UL
#define SECOND 1000UL

#define PIR_PIN A2         // Arduino analog pin A2
#define BUTTON_PIN A5      // Arduino analog pin A5
#define LED2_PIN A4        // Arduino analog pin A4 (Green)
#define LED1_PIN A3        // Arduino analog pin A3 (Red)

#define LCD_RS 8
#define LCD_E  9
#define LCD_D4 10
#define LCD_D5 11
#define LCD_D6 12
#define LCD_D7 13

#define BT_RX A0   // Arduino RX  (connects to Bluetooth TX)
#define BT_TX A1   // Arduino TX  (connects to Bluetooth RX)

LiquidCrystal lcd(LCD_RS, LCD_E, LCD_D4, LCD_D5, LCD_D6, LCD_D7);
SoftwareSerial btSerial(BT_RX, BT_TX); // RX, TX

volatile uint8_t lastPirVal = 0;
volatile uint8_t pirVal = 0;
volatile uint8_t buttonState = 0;
volatile uint8_t lastButtonState = 0;
volatile uint8_t useLed1 = 0;

unsigned long lastButtonPress = 0;
unsigned long lastLcdUpdate = 0;
const unsigned long DEBOUNCE_DELAY = 50;
const unsigned long LCD_UPDATE_INTERVAL = 100;

char currentLine0[17] = "";
char currentLine1[17] = "";
char newLine0[17] = "";
char newLine1[17] = "";

unsigned long myTime;
char printBuffer[128];

// -------- LCD Text Helpers --------
void updateLCD(int line, const char* text) {
    char* currentLine = (line == 0) ? currentLine0 : currentLine1;
    if (strcmp(currentLine, text) != 0) {
        lcd.setCursor(0, line);
        lcd.print("                "); // Clear line (16 spaces)
        lcd.setCursor(0, line);
        lcd.print(text);
        strncpy(currentLine, text, 16);
        currentLine[16] = '\0';
        delay(10);
    }
}
void setLCDMessage(int line, const char* text) {
    char* targetLine = (line == 0) ? newLine0 : newLine1;
    noInterrupts();
    strncpy(targetLine, text, 16);
    targetLine[16] = '\0';
    interrupts();
}

void refreshLCD() {
    unsigned long currentTime = millis();
    if (currentTime - lastLcdUpdate >= LCD_UPDATE_INTERVAL) {
        updateLCD(0, newLine0);
        updateLCD(1, newLine1);
        lastLcdUpdate = currentTime;
    }
}

uint8_t readButtonDebounced() {
    static uint8_t buttonStatePrev = 1;
    static unsigned long lastDebounceTime = 0;
    uint8_t reading = !digitalRead(BUTTON_PIN);
    if (reading != buttonStatePrev) {
        lastDebounceTime = millis();
    }
    if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) {
        if (reading != buttonState) {
            buttonState = reading;
            buttonStatePrev = reading;
            return buttonState;
        }
    }
    buttonStatePrev = reading;
    return buttonState;
}

// -------- Bluetooth Helpers (SoftwareSerial) --------
void setupBluetoothUART() {
    btSerial.begin(9600);
    // Optionally, Serial.begin(9600); // For USB debug
}
void btPrint(const char* str) {
    btSerial.print(str);
}
void btPrintln(const char* str) {
    btSerial.println(str);
}
uint8_t btAvailable() {
    return btSerial.available();
}
char btRead() {
    return btSerial.read();
}

void setup() {
    pinMode(LED1_PIN, OUTPUT);
    pinMode(LED2_PIN, OUTPUT);
    pinMode(PIR_PIN, INPUT);
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    digitalWrite(LED1_PIN, LOW);
    digitalWrite(LED2_PIN, LOW);

    setupBluetoothUART();

    delay(100);
    lcd.begin(16, 2);
    delay(50);

    setLCDMessage(0, "Senzor PIR");
    setLCDMessage(1, "Asteptare...");
    refreshLCD();
    lastLcdUpdate = millis();

    btPrintln("Sistem pornit cu Bluetooth!");
}

void loop() {
    unsigned long currentTime = millis();

    // --- 1. PIR Sensor ---
    pirVal = digitalRead(PIR_PIN);

    // --- 2. Bluetooth ---
    if (btAvailable()) {
        char cmd = btRead();
        btPrint("Comanda BT: ");
        btSerial.write(cmd);
        btPrintln("");

        // Allow Bluetooth to control regardless of PIR
        if (cmd == '1') {
            useLed1 = 1;
        } else if (cmd == '0') {
            useLed1 = 0;
        }
        setLCDMessage(1, useLed1 ? "LED ROSU aprins" : "LED VERDE aprins");
    }

    // --- 3. Button with debounce ---
    uint8_t currentButtonState = readButtonDebounced();
    if (currentButtonState && !lastButtonState && (currentTime - lastButtonPress > DEBOUNCE_DELAY * 4)) {
        useLed1 = !useLed1;
        lastButtonPress = currentTime;
        btPrint("Buton apasat! LED: ");
        btPrintln(useLed1 ? "ROSU" : "VERDE");
        setLCDMessage(1, useLed1 ? "LED ROSU aprins" : "LED VERDE aprins");
    }
    lastButtonState = currentButtonState;

    // --- 4. LEDs and LCD ---
    if (pirVal) {
        if (useLed1) {
            digitalWrite(LED1_PIN, HIGH);   // Red ON
            digitalWrite(LED2_PIN, LOW);    // Green OFF
        } else {
            digitalWrite(LED1_PIN, LOW);    // Red OFF
            digitalWrite(LED2_PIN, HIGH);   // Green ON
        }
        if (!lastPirVal) {
            myTime = millis();
            sprintf(printBuffer, "%lu min %lu sec: Miscare!", myTime / MINUTE, (myTime % MINUTE) / SECOND);
            btPrintln(printBuffer);
            setLCDMessage(0, "Miscare");
            setLCDMessage(1, useLed1 ? "LED ROSU aprins" : "LED VERDE aprins");
            lastPirVal = 1;
        }
    } else {
        digitalWrite(LED1_PIN, LOW);
        digitalWrite(LED2_PIN, LOW);
        if (lastPirVal) {
            myTime = millis();
            sprintf(printBuffer, "%lu min %lu sec: Miscare oprita!", myTime / MINUTE, (myTime % MINUTE) / SECOND);
            btPrintln(printBuffer);
            setLCDMessage(0, "Fara miscare");
            setLCDMessage(1, "LED-uri stinse");
            lastPirVal = 0;
        }
    }

    refreshLCD();
    delay(10);
}