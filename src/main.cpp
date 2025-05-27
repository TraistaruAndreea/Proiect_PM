#include <Arduino.h>
#include <LiquidCrystal.h>
#include <SoftwareSerial.h>
#include <stdio.h>
#include <avr/interrupt.h>

#define MINUTE 60000UL
#define SECOND 1000UL

#define PIR_PIN A2
#define BUTTON_PIN A5
#define LED2_PIN A4
#define LED1_PIN A3

#define LCD_RS 8
#define LCD_E  9
#define LCD_D4 10
#define LCD_D5 11
#define LCD_D6 12
#define LCD_D7 13

#define BT_RX A0
#define BT_TX A1

LiquidCrystal lcd(LCD_RS, LCD_E, LCD_D4, LCD_D5, LCD_D6, LCD_D7);
SoftwareSerial btSerial(BT_RX, BT_TX); // RX, TX

volatile uint8_t lastPirVal = 0;
volatile uint8_t pirVal = 0;
volatile uint8_t buttonState = 0;
volatile uint8_t lastButtonState = 0;
volatile uint8_t useLed1 = 0;

unsigned long lastButtonPress = 0;
const unsigned long DEBOUNCE_DELAY = 50;

char currentLine0[17] = "";
char currentLine1[17] = "";
char newLine0[17] = "";
char newLine1[17] = "";

unsigned long myTime;
char printBuffer[128];

volatile bool flag_lcd = false;

// -------- Timer1 for LCD refresh every 100ms --------
void setupTimer1() {
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;

  OCR1A = 24999; // 100ms at 16MHz with prescaler 64
  TCCR1B |= (1 << WGM12);              // CTC mode
  TCCR1B |= (1 << CS11) | (1 << CS10); // Prescaler 64
  TIMSK1 |= (1 << OCIE1A);             // Enable Timer1 compare interrupt
  interrupts();
}

ISR(TIMER1_COMPA_vect) {
  flag_lcd = true;
}

// -------- LCD Text Helpers --------
void updateLCD(int line, const char* text) {
    char* currentLine = (line == 0) ? currentLine0 : currentLine1;
    if (strcmp(currentLine, text) != 0) {
        lcd.setCursor(0, line);
        lcd.print("                ");
        lcd.setCursor(0, line);
        lcd.print(text);
        strncpy(currentLine, text, 16);
        currentLine[16] = '\0';
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
    updateLCD(0, newLine0);
    updateLCD(1, newLine1);
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

// -------- Bluetooth --------
void setupBluetoothUART() {
    btSerial.begin(9600);
}
void btPrint(const char* str) { btSerial.print(str); }
void btPrintln(const char* str) { btSerial.println(str); }
uint8_t btAvailable() { return btSerial.available(); }
char btRead() { return btSerial.read(); }

void setup() {
    pinMode(LED1_PIN, OUTPUT);
    pinMode(LED2_PIN, OUTPUT);
    pinMode(PIR_PIN, INPUT);
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    digitalWrite(LED1_PIN, LOW);
    digitalWrite(LED2_PIN, LOW);

    setupBluetoothUART();
    setupTimer1();

    delay(100);
    lcd.begin(16, 2);
    delay(50);

    setLCDMessage(0, "Senzor PIR");
    setLCDMessage(1, "Asteptare...");
    refreshLCD();

    btPrintln("Sistem pornit cu Bluetooth!");
}

void loop() {
    pirVal = digitalRead(PIR_PIN);
    if (pirVal) {
        setLCDMessage(0, "Miscare");
        if (btAvailable()) {
            char cmd = btRead();
            btPrint("Comanda BT: ");
            btSerial.write(cmd);
            btPrintln("");

            if (cmd == '1') {
                useLed1 = 1;
            } else if (cmd == '0') {
                useLed1 = 0;
            }
            setLCDMessage(1, useLed1 ? "LED ROSU aprins" : "LED VERDE aprins");
        }
    }

    uint8_t currentButtonState = readButtonDebounced();
    if (currentButtonState && !lastButtonState && (millis() - lastButtonPress > DEBOUNCE_DELAY * 4)) {
        useLed1 = !useLed1;
        lastButtonPress = millis();
        btPrint("Buton apasat! LED: ");
        btPrintln(useLed1 ? "ROSU" : "VERDE");
        setLCDMessage(1, useLed1 ? "LED ROSU aprins" : "LED VERDE aprins");
    }
    lastButtonState = currentButtonState;

    if (pirVal) {
        if (useLed1) {
            digitalWrite(LED1_PIN, HIGH);
            digitalWrite(LED2_PIN, LOW);
        } else {
            digitalWrite(LED1_PIN, LOW);
            digitalWrite(LED2_PIN, HIGH);
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

    if (flag_lcd) {
        flag_lcd = false;
        refreshLCD();
    }

    delay(10);
}
