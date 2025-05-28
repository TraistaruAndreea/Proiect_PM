
#include <LiquidCrystal.h>
#include <SoftwareSerial.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>

#define MINUTE 60000
#define SECOND 1000

// Pin Definitions
#define PIR_PIN         PC2     // A2
#define BUTTON_PIN      PC5     // A5
#define LED2_PIN        PC4     // A4
#define LED1_PIN        PC3     // A3

// LCD Pins
#define LCD_RS          PB0     // 8
#define LCD_E           PB1     // 9
#define LCD_D4          PB2     // 10
#define LCD_D5          PB3     // 11
#define LCD_D6          PB4     // 12
#define LCD_D7          PB5     // 13

// Bluetooth Pins
#define BT_RX           PC0     // A0
#define BT_TX           PC1     // A1

volatile uint8_t lastPirVal = 0;
volatile uint8_t pirVal = 0;
volatile uint8_t buttonState = 0;
volatile uint8_t lastButtonState = 0;
volatile uint8_t useLed1 = 0;

volatile uint32_t timer0_ticks = 0;  // For time tracking
volatile uint32_t lastButtonPress = 0;
volatile uint32_t lastLcdUpdate = 0;
const uint16_t DEBOUNCE_DELAY = 50;
const uint16_t LCD_UPDATE_INTERVAL = 100;

char currentLine0[17] = "";
char currentLine1[17] = "";
char newLine0[17] = "";
char newLine1[17] = "";

char printBuffer[128];

LiquidCrystal lcd(8, 9, 10, 11, 12, 13); // LCD_RS, LCD_E, LCD_D4, LCD_D5, LCD_D6, LCD_D7
SoftwareSerial btSerial(14, 15); // BT_RX (A0), BT_TX (A1)

// Timer0 interrupt for 1ms ticks
ISR(TIMER0_COMPA_vect) {
    timer0_ticks++;
}

// Initialize Timer0 for 1ms interrupts
void initTimer0() {
    TCCR0A = (1 << WGM01);             // CTC mode
    TCCR0B = (1 << CS01) | (1 << CS00); // Prescaler 64
    OCR0A = 249;                       // (16000000 / 64 / 1000) - 1 = 249
    TIMSK0 = (1 << OCIE0A);            // Enable compare match interrupt
}

uint32_t getTicks() {
    uint32_t ticks;
    cli();
    ticks = timer0_ticks;
    sei();
    return ticks;
}

// --- GPIO helpers ---
void setPinOutput(volatile uint8_t* ddr, uint8_t pin) {
    *ddr |= (1 << pin);
}

void setPinInput(volatile uint8_t* ddr, uint8_t pin) {
    *ddr &= ~(1 << pin);
}

void writePin(volatile uint8_t* port, uint8_t pin, uint8_t value) {
    if (value) *port |= (1 << pin);
    else *port &= ~(1 << pin);
}

uint8_t readPin(volatile uint8_t* pinReg, uint8_t pin) {
    return (*pinReg & (1 << pin)) ? 1 : 0;
}

// --- Bluetooth functions ---
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

// ----- LCD text helpers -----
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
    cli();
    strncpy(targetLine, text, 16);
    targetLine[16] = '\0';
    sei();
}

void refreshLCD() {
    uint32_t currentTicks = getTicks();
    if (currentTicks - lastLcdUpdate >= LCD_UPDATE_INTERVAL) {
        updateLCD(0, newLine0);
        updateLCD(1, newLine1);
        lastLcdUpdate = currentTicks;
    }
}

uint8_t readButtonDebounced() {
    static uint8_t buttonStatePrev = 1;
    static uint32_t lastDebounceTime = 0;
    uint8_t reading = !readPin(&PINC, BUTTON_PIN);
    uint32_t currentTicks = getTicks();

    if (reading != buttonStatePrev) {
        lastDebounceTime = currentTicks;
    }

    if ((currentTicks - lastDebounceTime) > DEBOUNCE_DELAY) {
        if (reading != buttonState) {
            buttonState = reading;
            buttonStatePrev = reading;
            return buttonState;
        }
    }

    buttonStatePrev = reading;
    return buttonState;
}

void initIO() {
    // Set LED pins as outputs
    DDRC |= (1 << LED1_PIN) | (1 << LED2_PIN);

    // Set PIR and button pins as inputs
    DDRC &= ~((1 << PIR_PIN) | (1 << BUTTON_PIN));

    // Enable pull-up for button
    PORTC |= (1 << BUTTON_PIN);

    // Ensure LEDs are off initially
    PORTC &= ~((1 << LED1_PIN) | (1 << LED2_PIN));
}

int main() {
    // Initialize all hardware
    initIO();
    initTimer0();

    // Initialize Bluetooth
    btSerial.begin(9600);

    // Initialize LCD
    lcd.begin(16, 2);
    _delay_ms(50);

    setLCDMessage(0, "Senzor PIR");
    setLCDMessage(1, "Asteptare...");
    refreshLCD();
    lastLcdUpdate = getTicks();

    btPrintln("Sistem pornit cu Bluetooth!");
    sei();  // Enable global interrupts
    while (1) {
        uint32_t currentTicks = getTicks();

        // Read PIR sensor
        pirVal = readPin(&PINC, PIR_PIN);

        // Check Bluetooth
        if (btAvailable()) {
            char cmd = btRead();
            btPrint("Comanda BT: ");
            btSerial.write(cmd);
            btPrintln("");

            if (cmd == '1') {
                useLed1 = 1;
                btPrintln("Selectat LED ROSU");
                if (pirVal) setLCDMessage(1, "LED ROSU aprins");
            }
            else if (cmd == '0') {
                useLed1 = 0;
                btPrintln("Selectat LED VERDE");
                if (pirVal) setLCDMessage(1, "LED VERDE aprins");
            }
        }

        // Button handling
        uint8_t currentButtonState = readButtonDebounced();
        if (currentButtonState && !lastButtonState && (currentTicks - lastButtonPress > DEBOUNCE_DELAY * 4)) {
            useLed1 = !useLed1;
            lastButtonPress = currentTicks;
            btPrint("Buton apasat! LED: ");
            btPrintln(useLed1 ? "ROSU" : "VERDE");
            if (pirVal) {
                setLCDMessage(1, useLed1 ? "LED ROSU aprins" : "LED VERDE aprins");
            }
        }
        lastButtonState = currentButtonState;

        // LED control
        if (pirVal) {
            if (useLed1) {
                PORTC |= (1 << LED1_PIN);
                PORTC &= ~(1 << LED2_PIN);
            } else {
                PORTC &= ~(1 << LED1_PIN);
                PORTC |= (1 << LED2_PIN);
            }

            if (!lastPirVal) {
                uint32_t time = currentTicks;
                sprintf(printBuffer, "%lu min %lu sec: Miscare!", time / MINUTE, (time % MINUTE) / SECOND);
                btPrintln(printBuffer);
                setLCDMessage(0, "Miscare");
                setLCDMessage(1, useLed1 ? "LED ROSU aprins" : "LED VERDE aprins");
                lastPirVal = 1;
            }
        } else {
            PORTC &= ~((1 << LED1_PIN) | (1 << LED2_PIN));

            if (lastPirVal) {
                uint32_t time = currentTicks;
                sprintf(printBuffer, "%lu min %lu sec: Miscare oprita!", time / MINUTE, (time % MINUTE) / SECOND);
                btPrintln(printBuffer);
                setLCDMessage(0, "Fara miscare");
                setLCDMessage(1, "LED-uri stinse");
                lastPirVal = 0;
            }
        }

        refreshLCD();
        _delay_ms(10);
    }
    return 0;
}
