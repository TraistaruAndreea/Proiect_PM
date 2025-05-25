#include <Arduino.h>
#include <LiquidCrystal.h>
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>

#define MINUTE 60000
#define SECOND 1000

#define PIR_PIN 2       // PD2
#define BUTTON_PIN 4    // PD4
#define LED2_PIN 6      // PD6
#define LED1_PIN 4      // PB4 (Pin 12)
#define CONTRAST_PIN 3  // PD3

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

LiquidCrystal lcd(11, 10, 9, 8, 7, 5);
uint8_t Contrast = 75;

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

// --- PWM for LCD contrast ---
void setupPWMContrast() {
    TCCR2A |= (1 << COM2B1) | (1 << WGM21) | (1 << WGM20);
    TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20);
    setPinOutput(&DDRD, CONTRAST_PIN);
    OCR2B = Contrast;
}

// --- UART for BT (hardware 0/1) ---
void setupBluetoothUART() {
    uint16_t baud = 103; // 16MHz / (16 * 9600) - 1 = 103
    UBRR0H = (baud >> 8);
    UBRR0L = baud;
    UCSR0B |= (1 << TXEN0) | (1 << RXEN0); // Enable TX si RX
    UCSR0C |= (1 << UCSZ01) | (1 << UCSZ00); // 8 data bits, 1 stop bit
}
void btTransmit(char data) {
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = data;
}
void btPrint(const char* str) {
    while (*str) btTransmit(*str++);
}
void btPrintln(const char* str) {
    btPrint(str);
    btTransmit('\r');
    btTransmit('\n');
}
uint8_t btAvailable() {
    return (UCSR0A & (1 << RXC0)) ? 1 : 0;
}
char btRead() {
    while (!(UCSR0A & (1 << RXC0)));
    return UDR0;
}
void delayMs(uint16_t ms) {
    for (uint16_t i = 0; i < ms; i++) _delay_ms(1);
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
        delayMs(10);
    }
}
void setLCDMessage(int line, const char* text) {
    char* targetLine = (line == 0) ? newLine0 : newLine1;
    cli(); // opreste intreruperile temporar
    strncpy(targetLine, text, 16);
    targetLine[16] = '\0';
    sei(); // porneste inapoi
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
    uint8_t reading = !readPin(&PIND, BUTTON_PIN);
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

int main() {
    init();

    setPinOutput(&DDRB, LED1_PIN);
    setPinOutput(&DDRD, LED2_PIN);
    setPinInput(&DDRD, PIR_PIN);
    setPinInput(&DDRD, BUTTON_PIN);
    PORTD |= (1 << BUTTON_PIN);
    writePin(&PORTB, LED1_PIN, 0);
    writePin(&PORTD, LED2_PIN, 0);

    setupBluetoothUART();
    setupPWMContrast();

    delayMs(100);
    lcd.begin(16, 2);
    delayMs(50);

    setLCDMessage(0, "Senzor PIR");
    setLCDMessage(1, "Asteptare...");
    refreshLCD();
    lastLcdUpdate = millis();

    btPrintln("Sistem pornit cu Bluetooth!");

    while (1) {
        unsigned long currentTime = millis();
        // --- 1. PIR ---
        pirVal = readPin(&PIND, PIR_PIN);

        // --- 2. Bluetooth ---
        if (btAvailable()) {
            char cmd = btRead();
            btPrint("Comanda BT: ");
            btTransmit(cmd);
            btPrintln("");

            setLCDMessage(1, "Led aprins");
            if (pirVal) {
                if (cmd == '1') {
                    useLed1 = 1;
                } else if (cmd == '0') {
                    useLed1 = 0;
                }
            }
        }

        // --- 3. Buton cu debounce ---
        uint8_t currentButtonState = readButtonDebounced();
        if (currentButtonState && !lastButtonState && (currentTime - lastButtonPress > DEBOUNCE_DELAY * 4)) {
            useLed1 = !useLed1;
            lastButtonPress = currentTime;
            btPrint("Buton apasat! LED: ");
            btPrintln(useLed1 ? "ROSU" : "VERDE");
        }
        lastButtonState = currentButtonState;


        // --- 4. LED-uri si LCD ---
        if (pirVal) {
            if (useLed1) {
                writePin(&PORTB, LED1_PIN, 1);
                writePin(&PORTD, LED2_PIN, 0);
            } else {
                writePin(&PORTB, LED1_PIN, 0);
                writePin(&PORTD, LED2_PIN, 1);
            }
            if (!lastPirVal) {
                myTime = millis();
                sprintf(printBuffer, "%lu min %lu sec: Miscare!", myTime / MINUTE, (myTime % MINUTE) / SECOND);
                btPrintln(printBuffer);
                setLCDMessage(0, "Miscare");
                setLCDMessage(1, "Led aprins");
                lastPirVal = 1;
            }
        } else {
            writePin(&PORTB, LED1_PIN, 0);
            writePin(&PORTD, LED2_PIN, 0);
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
        delayMs(10);
    }
    return 0;
}