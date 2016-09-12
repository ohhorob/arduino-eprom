#include <Arduino.h>

/** PINS
 *
 * http://www.pjrc.com/teensy/schematic.html
 *
 * https://forum.pjrc.com/threads/26054-Teensy-3-1-I-O-Ports
 * https://forum.pjrc.com/threads/23950-Parallel-GPIO-on-Teensy-3-0
 * https://forum.pjrc.com/threads/23950-Parallel-GPIO-on-Teensy-3-0?p=96384&viewfull=1#post96384
 *
 * Enable [active low]
 * 33 -> 18,20 (white)
 *
 * Address
 *
 * Data
 *
 * PWM RGB LED
 *
 */

#define ANSI_ESCAPE 0x1B
#define ANSI_LEFT_BRACKET 0x5B
const uint8_t ERASE_DISPLAY[] = {ANSI_ESCAPE, ANSI_LEFT_BRACKET, '2', 'J'};

#define EPROM_ENABLE 33

// PWM pins for RGB LED
#define LED_RED   25
#define LED_GREEN  3
#define LED_BLUE  32
void setupLED() {

//    int duty = 150;

    // Setup RGB LED on PWM
    pinMode(LED_RED, OUTPUT);
    pinMode(LED_GREEN, OUTPUT);
    pinMode(LED_BLUE, OUTPUT);

//    analogWrite(LED_GREEN, 0);
//    analogWrite(LED_BLUE,  0);
//    analogWrite(LED_RED,   200);
//    delay(duty);
//    analogWrite(LED_GREEN, 200);
//    delay(duty);
//    analogWrite(LED_BLUE,  200);
//    delay(duty);
    analogWrite(LED_RED,     0);
    analogWrite(LED_GREEN,   0);
    analogWrite(LED_BLUE,    0);
}

void printFlush() {
    Serial.flush();
    delay(5);
}

void printSlot(volatile uint8_t *slot) {
    Serial.print("slot@0x");
    Serial.print((long)slot, HEX);
    Serial.print(" = 0x"); Serial.print(*slot, HEX);
}

void print32Binary(uint32_t value) {
    printFlush();
    Serial.print("0b");
    for (uint8_t b = 31; (value >> b) < 1 && b > 0; b--) {
        Serial.print('0');
    }
    Serial.print(value, BIN);
}

/**
 * 11.14.1 Pin Control Register n (PORTx_PCRn)
 *
 * MUX: 10..8
 *
 * @param config control register pointer
 */
void printConfig(volatile uint32_t *config) {
    printFlush();
    Serial.print("config@0x");
    Serial.print((long)config, HEX);
    Serial.print(" = ");
    printFlush();
    print32Binary(*config);
    uint8_t mux = (uint8_t) ((*config & 0x0700) >> 8); // 10..8
    Serial.print(" mux= 0b"); Serial.print(mux, BIN);
}

void showPorts() {
    Serial.println("Port C");
    volatile uint32_t *config = digital_pin_to_info_PGM[15].config;
//    volatile uint8_t *mode = (uint8_t *) (digital_pin_to_info_PGM[15].reg + 160);

    for (uint8_t bit = 0; bit < 11; bit++) {
        Serial.print(bit < 10 ? " > bit 0" : " > bit ");
        Serial.print(bit);
        Serial.print(' ');
        printConfig(config + bit);
        Serial.println();
        printFlush();
    }
    Serial.print(" > direction@0x");
    Serial.print((long)&GPIOC_PDDR, HEX);
    Serial.print(" = ");
    print32Binary(GPIOC_PDDR);
    Serial.println();

    Serial.println("Port D");
    config = digital_pin_to_info_PGM[2].config;
//    mode = (uint8_t *) (digital_pin_to_info_PGM[2].reg + 160);
    for (uint8_t bit = 0; bit < 8; bit++) {
        Serial.print(" > bit ");
        Serial.print(bit);
        Serial.print(' ');
        printConfig(config + bit); Serial.println();
    }
    Serial.print(" > direction@0x");
    Serial.print((long)&GPIOD_PDDR, HEX);
    Serial.print(" = ");
    print32Binary(GPIOD_PDDR);
    Serial.println();

}

void setupPorts() {

    // GPIOx_PDDR; 49.2.6 Port Data Direction Register
    // The PDDR configures the individual port pins for input or output.
    // 0 Pin is configured as general-purpose input, for the GPIO function.
    // 1 Pin is configured as general-purpose output, for the GPIO function.

    // Take Port C as GPIO
    // On KINETISK, no bit mask required; on KINETISL, you resolve the mask with digitalPinToBitMask(15)
    //Serial.send_now();
    Serial.println("Claiming Port C pins as GPIO");

    // Pin configuration value is the same for all the address lines. Compute it once here
    static uint32_t gpioWrite = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1) | PORT_PCR_PE | PORT_PCR_PS;
    static uint32_t gpioRead = PORT_PCR_MUX(1);

//    volatile uint32_t *config = digital_pin_to_info_PGM[15].config;
//    volatile uint32_t *config = &CORE_PIN15_CONFIG;
    volatile uint32_t *config = &PORTC_PCR0;

    volatile uint8_t  *mode   = (uint8_t *) (digital_pin_to_info_PGM[15].reg + 160);

    for (uint8_t bit = 0; bit < 11; bit++) {
        *(config + bit) = gpioWrite;

        // Port mode; 1 = OUTPUT
        *(mode + bit) = 1;
//        *portModeRegister(22) |= 1;
    }

    // 11 address lines
    uint32_t writeEleven = 0x07FF; // 0..10

    // on Port C low bits
    GPIOC_PDDR |= writeEleven; // set the bottom 11 bits

    // Clear outputs
    GPIOC_PCOR |= writeEleven;

    // Port B inputs
    //Serial.send_now();
    Serial.println("Claiming Port D pins as GPIO");

    config = digital_pin_to_info_PGM[2].config;
    mode = (uint8_t *) (digital_pin_to_info_PGM[2].reg + 160);
    for (uint8_t bit = 0; bit < 8; bit++) {
        *(config + bit) = gpioRead;
        *(mode + bit) = 0;
    }

    // 8 data lines
    uint32_t readEight = ~((uint32_t)0xFF); // 0xFFFFFF00; // 0..7

    // on Port D low bits
    GPIOD_PDDR &= readEight; // clear the bottom 8 bits

}

char usb_chars[64];
void setup() {
    Serial.begin(1);  // USB, communication to PC or Mac
    setupLED();
    delay(750);

//    // Waits until USB is connected (only for two seconds)
    while (!Serial.availableForWrite()) {
        analogWrite(LED_RED, 180);
        delay(100);
    }
//    analogWrite(LED_RED, 0);
    analogWrite(LED_GREEN, 50);
    size_t len = 0;
    if (Serial.available()) {
        len = Serial.readBytes(usb_chars, 64);
    }
    Serial.write(ERASE_DISPLAY, 4);
    if (len > 0) {
        Serial.println("Slurped serial bytes.");
        for (size_t i = 0; i < len; i++) {
            Serial.print("0x");
            Serial.print(usb_chars[i], HEX);
            Serial.print(' ');
        }
        Serial.println();
        for (size_t i = 0; i < len; i++) {
            Serial.print(usb_chars[i]);
        }
        Serial.println();
    }

    Serial.println();
    Serial.println("Connected USB.");
    Serial.println(Serial.baud());

    // Enable the EPROM output
    pinMode(EPROM_ENABLE, OUTPUT);
    digitalWrite(EPROM_ENABLE, HIGH);
    delay(50);
    digitalWrite(EPROM_ENABLE, LOW);
    delay(200);
    analogWrite(LED_GREEN, 2);

    Serial.println();
    Serial.println("TAB => showPorts()");
    Serial.println("ENTER => setupPorts()");
    Serial.println("SPACE => readEprom()");

}

#define DELAY_READ 1000

uint8_t readEprom(uint16_t a) {
    // leave the unused bits of Port C alone
    GPIOC_PCOR = 0x7FF;
    delayMicroseconds(DELAY_READ);
    GPIOC_PSOR = a;

    delayMicroseconds(DELAY_READ);
    return (uint8_t) (GPIOD_PDIR & 0xFF);
}

uint8_t contents[0x7FF];
uint16_t addr = 0x0000;

void readUntil(uint16_t maxAddr) {
    __disable_irq();
    while (addr < maxAddr) {
        contents[addr] = readEprom(addr);
        addr += 1;
    }
    __enable_irq();
}

void dumpContents() {
    Serial.println();
    uint8_t b;
    for (uint16_t a = 0; a <= addr; a++) {
        if (0 == a % 32) {
            Serial.println();
            printFlush();
        }
        // 0xAAAA: 0xbb bb
//        Serial.print(a > 0xFFF ? "0x" : a > 0xFF ? "0x0" : a > 0xF ? "0x00" : "0x000");
//        Serial.print(a, HEX);
//        Serial.print(": ");
        b = contents[a];
        Serial.print(b > 0xF ? " " : " 0");
        Serial.print(b, HEX);

//        b = contents[a];
//        Serial.print(b > 0xF ? " " : " 0");
//        Serial.print(b, HEX);
//        Serial.println();
    }
}

int command_arg = 0;
char command[10];
int idx = 0;

void maintainInput() {
    if (Serial.available()) {
        size_t len = Serial.readBytes(usb_chars, 64);
        if (len == 1) {
            if (usb_chars[0] >= '0' && usb_chars[0] <= '9') {
                command_arg *= 10;
                command_arg += (usb_chars[0] - '0');

                if (command_arg > 0 && command_arg < 10) {
                    Serial.print("arg> ");
                } else if (command_arg > 0) {
                    Serial.print(usb_chars[0]);
                }

            } else if (0x09 == usb_chars[0]) {
                // Tab is to print the port configs
                showPorts();
            } else if (0x0D == usb_chars[0]) {
                // Enter Key
                setupPorts();
                analogWrite(LED_GREEN, 40);
            } else if (usb_chars[0] >= 'a' && usb_chars[0] <= 'z') {
                // Alpha input
                // Accumulate into a command
            } else if (' ' == usb_chars[0]) {
                addr = 0; // Reset addr for full read
                analogWrite(LED_BLUE, 200);
                readUntil(0x7FF);
                analogWrite(LED_BLUE, 0);

                Serial.print("Read from 0x0000 to 0x");
                Serial.print(addr, HEX);
                Serial.println();
                printFlush();

                dumpContents();
                Serial.println();
            } else {
                // Debug character output
                Serial.print("> 0x");
                Serial.print(usb_chars[0], HEX);
            }

        } else if (ANSI_LEFT_BRACKET == usb_chars[0]) {
            // Esc codes
            switch (usb_chars[1]) {
                case 0x41:
                case 0x42:
                    // Up-Down
                    // Adjust micros to wait?

                default:
                    Serial.print("Esc 0x");
                    Serial.println(usb_chars[1]);
            }
        }
    }
}


void loop() {
    maintainInput();
    if (addr < 0x0) {
        Serial.println("Reading...");
        ////Serial.send_now();
        readUntil(0x1FF);
    }
}