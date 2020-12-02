
#include "DmaSerialTeensy.h"
#include "HardwareSerial.h"
#include "usb_serial.h"
#include "Arduino.h"

void setup() {

    pinMode(13, OUTPUT);

    Serial.begin(115200);
    dmaSerial1.begin(115200);
    dmaSerial2.begin(115200);
    dmaSerial3.begin(115200);
    dmaSerial4.begin(115200);
    dmaSerial5.begin(115200);

    delay(1000);
}

void loop() {
    static uint32_t last = 1000;
    if (millis() >= last + 650) {
        last = millis();
        Serial.println("alive");

        /*
        dmaSerial1.println("alive");
        dmaSerial2.println("alive");
        dmaSerial3.println("alive");
        dmaSerial4.println("alive");
        dmaSerial5.println("alive");
        */

        static uint8_t led = 0;
        digitalWrite(13, led);
        led = !led;

        dmaSerial1.println(dmaSerial1.available());
        while (dmaSerial1.available()) {
            //dmaSerial1.write(
                    dmaSerial1.read();
                    //);
        }

    }

    while (dmaSerial2.available()) {
        dmaSerial2.write(dmaSerial2.read());
    }
    while (dmaSerial3.available()) {
        dmaSerial3.write(dmaSerial3.read());
    }
    while (dmaSerial4.available()) {
        dmaSerial4.write(dmaSerial4.read());
    }
    while (dmaSerial5.available()) {
        dmaSerial5.write(dmaSerial5.read());
    }

}
