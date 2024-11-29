#include "Sensors.h"

TripleBNO055 bno(0x28, 0x29, 0x29, &Wire, &Wire2);
TripleBMP581 bmp(0x46, 0x47, 0x46, &Wire, &Wire2);

const uint8_t bno_data_sz = 6;
const uint8_t bmp_data_sz = 2;
std::array<float, bno_data_sz> bno_data;
std::array<float, bmp_data_sz> bmp_data;


void initWire() {
    Wire.begin();
    Wire.setSCL(16);
    Wire.setSDA(17);
    Wire.setClock(400000);

    Wire2.begin();
    Wire2.setSCL(24);
    Wire2.setSDA(25);
    Wire2.setClock(400000);
}

void setup() {
    Serial.begin(115200);
    Serial.println("Initializing serial communication");
    initWire();
    bool bno_status = bno.setup();
    bno.displayStatus();
    Serial.println("Finished initializing BNO");

    bool bmp_status = bmp.setup();
    bmp.displayStatus();
    Serial.println("Finished initializing BMP");

    bno.calibrate();
    Serial.println("Finished calibrating BNO");

    bmp.calibrate();
    Serial.println("Finished calibrating BMP");

}

void loop() {
    bno_data = bno.read_data();
    bmp_data = bmp.read_data(); 
    Serial.print(millis());
    Serial.print(",");
    bno.print_data(bno_data);
    Serial.print(" ");
    bmp.print_data(bmp_data);
    Serial.println("");
}

