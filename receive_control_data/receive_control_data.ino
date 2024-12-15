#include "Client.h"

#define CM4_SERIAL Serial7

struct ControlOutput {
  double d1;
  double d2;
  double thrust;
  double mz;
};

constexpr char IDENTIFIER = 0x02;
constexpr int LENGTH = 1 + 4 * 4;  // 1 byte for module id, 4 bytes for each module

Manager manager;
struct ControlModules {
  OneFloatModule d1;
  OneFloatModule d2;
  OneFloatModule thrust;
  OneFloatModule mz;
} control_modules;

void setup() {
  Serial.begin(115200);
  CM4_SERIAL.begin(115200);

  // Configure manager with modules
  manager.set_module_configuration(IDENTIFIER, { &control_modules.d1, &control_modules.d2, &control_modules.thrust, &control_modules.mz });
}

ControlOutput read_control() {
  ControlOutput output{ 0, 0, 0, 0 };
  if (CM4_SERIAL.available() > 0) {
    String received_message = CM4_SERIAL.readStringUntil('\n');

    // Unpacking messages
    char decoded_identifier;
    std::vector<std::vector<float>> decoded_values;
    manager.unpack_combined_message(received_message.c_str(), decoded_identifier, decoded_values);

    if (decoded_identifier == IDENTIFIER) {
      output.d1 = decoded_values[0][0];
      output.d2 = decoded_values[1][0];
      output.thrust = decoded_values[2][0];
      output.mz = decoded_values[3][0];
    } else {
      Serial.println("Received unknown identifier, need to implement the decoder");
    }
  }
  return output;
}

void loop() {
  ControlOutput output = read_control();
  if (output.d1 != 0 || output.d2 != 0 || output.thrust != 0 || output.mz != 0) {
    Serial.print("Received data: ");
    Serial.print("d1: ");
    Serial.print(output.d1);
    Serial.print(", d2: ");
    Serial.print(output.d2);
    Serial.print(", thrust: ");
    Serial.print(output.thrust);
    Serial.print(", mz: ");
    Serial.println(output.mz);
  }
}
