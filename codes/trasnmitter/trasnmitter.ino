#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

const uint64_t pipeOut = 0xE9E8F0F0E1LL;
RF24 radio(9, 10); // CE, CSN

struct PacketData 
{
  byte lxAxisValue;  // Roll (A2)
  byte lyAxisValue;  // Pitch (A3)
  byte rxAxisValue;  // Throttle (A0)
  byte ryAxisValue;  // Yaw (A1)
  byte switch1Value; // AUX1 (Pin 4)
  byte switch2Value; // AUX2 (Pin 3)
};

PacketData data;

void setup()
{
  Serial.begin(9600);
  Serial.println("=== NRF24L01 Transmitter ===");
  Serial.println("Initializing...");

  if (!radio.begin()) {
    Serial.println("NRF24L01 not detected! Check wiring and power.");
    while (1);
  }

  radio.openWritingPipe(pipeOut);
  radio.setChannel(108);
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.stopListening();

  pinMode(4, INPUT_PULLUP);  // AUX1
  pinMode(3, INPUT_PULLUP);  // AUX2

  Serial.println("NRF24L01 initialized successfully!");
  delay(1000);
}

// Map joystick with deadband adjustment
int mapAndAdjustJoystickDeadBandValues(int value, int lower, int middle, int upper, bool reverse)
{
  if (value >= middle + 10)
  {
    value = map(value, middle + 10, upper, 127, 254);
  }
  else if (value <= middle - 10)
  {
    value = map(value, lower, middle - 10, 0, 127);  
  }
  else
  {
    value = 127;
  }
  
  if (reverse)
  {
    value = 254 - value;
  }
  
  return value;
}

void loop()
{
  // Read analog joysticks with calibrated center values
  data.lxAxisValue    = mapAndAdjustJoystickDeadBandValues(analogRead(A2), 0, 492, 1023, false);  // Roll
  data.lyAxisValue    = mapAndAdjustJoystickDeadBandValues(analogRead(A3), 0, 521, 1023, false);  // Pitch
  data.rxAxisValue    = mapAndAdjustJoystickDeadBandValues(analogRead(A0), 0, 510, 1023, false);  // Throttle
  data.ryAxisValue    = mapAndAdjustJoystickDeadBandValues(analogRead(A1), 0, 520, 1023, false);  // Yaw
  
  // Read digital switches (active LOW with pullup)
  data.switch1Value   = !digitalRead(4);  // AUX1
  data.switch2Value   = !digitalRead(3);  // AUX2
  
  bool success = radio.write(&data, sizeof(PacketData));

  if (success) {
    Serial.print("Sent -> ");
    Serial.print("Roll: "); Serial.print(data.lxAxisValue);
    Serial.print(" | Pitch: "); Serial.print(data.lyAxisValue);
    Serial.print(" | Throttle: "); Serial.print(data.rxAxisValue);
    Serial.print(" | Yaw: "); Serial.print(data.ryAxisValue);
    Serial.print(" | SW1: "); Serial.print(data.switch1Value);
    Serial.print(" | SW2: "); Serial.println(data.switch2Value);
  } else {
    Serial.println("Transmission failed!");
  }

  delay(100);
}