#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>

#define SIGNAL_TIMEOUT 1000  // Signal timeout in milliseconds

const uint64_t pipeIn = 0xE9E8F0F0E1LL;
RF24 radio(9, 10);  // CE, CSN

unsigned long lastRecvTime = 0;

struct PacketData
{
  byte lxAxisValue;    // Roll
  byte lyAxisValue;    // Pitch
  byte rxAxisValue;    // Throttle
  byte ryAxisValue;    // Yaw
  byte switch1Value;   // AUX1
  byte switch2Value;   // AUX2
};

PacketData receiverData;

Servo ch1;  // Roll
Servo ch2;  // Pitch
Servo ch3;  // Throttle
Servo ch4;  // Yaw
Servo ch5;  // AUX1
Servo ch6;  // AUX2

// Assign default input received values
void setInputDefaultValues()
{
  receiverData.lxAxisValue = 127;   // Roll center
  receiverData.lyAxisValue = 127;   // Pitch center
  receiverData.rxAxisValue = 0;     // Throttle minimum
  receiverData.ryAxisValue = 127;   // Yaw center
  receiverData.switch1Value = 0;
  receiverData.switch2Value = 0;
}

void mapAndWriteValues()
{
  // Map values and write to servos
  ch1.writeMicroseconds(map(receiverData.lxAxisValue, 0, 254, 1000, 2000));    // Roll
  ch2.writeMicroseconds(map(receiverData.lyAxisValue, 0, 254, 1000, 2000));    // Pitch
  ch3.writeMicroseconds(map(receiverData.rxAxisValue, 0, 254, 1000, 2000));    // Throttle
  ch4.writeMicroseconds(map(receiverData.ryAxisValue, 0, 254, 1000, 2000));    // Yaw
  ch5.writeMicroseconds(map(receiverData.switch1Value, 0, 1, 1000, 2000));     // AUX1
  ch6.writeMicroseconds(map(receiverData.switch2Value, 0, 1, 1000, 2000));     // AUX2
  
  // Control digital output based on switch
  digitalWrite(2, receiverData.switch1Value);
}

void setup()
{
  Serial.begin(9600);
  Serial.println("=== NRF24L01 Receiver ===");
  Serial.println("Initializing...");

  // Attach servos
  ch1.attach(3);   // Roll
  ch2.attach(4);   // Pitch
  ch3.attach(5);   // Throttle
  ch4.attach(6);   // Yaw
  ch5.attach(7);   // AUX1
  ch6.attach(8);   // AUX2
  
  pinMode(2, OUTPUT);  // LED/Digital output

  if (!radio.begin()) {
    Serial.println("NRF24L01 not detected! Check wiring and power.");
    while (1);
  }

  radio.openReadingPipe(1, pipeIn);
  radio.setChannel(108);
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.startListening();

  setInputDefaultValues();
  mapAndWriteValues();

  Serial.println("NRF24L01 initialized successfully!");
  delay(1000);
}

void loop()
{
  // Check if RF is connected and packet is available 
  if (radio.available())
  {
    radio.read(&receiverData, sizeof(PacketData)); 
    lastRecvTime = millis();
    
    Serial.print("Received -> ");
    Serial.print("Roll: "); Serial.print(receiverData.lxAxisValue);
    Serial.print(" | Pitch: "); Serial.print(receiverData.lyAxisValue);
    Serial.print(" | Throttle: "); Serial.print(receiverData.rxAxisValue);
    Serial.print(" | Yaw: "); Serial.print(receiverData.ryAxisValue);
    Serial.print(" | SW1: "); Serial.print(receiverData.switch1Value);
    Serial.print(" | SW2: "); Serial.println(receiverData.switch2Value);
  }
  else
  {
    // Check Signal lost
    unsigned long now = millis();
    if (now - lastRecvTime > SIGNAL_TIMEOUT) 
    {
      setInputDefaultValues();
      Serial.println("Signal lost! Resetting data...");
      lastRecvTime = millis();  // Prevent spam
    }
  }
  
  mapAndWriteValues();
  delay(10);
}