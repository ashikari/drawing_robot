
#include<SPI.h>
#include "Encoder.h"

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  initEncoders();
  clearEncoderCounts();
}


int enc1 = 0;
int enc2 = 0;
int enc3 = 0;
int enc4 = 0;

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print("Encoder 1: ");
  enc1 = readEncoder(1);
  Serial.print(enc1);
  Serial.print(" Encoder 2: ");
  enc2 = readEncoder(2);
  Serial.print(enc2);
  Serial.print(" Encoder 3: ");
  enc3 = readEncoder(3);
  Serial.print(enc3);
  Serial.print(" Encoder 4: ");
  enc4 = readEncoder(4);
  Serial.println(enc4);
}
