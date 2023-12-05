#define BUZZER_PIN 10 // Define the pin for the buzzer

// Define the frequencies for the notes
#define NOTE_E4 330
#define NOTE_G4 392
#define NOTE_C4 261
#define NOTE_D4 294
#define NOTE_F4 349

#define BIT_DURATION 100

// Define the notes for Jingle Bells melody
int melody[] = {
  NOTE_E4, NOTE_E4, NOTE_E4,
  NOTE_E4, NOTE_E4, NOTE_E4,
  NOTE_E4, NOTE_G4, NOTE_C4, NOTE_D4,
  NOTE_E4,
  NOTE_F4, NOTE_F4, NOTE_F4, NOTE_F4,
  NOTE_F4, NOTE_E4, NOTE_E4, NOTE_E4, NOTE_E4,
  NOTE_E4, NOTE_D4, NOTE_D4, NOTE_E4, NOTE_D4,
  NOTE_G4
};

// Define the note durations: 4 = quarter note, 8 = eighth note, etc.
int noteDurations[] = {
  8, 8, 4,
  8, 8, 4,
  8, 8, 8, 8,
  2,
  8, 8, 8, 8,
  8, 8, 8, 8, 8,
  8, 8, 8, 8, 4,
  4
};

#include "LSM6DS3.h"
#include "Wire.h"

//Create a instance of class LSM6DS3
LSM6DS3 myIMU(I2C_MODE, 0x6A);    //I2C device address 0x6A

void setup() {
    // put your setup code here, to run once:
    Serial.begin(9600);
    while (!Serial);
    //Call .begin() to configure the IMUs
    if (myIMU.begin() != 0) {
      Serial.println("Device error");
    } else {
      Serial.println("Device OK!");
    }

    pinMode(BUZZER_PIN, OUTPUT); // Set the BUZZER_PIN as an OUTPUT
}

void playJingle() {
  for (int i = 0; i < sizeof(melody) / sizeof(melody[0]); i++) {
    int duration = 1000 / noteDurations[i]; // Calculate the duration of the note

    tone(BUZZER_PIN, melody[i], duration); // Play the note

    int pause = duration * 1.30; // Add a slight pause between notes
    delay(pause);

    noTone(BUZZER_PIN);
    delay(50); // Add a short delay between tones for clarity
  }
}

void sendData() {
  float xac = myIMU.readFloatAccelX();
  float yac = myIMU.readFloatAccelY();
  float zac = myIMU.readFloatAccelZ();

  float xg = myIMU.readFloatGyroX();
  float yg = myIMU.readFloatGyroY();
  float zg = myIMU.readFloatGyroZ();

  String output = String(xac) + "," + String(yac) + "," + String(zac) + "," + String(xg) + "," + String(yg) + "," + String(zg);
  Serial.println(output);
}

String stringToBinary(String text) {
  String binaryText = "";

  for (int i = 0; i < text.length(); i++) {
    char c = text.charAt(i);
    int asciiValue = int(c);

    // Convert ASCII value to binary representation
    String binaryChar = String(asciiValue, BIN);

    // Pad with zeroes to ensure 8 bits per character
    while (binaryChar.length() < 8) {
      binaryChar = "0" + binaryChar;
    }

    binaryText += binaryChar + " "; // Add space between each character's binary representation
  }

  return binaryText;
}

void playBinary(String binaryString) {
  Serial.println(binaryString);
  for (int i = 0; i < binaryString.length(); i++) {
    if (binaryString.charAt(i) == '0') {
      noTone(BUZZER_PIN); // No sound for '0'
      delay(BIT_DURATION);
    } else if (binaryString.charAt(i) == '1') {
      tone(BUZZER_PIN, NOTE_E4); // Sound for '1' at a specific frequency
      delay(BIT_DURATION);
    }
    noTone(BUZZER_PIN);
  }
}

void sendProtocol(String lat, String lon) {
  Serial.println("Sent starting protocal message");
  playBinary(stringToBinary("666"));
  
  Serial.println("Sent lat");
  playBinary(stringToBinary(lat));
  Serial.println("Sent middle header");
  playBinary(stringToBinary("--"));
  Serial.println("Sent lon");
  playBinary(stringToBinary(lon));

  //checksum calc
  Serial.println("Sent checksum");
  playBinary(stringToBinary("09812128903")); //hard code checksum

  Serial.println("Sent ending protocal message");
  playBinary(stringToBinary("999"));
}

void loop() {
  String lat = "38.722252";
  String lon = "-9.139337";

  // sendProtocol(lat, lon);
  // sendData();
  
  // playJingle();

}
