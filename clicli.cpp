/*

CLI Software for Arduino
A Simple Command Line Interface

Feature                |  CLI Usage
___________________________________________________
Digital Write HIGH to a specific pin |  h (pin)
Digital Write LOW to a specific pin  |  l (pin)
Analog Write to PWM ports           |  a (pin) (value)
Digital Read                        |  r (pin)
Analog Read                         |  e (pin)


by Gal Arbel
Oct 2022

Credits: Shimi Mahluf

*/

#include "clicli.h"
#include "Arduino.h"

const unsigned int MAX_MESSAGE_LENGTH = 64;

clicli::clicli(M_Robot &move) : myrobot(move), number(7) {
}

void clicli::begin() {
  Serial.begin(115200);
}

void clicli::run() {

  // CLI - Messages from Terminal
  while (Serial.available() > 0) {
    char message[MAX_MESSAGE_LENGTH];
    static unsigned int message_pos = 0;
    char inByte = Serial.read();   // Read the next available byte in the serial receive buffer
    if (inByte != '\n' && (message_pos < MAX_MESSAGE_LENGTH - 1)) {
      message[message_pos] = inByte;  // Add the incoming byte to our message
      message_pos++;
    }
    // Full message received...
    else {
      message[message_pos] = '\0';     // Add null character to string
      Serial.println(message);     // Echo the message to terminal

      int command[4];
      int argindex = 0;
      char cmd;
      char delim[] = " ";
      char tmpmsg[MAX_MESSAGE_LENGTH];
      strcpy(tmpmsg, message);
      message_pos = 0;
      message[message_pos] = '\0';     // Add null character to string

      char *ptr = strtok(tmpmsg, delim);
      while (ptr != NULL) {
        if (argindex == 0) {
          cmd = ptr[0];
        }
        command[argindex] = atoi(ptr);
        argindex++;
        ptr = strtok(NULL, delim);
      }

      switch (cmd) {
        case 'h': // Set port to HIGH
          pinMode(command[1], OUTPUT);
          digitalWrite(command[1], HIGH);
          Serial.print("Pin ");
          Serial.print(command[1]);
          Serial.println(" is SET");
          delay(1000);
          break;
        case 'l': // Set port to LOW
          pinMode(command[1], OUTPUT);
          digitalWrite(command[1], LOW);
          Serial.print("Pin ");
          Serial.print(command[1]);
          Serial.println(" is RESET");
          delay(1000);
          break;
        case 'a': // Analog Write to PWM ports
          pinMode(command[1], OUTPUT);
          analogWrite(command[1], command[2]);
          Serial.print("Writing ");
          Serial.print(command[2]);
          Serial.print(" to pin ");
          Serial.println(command[1]);
          delay(1000);
          break;
        case 'r': // Digital Read
          pinMode(command[1], INPUT);
          Serial.print("Pin ");
          Serial.print(command[1]);
          Serial.print(" Value = ");
          Serial.println(digitalRead(command[1]));
          delay(1000);
          break;
        case 'e': // Analog Read
          pinMode(command[1], INPUT);
          Serial.print("Pin ");
          Serial.print(command[1]);
          Serial.print(" Value = ");
          Serial.println(analogRead(command[1]));
          delay(1000);
          break;
        case 'F':
          myrobot.moveF(command[1]);
          break;
        case 'B':
          myrobot.moveB(command[1]);
          break;
        case 'n':
          myrobot.moveR(command[1]);
          break;
        case 'p':
          myrobot.moveL(command[1]);
          break;
        case 'R':
          myrobot.turnR(command[1]);
          break;
        case 'L':
          myrobot.turnL(command[1]);
          break;
      }

      message_pos = 0;     // Reset for the next message
    }

    delay(60);
  }
}
