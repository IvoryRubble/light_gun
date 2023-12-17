
int positionX[4];               // RAW Sensor Values
int positionY[4];

int oneY = 0;                   // Re-mapped so left sensor is always read first
int oneX = 0;
int twoY = 0;
int twoX = 0;

int _tigger = A0;               // Label Pin to buttons
int _up = 5;
int _down = 6;
int _left = 7;
int _right = 8;
int _A = A1;
int _B = A1;
int _start = A1;
int _select = A1;
int _reload = 9;
int _pedal = A1;

int buttonState1 = 0;
int lastButtonState1 = 0;
int buttonState2 = 0;
int lastButtonState2 = 0;
int buttonState3 = 0;
int lastButtonState3 = 0;
int buttonState4 = 0;
int lastButtonState4 = 0;
int buttonState5 = 0;
int lastButtonState5 = 0;
int buttonState6 = 0;
int lastButtonState6 = 0;
int buttonState7 = 0;
int lastButtonState7 = 0;
int buttonState8 = 0;
int lastButtonState8 = 0;
int buttonState9 = 0;
int lastButtonState9 = 0;
int buttonState10 = 0;
int lastButtonState10 = 0;

int ledG = 12;
int ledR = 15; //11;
int ledB = 14; //10;

bool isCamAvailable = false;

struct REMOTE_DATA {
  int positionX[2];
  int positionY[2];
  unsigned int states;
};

struct REMOTE_DATA remoteData;

#include <Wire.h>
#include <DFRobotIRPosition.h>
#include "SerialTransfer.h"

#define bitset(byte, nbit, val) (val ? ((byte) |=  (1<<(nbit))) : ((byte) &= ~(1<<(nbit))))
#define bitcheck(byte,nbit) (!!((byte) & (1<<(nbit))))

SerialTransfer myTransfer;

DFRobotIRPosition myDFRobotIRPosition;

int res_x = 1920;               // Put your screen resolution width here
int res_y = 1080;               // Put your screen resolution height here
// int res_x = 1280;
// int res_y = 720;

#define Serial Serial1
//#define Serial Serial

void setup() {
  Serial.begin(115200);                     // For debugging (make sure your serial monitor has the same baud rate)
  myTransfer.begin(Serial);

  pinMode(_tigger, INPUT_PULLUP);         // Set pin modes
  pinMode(_up, INPUT_PULLUP);
  pinMode(_down, INPUT_PULLUP);
  pinMode(_left, INPUT_PULLUP);
  pinMode(_right, INPUT_PULLUP);         // Set pin modes
  pinMode(_A, INPUT_PULLUP);
  pinMode(_B, INPUT_PULLUP);
  pinMode(_start, INPUT_PULLUP);
  pinMode(_select, INPUT_PULLUP);
  pinMode(_reload, INPUT_PULLUP);

  pinMode(ledR, OUTPUT);
  pinMode(ledG, OUTPUT);
  pinMode(ledB, OUTPUT);

  myDFRobotIRPosition.begin();            // Start IR Camera

  delay(500);
}

void loop() {
  mouseButtons();
  getPosition();

  PrintResults();

  delay(10);
}


/*        -----------------------------------------------        */
/* --------------------------- METHODS ------------------------- */
/*        -----------------------------------------------        */

void getPosition() {    // Get tilt adjusted position from IR postioning camera

  myDFRobotIRPosition.requestPosition();

  if (myDFRobotIRPosition.available()) {
    isCamAvailable = true;
    for (int i = 0; i < 4; i++) {
      positionX[i] = myDFRobotIRPosition.readX(i);
      positionY[i] = myDFRobotIRPosition.readY(i);
    }

    digitalWrite(ledR, (positionX[0] != 1023));
    digitalWrite(ledB, (positionX[1] != 1023));

    if (positionX[0] != 1023 && positionX[1] != 1023) {

//      if (positionX[0] > positionX[1]) {
//        oneY = positionY[0];
//        oneX = positionX[0];
//        twoY = positionY[1];
//        twoX = positionX[1];
//      }
//      else {
//        oneY = positionY[1];
//        oneX = positionX[1];
//        twoY = positionY[0];
//        twoX = positionX[0];
//      }
    }
  } else {
    //Serial.println("Device not available!");
    isCamAvailable = false;
    digitalWrite(ledR, 0);
    digitalWrite(ledB, 0);
  }
}

void mouseButtons() {    // Setup Left, Right & Middle Mouse buttons
  buttonState1 = digitalRead(_reload);
  buttonState2 = digitalRead(_tigger);
  buttonState3 = digitalRead(_up);
  buttonState4 = digitalRead(_down);
  buttonState5 = digitalRead(_left);
  buttonState6 = digitalRead(_right);
  buttonState7 = digitalRead(_A);
  buttonState8 = digitalRead(_B);
  buttonState9 = digitalRead(_start);
  buttonState10 = digitalRead(_select);

  lastButtonState1 = buttonState1;
  lastButtonState2 = buttonState2;
  lastButtonState3 = buttonState3;
  lastButtonState4 = buttonState4;
  lastButtonState5 = buttonState5;
  lastButtonState6 = buttonState6;
  lastButtonState7 = buttonState7;
  lastButtonState8 = buttonState8;
  lastButtonState9 = buttonState9;
  lastButtonState10 = buttonState10;
}

void PrintResults() {
  //PrintResultsReadable();
  //PrintResultsForProcessing();
  PrintResultsRemote();
}

void PrintResultsForProcessing() {    // Print results for debugging
  for (int i = 0; i < 4; i++) {
    Serial.print(positionX[i]);
    Serial.print( "," );
    Serial.print(positionY[i]);
    Serial.print( "," );
  }

  Serial.print(buttonState1);
  Serial.print(",");
  Serial.print(buttonState2);
  Serial.println();
}

void PrintResultsRemote() {
  remoteData.positionX[0] = positionX[0];
  remoteData.positionY[0] = positionY[0];
  remoteData.positionX[1] = positionX[1];
  remoteData.positionY[1] = positionY[1];

  //remoteData.finalX = finalX;
  //remoteData.finalY = finalY;
  //remoteData.onScreenX = onScreenX;
  //remoteData.onScreenY = onScreenY;
  //remoteData.count = count;

  bitset(remoteData.states, 0, buttonState1);
  bitset(remoteData.states, 1, buttonState2);
  bitset(remoteData.states, 2, buttonState3);
  bitset(remoteData.states, 3, buttonState4);
  bitset(remoteData.states, 4, buttonState5);
  bitset(remoteData.states, 5, buttonState6);
  bitset(remoteData.states, 6, isCamAvailable);
  myTransfer.sendDatum(remoteData);
  Serial.flush();
}
