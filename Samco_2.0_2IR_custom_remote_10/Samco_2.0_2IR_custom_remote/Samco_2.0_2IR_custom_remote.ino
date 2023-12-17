
int positionX[4];               // RAW Sensor Values
int positionY[4];

const int tiggerButton = A0;               // Label Pin to buttons
const int upButton = 5;
const int downButton = 6;
const int leftButton = 7;
const int rightButton = 8;
const int reloadButton = 9;

int tiggerButtonState = 0;
int upButtonState = 0;
int downButtonState = 0;
int leftButtonState = 0;
int rightButtonState = 0;
int reloadButtonState = 0;

const int ledG = 12;
const int ledR = 15; //11;
const int ledB = 14; //10;

bool isCamAvailable = false;

struct RemoteData {
  int positionX[2];
  int positionY[2];
  unsigned int states;
};

struct RemoteData remoteData;

#include <Wire.h>
#include <DFRobotIRPosition.h>
#include "SerialTransfer.h"

#define bitset(byte, nbit, val) (val ? ((byte) |=  (1<<(nbit))) : ((byte) &= ~(1<<(nbit))))
#define bitcheck(byte,nbit) (!!((byte) & (1<<(nbit))))

SerialTransfer myTransfer;
DFRobotIRPosition myDFRobotIRPosition;

#define Serial Serial1
//#define Serial Serial

void setup() {
  Serial.begin(115200);
  myTransfer.begin(Serial);

  pinMode(tiggerButton, INPUT_PULLUP);
  pinMode(upButton, INPUT_PULLUP);
  pinMode(downButton, INPUT_PULLUP);
  pinMode(leftButton, INPUT_PULLUP);
  pinMode(rightButton, INPUT_PULLUP);
  pinMode(reloadButton, INPUT_PULLUP);

  pinMode(ledR, OUTPUT);
  pinMode(ledG, OUTPUT);
  pinMode(ledB, OUTPUT);

  myDFRobotIRPosition.begin();            // Start IR Camera

  delay(1000);
}

void loop() {
  getButtons();
  getCamData();

  PrintResults();

  delay(2);
}

void getCamData() {
  myDFRobotIRPosition.requestPosition();

  if (myDFRobotIRPosition.available()) {
    isCamAvailable = true;
    for (int i = 0; i < 4; i++) {
      positionX[i] = myDFRobotIRPosition.readX(i);
      positionY[i] = myDFRobotIRPosition.readY(i);
    }

    digitalWrite(ledR, (positionX[0] != 1023));
    digitalWrite(ledB, (positionX[1] != 1023));
  } else {
    //Serial.println("Device not available!");
    isCamAvailable = false;
    digitalWrite(ledR, 0);
    digitalWrite(ledB, 0);
  }
}

void getButtons() {   
  reloadButtonState = digitalRead(reloadButton);
  tiggerButtonState = digitalRead(tiggerButton);
  upButtonState = digitalRead(upButton);
  downButtonState = digitalRead(downButton);
  leftButtonState = digitalRead(leftButton);
  rightButtonState = digitalRead(rightButton);
}

void PrintResults() {
  //PrintResultsReadable();
  //PrintResultsForProcessing();
  PrintResultsRemote();
  Serial.flush();
}

void PrintResultsForProcessing() {
  for (int i = 0; i < 4; i++) {
    Serial.print(positionX[i]);
    Serial.print( "," );
    Serial.print(positionY[i]);
    Serial.print( "," );
  }

  Serial.print(tiggerButtonState);
  Serial.print(",");
  Serial.print(upButtonState);
  Serial.println();
}

void PrintResultsRemote() {
  remoteData.positionX[0] = positionX[0];
  remoteData.positionY[0] = positionY[0];
  remoteData.positionX[1] = positionX[1];
  remoteData.positionY[1] = positionY[1];

  bitset(remoteData.states, 0, tiggerButtonState);
  bitset(remoteData.states, 1, upButtonState);
  bitset(remoteData.states, 2, downButtonState);
  bitset(remoteData.states, 3, leftButtonState);
  bitset(remoteData.states, 4, rightButtonState);
  bitset(remoteData.states, 5, reloadButtonState);
  bitset(remoteData.states, 6, isCamAvailable);
  myTransfer.sendDatum(remoteData);
}
