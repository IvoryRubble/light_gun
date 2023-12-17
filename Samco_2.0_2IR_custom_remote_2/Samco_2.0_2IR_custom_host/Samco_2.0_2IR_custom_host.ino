
int positionX[4];               // RAW Sensor Values
int positionY[4];

int oneY = 0;                   // Re-mapped so left sensor is always read first
int oneX = 0;
int twoY = 0;
int twoX = 0;

int finalX = 0;                 // Values after tilt correction
int finalY = 0;

int xLeft = 0;                  // Stored calibration points
int yTop = 0;
int xRight = 0;
int yBottom = 0;

int MoveXAxis = 0;              // Unconstrained mouse postion
int MoveYAxis = 0;

int conMoveXAxis = 0;           // Constrained mouse postion
int conMoveYAxis = 0;

int onScreenX = 0;
int onScreenY = 0;

int count = 4;                  // Set intial count

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

int inputGuardPin = 4;
int inputEnabled = 0;

const bool isOffScreenEnabled = true;
bool isOffScreen = false;
bool isOffScreenButtonPressed = true;

struct REMOTE_DATA {
  int positionX[4];
  int positionY[4];
  int finalX;
  int finalY;
  int onScreenX;
  int onScreenY;
  int count;
  int buttonState1;
  int buttonState2;
  int isCamAvailable;
};

struct REMOTE_DATA remoteData;

#include <HID.h>                // Load libraries
#include <Wire.h>
#include <Keyboard.h>
#include <AbsMouse.h>
#include "SerialTransfer.h"

SerialTransfer myTransfer;

int res_x = 1920;               // Put your screen resolution width here
int res_y = 1080;               // Put your screen resolution height here
// int res_x = 1280;
// int res_y = 720;

// #define Serial Serial1
// #define Serial Serial

void setup() {

  Serial.begin(115200);                     // For debugging (make sure your serial monitor has the same baud rate)
  Serial1.begin(115200);
  myTransfer.begin(Serial1);

  AbsMouse.init(res_x, res_y);

  getInputEnabled();

  mouseMove((res_x / 2), (res_y / 2));

  delay(500);

}

void loop() {
  getRemoteData();
  PrintResults(); 
  Serial.flush();
  Serial1.flush();
}


/*        -----------------------------------------------        */
/* --------------------------- METHODS ------------------------- */
/*        -----------------------------------------------        */

void getRemoteData() {
  if (myTransfer.available()) {
    myTransfer.rxObj(remoteData);
    for (int i = 0; i < 4; i++) {
      positionX[i] = remoteData.positionX[i];
      positionY[i] = remoteData.positionY[i];
    };
    finalX = remoteData.finalX;
    finalY = remoteData.finalY;
    onScreenX = remoteData.onScreenX;
    onScreenY = remoteData.onScreenY;
    count = remoteData.count;
    buttonState1 = remoteData.buttonState1;
    buttonState2 = remoteData.buttonState2;
  }
}

void getInputEnabled() {
  //inputEnabled = !digitalRead(inputGuardPin);
  inputEnabled = false;
}

void mouseMove(int x, int y) {
  if (inputEnabled) {
    AbsMouse.move(x, y);          // Set mouse position to centre of the screen
  }
  onScreenX = x;
  onScreenY = y;
}

void PrintResults() {
  //PrintResultsReadable();
  PrintResultsForProcessing();
}

void PrintResultsReadable() {    // Print results for debugging

  Serial.print("RAW: ");
  Serial.print(finalX);
  Serial.print(", ");
  Serial.print(finalY);
  Serial.print("     Count: ");
  Serial.print(count);
  Serial.print("     Calibration: ");
  Serial.print(xLeft);
  Serial.print(", ");
  Serial.print(yTop);
  Serial.print(", ");
  Serial.print(xRight);
  Serial.print(", ");
  Serial.print(yBottom);
  Serial.print("     Position: ");
  Serial.print(conMoveXAxis);
  Serial.print(", ");
  Serial.println(conMoveYAxis);

}

void PrintResultsForProcessing() {
  for (int i = 0; i < 4; i++) {
    Serial.print(positionX[i]);
    Serial.print( "," );
    Serial.print(positionY[i]);
    Serial.print( "," );
  }

  Serial.print(finalX);
  Serial.print(",");
  Serial.print(finalY);
  Serial.print(",");

  Serial.print(onScreenX);
  Serial.print(",");
  Serial.print(onScreenY);
  Serial.print(",");

  Serial.print(count);
  Serial.print(",");

  Serial.print(buttonState1);
  Serial.print(",");
  Serial.print(buttonState2);
  Serial.println();
}
