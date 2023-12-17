#include <HID.h>                // Load libraries
#include <Wire.h>
#include <Keyboard.h>
#include <AbsMouse.h>
#include "SerialTransfer.h"
#include "buttonDebounce.h"

enum State {
  mainState = 0,
  calibrationState1 = 1,
  calibrationState2 = 2,
  pauseState = 3
};

int positionX[4];               // RAW Sensor Values
int positionY[4];

int oneY = 0;                   // Re-mapped so left sensor is always read first
int oneX = 0;
int twoY = 0;
int twoX = 0;

int finalX = 0;                 // Values after tilt correction
int finalY = 0;

int xLeftDefault = 100;                  // Stored calibration points
int yTopDefault = 150;
int xRightDefault = 600;
int yBottomDefault = 650;

int xLeft = xLeftDefault;                  // Stored calibration points
int yTop = yTopDefault;
int xRight = xRightDefault;
int yBottom = yBottomDefault;

int MoveXAxis = 0;              // Unconstrained mouse postion
int MoveYAxis = 0;

int conMoveXAxis = 0;           // Constrained mouse postion
int conMoveYAxis = 0;

int onScreenX = 0;
int onScreenY = 0;

State state = pauseState;

ButtonDebounce triggerButton;
ButtonDebounce upButton;
ButtonDebounce downButton;
ButtonDebounce leftButton;
ButtonDebounce rightButton;
ButtonDebounce reloadButton;


int triggerButtonState = 0;
int upButtonState = 0;
int downButtonState = 0;
int leftButtonState = 0;
int rightButtonState = 0;
int reloadButtonState = 0;

const bool isOffScreenEnabled = true;
bool isOffScreen = false;
bool isOffScreenButtonPressed = true;

bool isCamAvailable = false;

int mouseGuardPin = 4;
bool isInputEnabledByMouseGuard = false;

unsigned long inputTimeout = 5000;
unsigned long lastRemoteReadTime = 0;
bool isInputEnabledByTimeout = true;

bool buttonsReleased = false;
bool stateResetted = false;


struct RemoteData {
  int positionX[2];
  int positionY[2];
  unsigned int states;
};

struct RemoteData remoteData;

#define bitset(byte, nbit, val) (val ? ((byte) |=  (1<<(nbit))) : ((byte) &= ~(1<<(nbit))))
#define bitcheck(byte,nbit) (!!((byte) & (1<<(nbit))))

SerialTransfer myTransfer;

int res_x = 1920;
int res_y = 1080;
// int res_x = 1280;
// int res_y = 720;

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  myTransfer.begin(Serial1, false);

  pinMode(mouseGuardPin, INPUT_PULLUP);

  AbsMouse.init(res_x, res_y);
  delay(5000);

  getInputEnabledByMouseGuard();
  mouseMove((res_x / 2), (res_y / 2));
  mouseMove((res_x / 2), (res_y / 2));

  lastRemoteReadTime = millis();
}

void loop() {
  // return;

  getRemoteData();
  getInputEnabledByMouseGuard();
  checkInputEnabled();
  getPosition();
  updateButtons();

  switch (state) {
    case pauseState: {
        if (reloadButton.isBtnPressed) {
          releaseButtons();
          state = mainState;
        }
        if (triggerButton.isBtnPressed) {
          releaseButtons();
          state = calibrationState2;
        }
      }
      break;
    case calibrationState2: {
        mouseMove(300, 200);
        if (reloadButton.isBtnPressed) {
          releaseButtons();
          state = pauseState;
        }
        if (triggerButton.isBtnPressed) {
          releaseButtons();
          xLeft = finalX;
          yTop = finalY;
          state = calibrationState1;
        }
      }
      break;
    case calibrationState1: {
        mouseMove(res_x - 300, res_y - 200);
        if (reloadButton.isBtnPressed) {
          releaseButtons();
          resetCalibration();
          state = pauseState;
        }
        if (triggerButton.isBtnPressed) {
          releaseButtons();
          xRight = finalX;
          yBottom = finalY;
          state = mainState;
        }
      }
      break;
    case mainState: {
        setButtons();
        if (!isOffScreen) {
          mouseMove(conMoveXAxis, conMoveYAxis);
        }
        if (reloadButton.isBtnPressed) {
          releaseButtons();
          state = pauseState;
        }
      }
      break;
  }

  PrintResults();
  //delay(500);
}

void getRemoteData() {
  unsigned long currentTime = millis();
  isInputEnabledByTimeout = (currentTime - lastRemoteReadTime < inputTimeout);
  while (myTransfer.available()) {
    lastRemoteReadTime = currentTime;

    myTransfer.rxObj(remoteData);

    positionX[0] = remoteData.positionX[0];
    positionY[0] = remoteData.positionY[0];
    positionX[1] = remoteData.positionX[1];
    positionY[1] = remoteData.positionY[1];

    triggerButtonState = bitcheck(remoteData.states, 0);
    upButtonState = bitcheck(remoteData.states, 1);
    downButtonState = bitcheck(remoteData.states, 2);
    leftButtonState = bitcheck(remoteData.states, 3);
    rightButtonState = bitcheck(remoteData.states, 4);
    reloadButtonState = bitcheck(remoteData.states, 5);
    isCamAvailable = bitcheck(remoteData.states, 6);
  }
}

void getInputEnabledByMouseGuard() {
  isInputEnabledByMouseGuard = !digitalRead(mouseGuardPin);
  //isInputEnabledByMouseGuard = false;
}

void checkInputEnabled() {
  if (!isInputEnabledByTimeout) {
    if (!buttonsReleased) {
      releaseButtons();
      buttonsReleased = true;
    }
    if (!stateResetted) {
      state = pauseState;
      stateResetted = true;
    }
  }
  if (!isInputEnabledByMouseGuard) {
    if (!buttonsReleased) {
      releaseButtons();
      buttonsReleased = true;
    }
  }
  if (isInputEnabledByTimeout) {
    stateResetted = false;
  }
  if (isInputEnabledByMouseGuard && isInputEnabledByTimeout) {
    buttonsReleased = false;
  }
}

void mouseMove(int x, int y) {
  if (isInputEnabledByMouseGuard && isInputEnabledByTimeout) {
    AbsMouse.move(x, y);
  }
  onScreenX = x;
  onScreenY = y;
}

void getPosition() {    // Get tilt adjusted position from IR postioning camera
  if (isCamAvailable) {
    if (positionX[0] != 1023 && positionX[1] != 1023) {

      if (positionX[0] > positionX[1]) {
        oneY = positionY[0];
        oneX = positionX[0];
        twoY = positionY[1];
        twoX = positionX[1];
      } else {
        oneY = positionY[1];
        oneX = positionX[1];
        twoY = positionY[0];
        twoX = positionX[0];
      }

      finalX = 512 + cos(atan2(twoY - oneY, twoX - oneX) * -1) * (((oneX - twoX) / 2 + twoX) - 512) - sin(atan2(twoY - oneY, twoX - oneX) * -1) * (((oneY - twoY) / 2 + twoY) - 384);
      finalY = 384 + sin(atan2(twoY - oneY, twoX - oneX) * -1) * (((oneX - twoX) / 2 + twoX) - 512) + cos(atan2(twoY - oneY, twoX - oneX) * -1) * (((oneY - twoY) / 2 + twoY) - 384);
      MoveXAxis = map(finalY, yTop, yBottom, 300, (res_x - 300));
      MoveYAxis = map(finalX, xLeft, xRight, 200, (res_y - 200));
      conMoveXAxis = constrain(MoveXAxis, 0, res_x);
      conMoveYAxis = constrain(MoveYAxis, 0, res_y);
      isOffScreen = false;
    } else {
      isOffScreen = true;
    }
  }
}

void updateButtons() {
  triggerButton.updateState(triggerButtonState);
  upButton.updateState(upButtonState);
  downButton.updateState(downButtonState);
  leftButton.updateState(leftButtonState);
  rightButton.updateState(rightButtonState);
  reloadButton.updateState(reloadButtonState);
}

void setButton(ButtonDebounce btn, byte val) {
  if (btn.isBtnPressed) {
    Keyboard.press(val);
  } else if (btn.isBtnReleased) {
    Keyboard.release(val);
  }
}

void setButtons() {
  if (isInputEnabledByMouseGuard && isInputEnabledByTimeout) {
    if (triggerButton.isBtnPressed) {
      isOffScreenButtonPressed = isOffScreenEnabled && isOffScreen;
      if (isOffScreenButtonPressed) {
        AbsMouse.press(MOUSE_RIGHT);
      } else {
        AbsMouse.press(MOUSE_LEFT);
      }
    } else if (triggerButton.isBtnReleased) {
      if (isOffScreenButtonPressed) {
        AbsMouse.release(MOUSE_RIGHT);
      } else {
        AbsMouse.release(MOUSE_LEFT);
      }
    }

    setButton(upButton, 'w' /*KEY_UP_ARROW*/);
    setButton(downButton, 's' /*KEY_DOWN_ARROW*/);
    setButton(leftButton, 'a' /*KEY_LEFT_ARROW*/);
    setButton(rightButton, 'd' /*KEY_RIGHT_ARROW*/);
  }
}

void releaseButtons() {
  AbsMouse.release(MOUSE_RIGHT);
  AbsMouse.release(MOUSE_LEFT);
  Keyboard.releaseAll();
}

void resetCalibration() {
  xLeft = xLeftDefault;
  yTop = yTopDefault;
  xRight = xRightDefault;
  yBottom = yBottomDefault;
}

void PrintResults() {
  if (Serial.availableForWrite() < 32) {
    return;
  }
  //PrintResultsReadable();
  PrintResultsForProcessing();
  //Serial.flush();
}

void PrintResultsReadable() {    // Print results for debugging
  Serial.print("RAW: ");
  Serial.print(finalX);
  Serial.print(", ");
  Serial.print(finalY);
  Serial.print("\tState: ");
  Serial.print(state);
  Serial.print("\tCalibration: ");
  Serial.print(xLeft);
  Serial.print(", ");
  Serial.print(yTop);
  Serial.print(", ");
  Serial.print(xRight);
  Serial.print(", ");
  Serial.print(yBottom);
  Serial.print("\tPosition: ");
  Serial.print(conMoveXAxis);
  Serial.print(", ");
  Serial.print(conMoveYAxis);
  Serial.print("\tButton states: ");
  Serial.print(triggerButton.btnState);
  Serial.print(", ");
  Serial.print(reloadButton.btnState);
  Serial.print("\tButton pressed event: ");;
  Serial.print(triggerButton.isBtnPressed);
  Serial.print(", ");
  Serial.print(reloadButton.isBtnPressed);
  Serial.print("\tisInputEnabledByTimeout: ");
  Serial.print(isInputEnabledByTimeout);
  Serial.println();

}

void PrintResultsForProcessing() {
  for (int i = 0; i < 4; i++) {
    Serial.print(positionY[i]);
    Serial.print( "," );
    Serial.print(positionX[i]);
    Serial.print( "," );
  }

  Serial.print(finalY);
  Serial.print(",");
  Serial.print(finalX);
  Serial.print(",");

  Serial.print(onScreenX);
  Serial.print(",");
  Serial.print(onScreenY);
  Serial.print(",");

  Serial.print(state);
  Serial.print(",");

  Serial.print(triggerButtonState);
  Serial.print(",");
  Serial.print(reloadButtonState);
  Serial.println();
}
