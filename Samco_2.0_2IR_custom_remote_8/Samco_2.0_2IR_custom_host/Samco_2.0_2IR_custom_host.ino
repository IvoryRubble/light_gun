#include <HID.h>
#include <Wire.h>
#include <Keyboard.h>
#include <AbsMouse.h>
#include "SerialTransfer.h"
#include "buttonDebounce.h"

#define bitset(byte, nbit, val) (val ? ((byte) |=  (1<<(nbit))) : ((byte) &= ~(1<<(nbit))))
#define bitcheck(byte,nbit) (!!((byte) & (1<<(nbit))))

enum State {
  mainState = 0,
  calibrationState1 = 1,
  calibrationState2 = 2,
  pauseState = 3
};

// screen resolution
int res_x = 1920;
int res_y = 1080;
// int res_x = 1280;
// int res_y = 720;

// calibration offsets
int xCalibrationOffset = 300;
int yCalibrationOffset = 200;
int xCalibrationPoint2 = 0 + xCalibrationOffset;
int yCalibrationPoint2 = 0 + yCalibrationOffset;
int xCalibrationPoint1 = res_x - xCalibrationOffset;
int yCalibrationPoint1 = res_y - yCalibrationOffset;

// calibration defaults
int xLeftDefault = 100;
int yTopDefault = 150;
int xRightDefault = 600;
int yBottomDefault = 650;

// calibration points
int xLeft = xLeftDefault;
int yTop = yTopDefault;
int xRight = xRightDefault;
int yBottom = yBottomDefault;

// RAW Sensor Values
int rawX[4];
int rawY[4];

// button states
int triggerButtonState = HIGH;
int upButtonState = HIGH;
int downButtonState = HIGH;
int leftButtonState = HIGH;
int rightButtonState = HIGH;
int reloadButtonState = HIGH;

bool isCamAvailable = false;

// detected ir-light spots
int oneX = 0;
int oneY = 0;
int twoX = 0;
int twoY = 0;

// calculated point
int finalX = 0;
int finalY = 0;

int onScreenX_unconstrained = 0;
int onScreenY_unconstrained = 0;

int onScreenX = 0;
int onScreenY = 0;

State state = pauseState;

ButtonDebounce triggerButton;
ButtonDebounce upButton;
ButtonDebounce downButton;
ButtonDebounce leftButton;
ButtonDebounce rightButton;
ButtonDebounce reloadButton;

const bool isOffScreenEnabled = false;
bool isOffScreen = false;
bool isOffScreenButtonPressed = true;

int mouseGuardPin = 4;
bool isInputEnabledByMouseGuard = false;

const unsigned long inputTimeout = 1000;
unsigned long lastRemoteReadTime = 0;
bool isInputEnabledByTimeout = true;

bool buttonsReleased = false;
bool stateResetted = false;

struct RemoteData {
  int rawX[2];
  int rawY[2];
  unsigned int states;
};
struct RemoteData remoteData;

SerialTransfer myTransfer;

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
        mouseMove(xCalibrationPoint2, yCalibrationPoint2);
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
        mouseMove(xCalibrationPoint1, yCalibrationPoint1);
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
          mouseMove(onScreenX, onScreenY);
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

    rawX[0] = remoteData.rawX[0];
    rawY[0] = remoteData.rawY[0];
    rawX[1] = remoteData.rawX[1];
    rawY[1] = remoteData.rawY[1];

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
}

void getPosition() {    // Get tilt adjusted position from IR postioning camera
  if (isCamAvailable) {
    if (rawX[0] != 1023 && rawX[1] != 1023) {

      if (rawX[0] > rawX[1]) {
        oneX = rawX[0];
        oneY = rawY[0];
        twoX = rawX[1];
        twoY = rawY[1];
      } else {
        oneX = rawX[1];
        oneY = rawY[1];
        twoX = rawX[0];
        twoY = rawY[0];
      }

      finalX = 512 + cos(atan2(twoY - oneY, twoX - oneX) * -1) * (((oneX - twoX) / 2 + twoX) - 512) - sin(atan2(twoY - oneY, twoX - oneX) * -1) * (((oneY - twoY) / 2 + twoY) - 384);
      finalY = 384 + sin(atan2(twoY - oneY, twoX - oneX) * -1) * (((oneX - twoX) / 2 + twoX) - 512) + cos(atan2(twoY - oneY, twoX - oneX) * -1) * (((oneY - twoY) / 2 + twoY) - 384);
      onScreenX_unconstrained = map(finalY, yTop, yBottom, xCalibrationPoint2, xCalibrationPoint1);
      onScreenY_unconstrained = map(finalX, xLeft, xRight, yCalibrationPoint2, yCalibrationPoint1);
      onScreenX = constrain(onScreenX_unconstrained, 0, res_x);
      onScreenY = constrain(onScreenY_unconstrained, 0, res_y);
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
  PrintResultsReadable();
  //PrintResultsForProcessing();
  //Serial.flush();
}

void PrintResultsReadable() {    // Print results for debugging
  //  Serial.print("RAW: ");
  //  Serial.print(finalX);
  //  Serial.print(", ");
  //  Serial.print(finalY);
  //
  //  Serial.print("\tCalibration: ");
  //  Serial.print(xLeft);
  //  Serial.print(", ");
  //  Serial.print(yTop);
  //  Serial.print(", ");
  //  Serial.print(xRight);
  //  Serial.print(", ");
  //  Serial.print(yBottom);
  //
  //  Serial.print("\tPosition: ");
  //  Serial.print(onScreenX);
  //  Serial.print(", ");
  //  Serial.print(onScreenY);

  Serial.print("State: ");
  Serial.print(state);

  Serial.print("\tButton states: ");
  Serial.print(triggerButton.btnState);
  Serial.print(", ");
  Serial.print(reloadButton.btnState);

  Serial.print("\tButton pressed event: ");
  Serial.print(triggerButton.isBtnPressed);
  Serial.print(", ");
  Serial.print(reloadButton.isBtnPressed);

  Serial.print("\tisInputEnabledByTimeout: ");
  Serial.print(isInputEnabledByTimeout);

  Serial.print("\tisOffScreen: ");
  Serial.print(isOffScreen);

  Serial.println();
}

void PrintResultsForProcessing() {
  for (int i = 0; i < 4; i++) {
    Serial.print(rawY[i]);
    Serial.print( "," );
    Serial.print(rawX[i]);
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
