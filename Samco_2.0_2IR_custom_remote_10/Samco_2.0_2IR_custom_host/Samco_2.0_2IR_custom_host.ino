#include <HID.h>
#include <Wire.h>
#include <Keyboard.h>
#include <AbsMouse.h>
#include <EEPROM.h>
#include "SerialTransfer.h"
#include "buttonDebounce.h"

#define bitset(byte, nbit, val) (val ? ((byte) |=  (1<<(nbit))) : ((byte) &= ~(1<<(nbit))))
#define bitcheck(byte,nbit) (!!((byte) & (1<<(nbit))))

const char helpTitle[] = "Please stand by... \n\n";

const char helpMessage[] = "LightGun \n\n"

                           "Based on: \n"
                           "https://www.dfrobot.com/product-1088.html \n"
                           "https://github.com/samuelballantyne/IR-Light-Gun \n\n"

                           "Side leds: red and blue for two detected points \n"
                           "Side button: pause \n"
                           "Side button + up button (hold both for 2 seconds): calibrate \n"
                           "Side button + down button (hold both for 2 seconds): print help \n"
                           "Side button + left button (hold both for 2 seconds): reset \n\n\n\n"
                           "Gthtrk.xbnt zpsr rkfdbfnehs yf fyukbqcrbq \n\n"; // Переключите язык клавиатуры на английский

enum State {
  mainState = 0,
  calibrationState1 = 1,
  calibrationState2 = 2,
  pauseState = 3,
  noConnectionState = 4
};
State state = noConnectionState;
State previousStateForInputEnabled = mainState;

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
int xCalibration2Default = 490;
int yCalibration2Default = 160;
int xCalibration1Default = 690;
int yCalibration1Default = 610;

// calibration points
int xCalibration2 = xCalibration2Default;
int yCalibration2 = yCalibration2Default;
int xCalibration1 = xCalibration1Default;
int yCalibration1 = yCalibration1Default;

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

// detected ir-light spots in different orders
int topX = 0;
int topY = 0;
int bottomX = 0;
int bottomY = 0;

int leftX = 0;
int leftY = 0;
int rightX = 0;
int rightY = 0;

// calculated point
int finalX = 0;
int finalY = 0;

int onScreenX_unconstrained = 0;
int onScreenY_unconstrained = 0;

int onScreenX = 0;
int onScreenY = 0;

int mouseMoveX = 0;
int mouseMoveY = 0;

ButtonDebounce triggerButton;
ButtonDebounce upButton;
ButtonDebounce downButton;
ButtonDebounce leftButton;
ButtonDebounce rightButton;
ButtonDebounce reloadButton;

const bool isOffScreenEnabled = true;
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

struct StorageData {
  int xCalibration2;
  int yCalibration2;
  int xCalibration1;
  int yCalibration1;
  int hash;
  int calcHash() {
    return xCalibration2 + yCalibration2 + xCalibration1 + yCalibration1 + 11;
  }
  void updateHash() {
    hash = calcHash();
  }
  bool checkHash() {
    return hash == calcHash();
  }
};
struct StorageData storageData;
const int storageDataAddress = 16;

SerialTransfer myTransfer;

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  myTransfer.begin(Serial1, false);
  pinMode(mouseGuardPin, INPUT_PULLUP);
  AbsMouse.init(res_x, res_y);
  getStorageData();
  delay(5000);

  // getInputEnabledByMouseGuard();
  // mouseMove((res_x / 2), (res_y / 2));
  // mouseMove((res_x / 2), (res_y / 2));

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
    case noConnectionState: {
        if (isInputEnabledByTimeout) {
          state = previousStateForInputEnabled;
        }
      }
      break;
    case pauseState: {
        if (isCalibrateButtonsReleased()) {
          releaseButtons();
          state = calibrationState2;
        } else if (reloadButton.isBtnReleased || triggerButton.isBtnPressed || isRegularButtonPressed()) {
          releaseButtons();
          state = mainState;
        }
      }
      break;
    case calibrationState2: {
        mouseMove(xCalibrationPoint2, yCalibrationPoint2);
        if (reloadButton.isBtnReleased || isRegularButtonPressed()) {
          releaseButtons();
          state = mainState;
        }
        if (triggerButton.isBtnPressed) {
          releaseButtons();
          xCalibration2 = finalX;
          yCalibration2 = finalY;
          state = calibrationState1;
        }
      }
      break;
    case calibrationState1: {
        mouseMove(xCalibrationPoint1, yCalibrationPoint1);
        if (isCalibrateButtonsReleased() || reloadButton.isBtnReleased || isRegularButtonPressed()) {
          releaseButtons();
          resetCalibration();
          state = mainState;
        }
        if (triggerButton.isBtnPressed) {
          releaseButtons();
          xCalibration1 = finalX;
          yCalibration1 = finalY;
          putStorageData();
          state = mainState;
        }
      }
      break;
    case mainState: {
        setButtons();
        if (!isOffScreen) {
          mouseMove(onScreenX, onScreenY);
        }
        if (isCalibrateButtonsReleased()) {
          releaseButtons();
          state = calibrationState2;
        } else if (reloadButton.isBtnReleased) {
          releaseButtons();
          state = pauseState;
        }
      }
      break;
  }

  if (isResetButtonsReleased()) {
    resetAll();
  }

  if (isHelpButtonsReleased()) {
    printHelp();
  }

  PrintResults();
  // delay(100);
}

void putStorageData() {
  storageData.xCalibration2 = xCalibration2;
  storageData.yCalibration2 = yCalibration2;
  storageData.xCalibration1 = xCalibration1;
  storageData.yCalibration1 = yCalibration1;
  storageData.updateHash();
  EEPROM.put(storageDataAddress, storageData);
}

void getStorageData() {
  EEPROM.get(storageDataAddress, storageData);
  if (storageData.checkHash()) {
    xCalibration2 = storageData.xCalibration2;
    yCalibration2 = storageData.yCalibration2;
    xCalibration1 = storageData.xCalibration1;
    yCalibration1 = storageData.yCalibration1;
  }
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
      previousStateForInputEnabled = state;
      state = noConnectionState;
      stateResetted = true;
    }
  }
  if (!isInputEnabledByMouseGuard) {
    if (!buttonsReleased) {
      releaseButtons();
      buttonsReleased = true;
    }
  }
  if (reloadButton.btnState) {
    if (!buttonsReleased) {
      releaseButtons();
      buttonsReleased = true;
    }
  }

  if (isInputEnabledByTimeout) {
    stateResetted = false;
  }
  if (isInputEnabledByMouseGuard && isInputEnabledByTimeout && !reloadButton.btnState) {
    buttonsReleased = false;
  }
}

void mouseMove(int x, int y) {
  if (isInputEnabledByMouseGuard && isInputEnabledByTimeout && !reloadButton.btnState) {
    AbsMouse.move(x, y);
  }
  mouseMoveX = x;
  mouseMoveY = y;
}

void getPosition() {
  if (isCamAvailable) {
    if (rawX[0] != 1023 && rawX[1] != 1023) {

      if (rawX[0] > rawX[1]) {
        topX = rawX[0];
        topY = rawY[0];
        bottomX = rawX[1];
        bottomY = rawY[1];
      } else {
        topX = rawX[1];
        topY = rawY[1];
        bottomX = rawX[0];
        bottomY = rawY[0];
      }

      if (rawY[0] > rawY[1]) {
        leftX = rawX[0];
        leftY = rawY[0];
        rightX = rawX[1];
        rightY = rawY[1];
      } else {
        leftX = rawX[1];
        leftY = rawY[1];
        rightX = rawX[0];
        rightY = rawY[0];
      }

      finalX = 512 + cos(atan2(bottomY - topY, bottomX - topX) * -1) * (((topX - bottomX) / 2 + bottomX) - 512) - sin(atan2(bottomY - topY, bottomX - topX) * -1) * (((topY - bottomY) / 2 + bottomY) - 384);
      finalY = 384 + sin(atan2(bottomY - topY, bottomX - topX) * -1) * (((topX - bottomX) / 2 + bottomX) - 512) + cos(atan2(bottomY - topY, bottomX - topX) * -1) * (((topY - bottomY) / 2 + bottomY) - 384);

      // swap X and Y
      onScreenX_unconstrained = map(finalY, yCalibration2, yCalibration1, xCalibrationPoint2, xCalibrationPoint1);
      onScreenY_unconstrained = map(finalX, xCalibration2, xCalibration1, yCalibrationPoint2, yCalibrationPoint1);
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
  if (isInputEnabledByMouseGuard && isInputEnabledByTimeout && !reloadButton.btnState) {
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

bool isRegularButtonPressed() {
  return
    upButton.isBtnPressed ||
    downButton.isBtnPressed ||
    leftButton.isBtnPressed ||
    rightButton.isBtnPressed;
}

bool isCalibrateButtonsReleased() {
  return reloadButton.isBtnReleasedLongPress && upButton.btnState;
}

bool isResetButtonsReleased() {
  return reloadButton.isBtnReleasedLongPress && leftButton.btnState;
}

bool isHelpButtonsReleased() {
  return reloadButton.isBtnReleasedLongPress && downButton.btnState;
}

void resetAll() {
  Serial.println("Reset all");
  storageData.xCalibration2 = 0;
  storageData.yCalibration2 = 0;
  storageData.xCalibration1 = 0;
  storageData.yCalibration1 = 0;
  storageData.hash = 0;
  EEPROM.put(storageDataAddress, storageData);
  setCalibrationToDefault();
  Serial.println("Done");
}

void printHelp() {
  Serial.print(helpTitle);
  Serial.print(helpMessage);
  openNotepad();
  Keyboard.print(helpTitle);
  delay(3000);
  Keyboard.print(helpMessage);
}

void openNotepad() {
  Keyboard.press(KEY_LEFT_GUI);
  Keyboard.write('r');
  Keyboard.release(KEY_LEFT_GUI);
  delay(1000);
  Keyboard.print("notepad");
  Keyboard.press(KEY_RETURN);
  Keyboard.release(KEY_RETURN);
  delay(2000);
}

void releaseButtons() {
  AbsMouse.release(MOUSE_RIGHT);
  AbsMouse.release(MOUSE_LEFT);
  Keyboard.releaseAll();
}

void resetCalibration() {
  EEPROM.get(storageDataAddress, storageData);
  if (storageData.checkHash()) {
    xCalibration2 = storageData.xCalibration2;
    yCalibration2 = storageData.yCalibration2;
    xCalibration1 = storageData.xCalibration1;
    yCalibration1 = storageData.yCalibration1;
  } else {
    setCalibrationToDefault();
  }
}

void setCalibrationToDefault() {
  xCalibration2 = xCalibration2Default;
  yCalibration2 = yCalibration2Default;
  xCalibration1 = xCalibration1Default;
  yCalibration1 = yCalibration1Default;
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

  //Serial.print(onScreenX);
  //Serial.print(",");
  //Serial.print(onScreenY);
  //Serial.print(",");

  Serial.print(mouseMoveX);
  Serial.print(",");
  Serial.print(mouseMoveY);
  Serial.print(",");

  Serial.print(state);
  Serial.print(",");

  Serial.print(triggerButtonState);
  Serial.print(",");
  Serial.print(reloadButtonState);
  Serial.print(",");

  Serial.print(xCalibration2);
  Serial.print(",");
  Serial.print(yCalibration2);
  Serial.print(",");
  Serial.print(xCalibration1);
  Serial.print(",");
  Serial.print(yCalibration1);
  Serial.println();
}
