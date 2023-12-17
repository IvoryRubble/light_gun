// LightGun
// wired version

// Based on
// https://www.dfrobot.com/product-1088.html
// https://github.com/samuelballantyne/IR-Light-Gun

// Reciever led very fast blink: camera is disconnected
// Reciever led fast blink: LightGun is disconnected
// Reciever led slow blink: LightGun is connected

// Side leds: red and blue for two detected points
// Side leds very fast alternate blink: camera is disconnected
// Side button: pause
// Side button long press: print help
// Side button long press while hold up button: calibrate
// Side button long press while hold left button: reset

#include <Keyboard.h>
#include <AbsMouse.h>
#include <EEPROM.h>
#include <DFRobotIRPosition.h>
#include "buttonDebounce.h"
#include "blinker.h"

#define bitset(byte, nbit, val) (val ? ((byte) |=  (1<<(nbit))) : ((byte) &= ~(1<<(nbit))))
#define bitcheck(byte, nbit) (!!((byte) & (1<<(nbit))))
#define mapf(val, in_min, in_max, out_min, out_max) (float)(val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
// #define LED_BLINK LED_BUILTIN

Blinker blinker;

enum State {
  mainState = 0,
  calibrationState1 = 1,
  calibrationState2 = 2,
  pauseState = 3,
  noConnectionState = 4
};
State state = noConnectionState;
State previousStateForInputEnabled = mainState;

// Log timeout
const unsigned long logMessageTimeout = 0;
unsigned long logMessagePreviousTime = 0;

// screen resolution
const int res_x = 1920;
const int res_y = 1080;
// int res_x = 1280;
// int res_y = 720;

// calibration offsets
const int xCalibrationOffset = 300;
const int yCalibrationOffset = 200;
const int xCalibrationPoint2 = 0 + xCalibrationOffset;
const int yCalibrationPoint2 = 0 + yCalibrationOffset;
const int xCalibrationPoint1 = res_x - xCalibrationOffset;
const int yCalibrationPoint1 = res_y - yCalibrationOffset;

// calibration defaults
const float xCalibration2Default = 0.3;
const float yCalibration2Default = -0.5;
const float xCalibration1Default = 0.8;
const float yCalibration1Default = 0.5;

// calibration points
float xCalibration2 = xCalibration2Default;
float yCalibration2 = yCalibration2Default;
float xCalibration1 = xCalibration1Default;
float yCalibration1 = yCalibration1Default;

// RAW Sensor Values
int rawX[4];
int rawY[4];

const int triggerButtonPin = A0;
const int upButtonPin = 5;
const int downButtonPin = 6;
const int leftButtonPin = 7;
const int rightButtonPin = 8;
const int reloadButtonPin = 9;

const int ledG = 12;
const int ledR = 14; 
const int ledB = 15; 

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
float finalX = 0;
float finalY = 0;

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

const int mouseGuardPin = 4;
bool isInputEnabledByMouseGuard = false;

// const unsigned long inputTimeout = 1000;
// unsigned long lastRemoteReadTime = 0;
bool isInputEnabledByTimeout = true;

bool buttonsReleased = false;
bool stateResetted = false;

// getPosition() method variables
const int nX = 512;
const int nY = 384;
int nDeltaX = 0;
int nDeltaY = 0;
int a = 0;
int b = 0;
int c = 0;
int d = 0;

struct StorageData {
  float xCalibration2;
  float yCalibration2;
  float xCalibration1;
  float yCalibration1;
  int hash;
  int calcHash() {
    return (int)(xCalibration2 * 10) + (int)(yCalibration2 * 10) + (int)(xCalibration1 * 10) + (int)(yCalibration1 * 10) + 12;
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

DFRobotIRPosition myDFRobotIRPosition;

void setup() {
  pinMode(mouseGuardPin, INPUT_PULLUP);
  // pinMode(LED_BLINK, OUTPUT);

  pinMode(triggerButtonPin, INPUT_PULLUP);
  pinMode(upButtonPin, INPUT_PULLUP);
  pinMode(downButtonPin, INPUT_PULLUP);
  pinMode(leftButtonPin, INPUT_PULLUP);
  pinMode(rightButtonPin, INPUT_PULLUP);
  pinMode(reloadButtonPin, INPUT_PULLUP);

  pinMode(ledR, OUTPUT);
  pinMode(ledG, OUTPUT);
  pinMode(ledB, OUTPUT);

  delay(1000);

  myDFRobotIRPosition.begin();            // Start IR Camera

  Serial.begin(115200);
  Serial1.begin(115200);

  AbsMouse.init(res_x, res_y);
  getInputEnabledByMouseGuard();
  getStorageData();

  delay(4000);

  if (isInputEnabledByMouseGuard) {
    AbsMouse.move((res_x / 2), (res_y / 2));
    delay(1000);
  }

  startupBlink();
  // lastRemoteReadTime = millis();
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
        if (triggerButton.isBtnPressed && !isOffScreen) {
          releaseButtons();
          xCalibration2 = finalX;
          yCalibration2 = finalY;
          state = calibrationState1;
        }
      }
      break;
    case calibrationState1: {
        mouseMove(xCalibrationPoint1, yCalibrationPoint1);
        if (reloadButton.isBtnReleased || isRegularButtonPressed()) {
          releaseButtons();
          resetCalibration();
          state = mainState;
        }
        if (triggerButton.isBtnPressed && !isOffScreen) {
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

  // setBlinkerPeriod();
  // blinker.update();
  // digitalWrite(LED_BLINK, blinker.state);

  PrintResults();
  // delay(100);
}

void startupBlink() {
  for (int i = 0; i < 5; i++) {
    unsigned long delayTime = 100;
    digitalWrite(ledR, HIGH);
    digitalWrite(ledB, LOW);
    delay(delayTime);
    digitalWrite(ledR, LOW);
    digitalWrite(ledB, LOW);
    delay(delayTime);
    digitalWrite(ledR, LOW);
    digitalWrite(ledB, HIGH);
    delay(delayTime);
    digitalWrite(ledR, LOW);
    digitalWrite(ledB, LOW);
    delay(delayTime);
  }
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

void getCamData() {
  myDFRobotIRPosition.requestPosition();

  if (myDFRobotIRPosition.available()) {
    isCamAvailable = true;
    for (int i = 0; i < 4; i++) {
      rawX[i] = myDFRobotIRPosition.readX(i);
      rawY[i] = myDFRobotIRPosition.readY(i);
    }

    digitalWrite(ledR, (rawX[0] != 1023));
    digitalWrite(ledB, (rawX[1] != 1023));
  } else {
    // Serial.println("Device not available!");
    isCamAvailable = false;
    
    blinker.setPeriod(100, 100);
    blinker.update();
    digitalWrite(ledR, blinker.state);
    digitalWrite(ledB, !blinker.state);
  }
}

void getButtons() {
  reloadButtonState = digitalRead(reloadButtonPin);
  triggerButtonState = digitalRead(triggerButtonPin);
  upButtonState = digitalRead(upButtonPin);
  downButtonState = digitalRead(downButtonPin);
  leftButtonState = digitalRead(leftButtonPin);
  rightButtonState = digitalRead(rightButtonPin);
}

void getRemoteData() {
  getCamData();
  getButtons();
}

void getInputEnabledByMouseGuard() {
  // isInputEnabledByMouseGuard = !digitalRead(mouseGuardPin);
  isInputEnabledByMouseGuard = true;
  // isInputEnabledByMouseGuard = false;
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
  if (isInputEnabledByMouseGuard && isInputEnabledByTimeout && !reloadButton.btnState && isCamAvailable) {
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

      nDeltaX = nX - bottomX;
      nDeltaY = nY - bottomY;

      a = topX - bottomX;
      b = topY - bottomY;
      c = -topY + bottomY;
      d = topX - bottomX;

      finalX = ((float)nDeltaX * d - (float)nDeltaY * c) / ((float)a * d - (float)c * b);
      finalY = ((float)nDeltaX * b - (float)nDeltaY * a) / ((float)c * b - (float)a * d);

      // finalX = 512 + cos(atan2(bottomY - topY, bottomX - topX) * -1) * (((topX - bottomX) / 2 + bottomX) - 512) - sin(atan2(bottomY - topY, bottomX - topX) * -1) * (((topY - bottomY) / 2 + bottomY) - 384);
      // finalY = 384 + sin(atan2(bottomY - topY, bottomX - topX) * -1) * (((topX - bottomX) / 2 + bottomX) - 512) + cos(atan2(bottomY - topY, bottomX - topX) * -1) * (((topY - bottomY) / 2 + bottomY) - 384);

      // swap X and Y
      onScreenX_unconstrained = round(mapf(finalY, yCalibration2, yCalibration1, xCalibrationPoint2, xCalibrationPoint1));
      onScreenY_unconstrained = round(mapf(finalX, xCalibration2, xCalibration1, yCalibrationPoint2, yCalibrationPoint1));
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
  return reloadButton.isBtnReleasedLongPress && !upButton.btnState && !downButton.btnState && !leftButton.btnState && !rightButton.btnState && !triggerButton.btnState;
}

void resetAll() {
  if (isSerialPortOpened()) {
    Serial.println("Reset all...");
  }
  storageData.xCalibration2 = 0;
  storageData.yCalibration2 = 0;
  storageData.xCalibration1 = 0;
  storageData.yCalibration1 = 0;
  storageData.hash = 0;
  EEPROM.put(storageDataAddress, storageData);
  setCalibrationToDefault();
  if (isSerialPortOpened()) {
    Serial.println("Done");
  }
  delay(2000);
}

void printHelp() {
  if (isSerialPortOpened()) {
    Serial.print(F("Please stand by...\n\n"));
    Serial.print(F("LightGun\n\n"));
    Serial.print(F("Based on:\n"));
    Serial.print(F("https://www.dfrobot.com/product-1088.html\n"));
    Serial.print(F("https://github.com/samuelballantyne/IR-Light-Gun\n\n"));
    Serial.print(F("Reciever led very fast blink: camera is disconnected\n"));
    Serial.print(F("Reciever led fast blink: LightGun is disconnected\n"));
    Serial.print(F("Reciever led slow blink: LightGun is connected\n"));
    Serial.print(F("Side leds very fast alternate blink: camera is disconnected\n"));
    Serial.print(F("Side leds: red and blue for two detected points\n"));
    Serial.print(F("Side button: pause\n"));
    Serial.print(F("Side button long press: print help\n"));
    Serial.print(F("Side button long press while hold up button: calibrate\n"));
    Serial.print(F("Side button long press while hold left button: reset\n\n\n\n"));
    printLogSerial();
  }

  openNotepad();
  Keyboard.print(F("Please stand by...\n\n"));
  delay(3000);
  Keyboard.print(F("LightGun\n\n"));
  Keyboard.print(F("Based on:\n"));
  Keyboard.print(F("https://www.dfrobot.com/product-1088.html\n"));
  Keyboard.print(F("https://github.com/samuelballantyne/IR-Light-Gun\n\n"));
  Keyboard.print(F("Reciever led very fast blink: camera is disconnected\n"));
  Keyboard.print(F("Reciever led fast blink: LightGun is disconnected\n"));
  Keyboard.print(F("Reciever led slow blink: LightGun is connected\n"));
  Keyboard.print(F("Side leds very fast alternate blink: camera is disconnected\n"));
  Keyboard.print(F("Side leds: red and blue for two detected points\n"));
  Keyboard.print(F("Side button: pause\n"));
  Keyboard.print(F("Side button long press: print help\n"));
  Keyboard.print(F("Side button long press while hold up button: calibrate\n"));
  Keyboard.print(F("Side button long press while hold left button: reset\n\n\n\n"));
  printLogKeyboard();
  //delay(1000);
  //Keyboard.print(F("Gthtrk.xbnt zpsr rkfdbfnehs yf fyukbqcrbq\n\n")); // Переключите язык клавиатуры на английский
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

void printLogSerial() {
  Serial.println("Telemetry:");
  Serial.print("IsCamAvailable: ");
  Serial.println(isCamAvailable);
  Serial.print("rawPoint1: (");
  Serial.print(rawX[0]);
  Serial.print(", ");
  Serial.print(rawY[0]);
  Serial.println(")");
  Serial.print("rawPoint2: (");
  Serial.print(rawX[1]);
  Serial.print(", ");
  Serial.print(rawY[1]);
  Serial.print(")\n\n\n");
}

void printLogKeyboard() {
  Keyboard.println("Telemetry:");
  Keyboard.print("IsCamAvailable: ");
  Keyboard.println(isCamAvailable ? "true" : "false");
  Keyboard.print("rawPoint1: (");
  Keyboard.print(rawX[0]);
  Keyboard.print(", ");
  Keyboard.print(rawY[0]);
  Keyboard.println(")");
  Keyboard.print("rawPoint2: (");
  Keyboard.print(rawX[1]);
  Keyboard.print(", ");
  Keyboard.print(rawY[1]);
  Keyboard.print(")\n\n\n");
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

bool isSerialPortOpened() {
  return Serial.availableForWrite() >= 32;
}

void PrintResults() {
  if (!isSerialPortOpened()) {
    return;
  }

  unsigned long currentTime = millis();
  if (currentTime - logMessagePreviousTime > logMessageTimeout) {
    //PrintResultsReadable();
    PrintResultsForProcessing();
    //Serial.flush();
    logMessagePreviousTime = currentTime;
  }
}

void PrintResultsReadable() {    // Print results for debugging
  Serial.print("RAW: ");
  Serial.print(finalX);
  Serial.print(", ");
  Serial.print(finalY);

  Serial.print("\tnD: ");
  Serial.print(nDeltaX);
  Serial.print(", ");
  Serial.print(nDeltaY);
  //
  //  Serial.print("\tabcd: ");
  //  Serial.print(a);
  //  Serial.print(", ");
  //  Serial.print(b);
  //  Serial.print(", ");
  //  Serial.print(c);
  //  Serial.print(", ");
  //  Serial.print(d);

  Serial.print("\tCalibration: ");
  Serial.print(xCalibration2);
  Serial.print(", ");
  Serial.print(yCalibration2);
  Serial.print(", ");
  Serial.print(xCalibration1);
  Serial.print(", ");
  Serial.print(yCalibration1);

  Serial.print("\tPosition: ");
  Serial.print(onScreenX);
  Serial.print(", ");
  Serial.print(onScreenY);

  //  Serial.print("State: ");
  //  Serial.print(state);
  //
  //  Serial.print("\tButton states: ");
  //  Serial.print(triggerButton.btnState);
  //  Serial.print(", ");
  //  Serial.print(reloadButton.btnState);
  //
  //  Serial.print("\tButton pressed event: ");
  //  Serial.print(triggerButton.isBtnPressed);
  //  Serial.print(", ");
  //  Serial.print(reloadButton.isBtnPressed);
  //
  //  Serial.print("\tisInputEnabledByTimeout: ");
  //  Serial.print(isInputEnabledByTimeout);
  //
  //  Serial.print("\tisOffScreen: ");
  //  Serial.print(isOffScreen);

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
