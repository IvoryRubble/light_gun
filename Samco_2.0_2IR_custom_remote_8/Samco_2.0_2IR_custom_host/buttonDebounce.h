class ButtonDebounce {
  public:
    ButtonDebounce() {
    }

    bool isBtnPressed = false;
    bool isBtnReleased = false;
    bool btnState = false;

    void updateState(bool btnStateInput) {
      btnState = !btnStateInput; // pull_up buttons
      unsigned long currentTime = millis();
      isBtnPressed = false;
      isBtnReleased = false;

      if (btnState != previousState && currentTime - previousStateChangeTime >= debounceDelay) {
        isBtnPressed = btnState;
        isBtnReleased = !btnState;

        previousStateChangeTime = currentTime;
      }
      previousState = btnState;
    }
  private:
    int debounceDelay = 10;
    uint32_t previousStateChangeTime = 0;
    bool previousState = false;
};
