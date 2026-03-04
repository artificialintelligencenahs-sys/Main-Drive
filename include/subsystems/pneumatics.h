#pragma once

class PneumaticControl {
private:
    bool descoreState;
    bool unloaderState;
    bool R2_lastState;
    bool L2_lastState;
    bool Y_lastState;
    bool midDescoreState;

public:
    PneumaticControl();
    void update();
    bool getDescoreState();
    bool getUnloaderState();
};