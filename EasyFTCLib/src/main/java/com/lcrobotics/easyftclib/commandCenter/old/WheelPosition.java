package com.lcrobotics.easyftclib.commandCenter.old;

/**
 * First 2 values are used for motors in a 2 wheel configuration,
 * last 4 are used for a 4 wheeled robot
 */
public enum  WheelPosition {
    LEFT(0),
    RIGHT(1),
    SLIDE(2),
    FRONT_LEFT(0),
    FRONT_RIGHT(1),
    BACK_LEFT(2),
    BACK_RIGHT(3);

    public final int value;

    WheelPosition(int value) {
        this.value = value;
    }
}

