package com.lcrobotics.easyftclib.CommandCenter.driveTrain;

public enum  MoveMode {
    DIAGONAL(0), X_FIRST(1), Y_FIRST(2);
    public final byte bval;

    MoveMode(int i) {
        bval = (byte) i;
    }
}
