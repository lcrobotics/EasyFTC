package com.lcrobotics.easyftclib.commandCenter.old;

/**
 * MoveMode enum, create 3 new modes of travel: Diagonal, X_First -> drive x and then y, and
 * Y_First -> drive y first, then x
 * assign bytes to each mode
 */
public enum  MoveMode {
    // declare modes and set bytes
    DIAGONAL(0), X_FIRST(1), Y_FIRST(2);
    public final byte bval;

    MoveMode(int i) {
        bval = (byte) i;
    }
}
