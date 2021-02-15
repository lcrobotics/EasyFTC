package com.lcrobotics.easyftclib.CommandCenter.driveTrain.accelDrive;

import android.support.annotation.NonNull;

public class DriveState {
    public double x, y, w;

    public DriveState(double x, double y, double w) {
        this.x = x;
        this.y = y;
        this.w = w;
    }

    public DriveState(){
        this(0, 0, 0);
    }

    public void zeroOut() {
        x = y = w = 0;
    }
    @NonNull
    public String toString() {
        return "x: " + x + ", y: " + y + ", w: " + w;
    }

    public boolean equals(DriveState that) {
        return that.x == x && that.y == y && that.w == w;
    }
}
