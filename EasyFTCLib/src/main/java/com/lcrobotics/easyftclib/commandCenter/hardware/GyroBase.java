package com.lcrobotics.easyftclib.commandCenter.hardware;

import com.lcrobotics.easyftclib.tools.geometry.Rotation2d;

public abstract class GyroBase implements HardwareDevice {
    public abstract void init();

    public abstract double getHeading();
    public abstract double getAbsoluteHeading();
    public abstract double[] getAngles();
    public abstract Rotation2d getRotation2d();
    public abstract void reset();
}
