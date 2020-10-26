package com.lcrobotics.easyftclib.CommandCenter.driveTrain;

// class describing a point on the playing field
public class Location {

    public double x;
    public double y;
    public double angle;
    public MoveMode mode;

    public Location() {
        this(0, 0, 0);
    }

    public Location(double x, double y, double angle) {
        this.x = x;
        this.y = y;
        this.angle = angle;
        this.mode = MoveMode.DIAGONAL;
    }

    public Location(double x, double y, double angle, MoveMode mode) {
        this.x = x;
        this.y = y;
        this.angle = angle;
        this.mode = mode;
    }
}
