package com.lcrobotics.easyftclib.CommandCenter.driveTrain;

// class describing a point on the playing field
public class Location {

    // horizontal distance
    public double x;
    // vertical distance
    public double y;
    // angle robot is facing
    public double angle;
    public MoveMode mode;

    // set origin to robot's current position (recommended to run at the beginning of match)
    public Location() {
        this(0, 0, 0);
    }

    // set location variables to current values and MoveMode to default(Diagonal)
    public Location(double x, double y, double angle) {
        this.x = x;
        this.y = y;
        this.angle = angle;
        this.mode = MoveMode.DIAGONAL;
    }

    // set location variable to current variables and MoveMode to current mode
    public Location(double x, double y, double angle, MoveMode mode) {
        this.x = x;
        this.y = y;
        this.angle = angle;
        this.mode = mode;
    }
}
