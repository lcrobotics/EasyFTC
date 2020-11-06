package com.lcrobotics.easyftclib.CommandCenter.driveTrain;

public class SmartCommand {
    double theta;
    double speed;
    double distance;

    public SmartCommand(double distance, double theta, double speed) {
        this.theta = theta;
        this.speed = speed;
        this.distance = distance;
    }
    public SmartCommand(double distance, double theta) {
        this(distance, theta, 0.7);
    }
    @Override
    public String toString() {
        return "SmartCommand{" +
                "theta=" + theta +
                ", speed=" + speed +
                ", distance=" + distance +
                '}';
    }
}
