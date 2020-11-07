package com.lcrobotics.easyftclib.CommandCenter.driveTrain;

public class SmartCommand {
    double theta;
    double power;
    double distance;

    public SmartCommand(double distance, double theta, double power) {
        this.theta = theta;
        this.power = power;
        this.distance = distance;
    }
    public SmartCommand(double distance, double theta) {
        this(distance, theta, 0.7);
    }
    @Override
    public String toString() {
        return "SmartCommand{" +
                "theta=" + theta +
                ", speed=" + power +
                ", distance=" + distance +
                '}';
    }
}
