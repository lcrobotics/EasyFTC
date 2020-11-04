package com.lcrobotics.easyftclib.CommandCenter.driveTrain;

public class SmartCommand {
    double theta;
    double speed;
    double distance;

    public SmartCommand(double theta, double speed, double distance) {
        this.theta = theta;
        this.speed = speed;
        this.distance = distance;
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
