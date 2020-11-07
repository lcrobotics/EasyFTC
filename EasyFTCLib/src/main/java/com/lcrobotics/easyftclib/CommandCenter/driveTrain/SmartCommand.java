package com.lcrobotics.easyftclib.CommandCenter.driveTrain;

/**
 * Class that is used for creating SmartCommands, allows for users to create own commands. These
 * objects can be pushed onto a queue (CommandQueue)
 */
public class SmartCommand {
    // angle of rotation
    double theta;
    // motor power
    double power;
    // distance traveled
    double distance;

    /**
     * Constructor that takes in user distance, power and theta
     * @param distance
     * @param theta
     * @param power
     */
    public SmartCommand(double distance, double theta, double power) {
        this.theta = theta;
        this.power = power;
        this.distance = distance;
    }

    /**
     * Constructor that takes in user distance and theta, but sets default power to 0.7
     * @param distance
     * @param theta
     */
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
