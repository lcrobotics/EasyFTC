package com.lcrobotics.easyftclib.CommandCenter.driveTrain;

public class SmartCommand {
    CommandType type;
    public double measure;

    public SmartCommand(double measure, CommandType type) {
        this.measure = measure;
        this.type = type;
    }
    public SmartCommand() {
        this.measure = 0;
        this.type = CommandType.DRIVE;
    }
    public String toString() {
        return "measure: " + measure +
                "\n" + (type == CommandType.STRAFE ? "Strafe" : type == CommandType.DRIVE ? "Drive" : "Rotate");
    }
}
