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
}
