package com.lcrobotics.easyftclib.CommandCenter.driveTrain.commands;

import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

public class Strafe extends CommandImpl {
    double distance;
    double power;
    Position targetPos;
    public Strafe(double distance, double power) {
        this.distance = distance;
        this.power = power;
    }

    @Override
    public int init() {
        // make sure gyroscope is an IMU
        if (!(gyroscope instanceof BNO055IMU) && gyroscope != null) {
            return -2;
        }
        double cm = distanceUnit.toCm(distance);
        // only for our robot, abstract actual axes out later
        targetPos = new Position(DistanceUnit.CM, 0, cm, 0, 0);

        return 0;
    }

    @Override
    public int update() {
        Position curPos = CommandData.position;
        return 0;
    }
}
