package com.lcrobotics.easyftclib;

import com.lcrobotics.easyftclib.CommandCenter.driveTrain.DriveMotor;
import com.lcrobotics.easyftclib.CommandCenter.driveTrain.DriveTrain;
import com.lcrobotics.easyftclib.CommandCenter.driveTrain.WheelPosition;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Hardware;

import java.util.ArrayList;

public abstract class AdvancedOpMode extends OpMode {


    public DriveMotor registerDriveMotor(String motorName, WheelPosition wheelPosition) {
        return new DriveMotor(hardwareMap.get(DcMotor.class, motorName), wheelPosition);
    }

    public DriveMotor registerDriveMotor(DcMotor motor, WheelPosition wheelPosition) {
        return new DriveMotor(motor, wheelPosition);
    }

}
