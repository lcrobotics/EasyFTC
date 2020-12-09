package com.lcrobotics.easyftclib;

import com.lcrobotics.easyftclib.commandCenter.driveTrain.DriveMotor;
import com.lcrobotics.easyftclib.commandCenter.driveTrain.WheelPosition;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public abstract class AdvancedOpMode extends OpMode {

    // Wrapper around FTC's function to get a DcMotor and turn it into a DriveMotor
    public DriveMotor registerDriveMotor(String motorName, WheelPosition wheelPosition) {
        return new DriveMotor(hardwareMap.get(DcMotor.class, motorName), wheelPosition);
    }

    public DriveMotor registerDriveMotor(DcMotor motor, WheelPosition wheelPosition) {
        return new DriveMotor(motor, wheelPosition);
    }

}
