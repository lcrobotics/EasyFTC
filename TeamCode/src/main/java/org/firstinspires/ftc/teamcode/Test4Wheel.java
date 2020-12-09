package org.firstinspires.ftc.teamcode;

import com.lcrobotics.easyftclib.commandCenter.driveTrain.DriveTrain;
import com.lcrobotics.easyftclib.commandCenter.driveTrain.WheelType;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public abstract class Test4Wheel extends OpMode {

    public DcMotor FrontLeftDrive = null;
    public DcMotor FrontRightDrive = null;
    public DcMotor BackLeftDrive = null;
    public DcMotor BackRightDrive = null;

    public DriveTrain wheels = null;

    @Override
    public void init() {
        FrontLeftDrive = hardwareMap.get(DcMotor.class, "FrontLeftDrive");
        FrontRightDrive = hardwareMap.get(DcMotor.class, "FrontRightDrive");
        BackLeftDrive = hardwareMap.get(DcMotor.class, "BackLeftDrive");
        BackRightDrive = hardwareMap.get(DcMotor.class, "BackRightDrive");

        FrontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        FrontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        BackLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        BackRightDrive.setDirection(DcMotor.Direction.FORWARD);
        
        wheels = new DriveTrain(WheelType.MECANUM, FrontLeftDrive, FrontRightDrive, BackLeftDrive, BackRightDrive);
    }
}
