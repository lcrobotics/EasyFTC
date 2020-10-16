package org.firstinspires.ftc.teamcode;


import com.lcrobotics.easyftclib.CommandCenter.driveTrain.DriveTrain;
import com.lcrobotics.easyftclib.CommandCenter.driveTrain.WheelType;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public abstract class Test2Wheel extends OpMode {

    public DcMotor LeftDrive = null;
    public DcMotor RightDrive = null;

    public DriveTrain wheels;

    @Override
    public void init() {
        LeftDrive = hardwareMap.get(DcMotor.class, "LeftDrive");
        RightDrive = hardwareMap.get(DcMotor.class, "RightDrive");
        
        LeftDrive.setDirection(DcMotor.Direction.FORWARD);
        RightDrive.setDirection(DcMotor.Direction.REVERSE);
        
        wheels = new DriveTrain(WheelType.MECANUM, LeftDrive, RightDrive);
    }
}
