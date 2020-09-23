package org.firstinspires.ftc.teamcode;

import com.lcrobotics.easyftclib.AdvancedOpMode;
import com.lcrobotics.easyftclib.CommandCenter.driveTrain.DriveMotor;
import com.lcrobotics.easyftclib.CommandCenter.driveTrain.DriveTrain;
import com.lcrobotics.easyftclib.CommandCenter.driveTrain.WheelPosition;
import com.lcrobotics.easyftclib.CommandCenter.driveTrain.WheelType;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class motorControllerTest extends AdvancedOpMode {
    DriveMotor frontRightDrive;
    DriveMotor frontLeftDrive;
    DriveMotor backLeftDrive;
    DriveMotor backRightDrive;

    DriveTrain driveTrain;

    @Override
    public void init() {
        frontLeftDrive = registerDriveMotor("FrontLeftDrive", WheelPosition.FRONT_LEFT);
        frontRightDrive = registerDriveMotor( "FrontRightDrive", WheelPosition.FRONT_RIGHT);
        backLeftDrive = registerDriveMotor( "BackLeftDrive", WheelPosition.BACK_LEFT);
        backRightDrive = registerDriveMotor( "BackRightDrive", WheelPosition.BACK_RIGHT);
        driveTrain = new DriveTrain(WheelType.NORMAL, frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive);

    }

    @Override
    public void loop() {
        float x = gamepad1.left_stick_x;
        float y = gamepad1.left_stick_y;



        driveTrain.setPower(x, y);
    }
}
