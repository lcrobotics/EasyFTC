package org.firstinspires.ftc.teamcode;

import com.lcrobotics.easyftclib.AdvancedOpMode;
import com.lcrobotics.easyftclib.commandCenter.driveTrain.DriveMotor;
import com.lcrobotics.easyftclib.commandCenter.old.DriveTrain;
import com.lcrobotics.easyftclib.commandCenter.old.WheelPosition;
import com.lcrobotics.easyftclib.commandCenter.old.WheelType;
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
        driveTrain.setPower(gamepad1.right_stick_x, -gamepad1.left_stick_y);
    }
}
