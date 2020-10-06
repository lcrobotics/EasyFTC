package org.firstinspires.ftc.teamcode;

import com.lcrobotics.easyftclib.AdvancedOpMode;
import com.lcrobotics.easyftclib.CommandCenter.driveTrain.SmartMotor;
import com.lcrobotics.easyftclib.CommandCenter.driveTrain.WheelPosition;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
// class to test SmartMotors
public class SmartMotorTest extends AdvancedOpMode {
    // SmartMotor fields
    SmartMotor frontRightDrive;
    SmartMotor frontLeftDrive;
    SmartMotor backLeftDrive;
    SmartMotor backRightDrive;
    // boolean used to loop only once
    boolean once = true;

    @Override
    public void init() {
        // initialize all the SmartMotors (assumes that the reduction is 16 and wheelRadius is 5)
        frontLeftDrive = new SmartMotor(hardwareMap.get(DcMotor.class, "FrontLeftDrive"), WheelPosition.FRONT_LEFT, 16);
        frontRightDrive = new SmartMotor(hardwareMap.get(DcMotor.class, "FrontRightDrive"), WheelPosition.FRONT_RIGHT, 16);
        backLeftDrive = new SmartMotor(hardwareMap.get(DcMotor.class, "BackLeftDrive"), WheelPosition.BACK_LEFT, 16);
        backRightDrive = new SmartMotor(hardwareMap.get(DcMotor.class, "BackRightDrive"), WheelPosition.BACK_RIGHT, 16);
    }

    @Override
    public void loop() {
        // run only once
        if (once) {
            // robot moves forwards by 20cm (right motors are flipped)
            frontLeftDrive.drive(20, 0.5);
            frontRightDrive.drive(-20, 0.5);
            backLeftDrive.drive(20, 0.5);
            backRightDrive.drive(-20, 0.5);
            once = false;
        }
    }
}
