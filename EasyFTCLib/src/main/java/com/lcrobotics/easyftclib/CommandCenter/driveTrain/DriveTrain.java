package com.lcrobotics.easyftclib.CommandCenter.driveTrain;

import com.lcrobotics.easyftclib.tools.ArrayTools;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

public class DriveTrain {

    private WheelType wheelType;
    private DriveMotor[] motors;


    public DriveTrain(WheelType wheelType, DcMotor leftMotor, DcMotor rightMotor){
        this.wheelType = wheelType;
        this.motors = new DriveMotor[]{new DriveMotor(leftMotor, WheelPosition.LEFT), new DriveMotor(rightMotor, WheelPosition.RIGHT)};
    }

    // Creates a 4 wheel drive train
    public DriveTrain(WheelType wheelType, DcMotor frontLeftMotor, DcMotor frontRightMotor, DcMotor BackLeftMotor, DcMotor BackRightMotor){
        // Stores what type of wheel (i.e mechanum, omni, etc
        this.wheelType = wheelType;
        // stores the motors in our DriveMotor[], this allows us to know what they are attached to
        this.motors = new DriveMotor[] {
                new DriveMotor(frontLeftMotor, WheelPosition.FRONT_LEFT),
                new DriveMotor(frontRightMotor, WheelPosition.FRONT_RIGHT),
                new DriveMotor(BackLeftMotor, WheelPosition.BACK_LEFT),
                new DriveMotor(BackRightMotor, WheelPosition.BACK_RIGHT)
        };
    }

    public DriveTrain(WheelType wheelType, DriveMotor frontLeftMotor, DriveMotor frontRightMotor, DriveMotor backLeftMotor, DriveMotor backRightMotor){
        // Stores what type of wheel (i.e mechanum, omni, etc
        this.wheelType = wheelType;

        // stores the motors in our DriveMotor[], this allows us to know what they are attached to
        this.motors = new DriveMotor[] { frontLeftMotor,  frontRightMotor,
                                         backLeftMotor,   backRightMotor};
    }


    // Given an array of DcMotors and their motor positions reset drive train
    public void setMotors(DcMotor[] motors, WheelPosition[] motorPos) throws DriveTrainException {
        // checks position requirements
        checkValidPositions(motorPos);

        // validates number of motors
        if (motorPos.length == motors.length && (motors.length == 2 || motors.length == 4)) {

            // iterates through and creates new drive motors
            this.motors = new DriveMotor[motors.length];
            for (int i = 0; i < motors.length; i++) {
                this.motors[i] = new DriveMotor(motors[i], motorPos[i]);
            }

            // throws error for invalid motor count
        } else {
            throw new DriveTrainException("Invalid Motor count: only 2 or 4 motors per drive train is compatible, " + motors.length + " is detected");
        }
    }

    // Checks to see if there are any errors with wheel positions
    protected void checkValidPositions(WheelPosition[] positions) throws DriveTrainException {
        // make sure there are no 2 motors taking up the same space
        if (ArrayTools.containsDuplicates(positions)){
            throw new DriveTrainException("Invalid wheel positions: duplicate wheel positions detected");
        }

        // checks to ensure that its either 2wheel or 4wheel
        boolean two_wheel = positions[0] == WheelPosition.LEFT || positions[0] == WheelPosition.RIGHT;
        for (int i = 1; i < positions.length; i++) {
            if (!two_wheel && (positions[i] == WheelPosition.LEFT || positions[i] == WheelPosition.RIGHT)) {
                throw new DriveTrainException("Invalid wheel positions: both 2 wheel and 4 wheel drive detected");
            }
        }

    }


    public void setPower(double turn, double drive) {
        double rightDrivePow = Range.clip(drive - turn, -1, 1);

        double leftDrivePow = Range.clip(drive + turn, -1, 1);

        if (motors.length == 2) {
            motors[0].motor.setPower(leftDrivePow);
            motors[1].motor.setPower(rightDrivePow);
        } else {
            motors[0].motor.setPower(leftDrivePow);
            motors[1].motor.setPower(leftDrivePow);
            motors[2].motor.setPower(rightDrivePow);
            motors[3].motor.setPower(rightDrivePow);
        }
    }

    public void setPower(double x, double y, double w) {
        double frontLeftPow = Range.clip(y - x + w, -1, 1);
        double frontRightPow = Range.clip(y + x - w, -1, 1);
        double backLeftPow = Range.clip(y + x + w, -1, 1);
        double backRightPow = Range.clip(y - x - w, -1, 1);

        motors[0].motor.setPower(frontLeftPow);
        motors[1].motor.setPower(frontRightPow);
        motors[2].motor.setPower(backLeftPow);
        motors[3].motor.setPower(backRightPow);

    }

    @Override
    public String toString() {
        StringBuilder doc = new StringBuilder();
        if (motors.length == 2) {
            doc.append("2 motors\n");
        } else {
            doc.append("4 motors\n");
        }
        for (DriveMotor motor : motors) {

            doc.append(motor.motor.getDeviceName() + ": " + motor.motor.getPower() + "\n");
        }
        return doc.toString();
    }
}

