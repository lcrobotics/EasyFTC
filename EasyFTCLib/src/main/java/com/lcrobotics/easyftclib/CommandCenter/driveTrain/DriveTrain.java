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


    // Given an array of DcMotors and their positions, reset drive train
    public void setMotors(DcMotor[] motors, WheelPosition[] motorPos) throws DriveTrainException {

        int numMotors = motors.length;

        if (numMotors != motorPos.length){
            throw new DriveTrainException("setMotors must be passed two arrays of equal length");
        }

        if (numMotors != 2 && numMotors != 4){
            throw new DriveTrainException("There must be either 2 or 4 motors in array");
        }

        // In case of 2 wheel drive, make sure all motors are 2 wheel
        if (numMotors == 2){
            for (int i = 0; i < 2; i++){
                if (!is2Wheel(motorPos[i])){
                    throw new DriveTrainException("4 wheel motor found in 2 wheel array");
                }
            }
        }

        // In case of 4 wheel drive, make sure all motors are 4 wheel
        if (numMotors == 4){
            for (int i = 0; i < 4; i++){
                if (is2Wheel(motorPos[i])){
                    throw new DriveTrainException("2 wheel motor found in 4 wheel array");
                }
            }
        }

        if (ArrayTools.containsDuplicates(motorPos)){
            throw new DriveTrainException("Invalid wheel positions: duplicate wheel positions detected");
        }

        // At this point, all criteria have been satisfied:
        //
        // Iterates through and creates new drive motors
        this.motors = new DriveMotor[numMotors];
        for (int i = 0; i < numMotors; i++){
            int newMotorIdx = posIndex(motorPos[i]);
            this.motors[newMotorIdx] = new DriveMotor(motors[i], motorPos[i]);
        }

    }
    public boolean is2Wheel(WheelPosition pos){
        return pos == WheelPosition.LEFT || pos == WheelPosition.RIGHT;
    }

    // Maps wheel positions to index of motors[]
    // LEFT, RIGHT: 0, 1
    // FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT: 0, 1, 2, 3
    public int posIndex(WheelPosition pos){
        switch (pos){
            case LEFT:
            case FRONT_LEFT:
                return 0;
            case RIGHT:
            case FRONT_RIGHT:
                return 1;
            case BACK_LEFT:
                return 2;
            default: // BACK_RIGHT
                return 3;
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

