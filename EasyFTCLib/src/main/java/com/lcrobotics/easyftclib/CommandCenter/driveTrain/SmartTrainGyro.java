package com.lcrobotics.easyftclib.CommandCenter.driveTrain;

import com.lcrobotics.easyftclib.tools.MathUtils;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gyroscope;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;

// equivalent of DriveTrain with SmartMotors
public class SmartTrainGyro {


    private WheelType wheelType;
    public SmartMotor[] motors;
    private final double radius;
    private ModernRoboticsI2cGyro external;
    private BNO055IMU internal;
    private double currentAngle;
    public SmartTrainGyro(WheelType wheelType, DcMotor leftMotor, DcMotor rightMotor, int ratio, double radius) {
        // reverse right motor
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        motors = new SmartMotor[]{new SmartMotor(leftMotor, WheelPosition.LEFT, ratio), new SmartMotor(rightMotor, WheelPosition.RIGHT, ratio)};
        this.radius = radius;
        this.wheelType = wheelType;
    }

    public SmartTrainGyro(WheelType wheelType, DcMotor frontLeftMotor, DcMotor frontRightMotor, DcMotor backLeftMotor, DcMotor backRightMotor, int ratio, double radius) {
        // reverse the right motors
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        motors = new SmartMotor[]{
                new SmartMotor(frontLeftMotor, WheelPosition.FRONT_LEFT, ratio),
                new SmartMotor(frontRightMotor, WheelPosition.FRONT_RIGHT, ratio),
                new SmartMotor(backLeftMotor, WheelPosition.BACK_LEFT, ratio),
                new SmartMotor(backRightMotor, WheelPosition.BACK_RIGHT, ratio)};

        this.radius = radius;
        this.wheelType = wheelType;

    }
    public SmartTrainGyro(WheelType wheelType, SmartMotor frontLeftMotor, SmartMotor frontRightMotor, SmartMotor backLeftMotor, SmartMotor backRightMotor, double radius) {
        motors = new SmartMotor[]{frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor};
        this.wheelType = wheelType;
        this.radius = radius;
    }

    public void setGyro(Gyroscope gyro) {
        if (gyro instanceof BNO055IMU) {
            internal = (BNO055IMU) gyro;
            external = null;
        } else if (gyro instanceof ModernRoboticsI2cGyro) {
            external = (ModernRoboticsI2cGyro) gyro;
            internal = null;
        } else {
            throw new IllegalArgumentException("Please provide either a BNO055IMU or a ModernRoboticsI2cGyro");
        }
    }
    public void update() {
        if (internal == null && external == null) {
            //throw new
        } else if (internal == null){

        } else {

        }
    }

    private void resetMotors() {
        // turn off run_to_position
        motors[0].motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motors[1].motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motors[2].motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motors[3].motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // set all powers to 0
        motors[0].motor.setPower(0);
        motors[1].motor.setPower(0);
        motors[2].motor.setPower(0);
        motors[3].motor.setPower(0);
    }


    public void execute(SmartCommand command) {
        // stop motors if queue is empty
        if (command == null) {
            drive(0);
            return;
        }
        // call different methods based on command type
        switch (command.type) {
            case DRIVE:
                drive(command.measure);
                break;
            case STRAFE:
                strafe(command.measure);
                break;
            case ROTATION:
                rotate(command.measure);
                break;
        }
    }

    private double getRotationAngle(double x, double y) {

        if (y > 0) {

            return x < 0? Math.toDegrees(Math.atan(x/y)) : Math.toDegrees(-Math.atan(x/y));

        } else {

            double absolute = 90 + Math.toDegrees(Math.atan(y/x));

            return x < 0? absolute : -(absolute);
        }
    }
    /*
     * rotate theta degrees relative to robot's current heading
     * counter-clockwise is positive, clockwise is negative, as per the unit circle
     */
    public void rotateDegrees(double theta) {
        // convert to radians
        double thetaRadians = Math.toRadians(theta);

        // calculate arc length
        double distance = radius * thetaRadians;

        rotate(distance);
    }
    /*
     * rotate so that robot is at angle theta relative to its starting position
     * at the beginning of the opmode
     */
    public void rotateAbsolute(double theta) {

    }
    private void rotate(double distance) {
        // invert distance for right motors
        motors[0].drive(distance, 0.5);
        motors[1].drive(-distance, 0.5);
        motors[2].drive(distance, 0.5);
        motors[3].drive(-distance, 0.5);
    }
    private void drive(double distance) {
        // drive each of the motors same distance
        for (int i = 0; i < motors.length; i++) {
            motors[i].drive(distance, 0.5);
        }
    }

    private void strafe(double distance) {
        // invert distance for front right and back left motors
        motors[0].drive(distance, 0.5);
        motors[1].drive(-distance, 0.5);
        motors[2].drive(-distance, 0.5);
        motors[3].drive(distance, 0.5);
    }

    /**
     * move robot xDist cm on the x axis and yDist cm on the y axis
     * @param mode move mode to determine how the robot gets to its destination
     *             DIAGONAL: computes the shortest distance to the destination and moves along that line
     *             X_FIRST: goes horizontal then vertical
     *             Y_FIRST: goes vertical then horizontal.
     *
     *
     */
    public void move(double xDist, double yDist, MoveMode mode) {

        switch (mode) {
            case DIAGONAL:
                // angle to rotate to get to shortest line
                double theta;
                if (xDist == 0 || yDist == 0) {
                    theta = 0;
                } else {
                    theta = 90 - Math.atan(yDist / xDist);
                }
                // rotate to orient with diagonal
                rotateDegrees(theta);
                // compute diagonal distance
                double distance = Math.hypot(xDist, yDist);
                // drive
                drive(distance);
                // rotate back to original orientation
                rotateDegrees(-theta);
                break;

            // strafe first then drive
            case X_FIRST:
                strafe(xDist);
                drive(yDist);
                break;

            // drive first then strafe
            case Y_FIRST:
                drive(yDist);
                strafe(xDist);
                break;

        }
    }
    // overload method with default movemode of DIAGONAL
    public void move(double xDist, double yDist) {
        move(xDist, yDist, MoveMode.DIAGONAL);
    }

    @Override
    public String toString() {
        StringBuilder doc = new StringBuilder();
        if (motors.length == 2) {
            doc.append("2 motors\n");
        } else {
            doc.append("4 motors\n");
        }
        for (SmartMotor motor : motors) {
            doc.append(motor.motorName + " Distance to target position: "
                    + Math.abs(motor.motor.getCurrentPosition() - motor.motor.getTargetPosition()) + "\n");
        }
        return doc.toString();
    }
}

