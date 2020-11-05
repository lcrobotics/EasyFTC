package com.lcrobotics.easyftclib.CommandCenter.driveTrain;

import com.google.ftcresearch.tfod.BuildConfig;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import java.util.LinkedList;
import java.util.Queue;

// equivalent of DriveTrain with SmartMotors
public class SmartTrainGyro {

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.7;     // Nominal speed for better accuracy.
    static final double     TURN_SPEED              = 0.5;     // Nominal half speed for better accuracy.

    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable

    private final WheelType wheelType;
    public SmartMotor[] motors;
    private ModernRoboticsI2cGyro external;
    private BNO055IMU internal;
    Queue<SmartCommand> commandQueue;
    SmartCommand currCommand;

    public SmartTrainGyro(WheelType wheelType, DcMotor leftMotor, DcMotor rightMotor, int ratio, double radius) {
        // reverse right motor
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        motors = new SmartMotor[]{new SmartMotor(leftMotor, WheelPosition.LEFT, ratio, radius), new SmartMotor(rightMotor, WheelPosition.RIGHT, ratio, radius)};
        this.wheelType = wheelType;
        commandQueue = new LinkedList<>();
    }

    public SmartTrainGyro(WheelType wheelType, DcMotor frontLeftMotor, DcMotor frontRightMotor, DcMotor backLeftMotor, DcMotor backRightMotor, int ratio, double radius) {
        // reverse the right motors
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motors = new SmartMotor[]{
                new SmartMotor(frontLeftMotor, WheelPosition.FRONT_LEFT, ratio, radius),
                new SmartMotor(frontRightMotor, WheelPosition.FRONT_RIGHT, ratio, radius),
                new SmartMotor(backLeftMotor, WheelPosition.BACK_LEFT, ratio, radius),
                new SmartMotor(backRightMotor, WheelPosition.BACK_RIGHT, ratio, radius)};

        this.wheelType = wheelType;
        commandQueue = new LinkedList<>();
    }
    public SmartTrainGyro(WheelType wheelType, SmartMotor frontLeftMotor, SmartMotor frontRightMotor, SmartMotor backLeftMotor, SmartMotor backRightMotor, double radius) {
        motors = new SmartMotor[]{frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor};
        this.wheelType = wheelType;
        commandQueue = new LinkedList<>();
    }
    // Sets the gyroscope to be used
    public void setGyro(Gyroscope gyro) {
        // we are working with an IMU
        if (gyro instanceof BNO055IMU) {
            internal = (BNO055IMU) gyro;
            external = null;
        } else if (gyro instanceof ModernRoboticsI2cGyro) {
            // this is an external gyroscope
            external = (ModernRoboticsI2cGyro) gyro;
            internal = null;
            external.resetZAxisIntegrator();
        } else {
            throw new IllegalArgumentException("Please provide either a BNO055IMU or a ModernRoboticsI2cGyro");
        }
    }

    // push new command to accelDrive
    private void pushCommand(SmartCommand command) {
        commandQueue.add(command);
    }
    // push new command onto command queue
    public void pushCommand(double angle, double speed, double distance) {
        pushCommand(new SmartCommand(angle, speed, distance));
    }

    // correct heading to be on target for drive command
    public void update() {

        // only allow either an internal or external gyroscope, not both
        if (internal == null ^ external == null) {
            // if the last command has been completed
            if (currCommand == null) {
                // if there's nothing else to do, stop the motors
                if (commandQueue.isEmpty()) {
                    setPower(0);
                    return;
                }
                // otherwise, execute the next command
                currCommand = commandQueue.remove();

                int moveCounts = (int)(currCommand.distance * motors[0].countPerCm);
                for (SmartMotor motor : motors) {
                    motor.motor.setTargetPosition(motor.motor.getCurrentPosition() + moveCounts);
                    motor.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                }
            }
            // motors have reached their destination
            if (!isBusy()) {
                setPower(0);
                for (SmartMotor motor : motors) {
                    motor.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
                return;
            }
            // get error relative to desired heading
            double error = getError(currCommand.theta);
            double steer = getSteer(error, P_DRIVE_COEFF);

            // if we are going backwards, scale the error accordingly
            if (currCommand.distance < 0)
                steer *= -1;
            double leftSpeed = currCommand.speed - steer;
            double rightSpeed = currCommand.speed + steer;

            double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
            if (max > 1.0)
            {
                leftSpeed /= max;
                rightSpeed /= max;
            }

            setPower(new double[]{leftSpeed, rightSpeed, leftSpeed, rightSpeed});
        }
    }
    public double getError(double targetAngle) {

        double robotError;
        double angle;
        // calculate error in -179 to +180 range
        if (external == null) {
            angle = internal.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES).firstAngle;
        } else {
            angle = external.getIntegratedZValue();
        }
        robotError = targetAngle - angle;
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
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

    private double getRotationAngle(double x, double y) {

        if (y > 0) {

            return x < 0? Math.toDegrees(Math.atan(x/y)) : Math.toDegrees(-Math.atan(x/y));

        } else {

            double absolute = 90 + Math.toDegrees(Math.atan(y/x));

            return x < 0? absolute : -(absolute);
        }
    }
    private void rotate(double distance) {
        // invert distance for right motors
        motors[0].drive(distance, 0.5);
        motors[1].drive(-distance, 0.5);
        motors[2].drive(distance, 0.5);
        motors[3].drive(-distance, 0.5);
    }

    public void setPower(double power) {
        for (SmartMotor motor : motors) {
            motor.setPower(power);
        }
    }
    public void setPower(double[] powers) {
        if (BuildConfig.DEBUG && motors.length != powers.length) {
            throw new AssertionError("Assertion failed");
        }
        for (int i = 0; i < motors.length; i++) {
            motors[i].setPower(powers[i]);
        }
    }
    public boolean isBusy() {
        for (SmartMotor motor : motors) {
            if (!motor.isBusy())
                return false;
        }
        return true;
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

