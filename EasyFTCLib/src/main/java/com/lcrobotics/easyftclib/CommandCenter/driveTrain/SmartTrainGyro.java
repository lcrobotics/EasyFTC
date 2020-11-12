package com.lcrobotics.easyftclib.CommandCenter.driveTrain;

import com.google.ftcresearch.tfod.BuildConfig;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.Queue;

// equivalent of DriveTrain with SmartMotors

/**
 * A lot of code in this class was adapted from the gyro drive example in ftcrobotcontroller
 * <p>
 * This class uses {@link SmartMotor} as a basis to drive a robot given distances and angles. Each {@link SmartCommand}
 * contains a distance to drive, an angle to drive that distance upon, and an optional speed to go at
 */
public class SmartTrainGyro {
    private final WheelType wheelType;
    public SmartMotor[] motors;
    Queue<SmartCommand> commandQueue;
    SmartCommand currCommand;
    private ModernRoboticsI2cGyro external;
    private BNO055IMU internal;

    /**
     * constructor for robots with 2 wheels
     *
     * @param wheelType  the type of wheel on this robot. Either mecanum, omni, or normal
     * @param leftMotor  DcMotor controlling the left wheel
     * @param rightMotor DcMotor controlling the right wheel
     * @param wheelRatio ratio between wheel revolutions and encoder counts
     * @param radius     radius of each wheel
     */
    public SmartTrainGyro(WheelType wheelType, DcMotor leftMotor, DcMotor rightMotor, int wheelRatio, double radius) {
        // reverse right motor
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        // reset encoders' zero position to current position
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // populate array of SmartMotors
        motors = new SmartMotor[]{new SmartMotor(leftMotor, WheelPosition.LEFT, wheelRatio, radius), new SmartMotor(rightMotor, WheelPosition.RIGHT, wheelRatio, radius)};

        this.wheelType = wheelType;
        commandQueue = new LinkedList<>();
    }

    /**
     * constructor for robots with 4 wheels
     *
     * @param wheelType       the type of wheel on this robot. Either mecanum, omni, or normal
     * @param frontLeftMotor  DcMotor controlling the front left wheel
     * @param frontRightMotor DcMotor controlling the front right wheel
     * @param backLeftMotor   DcMotor controlling the back left wheel
     * @param backRightMotor  DcMotor controlling the back right wheel
     * @param wheelRatio      ratio between wheel revolutions and encoder counts
     * @param radius          radius of each wheel
     */
    public SmartTrainGyro(WheelType wheelType, DcMotor frontLeftMotor, DcMotor frontRightMotor, DcMotor backLeftMotor, DcMotor backRightMotor, int wheelRatio, double radius) {
        // reverse the right motors
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // reset encoders' zero position to current position
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // populate array of SmartMotors
        motors = new SmartMotor[]{
                new SmartMotor(frontLeftMotor, WheelPosition.FRONT_LEFT, wheelRatio, radius),
                new SmartMotor(frontRightMotor, WheelPosition.FRONT_RIGHT, wheelRatio, radius),
                new SmartMotor(backLeftMotor, WheelPosition.BACK_LEFT, wheelRatio, radius),
                new SmartMotor(backRightMotor, WheelPosition.BACK_RIGHT, wheelRatio, radius)};

        this.wheelType = wheelType;
        commandQueue = new LinkedList<>();
    }

    public SmartTrainGyro(WheelType wheelType, SmartMotor frontLeftMotor, SmartMotor frontRightMotor, SmartMotor backLeftMotor, SmartMotor backRightMotor, double radius) {
        // populate array of SmartMotors
        motors = new SmartMotor[]{frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor};

        this.wheelType = wheelType;
        commandQueue = new LinkedList<>();
    }

    /**
     * sets the gyroscope that will be used for heading correction
     *
     * @param gyro the Gyroscope object (either an IMU or a MR Gyro)
     */
    public void setGyro(IntegratingGyroscope gyro) {
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

    // push new command to command queue
    private void pushCommand(SmartCommand command) {
        commandQueue.add(command);
    }

    // push new command onto command queue
    public void pushCommand(double distance, double angle, double speed) {
        pushCommand(new SmartCommand(distance, angle, speed));
    }

    public void pushCommand(double distance, double angle) {
        // 0.7 is the optimal drive speed
        pushCommand(distance, angle, 0.7);
    }

    /**
     * either performs one cycle of heading correction if the current command
     * if not finished yet or starts running the next command in the queue
     */
    public void update() {
        // only allow either an internal or external gyroscope, not both
        if (internal == null ^ external == null) {
            // set power of motors
            setPower(CommandData.motorPowers);
        }
    }

    private void updateCommandData() {
        if (internal == null) {
            CommandData.angularOrientation = external.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            CommandData.zOrientation = external.getIntegratedZValue();
            CommandData.angularVelocity = external.getAngularVelocity(AngleUnit.DEGREES);
        } else if (external == null) {
            Orientation angle = internal.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            CommandData.angularOrientation = angle;
            CommandData.acceleration = internal.getLinearAcceleration();
            CommandData.velocity = internal.getVelocity();
            CommandData.angularVelocity = internal.getAngularVelocity();
            CommandData.zOrientation = angle.firstAngle;
        }
        int[] positions = new int[motors.length];
        for (int i = 0; i < motors.length; i++) {
            positions[i] = motors[i].motor.getCurrentPosition();
        }
        CommandData.currentPositions = positions;
    }

    /**
     * sets all motors to specified power
     *
     * @param power desired power
     */
    public void setPower(double power) {
        for (SmartMotor motor : motors) {
            motor.setPower(power);
        }
    }

    /**
     * sets power of each motor to specified value in provided array
     *
     * @param powers array of motor powers parallel to motors array
     */
    public void setPower(double[] powers) {
        if (BuildConfig.DEBUG && motors.length != powers.length) {
            throw new AssertionError("Invalid length of power array");
        }
        for (int i = 0; i < motors.length; i++) {
            motors[i].setPower(powers[i]);
        }
    }

    /**
     * check whether every motor is still trying to reach an encoder position
     *
     * @return whether the motors are still trying to reach a target position
     */
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

