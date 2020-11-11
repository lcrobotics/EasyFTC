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

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.

    // if we are within 1 degree of the desired heading, dont try and correct it
    static final double HEADING_THRESHOLD = 1;

    static final double P_TURN_COEFF = 0.1;
    static final double P_DRIVE_COEFF = 0.15;
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
            updateCommandData();
            // first command hasnt been done yet
            if (currCommand == null) {
                // if there's nothing else to do, stop the motors
                if (commandQueue.isEmpty()) {
                    setPower(0);
                    return;
                }
                // otherwise, execute the next command
                currCommand = commandQueue.remove();
                // check if it is just a rotation
                if (currCommand.distance == 0) {
                    // correct heading
                    if (onHeading()) {
                        // stop motors
                        setPower(0);
                        // get next command
                        currCommand = commandQueue.poll();
                    }
                    return;
                }
                // distance each encoder needs to move
                int moveCounts = (int) (currCommand.distance * motors[0].countPerCm);
                // set target positions
                for (SmartMotor motor : motors) {
                    motor.motor.setTargetPosition(motor.motor.getCurrentPosition() + moveCounts);
                    motor.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                }
            }
            // assume command is rotation
            if (currCommand.distance == 0) {
                // correct heading
                if (onHeading()) {
                    // stop motors
                    setPower(0);
                    // get next command
                    currCommand = commandQueue.poll();
                }
                return;
            }
            // motors have reached their destination
            if (!isBusy()) {
                // stop all movement
                setPower(0);
                for (SmartMotor motor : motors) {
                    motor.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
                // grab next command
                currCommand = commandQueue.poll();
                return;
            }
            // get error relative to desired heading
            double error = getError(currCommand.theta);
            double steer = getSteer(error, P_DRIVE_COEFF);

            // if we are going backwards, scale the error accordingly
            if (currCommand.distance < 0)
                steer *= -1;

            double leftSpeed = currCommand.power - steer;
            double rightSpeed = currCommand.power + steer;
            // normalize speeds if either speed goes above 1
            double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));

            if (max > 1.0) {
                leftSpeed /= max;
                rightSpeed /= max;
            }
            // set power of motors
            if (motors.length == 4) {
                setPower(new double[]{leftSpeed, rightSpeed, leftSpeed, rightSpeed});
            } else {
                setPower(new double[]{leftSpeed, rightSpeed});
            }

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
        int[] powers = new int[motors.length];
        for (int i = 0; i < motors.length; i++) {
            powers[i] = motors[i].motor.getCurrentPosition();
        }
        CommandData.currentPositions = powers;
    }

    /**
     * Perform one iteration of
     *
     * @return whether the robot is on target (the command is finished)
     */
    private boolean onHeading() {
        boolean onTarget = false;

        double leftPower;
        double rightPower;
        double steer;

        // determine turn power based on +/- error
        double error = getError(currCommand.theta);
        // ignore insignificant error
        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftPower = 0.0;
            rightPower = 0.0;
            onTarget = true;
        } else {
            // get steering power from error
            steer = getSteer(error, P_TURN_COEFF);
            // calculate powers for each side of robot
            rightPower = currCommand.power * steer;
            leftPower = -rightPower;
        }
        // set motor powers
        setPower(new double[]{leftPower, rightPower, leftPower, rightPower});

        return onTarget;
    }

    /**
     * returns the error between the target angle and the robot's current heading
     *
     * @param targetAngle Desired angle (relative to last Gyro reset).
     * @return error angle: Degrees in the range +/- 180.
     * +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;
        double angle;
        // calculate error in -179 to +180 range
        if (external == null) {
            angle = internal.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        } else {
            angle = external.getIntegratedZValue();
        }
        // make sure angle is in correct range
        robotError = targetAngle - angle;
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     *
     * @param error  Error angle in robot relative degrees
     * @param PCoeff Proportional Gain Coefficient
     * @return desired steering force
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
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

