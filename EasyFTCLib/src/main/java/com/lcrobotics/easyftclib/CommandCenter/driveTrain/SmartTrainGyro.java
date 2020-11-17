package com.lcrobotics.easyftclib.CommandCenter.driveTrain;

import com.google.ftcresearch.tfod.BuildConfig;
import com.lcrobotics.easyftclib.CommandCenter.driveTrain.Commands.Command;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

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
    Queue<Command> commandQueue;
    Command currCommand;
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

    public void setGyro(BNO055IMU gyro) {
        internal = gyro;
        external = null;
    }
    public void setGyro(ModernRoboticsI2cGyro gyro) {
        internal = null;
        external = gyro;
    }

    // push new command to command queue
    public void pushCommand(Command command) {
        commandQueue.add(command);
    }

    /**
     * Calls the update method on the current command. If the current command is finished,
     * starts the next one and calls its init method
     */
    public void update() {
        // only allow either an internal or external gyroscope, not both
        if (internal == null ^ external == null) {
            // update command data
            updateCommandData();
            if (currCommand == null) {
                setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                // no commands left to do
                if (commandQueue.isEmpty()) {
                    return;
                }
                // initialize next command
                currCommand = commandQueue.poll();
                currCommand.init();
                // set target positions if needed
                if (CommandData.needsEncoders) {
                    for (int i = 0; i < motors.length; i++) {
                        motors[i].motor.setTargetPosition(CommandData.targetPositions[i]);
                    }
                    setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
            }
            // step through command
            switch (currCommand.update()) {
                // command is finished
                case 0:
                    currCommand = null;
                    // stop motors and encoders
                    setPower(0);
                    setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    return;
                case -1:
                    throw new RuntimeException("Invalid Command");
                case -2:
                    throw new RuntimeException("Invalid Gyroscope");
                case 1:
                    setPower(CommandData.motorPowers);
            }
            updateCommandData();
            // set power of motors
            setPower(CommandData.motorPowers);
        }
    }

    private void setMode(DcMotor.RunMode mode) {
        for (int i = 0; i < motors.length; i++) {
            motors[i].motor.setMode(mode);
        }
    }

    private void updateCommandData() {
        // read data from gyroscopes
        if (internal == null) {
            CommandData.angularOrientation = external.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            CommandData.zOrientation = external.getIntegratedZValue();
            CommandData.angularVelocity = external.getAngularVelocity(AngleUnit.DEGREES);
        } else if (external == null) {
            Orientation angle = internal.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            CommandData.angularOrientation = angle;
            CommandData.zOrientation = angle.firstAngle;

            CommandData.angularVelocity = internal.getAngularVelocity();
            CommandData.acceleration = internal.getLinearAcceleration();
            CommandData.velocity = internal.getVelocity();

        }
        // read motor data if needed
        int[] positions = new int[motors.length];
        for (int i = 0; i < motors.length; i++) {
            positions[i] = motors[i].motor.getCurrentPosition();

        }
        CommandData.currentPositions = positions;
        CommandData.motorCount = motors.length;
        CommandData.countsPerCM = motors[0].countPerCm;
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
        for (int i = 0; i < motors.length; i++) {
            motors[i].setPower(powers[i]);
        }
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

