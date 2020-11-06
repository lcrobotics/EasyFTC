package com.lcrobotics.easyftclib.CommandCenter.driveTrain;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import java.util.LinkedList;
import java.util.Queue;

// equivalent of DriveTrain with SmartMotors
public class SmartTrain {
    // wheel type on robot
    private final WheelType wheelType;
    // queue for drive commands
    public Queue <SmartCommand> commandQueue;
    // array to hold all SmartMotors
    public SmartMotor[] motors;
    // command currently on the top of the queue
    SmartCommand currCommand;
    // radius of wheels on the robot
    private final double radius;

    /**
     * constructor that takes in wheel type, drive motors, ratio (reduction), and wheel radius
     * this constructor is used for robots with two wheels
     */
    public SmartTrain(WheelType wheelType, DcMotor leftMotor, DcMotor rightMotor, int ratio, double radius) {
        // reverse right motor on our robot, the motor is mechanically reversed
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // populate SmartMotor array of motors
        // having the SmartMotor objects in a single array makes it easier to run motors and
        // so we can have a variable amount of motors
        motors = new SmartMotor[]{new SmartMotor(leftMotor, WheelPosition.LEFT, ratio), new SmartMotor(rightMotor, WheelPosition.RIGHT, ratio)};
        // allow user to set wheel radius and the wheel type
        this.radius = radius;
        this.wheelType = wheelType;
        // initialize commandQueue
        // commandQueue is used to get around First's loop detection and because it is much easier
        // for the end user to put commands in a queue than call multiple methods
        commandQueue = new LinkedList <>();
    }

    /**
     * constructor that takes in wheel type, drive motors, the ratio (reduction), and the wheel radius
     * this constructor is for robots with four wheels
     */
    public SmartTrain(WheelType wheelType, DcMotor frontLeftMotor, DcMotor frontRightMotor, DcMotor backLeftMotor, DcMotor backRightMotor, int ratio, double radius) {
        // reverse the right motors (this is because the motors on our robot are mechanically reversed)
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // populate SmartMotor array
        // having the SmartMotor objects in a single array makes it easier to run motors and
        // so we can have a variable amount of motors
        motors = new SmartMotor[]{
                new SmartMotor(frontLeftMotor, WheelPosition.FRONT_LEFT, ratio),
                new SmartMotor(frontRightMotor, WheelPosition.FRONT_RIGHT, ratio),
                new SmartMotor(backLeftMotor, WheelPosition.BACK_LEFT, ratio),
                new SmartMotor(backRightMotor, WheelPosition.BACK_RIGHT, ratio)};

        // allow user to set wheel radius and the wheel type
        this.radius = radius;
        this.wheelType = wheelType;
        // initialize commandQueue
        // commandQueue is used to get around First's loop detection and because it is much easier
        // for the end user to put commands in a queue than call multiple methods
        commandQueue = new LinkedList <>();
    }

    /**
     * constructor that takes in wheel type, drive motors, and wheel radius (it does not require the reduction)
     * this constructor is for robots with four wheels
     */
    public SmartTrain(WheelType wheelType, SmartMotor frontLeftMotor, SmartMotor frontRightMotor, SmartMotor backLeftMotor, SmartMotor backRightMotor, double radius) {
        // populate SmartMotor array
        // having the SmartMotor objects in a single array makes it easier to run motors and
        // so we can have a variable amount of motors
        motors = new SmartMotor[]{frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor};
        // allow user to set wheel radius and the wheel type
        this.wheelType = wheelType;
        this.radius = radius;
        // initialize commandQueue
        // commandQueue is used to get around First's loop detection and because it is much easier
        // for the end user to put commands in a queue than call multiple methods
        commandQueue = new LinkedList <>();
    }

    /**
     * check if current command is done, if it is, pop off queue and move onto next command, if not,
     * continue completing the current command on the queue
     */
    public void update() {
        // if any of the motors are busy, we are not finished with the command yet
        if (currCommand != null) {
            for (SmartMotor s : this.motors) {
                if (s.isBusy()) {
                    return;
                }
            }
        }
        // call method resetMotors to stop movement of motors (and the robot)
        resetMotors();

        // get next command on queue and execute it
        currCommand = commandQueue.poll();
    }

    /**
     * reset and stop the motors (and the robot)
     */
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

    /**
     * gets the angle of the diagonal created by x and y relative to the positive y axis
     */
    private double getRotationAngle(double x, double y) {
        // if y is positive, do the math to find the angle
        if (y > 0) {
            return x < 0 ? Math.toDegrees(Math.atan(x / y)) : Math.toDegrees(-Math.atan(x / y));
        } else {
            // if y is negative, do the math to find the angle
            double absolute = 90 + Math.toDegrees(Math.atan(y / x));
            return x < 0 ? absolute : -(absolute);
        }
    }

    /**
     * rotate theta degrees relative to robot's current heading
     * counter-clockwise is positive, clockwise is negative, as per the unit circle
     */
    public void rotateDegrees(double theta) {
        // convert to radians
        double thetaRadians = Math.toRadians(theta);
        // calculate arc length
        double distance = radius * thetaRadians;
        // rotate the length of the arc
        rotate(distance);
    }

    /**
     * rotate so that robot is at angle theta relative to its starting position
     * at the beginning of the opmode
     */
    public void rotateAbsolute(double theta) {
        // TODO
    }

    /**
     * rotate each motor for a certain distance
     */
    private void rotate(double distance) {
        // invert distance for right motors so it turns instead of going forward
        motors[0].drive(distance, 0.5);
        motors[1].drive(-distance, 0.5);
        motors[2].drive(distance, 0.5);
        motors[3].drive(-distance, 0.5);
    }

    /**
     * loop through array of motors and drive forward for a certain distance
     */
    private void drive(double distance) {
        // loop through motors, set power and distance to travel
        for (int i = 0; i < motors.length; i++) {
            motors[i].drive(distance, 0.5);
        }
    }

    /**
     * set motor powers and distance for strafing
     */
    private void strafe(double distance) {
        // invert distance for front right and back left motors
        motors[0].drive(distance, 0.5);
        motors[1].drive(-distance, 0.5);
        motors[2].drive(-distance, 0.5);
        motors[3].drive(distance, 0.5);
    }

    /**
     * move robot xDist cm on the x axis and yDist cm on the y axis
     *
     * @param mode move mode to determine how the robot gets to its destination
     *             DIAGONAL: computes the shortest distance to the destination and moves along that line
     *             X_FIRST: goes horizontal then vertical
     *             Y_FIRST: goes vertical then horizontal.
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
        doc.append(currCommand);
        return doc.toString();
    }
}
