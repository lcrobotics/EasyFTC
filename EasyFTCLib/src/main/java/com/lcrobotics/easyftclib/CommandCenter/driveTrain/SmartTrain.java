package com.lcrobotics.easyftclib.CommandCenter.driveTrain;

import com.lcrobotics.easyftclib.tools.MathUtils;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;

// equivalent of DriveTrain with SmartMotors
public class SmartTrain {


    private WheelType wheelType;
    public Queue<SmartCommand> commandQueue;
    public SmartMotor[] motors;
    SmartCommand currCommand;
    private final double radius;
    public SmartTrain(WheelType wheelType, DcMotor leftMotor, DcMotor rightMotor, int ratio, double radius) {
        // reverse right motor
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        motors = new SmartMotor[]{new SmartMotor(leftMotor, WheelPosition.LEFT, ratio), new SmartMotor(rightMotor, WheelPosition.RIGHT, ratio)};
        this.radius = radius;
        this.wheelType = wheelType;
        commandQueue = new LinkedList<>();
    }

    public SmartTrain(WheelType wheelType, DcMotor frontLeftMotor, DcMotor frontRightMotor, DcMotor backLeftMotor, DcMotor backRightMotor, int ratio, double radius) {
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
        commandQueue = new LinkedList<>();
    }


    public SmartTrain(WheelType wheelType, SmartMotor frontLeftMotor, SmartMotor frontRightMotor, SmartMotor backLeftMotor, SmartMotor backRightMotor, double radius) {
        motors = new SmartMotor[]{frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor};
        this.wheelType = wheelType;
        this.radius = radius;
        commandQueue = new LinkedList<>();
    }

    public void update() {
        // if any of the motors are busy, we are not finished with the command yet
        if (currCommand != null) {
            for (SmartMotor s : this.motors) {
                if (s.isBusy()) {
                    return;
                }
            }
        }
        resetMotors();

        // get next command and execute it
        currCommand = commandQueue.poll();
        execute(currCommand);
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
    // parse path of points into drive commands needed to get there
    public void addPoints(List<Location> points) {
        // start at (0, 0, 0)
        Location current = new Location();
        // loop through all points
        for (Location l : points) {
            // change in x and y
            double xDiff = l.x - current.x;
            double yDiff = l.y - current.y;


            // points that only require one command
            if (l.angle != current.angle && l.y == current.y && l.x == current.x) {
                // only a rotation is needed
                commandQueue.add(new SmartCommand(l.angle - current.angle, CommandType.ROTATION));
                continue;
            }
            if (l.angle == current.angle && l.y != current.y && l.x == current.x) {
                // only drive forward
                commandQueue.add(new SmartCommand(yDiff, CommandType.DRIVE));
                continue;
            }
            if (l.angle == current.angle && l.y == current.y && l.x != current.x) {
                // only strafe
                commandQueue.add(new SmartCommand(xDiff, CommandType.STRAFE));
                continue;
            }
            // how we will get to the next point
            switch (l.mode) {

                // drive along hypotenuse
                case DIAGONAL:
                    // get shortest angle onto diagonal
                    double wrappedAngle = MathUtils.angleWrap(getRotationAngle(xDiff, yDiff) - current.angle);
                    // rotate onto diagonal
                    commandQueue.add(new SmartCommand(wrappedAngle, CommandType.ROTATION));
                    // drive on diagonal
                    commandQueue.add(new SmartCommand(Math.hypot(xDiff, yDiff), CommandType.DRIVE));

                    // if a rotate is required
                    if (l.angle != MathUtils.angleWrap(current.angle + wrappedAngle)) {
                        commandQueue.add(new SmartCommand(l.angle - current.angle, CommandType.ROTATION));
                    }
                    break;

                // drive horizontal leg first
                case X_FIRST:

                    commandQueue.add(new SmartCommand(xDiff, CommandType.STRAFE));
                    commandQueue.add(new SmartCommand(yDiff, CommandType.DRIVE));
                    // if a rotate is required after drive
                    if (l.angle != current.angle) {
                        commandQueue.add(new SmartCommand(l.angle - current.angle, CommandType.ROTATION));
                    }
                    break;

                // drive vertical leg first
                case Y_FIRST:

                    commandQueue.add(new SmartCommand(yDiff, CommandType.STRAFE));
                    commandQueue.add(new SmartCommand(xDiff, CommandType.DRIVE));

                    // if a rotate is required after drive
                    if (l.angle != current.angle) {
                        commandQueue.add(new SmartCommand(l.angle - current.angle, CommandType.ROTATION));
                    }

                    break;
            }
            // advance current location
            current = l;
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
        doc.append(currCommand);
        return doc.toString();
    }
}
