package com.lcrobotics.easyftclib.commandCenter.driveTrain.accelDrive;

import android.support.annotation.NonNull;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.LinkedList;
import java.util.Queue;

public class AccelDriveBasic {
    // represents a command to execute, containing directions and a time
    protected static class DriveCommand {
        public DriveState state;
        public double b1, b2, time;

        public DriveCommand(double x, double y, double w, double t, double b1, double b2) {
            state = new DriveState(x, y, w);
            this.b1 = b1;
            this.b2 = b2;
            this.time = t;
        }
        // constructor with default values for b1 and b2
        // test to find optimal b1 and b2
        // send a bunch of commands back and forth and measure drift
        public DriveCommand(double x, double y, double w, double t) {
            this(x, y, w, t, 1/3.0, 1/3.0);
        }
        @NonNull
        public String toString() {
            return state.toString() + ", time: " + time;
        }
    }

    private Queue<DriveCommand> commandQueue;
    private DriveCommand currCommand;
    private ElapsedTime timer;

    public AccelDriveBasic() {
        commandQueue = new LinkedList<>();

        timer = new ElapsedTime();

        currCommand = null;
    }
    // push new command to accelDrive
    private void pushCommand(DriveCommand command) {
        commandQueue.add(command);
    }
    // push new command onto command queue
    public void pushCommand(double x, double y, double w, double t) {
        pushCommand(new DriveCommand(x, y, w, t));
    }
    // either updates the motor powers based on the s curve,
    // switches to the next command in the queue, or stops the motors
    public DriveState update() {

        // if the last command has been completed
        if (currCommand == null) {
            // if there's nothing else to do, stop the motors
            if (commandQueue.isEmpty())
                return new DriveState(0, 0, 0);
            // otherwise, get the new command
            currCommand = commandQueue.remove();
            timer.reset();
        }

        // this only executes when currCommand is non-null
        // incidentally, it also means that the timer was reset
        // at the beginning of the command's execution
        double elapsedTime = timer.seconds();
        // when time has run out on the current command
        if (elapsedTime > currCommand.time) {
            // mark it as completed and stop
            // the next loop will look for another command
            currCommand = null;
            return new DriveState(0, 0, 0);
        }

        // get unscaled sCurve value:
        // portion of maximum speed the robot should be going at the current time
        double unscaled = sCurve(elapsedTime);
        // scale this portion by max x, y, and w values to get the current speed
        return new DriveState(
                unscaled * currCommand.state.x,
                unscaled * currCommand.state.y,
                unscaled * currCommand.state.w);
    }

    // function to calculate unscaled s curve from time x
    private double sCurve(double x) {
        // scale x from 0-1 as a portion of currCommand.time
        x /= currCommand.time;

        // return 0 if x is out of bounds
        if (x < 0 || x >= 1)
            return 0;

        // we are accelerating
        if (x < currCommand.b1) {
            // scale x from 0-1 as a portion of currCommand.b1
            x /= currCommand.b1;
        } else if (x < 1 - currCommand.b1) {
            // we are at a constant velocity
            return 1;
        } else {
            // we are decelerating
            // reflect x over midline of s curve then scale in terms of currCommand.b1
            // turn it into acceleration, in terms of currCommand.b1
            x = (1 - x) / currCommand.b1;
        }
        // concave up
        if (x < currCommand.b2) {

            return sCurveParabola(x);

        } else if (x < 1 - currCommand.b2) {
            // if x is in the straight line
            // between the two parabolas
            // m is slope of straight line
            double m = 1/(1 - currCommand.b2);
            // equation of straight line from point-slope with (0.5, 0.5) as point
            return m*(x-0.5)+0.5;
        }
        // if concave down, reflect horizontally and vertically to transform
        // into concave up parabola
        return 1 - sCurveParabola(1 - x);
    }

    private double sCurveParabola(double x) {
        // z is maximum acceleration
        double z = 1/(1 - currCommand.b2);
        // equation for concave up parabola (integral of (z/b2)x, which is
        // the slope of the triangles making up the trapezoidal acceleration profile)
        return 0.5*(z/currCommand.b2)*(x*x);
    }

    public boolean isEmpty() {
        return commandQueue.isEmpty();
    }

    @NonNull
    public String toString() {

        String result = "Queue:";

        if (isEmpty()) {
            result += " Empty";
        } else {
            for (DriveCommand cmd : commandQueue) {
                result += "\n" + cmd;
            }
        }

        if (currCommand == null) {
            result += "\nNo current command";
        } else {
            result += "\nCurrent command: " + currCommand;
        }

        result += "\nElapsed time: " + timer.seconds();
        return result;
    }
}
