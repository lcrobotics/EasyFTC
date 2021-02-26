package com.lcrobotics.easyftclib.CommandCenter.driveTrain.commands;

/**
 * This Command is designed to rotate a robot in place a certain angle,
 * using the gyroscope to correct the heading
 */
public class Rotate extends CommandImpl {
    // if we are within 1 degree of the desired heading, dont try and correct it
    static final double HEADING_THRESHOLD = 1;

    static final double P_TURN_COEFF = 0.1;

    public double angle;
    public double power;

    public Rotate(double angle, double power) {
        this.angle = angle;
        this.power = power;
    }

    /**
     *
     * @return 1 if command is still in progress, 0 if command is finished
     */
    @Override
    public int update() {
        // correct heading
        return onHeading()? 0 : 1;
    }

    /**
     * Perform one cycle of closed loop heading control
     *
     * @return whether the robot is on target (the command is finished)
     */
    private boolean onHeading() {

        double rightPower;

        // determine turn power based on +/- error
        double error = getError(angle, getZHeading());
        // ignore insignificant error
        if (Math.abs(error) <= HEADING_THRESHOLD) {
            rightPower = 0.0;
        } else {
            // calculate powers for each side of robot
            // and get steering power from error
            rightPower = power * getSteer(error, P_TURN_COEFF);
        }
        // set motor powers
        setMotorPowers(-rightPower, rightPower);
        // right power is 0 if we are on the correct heading
        return rightPower == 0.0;
    }
}
