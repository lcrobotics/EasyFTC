package com.lcrobotics.easyftclib.commandCenter.driveTrain.commands;

import android.support.annotation.NonNull;

import com.qualcomm.robotcore.hardware.IntegratingGyroscope;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Interface that all user-defined Commands should implement
 */
public interface Command {
    /**
     * This method is called when a Command is started
     * @return error code
     */
    int init();
    /**
     * Move through next step of command.
     * Should be called continuously in a loop
     * @return error code
     * 1 for command still going
     * 0 for finished command
     * -1 for invalid command
     * -2 for invalid gyro
     */
    int update();

    /**
     * Requests that the motors be set to given powers
     * @param motorPowers array of powers that the motors should be set to
     */
    void setMotorPowers(double[] motorPowers);
    /**
     * Initialize Command with given parameters
     * @param parameters an instance of the {@link Parameters} class
     * @return true if successful, false otherwise
     */
    boolean initialize(@NonNull Parameters parameters);

    /**
     * Initialize Command with default parameters
     * @return true if successful, false otherwise
     */
    boolean initialize();
    /**
     * Instances of Parameters contain data defining how a Command should operate and what data to use
     * @see #initialize(Parameters)
     */
    class Parameters implements Cloneable {
        /** units in which distance is measured. {@link DistanceUnit} */
        public DistanceUnit distanceUnit = DistanceUnit.CM;
        /** units in which angle is measured. {@link AngleUnit} */
        public AngleUnit angleUnit = AngleUnit.DEGREES;
        /** gyroscope to use to execute this command.
         * Defaults to gyroscope set in
         * {@link com.lcrobotics.easyftclib.commandCenter.driveTrain.SmartTrainGyro#setGyro)}
         */
        public IntegratingGyroscope gyroscope = null;

        public Parameters clone() {
            try {
                return (Parameters) super.clone();
            } catch (CloneNotSupportedException e) {
                throw new RuntimeException("internal error: Parameters can't be cloned");
            }
        }
    }
}
