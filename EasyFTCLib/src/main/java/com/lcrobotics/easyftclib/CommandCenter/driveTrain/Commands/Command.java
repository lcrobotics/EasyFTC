package com.lcrobotics.easyftclib.CommandCenter.driveTrain.Commands;

import android.support.annotation.NonNull;

import com.google.gson.internal.$Gson$Preconditions;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * encapsulates user-defined commands and basic ones we provide
 */
public interface Command {
    /**
     * start command
     * @return error code
     */
    int init();
    /**
     * move through next step of command
     * @return error code
     * 1 for command still going
     * 0 for finished command
     * -1 for invalid command
     * -2 for invalid gyro
     */
    int update();

    /**
     * requests that the motors be set to given powers
     * @param motorPowers array of powers that the motors should be set to
     */
    void setMotorPowers(double[] motorPowers);
    /**
     *
     */
    /**
     * initialize Command with given parameters
     * @param parameters an instance of the {@link Parameters} class
     * @return true if successful, false otherwise
     */
    boolean initialize(@NonNull Parameters parameters);

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
         * {@link com.lcrobotics.easyftclib.CommandCenter.driveTrain.SmartTrainGyro#setGyro(com.qualcomm.robotcore.hardware.IntegratingGyroscope)}
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
