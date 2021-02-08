package com.lcrobotics.easyftclib.commandCenter.driveTrain;

import com.lcrobotics.easyftclib.commandCenter.old.WheelPosition;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.ExpansionHubMotorControllerParamsState;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
// TODO: perhaps include methods contained in DcMotor for ease of access
public class DriveMotor {
    // FTC motor class
    public DcMotorEx motor;
    // Stores whether the motor is left or right (and maybe front or back)
    public WheelPosition position;
    public String motorName;

    boolean needsUpdate;
    double ticksPerCM;


    public DriveMotor(HardwareMap hw, String motorName, WheelPosition motorPosition, double radius) {
        // grab motor from hardware map
        motor = hw.get(DcMotorEx.class, motorName);

        position = motorPosition;
        // check if motor is configured
        MotorConfigurationType type = motor.getMotorType();
        if (type != MotorConfigurationType.getUnspecifiedMotorType()) {
            // calculate ticks per cm
            ticksPerCM = (type.getTicksPerRev() * type.getGearing()) / (2 * radius * Math.PI);
            // set position coefficient
            ExpansionHubMotorControllerParamsState params = type.getHubPositionParams();

            motor.setPositionPIDFCoefficients(params.p);
            // set velocity coefficients
            params = type.getHubVelocityParams();

            motor.setVelocityPIDFCoefficients(params.p, params.i, params.d, params.f);

            needsUpdate = false;
        } else {

            needsUpdate = true;
        }
    }
    // assumes wheel radius is 5
    public DriveMotor(HardwareMap hw, String motorName, WheelPosition motorPosition) {
        this(hw, motorName, motorPosition, 5);
    }

    public void setTicksPerCM(double ticksPerRev, double gearing, double radius) {
        ticksPerCM = (ticksPerRev * gearing) / (2 * radius * Math.PI);
    }
    /**
     * Constructor that includes a motor and a position
     * @param motor
     * @param motorPosition
     */
    public DriveMotor(DcMotor motor, WheelPosition motorPosition) {

        this.motor = (DcMotorEx) motor;
        this.position = motorPosition;
        this.motorName = motor.getDeviceName();
    }

    /**
     * Constructor that doesn't require a particular motor
     * Object serves as a placeholder until a motor can be connected with setMotor()
     * @param motorName
     * @param motorPosition
     */
    public DriveMotor(String motorName, WheelPosition motorPosition) {
        this.motorName = motorName;
        this.position = motorPosition;
    }

    public void driveDistance(double distance, DistanceUnit unit, double power) {
        driveDistance(unit.toCm(distance), power);
    }
    // assumes unit is cm
    public void driveDistance(double distance, double power) {
        // cant calculate counts without counts per centimeter
        if (needsUpdate) {
            throw new UnsupportedOperationException("Ticks per centimeter not set");
        }
        // calculate number of encoder counts required to rotate given distance
        double counts = distance * ticksPerCM;
        // reset motor's encoder
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // set target position for motor's encoder
        motor.setTargetPosition((int)Math.round(counts));
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // run motor
        motor.setPower(power);
    }

    // wrapper around dcmotor isbusy
    public boolean isBusy(int tolerance) {
        return Math.abs(this.motor.getTargetPosition() - this.motor.getCurrentPosition()) > 13;
    }

    public boolean isBusy() {
        return isBusy(13);
    }


    // Getters and setters

    public DcMotorEx getMotor() {
        return motor;
    }

    public void setMotor(DcMotorEx motor) {
        this.motor = motor;
    }

    public void setPosition(WheelPosition position) {
        this.position = position;
    }

    public WheelPosition getPosition() {
        return position;
    }
}