package com.lcrobotics.easyftclib.commandCenter.hardware;

import android.support.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Extension class to {@link Motor}, provides more functionality
 */
public class MotorEx extends Motor {

    public DcMotorEx motorEx;

    public MotorEx(@NonNull HardwareMap hw, String name) {
        super(hw, name);
        motorEx = hw.get(DcMotorEx.class, name);
        ACHIEVABLE_MAX_TICKS_PER_SECOND = motorEx.getMotorType().getAchieveableMaxTicksPerSecond();
    }

    public MotorEx(@NonNull HardwareMap hw, String name, double cpr, double rpm) {
        super(hw, name, cpr, rpm);
        motorEx = hw.get(DcMotorEx.class, name);
    }

    @Override
    public void set(double power) {
        switch (runMode) {
            case VelocityControl:
                double speed = power * ACHIEVABLE_MAX_TICKS_PER_SECOND;
                double velocity = velocityController.calculate(getVelocity(), speed) +
                        feedforward.calculate(speed);
                motorEx.setVelocity(velocity);
            case PositionControl:
                double error = positionController.calculate(encoder.getPosition());
                motorEx.setPower(power * error);
            case RawPower:
                motorEx.setPower(power);
        }
    }

    public void setVelocity(double velocity) {
        motorEx.setVelocity(velocity);
    }

    public void setVelocity(double velocity, AngleUnit angleUnit) {
        motorEx.setVelocity(velocity, angleUnit);
    }

    @Override
    public String getDeviceType() {
        return "Extended " + super.getDeviceType();
    }
}
