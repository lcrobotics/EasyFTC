package com.lcrobotics.easyftclib.commandCenter.hardware;

import com.lcrobotics.easyftclib.tools.MathUtils;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class SimpleServo implements ServoEx {

    Servo servo;
    double minAngle;
    double maxAngle;

    public SimpleServo(HardwareMap hw, String name) {
        this(hw, name, 0, 180);
    }

    public SimpleServo(HardwareMap hw, String name, double min, double max) {
        servo = hw.get(Servo.class, name);
        minAngle = min;
        maxAngle = max;
    }

    @Override
    public void rotateDegrees(double angle) {
        turnToAngle(getAngle() + angle);
    }

    @Override
    public void turnToAngle(double angle) {
        angle = MathUtils.clamp(angle, minAngle, maxAngle);

        setPosition((angle - minAngle) / (maxAngle - minAngle));
    }

    @Override
    public void rotate(double position) {
        setPosition(getPosition() + position);
    }

    @Override
    public void setRange(double min, double max) {
        minAngle = min;
        maxAngle = max;
    }

    @Override
    public boolean getInverted() {
        return servo.getDirection() == Servo.Direction.REVERSE;
    }

    @Override
    public void setInverted(boolean inverted) {
        servo.setDirection(inverted ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
    }

    @Override
    public double getPosition() {
        return servo.getPosition();
    }

    @Override
    public void setPosition(double position) {
        servo.setPosition(MathUtils.clamp(position, 0, 1));
    }

    @Override
    public double getAngle() {
        return getPosition() * (maxAngle - minAngle) + minAngle;
    }

    @Override
    public void disable() {
        servo.close();
    }

    @Override
    public String getDeviceType() {
        return "SimpleServo: " + servo.getPortNumber() + "; " + servo.getController();
    }
}
