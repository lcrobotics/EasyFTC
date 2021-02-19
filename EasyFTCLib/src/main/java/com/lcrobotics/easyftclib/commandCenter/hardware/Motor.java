package com.lcrobotics.easyftclib.commandCenter.hardware;

import android.support.annotation.NonNull;

import com.lcrobotics.easyftclib.commandCenter.controllers.PController;
import com.lcrobotics.easyftclib.commandCenter.controllers.PIDController;
import com.lcrobotics.easyftclib.commandCenter.controllers.SimpleMotorFeedforward;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.function.Supplier;

/**
 * Wrapper around {@link DcMotor}
 */
public class Motor implements HardwareDevice {

    public enum Direction {
        FORWARD(1), REVERSE(-1);

        private final int val;

        Direction(int value) {
            val = value;
        }

        public int getMultiplier() {
            return val;
        }
    }

    public enum RunMode {
        VelocityControl, PositionControl, RawPower
    }

    public enum ZeroPowerBehavior {
        UNKNOWN(DcMotor.ZeroPowerBehavior.UNKNOWN),
        BRAKE(DcMotor.ZeroPowerBehavior.BRAKE),
        FLOAT(DcMotor.ZeroPowerBehavior.FLOAT);

        private final DcMotor.ZeroPowerBehavior behavior;

        ZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
            this.behavior = behavior;
        }

        public DcMotor.ZeroPowerBehavior getBehavior() {
            return behavior;
        }
    }

    public class Encoder {

        private final static int CPS_STEP = 0x1000;
        private final Supplier<Integer> position;
        private int resetVal, lastPosition;
        private Direction direction;
        private double lastTimeStamp, velo, dpp;

        /**
         * Construct an encoder, using the given supplier as a way to get position
         *
         * @param pos Supplier of encoder position in ticks
         */
        public Encoder(Supplier<Integer> pos) {
            position = pos;
            dpp = 1;
            resetVal = 0;
            lastPosition = 0;
            velo = 0;
            direction = Direction.FORWARD;
            lastTimeStamp = System.nanoTime() / 1E9;
        }

        /**
         * @return the current position of the encoder
         */
        public int getPosition() {
            int currentPos = position.get();
            // encoder has moved
            if (currentPos != lastPosition) {
                double currentTime = System.nanoTime() / 1E9;
                velo = (currentPos - lastPosition) / (currentTime - lastTimeStamp);
                lastPosition = position.get();
                lastTimeStamp = currentTime;
            }
            return direction.getMultiplier() * (currentPos - resetVal);
        }

        public double getDistance() {
            return dpp * getPosition();
        }

        public void reset() {
            resetVal = getPosition();
        }

        /**
         * Sets the distance per pulse of the encoder
         *
         * @param distancePerPulse desired distance per pulse (in units per tick)
         */
        public Encoder setDistancePerPulse(double distancePerPulse) {
            dpp = distancePerPulse;
            return this;
        }

        /**
         * Sets direction of encoder
         *
         * @param d desired direction
         */
        public void setDirection(Direction d) {
            direction = d;
        }

        /**
         * @return revolutions turned by the encoder
         */
        public double getRevolutions() {
            return getPosition() / getCPR();
        }

        public double getRawVelocity() {
            return getVelocity();
        }

        /**
         * Corrects for velocity overflow
         *
         * @return the corrected velocity
         */
        public double getCorrectedVelocity() {
            double real = getRawVelocity();

            while (Math.abs(velo - real) > CPS_STEP / 2.0) {
                real += Math.signum(velo - real) * CPS_STEP;
            }
            return real;
        }
    }

    public DcMotor motor;
    public Encoder encoder;
    // max velocity in ticks per second
    public double ACHIEVABLE_MAX_TICKS_PER_SECOND;
    protected RunMode runMode;
    protected PIDController velocityController = new PIDController(1, 0, 0);
    protected PController positionController = new PController(1);
    protected SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 1, 0);
    private boolean targetIsSet = false;

    public double multiplier = 1;
    public Motor(){}

    /**
     * Constructs a Motor object
     *
     * @param hw   hardware map from the OpMode
     * @param name motor name from Robot Controller config
     */
    public Motor(@NonNull HardwareMap hw, String name) {
        this(hw, name, 1.0);
    }
    /**
     * Constructs a Motor object
     *
     * @param hw   hardware map from the OpMode
     * @param name motor name from Robot Controller config
     * @param scalar scales power values
     */
    public Motor(@NonNull HardwareMap hw, String name, double scalar) {
        motor = hw.get(DcMotor.class, name);
        runMode = RunMode.RawPower;
        multiplier = scalar;
        ACHIEVABLE_MAX_TICKS_PER_SECOND = motor.getMotorType().getAchieveableMaxTicksPerSecond();
        encoder = new Encoder(motor::getCurrentPosition);
    }
    /**
     * Constructs a Motor object
     *
     * @param hw   hardware map from the OpMode
     * @param name motor name from Robot Controller config
     * @param cpr  encoder counts per revolution of the motor
     * @param rpm  max revolutions per minute of the motor
     */
    public Motor(@NonNull HardwareMap hw, String name, double cpr, double rpm) {
        this(hw, name, cpr, rpm, 1.0);
    }
    /**
     * Constructs a Motor object
     *
     * @param hw   hardware map from the OpMode
     * @param name motor name from Robot Controller config
     * @param cpr  encoder counts per revolution of the motor
     * @param rpm  max revolutions per minute of the motor
     * @param scalar scales power values
     */
    public Motor(@NonNull HardwareMap hw, String name, double cpr, double rpm, double scalar) {
        multiplier = scalar;
        motor = hw.get(DcMotor.class, name);
        runMode = RunMode.RawPower;
        ACHIEVABLE_MAX_TICKS_PER_SECOND = cpr * rpm / 60;
        encoder = new Encoder(motor::getCurrentPosition);
    }
    /**
     * Set the speed of a motor
     *
     * @param power percentage of total power to set, between -1.0 and 1.0
     */
    public void set(double power) {
        // scale by multiplier
        power *= multiplier;
        switch (runMode) {
            case VelocityControl:
                double speed = power * ACHIEVABLE_MAX_TICKS_PER_SECOND;
                double velocity = velocityController.calculate(getVelocity(), speed) +
                        feedforward.calculate(speed);

                motor.setPower(velocity / ACHIEVABLE_MAX_TICKS_PER_SECOND);
                break;
            case PositionControl:
                double error = positionController.calculate(encoder.getPosition());
                motor.setPower(power * error);
            case RawPower:
                motor.setPower(power);
        }
    }

    /**
     * Sets the distance per pulse of the encoder
     *
     * @param distancePerPulse desired distance per pulse (in units per tick)
     * @return {@link Encoder} with given distance per pulse
     */
    public Encoder setDistancePerPulse(double distancePerPulse) {
        return encoder.setDistancePerPulse(distancePerPulse);
    }

    /**
     * @return distance traveled by encoder
     */
    public double getDistance() {
        return encoder.getDistance();
    }

    /**
     * @return true if motor is at target position
     */
    public boolean atTargetPosition() {
        return positionController.atSetPoint();
    }

    /**
     * Reset encoder so current position is 0
     */
    public void resetEncoder() {
        encoder.reset();
    }

    /**
     * @return velocity coefficients
     */
    public double[] getVelocityCoefficients() {
        return velocityController.getCoefficients();
    }

    /**
     * @return position coefficients
     */
    public double getPositionCoefficient() {
        return positionController.getP();
    }

    /**
     * Set position coefficient
     *
     * @param p proportional gain coefficient
     */
    public void setPositionCoefficient(double p) {
        positionController.setP(p);
    }

    /**
     * @return feedforward coefficients
     */
    public double[] getFeedforwardCoefficients() {
        return new double[]{feedforward.ks, feedforward.kv};
    }

    /**
     * Sets zero power behavior of motor
     *
     * @param behavior desired behavior
     */
    public void setZeroPowerBehavior(ZeroPowerBehavior behavior) {
        motor.setZeroPowerBehavior(behavior.getBehavior());
    }

    /**
     * @return the current position of encoder in ticks
     */
    public int getCurrentPosition() {
        return encoder.getPosition();
    }

    /**
     * @return the corrected velocity
     */
    public double getCorrectedVelocity() {
        return encoder.getCorrectedVelocity();
    }

    /**
     * @return the counts per revolution of the motor
     */
    public double getCPR() {
        return motor.getMotorType().getTicksPerRev();
    }

    /**
     * @return the max possible RPM of the motor
     */
    public double getMaxRPM() {
        return motor.getMotorType().getMaxRPM();
    }

    protected double getVelocity() {
        return get() * ACHIEVABLE_MAX_TICKS_PER_SECOND;
    }

    /**
     * Common method for getting the current set speed of a motor.
     *
     * @return The current set speed. Value is between -1.0 and 1.0.
     */
    public double get() {
        return motor.getPower();
    }

    /**
     * Sets the {@link RunMode} of the motor
     *
     * @param mode the desired runmode
     */
    public void setRunMode(RunMode mode) {
        runMode = mode;
        velocityController.reset();
        positionController.reset();

        if (runMode == RunMode.PositionControl && !targetIsSet) {
            setTargetPosition(getCurrentPosition());
            targetIsSet = false;
        }
    }

    /**
     * Sets the target position for the motor to the given position.
     * Once the motor power is set with {@link #set(double)}, the motor will attempt to move
     * towards the target position.
     *
     * @param position desired position
     */
    public void setTargetPosition(double position) {
        targetIsSet = true;
        positionController.setSetPoint(position);
    }

    public void setPositionTolerance(double tolerance) {
        positionController.setTolerance(tolerance);
    }

    /**
     * Get direction of motor and encoder
     *
     * @return true if reversed, false otherwise
     */
    public boolean getInverted() {
        return motor.getDirection() == DcMotor.Direction.REVERSE;
    }

    /**
     * Set direction of motor and encoder
     *
     * @param inverted true if reversed, false otherwise
     */
    public void setInverted(boolean inverted) {
        motor.setDirection(inverted ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        encoder.setDirection(inverted ? Direction.REVERSE : Direction.FORWARD);
    }

    /**
     * Set the velocity pid coefficients
     *
     * @param p proportional gain coefficient
     * @param i integral gain coefficient
     * @param d derivative gain coefficient
     */
    public void setVelocityCoefficients(double p, double i, double d) {
        velocityController.setPID(p, i, d);
    }

    /**
     * Set feedforward coefficients
     *
     * @param s static gain coefficient
     * @param v velocity gain coefficient
     */
    public void setFeedforwardCoefficients(double s, double v) {
        feedforward = new SimpleMotorFeedforward(s, v);
    }

    /**
     * Stop motor movement
     */
    public void stopMotor() {
        set(0);
    }

    /**
     * Disable the motor
     */
    @Override
    public void disable() {
        motor.close();
    }

    @Override
    public String getDeviceType() {
        return "Motor " + motor.getDeviceName() + " from " + motor.getManufacturer()
                + " in port " + motor.getPortNumber();
    }
}
