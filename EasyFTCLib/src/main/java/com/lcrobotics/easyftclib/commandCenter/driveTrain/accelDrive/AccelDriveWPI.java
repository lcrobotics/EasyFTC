package com.lcrobotics.easyftclib.commandCenter.driveTrain.accelDrive;

import java.util.Objects;

public class AccelDriveWPI {
    private int direction;

    private Constraints constraints;
    private State initial;
    private State goal;

    private double endAccel;
    private double endFullSpeed;
    private double endDeccel;

    public static class Constraints {
        public double maxVelocity;

        public double maxAccel;

        public Constraints() {
        }

        /**
         * Construct constrains for a trapezoid profile
         *
         * @param maxVelocity maximum velocity
         * @param maxAccel maximum acceleration
         */
        public Constraints(double maxVelocity, double maxAccel) {
            this.maxVelocity = maxVelocity;
            this.maxAccel = maxAccel;
        }
    }

    public static class State {
        public double position;

        public double velocity;

        public State() {
        }

        public State(double position, double velocity) {
            this.position = position;
            this.velocity = velocity;
        }

        @Override
        public boolean equals(Object o) {
            if (this == o) return true;
            if (o == null || getClass() != o.getClass()) return false;
            State state = (State) o;
            return Double.compare(state.position, position) == 0 &&
                    Double.compare(state.velocity, velocity) == 0;
        }

        @Override
        public int hashCode() {
            return Objects.hash(position, velocity);
        }
    }

    /**
     * Construct an AccelDrive
     *
     * @param constraints
     * @param initial
     * @param goal
     */
    public AccelDriveWPI(Constraints constraints, State initial, State goal) {
        this.constraints = constraints;
        this.initial = initial;
        this.goal = goal;
    }
}
