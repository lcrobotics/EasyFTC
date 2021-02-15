package com.lcrobotics.easyftclib.CommandCenter.driveTrain.accelDrive;

public class AccelDriveWPI {
    private int direction;

    public static class Constraints {
        public double maxVelocity;

        public double maxAccel;

        /**
         *
         * @param maxVelocity
         * @param maxAccel
         */
        public Constraints(double maxVelocity, double maxAccel) {
            this.maxVelocity = maxVelocity;
            this.maxAccel = maxAccel;
        }
    }
}
