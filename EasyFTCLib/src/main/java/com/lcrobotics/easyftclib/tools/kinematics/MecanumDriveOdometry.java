package com.lcrobotics.easyftclib.tools.kinematics;

import com.lcrobotics.easyftclib.tools.geometry.Pose2d;
import com.lcrobotics.easyftclib.tools.geometry.Rotation2d;
import com.lcrobotics.easyftclib.tools.geometry.Twist2d;

/**
 * Class for mecanum drive odometry. Odometry allows you to track the robot's position on the field
 * over a course of a match using readings from your mecanum wheel encoders.
 */
public class MecanumDriveOdometry {
    private final MecanumDriveKinematics kinematics;
    private Pose2d pose;
    private double prevTimestamp = -1;

    private Rotation2d gyroOffset;
    private Rotation2d previousAngle;

    /**
     * Constructs a MecanumDriveOdometry object.
     *
     * @param kinematics  The mecanum drive kinematics for your drivetrain.
     * @param gyroAngle   The angle reported by the gyroscope.
     * @param initialPose The starting position of the robot on the field.
     */
    public MecanumDriveOdometry(MecanumDriveKinematics kinematics, Rotation2d gyroAngle,
                                Pose2d initialPose) {
        this.kinematics = kinematics;
        pose = initialPose;
        gyroOffset = pose.getRotation().minus(gyroAngle);
        previousAngle = pose.getRotation();
    }

    /**
     * Constructs a MecanumDriveOdometry object with the default pose at the origin.
     *
     * @param kinematics The mecanum drive kinematics for your drivetrain.
     * @param gyroAngle  The angle reported by the gyroscope.
     */
    public MecanumDriveOdometry(MecanumDriveKinematics kinematics, Rotation2d gyroAngle) {
        this(kinematics, gyroAngle, new Pose2d());
    }

    /**
     * Resets the robot's position on the field.
     *
     * @param pose      Where your robot is on the field.
     * @param gyroAngle The angle reported by the gyroscope.
     */
    public void resetPosition(Pose2d pose, Rotation2d gyroAngle) {
        this.pose = pose;
        previousAngle = pose.getRotation();
        gyroOffset = this.pose.getRotation().minus(gyroAngle);
    }

    /**
     * Returns the position of the robot on the field.
     *
     * @return The pose of the robot.
     */
    public Pose2d getPose() {
        return pose;
    }

    /**
     * Updates the robot's position on the field using forward kinematics and integration of the pose
     * over time. This method takes in the current time as a parameter to calculate period (difference
     * between two timestamps). The period is used to calculate the change in distance from a
     * velocity. This also takes in an angle parameter which is used instead of the angular rate that
     * is calculated from forward kinematics.
     *
     * @param currentTime The current time in seconds.
     * @param gyroAngle   The angle reported by the gyroscope.
     * @param wheelSpeeds The current wheel speeds.
     * @return The new pose of the robot.
     */
    public Pose2d updateWithTime(double currentTime, Rotation2d gyroAngle,
                                 MecanumDriveWheelSpeeds wheelSpeeds) {
        // calculate time since last call
        double period = prevTimestamp >= 0 ? currentTime - prevTimestamp : 0.0;
        prevTimestamp = currentTime;
        // offset gyro
        Rotation2d angle = gyroAngle.plus(gyroOffset);
        // use forward kinematics to convert wheel speeds to chassis speed.
        ChassisSpeeds chassisState = kinematics.toChassisSpeeds(wheelSpeeds);
        // calculate new position
        Pose2d newPose = pose.exp(
                new Twist2d(
                        chassisState.velocityX * period,
                        chassisState.velocityY * period,
                        angle.minus(previousAngle).getRadians()
                )
        );

        previousAngle = angle;

        pose = new Pose2d(newPose.getTranslation(), angle);

        return pose;
    }

    /**
     * Updates the robot's position on the field using forward kinematics and integration of the pose
     * over time. This method automatically calculates the current time to calculate period
     * (difference between two timestamps). The period is used to calculate the change in distance
     * from a velocity. This also takes in an angle parameter which is used instead of the angular
     * rate that is calculated from forward kinematics.
     *
     * @param gyroAngle   The angle reported by the gyroscope.
     * @param wheelSpeeds The current wheel speeds.
     * @return The new pose of the robot.
     */
    public Pose2d update(Rotation2d gyroAngle, MecanumDriveWheelSpeeds wheelSpeeds) {
        return updateWithTime(System.nanoTime() * 1E-6, gyroAngle, wheelSpeeds);
    }
}
