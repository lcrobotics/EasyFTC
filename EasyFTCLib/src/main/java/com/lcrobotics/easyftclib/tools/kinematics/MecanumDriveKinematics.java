package com.lcrobotics.easyftclib.tools.kinematics;

import com.lcrobotics.easyftclib.tools.geometry.Translation2d;

import org.ejml.simple.SimpleMatrix;

/**
 * Helper class that converts a chassis velocity (dx, dy, and dtheta components) into individual
 * wheel speeds.
 *
 * <p>The inverse kinematics (converting from a desired chassis velocity to individual wheel speeds)
 * uses the relative locations of the wheels with respect to the center of rotation. The center of
 * rotation for inverse kinematics is also variable. This means that you can set your set your
 * center of rotation in a corner of the robot to perform special evasion maneuvers.
 *
 * <p>Forward kinematics (converting an array of wheel speeds into the overall chassis motion) is
 * performs the exact opposite of what inverse kinematics does. Since this is an overdetermined
 * system (more equations than variables), we use a least-squares approximation.
 *
 * <p>The inverse kinematics: [wheelSpeeds] = [wheelLocations] * [chassisSpeeds] We take the
 * Moore-Penrose pseudoinverse of [wheelLocations] and then multiply by [wheelSpeeds] to get our
 * chassis speeds.
 *
 * <p>Forward kinematics is also used for odometry -- determining the position of the robot on the
 * field using encoders and a gyro.
 */
public class MecanumDriveKinematics {
    private final SimpleMatrix forwardKinematics;
    private final Translation2d frontLeftWheel;
    private final Translation2d frontRightWheel;
    private final Translation2d backLeftWheel;
    private final Translation2d backRightWheel;
    private SimpleMatrix inverseKinematics;
    private Translation2d prevCoR = new Translation2d();

    /**
     * Constructs kinematics for a mecanum dirve.
     *
     * @param frontLeftWheel  The location of the front-left wheel relative to the physical
     *                        center of the robot.
     * @param frontRightWheel The location of the front-right wheel relative to the physical
     *                        center of the robot.
     * @param backLeftWheel   The location of the back-left wheel relative to the physical
     *                        center of the robot.
     * @param backRightWheel  The location of the back-right wheel relative to the physical
     *                        center of the robot.
     */
    public MecanumDriveKinematics(Translation2d frontLeftWheel, Translation2d frontRightWheel,
                                  Translation2d backLeftWheel, Translation2d backRightWheel) {
        this.frontLeftWheel = frontLeftWheel;
        this.frontRightWheel = frontRightWheel;
        this.backLeftWheel = backLeftWheel;
        this.backRightWheel = backRightWheel;

        inverseKinematics = new SimpleMatrix(4, 3);

        setInverseKinematics(frontLeftWheel, frontRightWheel, backLeftWheel, backRightWheel);
        forwardKinematics = inverseKinematics.pseudoInverse();
    }

    public MecanumDriveWheelSpeeds toWheelSpeeds(
            ChassisSpeeds chassisSpeeds, Translation2d centerOfRotation) {
        // compute matrix if new center of rotation
        if (!centerOfRotation.equals(prevCoR)) {
            Translation2d fl = frontLeftWheel.minus(centerOfRotation);
            Translation2d fr = frontRightWheel.minus(centerOfRotation);
            Translation2d bl = backLeftWheel.minus(centerOfRotation);
            Translation2d br = backRightWheel.minus(centerOfRotation);

            setInverseKinematics(fl, fr, bl, br);
            prevCoR = centerOfRotation;
        }

        SimpleMatrix chassisSpeedsVector = new SimpleMatrix(3, 1);

        chassisSpeedsVector.setColumn(
                0, 0,
                chassisSpeeds.velocityX,
                chassisSpeeds.velocityY,
                chassisSpeeds.omegaRadiansPerSecond
        );

        SimpleMatrix wheelsMatrix = inverseKinematics.mult(chassisSpeedsVector);

        return new MecanumDriveWheelSpeeds(
                wheelsMatrix.get(0, 0),
                wheelsMatrix.get(1, 0),
                wheelsMatrix.get(2, 0),
                wheelsMatrix.get(3, 0)
        );
    }

    /**
     * Performs inverse kinematics. See {@link #toWheelSpeeds(ChassisSpeeds, Translation2d)} for
     * more information.
     *
     * @param chassisSpeeds The desired chassis speeds.
     * @return The wheel speeds.
     */
    public MecanumDriveWheelSpeeds toWheelSpeeds(ChassisSpeeds chassisSpeeds) {
        return toWheelSpeeds(chassisSpeeds, new Translation2d());
    }

    /**
     * Performs forward kinematics to return the resulting chassis speed from the given wheel
     * speeds. This method is often used for odometry.
     *
     * @param wheelSpeeds The current mecanum drive wheel speeds.
     * @return The resulting chassis speed
     */
    public ChassisSpeeds toChassisSpeeds(MecanumDriveWheelSpeeds wheelSpeeds) {
        SimpleMatrix wheelSpeedsMatrix = new SimpleMatrix(4, 1);

        wheelSpeedsMatrix.setColumn(
                0, 0,
                wheelSpeeds.frontLeftSpeed,
                wheelSpeeds.frontRightSpeed,
                wheelSpeeds.backLeftSpeed,
                wheelSpeeds.backRightSpeed
        );

        SimpleMatrix chassisSpeedsVector = forwardKinematics.mult(wheelSpeedsMatrix);

        return new ChassisSpeeds(
                chassisSpeedsVector.get(0, 0),
                chassisSpeedsVector.get(1, 0),
                chassisSpeedsVector.get(2, 0)
        );
    }

    /**
     * Construct inverse kinematics matrix from wheel locations.
     *
     * @param fl The location of the front-left wheel relative to the physical center of the robot.
     * @param fr The location of the front-right wheel relative to the physical center of the robot.
     * @param bl The location of the back-left wheel relative to the physical center of the robot.
     * @param br The location of the back-right wheel relative to the physical center of the robot.
     */
    private void setInverseKinematics(Translation2d fl, Translation2d fr,
                                      Translation2d bl, Translation2d br) {
        inverseKinematics.setRow(0, 0, 1, -1, -(fl.getX() + fl.getY()));
        inverseKinematics.setRow(1, 0, 1, 1, fr.getX() - fr.getY());
        inverseKinematics.setRow(2, 0, 1, 1, bl.getX() - bl.getY());
        inverseKinematics.setRow(3, 0, 1, -1, -(br.getX() + br.getY()));
        inverseKinematics = inverseKinematics.scale(1.0 / Math.sqrt(2));
    }
}
