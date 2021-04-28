package org.firstinspires.ftc.teamcode.AutonomousOpModes;

import com.lcrobotics.easyftclib.CommandCenter.driveTrain.MecanumDrive;
import com.lcrobotics.easyftclib.CommandCenter.hardware.Motor;
import com.lcrobotics.easyftclib.encoderOdometry.HolonomicOdometry;
import com.lcrobotics.easyftclib.pathfinding.Path;
import com.lcrobotics.easyftclib.pathfinding.waypoints.EndWaypoint;
import com.lcrobotics.easyftclib.pathfinding.waypoints.GeneralWaypoint;
import com.lcrobotics.easyftclib.pathfinding.waypoints.StartWaypoint;
import com.lcrobotics.easyftclib.tools.geometry.Pose2d;
import com.lcrobotics.easyftclib.tools.geometry.Rotation2d;
import com.lcrobotics.easyftclib.tools.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class PathfindingTest extends LinearOpMode {

    // if needed, one can add a gearing term here
    public static final double WHEEL_DIAMETER = 2;
    private static final double TRACK_WIDTH = 5.5;
    private static final double CENTER_WHEEL_OFFSET = -2.97;
    public final double TICKS_PER_REV = 8192;
    public final double DISTANCE_PER_PULSE = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;
    // declare drive motors
    public Motor frontLeftDrive;
    public Motor backLeftDrive;
    public Motor frontRightDrive;
    public Motor backRightDrive;
    // declare encoders
    public Motor.Encoder leftEncoder;
    public Motor.Encoder rightEncoder;
    public Motor.Encoder horizontalEncoder;
    public HolonomicOdometry odometry;
    public MecanumDrive drive;
    @Override
    public void runOpMode() throws InterruptedException {
        // Get motors from hardware map
        backRightDrive = new Motor(hardwareMap, "BackRightDrive");
        backLeftDrive = new Motor(hardwareMap, "BackLeftDrive");
        frontRightDrive = new Motor(hardwareMap, "FrontRightDrive");
        frontLeftDrive = new Motor(hardwareMap, "FrontLeftDrive");
        frontRightDrive.setInverted(true);
        // set distance per pulse (how far each encoder tick is in inches)
        leftEncoder = backRightDrive.setDistancePerPulse(DISTANCE_PER_PULSE);
        rightEncoder = frontRightDrive.setDistancePerPulse(DISTANCE_PER_PULSE);
        horizontalEncoder = backLeftDrive.setDistancePerPulse(DISTANCE_PER_PULSE);
        // reset encoders
        horizontalEncoder.reset();
        leftEncoder.reset();
        rightEncoder.reset();

        telemetry.addLine("Encoder Positions")
                .addData("Left", leftEncoder::getPosition)
                .addData("Right", rightEncoder::getPosition)
                .addData("Middle", horizontalEncoder::getPosition);

        odometry = new HolonomicOdometry(
                leftEncoder::getDistance,
                rightEncoder::getDistance,
                horizontalEncoder::getDistance,
                TRACK_WIDTH,
                CENTER_WHEEL_OFFSET);

        drive = new MecanumDrive(frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive);
        Path path = new Path(
                new StartWaypoint(0, 0),
                new GeneralWaypoint(10, 5, 0, 0.3, 0.3, 5),
                new EndWaypoint(new Pose2d(new Translation2d(20, 5), new Rotation2d(0)), 0.3, 0.3, 5, 2, 2)
        );

        path.init();

        horizontalEncoder.reset();
        leftEncoder.reset();
        rightEncoder.reset();
        waitForStart();

        path.followPath(drive, odometry);
    }
}
