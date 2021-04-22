package org.firstinspires.ftc.teamcode.AutonomousOpModes;

import com.lcrobotics.easyftclib.pathfinding.Path;
import com.lcrobotics.easyftclib.pathfinding.Waypoint;
import com.lcrobotics.easyftclib.pathfinding.waypoints.GeneralWaypoint;
import com.lcrobotics.easyftclib.pathfinding.waypoints.StartWaypoint;
import com.lcrobotics.easyftclib.tools.geometry.Pose2d;
import com.lcrobotics.easyftclib.tools.geometry.Rotation2d;
import com.lcrobotics.easyftclib.tools.geometry.Translation2d;
import com.lcrobotics.easyftclib.tools.geometry.Twist2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class OdoPathTest extends ThreeWheelOdometryTest {
    Path path;
    @Override
    public void init() {
        Waypoint w1 = new StartWaypoint(0, 0);
        Waypoint w2 = new GeneralWaypoint(
                new Pose2d(new Translation2d(20, 5), new Rotation2d(0)),
                1, 1, 10);


    }

    @Override
    public void loop() {
        odometry.updatePose();
    }
}
