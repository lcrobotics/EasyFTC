package com.lcrobotics.easyftclib.CommandCenter.driveTrain;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

public class CommandData {

    public static int[] currentPositions;
    public static int[] targetPositions;

    public static double zOrientation;
    public static double[] motorPowers;

    public static Orientation angularOrientation;
    public static AngularVelocity angularVelocity;
    public static Acceleration acceleration;
    public static Velocity velocity;
    public static Position position;


    public CommandData() {
    }
}
