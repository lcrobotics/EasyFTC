package org.firstinspires.ftc.teamcode;

import com.lcrobotics.easyftclib.CommandCenter.driveTrain.CommandType;
import com.lcrobotics.easyftclib.CommandCenter.driveTrain.Location;
import com.lcrobotics.easyftclib.CommandCenter.driveTrain.MoveMode;
import com.lcrobotics.easyftclib.CommandCenter.driveTrain.SmartCommand;
import com.lcrobotics.easyftclib.CommandCenter.driveTrain.SmartTrain;
import com.lcrobotics.easyftclib.CommandCenter.driveTrain.WheelType;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;
import java.util.Arrays;

@Autonomous
public class SmartTrainTest extends OpMode {
    ArrayList<Double> commands = new ArrayList<Double>();

    DcMotor fl;
    DcMotor fr;
    DcMotor bl;
    DcMotor br;
    SmartTrain wheels;

    @Override
    public void init() {
        // get motors from hardware
        fl = hardwareMap.get(DcMotor.class, "FrontLeftDrive");
        fr = hardwareMap.get(DcMotor.class, "FrontRightDrive");
        bl = hardwareMap.get(DcMotor.class, "BackLeftDrive");
        br = hardwareMap.get(DcMotor.class, "BackRightDrive");

        wheels = new SmartTrain(WheelType.MECANUM, fl, fr, bl, br, 16, 27);
        Location[] points = new Location[]{
                new Location(0, 30, 0, MoveMode.Y_FIRST),
                new Location(-30, 30, 0, MoveMode.X_FIRST),
                new Location(-30, 0, 0, MoveMode.Y_FIRST),
                new Location(0, 0, 0, MoveMode.X_FIRST)
        };
        //wheels.addPoints(Arrays.asList(points));
        telemetry.addData("SmartTrain", wheels);
    }

    @Override
    public void start() {
       wheels.execute(new SmartCommand(-30, CommandType.DRIVE));
    }

    @Override
    public void loop() {
        //wheels.update();

        telemetry.update();
    }
}
