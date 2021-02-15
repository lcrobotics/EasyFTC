package org.firstinspires.ftc.teamcode;

import com.lcrobotics.easyftclib.commandCenter.driveTrain.MecanumDrive;
import com.lcrobotics.easyftclib.commandCenter.hardware.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
@Disabled
public class MecanumTest extends OpMode {
    final int CPR = 448;
    final int MAX_RPM = 64;
    Motor fl, fr, br, bl;
    MecanumDrive wheels;

    @Override
    public void init() {
        fl = new Motor(hardwareMap, "FrontLeftDrive", CPR, MAX_RPM);
        fr = new Motor(hardwareMap, "FrontRightDrive", CPR, MAX_RPM);
        br = new Motor(hardwareMap, "BackRightDrive", CPR, MAX_RPM);
        bl = new Motor(hardwareMap, "BackLeftDrive", CPR, MAX_RPM);

        fr.setPositionCoefficient(5);
        fl.setPositionCoefficient(5);
        bl.setPositionCoefficient(5);
        br.setPositionCoefficient(5);

        fr.setVelocityCoefficients(2.0, 0.5, 1);
        fl.setVelocityCoefficients(2.0, 0.5, 1);
        bl.setVelocityCoefficients(2.0, 0.5, 1);
        br.setVelocityCoefficients(2.0, 0.5, 1);

        wheels = new MecanumDrive(fl, fr, bl, br);
    }

    @Override
    public void loop() {
        wheels.driveRobotCentric(-gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, true);
    }
}
