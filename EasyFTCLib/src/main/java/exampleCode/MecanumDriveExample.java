package exampleCode;

import com.lcrobotics.easyftclib.commandCenter.driveTrain.MecanumDrive;
import com.lcrobotics.easyftclib.commandCenter.hardware.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class MecanumDriveExample extends OpMode {
    // Motor Counts per Revolution
    final int cpr = 448;
    // Motor Revolutions per Minute
    final int rpm = 64;
    // declare Motors
    Motor FrontLeftDrive, FrontRightDrive, BackLeftDrive, BackRightDrive;
    // declare drive constructor
    MecanumDrive drive;

    @Override
    public void init() {
        // initialize motors
        FrontLeftDrive = new Motor(hardwareMap, "FrontLeftDrive", cpr, rpm);
        FrontRightDrive = new Motor(hardwareMap, "FrontRightDrive", cpr, rpm);
        BackLeftDrive = new Motor(hardwareMap, "BackLeftDrive", cpr, rpm);
        BackRightDrive = new Motor(hardwareMap, "BackRightDrive", cpr, rpm);

        // initialize drive
        drive = new MecanumDrive(true, FrontLeftDrive, FrontRightDrive, BackLeftDrive, BackRightDrive);
    }

    @Override
    public void loop() {
        // move forward (values correspond to desired motor powers)
        // square squares all the inputs so drivers have greater control
        drive.drive(-gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, true);
    }
}
