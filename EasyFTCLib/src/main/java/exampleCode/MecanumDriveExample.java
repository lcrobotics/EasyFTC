package exampleCode;

import com.lcrobotics.easyftclib.CommandCenter.driveTrain.MecanumDrive;
import com.lcrobotics.easyftclib.CommandCenter.hardware.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class MecanumDriveExample extends OpMode {
    // declare Motors
    Motor FrontLeftDrive, FrontRightDrive, BackLeftDrive, BackRightDrive;
    // declare drive constructor
    MecanumDrive drive;
    @Override
    public void init() {
        // Motor Counts per Revolution
        double cpr = 12;
        // Motor Revolutions per Minute
        double rpm = 7;
        // initialize motors
        FrontLeftDrive = new Motor(hardwareMap, "FrontLeftDrive", cpr, rpm);
        FrontRightDrive = new Motor(hardwareMap, "FrontRightDrive", cpr, rpm);
        BackLeftDrive = new Motor(hardwareMap, "BackLeftDrive", cpr, rpm);
        BackRightDrive = new Motor(hardwareMap, "BackRightDrive", cpr, rpm);
        // initialize drive
        drive  = new MecanumDrive(true, FrontLeftDrive, FrontRightDrive, BackLeftDrive, BackRightDrive);
    }

    @Override
    public void loop() {
        // move forward (values correspond to desired motor powers)
        drive.drive(0, 1, 0);
    }
}
