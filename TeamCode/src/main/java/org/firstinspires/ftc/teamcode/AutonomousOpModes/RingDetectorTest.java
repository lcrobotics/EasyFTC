package org.firstinspires.ftc.teamcode.AutonomousOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class RingDetectorTest extends VuforiaSuperOp {
    public void loop() {

        // update the number of rings the RingDetector sees
        ringDetector.updateNumRings();

        // add the number of rings to telemetry
        telemetry.addData("# of Rings: ", ringDetector.numRings);

    }
}
