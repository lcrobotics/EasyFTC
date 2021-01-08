package org.firstinspires.ftc.teamcode.AutonomousOpModes;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

import java.util.List;

public class NewObjectLocator {

    List<VuforiaTrackable> allTrackables;

    public NewObjectLocator(List<VuforiaTrackable> allTrackables){
        this.allTrackables = allTrackables;
    }

    public OpenGLMatrix getRobotLocation(){
        // check all the trackable targets to see which one (if any) is visible.
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    return robotLocationTransform;
                }
                break;
            }
        }
        return null;
    }
}
