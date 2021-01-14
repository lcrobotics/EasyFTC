package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

import java.util.ArrayList;
import java.util.List;

public class NewObjectLocator {

    List<VuforiaTrackable> allTrackables;
    public boolean targetVisible = false;
    public OpenGLMatrix lastLocation = null;

    public NewObjectLocator(List<VuforiaTrackable> allTrackables){
        this.allTrackables = allTrackables;
    }

    //public BoolAndMatrix getRobotLocation(){
    public void updateRobotLocation(){
        // check all the trackable targets to see which one (if any) is visible.
        targetVisible = false;
        ArrayList<OpenGLMatrix> robotLocations = new ArrayList<>();
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                //telemetry.addData("Visible Target", trackable.getName());
                targetVisible = true;

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    robotLocations.add(robotLocationTransform);
                }
            }
        }

        if (!robotLocations.isEmpty()) {
            // Average all values in robotLocations
            OpenGLMatrix avgLocation = robotLocations.get(0);
            for (int i = 1; i < robotLocations.size(); i++)
                avgLocation.add(robotLocations.get(i));
            avgLocation.multiply(1.0f / robotLocations.size());
            lastLocation = avgLocation;
        }

        //return new BoolAndMatrix(targetVisible, lastLocation);
    }
}
