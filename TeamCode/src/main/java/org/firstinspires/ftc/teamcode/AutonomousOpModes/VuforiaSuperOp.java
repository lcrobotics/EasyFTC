package org.firstinspires.ftc.teamcode.AutonomousOpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.vuforia.PIXEL_FORMAT;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.ObjectLocator;
import org.firstinspires.ftc.teamcode.VuforiaFrameGetter;

public abstract class VuforiaSuperOp extends OpMode {

    private final static String VUFORIA_KEY = "ARgYuCf/////AAABmUYfc1+dVEQsgUBCPA2kCAFRmuTRB/XUfAJzLsRyFDRg6uMMjj6EXM8YNiY5l3oTw83H+PKgfF46gctdzrln2nnVXMebpgN9ULy1cOfdSsPk0hwSZqzcY0LWCj+rPPrZ3JyQT7gf2aw7bo8ZvWedWB7skuGIjg+9cyTJdDyXmXrQ8Bo4r4siTFNTVFxg21OH/Gd8wrVJF4RqjE+kcez3MzcnE2EPCqWTNixSge5yLg+tN87/R/dMPzqHWvmjE6F6J/7/sahPt7FQ9G6tYWnV1impzZsH7T/JT6pGr2SALwHdaNjBGbYY76ZfvAxixEdob9g6qMBhKOyLg6HTP9VzRZ06ksUhErmR2K2LSkyjxBBz";
    public static final float mmPerInch = 25.4f;

    // Class Members
    public VuforiaLocalizer vuforia;
    public VuforiaLocalizer vuforiaTop;
    public VuforiaFrameGetter frameGetter = null;
    public ObjectLocator objectLocator = null;
    public VuforiaFrameGetter frameGetterTop = null;
    public ObjectLocator objectLocatorTop = null;
    public OpenGLMatrix lastLocation = null;

    public void init(){
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        parameters.useExtendedTracking = false;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // This is necessary for getting pixels (integral image goal detection, etc)
        boolean[] results = vuforia.enableConvertFrameToFormat(PIXEL_FORMAT.RGB565, PIXEL_FORMAT.YUV);
        if (!results[0]) { // Failed to get Vuforia to convert to RGB565.
            throw new RuntimeException("Unable to convince Vuforia to generate RGB565 frames!");
        }
        vuforia.setFrameQueueCapacity(1);
        frameGetter = new VuforiaFrameGetter(vuforia.getFrameQueue());

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsUltimateGoal = this.vuforia.loadTrackablesFromAsset("UltimateGoal");
        objectLocator = new ObjectLocator(targetsUltimateGoal);
        targetsUltimateGoal.activate();




        // Same thing for top webcam
        VuforiaLocalizer.Parameters parametersTop = new VuforiaLocalizer.Parameters();

        parametersTop.vuforiaLicenseKey = VUFORIA_KEY;
        parametersTop.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parametersTop.cameraName = hardwareMap.get(WebcamName.class, "Webcam 2");
        parametersTop.useExtendedTracking = false;

        //  Instantiate the Vuforia engine
        vuforiaTop = ClassFactory.getInstance().createVuforia(parametersTop);

        // This is necessary for getting pixels (integral image goal detection, etc)
        boolean[] resultsTop = vuforiaTop.enableConvertFrameToFormat(PIXEL_FORMAT.RGB565, PIXEL_FORMAT.YUV);
        if (!resultsTop[0]) { // Failed to get Vuforia to convert to RGB565.
            throw new RuntimeException("Unable to convince Vuforia to generate RGB565 frames!");
        }
        vuforiaTop.setFrameQueueCapacity(1);
        frameGetterTop = new VuforiaFrameGetter(vuforiaTop.getFrameQueue());

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsUltimateGoalTop = this.vuforiaTop.loadTrackablesFromAsset("UltimateGoal");
        objectLocatorTop = new ObjectLocator(targetsUltimateGoalTop);
        targetsUltimateGoalTop.activate();
    }
}
