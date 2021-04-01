package com.lcrobotics.easyftclib.vision;

import com.lcrobotics.easyftclib.vision.RingDetector;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.vuforia.PIXEL_FORMAT;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamServer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import com.lcrobotics.easyftclib.vision.ObjectLocator;
import com.lcrobotics.easyftclib.vision.VuforiaFrameGetter;
import com.vuforia.Vuforia;

public abstract class VuforiaSuperOp extends OpMode {

    private final static String VUFORIA_KEY = "ARgYuCf/////AAABmUYfc1+dVEQsgUBCPA2kCAFRmuTRB/XUfAJzLsRyFDRg6uMMjj6EXM8YNiY5l3oTw83H+PKgfF46gctdzrln2nnVXMebpgN9ULy1cOfdSsPk0hwSZqzcY0LWCj+rPPrZ3JyQT7gf2aw7bo8ZvWedWB7skuGIjg+9cyTJdDyXmXrQ8Bo4r4siTFNTVFxg21OH/Gd8wrVJF4RqjE+kcez3MzcnE2EPCqWTNixSge5yLg+tN87/R/dMPzqHWvmjE6F6J/7/sahPt7FQ9G6tYWnV1impzZsH7T/JT6pGr2SALwHdaNjBGbYY76ZfvAxixEdob9g6qMBhKOyLg6HTP9VzRZ06ksUhErmR2K2LSkyjxBBz";
    public static final float mmPerInch = 25.4f;

    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    // Class Members
    public TFObjectDetector tfod;
    public VuforiaLocalizer vuforia;
    public RingDetector ringDetector = null;
    public VuforiaFrameGetter frameGetter = null;
    public ObjectLocator objectLocator = null;
    public OpenGLMatrix lastLocation = null;

    public void init() {



        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        parameters.useExtendedTracking = true;

        //  Instantiate the Vuforia engine

        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // This is necessary for getting pixels (integral image goal detection, etc)
        boolean[] results = vuforia.enableConvertFrameToFormat(PIXEL_FORMAT.RGB565, PIXEL_FORMAT.YUV);
        if (!results[0]) {
            // Failed to get Vuforia to convert to RGB565.
            throw new RuntimeException("Unable to convince Vuforia to generate RGB565 frames!");
        }
        vuforia.setFrameQueueCapacity(1);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        frameGetter = new VuforiaFrameGetter(vuforia.getFrameQueue());

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsUltimateGoal = this.vuforia.loadTrackablesFromAsset("UltimateGoal");
        objectLocator = new ObjectLocator(targetsUltimateGoal);
        targetsUltimateGoal.activate();
        CameraStreamServer.getInstance().setSource(vuforia);

        // initialize tfod
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
        // initialize ringDetector
        ringDetector = new RingDetector(tfod, frameGetter);
    }

    // Initializes our tf object detection
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);

        tfodParameters.minimumConfidence = 0.8f;

        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}
