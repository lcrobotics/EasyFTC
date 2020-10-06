package org.firstinspires.ftc.teamcode.AutonomousOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptTensorFlowObjectDetectionWebcam;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.SuperOp;

import java.util.List;
import java.util.Locale;

@Autonomous
public class VisionAutoTest extends SuperOp {
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private final static String VUFORIA_KEY = "ARgYuCf/////AAABmUYfc1+dVEQsgUBCPA2kCAFRmuTRB/XUfAJzLsRyFDRg6uMMjj6EXM8YNiY5l3oTw83H+PKgfF46gctdzrln2nnVXMebpgN9ULy1cOfdSsPk0hwSZqzcY0LWCj+rPPrZ3JyQT7gf2aw7bo8ZvWedWB7skuGIjg+9cyTJdDyXmXrQ8Bo4r4siTFNTVFxg21OH/Gd8wrVJF4RqjE+kcez3MzcnE2EPCqWTNixSge5yLg+tN87/R/dMPzqHWvmjE6F6J/7/sahPt7FQ9G6tYWnV1impzZsH7T/JT6pGr2SALwHdaNjBGbYY76ZfvAxixEdob9g6qMBhKOyLg6HTP9VzRZ06ksUhErmR2K2LSkyjxBBz";

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;
    // timer for various motor applications
    private static int numRings;
    @Override
    public void init() {
        super.init();
        // initialize vuforia
        initVuforia();
        // initialize tfod
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
        // add logging for motor powers of the drive train
        telemetry.addData("Drive Train: ", wheels);

        // activate the object detection
        if (tfod != null) {
            tfod.activate();
        }
    }

    @Override
    public void loop() {
        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
        // make sure objects were detected
        if (updatedRecognitions != null) {
            // get amount of rings
            telemetry.addData("# Objects Detected: ", updatedRecognitions.size());
            // get time to drive to configuration zone
            int i = 0;
            for (Recognition recognition : updatedRecognitions) {
                // print recognition boundaries
                telemetry.addData(String.format(Locale.US, "label (%d)", i), recognition.getLabel());
                telemetry.addData(String.format(Locale.US, "  left,top (%d)", i), "%.03f , %.03f",
                        recognition.getLeft(), recognition.getTop());
                telemetry.addData(String.format(Locale.US, "  right,bottom (%d)", i), "%.03f , %.03f",
                        recognition.getRight(), recognition.getBottom());
                i++;
            }
            telemetry.update();
            // get number of rings from tensorflow recognitions
            if (updatedRecognitions.size() == 0) {
                // if nothing was detected
                numRings = 0;
            } else if (updatedRecognitions.get(0).getLabel().equals(LABEL_SECOND_ELEMENT)) {
                // if a single ring was detected
                numRings = 1;
            } else if (updatedRecognitions.get(0).getLabel().equals(LABEL_FIRST_ELEMENT)) {
                // if 4 rings were detected
                numRings = 4;
            }
            telemetry.addData("number of rings: ", numRings);
        }
    }
    /**
     * Taken from {@link ConceptTensorFlowObjectDetectionWebcam}
     * initializes our vuforia object
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Taken from {@link ConceptTensorFlowObjectDetectionWebcam}
     * initializes our tf object detection
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);

        tfodParameters.minimumConfidence = 0.8f;

        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}
