package com.lcrobotics.easyftclib.vision;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

// RingDetector accepts TFObjectDetector and VuforiaFrameGetter object
// to detect the number of rings at the beginning of autonomous
// It analyzes the picture from the webcam to distinguish between 0 and 1 rings (which are inaccurate with TF)
public class RingDetector {

    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    // number of rings RingDetector sees
    public static int numRings;

    private TFObjectDetector tfod;
    public VuforiaFrameGetter frameGetter;

    // Constructor accepts TFObjectDetector and VuforiaFrameGetter object
    public RingDetector(TFObjectDetector tf, VuforiaFrameGetter fg) {
        tfod = tf;
        frameGetter = fg;
        // activate the object detection
        if (tfod != null) {
            tfod.activate();
        }
    }

    // Updates the number of rings that the RingDetector sees
    public void updateNumRings() {
        // Create list of detected objects
        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

        // make sure objects were detected
        if (updatedRecognitions != null) {

            // if 4 rings were detected
            if (updatedRecognitions.get(0).getLabel().equals(LABEL_FIRST_ELEMENT)) {
                numRings = 4;
            } else {
                // if 0 or 1 rings were detected, use frameGetter to confirm
                frameGetterCheck();
            }
        }
    }

    // Uses frameGetter to attempt to detect orange (ring) from webcam image
    // If it detects orange, numRings = 1, if not, numRings = 0
    private void frameGetterCheck() {
        // Predicted top left corner coordinates of the orange region of image
        int x = 400, y = 224;
        // Predicted width and height of the orange region of image
        int w = 100, h = 100;

        // Update frame to get update rgb array
        frameGetter.updateFrame();

        // Calculate the average red, green, and blue pixel values in the predicted orange region
        double avgR = frameGetter.sumOfRect(0, x, y, w, h) / (w * h);
        double avgG = frameGetter.sumOfRect(1, x, y, w, h) / (w * h);
        double avgB = frameGetter.sumOfRect(2, x, y, w, h) / (w * h);

        // Check if average RGB value is close to orange (255, 165, 0)
        if (avgR > 235 && avgG > 145 && avgB < 185 && avgG < 20) {
            // if it is close to orange, numRings is 1
            numRings = 1;
        } else {
            // if its is not close to orange, numRings is 0
            numRings = 0;
        }
    }

    // Shuts down tfod object
    public void shutdown() {
        if (tfod != null) {
            tfod.shutdown();
        }
    }
}
