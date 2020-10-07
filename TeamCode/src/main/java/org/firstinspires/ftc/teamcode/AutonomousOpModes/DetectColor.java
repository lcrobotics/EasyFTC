package org.firstinspires.ftc.teamcode.AutonomousOpModes;

import android.graphics.Bitmap;
import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.Locale;

@Autonomous
public class DetectColor extends OpMode {
    private final static String VUFORIA_KEY = "ARgYuCf/////AAABmUYfc1+dVEQsgUBCPA2kCAFRmuTRB/XUfAJzLsRyFDRg6uMMjj6EXM8YNiY5l3oTw83H+PKgfF46gctdzrln2nnVXMebpgN9ULy1cOfdSsPk0hwSZqzcY0LWCj+rPPrZ3JyQT7gf2aw7bo8ZvWedWB7skuGIjg+9cyTJdDyXmXrQ8Bo4r4siTFNTVFxg21OH/Gd8wrVJF4RqjE+kcez3MzcnE2EPCqWTNixSge5yLg+tN87/R/dMPzqHWvmjE6F6J/7/sahPt7FQ9G6tYWnV1impzZsH7T/JT6pGr2SALwHdaNjBGbYY76ZfvAxixEdob9g6qMBhKOyLg6HTP9VzRZ06ksUhErmR2K2LSkyjxBBz";

    private VuforiaLocalizer vuforia;

    Image rgb;
    @Override
    public void init() {
        // initialize vuforia
        initVuforia();
    }



    @Override
    public void loop() {
        VuforiaLocalizer.CloseableFrame frame;

        try {
            frame = vuforia.getFrameQueue().take();

            for (int i = 0; i < frame.getNumImages(); i++) {
                Image img = frame.getImage(i);

                if (img.getFormat() == PIXEL_FORMAT.RGB565) {
                    rgb = frame.getImage(i);

                    Bitmap bmp = Bitmap.createBitmap(rgb.getWidth(), rgb.getHeight(), Bitmap.Config.RGB_565);

                    bmp.copyPixelsFromBuffer(rgb.getPixels());

                    Bitmap cropped = Bitmap.createBitmap(bmp, 5, 5, 5, 5);

                    for (int h = 0; i < cropped.getWidth(); i++) {
                        for (int j = 0; j < cropped.getHeight(); j++) {

                            int pixel = cropped.getPixel(h, j);

                            int red = Color.red(pixel);
                            int blue = Color.blue(pixel);
                            int green = Color.green(pixel);
                            int alpha = Color.alpha(pixel);

                            telemetry.addData(String.format(Locale.US, "pixel %d %d", h, j),
                                    String.format(Locale.US, "red: %d green: %d blue: %d alpha: %d", red, green, blue, alpha));
                        }
                    }
                }
            }
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

    }
    /*
     * Taken from {@link ConceptTensorFlowObjectDetectionWebcam}
     * initializes our vuforia object
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        //parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }
}

