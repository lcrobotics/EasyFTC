package org.firstinspires.ftc.teamcode.AutonomousOpModes;

import android.graphics.Bitmap;
import android.graphics.Color;
import android.util.Pair;

import com.lcrobotics.easyftclib.tools.geometry.Translation2d;
import com.lcrobotics.easyftclib.vision.VuforiaSuperOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.jetbrains.annotations.NotNull;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.Arrays;
import java.util.Locale;

// EVERYTHING HERE IS IN MILLIMETERS
@Autonomous
public class FindPost extends VuforiaSuperOp {
    // width of rectangle to sample average red from
    final int SAMPLE_WIDTH = 10;
    // height of rectangle to sample average red from
    final int SAMPLE_HEIGHT = 10;
    // amount of pixels to skip in between samples
    final int SAMPLE_SKIP = SAMPLE_HEIGHT / 2;
    // in mm
    final double focalLength = 3.67;
    final double postHeight = 300.0;
    final int postWidth = 30;
    int i = 1;
    final int threshold = 15;
    int[] pixels;
    // where posts are
    Translation2d post1 = new Translation2d(595/2.0, 0);
    Translation2d post2 = new Translation2d(-595/2.0,0);

    Translation2d centerField = new Translation2d();
    @Override
    public void init() {
        msStuckDetectInit = Integer.MAX_VALUE;
        telemetry.setAutoClear(false);
        super.init();
        vuforia.enableConvertFrameToBitmap();
        VuforiaLocalizer.CloseableFrame frame = null;
        try {
            frame = vuforia
                    .getFrameQueue()
                    .take();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        if (frame == null) {
            return;
        }
        // convert frame to bitmap
        Bitmap bmp = vuforia.convertFrameToBitmap(frame);
        frame.close();
        assert bmp != null;

        Pair<Integer, Integer> firstPixel = findFirstPixel(bmp);

        assert firstPixel != null;
        telemetry.addData("First pixel", firstPixel);
        telemetry.update();
        // find first post
        double distance1 = 0;
        try {
            distance1 = findPost(bmp, firstPixel);
        } catch (IOException e) {
            e.printStackTrace();
        }

        telemetry.addData("Distance to first post", distance1);
        telemetry.update();
        // find second post
        firstPixel = findFirstPixel(bmp);

        double distance2 = 0;
        try {
            distance2 = findPost(bmp, firstPixel);
        } catch (IOException e) {
            e.printStackTrace();
        }

        telemetry.addData("Distance to second post", distance2);
        telemetry.update();
        Translation2d robot = intersection(post1, distance1, post2, distance2);

        telemetry.addData("Coordinates from midpost", robot);
        telemetry.update();
        robot = robot.plus(centerField);

        telemetry.addData("Coordinates", robot);
    }

    public Pair<Integer, Integer> findFirstPixel(Bitmap bmp) {
        for (int x = 0; x < bmp.getWidth(); x++) {
            for (int y = 0; y < bmp.getHeight(); y++) {
                int[] pixels = new int[SAMPLE_HEIGHT*SAMPLE_WIDTH];
                bmp.getPixels(pixels, 0, SAMPLE_WIDTH, x, y, SAMPLE_WIDTH, SAMPLE_HEIGHT);
                if (otherAverage(pixels) > 0.75) {
                    return new Pair<>(x, y);
                }
            }
        }
        return null;
    }
    public double findPost(Bitmap bmp, Pair<Integer, Integer> firstPixel) throws FileNotFoundException {
        int imgHeight = bmp.getHeight();
        int imgWidth = bmp.getWidth();

        pixels = new int[SAMPLE_HEIGHT*SAMPLE_WIDTH];
        int currentX = firstPixel.first;
        int currentY = firstPixel.second;
        while (true) {

            RobotLog.ii("HAX", String.format(Locale.ENGLISH, "Current X: %d", currentX));
            RobotLog.ii("HAX", String.format(Locale.ENGLISH, "Current Y: %d", currentY));

            Bitmap testArea = Bitmap.createBitmap(bmp, currentX, currentY, SAMPLE_WIDTH, SAMPLE_HEIGHT);

            testArea.getPixels(pixels, 0, SAMPLE_WIDTH, 0, 0, SAMPLE_WIDTH, SAMPLE_HEIGHT);

            if (otherAverage(pixels) > 0.75) {
                currentY += SAMPLE_HEIGHT + SAMPLE_SKIP;
                continue;
            }

            // if top half is good and bottom half isn't red, we reached the bottom of the post
            int[] half = new int[(SAMPLE_WIDTH*SAMPLE_HEIGHT)/2];

            // check top half for red
            testArea.getPixels(half, 0, SAMPLE_WIDTH, 0, 0, SAMPLE_WIDTH, SAMPLE_HEIGHT/2);

            // if top half is good, take midline of sample as bottom of post
            if (redAverage(half) > 120) {
                int postHeightPixels = (currentY + SAMPLE_HEIGHT / 2) - firstPixel.second;
                // assume distance is calculated from center of post
                double distance = (focalLength * postHeight) / postHeightPixels;

                // set post to black to make it easier to find other post
                int[] blackout = new int[postHeightPixels*postWidth];
                Arrays.fill(blackout, Color.BLACK);

                bmp.setPixels(blackout, 0, postWidth, firstPixel.first, firstPixel.second, postWidth, postHeightPixels);
                File file = AppUtil.getInstance().getSettingsFile(String.format("FindPost%d.png", i));
                FileOutputStream out = new FileOutputStream(file);
                // save bitmap as png to file
                bmp.compress(Bitmap.CompressFormat.PNG, 100, out);
                i++;
                // now repeat for other post and find intersection between the two circles
                // that's where the robot is
                return distance;
            } else {
                // might have gone past the post
                currentY -= SAMPLE_SKIP;
            }
        }
    }
    // get average red value in pixels
    public double redAverage(int[] pixels) {
        return Arrays.stream(pixels)
                    .map(Color::red)
                    .average()
                    .getAsDouble();
    }

    public double otherAverage(int[] pixels) {
        return Arrays.stream(pixels)
                .filter(this::filter)
                .count() / (double) pixels.length;
    }

    public boolean filter(int color) {
        int r = Color.red(color);
        int g = Color.green(color);
        int b = Color.blue(color);

        return (r > Math.max(g, b) * 2 && Math.abs(g-b) < threshold);
    }
    // see http://paulbourke.net/geometry/circlesphere/ for explanation
    public Translation2d intersection(Translation2d post1, double radius1, Translation2d post2, double radius2) {
        double d = post1.getDistance(post2);
        // no solutions or infinite solutions
        RobotLog.ii("HAX", "d: %s", d);
        RobotLog.ii("HAX", "post1: %s", post1);
        RobotLog.ii("HAX", "post2: %s", post2);
        RobotLog.ii("HAX", "radius 1: %s", radius1);
        RobotLog.ii("HAX", "radius 2: %s", radius2);
        if ((d > radius1 + radius2) || (d < Math.abs(radius1 - radius2)) || (d == 0 && radius1 == radius2)) {
            throw new RuntimeException("bad things");
        }

        double a = (radius1*radius1 - radius2*radius2 + d*d) / (2*d);

        double h = Math.sqrt(radius1*radius1 - a*a);

        Translation2d midpoint = post2.minus(post1).times(a).div(d).plus(post1);

        double x3 = midpoint.getX() + (h*(post2.getY() - post1.getY()))/d;

        double y3 = midpoint.getY() - (h*(post2.getX() - post1.getX()))/d;

        return new Translation2d(x3, y3);
    }
    @Override
    public void loop() {

    }
}
