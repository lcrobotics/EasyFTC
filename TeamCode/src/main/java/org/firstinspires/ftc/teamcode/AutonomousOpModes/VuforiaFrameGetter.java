package org.firstinspires.ftc.teamcode.AutonomousOpModes;

import android.graphics.Bitmap;
import android.graphics.Color;

import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import java.util.concurrent.BlockingQueue;

public class VuforiaFrameGetter {
    private BlockingQueue<VuforiaLocalizer.CloseableFrame> frameQueue = null;
    int croppedWidth = 5, croppedHeight = 5;
    int[][][] rgbValues;

    public VuforiaFrameGetter(BlockingQueue<VuforiaLocalizer.CloseableFrame> frameQueue, int imgWidth, int imgHeight){
        this.frameQueue = frameQueue;
        croppedWidth = imgWidth;
        croppedHeight = imgHeight;
        rgbValues = new int[imgHeight][imgWidth][3];
    }

    public void updateFrame(){
        try {
            VuforiaLocalizer.CloseableFrame frame = frameQueue.take();

            for (int i = 0; i < frame.getNumImages(); i++) {
                Image img = frame.getImage(i);

                if (img.getFormat() == PIXEL_FORMAT.RGB565) {
                    Bitmap bmp = Bitmap.createBitmap(img.getWidth(), img.getHeight(), Bitmap.Config.RGB_565);
                    bmp.copyPixelsFromBuffer(img.getPixels());
                    Bitmap cropped = Bitmap.createBitmap(bmp, 5, 5, croppedWidth, croppedHeight);

                    for (int y = 0; y < croppedHeight; y++) {
                        for (int x = 0; x < croppedWidth; x++) {
                            int pixel = cropped.getPixel(x, y);
                            rgbValues[y][x][0] = Color.red(pixel);
                            rgbValues[y][x][1] = Color.green(pixel);
                            rgbValues[y][x][2] = Color.blue(pixel);
                        }
                    }
                }
            }
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}
