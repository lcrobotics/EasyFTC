package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.Color;

import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import java.util.Locale;
import java.util.concurrent.BlockingQueue;

public class VuforiaFrameGetter {
    private BlockingQueue<VuforiaLocalizer.CloseableFrame> frameQueue = null;
    public int[][][] rgbValues = null;
    private int imgHeight = 0;
    private int imgWidth = 0;

    public VuforiaFrameGetter(BlockingQueue<VuforiaLocalizer.CloseableFrame> frameQueue){
        this.frameQueue = frameQueue;
    }

    public void updateFrame(){
        VuforiaLocalizer.CloseableFrame frame;

        try {
            frame = frameQueue.take();

            for (int i = 0; i < frame.getNumImages(); i++) {
                Image img = frame.getImage(i);

                if (img.getFormat() == PIXEL_FORMAT.RGB565) {
                    Image rgb = frame.getImage(i);

                    Bitmap bmp = Bitmap.createBitmap(rgb.getWidth(), rgb.getHeight(), Bitmap.Config.RGB_565);

                    bmp.copyPixelsFromBuffer(rgb.getPixels());

                    if (rgbValues == null) {
                        imgWidth = bmp.getWidth();
                        imgHeight = bmp.getHeight();
                        rgbValues = new int[3][imgWidth][imgHeight];
                    }

                    for (int h = 0; h < imgWidth; h++) {
                        for (int j = 0; j < imgHeight; j++) {

                            int pixel = bmp.getPixel(h, j);

                            rgbValues[0][h][j] = Color.red(pixel);
                            rgbValues[1][h][j] = Color.green(pixel);
                            rgbValues[2][h][j] = Color.blue(pixel);
                        }
                    }
                }
            }
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}
