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
    int[][][] rgbValues = null;
    private int imgHeight = 0;
    private int imgWidth = 0;

    public VuforiaFrameGetter(BlockingQueue<VuforiaLocalizer.CloseableFrame> frameQueue){
        this.frameQueue = frameQueue;
        // rgbValues = new int[imgHeight][imgWidth][3];
    }

    public void getFrame(){
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
//        try {
//            VuforiaLocalizer.CloseableFrame frame = frameQueue.take();
//
//            for (int i = 0; i < frame.getNumImages(); i++) {
//                Image img = frame.getImage(i);
//
//                if (img.getFormat() == PIXEL_FORMAT.RGB565) {
//                    Bitmap bmp = Bitmap.createBitmap(img.getWidth(), img.getHeight(), Bitmap.Config.RGB_565);
//                    bmp.copyPixelsFromBuffer(img.getPixels());
//                    Bitmap cropped = Bitmap.createBitmap(bmp, 5, 5, croppedWidth, croppedHeight);
//
//                    for (int y = 0; y < croppedHeight; y++) {
//                        for (int x = 0; x < croppedWidth; x++) {
//                            int pixel = cropped.getPixel(x, y);
//                            rgbValues[y][x][0] = Color.red(pixel);
//                            rgbValues[y][x][1] = Color.green(pixel);
//                            rgbValues[y][x][2] = Color.blue(pixel);
//                        }
//                    }
//                }
//            }
//        } catch (InterruptedException e) {
//            e.printStackTrace();
//        }
    }
}
