package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.Color;

import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import java.util.concurrent.BlockingQueue;

public class VuforiaFrameGetter {
    private BlockingQueue<VuforiaLocalizer.CloseableFrame> frameQueue = null;
    public int[][][] rgbValues = null;
    public int[][][] integralImg = null;
    public int imgWidth = 0;
    public int imgHeight = 0;
    public int xMax = -1, yMax = -1;

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
                        integralImg = new int[3][imgWidth+1][imgHeight+1];
                    }

                    for (int x = 0; x < imgWidth; x++) {
                        for (int y = 0; y < imgHeight; y++) {

                            int pixel = bmp.getPixel(x, y);
                            int red = Color.red(pixel);
                            int green = Color.green(pixel);
                            int blue = Color.blue(pixel);

                            rgbValues[0][x][y] = red;
                            rgbValues[1][x][y] = green;
                            rgbValues[2][x][y] = blue;

                            // Integral image needs to be initialized to original
                            integralImg[0][x+1][y+1] = red;
                            integralImg[1][x+1][y+1] = green;
                            integralImg[2][x+1][y+1] = blue;
                        }
                    }

                    // Calculate the integral image
                    for (int x = 1; x < imgWidth; x++) {
                        for (int y = 1; y < imgHeight+1; y++) {
                            integralImg[0][x+1][y] += integralImg[0][x][y];
                            integralImg[1][x+1][y] += integralImg[1][x][y];
                            integralImg[2][x+1][y] += integralImg[2][x][y];
                        }
                    }

                    for (int x = 1; x < imgWidth+1; x++) {
                        for (int y = 1; y < imgHeight; y++) {
                            integralImg[0][x][y+1] += integralImg[0][x][y];
                            integralImg[1][x][y+1] += integralImg[1][x][y];
                            integralImg[2][x][y+1] += integralImg[2][x][y];
                        }
                    }

                }
            }
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    // Return the sum of the values in channel c of the image
    public int sumOfRect(int c, int x, int y, int w, int h) {
        if (integralImg == null) {
            return 0;
        }
        return
                +integralImg[c][y+h][x+w]
                -integralImg[c][y+h][x]
                -integralImg[c][y][x+w]
                +integralImg[c][y][x];
    }

    public void updateMaxRect(int c, int w, int h) {
        xMax = -1;
        yMax = -1;
        int sMax = -1;
        for (int x = 0; x <= imgWidth-w; x++) {
            for (int y = 0; y <= imgHeight-h; y++) {
                int s = sumOfRect(c, x, y, w, h);
                if (s > sMax) {
                    xMax = x;
                    yMax = y;
                    sMax = s;
                }
            }
        }
    }
}
