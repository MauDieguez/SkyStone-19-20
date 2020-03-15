/*package org.firstinspires.ftc.teamcode.Autonomous;

import android.graphics.Bitmap;
import android.os.Environment;

import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStream;

public class SkystoneD {
    VuforiaLocalizer vuforia;

    public SkystoneD(VuforiaLocalizer vuforia)
    {
        this.vuforia = vuforia;
    }

    public enum skystonePos
    {
        LEFT, CENTER, RIGHT;
    }

    public skystonePos skystoneLocator(boolean save, boolean red)
    {
        Image isRGB = null;

        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        VuforiaLocalizer.CloseableFrame closeableFrame = null;
        this.vuforia.setFrameQueueCapacity(0);

        try
        {
            closeableFrame = this.vuforia.getFrameQueue().take();
            long nImages = closeableFrame.getNumImages();
            for (int i=0;i<nImages;i++)
            {
                if (closeableFrame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565)
                {
                    isRGB = closeableFrame.getImage(i);
                    break;
                }
            }
        } catch (InterruptedException a){} finally {
            if (isRGB != null)closeableFrame.close();
        }

        // Ya tenog la imagen, ahora a crear un bitmap

        if (isRGB != null)
        {
            Bitmap bitmap = Bitmap.createBitmap(isRGB.getWidth(), isRGB.getHeight(), Bitmap.Config.RGB_565);
            bitmap.copyPixelsFromBuffer(isRGB.getPixels());

            String path = Environment.getExternalStorageDirectory().toString();
            OutputStream out = null;

            String N;
            String CN;

            if (red)
            {
                N = "bitmapR.png";
                CN = "cbitmapR.png";
            }
            else
            {
                N = "bitmapB.png";
                CN = "cbitmapB.png";
            }

            if (save)
            {
                try {
                    File file = new File(path, N);
                    out = new FileOutputStream(file);
                    bitmap.compress(Bitmap.CompressFormat.PNG,100,out);
                    out.flush();
                    out.close();
                } catch (IOException a){}
            }
        }

    }
}
*/