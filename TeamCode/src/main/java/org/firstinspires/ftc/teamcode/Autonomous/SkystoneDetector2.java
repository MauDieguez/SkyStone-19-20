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

public class SkystoneDetector2 {
    VuforiaLocalizer vuforia;

    public SkystoneDetector2(VuforiaLocalizer vuforia)
    {
        this.vuforia = vuforia;
    }

    public enum skytonePOS
    {
        LEFT, CENTER, RIGHT;
    }

    public getPos(boolean save, boolean red)
    {
        Image isRGB = null;

        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        VuforiaLocalizer.CloseableFrame closeableFrame = null;
        this.vuforia.setFrameQueueCapacity(1);
        while (isRGB == null)
        {
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
            }
            catch (InterruptedException a)
            {
            }
            finally {
                if (closeableFrame!= null) closeableFrame.close();
            }
        }

        // Ya tengo la imagen con color. Hora de crear un bitmap

        if (isRGB != null)
        {
            Bitmap bitmap = Bitmap.createBitmap(isRGB.getWidth(),isRGB.getHeight(), Bitmap.Config.RGB_565); // Aqui solo lo estoy creando
            bitmap.copyPixelsFromBuffer(isRGB.getPixels()); // Aqui ya lo estoy pasando

            String path = Environment.getExternalStorageDirectory().toString();
            FileOutputStream out = null;

            String bitmapName;
            String croppedBitmapName;

            if (red)
            {
                bitmapName = "Red.png";
                croppedBitmapName = "CRed.png";
            }
            else
            {
                bitmapName = "Blue.png";
                croppedBitmapName = "CRed.png";
            }

            if (save)
            {
                try {
                    File file = new File(path, bitmapName);
                    out = new FileOutputStream(file);
                    bitmap.compress(Bitmap.CompressFormat.PNG, 100, out);
                    out.flush();
                    out.close();
                }
                catch (IOException e){}
            }

        }

    }

}
*/