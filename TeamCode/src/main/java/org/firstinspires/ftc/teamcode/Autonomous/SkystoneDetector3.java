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

public class SkystoneDetector3 {
    VuforiaLocalizer vuforia;

    public SkystoneDetector3(VuforiaLocalizer vuforia)
    {
        this.vuforia = vuforia;
    }

    public enum skystonePOS
    {
        LEFT, CENTER, RIGHT;
    }

    public skystonePOS detectSkystone(boolean save, boolean red)
    {
        Image isRGB = null;
        VuforiaLocalizer.CloseableFrame closeableFrame = null;

        while (isRGB == null)
        {
            Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565,true);
            this.vuforia.setFrameQueueCapacity(1);
            try {
                closeableFrame = this.vuforia.getFrameQueue().take();
                long nImages = closeableFrame.getNumImages();
                for (int i = 0; i <nImages; i++) {
                    if (closeableFrame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565)
                    {
                        isRGB = closeableFrame.getImage(i);
                        break;
                    }
                }
            } catch (InterruptedException a){}
            finally {
                if (closeableFrame!= null) closeableFrame.close();
            }
        }

        if (isRGB!=null)
        {
            Bitmap bitmap = Bitmap.createBitmap(isRGB.getWidth(), isRGB.getHeight(), Bitmap.Config.RGB_565);
            bitmap.copyPixelsFromBuffer(isRGB.getPixels());

            String path = Environment.getExternalStorageDirectory().toString();
            FileOutputStream out;

            String bitmapName;
            String bitmapCroppedName;

            if (red)
            {
                bitmapName = "red.png";
                bitmapCroppedName = "cred.png";
            }
            else
            {
                bitmapName = "blue.png";
                bitmapCroppedName = "cblue.png";
            }

            if (save)
            {
                try {
                    File file = new File(path, bitmapName);
                    out = new FileOutputStream(file);
                    bitmap.compress(Bitmap.CompressFormat.PNG, 100, out);
                    out.flush();
                    out.close();
                } catch (IOException e) {}

            }
        }
    }
}
*/