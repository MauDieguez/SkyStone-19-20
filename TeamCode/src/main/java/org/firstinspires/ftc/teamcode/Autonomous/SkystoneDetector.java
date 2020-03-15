package org.firstinspires.ftc.teamcode.Autonomous;


import android.graphics.Bitmap;
import android.graphics.Color;
import android.os.Environment;

import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;

import static android.os.SystemClock.sleep;
import static edu.spa.ftclib.internal.Global.telemetry;

public class SkystoneDetector {
    VuforiaLocalizer vuforia;

    // Inicializamos el detector
    public SkystoneDetector(VuforiaLocalizer vuforia)
    {
        this.vuforia = vuforia;
    }

    // Para saber donde esta cada skytone
    public enum skystonePos
    {
        LEFT, CENTER, RIGHT, NONE
    }

    // Regresara donde esta ubicada la skystone
    public skystonePos vuforiaScan(boolean save, boolean red)
    {
        Image isRgb = null;

        double YellowCountLeft = 1;
        double YellowCountCenter = 1;
        double YellowCountRight = 1;

        double BlackCountLeft = 1;
        double BlackCountCenter = 1;
        double BlackCountRight = 1;

        long greenRight = 0;
        long greenCenter = 0;
        long greenLeft = 0;

        skystonePos pos = skystonePos.NONE;



        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true); // Permite regresar una foto en RGB
        VuforiaLocalizer.CloseableFrame closeableFrame = null;
        this.vuforia.setFrameQueueCapacity(1);
        while (isRgb == null)
        {
            sleep(3000);
            try
            {
                closeableFrame = this.vuforia.getFrameQueue().take(); // Tomamos la foto con vuforia
                long nImages = closeableFrame.getNumImages(); // Cuantas fotos tomo Vuforia

                for (int i=0;i<nImages;i++)
                {
                    if (closeableFrame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) // Si la foto tiene color
                    {
                        isRgb = closeableFrame.getImage(i);
                        break;
                    }
                }
            }
            catch (InterruptedException exc)
            {
            }
            finally {
                if (closeableFrame != null) closeableFrame.close();
            }
        }

        // Tenemos que crear un bitmap


        if (isRgb!= null)
        {

            Bitmap bitmap = Bitmap.createBitmap(isRgb.getWidth(), isRgb.getHeight(), Bitmap.Config.RGB_565); // Creamos un bitmap con colores
            bitmap.copyPixelsFromBuffer(isRgb.getPixels()); //Transfiere el color al bitmap

            // Para hacer debug

            String path = Environment.getExternalStorageDirectory().toString(); // Path para salvar la imagen
            FileOutputStream out = null;

            String bitmapName;
            String bitmapNameCropped;

            if (red) {
                bitmapName = "Red.png";
                bitmapNameCropped = "CropRed.png";
            }
            else
            {
                bitmapName = "Blue.png";
                bitmapNameCropped = "CropBlue.png";
            }
            // Sirve solo para debuggera, salva archivos en el celular
            if (save)
            {
                try {
                    File file = new File(path, bitmapName);
                    out = new FileOutputStream(file);
                    bitmap.compress(Bitmap.CompressFormat.PNG, 100, out);
                } catch (Exception e){
                    e.printStackTrace();
                } finally {
                    try
                    {
                        if (out != null)
                        {
                            out.flush();
                            out.close();
                        }
                    }
                        catch (IOException e)
                        {
                            e.printStackTrace();
                        }
                    }
            }

            int cropStartX = (int) (bitmap.getWidth() / 1.84);
            int cropEndX = (int) (bitmap.getWidth() / 1.67);
            int xWidth = cropEndX - cropStartX;

            int cropStartY = 0;
            int yHeight = bitmap.getHeight();

            bitmap = Bitmap.createBitmap(bitmap, cropStartX, cropStartY, xWidth, yHeight);

            if (save)
            {
                try {
                    File file = new File(path, bitmapNameCropped);
                    out = new FileOutputStream(file);
                    bitmap.compress(Bitmap.CompressFormat.PNG, 100, out);
                } catch (Exception e){
                    e.printStackTrace();
                } finally {
                    try
                    {
                        if (out != null)
                        {
                            out.flush();
                            out.close();
                        }
                    }
                    catch (IOException e)
                    {
                        e.printStackTrace();
                    }
                }
            }

            // Hablando de Y

            int pixel;

            int rightStoneStart = 0;
            int centerStoneStart = (int) (bitmap.getHeight() / 2.92);
            int leftStoneStart = (int) (bitmap.getHeight() / 1.37);

            int blockLength = bitmap.getWidth() - 5;

            // Escaneamos el bloque derecho

            for (int i=0; i < blockLength; i ++)
            {
                for (int j = rightStoneStart; j < centerStoneStart; j ++)
                {
                    pixel = bitmap.getPixel(i,j);
                    if (Color.red(pixel) < 200 && Color.blue(pixel) <200 && Color.green(pixel) < 200)
                    {
                        if (Color.red(pixel) < 80 && Color.blue(pixel) < 80) {
                            BlackCountRight += 1;
                        }
                        else
                        {
                            YellowCountRight += 1;
                        }
                    }
                }
            }

            for (int i=0; i < blockLength; i ++)
            {
                for (int j = centerStoneStart; j < leftStoneStart; j ++)
                {
                    pixel = bitmap.getPixel(i, j);
                    if (Color.red(pixel) < 200 && Color.blue(pixel) <200 && Color.green(pixel) < 200)
                    {
                        if (Color.red(pixel) < 80 && Color.blue(pixel) < 80) {
                            BlackCountCenter += 1;
                        }
                        else
                        {
                            YellowCountCenter += 1;
                        }
                    }
                }
            }

            for (int i=0; i < blockLength; i ++)
            {
                for (int j = leftStoneStart; j < bitmap.getHeight(); j ++)
                {
                    pixel = bitmap.getPixel(i,j);
                    if (Color.red(pixel) < 200 && Color.blue(pixel) <200 && Color.green(pixel) < 200)
                    {
                        if (Color.red(pixel) < 80 && Color.blue(pixel) < 80) {
                            BlackCountLeft += 1;
                        }
                        else
                        {
                            YellowCountLeft += 1;
                        }
                    }
                }
            }

            double leftRatio = BlackCountLeft / YellowCountLeft;
            double centerRatio = BlackCountCenter / YellowCountCenter;
            double rightRatio = BlackCountRight / YellowCountRight;

            if (leftRatio > centerRatio && leftRatio > rightRatio)
            {
                pos = skystonePos.LEFT;
            }
            else if (centerRatio > leftRatio && centerRatio > rightRatio)
            {
                pos = skystonePos.CENTER;
            }
            else if (rightRatio > centerRatio && rightRatio > leftRatio)
            {
                pos = skystonePos.RIGHT;
            }
        }
        return pos;
    }
}
