package org.firstinspires.ftc.teamcode.Hardware;

import android.graphics.drawable.GradientDrawable;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static edu.spa.ftclib.internal.Global.telemetry;

public class IMU {
    BNO055IMU imu;
    Orientation angles;

    public IMU(BNO055IMU imu)
    {
        this.imu = imu;
    }

    public void initialize()
    {
        BNO055IMU.Parameters IMUParameters = new BNO055IMU.Parameters();
        IMUParameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        IMUParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        IMUParameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        IMUParameters.calibrationDataFile = "AdafruitIMUCalibration.json";

        imu.initialize(IMUParameters);
    }

    public double getCurrentHeading()
    {
        double currentHeading;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        currentHeading = angles.firstAngle;
        if (currentHeading > 0)
        {
            currentHeading = 360 - currentHeading;
        }
        else
        {
            currentHeading = -currentHeading;
        }
        return currentHeading;
    }

    public void printHeading()
    {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double currentHeading = angles.firstAngle;
        telemetry.addData("Current Heading", currentHeading);
        telemetry.update();
    }
}
