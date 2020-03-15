package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.robot;
import org.firstinspires.ftc.teamcode.Autonomous.SkystoneDetector;
import org.firstinspires.ftc.teamcode.GlobalPositions;

import edu.spa.ftclib.internal.controller.ErrorTimeThresholdFinishingAlgorithm;
import edu.spa.ftclib.internal.controller.FinishableIntegratedController;
import edu.spa.ftclib.internal.controller.PIDController;
import edu.spa.ftclib.internal.drivetrain.HeadingableMecanumDrivetrain;
import edu.spa.ftclib.internal.sensor.IntegratingGyroscopeSensor;
@Disabled
@Autonomous (name = "Blue Auto 2 Stone", group = "Autonomous")
public class Blue2Stones extends robot {

    SkystoneDetector skystoneDetector;
    VuforiaLocalizer vuforia;
    String key= "Ae/9I0z/////AAABmX27I07jhEbApKSztXZKpQ8Mu1qf2/F4gUKtWNRBm7DxSmxTFQrfvZAwH/T2nuQ5na7u5AR9tZ8jPBGj1XrtqsKfLKgc+r6tPsDeK8qRKRc/OYE5UEpFGpgiDMyZS3+Uk3dGWDHgVKOL1/liT6bh1dNe6j8wxe5PB+sPIjdZAondcYdgk31IoPA5O0FLobxqtEblgNMmtNLEYKrfKzJCbzpz0K/WS71jhaIZXepGbZQleAb7Qs+Y4qOxaFUwuQ0EtXMhTlJ2RqaoKnpS2bjvG9SPRuIQZKjaVT5HWkYCXv6vwuOvXsw72Z5RdqFk5leEpYfD3ehTWX8MilffsZphH5PxjTnTk8NAcPv86AueRMWG";

    public HeadingableMecanumDrivetrain drivetrain;

    public FinishableIntegratedController controller;

    public BNO055IMUImpl imu;

    SkystoneDetector.skystonePos pos;

    @Override
    public void runOpMode() throws InterruptedException
    {

        robotInit();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = key;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        skystoneDetector = new SkystoneDetector(vuforia);

        pos = skystoneDetector.vuforiaScan(true,true);

        waitForStart();

        while (opModeIsActive()) {
            switch (pos)
            {
                case LEFT:
                {
                    telemetry.addLine("LEFT");
                    telemetry.update();
                    break;
                }
                case RIGHT:
                {
                    telemetry.addLine("RIGHT");
                    telemetry.update();
                    break;
                }
                case CENTER:
                {
                    telemetry.addLine("CENTER");
                    telemetry.update();
                    break;
                }
                case NONE:
                {
                    telemetry.addLine("NONE");
                    telemetry.update();
                }
                sleep(500);
            }
        }

    }
}
