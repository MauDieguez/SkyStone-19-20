package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.GlobalPositions;
import org.firstinspires.ftc.teamcode.robot;

import edu.spa.ftclib.internal.Global;
import edu.spa.ftclib.internal.controller.FinishableIntegratedController;
import edu.spa.ftclib.internal.drivetrain.HeadingableMecanumDrivetrain;

@Autonomous (name = "Vuforia", group = "Autonomous")
public class Blue2Stones2 extends robot {

    SkystoneDetectorNEW skystoneDetector;
    VuforiaLocalizer vuforia;
    String key= "Ae/9I0z/////AAABmX27I07jhEbApKSztXZKpQ8Mu1qf2/F4gUKtWNRBm7DxSmxTFQrfvZAwH/T2nuQ5na7u5AR9tZ8jPBGj1XrtqsKfLKgc+r6tPsDeK8qRKRc/OYE5UEpFGpgiDMyZS3+Uk3dGWDHgVKOL1/liT6bh1dNe6j8wxe5PB+sPIjdZAondcYdgk31IoPA5O0FLobxqtEblgNMmtNLEYKrfKzJCbzpz0K/WS71jhaIZXepGbZQleAb7Qs+Y4qOxaFUwuQ0EtXMhTlJ2RqaoKnpS2bjvG9SPRuIQZKjaVT5HWkYCXv6vwuOvXsw72Z5RdqFk5leEpYfD3ehTWX8MilffsZphH5PxjTnTk8NAcPv86AueRMWG";

    public HeadingableMecanumDrivetrain drivetrain;

    public FinishableIntegratedController controller;

    public BNO055IMUImpl imu;

    SkystoneDetectorNEW.skystonePos pos;

    @Override
    public void runOpMode() throws InterruptedException
    {

        robotInit();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = key;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        skystoneDetector = new SkystoneDetectorNEW(vuforia);

        pos = skystoneDetector.vuforiaScan(true,true);


        waitForStart();

        if (opModeIsActive()) {
            arm.setPosition(GlobalPositions.ARM_DOWN);
            grabber.BLOCKGRABBER.deactivate();
            pos = SkystoneDetectorNEW.skystonePos.RIGHT;

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
                    setPower(0.8,"left");
                    sleep(1300);
                    setZero();

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
                sleep(2000);
            }

        }

    }
}
