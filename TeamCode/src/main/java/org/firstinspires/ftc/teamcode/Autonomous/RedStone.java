package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.GlobalPositions;
import org.firstinspires.ftc.teamcode.Hardware.FoundationArms;
import org.firstinspires.ftc.teamcode.Hardware.Grabber;
import org.firstinspires.ftc.teamcode.Hardware.LiftSystem;
import org.firstinspires.ftc.teamcode.robot;

import edu.spa.ftclib.internal.Global;
import edu.spa.ftclib.internal.controller.FinishableIntegratedController;
import edu.spa.ftclib.internal.drivetrain.HeadingableMecanumDrivetrain;

@Autonomous (name = "Red 1 Stone", group = "Autonomous")
public class RedStone extends robot {


    SkystoneDetectorNEW skystoneDetector;
    VuforiaLocalizer vuforia;
    String key = "Ae/9I0z/////AAABmX27I07jhEbApKSztXZKpQ8Mu1qf2/F4gUKtWNRBm7DxSmxTFQrfvZAwH/T2nuQ5na7u5AR9tZ8jPBGj1XrtqsKfLKgc+r6tPsDeK8qRKRc/OYE5UEpFGpgiDMyZS3+Uk3dGWDHgVKOL1/liT6bh1dNe6j8wxe5PB+sPIjdZAondcYdgk31IoPA5O0FLobxqtEblgNMmtNLEYKrfKzJCbzpz0K/WS71jhaIZXepGbZQleAb7Qs+Y4qOxaFUwuQ0EtXMhTlJ2RqaoKnpS2bjvG9SPRuIQZKjaVT5HWkYCXv6vwuOvXsw72Z5RdqFk5leEpYfD3ehTWX8MilffsZphH5PxjTnTk8NAcPv86AueRMWG";

    public HeadingableMecanumDrivetrain drivetrain;

    public FinishableIntegratedController controller;

    public BNO055IMU imu;

    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;

    Orientation angles;

    ElapsedTime elapsedTime;

    SkystoneDetectorNEW.skystonePos pos;

    @Override
    public void runOpMode() throws InterruptedException {

        elapsedTime = new ElapsedTime();

        DcMotor lifting = hardwareMap.dcMotor.get("arm");
        Servo foundationLeft = hardwareMap.servo.get("f1");
        Servo foundationRight = hardwareMap.servo.get("f2");
        Servo stoneGrabber = hardwareMap.servo.get("a3");
        Servo arm = hardwareMap.servo.get("a1");

        frontLeft = hardwareMap.get(DcMotor.class, "lf");
        frontRight = hardwareMap.get(DcMotor.class, "rf");
        backLeft = hardwareMap.get(DcMotor.class, "lb");
        backRight = hardwareMap.get(DcMotor.class, "rb");


        foundationLeft.setDirection(Servo.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        grabber = new Grabber(stoneGrabber, arm);
        liftSystem = new LiftSystem(lifting);
        foundationArms = new FoundationArms(foundationLeft, foundationRight);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BNO055IMU.Parameters IMUParameters = new BNO055IMU.Parameters();
        IMUParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        IMUParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        IMUParameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        IMUParameters.calibrationDataFile = "BNO055IMUCalibration.json";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(IMUParameters);


        // Init positions
        foundationArms.RIGHTARM.deactivate();
        foundationArms.LEFTARM.deactivate();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = key;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        skystoneDetector = new SkystoneDetectorNEW(vuforia);

        waitForStart();


        if (opModeIsActive()) {

            pos = skystoneDetector.vuforiaScan(false, true);
            arm.setPosition(0.15);
            stoneGrabber.setPosition(0);
            // grabber.BLOCKGRABBER.deactivate();

            switch (pos) {

                case LEFT: {
                    telemetry.addLine("LEFT");
                    telemetry.update();
                    left(0.9);
                    sleep(1000);
                    back(1);
                    sleep(500);
                    left(0.8);
                    sleep(500);
                    zero();
                    sleep(100);
                    grabber.BLOCKGRABBER.activate();
                    sleep(500);
                    arm.setPosition(GlobalPositions.ADJUST);
                    right(0.7);
                    sleep(350);
                    zero();
                    turnRight();
                    sleep(100);
                    strafe(1,4);
                    zero();
                    arm.setPosition(0.4);
                    turnLeft();
                    left(0.5);
                    sleep(500);
                    grabber.BLOCKGRABBER.deactivate();
                    sleep(500);
                    right(0.5);
                    sleep(1000);
                    arm.setPosition(GlobalPositions.ARM_DOWN);
                    turnRight();
                    strafe(-1,2);
                    zero();
                    break;
                }
                case RIGHT: {
                    telemetry.addLine("RIGHT");
                    telemetry.update();
                    left(0.9);
                    sleep(1500);
                    front(0.8);
                    sleep(200);
                    zero();
                    sleep(100);
                    grabber.BLOCKGRABBER.activate();
                    sleep(500);
                    arm.setPosition(GlobalPositions.ADJUST);
                    right(0.7);
                    sleep(300);
                    zero();
                    turnRight();
                    sleep(100);
                    left(0.8);
                    strafe(1,2);
                    // dma
                    zero();
                    grabber.BLOCKGRABBER.deactivate();
                    sleep(500);
                    arm.setPosition(GlobalPositions.ARM_DOWN);
                    strafe(-1,3);
                    right(0.5);
                    sleep(650);
                    zero();
                    turnLeft();
                    left(0.5);
                    sleep(1000);
                    zero();
                    grabber.BLOCKGRABBER.activate();
                    sleep(500);
                    arm.setPosition(GlobalPositions.ADJUST);
                    right(0.8);
                    sleep(300);
                    turnRight();
                    sleep(100);
                    left(0.8);
                    strafe(1,3);
                    zero();
                    grabber.BLOCKGRABBER.deactivate();
                    sleep(500);
                    strafe(-0.3,1);
                    zero();
                    break;
                }


                case CENTER: {
                    telemetry.addLine("CENTER");
                    telemetry.update();
                    left(0.8);
                    sleep(1500);
                    back(0.5);
                    sleep(500);
                    left(0.6);
                    sleep(100);
                    zero();
                    sleep(100);
                    grabber.BLOCKGRABBER.activate();
                    sleep(500);
                    arm.setPosition(GlobalPositions.ADJUST);
                    right(0.7);
                    sleep(300);
                    turnRight();
                    sleep(100);
                    left(0.8);
                    strafe(1,3);
                    zero();
                    grabber.BLOCKGRABBER.deactivate();
                    sleep(500);
                    arm.setPosition(GlobalPositions.ARM_DOWN);
                    strafe(-1,4);
                    right(0.35);
                    sleep(400);
                    zero();
                    turnLeft();
                    left(0.6); //
                    sleep(1000);
                    zero();
                    grabber.BLOCKGRABBER.activate();
                    sleep(800);
                    arm.setPosition(GlobalPositions.ADJUST);
                    right(0.8);
                    sleep(300);
                    turnRight();
                    sleep(100);
                    left(0.8);
                    strafe(1,4);
                    zero();
                    grabber.BLOCKGRABBER.deactivate();
                    sleep(500);
                    strafe(-1,1); //
                    zero();
                    break;
                }
                case NONE: {
                    telemetry.addLine("NONE, FIX IT! RUNNING CENTER BACKUP");
                    telemetry.update();
                    left(0.8);
                    sleep(1500);
                    back(0.5);
                    sleep(500);
                    left(0.6);
                    sleep(100);
                    zero();
                    sleep(100);
                    grabber.BLOCKGRABBER.activate();
                    sleep(500);
                    arm.setPosition(GlobalPositions.ADJUST);
                    right(0.7);
                    sleep(300);
                    turnRight();
                    sleep(100);
                    left(0.8);
                    strafe(1,3);
                    zero();
                    grabber.BLOCKGRABBER.deactivate();
                    sleep(500);
                    arm.setPosition(GlobalPositions.ARM_DOWN);
                    strafe(-1,4);
                    right(0.5);
                    sleep(400);
                    zero();
                    turnLeft();
                    left(0.5);
                    sleep(550);
                    zero();
                    grabber.BLOCKGRABBER.activate();
                    sleep(500);
                    arm.setPosition(GlobalPositions.ADJUST);
                    right(0.8);
                    sleep(300);
                    turnRight();
                    sleep(100);
                    left(0.8);
                    strafe(1,4);
                    zero();
                    grabber.BLOCKGRABBER.deactivate();
                    sleep(500);
                    strafe(-0.3,1);
                    zero();
                    break;
                }
            }

        }

    }

    void front(double vel) {
        frontLeft.setPower(vel);
        frontRight.setPower(vel);
        backLeft.setPower(vel);
        backRight.setPower(vel);
    }

    void back(double vel) {
        frontLeft.setPower(-vel);
        frontRight.setPower(-vel);
        backLeft.setPower(-vel);
        backRight.setPower(-vel);
    }

    void right(double vel) {
        frontLeft.setPower(vel);
        frontRight.setPower(-vel);
        backLeft.setPower(-vel);
        backRight.setPower(vel);
    }

    void left(double vel) {
        frontLeft.setPower(-vel);
        frontRight.setPower(vel);
        backLeft.setPower(vel);
        backRight.setPower(-vel);
    }

    void zero()
    {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
    void leftDiagonalBack(double vel)
    {
        frontLeft.setPower(-vel);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(-vel);
    }
    void leftDiagonalFront(double vel)
    {
        frontLeft.setPower(vel);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(vel);
    }
    void rightDiagonalBack(double vel)
    {
        frontLeft.setPower(0);
        frontRight.setPower(-vel);
        backLeft. setPower(-vel);
        backRight.setPower(0);
    }
    void rightDiagonalFront(double vel)
    {
        frontLeft.setPower(0);
        frontRight.setPower(vel);
        backLeft.setPower(vel);
        backRight.setPower(0);
    }

    void turnAngle(double angle)
    {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double currentHeading = angles.firstAngle;

        while (currentHeading > -angle) {

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            currentHeading = angles.firstAngle;

            frontLeft.setPower(0.6);
            frontRight.setPower(-0.6);
            backLeft.setPower(0.6);
            backRight.setPower(-0.6);

            telemetry.addData("Current Heading", currentHeading);
            telemetry.update();
        }
        while (currentHeading < -angle)
        {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            currentHeading = angles.firstAngle;

            frontLeft.setPower(-0.3);
            frontRight.setPower(0.3);
            backLeft.setPower(-0.3);
            backRight.setPower(0.3);

            telemetry.addData("Current Heading", currentHeading);
            telemetry.update();
        }

        zero();

    }

    void turnRight()
    {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double currentHeading = angles.firstAngle;

        while (currentHeading > -90) {

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            currentHeading = angles.firstAngle;

            frontLeft.setPower(0.6);
            frontRight.setPower(-0.6);
            backLeft.setPower(0.6);
            backRight.setPower(-0.6);

            telemetry.addData("Current Heading", currentHeading);
            telemetry.update();
        }
        while (currentHeading < -90)
        {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            currentHeading = angles.firstAngle;

            frontLeft.setPower(-0.3);
            frontRight.setPower(0.3);
            backLeft.setPower(-0.3);
            backRight.setPower(0.3);

            telemetry.addData("Current Heading", currentHeading);
            telemetry.update();
        }

        zero();

    }


    void strafe(double power, int time)
    {
        elapsedTime.reset();

        while (elapsedTime.seconds() < time) {
            double gain = 0.007;
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double heading = angles.firstAngle;
            heading += 180;

            double steering;

            double error = heading - 90; // 10 o -10

            steering = error * gain;

            frontLeft.setPower(-power + steering);
            backLeft.setPower(power + steering);
            frontRight.setPower(power - steering);
            backRight.setPower(-power - steering);
        }
        zero();

    }

    void turnLeft()
    {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double currentHeading = angles.firstAngle;

        while (currentHeading < 0) {

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            currentHeading = angles.firstAngle;

            frontLeft.setPower(-0.6);
            frontRight.setPower(0.6);
            backLeft.setPower(-0.6);
            backRight.setPower(0.6);

            telemetry.addData("Current Heading", currentHeading);
            telemetry.update();
        }
        while (currentHeading > 0)
        {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            currentHeading = angles.firstAngle;

            frontLeft.setPower(0.3);
            frontRight.setPower(-0.3);
            backLeft.setPower(0.3);
            backRight.setPower(-0.3);

            telemetry.addData("Current Heading", currentHeading);
            telemetry.update();
        }

        zero();

    }

}
