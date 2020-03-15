package org.firstinspires.ftc.teamcode.Tests;

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
import org.firstinspires.ftc.teamcode.Autonomous.SkystoneDetectorNEW;
import org.firstinspires.ftc.teamcode.GlobalPositions;
import org.firstinspires.ftc.teamcode.Hardware.FoundationArms;
import org.firstinspires.ftc.teamcode.Hardware.Grabber;
import org.firstinspires.ftc.teamcode.Hardware.LiftSystem;
import org.firstinspires.ftc.teamcode.robot;

import edu.spa.ftclib.internal.controller.FinishableIntegratedController;
import edu.spa.ftclib.internal.drivetrain.HeadingableMecanumDrivetrain;

@Autonomous (name = "Correctness Test Blue", group = "Autonomous")
public class CorrectnessTest extends robot {


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
    ElapsedTime gyroDrive;
    ElapsedTime correctTime;

    double driveTime;

    SkystoneDetectorNEW.skystonePos pos;

    @Override
    public void runOpMode() throws InterruptedException {

        elapsedTime = new ElapsedTime();
        gyroDrive = new ElapsedTime();
        correctTime = new ElapsedTime();

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

        telemetry.addLine("Gyro is Calibrating!");
        telemetry.update();
        while (!imu.isGyroCalibrated()) {}
        telemetry.addLine("Gyro ready!");
        telemetry.update();

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
                    turnLeft();
                    sleep(100);
                    //
                    gyroDrive.reset();
                    driveTime = 2000;
                    while (gyroDrive.milliseconds() < driveTime) {
                        gyroDrive(270, -0.8, 0.8, 0.8, -0.8);
                    }
                    zero();
                    grabber.BLOCKGRABBER.deactivate();
                    sleep(500);
                    arm.setPosition(GlobalPositions.ARM_DOWN);
                    gyroDrive.reset();
                    driveTime = 3300;
                    while (gyroDrive.milliseconds() < driveTime) {
                        gyroDrive(270, 0.8, -0.8, -0.8, 0.8);
                    }
                    zero();
                    turnRight();
                    left(0.5);
                    sleep(500);
                    zero();
                    grabber.BLOCKGRABBER.activate();
                    sleep(500);
                    arm.setPosition(GlobalPositions.ADJUST);
                    right(0.7);
                    sleep(300);
                    turnLeft();
                    sleep(100);
                    gyroDrive.reset();
                    driveTime = 3600;
                    while (gyroDrive.milliseconds() < driveTime) {
                        gyroDrive(270, -0.8, 0.8, 0.8, -0.8);
                    }
                    zero();
                    grabber.BLOCKGRABBER.deactivate();
                    sleep(500);
                    gyroDrive.reset();
                    driveTime = 800;
                    while (gyroDrive.milliseconds() < driveTime) {
                        gyroDrive(270, 0.8, -0.8, -0.8, 0.8);
                    }
                    zero();
                    break;
                }
                case RIGHT: {
                    telemetry.addLine("RIGHT");
                    telemetry.update();
                    left(0.9);
                    sleep(1500);
                    front(0.8);
                    zero();
                    sleep(100);
                    grabber.BLOCKGRABBER.activate();
                    sleep(500);
                    arm.setPosition(GlobalPositions.ADJUST);
                    right(0.7);
                    sleep(300);
                    zero();
                    turnLeft();
                    gyroDrive.reset();
                    driveTime = 3300;
                    while (gyroDrive.milliseconds() < driveTime) {
                        gyroDrive(270, -0.8, 0.8, 0.8, -0.8);
                    }
                    zero();
                    grabber.BLOCKGRABBER.deactivate();
                    sleep(500);
                    gyroDrive.reset();
                    driveTime = 800;
                    while (gyroDrive.milliseconds() < driveTime) {
                        gyroDrive(270, 0.8, -0.8, -0.8, 0.8);
                    }
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
                    turnLeft();
                    sleep(100);
                    //
                    gyroDrive.reset();
                    driveTime = 2700;
                    while (gyroDrive.milliseconds() < driveTime) {
                        gyroDrive(270, -0.8, 0.8, 0.8, -0.8);
                    }
                    zero();
                    grabber.BLOCKGRABBER.deactivate();
                    sleep(500);
                    arm.setPosition(GlobalPositions.ARM_DOWN);
                    gyroDrive.reset();
                    driveTime = 4000;
                    while (gyroDrive.milliseconds() < driveTime) {
                        gyroDrive(270, 0.8, -0.8, -0.8, 0.8);
                    }
                    zero();
                    turnRight();
                    left(0.5);
                    sleep(500);
                    zero();
                    grabber.BLOCKGRABBER.activate();
                    sleep(500);
                    arm.setPosition(GlobalPositions.ADJUST);
                    right(0.7);
                    sleep(300);
                    turnLeft();
                    sleep(100);
                    gyroDrive.reset();
                    driveTime = 4000;
                    while (gyroDrive.milliseconds() < driveTime) {
                        gyroDrive(270, -0.8, 0.8, 0.8, -0.8);
                    }
                    zero();
                    grabber.BLOCKGRABBER.deactivate();
                    sleep(500);
                    gyroDrive.reset();
                    driveTime = 650;
                    while (gyroDrive.milliseconds() < driveTime) {
                        gyroDrive(270, 0.8, -0.8, -0.8, 0.8);
                    }
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
                    turnLeft();
                    sleep(100);
                    //
                    gyroDrive.reset();
                    driveTime = 2700;
                    while (gyroDrive.milliseconds() < driveTime) {
                        gyroDrive(270, -0.8, 0.8, 0.8, -0.8);
                    }
                    zero();
                    grabber.BLOCKGRABBER.deactivate();
                    sleep(500);
                    arm.setPosition(GlobalPositions.ARM_DOWN);
                    gyroDrive.reset();
                    driveTime = 4000;
                    while (gyroDrive.milliseconds() < driveTime) {
                        gyroDrive(270, 0.8, -0.8, -0.8, 0.8);
                    }
                    zero();
                    turnRight();
                    left(0.5);
                    sleep(500);
                    zero();
                    grabber.BLOCKGRABBER.activate();
                    sleep(500);
                    arm.setPosition(GlobalPositions.ADJUST);
                    right(0.7);
                    sleep(300);
                    turnLeft();
                    sleep(100);
                    gyroDrive.reset();
                    driveTime = 4000;
                    while (gyroDrive.milliseconds() < driveTime) {
                        gyroDrive(270, -0.8, 0.8, 0.8, -0.8);
                    }
                    zero();
                    grabber.BLOCKGRABBER.deactivate();
                    sleep(500);
                    gyroDrive.reset();
                    driveTime = 650;
                    while (gyroDrive.milliseconds() < driveTime) {
                        gyroDrive(270, 0.8, -0.8, -0.8, 0.8);
                    }
                    zero();
                    break;
                    //
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

    void turnRight()
    {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double currentHeading = angles.firstAngle;

        while (currentHeading > 0) {

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            currentHeading = angles.firstAngle;

            frontLeft.setPower(0.6);
            frontRight.setPower(-0.6);
            backLeft.setPower(0.6);
            backRight.setPower(-0.6);

            telemetry.addData("Current Heading", currentHeading);
            telemetry.update();
        }
        while (currentHeading < 0)
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

            double error = heading - 180; // 10 o -10

            steering = error * gain;

            frontLeft.setPower(power + steering);
            backLeft.setPower(-power + steering);
            frontRight.setPower(-power - steering);
            backRight.setPower(power - steering);

            telemetry.addData("Heading", heading);
            telemetry.addData("OBJ", 180);
            telemetry.addData("Error", error);
            telemetry.addData("Steering", steering);
            telemetry.update();

        }
        zero();

    }

    void turnLeft()
    {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double currentHeading = angles.firstAngle;

        while (currentHeading < 90) {

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            currentHeading = angles.firstAngle;

            frontLeft.setPower(-0.6);
            frontRight.setPower(0.6);
            backLeft.setPower(-0.6);
            backRight.setPower(0.6);

            telemetry.addData("Current Heading", currentHeading);
            telemetry.update();
        }
        while (currentHeading > 90)
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


    public double getHeading360()
    {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double heading = angles.firstAngle;
        heading+=180;
        return heading;
    }

    public double getError(double desiredAngle, double currentAngle)
    {
        double error = currentAngle - desiredAngle;
        if (error > 180)
        {
            error -= 360;
        }
        return error;
    }

    public void gyroDrive(double desiredAngle, double velocityLF, double velocityRF, double velocityLB, double velocityRB) // Nunca tiene que ser 1
    {
        double currentAngle = Math.round(getHeading360());
        double error = Math.round(getError(desiredAngle, currentAngle)); // -3 -90 50
        // Ya tengo mi error en angulos
        double steer = 0;
        if (error >= -3 && error <=3) // Error chico
        {
            steer  = 0.05;
            if (error < 0)
            {
                steer = -steer;
            }

            // Poenmos energia al robot
            frontLeft.setPower(velocityLF + steer);
            backLeft.setPower(velocityLB + steer);
            frontRight.setPower(velocityRF - steer);
            backRight.setPower(velocityRB - steer);

            doHeadingTelemetry(error, currentAngle);
        }
        else if (error >= -10 && error <=10)
        {
            steer = 0.1;
            if (error <  0)
            {
                steer = -steer;
            }
            frontLeft.setPower(velocityLF + steer);
            backLeft.setPower(velocityLB + steer);
            frontRight.setPower(velocityRF - steer);
            backRight.setPower(velocityRB - steer);

            doHeadingTelemetry(error, currentAngle);

        }
        else // El robot se paso de lanza
        {
            correctTime.reset();
            if (error < desiredAngle) {
                while (error < 0) {
                    currentAngle = getHeading360();
                    error = getError(desiredAngle, currentAngle);
                    turnLeft(GlobalPositions.fastTurnVelocity);
                    doHeadingTelemetry(error, currentAngle);
                }
                while (error > 0) {
                    currentAngle = getHeading360();
                    error = getError(desiredAngle, currentAngle);
                    turnRight(GlobalPositions.slowTurnVelocity);
                    doHeadingTelemetry(error, currentAngle);
                }
                while (error < 0) {
                    currentAngle = getHeading360();
                    error = getError(desiredAngle, currentAngle);
                    turnLeft(GlobalPositions.slowTurnVelocity);
                    doHeadingTelemetry(error, currentAngle);
                }
            }
            else {
                while (error > 0) {
                    currentAngle = getHeading360();
                    error = getError(desiredAngle, currentAngle);
                    turnRight(GlobalPositions.fastTurnVelocity);
                    doHeadingTelemetry(error, currentAngle);
                }
                while (error < 0) {
                    currentAngle = getHeading360();
                    error = getError(desiredAngle, currentAngle);
                    turnLeft(GlobalPositions.slowTurnVelocity);
                    doHeadingTelemetry(error, currentAngle);
                }
                while (error > 0) {
                    currentAngle = getHeading360();
                    error = getError(desiredAngle, currentAngle);
                    turnRight(GlobalPositions.slowTurnVelocity);
                    doHeadingTelemetry(error, currentAngle);
                }
            }

            driveTime = driveTime + correctTime.milliseconds();
        }





        /*
        // Definimos nuestros rangos
        double minorRangeStart = desiredAngle - 3;
        double minorRangeEnd = desiredAngle + 3;

        if (minorRangeStart < 0)minorRangeStart  = 360 - Math.abs(minorRangeStart);
        if (minorRangeEnd > 360)minorRangeEnd = minorRangeEnd - 360;

        double largerRangeStart = desiredAngle - 10;
        double largerRangeEnd = desiredAngle + 10;

        if (largerRangeStart < 0)largerRangeStart = 360 - Math.abs(largerRangeStart);
        if (largerRangeEnd > 360)largerRangeEnd = largerRangeEnd - 360;

         */



    }

    public void doHeadingTelemetry(double error, double heading)
    {
        telemetry.addData("Heading:",heading);
        telemetry.addData("Error:", error);
        telemetry.update();
    }


    public void errorTest(double desiredAngle)
    {
        double heading = getHeading360();
        double error = getError(desiredAngle, heading);

        telemetry.addData("Heading:",heading);
        telemetry.addData("Error:", error);
        telemetry.update();
    }

    public void turnLeft(double velocity)
    {
        frontRight.setPower(velocity);
        backRight.setPower(velocity);
        frontLeft.setPower(-velocity);
        backLeft.setPower(-velocity);
    }

    public void turnRight(double velocity)
    {

        frontRight.setPower(-velocity);
        backRight.setPower(-velocity);
        frontLeft.setPower(velocity);
        backLeft.setPower(velocity);
    }
}


/*
            if (gamepad1.dpad_up)
            {
                velocityLF = 0.8;
                velocityLB = 0.8;
                velocityRB = 0.8;
                velocityRF = 0.8;
            }
            if (gamepad1.dpad_down)
            {
                velocityLF = -0.8;
                velocityLB = -0.8;
                velocityRB = -0.8;
                velocityRF = -0.8;
            }
            if (gamepad1.dpad_right)
            {
                velocityLF = 0.8;
                velocityLB = -0.8;
                velocityRB = 0.8;
                velocityRF = -0.8;
            }
            if (gamepad1.dpad_left)
            {
                velocityLF = -0.8;
                velocityLB = 0.8;
                velocityRB = -0.8;
                velocityRF = 0.8;
            }
 */
