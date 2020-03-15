package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.GlobalPositions;
import org.firstinspires.ftc.teamcode.robot;
@TeleOp (name = "GyroErrorTest", group = "Tests")
public class GyroErrorTest extends LinearOpMode {

    double angle = 90;
    ElapsedTime aTime, bTime, xTime, yTime, driveTime;

    DcMotor lf, lb, rf, rb;
    BNO055IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {

        aTime = new ElapsedTime();
        bTime = new ElapsedTime();
        xTime = new ElapsedTime();
        yTime = new ElapsedTime();
        driveTime = new ElapsedTime();

        lf = hardwareMap.dcMotor.get("lf");
        lb = hardwareMap.dcMotor.get("lb");
        rf = hardwareMap.dcMotor.get("rf");
        rb = hardwareMap.dcMotor.get("rb");

        rf.setDirection(DcMotor.Direction.REVERSE);
        lb.setDirection(DcMotor.Direction.REVERSE);

        Orientation angles;



        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        imu.initialize(parameters);
        double velocityLF = 0.8, velocityLB = 0.8, velocityRF = 0.8, velocityRB = 0.8;
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addLine("Select Desired Angle to Test");
            telemetry.addData("Current Angle:", angle);


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

            telemetry.addData("lF", velocityLF);
            telemetry.addData("LB", velocityLB);
            telemetry.addData("RB", velocityRB);
            telemetry.addData("RF",velocityRF);
            telemetry.update();




            if (gamepad1.a && aTime.seconds() > 0.2)
            {
                driveTime.reset();
                while (driveTime.seconds() < 10) {
                    errorTest(angle);
                }
                aTime.reset();
            }
            if (gamepad1.b && bTime.seconds() > 0.2)
            {
                angle+=10;
                bTime.reset();
            }
            if (gamepad1.x && xTime.seconds() > 0.2)
            {
                angle -=10;
                xTime.reset();
            }
            if (gamepad1.y && yTime.seconds() > 0.2)
            {
                driveTime.reset();
                while (driveTime.seconds() < 5) {
                    gyroDrive(angle,velocityLF,velocityRF,velocityLB,velocityRB);
                }
            }


            telemetry.update();
        }

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
            lf.setPower(velocityLF + steer);
            lb.setPower(velocityLB + steer);
            rf.setPower(velocityRF - steer);
            rb.setPower(velocityRB - steer);

            doHeadingTelemetry(error, currentAngle);
        }
        else if (error >= -10 && error <=10)
        {
            steer = 0.1;
            if (error <  0)
            {
                steer = -steer;
            }
            lf.setPower(velocityLF + steer);
            lb.setPower(velocityLB + steer);
            rf.setPower(velocityRF - steer);
            rb.setPower(velocityRB - steer);

            doHeadingTelemetry(error, currentAngle);

        }
        else // El robot se paso de lanza
        {
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
        rf.setPower(velocity);
        rb.setPower(velocity);
        lf.setPower(-velocity);
        lb.setPower(-velocity);
    }

    public void turnRight(double velocity)
    {

        rf.setPower(-velocity);
        rb.setPower(-velocity);
        lf.setPower(velocity);
        lb.setPower(velocity);
    }
}
