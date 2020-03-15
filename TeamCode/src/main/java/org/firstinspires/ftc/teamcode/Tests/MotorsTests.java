package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Hardware.DriveTrain;

import edu.spa.ftclib.sample.robot.BNO055HolonomicBot;

@Disabled
@TeleOp (name = "motorTest", group = "tests")
public class MotorsTests extends LinearOpMode {

    BNO055HolonomicBot drivetrain;
    @Override
    public void runOpMode() throws InterruptedException {

        drivetrain = new BNO055HolonomicBot(telemetry, hardwareMap);
        while (opModeIsActive())
        {
            drivetrain.frontLeft.setPower(1);
            sleep(5000);
            drivetrain.frontLeft.setPower(0);

            drivetrain.frontRight.setPower(1);
            sleep(5000);
            drivetrain.frontRight.setPower(0);

            drivetrain.backLeft.setPower(1);
            sleep(5000);
            drivetrain.backLeft.setPower(0);

            drivetrain.backRight.setPower(1);
            sleep(5000);
            drivetrain.backRight.setPower(0);

            sleep(5000);
        }
    }
}
