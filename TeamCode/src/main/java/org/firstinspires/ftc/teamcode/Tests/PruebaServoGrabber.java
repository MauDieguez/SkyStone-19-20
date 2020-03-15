package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp (name = "PruebaServosGrabber", group = "tests")
public class PruebaServoGrabber extends LinearOpMode {

    Servo servo1 = hardwareMap.servo.get("a1");
    Servo servo2 = hardwareMap.servo.get("a3");

    @Override
    public void runOpMode() throws InterruptedException {

     waitForStart();
     while (opModeIsActive())
     {
         if (gamepad1.a)
         {
             servo2.setPosition(.6);
         }
         if (gamepad1.b)
         {
             servo2.setPosition(0);
         }

         if (gamepad1.x)
         {
             servo1.setPosition(0);
         }
         if (gamepad1.y)
         {
             servo1.setPosition(1);
         }
     }

    }
}
