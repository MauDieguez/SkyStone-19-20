package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.robot;

@Disabled
@Autonomous (name = "Encoder Test", group = "tests")
public class EncoderTest extends robot {

    @Override
    public void runOpMode() throws InterruptedException
    {
        robotInit();
        waitForStart();
        while (opModeIsActive())
        {

        }
    }
}
