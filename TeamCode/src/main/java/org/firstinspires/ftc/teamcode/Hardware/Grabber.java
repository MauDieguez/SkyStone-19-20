package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.GlobalPositions;

import edu.spa.ftclib.internal.activator.ServoActivator;


public class Grabber {
    Servo blockGrabber;
    public Servo arm;
    public ServoActivator BLOCKGRABBER;

    public Grabber(Servo blockGrabber, Servo arm)
    {
        this.blockGrabber = blockGrabber;
        this.arm = arm;
        this.BLOCKGRABBER = new ServoActivator(blockGrabber, GlobalPositions.GRAB_STONE, GlobalPositions.DEPLOY_STONE);
    }

}
