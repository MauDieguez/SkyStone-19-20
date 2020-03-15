package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.GlobalPositions;

import edu.spa.ftclib.internal.activator.ServoActivator;

public class FoundationArms {

    Servo leftArm;
    Servo rightArm;
    public ServoActivator LEFTARM, RIGHTARM;

    public FoundationArms(Servo leftArm, Servo rightArm)
    {
        this.leftArm = leftArm;
        this.rightArm = rightArm;
        this.LEFTARM = new ServoActivator(this.leftArm, GlobalPositions.GRAB_FOUNDATION_LEFT,GlobalPositions.DEPLOY_FOUNDATION_LEFT);
        this.RIGHTARM = new ServoActivator(this.rightArm, GlobalPositions.GRAB_FOUNDATION_RIGHT , GlobalPositions.DEPLOY_FOUNDATION_RIGHT);
    }

}
