package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;

public class LiftSystem {
    public DcMotor lift;

    public LiftSystem(DcMotor lift)
    {
        lift.setDirection(DcMotor.Direction.REVERSE);

        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.lift = lift;
    }
}
