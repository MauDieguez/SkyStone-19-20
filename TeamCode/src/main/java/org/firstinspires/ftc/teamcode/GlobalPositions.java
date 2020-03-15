package org.firstinspires.ftc.teamcode;

public class GlobalPositions {
    public static final double ARM_UP = .8;
    public static final double ARM_DOWN = 0.15;
    public static final double ADJUST = 0.25;

    public static final double GRAB_STONE = 0.6;
    public static final double DEPLOY_STONE = 0;

    public static final double GRAB_FOUNDATION_LEFT  = 0.35;
    public static final double GRAB_FOUNDATION_RIGHT = 0.35;
    public static final double DEPLOY_FOUNDATION_LEFT = 0;
    public static final double DEPLOY_FOUNDATION_RIGHT = 0;

    public static final double COUNTS_PER_MOTOR_REV = 1425.2;
    public static final double DRIVE_GEAR_REDUCTION = 50.9;
    public static final double WHEEL_DIAMETER_CM = 10;
    public static final double COUNTS_PER_CM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_CM * Math.PI);
        // 1425.2    Pi * 25
    public static final double kP_Rotation_Fast = 0.2;
    public static final double kP_Rotation_Slow = 0.1;

    public static final double fastTurnVelocity = 0.7;
    public static final double slowTurnVelocity = 0.3;

}
