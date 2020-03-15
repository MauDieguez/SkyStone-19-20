package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.FoundationArms;
import org.firstinspires.ftc.teamcode.Hardware.Grabber;
import org.firstinspires.ftc.teamcode.Hardware.LiftSystem;
import org.firstinspires.ftc.teamcode.robot;

@Autonomous (name = "Just Park Red", group = "Autonomous")
public class JustParkRed extends robot {


    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;

    ElapsedTime elapsedTime;


    @Override
    public void runOpMode() throws InterruptedException {


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

        // Init positions
        foundationArms.RIGHTARM.deactivate();
        foundationArms.LEFTARM.deactivate();

        elapsedTime = new ElapsedTime();
        waitForStart();

        while (opModeIsActive()) {
            sleep(23000);
                front(0.3);
                sleep(200);
                left(0.5);
                sleep(1000);
                zero();
                break;
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

}
