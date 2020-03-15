package org.firstinspires.ftc.teamcode.Drive;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.GlobalPositions;
import org.firstinspires.ftc.teamcode.Hardware.FoundationArms;
import org.firstinspires.ftc.teamcode.Hardware.Grabber;
import org.firstinspires.ftc.teamcode.Hardware.LiftSystem;

@Disabled
@TeleOp(name = "Voltrons Driver", group = "Driver")
public class VoltronsTeleOp extends LinearOpMode {

    // Vamos a declarar las variables del hardware

    // Estos son los cuatro motores principales
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;

    // Este es el brazo encargado del lifting mechanism
    DcMotor liftArm;
    LiftSystem liftSystem;

    // Este es la mano que agarra los bloques y se ajusta
    Servo blockGrabber;
    Servo blockArm;
    Grabber grabber;

    // Estos son los servos de la foundation
    Servo leftFoundationArm;
    Servo rightFoundationArm;
    FoundationArms foundationArms;

    // Tiempos para controlar los botones
    ElapsedTime buttonA;
    ElapsedTime buttonB;
    ElapsedTime buttonX;
    ElapsedTime buttonY;
    ElapsedTime g2ButtonA;

    // Variables para futuras acciones
    double reverse = 1;
    double adjust = 10;
    boolean preciseMode = false;
    boolean invert = false;
    boolean fArms = false;
    boolean open = false;

    @Override
    public void runOpMode() throws InterruptedException {

        // Iniciamos localizando el hardware en el REV
        frontLeft = hardwareMap.dcMotor.get("lf");
        frontRight = hardwareMap.dcMotor.get("rf");
        backLeft = hardwareMap.dcMotor.get("lb");
        backRight = hardwareMap.dcMotor.get("rb");
        liftArm = hardwareMap.dcMotor.get("arm");
        blockGrabber = hardwareMap.servo.get("a3");
        blockArm = hardwareMap.servo.get("a1");
        leftFoundationArm = hardwareMap.servo.get("f1");
        rightFoundationArm = hardwareMap.servo.get("f2");

        // Algunos motores estan invertidos, entonces los tenemos que invertir en el codigo
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);


        // Para evitar el momentum, hacemos que cuando no se este presionando nada, los motores hagan brake
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Inicializamos el liftSystem, el grabber y la foundationArm
        liftSystem = new LiftSystem(liftArm);
        grabber = new Grabber(blockGrabber,blockArm);
        foundationArms = new FoundationArms(leftFoundationArm, rightFoundationArm);

        // Inicializamos los tiempo
        buttonA = new ElapsedTime();
        buttonB = new ElapsedTime();
        buttonX = new ElapsedTime();
        buttonY = new ElapsedTime();
        g2ButtonA = new ElapsedTime();

        // Inicializamos posiciones para asegurarnos que todo este en su lugar.
         grabber.BLOCKGRABBER.deactivate();
         grabber.arm.setPosition(GlobalPositions.ARM_UP);
         foundationArms.LEFTARM.deactivate();
         foundationArms.RIGHTARM.deactivate();


         waitForStart();
         while (opModeIsActive())
         {
            // We start with the OPMode

             frontLeft.setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x + (gamepad1.right_stick_x * reverse)) * (adjust/10));
             frontRight.setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x - (gamepad1.right_stick_x * reverse)) * (adjust/10));
             backLeft.setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x + (gamepad1.right_stick_x * reverse)) * (adjust/10));
             backRight.setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x - (gamepad1.right_stick_x * reverse)) * (adjust/10));

             // Lets start with gamepads


             // GAMEPAD 1

             // Precise mode, fast mode
             if (gamepad1.a && buttonA.seconds() > 0.5)
             {
                 preciseMode = !preciseMode;
                 if (preciseMode)adjust = 4;
                 else adjust = 10;

                 buttonA.reset();
             }

             // Invert direction
             if (gamepad1.b && buttonB.seconds() > 0.5)
             {
                 invert = !invert;
                 if (invert)
                 {
                     frontLeft.setDirection(DcMotor.Direction.FORWARD);
                     backRight.setDirection(DcMotor.Direction.FORWARD);
                     frontRight.setDirection(DcMotor.Direction.REVERSE);
                     backLeft.setDirection(DcMotor.Direction.REVERSE);
                     reverse = 1;
                 }
                 else
                 {
                     frontLeft.setDirection(DcMotor.Direction.REVERSE);
                     backRight.setDirection(DcMotor.Direction.REVERSE);
                     frontRight.setDirection(DcMotor.Direction.FORWARD);
                     backLeft.setDirection(DcMotor.Direction.FORWARD);
                     reverse = -1;
                 }

                 buttonB.reset();
             }

             // Expand foundation servos
             if (gamepad1.x && buttonX.seconds() > 0.5)
             {
                 fArms = !fArms;
                 if (fArms)
                 {
                     foundationArms.RIGHTARM.activate();
                     foundationArms.LEFTARM.activate();
                 }
                 else
                 {
                     foundationArms.RIGHTARM.deactivate();
                     foundationArms.LEFTARM.deactivate();
                 }

                 buttonX.reset();
             }



             // GAMEPAD 2


             // Precise lifting mechanism and fast lifting mechanism
             double g2LeftStick = 0;

             if (gamepad2.dpad_up)
             {
                 g2LeftStick = 0.4;
             }
             else if (gamepad2.dpad_down)
             {
                 g2LeftStick = -0.4;
             }
             else
             {
                 g2LeftStick = -gamepad2.left_stick_y;
             }

             liftSystem.lift.setPower(g2LeftStick);


             // Open and close the claw
             if (gamepad2.a && g2ButtonA.seconds() > 0.5)
             {
                 g2ButtonA.reset();
                 open = !open;
                 if (open)
                 {
                     //  It opens
                     grabber.BLOCKGRABBER.activate();
                     sleep(1000);
                     grabber.arm.setPosition(GlobalPositions.ADJUST);
                 }
                 else
                 {
                     // It closes
                     grabber.BLOCKGRABBER.deactivate();


                 }
             }


             // Retract and expand the arm
             if (gamepad2.right_trigger > 0)
             {
                 grabber.arm.setPosition(GlobalPositions.ARM_DOWN);
             }
             if (gamepad2.left_trigger > 0)
             {
                 grabber.arm.setPosition(GlobalPositions.ARM_UP);
             }

         }
    }
}
