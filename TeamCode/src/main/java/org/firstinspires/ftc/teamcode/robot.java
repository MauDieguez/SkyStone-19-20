package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.Hardware.FoundationArms;
import org.firstinspires.ftc.teamcode.Hardware.Grabber;
import org.firstinspires.ftc.teamcode.Hardware.LiftSystem;

import edu.spa.ftclib.internal.Global;
import edu.spa.ftclib.internal.controller.PIDController;
import edu.spa.ftclib.sample.robot.BNO055HolonomicBot;


abstract public class robot extends LinearOpMode {
    public DriveTrain drivetrain;
    public FoundationArms foundationArms;
    public LiftSystem liftSystem;
    public Grabber grabber;
    public BNO055HolonomicBot robot;
    public Servo arm;
    public boolean running = false;


    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;

    private ElapsedTime runtime = new ElapsedTime();

    public void robotInit()
    {

        DcMotor lifting = hardwareMap.dcMotor.get("arm");
        Servo foundationLeft = hardwareMap.servo.get("f1");
        Servo foundationRight = hardwareMap.servo.get("f2");
        Servo stoneGrabber = hardwareMap.servo.get("a3");
        arm = hardwareMap.servo.get("a1");


        frontLeft = hardwareMap.get(DcMotor.class, "lf");
        frontRight = hardwareMap.get(DcMotor.class, "rf");
        backLeft = hardwareMap.get(DcMotor.class, "lb");
        backRight = hardwareMap.get(DcMotor.class, "rb");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        grabber = new Grabber(stoneGrabber, arm);
        liftSystem = new LiftSystem(lifting);
        foundationArms = new FoundationArms(foundationLeft, foundationRight);

        // Init positions
        foundationArms.RIGHTARM.deactivate();
        foundationArms.LEFTARM.deactivate();
        // foundationArms.leftServo.activate();

        waitForStart();
    }

    /**
     *  Maneja en una direccion
        @param time el tiempo que estara manejando hacia esa direccion
        @param course la direccion a la que ira expresada en radianes. Si quiero que haga strafe hacia la derecha
        entonces mi course sera de Math.PI / 2
        @param velocity la velocidad a la que ira el robot. El maximo es 1.
     */

    public void driveTime(long time, double course, double velocity)
    {
        robot.drivetrain.setVelocity(velocity);
        robot.drivetrain.setCourse(course);
        sleep(time);
        robot.drivetrain.setVelocity(0);
    }

    /**
     * Gira de manera precisa.
     * @param angle Angulo en radianes indicando hacia donde quiero girar. Si quiero girar 90 graods
     *              hacia las manecilla del reloj, pongo Math.PI / 2
     */

    public void turnGyro(double angle)
    {
        robot.drivetrain.setTargetHeading(angle);
        while (robot.drivetrain.isRotating())
        {
            robot.drivetrain.updateHeading();
            doTelemetry();
        }
    }

    /**
     * Pone en cero todos los motores
     */

    public void setZero()
    {
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        running = false;

    }

    /**
     * Indica a los motores hacia donde se tienen que mover
     * @param leftTarget Distancia en pulgadas del lado izquierdo
     * @param rightTarget Distancia en pulgadas del lado derecho
     */

    public void setTargetPosition(int leftTarget, int rightTarget)
    {
        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + leftTarget);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() + rightTarget);
        backLeft.setTargetPosition(backLeft.getCurrentPosition() + leftTarget);
        backRight.setTargetPosition(backRight.getCurrentPosition()+ rightTarget);

        telemetry.addData("Front Left", frontLeft.getCurrentPosition() + leftTarget);
        telemetry.addData("Front Right", frontRight.getCurrentPosition() + rightTarget);
        telemetry.addData("Back Left", backLeft.getCurrentPosition() + leftTarget);
        telemetry.addData("Back Right", backRight.getCurrentPosition() + rightTarget);
        telemetry.update();
    }

    /**
     * Pone los motores en diferentes modos
     * @param mode El modo en el cual quieres que este el motor
     */
    public void setEncoderMode(DcMotor.RunMode mode)
    {
        frontLeft.setMode(mode);
        frontRight.setMode(mode);
        backLeft.setMode(mode);
        backRight.setMode(mode);
    }

    /**
     * Pone los motores a una velocidad especifica
     * @param vel La velocidad a la que quieres que vayan los motores
     */
    public void setPower(double vel, String direction)
    {
        running = true;
        switch (direction)
        {
            case "front":
            {
                frontLeft.setPower(vel);
                frontRight.setPower(vel);
                backLeft.setPower(vel);
                backRight.setPower(vel);
            }
            case "back":
            {
                frontLeft.setPower(-vel);
                frontRight.setPower(-vel);
                backLeft.setPower(-vel);
                backRight.setPower(-vel);
            }
            case "right": {
                frontLeft.setPower(vel);
                frontRight.setPower(-vel);
                backLeft.setPower(-vel);
                backRight.setPower(vel);
            }
            case "left":
            {
                frontLeft.setPower(-vel);
                frontRight.setPower(vel);
                backLeft.setPower(vel);
                backRight.setPower(-vel);
            }


        }

    }

    public void enconderDrive(double vel, double leftCM, double rightCM, double timeoutS, String direction)
    {
       // int leftDistance = (int) (leftCM * GlobalPositions.COUNTS_PER_CM);
        //int rightDistance = (int) (rightCM * GlobalPositions.COUNTS_PER_CM);

        // SI tenog 1400 ticks por revolcuion, y una revolucion es igual a una circunferencia del circulo, o se a
        // x cm, entonces cuantos ticks necesito para llegar ?


        //   78.5 Por revolucion            20 cm
        //   1400 ticks
        if (opModeIsActive())
        {
            //2000 ticks son 60 cm
            setEncoderMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            sleep(100);
            setEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);
            setTargetPosition((int) leftCM * 2000 / 60, (int) rightCM * 2000 / 60);
            sleep(100);
            setEncoderMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            setPower(Math.abs(vel), direction);

            while (opModeIsActive() && runtime.seconds() < timeoutS && frontLeft.isBusy() &&
            frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy())
            {
                telemetry.addData("Distance Target Left", frontLeft.getCurrentPosition() + (leftCM * 2000 / 60));
                telemetry.addData("Distance Target Right", frontRight.getCurrentPosition() + (rightCM * 2000 / 60));
                telemetry.addData("Left Front", frontLeft.getCurrentPosition());
                telemetry.addData("Right Front", frontRight.getCurrentPosition());
                telemetry.addData("Left Back", backLeft.getCurrentPosition());
                telemetry.addData("Right Back", backRight.getCurrentPosition());
                telemetry.update();
            }

            setZero();
            setEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sleep(250);
        }
    }

    /**
     * La funcion principal para moverse horizontalmente
     * @param desiredAngle El angulo que quieres mantener siempre
     * @param speed la velocidad a la que vas a ir
     */
    public void strafeGyro(int desiredAngle, double speed)
    {
        setEncoderMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double powerLeftFront = 0.8 * speed;
        double powerRightFront = 0.8 * speed;
        double powerLeftBack = 0.8 * speed;
        double powerRightBack = 0.8 * speed;

        double error = getError(desiredAngle);
        error = Range.clip((error * 0.1), -0.2, 0.2);

        frontLeft.setPower(powerLeftFront - error);
        backLeft.setPower(powerLeftBack - error);
        frontRight.setPower(powerRightFront + error);
        backRight.setPower(powerRightBack + error);
    }

    /**
     * Regresa el error del angulo deseado y del angulo actual
     * @param angle angulo deseado
     * @return regresa el error
     */
    public double getError(double angle)
    {
        double error;
        error = angle - getHeading();
        while (error > 180 && opModeIsActive())error -= 360;
        while (error <= -180 && opModeIsActive())error += 360;
        return error;
    }

    /**
     * Regresa el heading actual en grados
     * @return grados
     */
    public double getHeading()
    {
        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    void doTelemetry() {
        PIDController pid = (PIDController) robot.drivetrain.controller.algorithm;
        telemetry.addData("heading, target", robot.drivetrain.controller.getSensorValue()+","+pid.getTarget());
        telemetry.addData("KP", pid.getKP());
        telemetry.addData("KI", pid.getKI());
        telemetry.addData("KD", pid.getKD());
        telemetry.addData("error", pid.getError());
        telemetry.addData("integral", pid.getIntegral());
        telemetry.addData("derivative", pid.getDerivative());
        telemetry.update();
    }

    public void tel(double a, double b, double c, double d, double f, double g)
    {
        telemetry.addData("1", a);
        telemetry.addData("2", b);
        telemetry.addData("3", c);
        telemetry.addData("4", d);
        telemetry.addData("5", f);
        telemetry.addData("6", g);
        telemetry.update();

    }

    // Straight Drive functions


    public void straightDrive(double desiredAngle, double velocity)
    {
        double currentAngle = getHeading();
        double error = getError(currentAngle);
        double steer = 1;

        double leftPower = steer;
        double rightPower = -steer;

        if (error != 0) {
            frontLeft.setPower(leftPower);
            backLeft.setPower(leftPower);
            frontRight.setPower(rightPower);
            frontLeft.setPower(rightPower);
        }
        else{
            frontLeft.setPower(velocity);
            backLeft.setPower(velocity);
            frontRight.setPower(velocity);
            frontLeft.setPower(velocity);
        }
        // Ya tengo el error ahora tengo que sacar el STEER
    }



    // Seguda prueba con autonomous gyro y erroes

    // Entonces si el error esta en el rango ±3, entonces acomodamos poquito las ruedas

    public double getHeading360()
    {
        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
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

    public void gyroDrive(double desiredAngle, double velocity) // Nunca tiene que ser 1
    {
        double currentAngle = getHeading360();
        double error = getError(desiredAngle, currentAngle); // -3 -90 50
        // Ya tengo mi error en angulos

        double steer = 0;
        if (error >= -3 && error <=3) // Error chico
        {
            steer  = error * GlobalPositions.kP_Rotation_Slow;
            if (error < 0)
            {
                steer = -steer;
            }

            // Poenmos energia al robot
            frontLeft.setPower(velocity - steer);
            backLeft.setPower(velocity - steer);
            frontRight.setPower(velocity + steer);
            frontLeft.setPower(velocity + steer);

            doHeadingTelemetry(error, currentAngle);
        }
        else if (error >= -10 && error <=10)
        {
            steer = error * GlobalPositions.kP_Rotation_Fast;
            if (error <  0)
            {
                steer = -steer;
            }
            frontLeft.setPower(velocity - steer);
            backLeft.setPower(velocity - steer);
            frontRight.setPower(velocity + steer);
            frontLeft.setPower(velocity + steer);

            doHeadingTelemetry(error, currentAngle);

        }
        else // El robot se paso de lanza
        {
            if (error < desiredAngle) {
                while (error < 0) {
                    currentAngle = getHeading360();
                    error = getError(desiredAngle, currentAngle);
                    turnRight(GlobalPositions.fastTurnVelocity);
                    doHeadingTelemetry(error, currentAngle);
                }
                while (error > 0) {
                    currentAngle = getHeading360();
                    error = getError(desiredAngle, currentAngle);
                    turnLeft(GlobalPositions.slowTurnVelocity);
                    doHeadingTelemetry(error, currentAngle);
                }
                while (error < 0) {
                    currentAngle = getHeading360();
                    error = getError(desiredAngle, currentAngle);
                    turnRight(GlobalPositions.slowTurnVelocity);
                    doHeadingTelemetry(error, currentAngle);
                }
            }
            else {
                while (error > 0) {
                    currentAngle = getHeading360();
                    error = getError(desiredAngle, currentAngle);
                    turnLeft(GlobalPositions.fastTurnVelocity);
                    doHeadingTelemetry(error, currentAngle);
                }
                while (error < 0) {
                    currentAngle = getHeading360();
                    error = getError(desiredAngle, currentAngle);
                    turnRight(GlobalPositions.slowTurnVelocity);
                    doHeadingTelemetry(error, currentAngle);
                }
                while (error > 0) {
                    currentAngle = getHeading360();
                    error = getError(desiredAngle, currentAngle);
                    turnLeft(GlobalPositions.slowTurnVelocity);
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
        robot.frontRight.setPower(velocity);
        robot.backRight.setPower(velocity);
        robot.frontLeft.setPower(-velocity);
        robot.backRight.setPower(-velocity);
    }

    public void turnRight(double velocity)
    {

        robot.frontRight.setPower(-velocity);
        robot.backRight.setPower(-velocity);
        robot.frontLeft.setPower(velocity);
        robot.backRight.setPower(velocity);
    }




}
