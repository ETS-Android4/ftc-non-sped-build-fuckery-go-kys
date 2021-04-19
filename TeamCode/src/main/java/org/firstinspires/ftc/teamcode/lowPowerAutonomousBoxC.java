package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "LowPowerAutonomousBoxC", group = "nobody asked")
public class lowPowerAutonomousBoxC extends LinearOpMode
{
    DcMotor TopLeft;
    DcMotor TopRight;
    DcMotor BottomLeft;
    DcMotor BottomRight;
    DcMotor Intake;
    DcMotor Shooter;
    Servo ShooterServo;
    Servo LinearSlidesServo;
    Servo ClawServo;

    @Override
    public void runOpMode() throws InterruptedException{
        TopLeft = hardwareMap.dcMotor.get("topLeft");
        TopRight = hardwareMap.dcMotor.get("topRight");
        BottomLeft = hardwareMap.dcMotor.get("bottomLeft");
        BottomRight = hardwareMap.dcMotor.get("bottomRight");
        Intake = hardwareMap.dcMotor.get("intakeMotor");
        ShooterServo = hardwareMap.servo.get("test");
        ClawServo = hardwareMap.servo.get("test2");
        Shooter = hardwareMap.dcMotor.get("shooterMotor");
        LinearSlidesServo = hardwareMap.servo.get("linearSlideServo");
        TopLeft.setDirection(DcMotor.Direction.REVERSE);
        TopRight.setDirection(DcMotor.Direction.FORWARD);
        BottomLeft.setDirection(DcMotor.Direction.REVERSE);
        BottomRight.setDirection(DcMotor.Direction.FORWARD);
        LinearSlidesServo.setPosition(0.9);
        ClawServo.setPosition(.1);
        ShooterServo.setPosition(.2);

        waitForStart();


        //Moving to shooter position
        TopLeft.setPower(20);
        TopRight.setPower(20);
        BottomRight.setPower(20);
        BottomLeft.setPower(20);
        sleep(1100); // go forward

        TopLeft.setPower(0);
        TopRight.setPower(0);
        BottomRight.setPower(0);
        BottomLeft.setPower(0);
        sleep(1000); // stop for a bit.
        ClawServo.setPosition(.5);

        TopLeft.setPower(-20);
        TopRight.setPower(-20);
        BottomRight.setPower(-20); // back
        BottomLeft.setPower(-20);
        sleep(1050);


        TopLeft.setPower(0);
        TopRight.setPower(0); //stop
        BottomRight.setPower(0);
        BottomLeft.setPower(0);
        sleep(100);

        TopLeft.setPower(20);
        TopRight.setPower(-20);
        BottomRight.setPower(-20); // rotate left
        BottomLeft.setPower(20);
        sleep(480);

        TopLeft.setPower(0);
        TopRight.setPower(0);
        BottomRight.setPower(0);
        BottomLeft.setPower(0); //stop
        sleep(1000);

        TopLeft.setPower(-10);
        TopRight.setPower(-10);     //rotate right
        BottomRight.setPower(10);
        BottomLeft.setPower(10);
        sleep(315);

        TopLeft.setPower(0);
        TopRight.setPower(0);
        BottomRight.setPower(0);    //stop
        BottomLeft.setPower(0);
        sleep(100);



        Shooter.setPower(100);
        sleep(1500);
        ShooterServo.setPosition(0);
        sleep(1000);
        ShooterServo.setPosition(.2);
        sleep(1000);
        //.2 is base. 0 is shooter position.
        Shooter.setPower(100);
        sleep(1500);
        ShooterServo.setPosition(0);
        sleep(1000);
        ShooterServo.setPosition(.2);
        sleep(1000);
        //2nd shot
        Shooter.setPower(100);
        sleep(1800);
        ShooterServo.setPosition(0);
        sleep(1500);
        ShooterServo.setPosition(.2);
        sleep(1000);
        // 3rd shot.

        // parking
        TopLeft.setPower(-20);
        TopRight.setPower(-20);
        BottomRight.setPower(-20);
        BottomLeft.setPower(-20);
        sleep(100);

    }
}