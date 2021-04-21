package org.firstinspires.ftc.teamcode;

//import android.view.View;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import java.lang.*;
// import com.qualcomm.robotcore.eventloop.opmode.OpMode;
// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
// import com.qualcomm.robotcore.hardware.DistanceSensor;
// import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
// import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.*;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;

@Autonomous(name = "BoxBAuto", group = "Sensor")

public class AutoBoxB extends LinearOpMode {
    ColorSensor color;
    BNO055IMU imu;
    boolean check = false;
    DcMotor TopRight;
    Orientation angles;
    DcMotor TopLeft;
    DcMotor BottomRight;
    DcMotor BottomLeft;
    DcMotor Intake;
    DcMotor Shooter;
    Servo ShooterServo;
    Servo LinearSlidesServo;
    Servo ClawServo;
    int counter = 0;
    //Amp Stuff
    ExpansionHubMotor IntakeAmp;
    ExpansionHubEx expansionHub;

    public void turn(double turnAngle, double timeoutS){
        sleep(250);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double speed = 0.5;
        double oldDegreesLeft = turnAngle;
        double scaledSpeed = speed;
        double targetHeading = angles.firstAngle + turnAngle;
        double oldAngle = angles.firstAngle;
        if(targetHeading < -180){targetHeading += 360;}
        if(targetHeading >180){targetHeading -= 360;}
        double degreesLeft = ((Math.signum(angles.firstAngle - targetHeading) + 1)/2) * (360 - Math.abs(angles.firstAngle - targetHeading)) + (Math.signum(targetHeading - angles.firstAngle) + 1) / 2 * Math.abs(angles.firstAngle - targetHeading);
        resetStartTime();
        while(opModeIsActive() && time < timeoutS && degreesLeft > 1 && oldDegreesLeft - degreesLeft >= 0){
            scaledSpeed = degreesLeft/(100 + degreesLeft) * speed;
            if(scaledSpeed > 1){scaledSpeed = .1;}
            TopLeft.setPower(scaledSpeed);
            TopRight.setPower(-scaledSpeed);
            BottomLeft.setPower(scaledSpeed);
            BottomRight.setPower(-scaledSpeed);
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            oldDegreesLeft = degreesLeft;
            degreesLeft = ((Math.signum(angles.firstAngle - targetHeading) + 1)/2) * (360 - Math.abs(angles.firstAngle - targetHeading)) + (Math.signum(targetHeading - angles.firstAngle) + 1) / 2 * Math.abs(angles.firstAngle - targetHeading);
            if(Math.abs(angles.firstAngle - oldAngle)<1){speed *= 1.1;}
            oldAngle = angles.firstAngle;
        }
    }
    public void runOpMode() throws InterruptedException{
        //Amp Stuff
        expansionHub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");
        IntakeAmp = (ExpansionHubMotor) hardwareMap.dcMotor.get("intakeMotor");


        color = hardwareMap.get(ColorSensor.class,"color");

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

        ClawServo.setPosition(.1);
        LinearSlidesServo.setPosition(0.9);
        ShooterServo.setPosition(.2);
        float gain = 12;
        //color.setGain(gain);
        waitForStart();
        // TopRight.setPower(.4);
        // TopLeft.setPower(.4);
        // BottomRight.setPower(.32);
        // BottomLeft.setPower(.32);
        // sleep(300);

        // TopRight.setPower(0);
        // TopLeft.setPower(0);
        // BottomRight.setPower(0);
        // BottomLeft.setPower(0);
        // sleep(200);
        while(counter!= 1){
            if(color.blue() >= 2000){
                TopRight.setPower(0);
                TopLeft.setPower(0);
                BottomRight.setPower(0);
                BottomLeft.setPower(0);
                sleep(50);
                check = true;
                counter++;
            }
            else{
                TopLeft.setPower(.6);
                TopRight.setPower(.6);
                BottomLeft.setPower(.6);
                BottomRight.setPower(.6);
                sleep(1);
            }
            telemetry.addData("boolean check: ",check);

        }
        telemetry.addLine("hi");
        telemetry.addData("counter:",counter);
        telemetry.update();

        TopRight.setPower(0);
        TopLeft.setPower(0);
        BottomRight.setPower(0);
        BottomLeft.setPower(0);
        sleep(500);

        TopRight.setPower(.10);
        TopLeft.setPower(.10);
        BottomRight.setPower(.10);
        BottomLeft.setPower(.10);
        //ClawServo.setPosition(.5);

        TopRight.setPower(0);
        TopLeft.setPower(0);
        BottomRight.setPower(0);
        BottomLeft.setPower(0);
        sleep(500);

        TopLeft.setPower(0.8);
        TopRight.setPower(0.8);
        BottomRight.setPower(-0.8);
        BottomLeft.setPower(-0.8);
        sleep(400);


        TopLeft.setPower(0);
        TopRight.setPower(0);
        BottomRight.setPower(0);
        BottomLeft.setPower(0);
        sleep(1000);
        counter = 0;

        TopLeft.setPower(-.5);
        TopRight.setPower(-.5);
        BottomLeft.setPower(-.5);
        BottomRight.setPower(-.5);
        sleep(70);

        TopLeft.setPower(0);
        TopRight.setPower(0);
        BottomRight.setPower(0);
        BottomLeft.setPower(0);
        sleep(300);

        ClawServo.setPosition(.5);
        sleep(250);

//        TopLeft.setPower(0);
//        TopRight.setPower(0);
//        BottomRight.setPower(0);
//        BottomLeft.setPower(0);
//        sleep(500);

        TopLeft.setPower(-.5);
        TopRight.setPower(-.5);
        BottomLeft.setPower(-.5);
        BottomRight.setPower(-.5);
        sleep(600);

        TopLeft.setPower(0);
        TopRight.setPower(0);
        BottomRight.setPower(0);
        BottomLeft.setPower(0);
        sleep(1000);

        TopRight.setPower(-20);
        TopLeft.setPower(20);
        BottomLeft.setPower(20);
        BottomRight.setPower(-20);
        sleep(350);

        TopLeft.setPower(0);
        TopRight.setPower(0);
        BottomRight.setPower(0);
        BottomLeft.setPower(0);
        sleep(500);

        Shooter.setPower(100);
        sleep(1500);
        ShooterServo.setPosition(0);
        sleep(500);
        ShooterServo.setPosition(.3);
        sleep(500);
        //.2 is base. 0 is shooter position.
        Shooter.setPower(100);
        sleep(1500);
        ShooterServo.setPosition(0);
        sleep(500);
        ShooterServo.setPosition(.3);
        sleep(500);
        //2nd shot
        Shooter.setPower(100);
        sleep(1800);
        ShooterServo.setPosition(0);
        sleep(500);
        ShooterServo.setPosition(.2);
        sleep(1000);
        // 3rd shot.


        //INTAKE STUFFFFFF
        TopLeft.setDirection(DcMotor.Direction.FORWARD);
        TopRight.setDirection(DcMotor.Direction.REVERSE);
        BottomLeft.setDirection(DcMotor.Direction.FORWARD);
        BottomRight.setDirection(DcMotor.Direction.REVERSE);
        sleep(500);

        Intake.setPower(1);

        for (int i = 1; i <=1; i++) {

            Intake.setPower(1);

            telemetry.addData("Intake current", IntakeAmp.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));
            telemetry.update();

            //Moves Backwards until amperage if intake motor goes high, meaning that a ring has gotten in the intake
            while (IntakeAmp.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS) < 6) {
                TopLeft.setPower(-0.3);
                TopRight.setPower(-0.3);
                BottomRight.setPower(-0.3);
                BottomLeft.setPower(-0.3);

                telemetry.addData("Intake current", IntakeAmp.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));
                telemetry.update();
            }

            telemetry.addData("Intake current", IntakeAmp.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));
            telemetry.update();

            TopLeft.setPower(0);
            TopRight.setPower(0);
            BottomRight.setPower(0); //stop
            BottomLeft.setPower(0);
            sleep(1000);

            if (IntakeAmp.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS) < 3) {
                continue;
            }

            Intake.setPower(0);

            TopLeft.setPower(0.4);
            TopRight.setPower(0.4);
            BottomRight.setPower(0.4);
            BottomLeft.setPower(0.4);
            sleep(200);

            TopLeft.setPower(0);
            TopRight.setPower(0);
            BottomRight.setPower(0); //stop
            BottomLeft.setPower(0);
            sleep(500);

            Intake.setPower(1);
            sleep(500);

            //If the ring is still stuck in the intake for 2 seconds, we strafe right to get it out
            while (IntakeAmp.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS) < 6) {
                TopLeft.setPower(-0.2);
                TopRight.setPower(-0.2);
                BottomRight.setPower(-0.2);
                BottomLeft.setPower(-0.2);
                if (IntakeAmp.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS) < 6) {
                    continue;
                }
            }
            sleep(500);

            Intake.setPower(0);

            telemetry.addData("Intake current", IntakeAmp.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));
            telemetry.update();

            TopLeft.setPower(0);
            TopRight.setPower(0);
            BottomRight.setPower(0); //stop
            BottomLeft.setPower(0);
            sleep(1000);

        }

        Intake.setPower(0);

        //Using color sensor to align with white line
        int counter = 0;
        while (counter != 1) {
            if(color.blue() >= 2000){
                TopLeft.setPower(0);
                TopRight.setPower(0);
                BottomRight.setPower(0);
                BottomLeft.setPower(0);
                sleep(100);
                counter++;
            }
            else{
                TopLeft.setPower(0.3);
                TopRight.setPower(0.3);
                BottomRight.setPower(0.3);
                BottomLeft.setPower(0.3);
                sleep(1);
            }
            ;
        }

        TopLeft.setPower(0);
        TopRight.setPower(0);
        BottomRight.setPower(0); //stop
        BottomLeft.setPower(0);
        sleep(500);

        TopLeft.setPower(-0.8);
        TopRight.setPower(0.8);
        BottomRight.setPower(0.8);
        BottomLeft.setPower(-0.8);    // rotate right
        sleep(50);

        TopLeft.setPower(0);
        TopRight.setPower(0);
        BottomRight.setPower(0); //stop
        BottomLeft.setPower(0);
        sleep(500);

        TopLeft.setPower(-0.4);
        TopRight.setPower(-0.4);
        BottomRight.setPower(-0.4);
        BottomLeft.setPower(-0.4);    // rotate right
        sleep(160);

        TopLeft.setPower(0);
        TopRight.setPower(0);
        BottomRight.setPower(0); //stop
        BottomLeft.setPower(0);
        sleep(500);

        Shooter.setPower(100);
        sleep(2000);
        ShooterServo.setPosition(0);
        sleep(500);
        ShooterServo.setPosition(.3);
        sleep(1000);
        //.2 is base. 0 is shooter position.

        counter = 0;
        while (counter != 1) {
            if(color.blue() >= 2000){
                TopLeft.setPower(0);
                TopRight.setPower(0);
                BottomRight.setPower(0);
                BottomLeft.setPower(0);
                sleep(100);
                counter++;
            }
            else{
                TopLeft.setPower(-0.3);
                TopRight.setPower(-0.3);
                BottomRight.setPower(-0.3);
                BottomLeft.setPower(-0.3);
                sleep(1);
            }
            ;
        }

        TopLeft.setPower(0.3);
        TopRight.setPower(0.3);
        BottomRight.setPower(0.3);
        BottomLeft.setPower(0.3);
        sleep(100);

        TopLeft.setPower(0);
        TopRight.setPower(0);
        BottomRight.setPower(0); //stop
        BottomLeft.setPower(0);
        sleep(1000);



    }
}