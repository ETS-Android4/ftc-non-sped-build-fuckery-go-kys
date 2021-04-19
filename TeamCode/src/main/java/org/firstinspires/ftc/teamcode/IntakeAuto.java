package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;

@Autonomous(name = "IntakeAutoTest", group = "nobody asked")
public class IntakeAuto extends LinearOpMode {
    DcMotor TopLeft;
    DcMotor TopRight;
    DcMotor BottomLeft;
    DcMotor BottomRight;
    DcMotor Intake;
    DcMotor Shooter;
    Servo ShooterServo;
    Servo LinearSlidesServo;
    Servo ClawServo;

    //Amp Stuff
    ExpansionHubMotor IntakeAmp;
    ExpansionHubEx expansionHub;

    //Color Sensor
    ColorSensor colorSensor;


    @Override
    public void runOpMode() throws InterruptedException {
        TopLeft = hardwareMap.dcMotor.get("topLeft");
        TopRight = hardwareMap.dcMotor.get("topRight");
        BottomLeft = hardwareMap.dcMotor.get("bottomLeft");
        BottomRight = hardwareMap.dcMotor.get("bottomRight");
        Intake = hardwareMap.dcMotor.get("intakeMotor");
        ShooterServo = hardwareMap.servo.get("test");
        ClawServo = hardwareMap.servo.get("test2");
        Shooter = hardwareMap.dcMotor.get("shooterMotor");
        LinearSlidesServo = hardwareMap.servo.get("linearSlideServo");
        TopLeft.setDirection(DcMotor.Direction.FORWARD);
        TopRight.setDirection(DcMotor.Direction.REVERSE);
        BottomLeft.setDirection(DcMotor.Direction.FORWARD);
        BottomRight.setDirection(DcMotor.Direction.REVERSE);
        LinearSlidesServo.setPosition(0.9);
        ClawServo.setPosition(0);
        ShooterServo.setPosition(.2);

        //Amp Stuff
        expansionHub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");
        IntakeAmp = (ExpansionHubMotor) hardwareMap.dcMotor.get("intakeMotor");

        //Color Sensor
        colorSensor = hardwareMap.get(ColorSensor.class, "color");

        //Checking Amperage, should be ~0.25 Amps
        telemetry.addData("Intake current", IntakeAmp.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));
        telemetry.update();

        waitForStart();
        //Remember to test code, and if its taking too long, grab 2 rings instead of 3

        Intake.setPower(1);

        for (int i = 1; i <=1; i++) {

            Intake.setPower(1);

            telemetry.addData("Intake current", IntakeAmp.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));
            telemetry.update();

            //Moves Backwards until amperage if intake motor goes high, meaning that a ring has gotten in the intake
            while (IntakeAmp.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS) < 7) {
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
            sleep(1500);

            if (IntakeAmp.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS) < 5) {
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
            while (IntakeAmp.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS) < 7) {
                TopLeft.setPower(-0.2);
                TopRight.setPower(-0.2);
                BottomRight.setPower(-0.2);
                BottomLeft.setPower(-0.2);
            }
            sleep(500);
//            sleep(500);
//            if (IntakeAmp.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS) > 7) {
////                TopLeft.setPower(0.7);
////                TopRight.setPower(0.7);
////                BottomRight.setPower(-0.7);   //strafe right
////                BottomLeft.setPower(-0.7);
////                sleep(800);
////
////                TopLeft.setPower(0);
////                TopRight.setPower(0);
////                BottomRight.setPower(0); //stop
////                BottomLeft.setPower(0);
////                sleep(500);
////
////                telemetry.addData("Intake current", IntakeAmp.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));
////                telemetry.update();
////
////                TopLeft.setPower(-0.7);
////                TopRight.setPower(-0.7);
////                BottomRight.setPower(0.7);   //strafe left
////                BottomLeft.setPower(0.7);
////                sleep(800);
//
//                if (IntakeAmp.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS) > 2.5) {
//                    TopLeft.setPower(0);
//                    TopRight.setPower(0);
//                    BottomRight.setPower(0); //stop
//                    BottomLeft.setPower(0);
//                    sleep(1000);
//                    Intake.setPower(0);
//                    TopLeft.setPower(0.2);
//                    TopRight.setPower(0.2);
//                    BottomRight.setPower(0.2);
//                    BottomLeft.setPower(0.2);
//                    sleep(1000);
//


            Intake.setPower(0);

            telemetry.addData("Intake current", IntakeAmp.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));
            telemetry.update();

            TopLeft.setPower(0);
            TopRight.setPower(0);
            BottomRight.setPower(0); //stop
            BottomLeft.setPower(0);
            sleep(2000);

        }

        Intake.setPower(0);

        //Using color sensor to align with white line
        int counter = 0;
        while (counter != 1) {
            if(colorSensor.blue() >= 2000){
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
        sleep(1000);

        TopLeft.setPower(-0.8);
        TopRight.setPower(0.8);
        BottomRight.setPower(0.8);
        BottomLeft.setPower(-0.8);    // rotate right
        sleep(60);

        TopLeft.setPower(0);
        TopRight.setPower(0);
        BottomRight.setPower(0); //stop
        BottomLeft.setPower(0);
        sleep(1000);

        TopLeft.setPower(-0.4);
        TopRight.setPower(-0.4);
        BottomRight.setPower(-0.4);
        BottomLeft.setPower(-0.4);    // rotate right
        sleep(100);

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
            if(colorSensor.blue() >= 2000){
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

//
//        //Moving backwards and getting into position to shoot
//        TopLeft.setPower(-0.9);
//        TopRight.setPower(-0.9);
//        BottomRight.setPower(-0.9);
//        BottomLeft.setPower(-0.9);
//        sleep(150);
//
//        TopLeft.setPower(0);
//        TopRight.setPower(0);
//        BottomRight.setPower(0); //stop
//        BottomLeft.setPower(0);
//        sleep(500);
//
//        //Rotate in case of misalignment
////        TopLeft.setPower(0.8);
////        TopRight.setPower(-0.8);
////        BottomRight.setPower(-0.8);
////        BottomLeft.setPower(0.8);    // rotate right
////        sleep(200);
//
//        //Shooter
//        Shooter.setPower(100);
//        sleep(2000);
//        ShooterServo.setPosition(0);
//        sleep(500);
//        ShooterServo.setPosition(.3);
//        sleep(500);
//        //.2 is base. 0 is shooter position.
//        Shooter.setPower(100);
//        sleep(2000);
//        ShooterServo.setPosition(0);
//        sleep(500);
//        ShooterServo.setPosition(.3);
//        sleep(500);
//        //2nd shot
//
//        //Powershot, test after auto works
////        TopLeft.setPower(0.8);
////        TopRight.setPower(-0.8);
////        BottomRight.setPower(-0.8);
////        BottomLeft.setPower(0.8);    // rotate right
////        sleep(310);
//
//        Shooter.setPower(100);
//        sleep(1800);
//        ShooterServo.setPosition(0);
//        sleep(500);
//        ShooterServo.setPosition(.3);
//        sleep(500);
//        // 3rd shot.
//
//        //Moving forward to line
//        while ((hsvValues[1] > 0.2) && (hsvValues[2] < 0.9)) {
//            TopLeft.setPower(0.9);
//            TopRight.setPower(0.9);
//            BottomRight.setPower(0.9);
//            BottomLeft.setPower(0.9);
//
//            telemetry.addLine()
//                    .addData("Red", "%.3f", colors.red)
//                    .addData("Green", "%.3f", colors.green)
//                    .addData("Blue", "%.3f", colors.blue);
//            telemetry.addLine()
//                    .addData("Hue", "%.3f", hsvValues[0])
//                    .addData("Saturation", "%.3f", hsvValues[1])
//                    .addData("Value", "%.3f", hsvValues[2]);
//            telemetry.addData("Alpha", "%.3f", colors.alpha);
//        }
//
//        //Move a little forward to ensure robot's over the line
//        TopLeft.setPower(0.9);
//        TopRight.setPower(0.9);
//        BottomRight.setPower(0.9);
//        BottomLeft.setPower(0.9);
//        sleep(100);
//
//        TopLeft.setPower(0);
//        TopRight.setPower(0);
//        BottomRight.setPower(0); //stop
//        BottomLeft.setPower(0);
//        sleep(500);
//


    }
}
