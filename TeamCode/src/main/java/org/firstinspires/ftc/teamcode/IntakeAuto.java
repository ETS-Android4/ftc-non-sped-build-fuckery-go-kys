package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

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
    ColorSensor color;

    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    private static final String VUFORIA_KEY =
            "Abwb1Df/////AAABmRO3q8AgDUqAhwU7QR+WS7dI6p3hpQCsmemiXe+oibUizjMS3TkVaTXrjCueHyGFClD4pL6klURnCuJiEM3peaceY+uEbqjkuUqqZljr6Pe1XYIl51L+jwztIkLFpwo/5wc6dxDEe0aY4guq/uHID/fghh+kKqxTy58leUOcQaBx9XjAP7aTM8XHjIBtFHwNw28UBFnGFi6VD15Ybi5l14xu/XDemJduUIGPOpCSGQVbgdARN3MohoZSU1Xr6zBmEdrXLY9TXmF/irNTcHh80Q27u1XgdEZF61zE0EH3yDOsCCWHiL0sEZvXIU1Sx6bbmeyQffNynOPKsZ5reb6NcPEMZvPqYZCq3RAwoZBpTbF/";

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

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
        color = hardwareMap.get(ColorSensor.class, "color");

        //Checking Amperage, should be ~0.25 Amps
        telemetry.addData("Intake current", IntakeAmp.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));
        telemetry.update();

        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(2.5, 16.0/9.0);
        }

        waitForStart();
        //Remember to test code, and if its taking too long, grab 2 rings instead of 3

        while(opModeIsActive()) {

            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel() == "Single") {
                            break;
                        }

                    }

                }
            }

            tfod.shutdown();

            Intake.setPower(1);

            for (int i = 1; i <= 1; i++) {

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

                if (IntakeAmp.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS) < 2) {
                    continue;
                } else {
                    while (IntakeAmp.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS) < 5) {
                        TopLeft.setPower(0);
                        TopRight.setPower(0);
                        BottomRight.setPower(0);
                        BottomLeft.setPower(0);
                        if (IntakeAmp.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS) < 3) {
                            continue;
                        }
                    }
                }

                Intake.setPower(0);

                telemetry.addData("Intake current", IntakeAmp.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));
                telemetry.update();

            }

            Intake.setPower(0);

            //Using color sensor to align with white line
            int counter = 0;
            while (counter != 1) {
                if (color.blue() >= 2000) {
                    TopLeft.setPower(0);
                    TopRight.setPower(0);
                    BottomRight.setPower(0);
                    BottomLeft.setPower(0);
                    sleep(100);
                    counter++;
                } else {
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
                if (color.blue() >= 2000) {
                    TopLeft.setPower(0);
                    TopRight.setPower(0);
                    BottomRight.setPower(0);
                    BottomLeft.setPower(0);
                    sleep(100);
                    counter++;
                } else {
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

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}
