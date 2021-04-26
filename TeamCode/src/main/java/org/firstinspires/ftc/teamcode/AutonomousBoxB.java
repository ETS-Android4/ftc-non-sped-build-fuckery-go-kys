package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;


// import com.qualcomm.robotcore.eventloop.opmode.OpMode;
// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
// import com.qualcomm.robotcore.hardware.DistanceSensor;
// import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
// import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.*;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;

@Autonomous(name = "FINAL AUTO", group = "Sensor")

public class AutonomousBoxB extends LinearOpMode {
    ColorSensor color;
    boolean check = false;
    DcMotor TopRight;
    DcMotor TopLeft;
    BNO055IMU imu;
    Orientation angles;
    DcMotor BottomRight;
    DcMotor BottomLeft;
    DcMotor Intake;
    DcMotorEx Shooter;
    Servo ShooterServo;
    Servo LinearSlidesServo;
    Servo ClawServo;
    int counter = 0;

    Orientation             lastAngles = new Orientation();
    double                  globalAngle, correction;

    //Amp Stuff
    ExpansionHubMotor IntakeAmp;
    ExpansionHubEx expansionHub;

    //Vuforia
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    private static final String VUFORIA_KEY =
            "Abwb1Df/////AAABmRO3q8AgDUqAhwU7QR+WS7dI6p3hpQCsmemiXe+oibUizjMS3TkVaTXrjCueHyGFClD4pL6klURnCuJiEM3peaceY+uEbqjkuUqqZljr6Pe1XYIl51L+jwztIkLFpwo/5wc6dxDEe0aY4guq/uHID/fghh+kKqxTy58leUOcQaBx9XjAP7aTM8XHjIBtFHwNw28UBFnGFi6VD15Ybi5l14xu/XDemJduUIGPOpCSGQVbgdARN3MohoZSU1Xr6zBmEdrXLY9TXmF/irNTcHh80Q27u1XgdEZF61zE0EH3yDOsCCWHiL0sEZvXIU1Sx6bbmeyQffNynOPKsZ5reb6NcPEMZvPqYZCq3RAwoZBpTbF/";

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;



    public void runOpMode() throws InterruptedException{
        //Amp Stuff
        expansionHub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");
        IntakeAmp = (ExpansionHubMotor) hardwareMap.dcMotor.get("intakeMotor");


        color = hardwareMap.get(ColorSensor.class,"color");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu 1");
        imu.initialize(parameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        //Make sure Vuforia is initialized
        initVuforia();
        initTfod();



        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated() && tfod != null)
        {
            if (tfod != null) {
                tfod.activate();
                telemetry.addLine("Vuforia Init");
                telemetry.update();
            }
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();

        TopLeft = hardwareMap.dcMotor.get("topLeft");
        TopRight = hardwareMap.dcMotor.get("topRight");
        BottomLeft = hardwareMap.dcMotor.get("bottomLeft");
        BottomRight = hardwareMap.dcMotor.get("bottomRight");
        Intake = hardwareMap.dcMotor.get("intakeMotor");

        ShooterServo = hardwareMap.servo.get("test");
        ClawServo = hardwareMap.servo.get("test2");
        Shooter = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        LinearSlidesServo = hardwareMap.servo.get("linearSlideServo");

        TopLeft.setDirection(DcMotor.Direction.REVERSE);
        TopRight.setDirection(DcMotor.Direction.FORWARD);
        BottomLeft.setDirection(DcMotor.Direction.REVERSE);
        BottomRight.setDirection(DcMotor.Direction.FORWARD);

        ClawServo.setPosition(0);
        LinearSlidesServo.setPosition(0.9);
        ShooterServo.setPosition(.2);

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("Z angle", angles.firstAngle);
        telemetry.addData("Y Angle", angles.secondAngle);
        telemetry.addData("X Angle", angles.thirdAngle);
        telemetry.update();

        waitForStart();

        //Move to Box B, Drop Wobble, Shoot
        toLineForward();

        stopRobot(500);

//        LinearSlidesServo.setPosition(.4);
//
//        sleep(1000);

        ClawServo.setPosition(.5);

        sleep(500);

        moveBack(580, 0.5);

        stopRobot(400);

        Shooter.setPower(1);

        strafeLeft(550, 0.3);
        stopRobot(400);

        turn(125, 0.8);

        stopRobot(500);

        shootRings();
        shootRings();
        shootRings();

        Shooter.setPower(0);

        //Reset to straight for Vuforia
        turn(1, 0.1);

        TopLeft.setDirection(DcMotor.Direction.FORWARD);
        TopRight.setDirection(DcMotor.Direction.REVERSE);
        BottomLeft.setDirection(DcMotor.Direction.FORWARD);
        BottomRight.setDirection(DcMotor.Direction.REVERSE);
        LinearSlidesServo.setPosition(0.9);
        ClawServo.setPosition(0);
        ShooterServo.setPosition(.2);

        //Intake

        while(opModeIsActive()) {

            strafeRight(1, 0.28);

            boolean detect = false;

            while (detect == false && opModeIsActive()) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel() == "Single") {
                            if (recognition.getLeft() < 50 && recognition.getRight() < 380) {

                                detect = true;
                            }

                        }

                    }

                }
            }

            stopRobot(500);

            tfod.shutdown();

            Intake.setPower(1);

            for (int i = 1; i <= 1; i++) {

                Intake.setPower(1);

                telemetry.addData("Intake current", IntakeAmp.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));
                telemetry.update();

                //Moves Backwards until amperage if intake motor goes high, meaning that a ring has gotten in the intake
                while (IntakeAmp.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS) < 6 && opModeIsActive()) {
                    moveBack(1, 0.3);

                    telemetry.addData("Intake current", IntakeAmp.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));
                    telemetry.update();


                }

                while (IntakeAmp.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS) > 3 && opModeIsActive()) {

                    stopRobot(1000);

                    telemetry.addData("Intake current", IntakeAmp.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));
                    telemetry.update();

                }


                Intake.setPower(0);

            }

            Intake.setPower(0);

            //Using color sensor to align with white line
            int counter = 0;
            while (counter != 1) {
                if (color.blue() >= 2000) {
                    stopRobot(100);
                    counter++;
                } else {
                    moveForward(1, 0.3);
                }
            }

            stopRobot(500);

            Shooter.setVelocity(28 * 5100);

            strafeRight(250, 0.6);

            stopRobot(500);

            moveBack(300, 0.4);

            stopRobot(500);

            Shooter.setVelocity(28 * 5100);
            sleep(500);
            ShooterServo.setPosition(0);
            sleep(500);
            ShooterServo.setPosition(.3);
            sleep(500);
            //.2 is base. 0 is shooter position.

            counter = 0;
            while (counter != 1) {
                if (color.blue() >= 2000) {
                    stopRobot(100);
                    counter++;
                } else {
                    moveForward(1, 0.3);
                }
                ;
            }

            moveForward(100, 0.3);

            stopRobot(100);

        }




    }

    /**
        These are all the functions
     */

    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    private void turn(int degrees, double power)
    {
        double  leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
            leftPower = power;
            rightPower = -power;
        }
        else if (degrees > 0)
        {   // turn left.
            leftPower = -power;
            rightPower = power;
        }
        else return;

        // set power to rotate.
        TopLeft.setPower(leftPower);
        TopRight.setPower(rightPower);
        BottomLeft.setPower(leftPower);
        BottomRight.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {}

            while (opModeIsActive() && getAngle() > degrees) {}
        }
        else if (degrees == 0){
            while (opModeIsActive() && getAngle() == 0) {}
        }
        else    // left turn.
            while (opModeIsActive() && getAngle() < degrees) {}

        // turn the motors off.
        TopRight.setPower(0);
        TopLeft.setPower(0);
        BottomLeft.setPower(0);
        BottomRight.setPower(0);

        // wait for rotation to stop.
        sleep(1000);

        // reset angle tracking on new heading.
        resetAngle();
    }
    public void toLineForward(){
        while(counter!= 1){
            if(color.blue() >= 1500){
                TopRight.setPower(0);
                TopLeft.setPower(0);
                BottomRight.setPower(0);
                BottomLeft.setPower(0);
                sleep(10);
                check = true;
                counter++;
            }
            else{
                TopLeft.setPower(.45);
                TopRight.setPower(.45);
                BottomLeft.setPower(.45);
                BottomRight.setPower(.45);
                sleep(1);
            }
            //telemetry.addData("boolean check: ",check);

        }
    }

    public void moveForward(int time, double power){
        TopRight.setPower(power);
        TopLeft.setPower(power);
        BottomLeft.setPower(power);
        BottomRight.setPower(power);
        sleep(time);
    }

    public void moveBack(int time, double power){
        TopRight.setPower(-power);
        TopLeft.setPower(-power);
        BottomLeft.setPower(-power);
        BottomRight.setPower(-power);
        sleep(time);
    }

    public void strafeLeft(int time, double power){
        TopRight.setPower(-power);
        TopLeft.setPower(-power);
        BottomLeft.setPower(power);
        BottomRight.setPower(power);
        sleep(time);
    }

    public void strafeRight(int time, double power){
        TopRight.setPower(power);
        TopLeft.setPower(power);
        BottomLeft.setPower(-power);
        BottomRight.setPower(-power);
        sleep(time);
    }

    public void stopRobot(int time){
        TopRight.setPower(0);
        TopLeft.setPower(0);
        BottomLeft.setPower(0);
        BottomRight.setPower(0);
        sleep(time);
    }
    public void shootRings(){
        Shooter.setPower(1);
        sleep(500);
        ShooterServo.setPosition(0);
        sleep(250);
        ShooterServo.setPosition(0.2);
        sleep(250);
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