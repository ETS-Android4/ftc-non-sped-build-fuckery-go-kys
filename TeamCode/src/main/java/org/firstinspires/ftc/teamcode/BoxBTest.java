package org.firstinspires.ftc.teamcode;

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

@Autonomous(name = "BoxBFinal", group = "Sensor")

public class BoxBTest extends LinearOpMode {
    ColorSensor color;
    boolean check = false;
    DcMotor TopRight;
    DcMotor TopLeft;
    BNO055IMU imu;
    Orientation angles;
    DcMotor BottomRight;
    DcMotor BottomLeft;
    DcMotor Intake;
    DcMotor Shooter;
    Servo ShooterServo;
    Servo LinearSlidesServo;
    Servo ClawServo;
    int counter = 0;

    Orientation             lastAngles = new Orientation();
    double                  globalAngle, correction;

    //Amp Stuff
    ExpansionHubMotor IntakeAmp;
    ExpansionHubEx expansionHub;
    // public void turn(double power, int time){
    //     TopRight.setPower(-power);
    //     TopLeft.setPower(power);
    //     BottomRight.setPower(-power);
    //     BottomLeft.setPower(power);
    //     sleep(time);
    // }
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
        sleep(1500);
        ShooterServo.setPosition(0);
        sleep(250);
        ShooterServo.setPosition(0.2);
        sleep(250);
    }

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

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
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
        Shooter = hardwareMap.dcMotor.get("shooterMotor");
        LinearSlidesServo = hardwareMap.servo.get("linearSlideServo");

        TopLeft.setDirection(DcMotor.Direction.REVERSE);
        TopRight.setDirection(DcMotor.Direction.FORWARD);
        BottomLeft.setDirection(DcMotor.Direction.REVERSE);
        BottomRight.setDirection(DcMotor.Direction.FORWARD);

        ClawServo.setPosition(0);
        LinearSlidesServo.setPosition(0.9);
        ShooterServo.setPosition(.2);
        float gain = 12;
        //color.setGain(gain);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("Z angle", angles.firstAngle);
        telemetry.addData("Y Angle", angles.secondAngle);
        telemetry.addData("X Angle", angles.thirdAngle);
        telemetry.update();
        waitForStart();
        // toLineForward();
        // stopRobot();
        // telemetry.addLine("hi");
        // telemetry.update();
        // LinearSlidesServo.setPosition(.4);
        // sleep(100);
        // ClawServo.setPosition(.5);
        // sleep(1000);
        // moveForward(800, -0.5);
        // stopRobot();
        // angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        // telemetry.addData("Z-Angle", angles.firstAngle);
        // telemetry.update();
        // sleep(1000);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//        sleep(100);
//        turn(210, 10);

        toLineForward();

        stopRobot(500);

        LinearSlidesServo.setPosition(.4);

        sleep(1000);

        ClawServo.setPosition(.5);

        sleep(500);

        moveBack(580, 0.5);

        stopRobot(400);

        strafeLeft(550, 0.3);
        stopRobot(400);

        turn(125, 0.8);

        stopRobot(500);

        shootRings();
        shootRings();
        shootRings();

        turn(0, 0.8);

        TopLeft.setDirection(DcMotor.Direction.FORWARD);
        TopRight.setDirection(DcMotor.Direction.REVERSE);
        BottomLeft.setDirection(DcMotor.Direction.FORWARD);
        BottomRight.setDirection(DcMotor.Direction.REVERSE);
        LinearSlidesServo.setPosition(0.9);
        ClawServo.setPosition(0);
        ShooterServo.setPosition(.2);

        // stopRobot();
        // strafeLeft(265, .3);
        // stopRobot();
        // shootRings();
        // shootRings();
        // shootRings();
        // //turn(.6,1200);
        // stopRobot();
        // moveForward(1000,-.5);
        // moveForward(500,-.2);
//         TopRight.setPower(0);
//         TopLeft.setPower(0);
//         BottomRight.setPower(0);
//         BottomLeft.setPower(0);
//         sleep(500);

//         TopRight.setPower(.10);
//         TopLeft.setPower(.10);
//         BottomRight.setPower(.10);
//         BottomLeft.setPower(.10);
//         //ClawServo.setPosition(.5);

//         TopRight.setPower(0);
//         TopLeft.setPower(0);
//         BottomRight.setPower(0);
//         BottomLeft.setPower(0);
//         sleep(500);

//         TopLeft.setPower(0.8);
//         TopRight.setPower(0.8);
//         BottomRight.setPower(-0.8);
//         BottomLeft.setPower(-0.8);
//         sleep(400);


//         TopLeft.setPower(0);
//         TopRight.setPower(0);
//         BottomRight.setPower(0);
//         BottomLeft.setPower(0);
//         sleep(1000);
//         counter = 0;

//         TopLeft.setPower(-.5);
//         TopRight.setPower(-.5);
//         BottomLeft.setPower(-.5);
//         BottomRight.setPower(-.5);
//         sleep(70);

//         TopLeft.setPower(0);
//         TopRight.setPower(0);
//         BottomRight.setPower(0);
//         BottomLeft.setPower(0);
//         sleep(300);

//         ClawServo.setPosition(.5);
//         sleep(250);

// //        TopLeft.setPower(0);
// //        TopRight.setPower(0);
// //        BottomRight.setPower(0);
// //        BottomLeft.setPower(0);
// //        sleep(500);

//         TopLeft.setPower(-.5);
//         TopRight.setPower(-.5);
//         BottomLeft.setPower(-.5);
//         BottomRight.setPower(-.5);
//         sleep(600);

//         TopLeft.setPower(0);
//         TopRight.setPower(0);
//         BottomRight.setPower(0);
//         BottomLeft.setPower(0);
//         sleep(1000);

//         TopRight.setPower(-20);
//         TopLeft.setPower(20);
//         BottomLeft.setPower(20);
//         BottomRight.setPower(-20);
//         sleep(350);

//         TopLeft.setPower(0);
//         TopRight.setPower(0);
//         BottomRight.setPower(0);
//         BottomLeft.setPower(0);
//         sleep(500);

//         Shooter.setPower(100);
//         sleep(1500);
//         ShooterServo.setPosition(0);
//         sleep(500);
//         ShooterServo.setPosition(.3);
//         sleep(500);
//         //.2 is base. 0 is shooter position.
//         Shooter.setPower(100);
//         sleep(1500);
//         ShooterServo.setPosition(0);
//         sleep(500);
//         ShooterServo.setPosition(.3);
//         sleep(500);
//         //2nd shot
//         Shooter.setPower(100);
//         sleep(1800);
//         ShooterServo.setPosition(0);
//         sleep(500);
//         ShooterServo.setPosition(.2);
//         sleep(1000);
//         // 3rd shot.


//         //INTAKE STUFFFFFF
//         TopLeft.setDirection(DcMotor.Direction.FORWARD);
//         TopRight.setDirection(DcMotor.Direction.REVERSE);
//         BottomLeft.setDirection(DcMotor.Direction.FORWARD);
//         BottomRight.setDirection(DcMotor.Direction.REVERSE);
//         sleep(500);

//         Intake.setPower(1);

//         for (int i = 1; i <=1; i++) {

//             Intake.setPower(1);

//             telemetry.addData("Intake current", IntakeAmp.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));
//             telemetry.update();

//             //Moves Backwards until amperage if intake motor goes high, meaning that a ring has gotten in the intake
//             while (IntakeAmp.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS) < 6) {
//                 TopLeft.setPower(-0.3);
//                 TopRight.setPower(-0.3);
//                 BottomRight.setPower(-0.3);
//                 BottomLeft.setPower(-0.3);

//                 telemetry.addData("Intake current", IntakeAmp.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));
//                 telemetry.update();
//             }

//             telemetry.addData("Intake current", IntakeAmp.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));
//             telemetry.update();

//             TopLeft.setPower(0);
//             TopRight.setPower(0);
//             BottomRight.setPower(0); //stop
//             BottomLeft.setPower(0);
//             sleep(1000);

//             if (IntakeAmp.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS) < 3) {
//                 continue;
//             }

//             Intake.setPower(0);

//             TopLeft.setPower(0.4);
//             TopRight.setPower(0.4);
//             BottomRight.setPower(0.4);
//             BottomLeft.setPower(0.4);
//             sleep(200);

//             TopLeft.setPower(0);
//             TopRight.setPower(0);
//             BottomRight.setPower(0); //stop
//             BottomLeft.setPower(0);
//             sleep(500);

//             Intake.setPower(1);
//             sleep(500);

//             //If the ring is still stuck in the intake for 2 seconds, we strafe right to get it out
//             while (IntakeAmp.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS) < 6) {
//                 TopLeft.setPower(-0.2);
//                 TopRight.setPower(-0.2);
//                 BottomRight.setPower(-0.2);
//                 BottomLeft.setPower(-0.2);
//                 if (IntakeAmp.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS) < 6) {
//                     continue;
//                 }
//             }
//             sleep(500);

//             Intake.setPower(0);

//             telemetry.addData("Intake current", IntakeAmp.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));
//             telemetry.update();

//             TopLeft.setPower(0);
//             TopRight.setPower(0);
//             BottomRight.setPower(0); //stop
//             BottomLeft.setPower(0);
//             sleep(1000);

//         }

//         Intake.setPower(0);

//         //Using color sensor to align with white line
//         int counter = 0;
//         while (counter != 1) {
//             if(color.blue() >= 2000){
//                 TopLeft.setPower(0);
//                 TopRight.setPower(0);
//                 BottomRight.setPower(0);
//                 BottomLeft.setPower(0);
//                 sleep(100);
//                 counter++;
//             }
//             else{
//                 TopLeft.setPower(0.3);
//                 TopRight.setPower(0.3);
//                 BottomRight.setPower(0.3);
//                 BottomLeft.setPower(0.3);
//                 sleep(1);
//             }
//             ;
//         }

//         TopLeft.setPower(0);
//         TopRight.setPower(0);
//         BottomRight.setPower(0); //stop
//         BottomLeft.setPower(0);
//         sleep(500);

//         TopLeft.setPower(-0.8);
//         TopRight.setPower(0.8);
//         BottomRight.setPower(0.8);
//         BottomLeft.setPower(-0.8);    // rotate right
//         sleep(50);

//         TopLeft.setPower(0);
//         TopRight.setPower(0);
//         BottomRight.setPower(0); //stop
//         BottomLeft.setPower(0);
//         sleep(500);

//         TopLeft.setPower(-0.4);
//         TopRight.setPower(-0.4);
//         BottomRight.setPower(-0.4);
//         BottomLeft.setPower(-0.4);    // rotate right
//         sleep(160);

//         TopLeft.setPower(0);
//         TopRight.setPower(0);
//         BottomRight.setPower(0); //stop
//         BottomLeft.setPower(0);
//         sleep(500);

//         Shooter.setPower(100);
//         sleep(2000);
//         ShooterServo.setPosition(0);
//         sleep(500);
//         ShooterServo.setPosition(.3);
//         sleep(1000);
//         //.2 is base. 0 is shooter position.

//         counter = 0;
//         while (counter != 1) {
//             if(color.blue() >= 2000){
//                 TopLeft.setPower(0);
//                 TopRight.setPower(0);
//                 BottomRight.setPower(0);
//                 BottomLeft.setPower(0);
//                 sleep(100);
//                 counter++;
//             }
//             else{
//                 TopLeft.setPower(-0.3);
//                 TopRight.setPower(-0.3);
//                 BottomRight.setPower(-0.3);
//                 BottomLeft.setPower(-0.3);
//                 sleep(1);
//             }
//             ;
//         }

//         TopLeft.setPower(0.3);
//         TopRight.setPower(0.3);
//         BottomRight.setPower(0.3);
//         BottomLeft.setPower(0.3);
//         sleep(100);

//         TopLeft.setPower(0);
//         TopRight.setPower(0);
//         BottomRight.setPower(0); //stop
//         BottomLeft.setPower(0);
//         sleep(1000);



    }
}