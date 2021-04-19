
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note: All names are lower case and some have single spaces between words.
 *
 * Motor channel: Drive Motor 1: "drive1"
 * Motor channel: Drive Motor 2: "drive2"
 * Motor channel: Drive Motor 3: "drive3"
 * Motor channel: Drive Motor 4: "drive4"
 */



public class HardwareRobotDriverControlled
{
    /* Public OpMode members. */
    public DcMotor TopLeft = null;
    public DcMotor TopRight = null;
    public DcMotor BottomLeft = null;
    public DcMotor BottomRight = null;
    public DcMotor IntakeMotor = null;
    public DcMotor LinearSlides = null;
    public DcMotor ShooterMotor = null;
    public Servo ShooterServo = null;
    public Servo ClawServo = null;
    public Servo LinearSlidesServo = null;

    /* Local OpMode members. */
    HardwareMap hwMapRobot = null;

    /* Constructor */
    public HardwareRobotDriverControlled(){
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {

// Save reference to Hardware map
        hwMapRobot = ahwMap;

// Define and Initialize Motors
        TopLeft = hwMapRobot.dcMotor.get("topLeft");
        TopRight = hwMapRobot.dcMotor.get("topRight");
        BottomLeft = hwMapRobot.dcMotor.get("bottomLeft");
        BottomRight = hwMapRobot.dcMotor.get("bottomRight");
        ShooterServo = hwMapRobot.servo.get("test");
        IntakeMotor = hwMapRobot.dcMotor.get("intakeMotor");
        ClawServo = hwMapRobot.servo.get("test2");
        ShooterMotor = hwMapRobot.dcMotor.get("shooterMotor");
        LinearSlides = hwMapRobot.dcMotor.get("linearSlideMotor");
        LinearSlidesServo = hwMapRobot.servo.get("linearSlideServo");


//Define Motor Directions
        TopLeft.setDirection(DcMotor.Direction.REVERSE);
        TopRight.setDirection(DcMotor.Direction.FORWARD);
        BottomLeft.setDirection(DcMotor.Direction.REVERSE);
        BottomRight.setDirection(DcMotor.Direction.FORWARD);
        IntakeMotor.setDirection(DcMotor.Direction.FORWARD);

        ClawServo.setDirection(Servo.Direction.REVERSE);
        ShooterMotor.setDirection(DcMotor.Direction.FORWARD);
        LinearSlides.setDirection(DcMotor.Direction.FORWARD);
        LinearSlides.setDirection(DcMotor.Direction.REVERSE);
       // ShooterServo.setDirection(CRServo.Direction.REVERSE);

// Set all motors to zero power
        TopLeft.setPower(0);
        TopRight.setPower(0);
        BottomLeft.setPower(0);
        BottomRight.setPower(0);
        IntakeMotor.setPower(0);
        ClawServo.setPosition(0.6);
        ShooterMotor.setPower(0);
        LinearSlides.setPower(0);

// Set all motors to run with encoders.
// May want to use RUN_WITHOUT_ENCODERS if encoders aren't installed.
        TopLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        TopRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BottomLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BottomRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        IntakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ShooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LinearSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }


}