
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * This is NOT an op-mode.
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
public class HardwareRobotAutonomous
{
    /* Public OpMode members. */
    public DcMotor TopLeft = null;
    public DcMotor TopRight = null;
    public DcMotor BottomLeft = null;
    public DcMotor BottomRight = null;
    public DcMotor ShooterMotor = null;
    public DcMotor intakeMotor = null;

    public Servo testServo = null;

    /* Local OpMode members. */
    HardwareMap hwMapRobot = null;

    /* Constructor */
    public HardwareRobotAutonomous(){
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {

// Save reference to Hardware map
        hwMapRobot = ahwMap;

// Define and Initialize Motors
        TopLeft = hwMapRobot.dcMotor.get("drive1");
        TopRight = hwMapRobot.dcMotor.get("drive2");
        BottomLeft = hwMapRobot.dcMotor.get("drive3");
        BottomRight = hwMapRobot.dcMotor.get("drive4");
        testServo = hwMapRobot.servo.get("servo1");

//Define Motor Directions
        TopLeft.setDirection(DcMotor.Direction.REVERSE);
        TopRight.setDirection(DcMotor.Direction.FORWARD);
        BottomLeft.setDirection(DcMotor.Direction.REVERSE);
        BottomRight.setDirection(DcMotor.Direction.FORWARD);

// Set all motors to zero power
        TopLeft.setPower(0);
        TopRight.setPower(0);
        BottomLeft.setPower(0);
        BottomRight.setPower(0);
        testServo.setPosition(0);

// Set all motors to run with encoders.
// May want to use RUN_WITHOUT_ENCODERS if encoders aren't installed.
        TopLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        TopRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BottomLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BottomRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
        public void Forward() {

            TopLeft.setPower(0.4);
            BottomLeft.setPower(0.4);
            TopRight.setPower(0.4);
            BottomRight.setPower(0.4);

        }

        public void Backward()
        {
           TopLeft.setPower(0.4);
           BottomLeft.setPower(-0.4);
           TopRight.setPower(-0.4);
           BottomRight.setPower(-0.4);
        }

        public void MoveRight()
        {
            TopLeft.setPower(0.4);
            BottomLeft.setPower(-0.4);
            TopRight.setPower(0.4);
            BottomRight.setPower(-0.4);
        }

        public void MoveLeft()
        {
            TopLeft.setPower(-0.4);
            BottomLeft.setPower(0.4);
            TopRight.setPower(-0.4);
            BottomRight.setPower(0.4);
        }

        public void RotateRight()
        {
            TopLeft.setPower(0.4);
            BottomLeft.setPower(0.4);
            TopRight.setPower(-0.4);
            BottomRight.setPower(-0.4);
        }

        public void RotateLeft() {
            TopLeft.setPower(-0.4);
            BottomLeft.setPower(-0.4);
            TopRight.setPower(0.4);
            BottomRight.setPower(0.4);

        }

        public void intakeMotor(){
            intakeMotor.setPower(1);

        }

        public void Stop()
        {
            TopLeft.setPower(0);
            BottomLeft.setPower(0);
            TopRight.setPower(0);
            BottomRight.setPower(0);
        }


}