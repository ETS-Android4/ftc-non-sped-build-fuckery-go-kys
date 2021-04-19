package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="ShooterIntakeServoTest", group = "Test")

public class ShooterIntakeServoTest extends OpMode{

    HardwareRobotDriverControlled robot = new HardwareRobotDriverControlled();


    @Override
    public void init() {

        robot.init(hardwareMap);

    }

    @Override
    public void loop() {

        //Use the triggers to control motor - Plug into 4th port on control hub
        double RT = gamepad1.right_trigger;
        double LT = gamepad1.left_trigger;


        //For Servos
        boolean RB = gamepad1.right_bumper;
        boolean LB = gamepad1.left_bumper;
        //double servoPos = robot.ShooterServo.getPower();

        if(RT > 0) {
            robot.ShooterServo.setPosition(1);
        }
        else if (LT > 0) {
            robot.ShooterServo.setPosition(0);
        }
    }
}