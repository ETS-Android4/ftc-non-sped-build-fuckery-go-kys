package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="DriverControlled1", group = "Test")

public class TryThisDriver extends OpMode{

    HardwareRobotDriverControlled robot = new HardwareRobotDriverControlled();
    double speedMultiplier = 0.2;



    @Override
    public void init() {

        robot.init(hardwareMap);
    }

    @Override
    public void loop() {

        double x = gamepad1.left_stick_y; // Remember, this is reversed!
        double y = gamepad1.left_stick_x;
        double ry = gamepad1.right_stick_x;
        double rx = gamepad1.right_stick_y;
        double RT = gamepad1.right_trigger;
        double LT = gamepad1.left_trigger;
        boolean RB = gamepad1.right_bumper;
        boolean LB = gamepad1.left_bumper;
        boolean C = gamepad1.dpad_left;
        boolean CS = gamepad1.dpad_right;
        boolean L = gamepad1.dpad_up;
        boolean LS = gamepad1.dpad_down;
        boolean X = gamepad1.x;
        boolean Y = gamepad1.y;
        boolean A = gamepad1.a;
        boolean B = gamepad1.b;




        robot.TopLeft.setPower((rx - ry + y) * speedMultiplier);
        robot.BottomLeft.setPower((rx + ry + y + x) * speedMultiplier);
        robot.TopRight.setPower((rx - ry - y + x) * speedMultiplier);
        robot.BottomRight.setPower((rx + ry - y) * speedMultiplier);
        robot.IntakeMotor.setPower(RT * 0.9);
        robot.ClawServo.setPosition(1);


        double TopLeftPower = robot.TopLeft.getPower();
        double TopRightPower = robot.TopRight.getPower();
        double BottomLeftPower = robot.BottomLeft.getPower();
        double BottomRightPower = robot.BottomRight.getPower();


        if (Math.abs(TopLeftPower) > 1 || Math.abs(BottomLeftPower) > 1 ||
                Math.abs(TopRightPower) > 1 || Math.abs(BottomRightPower) > 1) {
            // Find the largest power
            double max = 0;
            max = Math.max(Math.abs(TopLeftPower), Math.abs(BottomLeftPower));
            max = Math.max(Math.abs(TopRightPower), max);
            max = Math.max(Math.abs(BottomRightPower), max);
        }
        //LT } first shooter motor and then 0.5 servo on

        if (LT != 0)
        {
            robot.ShooterMotor.setPower(1);
        } else
        {
            robot.ShooterMotor.setPower(0);
        }


        if (LB == true)
        {
            robot.ShooterServo.setPosition(0.05);
        }

        else {
            robot.ShooterServo.setPosition(0.2);
        }

        if(CS == true)
        {
            robot.ClawServo.setPosition(0);
        }

        if (L == true)
        {
            robot.LinearSlides.setPower(-0.5);
        }
        else {
            robot.LinearSlides.setPower(0);
        }

        if (LS == true) {
            robot.LinearSlides.setPower(0.5);
        }
        else {
            robot.LinearSlides.setPower(0);
        }

        if (X == true) {
            speedMultiplier += 0.1;
        }
        if (B == true) {
            speedMultiplier -= 0.1;
        }

        if (A == true) {
            robot.LinearSlidesServo.setPosition(0.4);
        }
    }
}


