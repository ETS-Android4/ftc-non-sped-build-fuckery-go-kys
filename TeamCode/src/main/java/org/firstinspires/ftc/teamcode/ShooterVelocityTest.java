package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp(name="Shooter Velocity Test", group ="Test")
public class ShooterVelocityTest extends OpMode {

    @Override
    public void init() {

    }




    @Override
    public void loop() {
        DcMotorEx Shooter;
        DcMotorEx Intake;
        Servo ShooterServo;

        int ShooterRPM = 1000;
        int IntakeRPM = 800;

        Shooter = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        Intake = hardwareMap.get(DcMotorEx.class, "intakeMotor");

        ShooterServo = hardwareMap.get(Servo.class, "shooterServo");

        boolean A = gamepad1.a;
        boolean B = gamepad1.b;
        boolean X = gamepad1.x;
        boolean Y = gamepad1.y;
        boolean RB = gamepad1.right_bumper;
        double RT = gamepad1.right_trigger;
        double LT = gamepad1.left_trigger;


        Shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            if (A) {
                ShooterRPM += 100;
            }
            if (B) {
                ShooterRPM -= 100;
            }
            if (X) {
                ShooterRPM += 50;
            }
            if (Y) {
                ShooterRPM -= 50;
            }
            if (RB) {
                ShooterServo.setPosition(0);
            } else {
                ShooterServo.setPosition(0.2);
            }
            if (LT != 0) {
                Shooter.setVelocity(28 * ShooterRPM);
            } else {
                Shooter.setVelocity(0);
            }
            if (RT != 0) {
                Intake.setVelocity(28 * IntakeRPM);
            } else {
                Intake.setVelocity(0);
            }

            telemetry.addLine("A to increase Shooter RPM, B to decrease Shooter RPM, X to increase Intake RPM, Y to decrease Intake RPM,");
            telemetry.addData("Shooter RPM", ShooterRPM);
            telemetry.addData("velocity", Shooter.getVelocity());
            telemetry.addData("position", Shooter.getCurrentPosition());
            telemetry.addData("is at target", !Shooter.isBusy());
            telemetry.addData("Intake RPM", IntakeRPM);
            telemetry.addData("velocity", Intake.getVelocity());
            telemetry.addData("position", Intake.getCurrentPosition());
            telemetry.addData("is at target", !Intake.isBusy());
            telemetry.update();



    }
}
