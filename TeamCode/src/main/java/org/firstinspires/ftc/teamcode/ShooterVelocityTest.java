package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Shooter Velocity Test", group ="Test Code")
public class ShooterVelocityTest extends LinearOpMode {
    DcMotorEx Shooter;
    Servo ShooterServo;

    int RPM = 1000;


    @Override
    public void runOpMode() {
        Shooter = hardwareMap.get(DcMotorEx.class, "Motor");
        ShooterServo = hardwareMap.get(Servo.class, "shooterServo");

        boolean A = gamepad1.a;
        boolean B = gamepad1.b;
        boolean RB = gamepad1.right_bumper;

        Shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()) {

            if (A) {
                RPM += 100;
            }
            if (B) {
                RPM -= 100;
            }
            if (RB) {
                ShooterServo.setPosition(0);
            } else {
                ShooterServo.setPosition(0.2);
            }

            Shooter.setVelocity(28 * RPM);

            telemetry.addData("RPM", RPM);
            telemetry.addData("velocity", Shooter.getVelocity());
            telemetry.addData("position", Shooter.getCurrentPosition());
            telemetry.addData("is at target", !Shooter.isBusy());
            telemetry.update();

        }


    }
}
