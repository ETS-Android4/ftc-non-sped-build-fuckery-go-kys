package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Intake", group = "test")
public class IntakeTest extends LinearOpMode {

    public DcMotor intakeMotor;

    public void runOpMode() throws InterruptedException{

        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        waitForStart();
        while (opModeIsActive()) {
            intakeMotor.setPower(1);
            sleep(10000);
        }


    }

}
