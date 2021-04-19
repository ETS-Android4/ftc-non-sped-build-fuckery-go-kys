package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Autos", group = "nobody asked")
public class AutoHardware extends LinearOpMode
{

    DcMotor TopLeft;
    DcMotor TopRight;
    DcMotor BottomLeft;
    DcMotor BottomRight;

    @Override
    public void runOpMode() throws InterruptedException{
        TopLeft = hardwareMap.dcMotor.get("drive1");
        TopRight = hardwareMap.dcMotor.get("drive2");
        BottomLeft = hardwareMap.dcMotor.get("drive3");
        BottomRight = hardwareMap.dcMotor.get("drive4");

        TopLeft.setDirection(DcMotor.Direction.REVERSE);
        TopRight.setDirection(DcMotor.Direction.FORWARD);
        BottomLeft.setDirection(DcMotor.Direction.REVERSE);
        BottomRight.setDirection(DcMotor.Direction.FORWARD);

        TopLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        TopRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BottomLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BottomRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        MoveForward(800);//place wobble
        MoveBackward(110); //to then shoot
        StrafeRight(1000);//to get into position for shooting
        //shoot
        /*when done::*/ MoveForward(65);
    }

    public void MoveForward(int pos) {
        TopLeft.setTargetPosition(pos);
        TopRight.setTargetPosition(pos);
        BottomLeft.setTargetPosition(pos);
        BottomRight.setTargetPosition(pos);
        TopRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        TopLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BottomRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BottomLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        TopLeft.setPower(0.5);
        TopRight.setPower(0.5);
        BottomLeft.setPower(0.5);
        BottomRight.setPower(0.5);
        //sleep(pos);
        while(TopLeft.isBusy() && TopRight.isBusy() && BottomLeft.isBusy() && BottomRight.isBusy())
        {

        }
        TopLeft.setPower(0);
        TopRight.setPower(0);
        BottomLeft.setPower(0);
        BottomRight.setPower(0);
        TopLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        TopRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BottomLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BottomRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        TopLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
        TopRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
        BottomLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
        BottomRight.setMode(DcMotor.RunMode.RESET_ENCODERS);

    }

    public void MoveBackward(int pos) {
        TopLeft.setTargetPosition(-pos);
        TopRight.setTargetPosition(-pos);
        BottomLeft.setTargetPosition(-pos);
        BottomRight.setTargetPosition(-pos);
        TopRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        TopLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BottomRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BottomLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        TopLeft.setPower(0.5);
        TopRight.setPower(0.5);
        BottomLeft.setPower(0.5);
        BottomRight.setPower(0.5);
        //sleep(pos);
        while(TopLeft.isBusy() && TopRight.isBusy() && BottomLeft.isBusy() && BottomRight.isBusy())
        {

        }
        TopLeft.setPower(0);
        TopRight.setPower(0);
        BottomLeft.setPower(0);
        BottomRight.setPower(0);
        TopLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
        TopRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
        BottomLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
        BottomRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
        TopLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        TopRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BottomLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BottomRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void StrafeLeft(int pos) {
        TopLeft.setTargetPosition(-pos);
        TopRight.setTargetPosition(pos);
        BottomLeft.setTargetPosition(-pos);
        BottomRight.setTargetPosition(pos);
        TopRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        TopLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BottomRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BottomLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        TopLeft.setPower(0.5);
        TopRight.setPower(0.5);
        BottomLeft.setPower(0.5);
        BottomRight.setPower(0.5);
        //sleep(pos);
        while(TopLeft.isBusy() && TopRight.isBusy() && BottomLeft.isBusy() && BottomRight.isBusy())
        {

        }
        TopLeft.setPower(0);
        TopRight.setPower(0);
        BottomLeft.setPower(0);
        BottomRight.setPower(0);
        TopLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
        TopRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
        BottomLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
        BottomRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
        TopLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        TopRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BottomLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BottomRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void StrafeRight(int pos) {
        TopLeft.setTargetPosition(-pos);
        TopRight.setTargetPosition(-pos);
        BottomLeft.setTargetPosition(pos);
        BottomRight.setTargetPosition(pos);
        TopRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        TopLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BottomRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BottomLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        TopLeft.setPower(0.5);
        TopRight.setPower(0.5);
        BottomLeft.setPower(0.5);
        BottomRight.setPower(0.5);
        //sleep(pos);
        while(TopLeft.isBusy() && TopRight.isBusy() && BottomLeft.isBusy() && BottomRight.isBusy())
        {

        }
        TopLeft.setPower(0);
        TopRight.setPower(0);
        BottomLeft.setPower(0);
        BottomRight.setPower(0);
        TopLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
        TopRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
        BottomLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
        BottomRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
        TopLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        TopRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BottomLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BottomRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void RotateRight(int pos) {
        TopLeft.setTargetPosition(pos);
        TopRight.setTargetPosition(-pos);
        BottomLeft.setTargetPosition(pos);
        BottomRight.setTargetPosition(-pos);
        TopRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        TopLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BottomRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BottomLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        TopLeft.setPower(0.5);
        TopRight.setPower(0.5);
        BottomLeft.setPower(0.5);
        BottomRight.setPower(0.5);
        //sleep(pos);
        while(TopLeft.isBusy() && TopRight.isBusy() && BottomLeft.isBusy() && BottomRight.isBusy())
        {

        }
        TopLeft.setPower(0);
        TopRight.setPower(0);
        BottomLeft.setPower(0);
        BottomRight.setPower(0);
        TopLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
        TopRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
        BottomLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
        BottomRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
        TopLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        TopRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BottomLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BottomRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void RotateLeft(int pos) {
        TopLeft.setTargetPosition(-pos);
        TopRight.setTargetPosition(pos);
        BottomLeft.setTargetPosition(-pos);
        BottomRight.setTargetPosition(pos);
        TopRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        TopLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BottomRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BottomLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        TopLeft.setPower(0.5);
        TopRight.setPower(0.5);
        BottomLeft.setPower(0.5);
        BottomRight.setPower(0.5);
        //sleep(pos);
        while(TopLeft.isBusy() && TopRight.isBusy() && BottomLeft.isBusy() && BottomRight.isBusy())
        {

        }
        TopLeft.setPower(0);
        TopRight.setPower(0);
        BottomLeft.setPower(0);
        BottomRight.setPower(0);
        TopLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TopRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BottomLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BottomRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TopLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        TopRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BottomLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BottomRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
