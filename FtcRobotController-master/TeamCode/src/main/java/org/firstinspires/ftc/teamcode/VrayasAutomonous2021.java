/*package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;

@Autonomous(name="VrayasAuto")
@Disabled
public class VrayasAutomonous2021 extends LinearOpMode
{

    DcMotor vertFront;
    DcMotor vertBack;
    DcMotor horBack;
    DcMotor horFront;



    public void runOpMode()
    {
        vertFront = hardwareMap.dcMotor.get("lr");
        vertBack = hardwareMap.dcMotor.get("rf");
        horFront = hardwareMap.dcMotor.get("lf");

        horBack = hardwareMap.dcMotor.get("rr");
        vertBack.setDirection(DcMotorSimple.Direction.REVERSE);
        horBack.setDirection(DcMotorSimple.Direction.REVERSE);

        vertFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        vertBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        horFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        horBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        if (opModeIsActive())
        {
            Driveforward(0.5, 12);

            DriveBackwards(0.5, 12);

            DriveLeft(0.5,12 );

            DriveBackwards(0.5, 12);

            StopDriving();

        }
    }

    public void DriveRight
/*
this method will used to drive left.

    public void DriveLeft(double power, int distance)
    {
        //
        horFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        horFront.setTargetPosition(distance * 31);
        horBack.setTargetPosition(distance * 31);

        horBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        horFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        horFront.setPower(power);
        horBack.setPower(power);

        while( vertFront.isBusy() && vertBack.isBusy())
        {

        }

        StopDriving();
        horFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        horBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void DriveBackwards(double power, int distance)
    {
        //Reset Encoders
        vertFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vertBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Set Position

        vertFront.setTargetPosition(-distance * 31);
        horBack.setTargetPosition(-distance * 31);

        //Movement

        vertBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        vertFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        vertFront.setPower(power);
        vertBack.setPower(power);

        while( vertFront.isBusy() && vertBack.isBusy())
        {

        }

    }

    public void Driveforward(double power, int distance)
    {




        //Reset Encoders
        vertFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vertBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //Set Position
        vertFront.setTargetPosition(distance * 31);
        vertBack.setTargetPosition(distance * 31);


        //Movement

        vertFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        vertBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        vertFront.setPower(power);
        vertBack.setPower(power);

        while(vertFront.isBusy() && vertBack.isBusy())
        {

        }


    }
    public void StopDriving()
    {

        vertFront.setPower(0);
        vertBack.setPower(0);
        horBack.setPower(0);
        horFront.setPower(0);


    }
}*/