package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;

@Autonomous
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

        vertFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        vertBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        horFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        horBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (opModeIsActive()){
            Driveforward();
        }
    }

    public void Driveforward()
    {
        int distance;
        int power;

        horFront.setPower(1);
        horBack.setPower(1);
        vertFront.setPower(1);
        vertBack.setPower(1);

        //Reset Encoders
        vertFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vertBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //Set Postion
        vertFront.setTargetPosition(1000);
        vertBack.setTargetPosition(1000);
        horFront.setTargetPosition(1000);
        horBack.setTargetPosition(1000);

        //Movement

        vertFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        vertBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        horFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        horBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);




    }
}