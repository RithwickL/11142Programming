package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Test")
public class Test extends LinearOpMode
{
    DcMotor rightFront;
    DcMotor leftFront;
    DcMotor leftRear;
    DcMotor rightRear;

    public void runOpMode()
    {
        leftFront.setPower(0.5);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setTargetPosition(500);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);


    }


    }
