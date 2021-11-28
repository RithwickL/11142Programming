package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Arm_Control extends LinearOpMode
{
    DcMotor leftFront;
    DcMotor leftRear;
    DcMotor rightRear;
    DcMotor rightFront;
    DcMotor Top;
    DcMotor Arm1;
    DcMotor Pick;

    public void runOpMode()
    {

        leftFront = hardwareMap.dcMotor.get("lf");
        leftRear = hardwareMap.dcMotor.get("lr");
        rightRear = hardwareMap.dcMotor.get("rr");
        rightFront = hardwareMap.dcMotor.get("rf");
        Arm1 = hardwareMap.dcMotor.get("arm");
        Top = hardwareMap.dcMotor.get("carousel");
        //Slide = hardwareMap.dcMotor.get("Slide");
        Pick = hardwareMap.dcMotor.get("");
    }
}
