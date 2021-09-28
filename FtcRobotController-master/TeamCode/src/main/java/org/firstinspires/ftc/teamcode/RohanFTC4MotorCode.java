package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;



@TeleOp

public class RohanFTC4MotorCode extends OpMode {

    DcMotor leftHorizontal ;
    DcMotor rightHorizontal ;
    DcMotor leftvertical ;
    DcMotor rightvertical;

    public void init() {
        leftHorizontal = hardwareMap.dcMotor.get("leftHorizontal ");
        rightHorizontal = hardwareMap.dcMotor.get("rightHorizontal ");
        leftvertical = hardwareMap.dcMotor.get("leftvertical ");
        rightvertical = hardwareMap.dcMotor.get("rightvertical");

    }

    public void loop(){




    // Forward and Backward


        rightHorizontal .setPower(gamepad1.right_stick_x);
        leftHorizontal.setPower(-gamepad1.left_stick_x);



        rightvertical.setPower(gamepad1.right_stick_y);
        leftvertical.setPower(-gamepad1.right_stick_y);


    }
}
