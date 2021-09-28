package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class RithwickFTC2021 extends OpMode {
    DcMotor leftvertical;
    DcMotor rightvertical;
    DcMotor lefthorizontal;
    DcMotor righthorzontal;
    /*Servo Front;
    DcMotor Intake;*/


    public void init() {
        leftvertical = hardwareMap.dcMotor.get("rf");
        rightvertical= hardwareMap.dcMotor.get("lr");
        lefthorizontal = hardwareMap.dcMotor.get("rr");
        righthorzontal= hardwareMap.dcMotor.get("lf");
        //Front = hardwareMap.servo.get("blocker");//
        leftvertical.setDirection(DcMotorSimple.Direction.REVERSE);
        lefthorizontal.setDirection(DcMotorSimple.Direction.REVERSE);
        //Intake = hardwareMap.dcMotor.get("intake");//

    }

    public void loop() {
        //Left and Right - right stick - Right/Left
        leftvertical.setPower(gamepad1.right_stick_y);
        rightvertical.setPower(gamepad1.right_stick_y);
        //Forward and Backward - right stick - up/down
        lefthorizontal.setPower(gamepad1.right_stick_x);
        righthorzontal.setPower(gamepad1.right_stick_x);

        //Spin from center orign2 - Left stick- Left/right
        lefthorizontal.setPower(gamepad1.left_stick_x);
        righthorzontal.setPower(-gamepad1.left_stick_x);
        leftvertical.setPower(gamepad1.left_stick_x);
        rightvertical.setPower(-gamepad1.left_stick_x);

        /*//Spin from center orign - Left stick- Left/right
        leftvertical.setPower(-gamepad1.left_stick_x);
        rightvertical.setPower(gamepad1.left_stick_x);*/



        /*Front.setPostion(0);
        //Front Servo for Blocker use x to open
        if (gamepad1.x) {
            Front.setPostion(1);
        }
        //Front Servo for Blocker use b to close
        if (gamepad1.b) {
            Front.setPostion(0);
        }*/
        //Intaker Motor for intake use y to run
        /*if (gamepad1.y) {
            Intake.setPower(- 1);
        }
        //Intaker Motor for intake use nothing to stop
        else {
            Intake.setPower(0);
        }*/





    }
}