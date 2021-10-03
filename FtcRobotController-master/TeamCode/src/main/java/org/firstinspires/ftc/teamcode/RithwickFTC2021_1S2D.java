package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class RithwickFTC2021_1S2D extends OpMode {
    DcMotor leftvertical;
    DcMotor rightvertical;
    DcMotor lefthorizontal;
    DcMotor righthorzontal;
    /*Servo Front;
    DcMotor Intake;*/
//1S2D

    public void init() {
        leftvertical = hardwareMap.dcMotor.get("lf");
        rightvertical= hardwareMap.dcMotor.get("rr");
        lefthorizontal = hardwareMap.dcMotor.get("lr");
        righthorzontal= hardwareMap.dcMotor.get("rf");
        //Front = hardwareMap.servo.get("blocker");//
        leftvertical.setDirection(DcMotorSimple.Direction.REVERSE);
        lefthorizontal.setDirection(DcMotorSimple.Direction.REVERSE);
        //Intake = hardwareMap.dcMotor.get("intake");//

    }

    public void loop() {
        //Left and Right - right stick - Right/Left
        leftvertical.setPower(gamepad1.right_stick_x);
        rightvertical.setPower(gamepad1.right_stick_x);
        //Forward and Backward - left stick - up/down
        lefthorizontal.setPower(gamepad1.right_stick_y);
        righthorzontal.setPower(gamepad1.right_stick_y);

        //Spin from center orign2 - Left stick- Left/right
        lefthorizontal.setPower(gamepad1.left_trigger);
        righthorzontal.setPower(-gamepad1.left_trigger);
        leftvertical.setPower(gamepad1.left_trigger);
        rightvertical.setPower(-gamepad1.left_trigger);

        lefthorizontal.setPower(-gamepad1.right_trigger);
        righthorzontal.setPower(gamepad1.right_trigger);
        leftvertical.setPower(-gamepad1.right_trigger);
        rightvertical.setPower(gamepad1.right_trigger);

        /*if(gamepad1.b == true) {

        }
        else{

        }

        if(gamepad1.y == true) {

        }
        else{

        }

        if(gamepad1.a == true) {

        }
        else{

        }

        if(gamepad1.x == true) {

        }
        else{

        }*/

        /*if(gamepad1.dpad_down == true && gamepad1.dpad_right == true) {
            lefthorizontal.setPower(1);
            rightvertical.setPower(gamepad1.left_trigger);
        }


        //Horizontal top left - Left trigger
        lefthorizontal.setPower(-gamepad1.left_trigger);
        rightvertical.setPower(gamepad1.left_trigger);

        //Horizontal top right - right trigger
        lefthorizontal.setPower(-gamepad1.right_trigger);
        rightvertical.setPower(gamepad1.right_trigger);

        //Horizontal top left - Left bummper
        lefthorizontal.setPower(-gamepad1.);
        rightvertical.setPower(gamepad1.left_stick_x);

        //Horizontal top left - Left trigger
        lefthorizontal.setPower(-gamepad1.left_stick_x);
        rightvertical.setPower(gamepad1.left_stick_x);*/



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