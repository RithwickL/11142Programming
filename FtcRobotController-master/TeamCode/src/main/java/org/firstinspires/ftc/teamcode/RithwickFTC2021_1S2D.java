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
    DcMotor Top;
    DcMotor Spin;
    DcMotor Spin2;
    /*Servo Front;
    DcMotor Intake;*/
//1S2D

    public void init() {
        leftvertical = hardwareMap.dcMotor.get("lf");
        rightvertical= hardwareMap.dcMotor.get("rr");
        lefthorizontal = hardwareMap.dcMotor.get("lr");
        righthorzontal= hardwareMap.dcMotor.get("rf");
        Top= hardwareMap.dcMotor.get("Top");
        Spin= hardwareMap.dcMotor.get("Spin");
        Spin2= hardwareMap.dcMotor.get("Spin2");
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

        if(gamepad1.b == true) {
         Spin.setPower(0.75);
        }
        else{
            Spin.setPower(0);
        }

        if(gamepad1.y == true) {
            Spin2.setPower(0.75);
        }
        else{
            Spin2.setPower(0);
        }

        if(gamepad1.a == true) {
            Top.setPower(1);
        }
        else{
            Top.setPower(0);
        }

        /*if(gamepad1.x == true) {

        }
        else{*/


    }
}