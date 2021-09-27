package org.firstinspires.ftc.teamcode;

import
        com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp
public class RithwickFirstOpMode4W2 extends OpMode {
    DcMotor left;
    DcMotor right;
    DcMotor leftback;
    DcMotor rightback;
    /*Servo Front;
    DcMotor Intake;*/


    public void init() {
        right = hardwareMap.dcMotor.get("rf");
        leftback= hardwareMap.dcMotor.get("lr");
        rightback = hardwareMap.dcMotor.get("rr");
        left= hardwareMap.dcMotor.get("lf");
        //Front = hardwareMap.servo.get("blocker");//
        left.setDirection(DcMotorSimple.Direction.REVERSE);
        leftback.setDirection(DcMotorSimple.Direction.REVERSE);
        //Intake = hardwareMap.dcMotor.get("intake");//

    }

    public void loop() {
        //forward and Backward - right stick - up/down
        right.setPower(gamepad1.right_stick_y);
        left.setPower(gamepad1.right_stick_y);
        leftback.setPower(gamepad1.right_stick_y);
        rightback.setPower(gamepad1.right_stick_y);

        //Spin left and right - right stick - left/right
        right.setPower(-gamepad1.right_stick_x);
        left.setPower(gamepad1.right_stick_x);
        leftback.setPower(gamepad1.right_stick_x);
        rightback.setPower(-gamepad1.right_stick_x);

        //Mechanical slide - left trigger - click/hold
        right.setPower(gamepad1.left_trigger);
        left.setPower(-gamepad1.left_trigger);
        leftback.setPower(gamepad1.left_trigger);
        rightback.setPower(-gamepad1.left_trigger);

        //Mechanical slide - right trigger - click/hold
        right.setPower(-gamepad1.right_trigger);
        left.setPower(gamepad1.right_trigger);
        leftback.setPower(-gamepad1.right_trigger);
        rightback.setPower(gamepad1.right_trigger);

        /*
        Mechanical diagonal forward right - right bumper - click/hold
        rightback.setPower(gamepad1.right_trigger);
        left.setPower(-gamepad1.right_trigger);
        Mechanical diagonal forward left - left bumper - click/hold
        leftback.setPower(-gamepad1.right_trigger);
        right.setPower(gamepad1.left_bumper);
        */
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