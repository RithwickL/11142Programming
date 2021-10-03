package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class Rithwick_AMT extends OpMode {
    DcMotor leftvertical;
    DcMotor rightvertical;
    DcMotor lefthorizontal;
    DcMotor righthorzontal;
    /*Servo Front;
    DcMotor Intake;*/
//1S2D

    public void init() {
        leftvertical = hardwareMap.dcMotor.get("lr");
        rightvertical = hardwareMap.dcMotor.get("rr");
        lefthorizontal = hardwareMap.dcMotor.get("rf");
        righthorzontal = hardwareMap.dcMotor.get("lf");
        //Front = hardwareMap.servo.get("blocker");//
        leftvertical.setDirection(DcMotorSimple.Direction.REVERSE);
        lefthorizontal.setDirection(DcMotorSimple.Direction.REVERSE);
        //Intake = hardwareMap.dcMotor.get("intake");//

    }

    public void loop() {
//Use Right stick Y axis to run all motor continuously
        leftvertical.setPower(gamepad1.right_stick_y);

        rightvertical.setPower(gamepad1.right_stick_y);

        lefthorizontal.setPower(gamepad1.right_stick_y);

        righthorzontal.setPower(gamepad1.right_stick_y);

    }
}