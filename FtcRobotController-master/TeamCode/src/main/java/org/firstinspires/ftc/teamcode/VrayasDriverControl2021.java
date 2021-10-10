package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class VrayasDriverControl2021 extends OpMode {

    DcMotor vertFront;
    DcMotor vertBack;
    DcMotor horFront;
    DcMotor horBack;

    public void init() {
        //Motors
        vertFront = hardwareMap.dcMotor.get("lr");
        vertBack = hardwareMap.dcMotor.get("rf");
        horFront = hardwareMap.dcMotor.get("lf");
        horBack = hardwareMap.dcMotor.get("rr");

        //
        vertBack.setDirection(DcMotorSimple.Direction.REVERSE);
        horFront.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public void loop() {

        // Forward & Backwards
        horFront.setPower(gamepad1.right_stick_y);
        horBack.setPower(gamepad1.right_stick_y);


        //Left & Right
        vertFront.setPower(gamepad1.left_stick_x);
        vertBack.setPower(gamepad1.left_stick_x);


        //Rotate right
        horFront.setPower(gamepad1.right_trigger);
        horBack.setPower(gamepad1.right_trigger);

        //Rotate left
        horFront.setPower(gamepad1.left_trigger);
        horBack.setPower(gamepad1.left_trigger);

    }
}
