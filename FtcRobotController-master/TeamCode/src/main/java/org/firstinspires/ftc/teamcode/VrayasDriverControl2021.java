package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class VrayasDriverControl2021 extends OpMode {

    DcMotor vert1;
    DcMotor vert2;
    DcMotor hor1;
    DcMotor hor2;

    public void init() {
        vert1 = hardwareMap.dcMotor.get("vert1");
        vert2 = hardwareMap.dcMotor.get("vert2");
        hor1 = hardwareMap.dcMotor.get("hor1");
        hor2 = hardwareMap.dcMotor.get("hor2");

        vert1.setDirection(DcMotorSimple.Direction.REVERSE);
        vert2.setDirection(DcMotorSimple.Direction.REVERSE);

        hor1.setDirection(DcMotorSimple.Direction.REVERSE);
        hor2.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public void loop() {

        // Forward & Backwards
        vert1.setPower(gamepad1.right_stick_y);
        vert2.setPower(gamepad1.right_stick_y);


        //Left & Right
        hor1.setPower(gamepad1.right_stick_x);
        hor2.setPower(gamepad1.right_stick_x);


        //Rotate right
        hor1.setPower(gamepad1.right_trigger);
        hor2.setPower(-gamepad1.right_trigger);

        //Rotate left
        hor1.setPower(-gamepad1.left_trigger);
        hor2.setPower(gamepad1.left_trigger);

    }
}
