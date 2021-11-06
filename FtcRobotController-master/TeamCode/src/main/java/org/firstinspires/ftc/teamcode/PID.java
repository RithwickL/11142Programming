package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="PID" , group="Main")
public class PID extends OpMode {
    DcMotor leftvertical;
    DcMotor rightvertical;
    DcMotor lefthorizontal;
    DcMotor righthorzontal;
    DcMotor Top;
    DcMotor Arm1;
    double integral = 0;
    ElapsedTime PIDTimer = new ElapsedTime();

    public void init() {
        Rithwick_Gyro rg =  new Rithwick_Gyro();
        rg.getAngles();

        leftvertical = hardwareMap.dcMotor.get("lf");
        rightvertical = hardwareMap.dcMotor.get("rr");
        lefthorizontal = hardwareMap.dcMotor.get("lr");
        righthorzontal = hardwareMap.dcMotor.get("rf");
        //Claw = hardwareMap.servo.get("Claw");
        Arm1 = hardwareMap.dcMotor.get("Spin1");
        //Arm2 = hardwareMap.dcMotor.get("Spin2");
        Top = hardwareMap.dcMotor.get("TOP");
        leftvertical.setDirection(DcMotorSimple.Direction.REVERSE);
        lefthorizontal.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    void Arm_1(double targetPosition){
         double error = Arm1.getCurrentPosition();
         double lastError = 0;
        double Kp = 0;
        double Ki = 0;
        double Kd = 0;
        while(Math.abs(error) <= 4){
            error = Arm1.getCurrentPosition() - targetPosition;
            double changeInError = lastError - error;
            integral += changeInError * PIDTimer.time();
            double derivative = changeInError / PIDTimer.time();
            double P = Kp * error;
            double I = Ki * integral;
            double D = Kd * derivative;
            Arm1.setPower(P+I+D);
            error = lastError;
            PIDTimer.reset();
        }
    }
    public void loop() {
        double Arm1PS = Arm1.getCurrentPosition();
        //Left and Right - right stick - Right/Left
        leftvertical.setPower(gamepad1.right_stick_x);
        rightvertical.setPower(gamepad1.right_stick_x);
        //Forward and Backward - left stick - up/down
        lefthorizontal.setPower(gamepad1.right_stick_y);
        righthorzontal.setPower(gamepad1.right_stick_y);

        //Spin from center orign2 - Left stick- Left/right
        lefthorizontal.setPower(gamepad1.left_trigger / 2);
        righthorzontal.setPower(-gamepad1.left_trigger / 2);
        leftvertical.setPower(gamepad1.left_trigger / 2);
        rightvertical.setPower(-gamepad1.left_trigger / 2);

        lefthorizontal.setPower(-gamepad1.right_trigger / 2);
        righthorzontal.setPower(gamepad1.right_trigger / 2);
        leftvertical.setPower(-gamepad1.right_trigger / 2);
        rightvertical.setPower(gamepad1.right_trigger / 2);

        if (gamepad1.b) {
            Arm1.setPower(-0.25);
        }
        else {
            Arm_1(Arm1PS);
        }

        if (gamepad1.y) {
            Arm1.setPower(0.25);

        }

        if (gamepad1.a) {
            Top.setPower(0.5);
        } else {
            Top.setPower(0);
        }

        /*if (gamepad1.x == true) {

        } else {

        }*/

        if (gamepad1.x == true) {
            //Claw.setPosition(90);
        } else {
            //Claw.setPosition(0);

        }
        /*if (gamepad1.right_bumper == true) {

        } else {

        }*/
    }
}