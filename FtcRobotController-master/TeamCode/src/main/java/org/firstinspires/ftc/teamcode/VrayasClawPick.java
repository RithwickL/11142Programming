package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name="Claw")
public class VrayasClawPick extends LinearOpMode
{
    DcMotor leftvertical;
    DcMotor rightvertical;
    DcMotor lefthorizontal;
    DcMotor righthorizontal;
    DcMotor Top;
    DcMotor Arm1;
    DcMotor Arm2;
    Servo Finger1;
    Servo Finger2;

    public void runOpMode() {

        //Motors & Servos
        leftvertical = hardwareMap.dcMotor.get("lf");
        rightvertical = hardwareMap.dcMotor.get("rr");
        lefthorizontal = hardwareMap.dcMotor.get("lr");
        righthorizontal = hardwareMap.dcMotor.get("rf");
        Arm1 = hardwareMap.dcMotor.get("Spin1");
        Arm2 = hardwareMap.dcMotor.get("Spin2");
        Top = hardwareMap.dcMotor.get("TOP");
        Finger1 = hardwareMap.servo.get("");
        Finger2 = hardwareMap.servo.get("");

        righthorizontal.setDirection(DcMotorSimple.Direction.REVERSE);
        rightvertical.setDirection(DcMotorSimple.Direction.REVERSE);


        //set modes
        Fvertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Bvertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Fhorizontal.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Bhorizontal.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Top.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Arm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Arm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        waitForStart();

        if (opModeIsActive()) {

            DriveForward(0.2, -42);
            Spin(0.2, 25);
            DriveForward(0.5, 42);
            DriveSide(0.5,10);
            RobotSpin(0.5,50);
            DriveForward(0.5, -10);
            DriveSide(0.5,-70);
        }
    }
    public void DriveForward (double power, int distance) {
        //reset encoder
        Fvertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Bvertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //set target position
        Fvertical.setTargetPosition(distance * 31);
        Bvertical.setTargetPosition(distance * 31);

        Fvertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Bvertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //set power
        Fvertical.setPower(power);
        Bvertical.setPower(power);

        while (Fvertical.isBusy() && Bvertical.isBusy()){

        }
        StopDriving();

    }
    public void RobotSpin (double power, int distance) {
        //reset encoder
        Fvertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Bvertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Fhorizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Bhorizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //set target position
        Fvertical.setTargetPosition(distance * 31);
        Bvertical.setTargetPosition(distance * 31);
        Fhorizontal.setTargetPosition(distance * 31);
        Bhorizontal.setTargetPosition(distance * 31);

        Fvertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Bvertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Fhorizontal.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Bhorizontal.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //set power
        Fvertical.setPower(power);
        Bvertical.setPower(power);
        Fhorizontal.setPower(power);
        Bhorizontal.setPower(power);

        while (Fvertical.isBusy() && Bvertical.isBusy() && Fhorizontal.isBusy() && Bhorizontal.isBusy()){

        }
        StopDriving();

    }
    public void DriveSide (double power, int distance){
        //reset
        Fhorizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Bhorizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //target position
        Fhorizontal.setTargetPosition(distance * 31);
        Bhorizontal.setTargetPosition(distance * 31);

        Fhorizontal.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Bhorizontal.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //set power
        Fhorizontal.setPower(power);
        Bhorizontal.setPower(power);

        while (Fhorizontal.isBusy() && Bhorizontal.isBusy()) {

        }
        StopDriving();
    }

    public void Spin(double power, int distance) {
        //reset encoder
        Top.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //set target position
        Top.setTargetPosition(distance * 31);
        Top.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //set power
        Top.setPower(power);

        while (Top.isBusy()) {

        }
        StopDriving();

    }
    public void StopDriving(){

        Fvertical.setPower(0);
        Fhorizontal.setPower(0);
        Bvertical.setPower(0);
        Bhorizontal.setPower(0);
        Top.setPower(0);
    }

}
