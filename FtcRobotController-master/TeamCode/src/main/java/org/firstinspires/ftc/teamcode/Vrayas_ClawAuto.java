/*package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@Autonomous(name="ClawAutoVP")
public class Vrayas_ClawAuto extends LinearOpMode {
    DcMotor leftvertical;
    DcMotor rightvertical;
    DcMotor lefthorizontal;
    DcMotor righthorizontal;
    DcMotor Slide;
    DcMotor Arm1;
    DcMotor Arm2;
    Servo Finger;

    public class TopDrop
    {
        if (opModeIsActive())
        {
            DriveForward(0.2, 21);
        }
        else


    }


    public void runOpMode() {
        leftvertical = hardwareMap.dcMotor.get("lf");
        rightvertical = hardwareMap.dcMotor.get("rr");
        lefthorizontal = hardwareMap.dcMotor.get("lr");
        righthorizontal = hardwareMap.dcMotor.get("rf");

        lefthorizontal.setDirection(DcMotorSimple.Direction.REVERSE);
        leftvertical.setDirection(DcMotorSimple.Direction.REVERSE);

        Arm1 = hardwareMap.dcMotor.get("Spin1");
        Arm2 = hardwareMap.dcMotor.get("Spin2");

        Slide = hardwareMap.dcMotor.get("");

        Finger = hardwareMap.servo.get("");

        //Run Motors with Encoders
        leftvertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightvertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lefthorizontal.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        righthorizontal.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Arm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Arm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
// top

    }

    switch("TensorFlowInput")
    {
        case "Top":
            DriveForward(1,15);
            DriveSide(1,15);
            break;
        case "Middle":

            // code block
            break;
        default: //Last
            // code block
    }
    public void DriveForward(double power, int distance) {
        leftvertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightvertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Move to Position
        leftvertical.setTargetPosition(distance * 31);
        rightvertical.setTargetPosition(distance * 31);

        leftvertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightvertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Set Power
        leftvertical.setPower(power);
        rightvertical.setPower(power);

        while (leftvertical.isBusy() && rightvertical.isBusy()) {

            StopDriving();
        }
    }

    public void DriveSide(double power, int distance) {
        lefthorizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        righthorizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Move to Position
        lefthorizontal.setTargetPosition(distance * 31);
        righthorizontal.setTargetPosition(distance * 31);

        lefthorizontal.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        righthorizontal.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Set Power
        lefthorizontal.setPower(power);
        righthorizontal.setPower(power);

        while (lefthorizontal.isBusy() && righthorizontal.isBusy()) {

            StopDriving();
        }
    }

    public void Drive

    public void StopDriving(){

        leftvertical.setPower(0);
        righthorizontal.setPower(0);
        rightvertical.setPower(0);
        lefthorizontal.setPower(0);
        Slide.setPower(0);
        Arm1.setPower(0);
        Arm2.setPower(0);
    }
}*/

