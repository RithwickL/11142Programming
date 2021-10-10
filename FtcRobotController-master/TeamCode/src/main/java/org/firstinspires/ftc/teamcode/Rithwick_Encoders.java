package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;


@Autonomous
public abstract class Rithwick_Encoders extends OpMode {
    DcMotor leftvertical = null;
    DcMotor rightvertical = null;
    DcMotor lefthorizontal = null;
    DcMotor righthorzontal = null;
    /*Servo Front;
    DcMotor Intake;*/
//1S2D

    public void init() {
        leftvertical = hardwareMap.dcMotor.get("lf");
        rightvertical = hardwareMap.dcMotor.get("rr");
        lefthorizontal = hardwareMap.dcMotor.get("lr");
        righthorzontal = hardwareMap.dcMotor.get("rf");
        //Front = hardwareMap.servo.get("blocker");//
        leftvertical.setDirection(DcMotorSimple.Direction.REVERSE);
        lefthorizontal.setDirection(DcMotorSimple.Direction.REVERSE);
        //Intake = hardwareMap.dcMotor.get("intake");//
        //Runs withouth encoders but still reads info
        //lefthorizontal.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        //Uses encoderes
        //lefthorizontal.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        //Goes to a postion
        //lefthorizontal.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        //Resets/Callabrates encoders
        //lefthorizontal.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        leftvertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightvertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lefthorizontal.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        righthorzontal.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //Gets position
        //leftvertical.getCurrentPosition();
        //Set a postion to go to
        //leftvertical.setTargetPosition();
        //CHecks to see is motor is moving
        //leftvertical.isBusy();
    }

    public void DriveFBD(int distance) {
        lefthorizontal.setMode(RunMode.STOP_AND_RESET_ENCODER);
        righthorzontal.setMode(RunMode.STOP_AND_RESET_ENCODER);

        lefthorizontal.setTargetPosition(10);
        righthorzontal.setTargetPosition(10);

        lefthorizontal.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        righthorzontal.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (lefthorizontal.isBusy() || righthorzontal.isBusy()) {

        }
        lefthorizontal.setMode(RunMode.RUN_USING_ENCODER);
        righthorzontal.setMode(RunMode.RUN_USING_ENCODER);
    }


        public void loop() {

        }
    }
