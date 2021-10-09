package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@Autonomous
public class Rithwick_Encoders extends OpMode {
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
        leftvertical.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        rightvertical.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        lefthorizontal.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        righthorzontal.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        //Gets position
        //leftvertical.getCurrentPosition();
        //Set a postion to go to
        //leftvertical.setTargetPosition();
        //CHecks to see is motor is moving
        //leftvertical.isBusy();
    }

    public void DriveFBD(double power, int distance) {
        lefthorizontal.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        righthorzontal.setMode(DcMotorController.RunMode.RESET_ENCODERS);

        lefthorizontal.setTargetPosition(distance);
        righthorzontal.setTargetPosition(distance);

        lefthorizontal.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        righthorzontal.setMode(DcMotorController.RunMode.RUN_TO_POSITION);

        while (lefthorizontal.isBusy() || righthorzontal.isBusy()) {

        }
        lefthorizontal.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        righthorzontal.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
    }

        public void loop() {

        }
    }
}