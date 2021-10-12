package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;

@Autonomous(name = "Rithwick_E")
public class Hi extends LinearOpMode { // Define motors
    DcMotor LV;
    DcMotor RV;
    DcMotor LH;
    DcMotor RH;
    DcMotor TOP;

    public void runOpMode() {   //Pull Configure
        LV = hardwareMap.dcMotor.get("lr");
        RV = hardwareMap.dcMotor.get("rf");
        LH = hardwareMap.dcMotor.get("lf");
        RH = hardwareMap.dcMotor.get("rr");
        RH = hardwareMap.dcMotor.get("rr");
        //Reverse inputs for left motors
        LV.setDirection(DcMotorSimple.Direction.REVERSE);
        RH.setDirection(DcMotorSimple.Direction.REVERSE);

        LV.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RV.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LH.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RH.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        if (opModeIsActive()) {


            DriveTW(0.5,10);


            /*
                DriveFB(0.5, 12);
                DriveSLR(0.5, 24);
                DriveSTS(0.5, -12);
                DriveFB(0.5, 108);
                DriveSTS(0.5, 1);
                DriveSLR(0.5, 5);
                DriveSLR(0.5, -5);
                DriveSTS(0.5, -1);
                DriveFB(0.5, -108);*/

        }
    }

    public void DriveFB(double power, int distance) {   //Reset Encoders
        LV.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RV.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Set Position
        LV.setTargetPosition(distance * 31);
        RV.setTargetPosition(distance * 31);

        //Movement
        LV.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RV.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        LV.setPower(power);
        RV.setPower(power);

        while (LV.isBusy() && RV.isBusy()) {
        }
        //Resets all input data
        StopDriving();
        LV.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RV.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void DriveTW(double power, int distance) {   //Reset Encoders
        TOP.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Set Position
        TOP.setTargetPosition(distance * 31);

        //Movement
        TOP.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        TOP.setPower(power);

        while (TOP.isBusy()) {
        }
        //Resets all input data
        StopDriving();
        TOP.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void DriveSTS(double power, int distance2) {   //Reset Encoders
        LH.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RH.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Set Position
        LH.setTargetPosition(distance2 * 31);
        RH.setTargetPosition(distance2 * 31);

        //Movement
        LH.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RH.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        LH.setPower(power);
        RH.setPower(power);

        while (LV.isBusy() && RV.isBusy()) {
        }
        //Resets all input data
        StopDriving();
        LV.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RV.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void DriveSLR(double power, int distance3) {   //Reset Encoders
        LV.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RV.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LH.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RH.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Set Position
        LV.setTargetPosition(distance3 * 31);
        RV.setTargetPosition(distance3 * 31);
        LH.setTargetPosition(distance3 * 31);
        RH.setTargetPosition(distance3 * 31);

        //Movement
        LV.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RV.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LH.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RH.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        LV.setPower(power);
        RV.setPower(power);
        LH.setPower(power);
        RH.setPower(power);

        while (LV.isBusy() && RV.isBusy() && LV.isBusy() && RV.isBusy()) {
        }
        //Resets all input data
        StopDriving();
        LV.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RV.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LH.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RH.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void StopDriving() {
        //Stops motors
        LV.setPower(0);
        RV.setPower(0);
        RH.setPower(0);
        LH.setPower(0);
        TOP.setPower(0);
    }
}