/*package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;


@Disabled
@Autonomous(name="MarkerPosition")
public class MarkerPosition extends LinearOpMode {    //Declare motors
    DcMotor Fvertical;
    DcMotor Fhorizontal;
    DcMotor Bvertical;
    DcMotor Bhorizontal;
    DcMotor UpDown;
    DcMotor Claw;

    public void runOpMode() { //code will run once only

        //initilize motors
        Fvertical = hardwareMap.dcMotor.get("lr");
        Bvertical = hardwareMap.dcMotor.get("rf");
        Fhorizontal = hardwareMap.dcMotor.get("lf");
        Bhorizontal = hardwareMap.dcMotor.get("rr");
        UpDown = hardwareMap.dcMotor.get("Spin");
        Claw = hardwareMap.dcMotor.get("pick");


        //set modes
        Fvertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Bvertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Fhorizontal.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Bhorizontal.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        UpDown.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Claw.setMode(RunMode.RUN_USING_ENCODER);


        waitForStart();
        if (top) {
            DriveForwardA(0.5, 25);
            UpDown(0.5, 25);
            Claw(0.5, 25);
            Claw(-0.5, 50);
        } else if (middle) {
            DriveForwardA(0.5, 50);
            UpDown(0.5, 50);
            Claw(0.5, 50);
            Claw(-0.5, 50);
        } else (bottom) {
            DriveForwardA(0.5, 75);
            UpDown(0.5, 50);
            Claw(0.5, 50);
            Claw(-0.5, 50);
        }
    }


    public void DriveForwardA(double power, int distance) {
        //reset encoder
        Fvertical.setMode(RunMode.STOP_AND_RESET_ENCODER);
        Bvertical.setMode(RunMode.STOP_AND_RESET_ENCODER);


        //set target position
        Fvertical.setTargetPosition(distance * 31);
        Bvertical.setTargetPosition(distance * 31);

        Fvertical.setMode(RunMode.RUN_TO_POSITION);
        Bvertical.setMode(RunMode.RUN_TO_POSITION);

        //set power
        Fvertical.setPower(power);
        Bvertical.setPower(power);

        while (Fvertical.isBusy() && Bvertical.isBusy()) {
        }
    }

    public void UpDown(double power, int distance) {

        //reset encoder
        UpDown.setMode(RunMode.STOP_AND_RESET_ENCODER);
        //set target position
        UpDown.setTargetPosition(distance * 31);
        UpDown.setMode(RunMode.RUN_TO_POSITION);
        //set power;
        UpDown.setPower(power);

        while (UpDown.isBusy()) {
        }
    }

    public void Claw(double power, int distance) {

        //reset encoder
        Claw.setMode(RunMode.STOP_AND_RESET_ENCODER);
        //set target position
        Claw.setTargetPosition(distance * 31);
        Claw.setMode(RunMode.RUN_TO_POSITION);
        //set power;
        Claw.setPower(power);

        while (Claw.isBusy()) {
        }
    }
}*/


