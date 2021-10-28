package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name="EncoderNishita")
class EncoderNishita extends LinearOpMode {    //Declare motors
DcMotor Fvertical;
DcMotor Fhorizontal;
DcMotor Bvertical;
DcMotor Bhorizontal;

public void runOpMode() { //code will run once only

    //initilize motors
    Fvertical = hardwareMap.dcMotor.get("lr");
    Bvertical = hardwareMap.dcMotor.get("rf");
    Fhorizontal = hardwareMap.dcMotor.get("lf");
    Bhorizontal = hardwareMap.dcMotor.get("rr");

    Bhorizontal.setDirection(DcMotorSimple.Direction.REVERSE);
    Bvertical.setDirection(DcMotorSimple.Direction.REVERSE);


    //set modes
    Fvertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    Bvertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    Fhorizontal.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    Bhorizontal.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    waitForStart();

    if (opModeIsActive()) {
        DriveForward(0.5, 50);
        DriveSide(0.5, 50);
        DriveBack(0.5, -50);
        SideReverse(0.5, -50);
    }
}
    public void DriveForward (double power, int distance) {
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

        while (Fvertical.isBusy() && Bvertical.isBusy()){

        }

    }
    public void DriveSide (double power, int distance){
       //reset
        Fhorizontal.setMode(RunMode.STOP_AND_RESET_ENCODER);
        Bhorizontal.setMode(RunMode.STOP_AND_RESET_ENCODER);

      //target position
      Fhorizontal.setTargetPosition(distance * 31);
      Bhorizontal.setTargetPosition(distance * 31);

      Fhorizontal.setMode(RunMode.RUN_TO_POSITION);
      Bhorizontal.setMode(RunMode.RUN_TO_POSITION);

      //set power
        Fhorizontal.setPower(power);
        Bhorizontal.setPower(power);

       while (Fhorizontal.isBusy() && Bhorizontal.isBusy()) {

       }
    }
    public void DriveBack (double power, int distance){
    //reset
        Fvertical.setMode(RunMode.STOP_AND_RESET_ENCODER);
        Bvertical.setMode(RunMode.STOP_AND_RESET_ENCODER);

    //target position
        Fvertical.setTargetPosition(distance*31);
        Bvertical.setTargetPosition(distance*31);

        Fvertical.setMode(RunMode.RUN_TO_POSITION);
        Bvertical.setMode(RunMode.RUN_TO_POSITION);

    //set power
        Fvertical.setPower(power);
        Bvertical.setPower(power);

        while (Fvertical.isBusy() && Bvertical.isBusy()) {

        }
    }
    public void SideReverse (double power, int distance){
    //reset
        Fhorizontal.setMode(RunMode.STOP_AND_RESET_ENCODER);
        Bhorizontal.setMode(RunMode.STOP_AND_RESET_ENCODER);

    //target position
        Fhorizontal.setTargetPosition(distance*31);
        Bhorizontal.setTargetPosition(distance*31);

        Fhorizontal.setMode(RunMode.RUN_TO_POSITION);
        Bhorizontal.setMode(RunMode.RUN_TO_POSITION);

    //set power
        Fhorizontal.setPower(power);
        Bhorizontal.setPower(power);

        while (Fhorizontal.isBusy() && Bhorizontal.isBusy()) {

        }

        StopDriving();

        Fvertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Bvertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Fhorizontal.setMode(RunMode.RUN_WITHOUT_ENCODER);
        Bhorizontal.setMode(RunMode.RUN_USING_ENCODER);

    }
    public void StopDriving(){

    Fvertical.setPower(0);
    Fhorizontal.setPower(0);
    Bvertical.setPower(0);
    Bhorizontal.setPower(0);
    }
}
