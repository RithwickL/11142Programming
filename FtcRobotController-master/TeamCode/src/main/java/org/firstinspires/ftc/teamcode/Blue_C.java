package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name="Blue Carousel/Park")
public class Blue_C extends LinearOpMode {    //Declare motors
    DcMotorEx Fvertical;
    DcMotorEx Fhorizontal;
    DcMotorEx Bvertical;
    DcMotorEx Bhorizontal;
    DcMotor Top;
    DcMotor Arm1;
    //DcMotor Slide;
    DcMotor Pick;

    public void runOpMode() { //code will run once only

        //initilize motors
        Fvertical = (DcMotorEx) hardwareMap.dcMotor.get("lr");
        Bvertical = (DcMotorEx) hardwareMap.dcMotor.get("rf");
        Fhorizontal = (DcMotorEx) hardwareMap.dcMotor.get("lf");
        Bhorizontal = (DcMotorEx) hardwareMap.dcMotor.get("rr");
        Arm1 = hardwareMap.dcMotor.get("Spin");
        Top = hardwareMap.dcMotor.get("TOP");
        //Slide = hardwareMap.dcMotor.get("Slide");
        Pick = hardwareMap.dcMotor.get("Pick");

        Bhorizontal.setDirection(DcMotor.Direction.REVERSE);
        Bvertical.setDirection(DcMotor.Direction.REVERSE);

        Fvertical.setMode(RunMode.STOP_AND_RESET_ENCODER);
        Bvertical.setMode(RunMode.STOP_AND_RESET_ENCODER);
        Fhorizontal.setMode(RunMode.STOP_AND_RESET_ENCODER);
        Bhorizontal.setMode(RunMode.STOP_AND_RESET_ENCODER);

        //set modes
        Fvertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Bvertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Fhorizontal.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Bhorizontal.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Top.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Pick.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Arm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        if (opModeIsActive()) {

            DriveForward(-1, -0.5,400);
            RobotSpin(-0.1, 5);
            Spin(0.05, -25);
            DriveSide(0.1,20);
            /*DriveForward(1, 0.5,500);
            RobotSpin(0.1, 15);
            DriveForward(-0.5, -0.25,1000);
            DriveForward(-0.2, -0.1,100);
            DriveSide(0.1,110);
            DriveSide(1,30);*/

        }
    }
    public void DriveForward (double power, double power2, int distance) {
        Fvertical.setPower(power);
        Bvertical.setPower(power2);
        sleep(distance);

        while (Fvertical.isBusy() && Bvertical.isBusy()){

        }
        StopDriving();

    }
    public void Arm (double power, int distance) {
        //reset encoder
        Arm1.setMode(RunMode.STOP_AND_RESET_ENCODER);


        //set target position
        Arm1.setTargetPosition(distance * 31);


        Arm1.setMode(RunMode.RUN_TO_POSITION);


        //set power
        Arm1.setPower(power);


        while (Arm1.isBusy()){

        }
        StopDriving();

    }
    public void RobotSpin (double power, int distance) {
        //reset encoder
        Fvertical.setMode(RunMode.STOP_AND_RESET_ENCODER);
        Bvertical.setMode(RunMode.STOP_AND_RESET_ENCODER);
        Fhorizontal.setMode(RunMode.STOP_AND_RESET_ENCODER);
        Bhorizontal.setMode(RunMode.STOP_AND_RESET_ENCODER);

        //set target position
        Fvertical.setTargetPosition(distance * -31);
        Bvertical.setTargetPosition(distance * 31);
        Fhorizontal.setTargetPosition(distance * -31);
        Bhorizontal.setTargetPosition(distance * 31);

        Fvertical.setMode(RunMode.RUN_TO_POSITION);
        Bvertical.setMode(RunMode.RUN_TO_POSITION);
        Fhorizontal.setMode(RunMode.RUN_TO_POSITION);
        Bhorizontal.setMode(RunMode.RUN_TO_POSITION);

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
        StopDriving();
    }

    public void Spin(double power, int distance) {
        //reset encoder
        Top.setMode(RunMode.STOP_AND_RESET_ENCODER);

        //set target position
        Top.setTargetPosition(distance * 31);
        Top.setMode(RunMode.RUN_TO_POSITION);

        //set power
        Top.setPower(power);

        while (Top.isBusy()) {

        }
        StopDriving();

    }
    public void StopDriving(){

        Fvertical.setPower(-1);
        Fhorizontal.setPower(-1);
        Bvertical.setPower(-1);
        Bhorizontal.setPower(-1);
        sleep(100);
        Fvertical.setPower(0);
        Fhorizontal.setPower(0);
        Bvertical.setPower(0);
        Bhorizontal.setPower(0);
        Top.setPower(0);
    }
}
