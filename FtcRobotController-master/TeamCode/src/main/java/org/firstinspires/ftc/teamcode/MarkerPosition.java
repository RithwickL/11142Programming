/*package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous(name="MarkerPosition")
public class MarkerPosition extends LinearOpMode {    //Declare motors
    DcMotor Fvertical;
    DcMotor Fhorizontal;
    DcMotor Bvertical;
    DcMotor Bhorizontal;
    DcMotor Arm1;
    DcMotor Arm2;
    DcMotor Slide;
    Servo Claw1;
    Servo Claw2

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


        if (condition1) {
            DriveForwardA(0.5, 25);
            PositionA(0.5, 0);
            SlideA (0.5, 0);
        } else if (condition2) {
            DriveForwardB(0.5, 25);
            PositionB(0.5, 0);
            SlideA (0.5, 0);
        } else (condition3) {
            DriveForwardC(0.5, 25);
            PositionC(0.5, 0);
            SlideC(0.5, 0);
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

    //public void PositionA(double power, int distance) {
        //reset encoder
       //Arm1.setMode(RunMode.STOP_AND_RESET_ENCODER);
        //Arm2.setMode(RunMode.STOP_AND_RESET_ENCODER);

        //set target position
        //Arm1.setTargetPosition(distance * 31);
        //Arm2.setTargetPosition(distance * 31);

        //Arm1.setMode(RunMode.RUN_TO_POSITION);
        //Arm2.setMode(RunMode.RUN_TO_POSITION);


        //set power
        //Arm1.setPower(power);
        //Arm2.setPower(power);


        //while (Arm1.isBusy() && Arm2.isBusy()) {

        }
    }

    public void SlideA(double power, int distance) {
        //reset
        Slide.setMode(RunMode.STOP_AND_RESET_ENCODER);

        //target position
        Slide.setTargetPosition(distance * 31);

        Slide.setMode(RunMode.RUN_TO_POSITION);

        //set power
        Slide.setPower(power);

        while (Slide.isBusy()){

        }

    }


        public void DriveForwardB(double power, int distance) {
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
        public void PositionB (double power, int distance ) {
            //reset encoder
            Arm1.setMode(RunMode.STOP_AND_RESET_ENCODER);
            Arm2.setMode(RunMode.STOP_AND_RESET_ENCODER);

            //set target position
            Arm1.setTargetPosition(distance * 31);
            Arm2.setTargetPosition(distance * 31);

            Arm1.setMode(RunMode.RUN_TO_POSITION);
            Arm2.setMode(RunMode.RUN_TO_POSITION);

            //set power
            Arm1.setPower(power);
            Arm2.setPower(power);

            while (Arm1.isBusy() && Arm2.isBusy()) {

            }
        }


        public void SlideB(double power, int distance) {
             //reset
                Slide.setMode(RunMode.STOP_AND_RESET_ENCODER);

             //target position
                Slide.setTargetPosition(distance * 31);

                Slide.setMode(RunMode.RUN_TO_POSITION);

                //set power
                Slide.setPower(power);

                while (Slide.isBusy()){

                }
        }
    public void DriveForwardC(double power, int distance) {
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
    public void PositionC (double power, int distance ) {
        //reset encoder
        Arm1.setMode(RunMode.STOP_AND_RESET_ENCODER);
        Arm2.setMode(RunMode.STOP_AND_RESET_ENCODER);

        //set target position
        Arm1.setTargetPosition(distance * 31);
        Arm2.setTargetPosition(distance * 31);

        Arm1.setMode(RunMode.RUN_TO_POSITION);
        Arm2.setMode(RunMode.RUN_TO_POSITION);

        //set power
        Arm1.setPower(power);
        Arm2.setPower(power);

        while (Arm1.isBusy() && Arm2.isBusy()) {
        }
    }

    public void SlideC(double power, int distance) {
        //reset
        Slide.setMode(RunMode.STOP_AND_RESET_ENCODER);

        //target position
        Slide.setTargetPosition(distance * 31);

        Slide.setMode(RunMode.RUN_TO_POSITION);

        //set power
        Slide.setPower(power);

        while (Slide.isBusy()) {

        }
    }
}*/






