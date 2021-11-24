package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@Autonomous(name="Mec_Auto")
public class Mec_Auto extends LinearOpMode {    
    //Declare motorsnames
    DcMotor rightFront;
    DcMotor leftFront;
    DcMotor leftRear;
    DcMotor rightRear;
    DcMotor Top;
    DcMotor Arm1;
    //DcMotor Slide;
    DcMotor Pick;
    
    public void runOpMode() { //code will run once only
        //Config
        leftFront = hardwareMap.dcMotor.get("lf");
        leftRear = hardwareMap.dcMotor.get("lr");
        rightRear = hardwareMap.dcMotor.get("rr");
        rightFront = hardwareMap.dcMotor.get("rf");
        Arm1 = hardwareMap.dcMotor.get("arm");
        Top = hardwareMap.dcMotor.get("carousel");
        //Slide = hardwareMap.dcMotor.get("Slide");
        Pick = hardwareMap.dcMotor.get("intake");

        //this puts the motors in reverse
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        
        //set modes for encoders
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Top.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        if (opModeIsActive()) {
        //STUFF
        }
    }

    public void DriveSpin(double power, int distance) {
        //reset encoder
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        //set target position
        leftRear.setTargetPosition(distance * 31);
        rightRear.setTargetPosition(-distance * 31);
        leftFront.setTargetPosition(distance * 31);
        rightFront.setTargetPosition(-distance * 31);

        //Go to Position
        leftRear.setMode(RunMode.RUN_TO_POSITION);
        rightRear.setMode(RunMode.RUN_TO_POSITION);
        leftFront.setMode(RunMode.RUN_TO_POSITION);
        rightFront.setMode(RunMode.RUN_TO_POSITION);

        //set power
        leftRear.setPower(power);
        rightRear.setPower(power);
        leftFront.setPower(power);
        rightFront.setPower(power);

        while (leftRear.isBusy() && rightRear.isBusy() && leftFront.isBusy() && rightFront.isBusy()){

        }
        StopDriving();

    }

    public void DriveSlide(double power, int distance) {
        //reset encoder
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        //set target position
        leftRear.setTargetPosition(-distance * 31);
        rightRear.setTargetPosition(distance * 31);
        leftFront.setTargetPosition(distance * 31);
        rightFront.setTargetPosition(-distance * 31);

        //Go to Position
        leftRear.setMode(RunMode.RUN_TO_POSITION);
        rightRear.setMode(RunMode.RUN_TO_POSITION);
        leftFront.setMode(RunMode.RUN_TO_POSITION);
        rightFront.setMode(RunMode.RUN_TO_POSITION);

        //set power
        leftRear.setPower(power);
        rightRear.setPower(power);
        leftFront.setPower(power);
        rightFront.setPower(power);

        while (leftRear.isBusy() && rightRear.isBusy() && leftFront.isBusy() && rightFront.isBusy()){

        }
        StopDriving();

    }

    public void Caro(double power, int distance) {
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

    public void Arm(double power, int distance) {
        //reset encoder
        Arm1.setMode(RunMode.STOP_AND_RESET_ENCODER);

        //set target position
        Arm1.setTargetPosition(distance * 31);

        //Go to Pos
        Arm1.setMode(RunMode.RUN_TO_POSITION);

        //set power
        Arm1.setPower(power);
        //Wait
        while (Arm1.isBusy()){}
        StopDriving();
    }
    public void StopDriving(){

        leftRear.setPower(0);
        rightRear.setPower(0);
        leftFront.setPower(0);
        rightFront.setPower(0);
        Top.setPower(0);
    }
}
