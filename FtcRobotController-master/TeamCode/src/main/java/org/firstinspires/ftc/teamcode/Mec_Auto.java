package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name="Mec_Auto")
public class Mec_Auto extends LinearOpMode {    
    //Declare motorsnames
    DcMotor BackLeft;
    DcMotor BackRight;
    DcMotor FrontLeft;
    DcMotor FrontRight;
    DcMotor Top;
    DcMotor Arm1;
    DcMotor Pick;
    
    public void runOpMode() { //code will run once only
        //Config
        BackLeft = hardwareMap.dcMotor.get("lf");
        BackRight = hardwareMap.dcMotor.get("rr");
        FrontLeft = hardwareMap.dcMotor.get("lr");
        FrontRight = hardwareMap.dcMotor.get("rf");
        Arm1 = hardwareMap.dcMotor.get("Spin");
        Top = hardwareMap.dcMotor.get("TOP");
        Pick = hardwareMap.dcMotor.get("Pick");
        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        Arm1.setDirection(DcMotorSimple.Direction.REVERSE);
        
        //set modes for encoders
        BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Top.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        if (opModeIsActive()) {
        //STUFF
        }
    }

    public void DriveSpin(double power, int distance) {
        //reset encoder
        BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        //set target position
        BackLeft.setTargetPosition(distance * 31);
        BackRight.setTargetPosition(-distance * 31);
        FrontLeft.setTargetPosition(distance * 31);
        FrontRight.setTargetPosition(-distance * 31);

        //Go to Position
        BackLeft.setMode(RunMode.RUN_TO_POSITION);
        BackRight.setMode(RunMode.RUN_TO_POSITION);
        FrontLeft.setMode(RunMode.RUN_TO_POSITION);
        FrontRight.setMode(RunMode.RUN_TO_POSITION);

        //set power
        BackLeft.setPower(power);
        BackRight.setPower(power);
        FrontLeft.setPower(power);
        FrontRight.setPower(power);

        while (BackLeft.isBusy() && BackRight.isBusy() && FrontLeft.isBusy() && FrontRight.isBusy()){

        }
        StopDriving();

    }

    public void DriveSlide(double power, int distance) {
        //reset encoder
        BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        //set target position
        BackLeft.setTargetPosition(-distance * 31);
        BackRight.setTargetPosition(distance * 31);
        FrontLeft.setTargetPosition(distance * 31);
        FrontRight.setTargetPosition(-distance * 31);

        //Go to Position
        BackLeft.setMode(RunMode.RUN_TO_POSITION);
        BackRight.setMode(RunMode.RUN_TO_POSITION);
        FrontLeft.setMode(RunMode.RUN_TO_POSITION);
        FrontRight.setMode(RunMode.RUN_TO_POSITION);

        //set power
        BackLeft.setPower(power);
        BackRight.setPower(power);
        FrontLeft.setPower(power);
        FrontRight.setPower(power);

        while (BackLeft.isBusy() && BackRight.isBusy() && FrontLeft.isBusy() && FrontRight.isBusy()){

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

        BackLeft.setPower(0);
        BackRight.setPower(0);
        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        Top.setPower(0);
    }
}
