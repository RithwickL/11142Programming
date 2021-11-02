package org.firstinspires.ftc.teamcode;
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
    Servo Finger;

    public void runOpMode() { //code will run once only

        //initilize motors
        Fvertical = hardwareMap.dcMotor.get("lr");
        Bvertical = hardwareMap.dcMotor.get("rf");
        Fhorizontal = hardwareMap.dcMotor.get("lf");
        Bhorizontal = hardwareMap.dcMotor.get("rr");
        Arm1= hardwareMap.dcMotor.get("Spin1");
        Arm2 = hardwareMap.dcMotor.get("Spin2");
        Slide = hardwareMap.dcMotor.get("");
        Finger = hardwareMap.servo.get("");



        Bhorizontal.setDirection(DcMotorSimple.Direction.REVERSE);
        Bvertical.setDirection(DcMotorSimple.Direction.REVERSE);


        //set modes
        Fvertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Bvertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Fhorizontal.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Bhorizontal.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Arm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Arm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Slide.setMode(RunMode.RUN_USING_ENCODER);

        waitForStart();


        if (opModeIsActive()) {
            DriveForward(0.5, 50);
            PositionA(0.5, 0);
        }
    }

    public void DriveForward(double power, int distance) {
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

    public void PositionA(double power, int distance){
        //reset encoder
        Arm1.setMode(RunMode.STOP_AND_RESET_ENCODER);
        Arm2.setMode(RunMode.STOP_AND_RESET_ENCODER);
        Slide.setMode(RunMode.STOP_AND_RESET_ENCODER);

        //set target position
        Arm1.setTargetPosition(distance * 31);
        Arm2.setTargetPosition(distance * 31);
        Slide.setTargetPosition(distance * 31);

        Arm1.setMode(RunMode.RUN_TO_POSITION);
        Arm2.setMode(RunMode.RUN_TO_POSITION);
        Slide.setMode(RunMode.RUN_TO_POSITION);

        //set power
        Arm1.setPower(power);
        Arm2.setPower(power);
        Slide.setPower(power);
    }
}




