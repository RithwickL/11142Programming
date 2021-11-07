package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous(name="MarkerPosition_2")
public class MarkerPosition_2 extends LinearOpMode {    //Declare motors
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

        //set modes
        Fvertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Bvertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Fhorizontal.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Bhorizontal.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
    }



















































































}


