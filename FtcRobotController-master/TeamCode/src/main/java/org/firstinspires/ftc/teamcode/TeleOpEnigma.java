package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import static java.lang.Math.max;
import static java.lang.Math.min;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.List;

/*
 * TeleOp Version 1 For Ball Drive
 */
@TeleOp(name = "TeleOp BD")
public class TeleOpEnigma extends OpMode
{
    //Initializing all necessary variables
    float y;
    float x;

    public DcMotor armShoulder, armExtend;
    public DcMotor leftFront, leftRear, rightRear, rightFront;
    public List<DcMotorEx> motors;
    public Servo right, left, dispenser;

    private double liftPosScale = 25, liftPowScale = 0.0025;
    private double liftPosCurrent=0, liftPosDes=0, liftPosError=0, liftPow=0;
    double multiplier = 1;
    

    //This is where all the variables used in the main program are initialized
    @Override
    public void init()
    {
        //Assigning gamepad values to hardware parts
        y = gamepad1.left_stick_y;
        x = gamepad1.right_stick_x;


        //Defining all the hardware parts
        //Quoted name is entered into robot config only
        rightFront = hardwareMap.dcMotor.get("rf");
        leftRear = hardwareMap.dcMotor.get("lr");
        rightRear = hardwareMap.dcMotor.get("rr");
        leftFront = hardwareMap.dcMotor.get("lf");

        //Switch to control hub ports
        armExtend = hardwareMap.dcMotor.get("arm"); //changed from arme to arm
        armShoulder = hardwareMap.dcMotor.get("intake"); //changed from arms to intake

        //right = hardwareMap.servo.get("right");
        //left = hardwareMap.servo.get("left");


        //Setting the direction of each motor
        //Opposite side motors are reversed to move in the same direction
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);                                  //alternating between forward and reverse depending on motor placement
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.FORWARD);
        armExtend.setDirection(DcMotorSimple.Direction.FORWARD);

        armExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armExtend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    @Override
    public void loop()
    {
        //Initializing wheel variables at a constant
        float powerXWheels = 0;
        float powerYWheels = 0;

        //Handle regular movement
        powerYWheels += gamepad1.left_stick_y;

        //Handle sliding movement
        powerXWheels += gamepad1.right_stick_x;

        // Handle turning movement
        double maxX = (double) powerXWheels;
        double maxY = (double) powerYWheels;

        //Give X motors at most 75% power
        leftRear.setPower((Math.abs(maxX)*maxX) * -0.75);
        rightFront.setPower((Math.abs(maxX)*maxX) * -0.75);

        // Give Y motors at most 75% power
        rightRear.setPower((Math.abs(maxY)*maxY) * 0.75);
        leftFront.setPower((Math.abs(maxY)*maxY) * 0.75);

        //Turning clockwise with a 50% power scale on the motors
        if (gamepad1.right_trigger > 0)
        {
            rightFront.setPower(0.5 * gamepad1.right_trigger);
            leftFront.setPower(0.5 * gamepad1.right_trigger);
            leftRear.setPower(-0.5 * gamepad1.right_trigger);
            rightRear.setPower(-0.5 * gamepad1.right_trigger);
        }

        //Turning anticlockwise with a 50% power scale on the motors
        if (gamepad1.left_trigger > 0)
        {

            rightFront.setPower(-0.5 * gamepad1.left_trigger);
            leftFront.setPower(-0.5 * gamepad1.left_trigger);
            leftRear.setPower(0.5 * gamepad1.left_trigger);
            rightRear.setPower(0.5 * gamepad1.left_trigger);
        }

        //Hold X motor position when no power supplied
        if (gamepad1.right_stick_x == 0 && gamepad1.left_trigger == 0 && gamepad1.right_trigger == 0)
        {
            rightFront.setPower(0);
            leftRear.setPower(0);
        }

        //Hold Y motor position when no power supplied
        if (gamepad1.left_stick_y == 0 && gamepad1.left_trigger == 0 && gamepad1.right_trigger == 0)
        {
            leftFront.setPower(0);
            rightRear.setPower(0);
        }

        //Give shoulder positive power to move up
        if (gamepad2.right_stick_y > 0) {
            armShoulder.setPower(gamepad2.right_stick_y * 0.75);
        }
        //Give shoulder negative power to move down
        else if (gamepad2.right_stick_y < 0) {
            armShoulder.setPower(gamepad2.right_stick_y * 0.75);
        }
        //Hold position when no power supplied
        else {
            armShoulder.setPower(0);
        }

        if (gamepad2.right_trigger > 0) {
            right.setPosition(0.50);
            left.setPosition(0.50);
        } else if (gamepad2.left_trigger > 0) {
            right.setPosition(1.00);
            left.setPosition(0);
        }
        if (gamepad2.right_bumper == true) {
            dispenser.setPosition(0.375);
        } else if (gamepad2.left_bumper == true) {
            dispenser.setPosition(0);
        }

        liftPosCurrent = armExtend.getCurrentPosition();

        liftPosDes += multiplier*liftPosScale*gamepad2.left_stick_y;                //input scale factor
        liftPosError = liftPosDes - liftPosCurrent;
//        integrater += liftPosError;                                           //unnecessary
        liftPow = Math.min(Math.max(liftPowScale*liftPosError, -1.00), 1.00);   //proportional gain
        if(liftPow >= 1){ liftPosDes = liftPosCurrent+(1/liftPowScale); }       //AntiWindup Code
        if(liftPow <= -1) {liftPosDes = liftPosCurrent-(1/liftPowScale); }      //AntiWindup Code
       armExtend.setPower(liftPow);
    }

    //Exit loop after OpMode Stop is requested
    @Override
    public void stop()
    {
    }
}