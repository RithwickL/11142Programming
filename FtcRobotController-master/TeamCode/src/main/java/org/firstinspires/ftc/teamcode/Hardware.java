package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class Hardware {
    public DcMotor rightFront;
    public DcMotor leftFront;
    public DcMotor leftRear;
    public DcMotor rightRear;
    public DcMotor Top;
    public DcMotor Arm1;
    public DcMotor Spin;
    public Servo intake;
    public Servo drop;


    com.qualcomm.robotcore.hardware.HardwareMap hardwareMap = null;

    //On Initialization
    public void init(com.qualcomm.robotcore.hardware.HardwareMap hmMap) {
        hardwareMap = hmMap;

        //Motors
        leftFront = hardwareMap.dcMotor.get("lf");
        leftRear = hardwareMap.dcMotor.get("lr");
        rightRear = hardwareMap.dcMotor.get("rr");
        rightFront = hardwareMap.dcMotor.get("rf");
        Arm1 = hardwareMap.dcMotor.get("arm");
        Top = hardwareMap.dcMotor.get("carousel");

        //Servos
        intake = hardwareMap.servo.get("intake");
        drop = hardwareMap.servo.get("drop");

        //Resetting all Motors
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
        rightFront.setPower(0);
        Arm1.setPower(0);
        Top.setPower(0);

        //
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        Arm1.setDirection(DcMotorSimple.Direction.REVERSE);

        //Running all Motors with Encoders
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Arm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);






    }
}
