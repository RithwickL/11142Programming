package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="1Driver")
public class RithwickFTC2021_1S2D extends OpMode {
    DcMotor leftvertical;
    DcMotor rightvertical;
    DcMotor lefthorizontal;
    DcMotor righthorzontal;
    DcMotor Top;
    DcMotor Arm1;
    //DcMotor Arm2;
    //Servo Claw;
    BNO055IMU imu;
    Orientation angles;

    public void init() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;

        imu = hardwareMap.get(BNO055IMU.class,  "imu");
        imu.initialize(parameters);

        leftvertical = hardwareMap.dcMotor.get("lf");
        rightvertical = hardwareMap.dcMotor.get("rr");
        lefthorizontal = hardwareMap.dcMotor.get("lr");
        righthorzontal = hardwareMap.dcMotor.get("rf");
        //Claw = hardwareMap.servo.get("Claw");
        Arm1 = hardwareMap.dcMotor.get("Spin1");
        //Arm2 = hardwareMap.dcMotor.get("Spin2");
        Top = hardwareMap.dcMotor.get("TOP");
        leftvertical.setDirection(DcMotorSimple.Direction.REVERSE);
        lefthorizontal.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void loop() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("Heading", angles.firstAngle);
        //telemetry.addData("Role", angles.secondAngle);
        //telemetry.addData("Pitch", angles.thirdAngle);
        telemetry.update();
        if (angles == 0){}

        //Left and Right - right stick - Right/Left
        leftvertical.setPower(gamepad1.right_stick_x);
        rightvertical.setPower(gamepad1.right_stick_x);
        //Forward and Backward - left stick - up/down
        lefthorizontal.setPower(gamepad1.right_stick_y);
        righthorzontal.setPower(gamepad1.right_stick_y);

        //Spin from center orign2 - Left stick- Left/right
        lefthorizontal.setPower(gamepad1.left_trigger / 2);
        righthorzontal.setPower(-gamepad1.left_trigger / 2);
        leftvertical.setPower(gamepad1.left_trigger / 2);
        rightvertical.setPower(-gamepad1.left_trigger / 2);

        lefthorizontal.setPower(-gamepad1.right_trigger / 2);
        righthorzontal.setPower(gamepad1.right_trigger / 2);
        leftvertical.setPower(-gamepad1.right_trigger / 2);
        rightvertical.setPower(gamepad1.right_trigger / 2);

        if (gamepad1.b == true) {
            Arm1.setPower(-0.25);

        } else {
            Arm1.setPower(0.25);
            Arm1.setPower(-0.25);
        }

        if (gamepad1.y == true) {
            Arm1.setPower(0.25);

        }

        if (gamepad1.a == true) {
            Top.setPower(0.5);
        } else {
            Top.setPower(0);
        }

        /*if (gamepad1.x == true) {

        } else {

        }*/

        if (gamepad1.x == true) {
            //Claw.setPosition(90);
        } else {
            //Claw.setPosition(0);

        }
        /*if (gamepad1.right_bumper == true) {

        } else {

        }*/
    }
}