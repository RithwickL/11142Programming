package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Driver")
public class Mec_Driver extends OpMode {
    //Arm1
    /*private double liftPosScale = 50, liftPowScale = 0.0025;
    private double liftPosCurrent=0, liftPosDes=0, liftPosError=0, liftPow=0;
    private double integrater = 0.001, intpower = 0.00075;
    double multiplier = 1, speedK = 1;
    boolean turtle = false, sloth = false;
    double rotPos = 0, foundPos = 1;
    int shootPos = 0;*/

    DcMotor rightFront;
    DcMotor leftFront;
    DcMotor leftRear;
    DcMotor rightRear;
    DcMotor Top;
    DcMotor Arm1;
    //DcMotor Slide;
    DcMotor Pick;

    // above initializes all the aspects we need to make our robot function
    @Override
    public void init() {
        // defining all the hardware
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
        Arm1.setDirection(DcMotorSimple.Direction.REVERSE);


    }
    public void loop () {
        float y1 = gamepad1.left_stick_x;
        float x1 = gamepad1.right_stick_y/2;
        float r1 = gamepad1.left_trigger;
        float r2 = gamepad1.right_trigger;
        // Reset variables
        float leftFrontPower = 0;
        float leftBackPower = 0;
        float rightFrontPower = 0;
        float rightBackPower = 0;
        // Handle regular movement
        leftFrontPower += y1;
        leftBackPower += y1;
        rightFrontPower += y1;
        rightBackPower += y1;
        // Handle strafing movement
        leftFrontPower += -x1;
        leftBackPower -= -x1;
        rightFrontPower -= -x1;
        rightBackPower += -x1;

        leftFront.setPower(-gamepad1.left_trigger / 2);
        leftRear.setPower(gamepad1.left_trigger / 2);
        rightRear.setPower(gamepad1.left_trigger / 2);
        rightFront.setPower(-gamepad1.left_trigger / 2);
        // Spin other way
        leftFront.setPower(gamepad1.right_trigger / 2);
        leftRear.setPower(-gamepad1.right_trigger / 2);
        rightRear.setPower(-gamepad1.right_trigger / 2);
        rightFront.setPower(gamepad1.right_trigger / 2);

        if (gamepad2.a) {
            Top.setPower(0.2);
        } else if (gamepad2.y){
            Top.setPower(-0.2);
        }else{
            Top.setPower(0);
        }
        Arm1.setPower(gamepad2.right_stick_y);
        //Intake
        if (gamepad2.x) {
            Pick.setPower(1);
        } else if (gamepad2.b) {
            Pick.setPower(-1);
        } else {
            Pick.setPower(0);
        }

        // Scale movement
        double max = Math.max(Math.abs(leftFrontPower), Math.max(Math.abs(leftBackPower),
                Math.max(Math.abs(rightFrontPower), Math.abs(rightBackPower))));
        if (max > 1) {
            leftFrontPower = (float) Range.scale(leftFrontPower, -max, max, -.30, .30);
            leftBackPower = (float) Range.scale(leftBackPower, -max, max, -.30, .30);
            rightFrontPower = (float) Range.scale(rightFrontPower, -max, max, -.30, .30);
            rightBackPower = (float) Range.scale(rightBackPower, -max, max, -.30, .30);
        }
        leftRear.setPower(-leftBackPower);
        leftFront.setPower(-leftFrontPower);
        rightFront.setPower(-rightFrontPower);
        rightRear.setPower(-rightBackPower);

            /*PID for Arm1
            liftPosCurrent = Arm1.getCurrentPosition();
            liftPosDes += speedK*liftPosScale*gamepad2.left_stick_y/4;                //input scale factor
            liftPosError = liftPosDes - liftPosCurrent;
//      integrater += liftPosError;                                             //unnecessary
            liftPow = Math.min(Math.max(liftPowScale*liftPosError, -1.00), 1.00);   //proportional gain
            if(liftPow >= 1){ liftPosDes = liftPosCurrent+(1/liftPowScale); }       //AntiWindup Code
            if(liftPow <= -1) {liftPosDes = liftPosCurrent-(1/liftPowScale); }      //AntiWindup Code
            Arm1.setPower(liftPow);*/
    }
}
