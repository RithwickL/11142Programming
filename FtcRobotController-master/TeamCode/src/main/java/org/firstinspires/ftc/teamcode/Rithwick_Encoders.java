package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;

@Autonomous(name="Drive Encoder", group="Exercises")
//@Disabled
public class Rithwick_Encoders extends LinearOpMode
{
    DcMotor leftvertical;
    DcMotor rightvertical;
    DcMotor lefthorizontal;
    DcMotor righthorzontal;

    @Override
    public void runOpMode() throws InterruptedException
    {
        leftvertical = hardwareMap.dcMotor.get("lr");
        rightvertical = hardwareMap.dcMotor.get("rf");
        rightvertical.setDirection(DcMotor.Direction.REVERSE);

        // reset encoder count kept by left motor.
        leftvertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // set left motor to run to target encoder position and stop with brakes on.
        leftvertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // set right motor to run without regard to an encoder.
        rightvertical.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Mode", "waiting");
        telemetry.update();

        // wait for start button.

        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        // set left motor to run for 5000 encoder counts.

        leftvertical.setTargetPosition(-5000);

        // set both motors to 25% power. Movement will start.

        leftvertical.setPower(-0.25);
        rightvertical.setPower(-0.25);

        // wait while opmode is active and left motor is busy running to position.

        while (opModeIsActive() && leftvertical.isBusy())
        {
            telemetry.addData("encoder-fwd", leftvertical.getCurrentPosition() + "  busy=" + leftvertical.isBusy());
            telemetry.update();
            idle();
        }

        // set motor power to zero to turn off motors. The motors stop on their own but
        // power is still applied so we turn off the power.

        leftvertical.setPower(0.0);
        rightvertical.setPower(0.0);

        // wait 5 sec to you can observe the final encoder position.

        resetStartTime();

        while (opModeIsActive() && getRuntime() < 5)
        {
            telemetry.addData("encoder-fwd-end", leftvertical.getCurrentPosition() + "  busy=" + leftvertical.isBusy());
            telemetry.update();
            idle();
        }

        // set position for back up to starting point. In this example instead of
        // having the motor monitor the encoder we will monitor the encoder ourselves.

        leftvertical.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftvertical.setTargetPosition(0);

        leftvertical.setPower(0.25);
        rightvertical.setPower(0.25);

        while (opModeIsActive() && leftvertical.getCurrentPosition() < leftvertical.getTargetPosition())
        {
            telemetry.addData("encoder-back", leftvertical.getCurrentPosition());
            telemetry.update();
            idle();
        }

        // set motor power to zero to stop motors.

        leftvertical.setPower(0.0);
        rightvertical.setPower(0.0);

        resetStartTime();

        while (opModeIsActive() && getRuntime() < 5)
        {
            telemetry.addData("encoder-back-end", leftvertical.getCurrentPosition());
            telemetry.update();
            idle();
        }
    }
}