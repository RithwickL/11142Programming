package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "")
public class PID_Test extends LinearOpMode
{

    private ElapsedTime Timer = new ElapsedTime();

    public static PIDCoefficients pidCoffs = new PIDCoefficients(0,0,0);

    public static PIDCoefficients pidGains = new PIDCoefficients(0,0,0);

    DcMotorEx ArmMotor;

    public void runOpMode()
    {
        ArmMotor = hardwareMap.get(DcMotorEx.class,"arm");

        ArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        ArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        if(opModeIsActive()) {
            while (opModeIsActive())
            {
                PID(100);

            }


        }
    }

    double integral = 0;
    double lastError = 0;

    public void PID(double targetVelocity)
    {

        double currentVelocity = ArmMotor.getVelocity();
        double error = targetVelocity - currentVelocity;

        integral += error * Timer.time();

        double deltaError = error - lastError;
        double derivative = deltaError / Timer.time();

        pidCoffs.p = pidCoffs.p * error;
        pidCoffs.p = pidCoffs.i * integral;
        pidCoffs.p = pidCoffs.d * derivative;


        ArmMotor.setVelocity(pidGains.p+ pidGains.i + pidGains.d + targetVelocity);

        lastError = error;
    }
}
