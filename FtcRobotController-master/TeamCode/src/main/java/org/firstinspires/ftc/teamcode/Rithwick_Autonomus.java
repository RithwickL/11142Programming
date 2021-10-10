package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@Autonomous
public abstract class Rithwick_Autonomus extends OpMode {
    DcMotor leftvertical;
    DcMotor rightvertical;
    DcMotor lefthorizontal;
    DcMotor righthorzontal;



@Override public void init() {
    leftvertical = hardwareMap.dcMotor.get("lf");
    rightvertical= hardwareMap.dcMotor.get("rr");
    lefthorizontal = hardwareMap.dcMotor.get("lr");
    righthorzontal= hardwareMap.dcMotor.get("rf");
    //Front = hardwareMap.servo.get("blocker");//
    leftvertical.setDirection(DcMotorSimple.Direction.REVERSE);
    lefthorizontal.setDirection(DcMotorSimple.Direction.REVERSE);
    //Intake = hardwareMap.dcMotor.get("intake");//

}
public void DriveFB(double power,int time){
    leftvertical.setPower(power);
    rightvertical.setPower(power);
    Thread.sleep(time);
}



public void loop() {

    }

}

