package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class Nishita extends OpMode{
    DcMotor lvertical;
    DcMotor rvertical;
    DcMotor lhorizontal;
    DcMotor rhorizontal;

public void init() {
    lvertical = hardwareMap.dcMotor.get("rf");
    rvertical= hardwareMap.dcMotor.get("lr");
    lhorizontal = hardwareMap.dcMotor.get("rr");
    rhorizontal= hardwareMap.dcMotor.get("lf");
    //Reverse
    lvertical.setDirection(DcMotorSimple.Direction.REVERSE);
    lhorizontal.setDirection(DcMotorSimple.Direction.REVERSE);
}
public void loop (){
    //Forward - Backward
    lvertical.setPower(gamepad1.right_stick_y);
    rvertical.setPower(gamepad1.right_stick_y);
    //left right
    lhorizontal.setPower(gamepad1.right_stick_x);
    rhorizontal.setPower(gamepad1.right_stick_x);
    //

}
}