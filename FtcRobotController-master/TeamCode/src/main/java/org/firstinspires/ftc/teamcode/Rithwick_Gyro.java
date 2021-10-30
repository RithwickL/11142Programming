/*package org.firstinspires.ftc.teamcode;

import android.graphics.drawable.GradientDrawable;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.GyroSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Rithwick_Gyro extends LinearOpMode {
   BNO055IMU imu;

   Orientation angles;
    @Override
    public void runOpMode() throws InterruptedException {
BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;

imu = hardwareMap.get(BNO055IMU.class, deviceName:"imu");
imu.initialize(parameters);

waitForStart();

while (opModeIsActive()){
    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    telemetry.addData(caption: "Heading", angles.firstAngle);
    telemetry.addData(caption: "Heading", angles.secondAngle);
    telemetry.addData(caption: "Heading", angles.thirdAngle);
    telemetry.update();
}
    }
}*/
