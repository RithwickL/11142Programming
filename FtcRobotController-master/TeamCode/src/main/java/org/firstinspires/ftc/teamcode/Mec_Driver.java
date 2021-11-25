package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@TeleOp(name="Mec_Driver", group = "Default")
public class Mec_Driver extends OpMode {
    /*Arm1 PID
    private double liftPosScale = 50, liftPowScale = 0.0025;
    private double liftPosCurrent=0, liftPosDes=0, liftPosError=0, liftPow=0;
    private double integrater = 0.001, intpower = 0.00075;
    double multiplier = 1, speedK = 1;
    boolean turtle = false, sloth = false;
    double rotPos = 0, foundPos = 1;
    int shootPos = 0;
    //Slide PID
    private double liftPosScale2 = 50, liftPowScale2 = 0.0025;
    private double liftPosCurrent2=0, liftPosDes2=0, liftPosError2=0, liftPow2=0;
    private double integrater2 = 0.001, intpower2 = 0.00075;
    double multiplier2 = 1, speedK2 = 1;
    boolean turtle2 = false, sloth2 = false;
    double rotPos2 = 0, foundPos2 = 1;
    int shootPos2 = 0;*/

    //Delclare Motor Names
    DcMotor BackLeft;
    DcMotor BackRight;
    DcMotor FrontLeft;
    DcMotor FrontRight;
    DcMotor Top;
    DcMotor Arm1;
    DcMotor Pick;
    //gyro init
    public BNO055IMU imu;
    public Orientation angles;

    HardwareMap hard;

    public void init() {
        //Config
        BackLeft = hardwareMap.dcMotor.get("lf");
        BackRight = hardwareMap.dcMotor.get("rr");
        FrontLeft = hardwareMap.dcMotor.get("lr");
        FrontRight = hardwareMap.dcMotor.get("rf");
        Arm1 = hardwareMap.dcMotor.get("Spin");
        Top = hardwareMap.dcMotor.get("TOP");
        Pick = hardwareMap.dcMotor.get("Pick");
        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        Arm1.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    /*public void initGyro(HardwareMap hw)
    {
        hard = hw;

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = com.qualcomm.hardware.bosch.BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = com.qualcomm.hardware.bosch.BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".

        imu = hard.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }

    Orientation getAngles()
    {
        return (imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES));
    }*/

    public void loop() {
        /*angles = getAngles();
        float ang = angles.firstAngle;
        double ang2 = ang;
        telemetry.addData("ANGLE READ ",ang);
        telemetry.update();

        if(angles.firstAngle == 1){

        }*/
        //Forward and Back
        FrontLeft.setPower(gamepad1.right_stick_y);
        FrontRight.setPower(gamepad1.right_stick_y);
        BackLeft.setPower(gamepad1.right_stick_y);
        BackRight.setPower(gamepad1.right_stick_y);

        //Slide
        FrontLeft.setPower(gamepad1.left_stick_x);
        FrontRight.setPower(-gamepad1.left_stick_x);
        BackLeft.setPower(-gamepad1.left_stick_x);
        BackRight.setPower(gamepad1.left_stick_x);

        //Spin
        FrontLeft.setPower(gamepad1.left_trigger/2);
        FrontRight.setPower(-gamepad1.left_trigger/2);
        BackLeft.setPower(gamepad1.left_trigger/2);
        BackRight.setPower(-gamepad1.left_trigger/2);

        // Spin other way
        FrontLeft.setPower(-gamepad1.right_trigger/2);
        FrontRight.setPower(gamepad1.right_trigger/2);
        BackLeft.setPower(-gamepad1.right_trigger/2);
        BackRight.setPower(gamepad1.right_trigger/2);

        //Spin Carousel
        /*if (gamepad2.a) {
            Top.setPower(0.2);
        } else if (gamepad2.y){
            Top.setPower(-0.2);
        }else{
            Top.setPower(0);
        }

        //Intake and Output
        if (gamepad2.x) {
            Pick.setPower(1);
        } else if (gamepad2.b) {
            Pick.setPower(-1);
        } else {
            Pick.setPower(0);
        }*/

        //PID for Arm1
        /*liftPosCurrent = Arm1.getCurrentPosition();
        liftPosDes += speedK*liftPosScale*gamepad2.left_stick_y/2;                //input scale factor
        liftPosError = liftPosDes - liftPosCurrent;
//      integrater += liftPosError;                                             //unnecessary
        liftPow = Math.min(Math.max(liftPowScale*liftPosError, -1.00), 1.00);   //proportional gain
        if(liftPow >= 1){ liftPosDes = liftPosCurrent+(1/liftPowScale); }       //AntiWindup Code
        if(liftPow <= -1) {liftPosDes = liftPosCurrent-(1/liftPowScale); }      //AntiWindup Code
        Arm1.setPower(liftPow);

        // PID for Slide
        liftPosCurrent2 = Slide.getCurrentPosition();
        liftPosDes2 += speedK2*liftPosScale2*gamepad2.right_stick_x;                //input scale factor
        liftPosError2 = liftPosDes2 - liftPosCurrent2;
//      integrater2 += liftPosError2;                                              //unnecessary
        liftPow2 = Math.min(Math.max(liftPowScale2*liftPosError2, -1.00), 1.00);   //proportional gain
        if(liftPow2 >= 1){ liftPosDes2 = liftPosCurrent2+(1/liftPowScale2); }       //AntiWindup Code
        if(liftPow2 <= -1) {liftPosDes2 = liftPosCurrent2-(1/liftPowScale2); }      //AntiWindup Code
        Slide.setPower(liftPow2);*/
    }
}