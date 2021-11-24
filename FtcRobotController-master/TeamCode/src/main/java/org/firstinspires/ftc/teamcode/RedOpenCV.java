package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@Disabled
@Autonomous(name = "RedOpenCVDect")
public class RedOpenCV extends LinearOpMode{

    OpenCvCamera webcam;
    static RingPipeline pipeline;

    private ElapsedTime runtime = new ElapsedTime();

    DcMotor Fvertical;
    DcMotor Fhorizontal;
    DcMotor Bvertical;
    DcMotor Bhorizontal;
    DcMotor Top;
    DcMotor Arm1;
    DcMotor Intake;

    @Override
    public void runOpMode() {
        Fvertical = hardwareMap.dcMotor.get("rr");
        Bvertical = hardwareMap.dcMotor.get("lf");
        Fhorizontal = hardwareMap.dcMotor.get("rf");
        Bhorizontal = hardwareMap.dcMotor.get("lr");
        Arm1 = hardwareMap.dcMotor.get("Spin");
        Top = hardwareMap.dcMotor.get("TOP");
        Intake = hardwareMap.dcMotor.get("Pick");

        Bhorizontal.setDirection(DcMotorSimple.Direction.REVERSE);
        Bvertical.setDirection(DcMotorSimple.Direction.REVERSE);


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new RingPipeline();
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        telemetry.addLine("Waiting for start");
        telemetry.update();

        /*
         * Wait for the user to press start on the Driver Station
         */
        waitForStart();


        telemetry.addData("Region 1", pipeline.region1Avg());
        telemetry.addData("Region 2", pipeline.region2Avg());
        telemetry.addData("Region 3", pipeline.region3Avg());

        if ((pipeline.region1Avg() > pipeline.region2Avg()))
        {
            if ((pipeline.region1Avg() > pipeline.region3Avg()))
            {
                DriveForward(.5,20);
                DriveRight(.5,-30);
                ArmPosTOP(.5, -1500, 0);
                sleep(1000);
                StopDriving();

            }
            else
            {
                DriveForward(.5,20);
                DriveRight(.5,-300);
                ArmPosBOT(.5, -500,0);
                StopDriving();
            }

        }else {
            if (pipeline.region2Avg() > pipeline.region3Avg()) {
                DriveForward(.5, 20);
                DriveRight(.5, -20);
                ArmPosMid(.5, -1000,0);
                StopDriving();
            } else
                {
                DriveForward(.5, 20);
                DriveRight(.5, -300);
                ArmPosBOT(.5, -500,0);
                StopDriving();
            }
        }

        telemetry.update();



        sleep(20);
    }




    public static class RingPipeline extends OpenCvPipeline {

        /** Most important section of the code: Colors **/
        static final Scalar CRIMSON = new Scalar(220, 20, 60);
        static final Scalar AQUA = new Scalar(79, 195, 247);
        static final Scalar PARAKEET = new Scalar(3, 192, 74);
        static final Scalar GOLD = new Scalar(255, 215, 0);
        static final Scalar CYAN = new Scalar(0, 139, 139);

        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(100, 400);
        static final int REGION1_WIDTH = 200;
        static final int REGION1_HEIGHT = 200;
        static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(575,400);
        static final int REGION2_WIDTH = 200;
        static final int REGION2_HEIGHT = 200;
        static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(1059,340);
        static final int REGION3_WIDTH = 200;
        static final int REGION3_HEIGHT = 200;


        Point region1_pointA = new Point(REGION1_TOPLEFT_ANCHOR_POINT.x, REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(REGION1_TOPLEFT_ANCHOR_POINT.x + REGION1_WIDTH, REGION1_TOPLEFT_ANCHOR_POINT.y + REGION1_HEIGHT);

        Point region2_pointA = new Point(REGION2_TOPLEFT_ANCHOR_POINT.x, REGION2_TOPLEFT_ANCHOR_POINT.y);
        Point region2_pointB = new Point(REGION2_TOPLEFT_ANCHOR_POINT.x + REGION2_WIDTH, REGION2_TOPLEFT_ANCHOR_POINT.y + REGION2_HEIGHT);

        Point region3_pointA = new Point(REGION3_TOPLEFT_ANCHOR_POINT.x, REGION3_TOPLEFT_ANCHOR_POINT.y);
        Point region3_pointB = new Point(REGION3_TOPLEFT_ANCHOR_POINT.x + REGION3_WIDTH, REGION2_TOPLEFT_ANCHOR_POINT.y + REGION3_HEIGHT);

        Mat region1_G, region2_G, region3_G;
        Mat BGR = new Mat();
        Mat G = new Mat();
        int avg1, avg2, avg3;

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToG(Mat input)
        {
            Imgproc.cvtColor(input, BGR, Imgproc.COLOR_RGB2BGR);
            Core.extractChannel(BGR, G, 1);
        }

        @Override
        public void init(Mat firstFrame) {
            inputToG(firstFrame);

            region1_G = G.submat(new Rect(region1_pointA, region1_pointB));
            region2_G = G.submat(new Rect(region2_pointA, region2_pointB));
            region3_G = G.submat(new Rect(region3_pointA, region3_pointB));
        }

        @Override
        public Mat processFrame(Mat input) {

            inputToG(input);

            avg1 = (int) Core.mean(region1_G).val[0];
            avg2 = (int) Core.mean(region2_G).val[0];
            avg3 = (int) Core.mean(region3_G).val[0];


            Imgproc.rectangle(input, region1_pointA, region1_pointB, CRIMSON,2);
            Imgproc.rectangle(input, region2_pointA, region2_pointB, AQUA,2);
            Imgproc.rectangle(input, region3_pointA, region3_pointB, PARAKEET,2);


            return input;

        }

        public int region1Avg() {
            return avg1;
        }
        public int region2Avg() {
            return avg2;
        }
        public int region3Avg() {
            return avg3;
        }

    }

    public void DriveBackward (double power, int distance)
    {
        Fvertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Bvertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //set target position
        Fvertical.setTargetPosition(distance * 31);
        Bvertical.setTargetPosition(distance * 31);

        Fvertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Bvertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //set power
        Fvertical.setPower(power);
        Bvertical.setPower(power);

        while (Fvertical.isBusy() && Bvertical.isBusy()){

        }
        StopDriving();
    }

    public void ResetArm (double power, int distance)
    {
        Arm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void DriveForward (double power, int distance) {
        //reset encoder
        Fvertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Bvertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //set target position
        Fvertical.setTargetPosition(distance * 31);
        Bvertical.setTargetPosition(distance * 31);

        Fvertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Bvertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //set power
        Fvertical.setPower(power);
        Bvertical.setPower(power);

        telemetry.addLine("DriveForward");
        telemetry.update();
        sleep(1000);

        while (Fvertical.isBusy() && Bvertical.isBusy()){

        }


        StopDriving();

    }
    public void Arm (double power, int distance) {
        //reset encoder
        Arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //set target position
        Arm1.setTargetPosition(distance * 31);


        Arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        //set power
        Arm1.setPower(power);


        while (Arm1.isBusy()){

        }
        StopDriving();

    }
    public void RobotSpin (double power, int distance) {
        //reset encoder
        Fvertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Bvertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Fhorizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Bhorizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //set target position
        Fvertical.setTargetPosition(distance * -31);
        Bvertical.setTargetPosition(distance * 31);
        Fhorizontal.setTargetPosition(distance * -31);
        Bhorizontal.setTargetPosition(distance * 31);

        Fvertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Bvertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Fhorizontal.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Bhorizontal.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //set power
        Fvertical.setPower(power);
        Bvertical.setPower(power);
        Fhorizontal.setPower(power);
        Bhorizontal.setPower(power);

        while (Fvertical.isBusy() && Bvertical.isBusy() && Fhorizontal.isBusy() && Bhorizontal.isBusy()){

        }
        StopDriving();

    }
    public void DriveLeft (double power, int distance){
        //reset
        Fhorizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Bhorizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //target position
        Fhorizontal.setTargetPosition(distance * 31);
        Bhorizontal.setTargetPosition(distance * 31);

        Fhorizontal.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Bhorizontal.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //set power
        Fhorizontal.setPower(power);
        Bhorizontal.setPower(power);

        while (Fhorizontal.isBusy() && Bhorizontal.isBusy()) {

        }
        StopDriving();
    }

    public void DriveRight(double power, int distance)
    {

        Fhorizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Bhorizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //target position
        Fhorizontal.setTargetPosition(distance * 31);
        Bhorizontal.setTargetPosition(distance * 31);

        Fhorizontal.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Bhorizontal.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        telemetry.addLine("Drive Right");
        telemetry.update();
        sleep(1000);

        //set power
        Fhorizontal.setPower(power);
        Bhorizontal.setPower(power);
    }

    public void Spin(double power, int distance)
    {
        //reset encoder
        Top.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //set target position
        Top.setTargetPosition(distance * 31);
        Top.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //set power
        Top.setPower(power);

        while (Top.isBusy()) {

        }
        StopDriving();

    }
    public void StopDriving(){

        Fvertical.setPower(0);
        Fhorizontal.setPower(0);
        Bvertical.setPower(0);
        Bhorizontal.setPower(0);
        Top.setPower(0);
        Arm1.setPower(0);
        Intake.setPower(0);
    }

    public void ArmPosTOP(double power, int degrees, int spin)
    {
        Arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Move to Position
        Arm1.setTargetPosition(degrees);
        Intake.setTargetPosition(-spin);

        Arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Arm1.setPower(1);
        Intake.setPower(1);

        telemetry.addLine("Top");
        telemetry.update();
        sleep(1000);

        StopDriving();

    }

    public void ArmPosMid(double power, int degrees, int spin)
    {
        Arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Move to Position
        Arm1.setTargetPosition(degrees);
        Intake.setTargetPosition(-spin);

        Arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        telemetry.addLine("Middle");
        telemetry.update();
        sleep(1000);


        Arm1.setPower(1);
        Intake.setPower(1);

        StopDriving();

    }
    public void ArmPosBOT(double power, int degrees, int spin)
    {
        Arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Move to Position
        Arm1.setTargetPosition(degrees);
        Intake.setTargetPosition(-spin);

        Arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        telemetry.addLine("Bottom");
        telemetry.update();
        sleep(1000);

        Arm1.setPower(1);
        Intake.setPower(1);

        StopDriving();

    }

}
