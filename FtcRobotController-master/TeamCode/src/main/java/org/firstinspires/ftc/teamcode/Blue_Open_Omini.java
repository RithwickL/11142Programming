package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

@Autonomous(name = "OMINI DECT")
public class Blue_Open_Omini extends LinearOpMode
{
    OpenCvCamera webcam;
    static BlueOpenCV.RingPipeline pipeline;

    private ElapsedTime runtime = new ElapsedTime();

    DcMotor leftvertical;
    DcMotor rightvertical;
    DcMotor lefthorizontal;
    DcMotor righthorzontal;
    DcMotor Top;
    DcMotor Arm1;
    //DcMotor Slide;
    DcMotor Pick;

    @Override
    public void runOpMode() {
        leftvertical = hardwareMap.dcMotor.get("lf");
        rightvertical = hardwareMap.dcMotor.get("rr");
        lefthorizontal = hardwareMap.dcMotor.get("lr");
        righthorzontal = hardwareMap.dcMotor.get("rf");
        Arm1 = hardwareMap.dcMotor.get("Spin");
        Top = hardwareMap.dcMotor.get("TOP");
        //Slide = hardwareMap.dcMotor.get("Slide");
        Pick = hardwareMap.dcMotor.get("Pick");
        leftvertical.setDirection(DcMotorSimple.Direction.REVERSE);
        lefthorizontal.setDirection(DcMotorSimple.Direction.REVERSE);
        Arm1.setDirection(DcMotorSimple.Direction.REVERSE);


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new BlueOpenCV.RingPipeline();
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

        leftvertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        righthorzontal.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lefthorizontal.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightvertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if ((pipeline.region1Avg() > pipeline.region2Avg()))
        {
            if ((pipeline.region1Avg() > pipeline.region3Avg()))
            {
                telemetry.addLine("Bottom");
                telemetry.update();
                sleep(1000);

                DriveForward(0.2,10);
                telemetry.addLine("Forward");
                telemetry.update();
                sleep(1000);
                DriveLeft(0.2,10);

            }
            else
            {
                telemetry.addLine("Top");
                telemetry.update();
                sleep(1000);

                DriveForward(0.2,10);
                telemetry.addLine("Forward");
                DriveLeft(0.2,100);
                telemetry.update();
            }

        }else {
            if (pipeline.region2Avg() > pipeline.region3Avg()) {

                telemetry.addLine("Middle");
                telemetry.update();
                sleep(1000);

                DriveForward(0.2,10);
                telemetry.addLine("Forward");
                DriveLeft(0.2,100);
                telemetry.update();
            } else
            {
                telemetry.addLine("Top");
                telemetry.update();
                sleep(1000);

                DriveForward(0.2,10);
                telemetry.addLine("Forward");
                DriveLeft(0.2,100);
                telemetry.update();
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

    public void DriveForward(double power, int distance)
    {

        //Vertical is decided by the way the camera is facing

        //reset encoder
        leftvertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightvertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //set target position
        leftvertical.setTargetPosition(distance * 31);
        rightvertical.setTargetPosition(distance * 31);

        //Go to Position
        lefthorizontal.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftvertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightvertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        righthorzontal.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //set power
        lefthorizontal.setPower(power);
        leftvertical.setPower(power);
        rightvertical.setPower(power);
        righthorzontal.setPower(power);

        while (leftvertical.isBusy() && lefthorizontal.isBusy() && righthorzontal.isBusy() && rightvertical.isBusy()){
        }
        StopDriving();
    }

    public void DriveLeft(double power, int distance)
    {
        //reset encoder
        lefthorizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        righthorzontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //set target position
        lefthorizontal.setTargetPosition(distance * -31);
        righthorzontal.setTargetPosition(distance * -31);

        //Go to Position
        lefthorizontal.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftvertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightvertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        righthorzontal.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //set power
        lefthorizontal.setPower(power);
        leftvertical.setPower(power);
        rightvertical.setPower(power);
        righthorzontal.setPower(power);

        while (leftvertical.isBusy() && lefthorizontal.isBusy() && righthorzontal.isBusy() && rightvertical.isBusy()){
        }
        StopDriving();
    }

    public void DriveRight(double power, int distance) {

        //reset encoder
        lefthorizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        righthorzontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //set target position
        lefthorizontal.setTargetPosition(distance * 31);
        righthorzontal.setTargetPosition(distance * 31);

        //Go to Position
        lefthorizontal.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        righthorzontal.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //set power
        lefthorizontal.setPower(power);
        righthorzontal.setPower(power);

        while (leftvertical.isBusy() && lefthorizontal.isBusy() && righthorzontal.isBusy() && rightvertical.isBusy()){
        }
        StopDriving();
    }

    public void DriveSpin(double power, int distance) {
        //reset encoder
        lefthorizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftvertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightvertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        righthorzontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //set target position
        lefthorizontal.setTargetPosition(distance * 31);
        leftvertical.setTargetPosition(distance * 31);
        rightvertical.setTargetPosition(distance * 31);
        righthorzontal.setTargetPosition(distance * 31);

        //Go to Position
        lefthorizontal.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftvertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightvertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        righthorzontal.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //set power
        lefthorizontal.setPower(power);
        leftvertical.setPower(power);
        rightvertical.setPower(power);
        righthorzontal.setPower(power);

        while (leftvertical.isBusy() && lefthorizontal.isBusy() && righthorzontal.isBusy() && rightvertical.isBusy()){
        }
        StopDriving();
    }

    public void Caro(double power, int distance) {

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

    public void Arm(double power, int distance) {

        //reset encoder
        Arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //set target position
        Arm1.setTargetPosition(distance * 31);

        //Go to Pos
        Arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //set power
        Arm1.setPower(power);
        //Wait
        while (Arm1.isBusy()){}
        StopDriving();
    }

    public void Intake(double power, int distance)
    {

        //reset encoder
        Pick.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //set target position
        Pick.setTargetPosition(distance * 31);

        //Go to Position
        Pick.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //set power
        Pick.setPower(power);

        while (Pick.isBusy()){
        }
        StopDriving();
    }

    public void StopDriving(){
        leftvertical.setPower(0);
        leftvertical.setPower(0);
        righthorzontal.setPower(0);
        rightvertical.setPower(0);
        Top.setPower(0);
        Arm1.setPower(0);
        Pick.setPower(0);
    }

}

