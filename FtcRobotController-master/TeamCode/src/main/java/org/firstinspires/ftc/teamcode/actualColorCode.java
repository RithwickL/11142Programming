package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
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
@Autonomous
public class actualColorCode extends LinearOpMode{

    OpenCvCamera webcam;
    static RingPipeline pipeline;

    private ElapsedTime runtime = new ElapsedTime();

    DcMotor Fvertical;
    DcMotor Fhorizontal;
    DcMotor Bvertical;
    DcMotor Bhorizontal;
    DcMotor Top;
    DcMotor Arm1;
    //DcMotor Slide;
    DcMotor Pick;

    @Override
    public void runOpMode() {
        Fvertical = hardwareMap.dcMotor.get("lr");
        Bvertical = hardwareMap.dcMotor.get("rf");
        Fhorizontal = hardwareMap.dcMotor.get("lf");
        Bhorizontal = hardwareMap.dcMotor.get("rr");
        Arm1 = hardwareMap.dcMotor.get("Spin");
        Top = hardwareMap.dcMotor.get("TOP");
        //Slide = hardwareMap.dcMotor.get("Slide");
        Pick = hardwareMap.dcMotor.get("Pick");


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

        if ((pipeline.region1Avg() > pipeline.region2Avg()) && (pipeline.region1Avg() > pipeline.region3Avg())) {

            DriveForward(0.5, 2);
            DriveSide(0.5,3);
            ArmPosBOT(0.5,50);
           /*DriveBackward(0.5,-2);
            DriveSide(.5,-5);
            RobotSpin(.1,20);*/
            StopDriving();


            while (Fvertical.isBusy() && Bvertical.isBusy() && Fhorizontal.isBusy() && Bhorizontal.isBusy()){

            }
            telemetry.addLine("Bottom");
            telemetry.update();
        }
        else if ((pipeline.region2Avg() > pipeline.region1Avg()) && (pipeline.region2Avg() > pipeline.region3Avg())){

            DriveForward(0.5, 2);
            DriveSide(0.5,3);
            ArmPosMid(0.5,100);
            /*DriveBackward(0.5,-2);
            DriveSide(.5,-5);
            RobotSpin(.1,20);*/
            StopDriving();

            while (Fvertical.isBusy() && Bvertical.isBusy() && Fhorizontal.isBusy() && Bhorizontal.isBusy()){

            }
            telemetry.addLine("Middle");
            telemetry.update();
        }
        else if ((pipeline.region3Avg() > pipeline.region1Avg()) && (pipeline.region3Avg() > pipeline.region2Avg())) {

            DriveForward(0.5, 2);
            DriveSide(0.5,3);
            ArmPosTOP(0.5,150);
            /*DriveBackward(0.5,-2);
            DriveSide(.5,-5);
            RobotSpin(.1,20);*/
            StopDriving();

            while (Fvertical.isBusy() && Bvertical.isBusy() && Fhorizontal.isBusy() && Bhorizontal.isBusy()){

            }
            telemetry.addLine("Top");
            telemetry.update();
        }else
            {
            telemetry.addLine("unknown");
            telemetry.update();
            StopDriving();
        }


        /*
        telemetry.addData("Frame Count", webcam.getFrameCount());
        telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
        telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
        telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
        telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
        telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
        */

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

        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(0, 375);
        static final int REGION1_WIDTH = 200;
        static final int REGION1_HEIGHT = 200;
        static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(535,375);
        static final int REGION2_WIDTH = 200;
        static final int REGION2_HEIGHT = 200;
        static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(1059,350);
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
    public void DriveSide (double power, int distance){
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

    public void Spin(double power, int distance) {
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
    }

    public void ArmPosTOP(double power, int degrees)
    {
        Arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Pick.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Move to Position
        Arm1.setTargetPosition(degrees);
        Pick.setTargetPosition(-degrees * 31);

        Arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Pick.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Arm1.setPower(1);
        Pick.setPower(1);

        StopDriving();

    }

    public void ArmPosMid(double power, int degrees)
    {
        Arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Pick.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Move to Position
        Arm1.setTargetPosition(degrees);
        Pick.setTargetPosition(-degrees * 31);

        Arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Pick.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        Arm1.setPower(1);
        Pick.setPower(1);

        StopDriving();

    }
    public void ArmPosBOT(double power, int degrees)
    {
        Arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Pick.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Move to Position
        Arm1.setTargetPosition(degrees);
        Pick.setTargetPosition(-degrees * 31);

        Arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Pick.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Arm1.setPower(1);
        Pick.setPower(1);

        StopDriving();

    }

}
