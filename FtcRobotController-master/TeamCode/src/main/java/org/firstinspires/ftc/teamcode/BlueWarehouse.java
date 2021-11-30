package org.firstinspires.ftc.teamcode;

import android.view.MenuInflater;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


// Name the program as it shows up in the robotr Station
@Autonomous(name = "Blue Warehouse Full")

public class BlueWarehouse extends LinearOpMode {

    SampleMecanumDrive enigma;
    /* Note: This sample uses the all-objects Tensor Flow model (FreightFrenzy_BCDM.tflite), which contains
     * the following 4 detectable objects
     *  0: Ball,
     *  1: Cube,
     *  2: Duck,
     *  3: Marker (duck location tape marker)
     *
     *  Two additional model assets are available which only contain a subset of the objects:
     *  FreightFrenzy_BC.tflite  0: Ball,  1: Cube
     *  FreightFrenzy_DM.tflite  0: Duck,  1: Marker
     */
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };
    // Import the variables from the util class for the motors and servos

    // Declare counters at natural starting point
    /*
    As the Op mode runs, either of the integer counters will increase based on whether an object is detected or absent.
    The variable with a higher count after 10 loops determines the likelihood of an object actually present in the webcam's view.
     */
    int objectDetected = 0;
    int objectAbsent= 0;

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY = "AeGmRrj/////AAABmRecVnevhUavneR/YWLa66hGsMcYY8WsLL50I1NLom/ckPQQos4M+4LgHH2DHWUoxnJPaZqHOW7CwPJDAlfakZ002uG5WpakbNO9WI+Hxkg+qUJ9EcG9/l30pCTezJsRePriqUXLwCfIryBh2ddWyTYSFA7xHQhj6pv4wF60EAsfPOVspjREge75rGeJA43i/z2xGMWZN+cwniAk5oM+GjRFfd7NjztSp9j1oYh4EqqnuQMVhClW92nv8TDFsd7ch4FSgbH1/n4yUZpLhCZ3e9/y+gq7AfngZW7kuaFuEQ0BUjdTN/2ciWcKntxW6Lw1AmI1KF9UX80pi1gbHUsFvElwgMuiTnI2xLLRgrcLZ1rU";
    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        enigma = new SampleMecanumDrive(hardwareMap);
        //initVuforia();
        //initTfod();
        //robot inputs all defined variables from the SampleMecanumrobot class

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        /*if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(2.5, 16/9);

            // Only a single object should be visible in the webcam's view, so the viewing area is squared.
        }*/

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            objectRight();
        }
    }
    public void gyroCorrect()
    {
        enigma.angles = enigma.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        enigma.absHeading = enigma.angles.firstAngle;
        telemetry.addData("Heading", enigma.absHeading);
        telemetry.update();
        for(int f = 1; f < 20; f++)
        {
            enigma.leftFront.setPower(-0.1 * (enigma.absHeading - enigma.originalHeading));
            enigma.leftRear.setPower(-0.1 * (enigma.absHeading - enigma.originalHeading));
            enigma.rightFront.setPower(0.1 * (enigma.absHeading - enigma.originalHeading));
            enigma.rightRear.setPower(0.1 * (enigma.absHeading - enigma.originalHeading));
            enigma.angles = enigma.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            enigma.absHeading = enigma.angles.firstAngle;
            telemetry.addData("Heading", enigma.absHeading);
            telemetry.update();
        }
        enigma.leftFront.setPower(0);
        enigma.leftRear.setPower(0);
        enigma.rightFront.setPower(0);
        enigma.rightRear.setPower(0);
        telemetry.update();
    }


    public void objectRight()
    {

        //arm is fw now
       Trajectory fw = enigma.trajectoryBuilder(new Pose2d(0, 0, 0))
                .strafeRight(27)
                .build();
        enigma.followTrajectory(fw);


        Trajectory toCarousel = enigma.trajectoryBuilder(new Pose2d(0, 0, 0))
                .back(40)
                .build();
        enigma.followTrajectory(toCarousel);

        Trajectory bk = enigma.trajectoryBuilder(new Pose2d(0, 0, 0))
                .strafeLeft(10)
                .build();
        enigma.followTrajectory(bk);

        enigma.carousel.setPower(0.3);

        Trajectory toCarousel2 = enigma.trajectoryBuilder(new Pose2d(0, 0, 0))
                .back(10)
                .build();
        enigma.followTrajectory(toCarousel2);


        sleep(2000);
        enigma.carousel.setPower(0);

        //says back but really forward
        Trajectory fw2 = enigma.trajectoryBuilder(new Pose2d(0, 0, 0))
                .strafeRight(70)
                .build();
        enigma.followTrajectory(fw2);

        Trajectory rt = enigma.trajectoryBuilder(new Pose2d(0, 0, 0))
                .back(35)
                .build();
        enigma.followTrajectory(rt);

        Trajectory left = enigma.trajectoryBuilder(new Pose2d(0, 0, 0))
                .forward(48)
                .build();
        enigma.followTrajectory(left);

        enigma.intake.setPosition(.1);
        sleep(1000);
        enigma.intake.setPosition(.1);

        Trajectory right = enigma.trajectoryBuilder(new Pose2d(0, 0, 0))
                .back(52)
                .build();
        enigma.followTrajectory(right);

        Trajectory back = enigma.trajectoryBuilder(new Pose2d(0, 0, 0))
                .strafeLeft(130)
                .build();
        enigma.followTrajectory(back);

        Trajectory left2 = enigma.trajectoryBuilder(new Pose2d(0, 0, 0))
                .forward(75)
                .build();
        enigma.followTrajectory(left2);

        Trajectory back2 = enigma.trajectoryBuilder(new Pose2d(0, 0, 0))
                .strafeLeft(50)
                .build();
        enigma.followTrajectory(back2);

        enigma.followTrajectory(left2);


    }
    public void objectMiddle()
    {
        objectRight();

    }
    public void objectLeft()
    {

       objectRight();
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.75f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }

    public void startCarousel(double power)
    {
        enigma.carousel.setPower(power);
        // The program will double check the power and make sure its is correct
        if(power > 0) {
            double currentPower = enigma.carousel.getPower();
            double error = (power - currentPower);
            enigma.carousel.setPower(currentPower + error);
        }
    }
    // Stops the flywheel
    public void stopCarousel() {
        enigma.carousel.setPower(0.00);
    }
}
