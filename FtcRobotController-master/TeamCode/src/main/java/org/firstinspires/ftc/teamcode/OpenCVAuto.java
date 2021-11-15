package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
@Disabled
@Autonomous(name = "Open Cv")
public class OpenCVAuto extends LinearOpMode
{
    private OpenCvWebcam webcam;
    private OpenCVDectactor dectector;
    private String position;

    @Override
    public void runOpMode()
    {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(dectector);
        webcam.openCameraDevice();
        webcam.startStreaming(320,240,OpenCvCameraRotation.UPRIGHT);//Find The right number for each



        while  (!isStarted())
        {
            position = dectector.position;
            telemetry.addData("position",position);
        }
    }

}
