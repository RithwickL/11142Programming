package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import java.util.List;

@Autonomous(name="CameraVP")
public class CameraVrayas extends LinearOpMode
{

    DcMotor vertFront;
    DcMotor vertBack;
    DcMotor horBack;
    DcMotor horFront;


    public void runOpMode()
    {
        vertFront = hardwareMap.dcMotor.get("lr");
        vertBack = hardwareMap.dcMotor.get("rf");
        horFront = hardwareMap.dcMotor.get("lf");
        horBack = hardwareMap.dcMotor.get("rr");

        vertBack.setDirection(DcMotorSimple.Direction.REVERSE);
        horBack.setDirection(DcMotorSimple.Direction.REVERSE);

        vertFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        vertBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        horFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        horBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (opModeIsActive()) {
        while (opModeIsActive()) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null)
                {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions)
                        {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());
                        i++;
                        }
                    telemetry.update();
                    }
                }
            }
        }
    }
}
