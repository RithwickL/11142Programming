package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class OpenCVDectactor extends OpenCvPipeline
{
    private Mat workingMatrix = new Mat();
    public String position = "LEFT";
    public OpenCVDectactor()
    {

    }
    @Override
    public final Mat processFrame(Mat input)
    {
        input.copyTo(workingMatrix);
        if(workingMatrix.empty())
        {
             return input;
        }
        Imgproc.cvtColor(workingMatrix, workingMatrix, Imgproc.COLOR_RGB2YCrCb);



        Imgproc.rectangle(workingMatrix, new Rect(50,120,40,30), new Scalar(0,255,0));
        Imgproc.rectangle(workingMatrix, new Rect(100,120,40,30), new Scalar(0,0,255));
        Imgproc.rectangle(workingMatrix, new Rect(150,120,40,30), new Scalar(255,0,0));

        Mat matleft = workingMatrix.submat(50,90,120,150);
        Mat matmiddle = workingMatrix.submat(100,140,120,150);
        Mat matright = workingMatrix.submat(150,190,120,150);



        double lefttotal = Core.sumElems(matleft).val[2];
        double middletotal = Core.sumElems(matmiddle).val[2];
        double righttotal = Core.sumElems(matright).val[2];

        if (lefttotal < middletotal)
        {
            if (lefttotal < righttotal)
            {
                //Left is the Duck
                position = "LEFT";
            }
            else
                {
                    // Right is the duck
                    position = "RIGHT";
                }

        } else
            {
             if (middletotal < righttotal)
             {
                 //center is the Duck
                 position = "CENTER";
             } else
                 {
                    //right is the duck
                     position = "RIGHT";
                 }
            }
        return workingMatrix;
    }
}

