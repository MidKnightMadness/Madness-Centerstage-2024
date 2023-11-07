package org.firstinspires.ftc.teamcode;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

public class OpenCVDetector {
    private Mat matrix = new Mat();
    public OpenCVDetector(){

    }

    public final Mat processFrame(Mat input){
        input.copyTo(matrix);
        if(matrix.empty()){
            return input;
        }
        Imgproc.cvtColor(matrix,matrix, Imgproc.COLOR_RGB2YCrCb);

        Mat matLeft = matrix.submat(120,150,10, 50);

        return null;
    }
}
