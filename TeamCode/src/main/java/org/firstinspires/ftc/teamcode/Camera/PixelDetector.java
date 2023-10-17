package org.firstinspires.ftc.teamcode.Camera;

import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;

public class PixelDetector extends OpenCvPipeline {
    private Mat output;

    // Screen dimensions, number of individual rgba units
    public int height = 0;
    public int width = 0;

    // 3x3 Kernels
    int [][] edgeDetection = {
            {0, -1, 0},
            {-1, 2, -1},
            {0, -1, 0}
    };
    int [][] verticalEdge = {
            {-1, 0, 1},
            {-2, 0, 2},
            {-1, 0, 1}
    };
    int [][] horizontalEdge = {
            { 1,  2,  1},
            { 0,  0,  0},
            {-1, -2, -1}
    };

    @Override
    public Mat processFrame(Mat input) {

        // Get input data, ignore outermost border of pixels
        height = input.height() - 2;
        width = input.width() - 2;
        int pixelValue = 0; // Required since kernels act as an inner product

        output = new Mat();
        output.copySize(input);

        for(int i = 1; i < height - 1; i++){ // row i
            for(int j = 1; j < width - 1; j++){ // column j
                // Each pixel is the inner product of the 9 pixels (itself and surrounding) with the kernel in use
                // note: THIS IS NOT OPTIMIZED
                pixelValue = 0;

                for(int y = -1; y < 2; y++){ // 3x3 matrix for kernel
                    for(int x = -1; x < 2; x++){

                    }
                }
            }
        }



        return output;
    }
}
