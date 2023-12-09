package org.firstinspires.ftc.teamcode.Utility;

import org.firstinspires.ftc.teamcode.Camera.CameraEnums.*;



public class Coordinates {
    //coordinates of the lines of the spike mark

    public static Pose[] coordinatesLines = {
            //red
            new Pose(-12,35.25, 90),
            new Pose(0, 41.25, 90),
            new Pose(12,35.25, 90),


            //blue
            new Pose(-12,35.25, 270),
            new Pose(0, 41.25, 270),
            new Pose(12,35.25, 270),};


    //coordinates class
     public Coordinates(){

     }



    public static Pose getSpikeMark(CameraModes mode, SpikeMarkPositions position) {
         //red side
        if (mode == CameraModes.BLUE) {
            if (position == SpikeMarkPositions.LEFT) {//left side
                return coordinatesLines[2];
            } else if (position == SpikeMarkPositions.RIGHT) {//right side
                return coordinatesLines[0];
            } else {//returns coordinates at center
                return coordinatesLines[1];
            }
        } else {//blue side
            if(position == SpikeMarkPositions.LEFT){//left side
                return coordinatesLines[3];
            } else if(position ==SpikeMarkPositions.RIGHT){//right side
                return coordinatesLines[5];
            }else{
                return coordinatesLines[4];
            }

        }


    }
}
