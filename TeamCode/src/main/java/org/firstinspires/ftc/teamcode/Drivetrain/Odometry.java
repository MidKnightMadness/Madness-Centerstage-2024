package org.firstinspires.ftc.teamcode.Drivetrain;

//import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Utility.Vector2;

import java.util.concurrent.TimeUnit;
//@Config
public class Odometry {
    //Constants
    double inPerTickLeft = 30.0 / 38000d;
    double inPerTickRight = 30.0 / 38000d;
    double inPerTickFront= 30.0 / 38000d;
    double HEADING_CORRECTION = 360d / 359.99;

    double trackWidth = 12.5;
    double distanceToFront = 1.5;

    double [] deltaTicks = {0d, 0d, 0d}; // Left, right, center
    double [] lastTicks = {0d, 0d, 0d}; // Left, right, center
    double deltaRadians = 0.0;
    double [] deltaCoords = {0.0, 0.0};

    double perceivedHeading = 0.0;
    double [] perceivedCoords = {0.0, 0.0};


    //Tracking Time
    public double deltaTime = 0;
    double lastTime = 0;
    ElapsedTime elapsedTime;

    //Pose Tracking Variables
    public Vector2 position = new Vector2();
    Vector2 velocity = new Vector2();
    Vector2 velocityRelativeToRobot = new Vector2();
    public double angularVelocity = 0.0;
    public double rotationRadians = 0d;

    // Hardware variables
    public DcMotorEx leftEncoder;
    public DcMotorEx rightEncoder;
    public DcMotorEx horizontalEncoder;

    public Odometry(HardwareMap hardwareMap, double startingAngleRadians, Vector2 startingPosition) {
        deltaRadians = 0.0;
        deltaCoords = new double [2];

        perceivedHeading = startingAngleRadians;
        perceivedCoords [0] = startingPosition.x;
        perceivedCoords [1] = startingPosition.y;
        elapsedTime = new ElapsedTime();

        //Initialize Motors
        leftEncoder = hardwareMap.get(DcMotorEx.class, "BL");
        rightEncoder = hardwareMap.get(DcMotorEx.class, "FR");
        horizontalEncoder = hardwareMap.get(DcMotorEx.class, "FL");

        //Reset Position
        this.rotationRadians = startingAngleRadians;
        this.position = startingPosition;
    }

    public void updatePosition() { // Assuming left and right are positive for forward, horizontal is positive for counterclockwise
        // Update encoder ticks
        deltaTicks [0] = leftEncoder.getCurrentPosition() - lastTicks [0];
        deltaTicks [1] = rightEncoder.getCurrentPosition() - lastTicks [1];
        deltaTicks [2] = horizontalEncoder.getCurrentPosition() - lastTicks [2];

        lastTicks [0] = leftEncoder.getCurrentPosition();
        lastTicks [1] = rightEncoder.getCurrentPosition();
        lastTicks [2] = horizontalEncoder.getCurrentPosition();

        // Assume left and right are positive for forward, front is positive for counterclockwise
        deltaRadians = (deltaTicks [1] * inPerTickRight - deltaTicks [0] * inPerTickLeft) * HEADING_CORRECTION / (trackWidth);

        perceivedHeading += deltaRadians;
        rotationRadians += deltaRadians;

        deltaCoords [0] = -Math.sin(perceivedHeading) * (deltaTicks [2] * inPerTickFront - deltaRadians * distanceToFront) + Math.cos(perceivedHeading) * (deltaTicks [1] * inPerTickRight + deltaTicks [0] * inPerTickLeft) / 2d;
        deltaCoords [1] = Math.cos(perceivedHeading) * (deltaTicks [2] * inPerTickFront - deltaRadians * distanceToFront) + Math.sin(perceivedHeading) * (deltaTicks [1] * inPerTickRight + deltaTicks [0] * inPerTickLeft) / 2d;

        position.x += deltaCoords [0];
        position.y += deltaCoords [1];
        perceivedCoords [0] += deltaCoords [0];
        perceivedCoords [1] += deltaCoords [1];

        deltaTime = elapsedTime.seconds() - lastTime;
        angularVelocity = deltaRadians / deltaTime;
        velocity.x = deltaCoords [0] / deltaTime;
        velocity.y = deltaCoords [1] / deltaTime;
        velocityRelativeToRobot.x = (deltaTicks [2] * inPerTickFront - deltaRadians * distanceToFront) / deltaTime;
        velocityRelativeToRobot.y = (deltaTicks [1] * inPerTickRight + deltaTicks [0] * inPerTickLeft) / (2d * deltaTime);

        lastTime = elapsedTime.seconds();
        elapsedTime.reset();
    }

    public void resetEncoders() {
        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        position = new Vector2();
        rotationRadians = 0;

//        leftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        horizontalEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rightEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public String positionToString() {return String.format("(%f, %f)", position.x, position.y); }

    public void telemetry(Telemetry telemetry) {
        telemetry.addLine();
        telemetry.addLine("THREE WHEEL ODOMETRY");

        telemetry.addLine(String.valueOf(deltaTime));

        telemetry.addData("Wheel ticks", String.format("%d, %d, %d", leftEncoder.getCurrentPosition(), rightEncoder.getCurrentPosition(), horizontalEncoder.getCurrentPosition()));

        telemetry.addLine("--------");
        telemetry.addLine("POSITION " + position);
        telemetry.addLine("ROTATION " + getRotationDegrees());
        telemetry.addLine("--------");

        telemetry.addLine("Velocity " + velocity.toString());
        telemetry.addData("Left Dead Wheel Position", leftEncoder.getCurrentPosition());
        telemetry.addData("Right Dead Wheel Position", rightEncoder.getCurrentPosition());
        telemetry.addData("Top Dead Wheel Position", horizontalEncoder.getCurrentPosition());
        telemetry.addData("netX", deltaCoords [0]);
        telemetry.addData("netY", deltaCoords [1]);
    }


    public void setPostion(Vector2 pos) {
        this.position = pos;
    }

    public void setRotation(double rotation) {
        this.rotationRadians = rotation;
    }

    public double getXCoordinate() {
        return position.x;
    }

    public double getYCoordinate() {
        return position.y;
    }

    public double getRotationRadians() {
        return rotationRadians;
    }

    public double getRotationDegrees() {
        return rotationRadians * 180 / Math.PI;
    }

    public Vector2 getVelocity() {
        return velocity;
    }
}