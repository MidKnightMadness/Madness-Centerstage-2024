package org.firstinspires.ftc.teamcode.Drivetrain;

//import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.ftccommon.internal.manualcontrol.parameters.ImuParameters;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Utility.Vector2;

import java.util.concurrent.TimeUnit;

//@Config
public class IMUOdometry implements Runnable{ // "implements runnable" is for multithreading

    //All variables mentioned here will be addessed as "this.VARIABLE_NAME"

    //Constants
    // Same for now
    double inPerTickLeft = 30.0d / 38828d;
    double inPerTickRight = 30.0d / 38358d;
    double inPerTickCenter = 30.0d / 38493d;
    double verticalWheelDistance = 3.0;
    // Need to retry this, somehow was not equalizing properly (centerDistanceTraveled - deltaRadians*distanceToFront)
    double lateralWheelDistance = 12.5 * 68.84d / 90.0d;


    //Tracking Time
    public double deltaTime = 0;
    double lastTime = 0;
    ElapsedTime elapsedTime;

    //Pose Tracking Variables
    public Vector2 position = new Vector2();
    Vector2 velocity = new Vector2();
    public double angularVelocity = 0.0;
    public double rotationRadians;

    //Odometry Wheel Variables
    int lastLeftTicks = 0;
    int deltaLeftTicks = 0;
    double leftDistanceMoved;

    int lastRightTicks = 0;
    int deltaRightTicks = 0;
    double rightDistanceMoved;

    int lastTopTicks = 0;
    int deltaTopTicks = 0;
    double topDistanceMoved;


    //Hardware Variables
    public DcMotorEx leftEncoder;
    public DcMotorEx horizontalEncoder;
    public DcMotorEx rightEncoder;
    BNO055IMU imu;
    BNO055IMU.Parameters parameters;
    Orientation angles;


    //Intermediate Variables OwO

    //Tick readings for encoders
    public int leftTicks;
    public int rightTicks;
    public int topTicks;

    //Angle change
    double deltaRadians;

    //Avg Distance of left and right encoders
    double forwardMovement;

    //Middle encoder movement from rotation
    double lateralMovementFromRotation;
    double trueLateralMovement;

    //sin and cos of robot angle from horizontal right (unit circle style)
    double rotSin;
    double rotCosine;

    //Change X and Y position of robot
    double netX;
    double netY;
    Vector2 lastVelocity = new Vector2();
    double lastExternalInputAngle = 0.0;
    double externalInputAngle = 0.0;
    double startingAngle = 0.0;



    // Low pass filter used to smooth position
    double prevLeft = 0.0;
    double prevRight = 0.0;
    double prevFront = 0.0;


    public IMUOdometry(HardwareMap hardwareMap, double startingAngleRadians, Vector2 startingPosition) {
        OpModeIsActive = true;
        elapsedTime = new ElapsedTime();

        //Initialize Motors
        leftEncoder = hardwareMap.get(DcMotorEx.class, "FL");
        rightEncoder = hardwareMap.get(DcMotorEx.class, "BL");
        horizontalEncoder = hardwareMap.get(DcMotorEx.class, "FR");

        //Reset Position
        this.rotationRadians = startingAngleRadians;
        this.position = startingPosition;
        externalInputAngle = startingAngleRadians;
        lastExternalInputAngle = startingAngleRadians;
        startingAngle = startingAngleRadians;

        // IMU
        parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.loggingEnabled = true;
        parameters.loggingTag = "imu";
        parameters.calibrationDataFile = "BN0055IMUCalibration.json";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    public void updatePosition() {

        //Updating Time and angle
        double currentTime = elapsedTime.time(TimeUnit.MICROSECONDS) / 1000000.0d;
        deltaTime = currentTime - lastTime;
        lastTime = currentTime;

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS);

        //Implementing low pass filter for smoothing
        //Note: change coefficients to make filter more/less responsive
        leftTicks = (int) (-0.9 * leftEncoder.getCurrentPosition()+ 0.1 * lastLeftTicks);
        rightTicks = (int) (-0.9 * rightEncoder.getCurrentPosition() + 0.1 * lastRightTicks);
        topTicks = (int) (0.9 * horizontalEncoder.getCurrentPosition() + 0.1 * lastTopTicks);

        //calculate change in tick reading
        deltaLeftTicks = leftTicks - lastLeftTicks;
        deltaRightTicks = rightTicks - lastRightTicks;
        deltaTopTicks = topTicks - lastTopTicks;

        //update last enocder values with new low pass filter values
        lastLeftTicks = leftTicks;
        lastRightTicks = rightTicks;
        lastTopTicks = topTicks;

        // raw distance from each encoder
        leftDistanceMoved = inPerTickLeft * deltaLeftTicks;
        rightDistanceMoved = inPerTickRight * deltaRightTicks;
        topDistanceMoved = inPerTickCenter * deltaTopTicks;

        // calculate change in angles
        externalInputAngle = angles.thirdAngle + startingAngle;
        deltaRadians = externalInputAngle - lastExternalInputAngle;//getDeltaRotation(leftDistanceMoved, rightDistanceMoved); //externalInputAngle - lastExternalInputAngle; // Added calibration for systematic error
        lastExternalInputAngle = externalInputAngle;
        angularVelocity = deltaRadians / deltaTime;
        rotationRadians += deltaRadians; //Finding integral part 1

        //average left and right encoder distance
        forwardMovement = (leftDistanceMoved + rightDistanceMoved) / 2.0;
        //Actual horizontal movement without rotational influence
        trueLateralMovement = topDistanceMoved - deltaRadians * verticalWheelDistance;

        //assigning sin and cos of rotation
        rotSin = Math.sin(rotationRadians);
        rotCosine = Math.cos(rotationRadians);

        //Calculating change X and Y position of robot
        netX = -forwardMovement * rotCosine + trueLateralMovement * rotSin;
        netY = forwardMovement * rotSin + trueLateralMovement * rotCosine;

        rotationRadians += .5 * deltaRadians; //Finding integral part 2

        //Calculating final x and y
        //Note: Changed signs since was reversed, had to re-swap variables
        this.position.x -= (12.0 / 10.75) * netX * (3.0d / 3.4d);
        this.position.y += (12.0 / 10.75) * netY * (3.0d / 3.4d);

        //getting x and y velocities
        velocity.x = - 0.75 * netX / deltaTime + 0.25 * lastVelocity.x;
        velocity.y = 0.75 * netY / deltaTime + 0.25 * lastVelocity.y;

        lastVelocity.x = velocity.x;
        lastVelocity.y = velocity.y;
    }


    public void resetEncoders() {
        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        position = new Vector2();
        rotationRadians = 0;

        leftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontalEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public String positionToString() {return String.format("(%f, %f)", position.x, position.y); }

    public void telemetry(Telemetry telemetry) {
        telemetry.addLine();
        telemetry.addLine("THREE WHEEL ODOMETRY");

        telemetry.addLine(String.valueOf(deltaTime));

        telemetry.addData("Wheel ticks", String.format("%d, %d, %d", leftTicks, rightTicks, topTicks));

        telemetry.addLine("--------");
        telemetry.addLine("POSITION " + position);
        telemetry.addLine("ROTATION " + getRotationDegrees());
        telemetry.addLine("--------");

        telemetry.addLine("Velocity " + velocity.toString());
        telemetry.addData("Left Dead Wheel Position", leftEncoder.getCurrentPosition());
        telemetry.addData("Right Dead Wheel Position", rightEncoder.getCurrentPosition());
        telemetry.addData("Top Dead Wheel Position", horizontalEncoder.getCurrentPosition());
        telemetry.addData("netX", netX);
        telemetry.addData("netY", netY);
        telemetry.addData("trueLateralMovement", trueLateralMovement);
    }


    public void setPostion(Vector2 pos) {
        this.position = pos;
    }

    public void setRotation(double rotation) {
        this.rotationRadians = rotation;
    }

    public double getDeltaRotation(double leftChange, double rightChange) {
        return (rightChange - leftChange) * 180.0d / (lateralWheelDistance * 2.0d * 178.1822d);
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


    // For multithreading
    public boolean OpModeIsActive = true;
    @Override
    public void run() { // Run, ideally at a regulated rate, the odometry algorithm
        // Will assume odometry object is already initialized
        // OpModeIsActive variable for emergency stop
        while(OpModeIsActive){

            this.updatePosition();
            try {
                Thread.sleep(20);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
    }
}