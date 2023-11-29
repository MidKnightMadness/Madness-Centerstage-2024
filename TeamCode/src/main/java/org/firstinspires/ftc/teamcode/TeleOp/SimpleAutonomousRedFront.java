//package org.firstinspires.ftc.teamcode.TeleOp;
//
//
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.Drivetrain.MecanumDrive;
//import org.firstinspires.ftc.teamcode.Drivetrain.Odometry;
//import org.firstinspires.ftc.teamcode.Drivetrain.PIDDrive;
//import org.firstinspires.ftc.teamcode.Utility.Vector2;
//
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//@TeleOp(name = "autonomous")
//public class SimpleAutonomousRed1 extends OpMode {
//    MecanumDrive mecanumDrive;
//    Odometry odometry;
//   //no pid drive this time PIDDrive pidDrive;
//    ElapsedTime timer;
//
//
//   //no target states double[][] targetStates{
//
//   // }
//
//    @Override
//    public void init() {
//        mecanumDrive = new MecanumDrive(hardwareMap, telemetry);
//        odometry = new Odometry(hardwareMap, Math.PI/2, new Vector2(0,0));
//       // pidDrive = new PIDDrive(odometry, 0, 0, 0, telemetry);
//        timer = new ElapsedTime();
//        timer.startTime();
//
//       // PIDDrive.setTargetState();
//    }
//
//    @Override
//    public void loop() {
//        odometry.updatePosition();
////        mecanumDrive.normalDrive();
//    }
//}
