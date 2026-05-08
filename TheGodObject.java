package org.firstinspires.ftc.teamcode;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.limelightvision.LLFieldMap;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLResultTypes.*;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import java.util.List;
import com.acmerobotics.*;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.*;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
@Autonomous(name = "PathFollow")
public class TheGodObject extends LinearOpMode{
    double x = 0.0, y = 0.0, theta = 0.0;

    static final double p = 0, i = 0, d = 0;


    //--Drive Motors--


    //--Pinpoint--
    private GoBildaPinpointDriver pinpoint;

    private PIDController xCon = new PIDController(p, i, d);
    private PIDController yCon = new PIDController(p, i, d);
    private PIDController thetaCon = new PIDController(p, i, d);

    private Curves path;
    private MoveScum profile;
    private ElapsedTime timer;

    private CustomDrive drive;

    // Constants for Feedforward
    // If your max speed is 40 inches/sec, Kv is roughly 1/40 (0.025) to convert inches/sec to motor power.
    private final double kV = 0.025;


    @Override
    public void runOpMode() throws InterruptedException {

        drive = new CustomDrive(hardwareMap);

        Point start = new Point(0, 0);
        Point p1 = new Point(0, 40);
        Point p2 = new Point(40, 0);
        Point end = new Point(40, 40);
        path = new Curves(start, p1, p2, end);

        profile = new MoveScum(40, 30);

        double targetheta = 0;
        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();

        timer = new ElapsedTime();

        double totPathDist = path.arclength;

        while(opModeIsActive()){
            drive.updatePose();
            // 2. ASK THE PACE CAR (Where should we be right now?)
            double currentTime = timer.seconds();
            MoveScum.Profile targetState = profile.calculate(totPathDist, currentTime);

            // 3. ASK THE MAP (Translate that distance into X, Y, and Velocity)
            double t = path.getT(targetState.pos);
            Point targetCoord = path.getPosT(t);
            Point rawDerivative = path.getDerivT(t);

            // Calculate the angle of our Feedforward velocity vector
            double pathAngle = Math.atan2(rawDerivative.y, rawDerivative.x);

            // Calculate pure Feedforward Power (Target Velocity * kV)
            double ffPowerX = Math.cos(pathAngle) * targetState.v * kV;
            double ffPowerY = Math.sin(pathAngle) * targetState.v * kV;

            // 4. ASK THE CORRECTORS (PID)
            // PID calculates the power needed to fix any drift
            double pidPowerX = xCon.calculate(drive.x, targetCoord.x);
            double pidPowerY = yCon.calculate(drive.y, targetCoord.y);
            double pidPowerHeading = thetaCon.calculate(drive.theta, targetheta);

            // 5. COMBINE AND DRIVE
            // Total Power = Proactive Push (FF) + Reactive Fix (PID)
            double finalDriveX = ffPowerX + pidPowerX;
            double finalDriveY = ffPowerY + pidPowerY;
            double finalTurn = pidPowerHeading;

            // 6. drive
            drive.fieldrive(finalDriveX, finalDriveY, finalTurn, drive.theta);

            telemetry.addData("Target Dist", targetState.pos);
            telemetry.addData("Error X", targetCoord.x - drive.x);
            telemetry.addData("Error Y", targetCoord.y - drive.y);
            telemetry.update();
        }

    }
}


