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
import com.qualcomm.robotcore.hardware.HardwareMap;
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

public class CustomDrive {
    double x = 0.0, y = 0.0, theta = 0.0;


    //--Drive Motors--
    private DcMotorEx FL;
    private DcMotorEx BL;
    private DcMotorEx FR;
    private DcMotorEx BR;

    //--Pinpoint--
    private GoBildaPinpointDriver pinpoint;
    public CustomDrive(HardwareMap hardwareMap) {
        FL = hardwareMap.get(DcMotorEx.class, "FL");
        BL = hardwareMap.get(DcMotorEx.class, "BL");
        FR = hardwareMap.get(DcMotorEx.class, "FR");
        BR = hardwareMap.get(DcMotorEx.class, "BR");

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.resetPosAndIMU();
    }

    public void fieldrive(double fx, double fy, double rx, double heading){
        double botheading = Math.toRadians(heading);
        double rotx = fx*Math.cos(botheading) - fy*Math.sin(botheading);
        double roty = fx*Math.sin(botheading) + fy*Math.cos(botheading);

        drive(rotx, roty, rx);
    }
    public void drive(double x, double y, double rx){
        double powerFL = x - y - rx;
        double powerBL = x + y - rx;
        double powerFR = x + y + rx;
        double powerBR = x - y + rx;

        double max = Math.max(Math.abs(powerFL), Math.abs(powerBL));
        max = Math.max(max, Math.abs(powerFR));
        max = Math.max(max, Math.abs(powerBR));

        // 3. Normalize powers if they exceed 1.0
        if (max > 1.0) {
            powerFL /= max;
            powerBL /= max;
            powerFR /= max;
            powerBR /= max;
        }

        FL.setPower(powerFL);
        FR.setPower(powerFR);
        BL.setPower(powerBL);
        BR.setPower(powerBR);

    }
    public void updatePose() {
        pinpoint.update();
        x = pinpoint.getPosX(DistanceUnit.INCH);
        y = pinpoint.getPosY(DistanceUnit.INCH);
        theta = pinpoint.getHeading(AngleUnit.DEGREES);
    }


}
