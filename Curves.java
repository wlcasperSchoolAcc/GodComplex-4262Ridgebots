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

public class Curves {
    private Point start, p1, p2, end;

    private double[] arctable;
    private double arclength;

    public Curves(Point start, Point p1, Point p2, Point end){
        this.start = start;
        this.p1 = p1;
        this.p2 = p2;
        this.end = end;
        lookup();
    }

    //--Math--
    public Point getPosT(double t){
        double u = 1 - t;

        double x = (u*u*u * this.start.x) +
                (3 * u*u * t * this.p1.x) +
                (3 * u * t*t * this.p2.x) +
                (t*t*t * this.end.x);

        double y = (u*u*u * this.start.y) +
                (3 * u*u * t * this.p1.y) +
                (3 * u * t*t * this.p2.y) +
                (t*t*t * this.end.y);

        return new Point(x, y);
    }
    public Point getDerivT(double t){
        double u = 1 - t;

        double dx = (3 * u*u * (this.p1.x - this.start.x)) +
                (6 * u * t * (this.p2.x - this.p1.x)) +
                (3 * t*t * (this.end.x - this.p2.x));

        double dy = (3 * u*u * (this.p1.y - this.start.y)) +
                (6 * u * t * (this.p2.y - this.p1.y)) +
                (3 * t*t * (this.end.y - this.p2.y));

        return new Point(dx, dy);
    }

    //--arrays and stuff--

    private void lookup(){
        arctable = new double[101];
        arctable[0] = 0;

        double currTotDist = 0;
        Point prevPoint = getPosT(0);

        for(int i= 1; i <=100; i++){
            double t = (double) i /100;
            Point currPoint = getPosT(t);
            double dx = currPoint.x-prevPoint.x;
            double dy = currPoint.y - prevPoint.y;
            double segDist = Math.sqrt((dx*dx)+(dy*dy));
            currTotDist += segDist;
            arctable[i] = currTotDist;
            prevPoint = currPoint;
        }
        arclength = currTotDist;
    }
    public double getT(double target){
        if(target <= 0) return 0;
        if (target >= arclength) return 1;

        int low = 0;
        int high = 100;

        while (low<high){
            int mid = (low+high)/2;
            if(arctable[mid] == target){
                return (double) mid /100;
            }

            if(arctable[mid]<target){
                low = mid+1;
            } else {
                high = mid;
            }

        }

        int after = low;
        int before = low-1;
        double distafter = arctable[after];
        double distbefore = arctable[before];

        double segfrac = (target - distbefore)/(distafter-distbefore);

        double tb = (double) before /100;
        double ta = (double) after /100;

        return tb + (segfrac*(ta-tb));
    }


}
