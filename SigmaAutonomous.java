package org.firstinspires.ftc.teamcode;



import com.qualcomm.hardware.bosch.BNO055IMU;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.hardware.PIDCoefficients;

import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.util.DifferentialControlLoopCoefficients;

import com.qualcomm.robotcore.util.ElapsedTime;



import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;

import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;






@Autonomous(name="SigmaAutonomous")


public class SigmaAutonomous extends LinearOpMode{

    Robot robot;
    Detector detector;
    Object object;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this);
        detector = new Detector(this);
        detector.setMode(Detector.Mode.SKYSTONE);
        waitForStart();
        detector.startCamera();
        while (opModeIsActive()){
            object = detector.getObject();
            telemetry.addData("Object position", object.getObjectLocation().x + " " + object.getObjectLocation().y);
            telemetry.update();
            if (isStopRequested()){
                detector.closeCamera();
            }
        }
        detector.closeCamera();
    }





    // Declare OpMode members.

   }