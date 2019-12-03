package org.firstinspires.ftc.teamcode;



import com.qualcomm.hardware.bosch.BNO055IMU;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.hardware.PIDCoefficients;

import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.util.DifferentialControlLoopCoefficients;

import com.qualcomm.robotcore.util.ElapsedTime;



import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;

import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;






@TeleOp(name="SigmaAutonomous")


public class SigmaAutonomous extends LinearOpMode{

    Robot robot;
    Detector detector;
    Object object;

    @Override
    public void runOpMode() throws InterruptedException {
        //robot = new Robot(this);
        detector = new Detector(this);
        detector.setMode(Detector.Mode.SKYSTONE);
        detector.startCamera();
        telemetry.addData("Hi", "HI");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()){
            object = detector.getObject();
            try{
                telemetry.addData("Position", getRedQuarryPositions());
                telemetry.addData("Position", object.getObjectLocation().y);
            }catch (NullPointerException e){

            }
            telemetry.update();
            if (isStopRequested()){
                detector.closeCamera();
            }
        }
        detector.closeCamera();
    }

    private int getBlueQuarryPositions(){
        double position = object.getObjectLocation().y;
        try{
            if (position > 60 && position < 250){
                return 2;
            }
            if (position > 250 && position < 380){
                return 3;
            }

        }catch (Exception e){

        }
        return 1;
    }

    private int getRedQuarryPositions(){
        double position = object.getObjectLocation().y;
        try{
            if ((position < 360 && position > 200)){
                return 2;
            }
            if ((position > 520 && position < 670) || (position > 50 && position < 200)){
                return 3;
            }
            if ((position > 360 && position < 520)){
                return 1;
            }

        }catch (Exception e){

        }
        return 2;

    }

   }