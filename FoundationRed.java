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






@Autonomous(name="FoundationRed")


public class FoundationRed extends LinearOpMode{

    Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this, SkystoneDetector.Mode.SKYSTONE);
        robot.setParkOnWall(true);
        robot.foundationRed();
    }



}