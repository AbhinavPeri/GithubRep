/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;
import java.lang.Math;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="SigmaAuto", group="Linear Opmode")

public class SigmaAuto extends LinearOpMode{

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftRearDrive = null;
    private DcMotor rightRearDrive = null;
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private Servo leftHook = null;
    private Servo rightHook = null;
    private double ticksPerInch = 44.5633840657;
    private BNO055IMU imu;


    public void initializeRobot(){
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftRearDrive  = hardwareMap.get(DcMotor.class, "leftRear");
        rightRearDrive = hardwareMap.get(DcMotor.class, "rightRear");
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
        rightHook = hardwareMap.servo.get("rightHook");
        leftHook = hardwareMap.servo.get("leftHook");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftRearDrive.setDirection(DcMotor.Direction.FORWARD);
        rightRearDrive.setDirection(DcMotor.Direction.REVERSE);
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        while(!imu.isGyroCalibrated()){
            if (isStopRequested()){
                break;
            }
        }
    }

    @Override
    public void runOpMode(){
        waitForStart();
        initializeRobot();
        quarryRed2();

    }

    public void askPath(){
        telemetry.addData("Path", "Do you want to run quarryRed1? Press (A) for yes and (B) for no");
        telemetry.update();
        while (!gamepad1.a || !gamepad1.b){
            if (isStopRequested()){
                break;
            }
        }
        if (gamepad1.a){
            quarryRed1();
        }else if (gamepad1.b){

        }
        telemetry.addData("Path", "Do you want to run quarryRed2? Press (A) for yes and (B) for no");
        telemetry.update();
        while (!gamepad1.a || !gamepad1.b){
            if (isStopRequested()){
                break;
            }
        }
        if (gamepad1.a){
            quarryRed2();
        }else if (gamepad1.b){

        }
        telemetry.addData("Path", "Do you want to run foundationBlue? Press (A) for yes and (B) for no");
        telemetry.update();
        while (!gamepad1.a || !gamepad1.b){
            if (isStopRequested()){
                break;
            }
        }
        if (gamepad1.a){
            foundationBlue();
        }else if (gamepad1.b){

        }
    }

    private void quarryRed1(){
        //!!!strafe left here!!!
        moveInches(0.7, 60);
        turnAccurate(imu, 180, 0.1);
        moveInches(0.7, 10);
        turnAccurate(imu, -90, 0.1);
        moveInches(0.7, 48);
        moveInches(0.7, -72);
        turnAccurate(imu, 0, 0.1);
        moveInches(0.7, 10);
        turnAccurate(imu, 180, 0.1);
        moveInches(0.7, 10);
        turnAccurate(imu, -90, 0.1);
        moveInches(0.7, 70);
        moveInches(0.7, -22);
        stopMotors();
    }

    private void quarryRed2(){
        //Go forward to collect blocks
        moveInches(0.7, 60);
        //Turn 180 to face wall
        turnAccurate(imu, 180, 0.1);
        //Go to the wall
        moveInches(0.7, 55);
        //Turn to face Skybridge
        turnAccurate(imu, -90, 0.1);
        //deliver Skystone
        moveInches(0.7, 48);
        //Go backwards to orient with the next Skystone
        moveInches(0.7, -82);
        //Turn to face Skystone
        turnAccurate(imu, -5, 0.1);
        //
        //Collect Skystone the 2nd time
        //
        moveInches(0.7, 48);
        //Turn 90 degrees
        turnAccurate(imu, -90, 0.1);
        //Turn to face wall
        turnAccurate(imu, -175, 0.1);
        //Go to the wall
        moveInches(0.7, 52);
        //Turn to face Skybridge
        turnAccurate(imu, -90, 0.1);
        //Deliver Skystone
        moveInches(1, 70);
        //Park
        moveInches(0.7, -11);

    }

    private void foundationBlue(){
        moveInches(0.3, -4);
        moveUsingEncoders(0.3, ticksPerInch * -17.5/2, ticksPerInch * 17.5/2, ticksPerInch * 17.5/2, -ticksPerInch * 17.5/2);
        moveInches(0.7, -26.5);
        rightHook.setPosition(0.3);
        leftHook.setPosition(0.7);
        runtime.reset();
        while (runtime.time() < 2){
            if (isStopRequested()){
                break;
            }
        }
        moveInches(0.4, 36);
        rightHook.setPosition(0.7);
        leftHook.setPosition(0.3);
        runtime.reset();
        while (runtime.time() < 2){
            if (isStopRequested()){
                break;
            }

        }
        rightHook.setPosition(0.5);
        leftHook.setPosition(0.5);
        moveUsingEncoders(0.3, ticksPerInch * (17.5 + 34), ticksPerInch * -(17.5 + 34), ticksPerInch * -(17.5 + 34), ticksPerInch * (17.5 + 34));
        stopMotors();
    }

    private double getAveragePosition(){
        return (leftRearDrive.getCurrentPosition() + leftFrontDrive.getCurrentPosition() + rightRearDrive.getCurrentPosition() + rightFrontDrive.getCurrentPosition())/4.0;
    }

    private void stopMotors(){
        leftFrontDrive.setPower(0);
        leftRearDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightRearDrive.setPower(0);
    }

    private void moveUsingEncoders(double power, double leftRear, double leftFront, double rightRear, double rightFront){
        leftFront = Math.round(leftFront);
        leftRear = Math.round(leftRear);
        rightFront = Math.round(rightFront);
        rightRear = Math.round(rightRear);
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setTargetPosition((int) -leftFront);
        leftRearDrive.setTargetPosition((int) -leftRear);
        rightFrontDrive.setTargetPosition((int) -rightFront);
        rightRearDrive.setTargetPosition((int) -rightRear);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightRearDrive.setPower(power);
        leftRearDrive.setPower(power);
        rightFrontDrive.setPower(power);
        leftFrontDrive.setPower(power);
        while (rightFrontDrive.isBusy() && leftFrontDrive.isBusy() && leftRearDrive.isBusy() && rightRearDrive.isBusy() && !isStopRequested()) {
            if (isStopRequested()){
                break;
            }
        }
        stopMotors();
    }

    public void moveInches(double power, double inches){
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setTargetPosition((int) Math.round(-ticksPerInch * inches));
        leftRearDrive.setTargetPosition((int) Math.round(-ticksPerInch * inches));
        rightFrontDrive.setTargetPosition((int) Math.round(-ticksPerInch * inches));
        rightRearDrive.setTargetPosition((int) Math.round(-ticksPerInch * inches));

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightRearDrive.setPower(power);
        leftRearDrive.setPower(power);
        rightFrontDrive.setPower(power);
        leftFrontDrive.setPower(power);
        while (rightFrontDrive.isBusy() && leftFrontDrive.isBusy() && leftRearDrive.isBusy() && rightRearDrive.isBusy() && !isStopRequested()) {
            if (isStopRequested()){
                break;
            }
        }
        stopMotors();
    }

    private void turnAccurate(BNO055IMU imu, double degrees, double threshold)
    {

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double initAngle = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES).firstAngle;
        double error = calculateAngularError(convertImuRegular(degrees), convertImuRegular(initAngle));
        double pow = sigmoidFunction1(error);
        double time = sigmoidFunction2(Math.abs(error));
        runtime.reset();
        while (error > threshold || error < -threshold && !isStopRequested()){
            telemetry.addData("Error:", error);
            telemetry.addData("Init Angle", initAngle);
            telemetry.update();

            rightRearDrive.setPower(pow);
            leftRearDrive.setPower(-pow);
            rightFrontDrive.setPower(pow);
            leftFrontDrive.setPower(-pow);

            error = calculateAngularError(convertImuRegular(imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES).firstAngle), convertImuRegular(degrees));
            pow = sigmoidFunction1(error);
            if (runtime.time() > time){
                break;
            }
            if (isStopRequested()){
                break;
            }
        }
        stopMotors();
    }
    private double convertImuRegular(double angle){
        if (angle > 0){
            return 360 - angle;
        }else{
            return 0 + (-1 * angle);
        }
    }
    private double sigmoidFunction1(double error){
        double e = Math.E;
        return (1.4/(1 + Math.pow(e, -1 * (error/20.0)))) - 0.7;
    }

    private double sigmoidFunction2(double error){
        double e = Math.E;
        return (8/(1 + Math.pow(e, -1 * (error/50))) - 4);
    }

    private double calculateAngularError(double currentAngle, double targetAngle){
        //getting both distances
        double diff1 = currentAngle - targetAngle;
        double diff2;
        if (diff1 > 0){
            diff2 = (360 - diff1);
            diff1 *= -1;
        }else {
            diff2 = (360 + diff1) * -1;
            diff1 *= -1;
        }

        //seeing which one is bigger and returning the smaller one
        if (Math.abs(diff1) <= Math.abs(diff2)){
            return diff1;
        }else{
            return diff2;
        }
    }
}

