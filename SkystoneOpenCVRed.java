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

import android.app.Activity;
import android.view.SurfaceView;
import android.view.View;
import android.view.WindowManager;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;


/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Skystone OpenCV Red", group="Iterative Opmode")
// @Disabled
public class SkystoneOpenCVRed extends LinearOpMode implements CameraBridgeViewBase.CvCameraViewListener2
{
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftRearDrive = null;
    private DcMotor rightRearDrive = null;
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private double centerX;
    private double centerY;
    private CameraBridgeViewBase openCVCamera;
    private Mat hierarchy;
    private List<MatOfPoint> contours1 = new ArrayList<>();
    private List<MatOfPoint> contours2 = new ArrayList<>();
    private List<MatOfPoint> largestContours = new ArrayList<>();
    private MatOfPoint selectedContour1, selectedContour2;
    private Mat mRgba;
    private Mat subMat;
    private Mat mRgbaT;
    private Mat mGray2;
    private Rect boundRect1, boundRect2;
    private Mat mIntermediateMat;
    private Mat mGray;
    private int skystonePosition;
    private Scalar scalarHigh1, scalarLow1, scalarLow2, scalarHigh2;
    private double boundRect2Width, boundRect1x, boundRect2x, boundRect2Area;
    private String returnMessage;
    private double screenWidth;
    private Servo leftHook = null;
    private Servo rightHook = null;
    private double ticksPerInch = 44.5633840657;
    private BNO055IMU imu;
    private Activity activity;
    final SkystoneOpenCVRed t = this;
    private boolean parkOnWall;


    static {
        System.loadLibrary("opencv_java4");
    }

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


        activity = (Activity) hardwareMap.appContext;


        hierarchy = new Mat();
        scalarLow1 = new Scalar(16, 80, 80);
        scalarHigh1 = new Scalar(30, 255, 255);
        scalarLow2 = new Scalar(0, 0, 0);
        scalarHigh2 = new Scalar(180, 255, 30);

        activity.runOnUiThread(new Runnable() {
            @Override
            public void run() {


                activity.getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
                openCVCamera = activity.findViewById(R.id.activity_java_surface_view);
                openCVCamera.setCvCameraViewListener(t);
                openCVCamera.setCameraPermissionGranted();
                openCVCamera.setVisibility(SurfaceView.VISIBLE);
                openCVCamera.enableView();


            }
        });

        while (true){

            if (centerX < 100 || centerX > 530){
                telemetry.addData("Position", "Right");
                skystonePosition = 2;
            }else if(centerX < 280){
                telemetry.addData("Position", "Mid");
                skystonePosition = 1;
            }else if(centerX < 430){
                telemetry.addData("Position", "Left");
                skystonePosition = 0;
            }
            telemetry.addData("boundRect2Area", boundRect2Area);
            telemetry.addData("X Position", centerX);
            telemetry.addData("Y Position", centerY);


            telemetry.update();
            if (isStopRequested() || opModeIsActive()){
                activity.runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        if (openCVCamera != null){
                            openCVCamera.setVisibility(View.GONE);
                        }
                    }
                });
                openCVCamera.disableView();
                openCVCamera = null;
                break;
            }
        }

        parkOnWall = true;
    }

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        initializeRobot();
        waitForStart();
        switch (skystonePosition){
            case 0:
                quarryRed1();
                break;
            case 1:
                quarryRed2();
                break;
            case 2:
                quarryRed3();
                break;
            default: quarryRed2();
        }
        stopMotors();
    }




    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */


    /*
     * Code to run ONCE after the driver hits STOP
     */

    public void onCameraViewStarted(int width, int height) {
        screenWidth = width;
        mRgba = new Mat(height, width, CvType.CV_8U);
        mIntermediateMat = new Mat(height, width, CvType.CV_32F);
        mGray = new Mat(height, width/2, CvType.CV_8U);
        mGray2 = new Mat(height, width/2, CvType.CV_8U);

    }

    public void onCameraViewStopped() {
        mRgba.release();
        mGray.release();
        mIntermediateMat.release();
        mGray2.release();
        subMat.release();
    }

    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame){

        return detectSkystone(inputFrame);
    }

    private Mat rotateImage(Mat mat){
        Mat mRgbaT = mat.t();
        Core.flip(mat.t(), mRgbaT, 1);
        Imgproc.resize(mRgbaT, mRgbaT, mat.size());
        return mRgbaT;
    }

    public Mat detectSkystone(CameraBridgeViewBase.CvCameraViewFrame inputFrame){
        mRgba = inputFrame.rgba();
        Imgproc.cvtColor(mRgba, mIntermediateMat, Imgproc.COLOR_RGBA2BGR);
        Imgproc.cvtColor(mIntermediateMat, mIntermediateMat, Imgproc.COLOR_BGR2HSV);
        Core.inRange(mIntermediateMat, scalarLow1, scalarHigh1, mGray);
        Imgproc.findContours(mGray, contours1, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_NONE);
        try {
            selectedContour1 = contours1.get(0);
            for (MatOfPoint contour : contours1){
                if (Imgproc.contourArea(contour) > 10000){
                    if (Imgproc.contourArea(contour) > Imgproc.contourArea(selectedContour1)){
                        selectedContour1 = contour;
                    }
                }
            }
            boundRect1 = Imgproc.boundingRect(selectedContour1);
            subMat = new Mat(boundRect1.width, boundRect1.height, CvType.CV_8U);
            subMat = mRgba.submat(boundRect1);
            Imgproc.cvtColor(subMat, mIntermediateMat, Imgproc.COLOR_RGBA2BGR);
            Imgproc.cvtColor(mIntermediateMat, mIntermediateMat, Imgproc.COLOR_BGR2HSV);
            Core.inRange(mIntermediateMat, scalarLow2, scalarHigh2, mGray2);
            Imgproc.findContours(mGray2, contours2, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_NONE);
            subMat.copyTo(mRgba.submat(boundRect1));
            selectedContour2 = contours2.get(0);
            for (MatOfPoint contour : contours2){
                if (Imgproc.contourArea(contour) > 5000){
                    if (Imgproc.contourArea(contour) > Imgproc.contourArea(selectedContour2)){
                        selectedContour2 = contour;
                    }
                }
            }
            boundRect2 = Imgproc.boundingRect(selectedContour2);
            Imgproc.circle(mRgba, new Point(boundRect1.x + boundRect2.x + boundRect2.width/2.0, boundRect1.y + boundRect2.y + boundRect2.height/2.0), 10, new Scalar(255, 0, 0), 10);

            boundRect1x = boundRect1.y;
            boundRect2x = boundRect2.y;
            centerX = boundRect1.y + boundRect2.y + boundRect2.height/2.0;
            boundRect2Width = boundRect2.height;
            boundRect2Area = boundRect2.height * boundRect2.width;
            Imgproc.rectangle(mRgba, new Point(boundRect1.x + boundRect2.x, boundRect1.y + boundRect2.y), new Point(boundRect1.x + boundRect2.x + boundRect2.width, boundRect1.y + boundRect2.y + boundRect2.height), new Scalar(255, 0, 0), 10);
        }catch (Exception e){

        }
        contours1.clear();
        contours2.clear();
        mRgba = rotateImage(mRgba);
        return mRgba;
    }

    public Mat detectStones(CameraBridgeViewBase.CvCameraViewFrame inputFrame){
        mRgba = inputFrame.rgba();
        Imgproc.cvtColor(mRgba, mIntermediateMat, Imgproc.COLOR_RGBA2BGR);
        Imgproc.cvtColor(mIntermediateMat, mGray, Imgproc.COLOR_BGR2HSV);
        Core.inRange(mGray, scalarLow1, scalarHigh1, mGray);
        Imgproc.findContours(mGray, contours1, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_NONE);
        Imgproc.drawContours(mRgba, contours1, -1, new Scalar(0, 255, 0), 5);
        telemetry.addData("Contour Size", contours1.size());
        try{
            if (contours1.size() > 4){
                largestContours.add(contours1.get(0));
                for (MatOfPoint contour : contours1){
                    if (Imgproc.contourArea(contour) > Imgproc.contourArea(largestContours.get(0))){

                    }
                }
            }
            for (int i = 0; i < 4; i++){
                if (Imgproc.contourArea(contours1.get(i)) > 10000){
                    boundRect1 = Imgproc.boundingRect(contours1.get(i));
                    Imgproc.rectangle(mRgba, new Point(boundRect1.x, boundRect1.y), new Point(boundRect1.x + boundRect1.width, boundRect1.y + boundRect1.height), new Scalar(0, 255, 255), 3);
                }
            }
        }catch (Exception e){

        }
        telemetry.addData("Contour Size", contours1.size());
        contours1.clear();
        contours2.clear();
        mRgbaT = mRgba.t();
        Core.flip(mRgba.t(), mRgbaT, 1);
        Imgproc.resize(mRgbaT, mRgbaT, mRgba.size());
        return mRgbaT;


    }

    public Mat detectSkystoneOptimized(CameraBridgeViewBase.CvCameraViewFrame inputFrame){
        mRgba = inputFrame.rgba();
        Imgproc.rectangle(mRgba, new Point(20, 20), new Point(100, 100), new Scalar(3, 3, 100));
        Imgproc.rectangle(mRgba, new Point(120, 20), new Point(220, 100), new Scalar(3, 3, 100));
        Imgproc.rectangle(mRgba, new Point(240, 20), new Point(340, 100), new Scalar(3, 3, 100));
        mRgbaT = mRgba.t();
        Core.flip(mRgba.t(), mRgbaT, 1);
        Imgproc.resize(mRgbaT, mRgbaT, mRgba.size());
        return mRgbaT;
    }

    private void quarryRed1(){
        //move forward to make strafing easier
        moveInches(0.7, 2);
        //strafe left
        moveUsingEncoders(0.3, 11 * ticksPerInch, -11 * ticksPerInch, -11 * ticksPerInch, 11 * ticksPerInch);
        //collect skystone
        moveInches(0.7, 50);
        //turn to wall
        turnAccurate(imu, -175, 0.5);
        //go to wall
        moveInches(0.7, 47);
        //turn to skybridge
        turnAccurate(imu, -92, 0.5);
        //deposit stone
        moveInches(0.7, 60);
        //go back to retrieve the second stone
        moveInches(0.7, -73);
        //turn to collect next skystone
        turnAccurate(imu, 0, 0.5);
        //move the position 2 stone out of the way
        moveInches(0.7, 45);
        //move backwards to prepare for collecting the skystone
        moveInches(0.7, -8);
        //turn to collect the second stone
        turnAccurate(imu, 90, 0.5);
        //collect skystone
        moveInches(0.7, 14);
        turnAccurate(imu, -177, 0.5);
        moveInches(0.7, 37);
        turnAccurate(imu, -90, 0.5);
        moveInches(0.7, 80);
        moveInches(0.7, -15);
    }

    private void quarryRed2(){
        //Go forward to collect blocks
        moveInches(0.7, 50);
        //Turn 180 to face wall
        turnAccurate(imu, -179, 0.2);
        //Go to the wall
        if (parkOnWall){
            moveInches(0.7, 45);
        }else{
            moveInches(0.7, 22);
        }
        //Turn to face Skybridge
        turnAccurate(imu, -90, 0.1);
        //deliver Skystone
        moveInches(0.7, 54);
        //Go backwards to orient with the next Skystone
        if (parkOnWall){
            moveInches(0.7, -85);
        }else{
            moveInches(0.7, -83);
        }
        //Turn to face Skystone
        turnAccurate(imu, -5, 0.1);
        //
        //Collect Skystone the 2nd time
        //
        if (parkOnWall){
            moveInches(0.7, 48);
        }else{
            moveInches(0.7, 23);
        }
        //Turn 90 degrees
        turnAccurate(imu, -175, 0.1);
        //Go to the wall
        if (parkOnWall){
            moveInches(0.7, 52);
        }else{
            moveInches(0.7, 23);
        }
        //Turn to face Skybridge
        if (parkOnWall){
            turnAccurate(imu, -97, 0.1);
        }else{
            turnAccurate(imu, -90, 0.1);
        }
        //Deliver Skystone
        moveInches(0.7, 75);
        //Park
        moveInches(0.7, -15);

    }

    private void quarryRed3(){
        //Strafe right
        moveUsingEncoders(0.4, -15 * ticksPerInch, 15 * ticksPerInch, 15 * ticksPerInch, -15 * ticksPerInch);
        //Go forward to collect blocks
        moveInches(0.7, 50);
        //Turn 180 to face wall
        turnAccurate(imu, -179, 0.2);
        //Go to the wall
        if (parkOnWall){
            moveInches(0.7, 45);
        }else{
            moveInches(0.7, 22);
        }
        //Turn to face Skybridge
        turnAccurate(imu, -90, 0.1);
        //deliver Skystone
        moveInches(0.7, 54);
        //Go backwards to orient with the next Skystone
        if (parkOnWall){
            moveInches(0.7, -85);
        }else{
            moveInches(0.7, -83);
        }
        //Turn to face Skystone
        turnAccurate(imu, -5, 0.1);
        //
        //Collect Skystone the 2nd time
        //
        if (parkOnWall){
            moveInches(0.7, 48);
        }else{
            moveInches(0.7, 23);
        }
        //Turn 90 degrees
        turnAccurate(imu, -175, 0.1);
        //Go to the wall
        if (parkOnWall){
            moveInches(0.7, 52);
        }else{
            moveInches(0.7, 23);
        }
        //Turn to face Skybridge
        if (parkOnWall){
            turnAccurate(imu, -97, 0.1);
        }else{
            turnAccurate(imu, -90, 0.1);
        }
        //Deliver Skystone
        moveInches(0.7, 75);
        //Park
        moveInches(0.7, -15);

    }

    private void foundationBlue(){
        moveInches(0.3, -4);
        moveUsingEncoders(0.3, ticksPerInch * -17.5/2, ticksPerInch * 17.5/2, ticksPerInch * 17.5/2, -ticksPerInch * 17.5/2);
        moveInches(0.7, -28);
        rightHook.setPosition(0.3);
        leftHook.setPosition(0.7);
        runtime.reset();
        while (runtime.time() < 2){
            if (isStopRequested()){
                break;
            }
        }
        moveInches(0.4, 38);
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
        moveUsingEncoders(0.3, ticksPerInch * (17.5 ), ticksPerInch * -(17.5 ), ticksPerInch * -(17.5 ), ticksPerInch * (17.5 ));
        moveInches(0.3, -10);
        moveInches(0.3, 7);
        moveUsingEncoders(0.3, ticksPerInch * (40), ticksPerInch * -(40), ticksPerInch * -(40), ticksPerInch * (40));
        stopMotors();
    }

    private void foundationRed(){
        moveInches(0.3, -4);
        moveUsingEncoders(0.3, ticksPerInch * 17.5 + 30, ticksPerInch * -17.5 + 30, ticksPerInch * -17.5 + 30,  ticksPerInch * 17.5 + 30);
        moveInches(0.7, -28);
        rightHook.setPosition(0.3);
        leftHook.setPosition(0.7);
        runtime.reset();
        while (runtime.time() < 2){
            if (isStopRequested()){
                break;
            }
        }
        moveInches(0.4, 38);
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
        moveUsingEncoders(0.3, -ticksPerInch * (17.5 + 20), -ticksPerInch * -(17.5 + 20), -ticksPerInch * -(17.5 + 20), -ticksPerInch * (17.5 + 20));
        if (parkOnWall){
            moveInches(0.3, -3);
        }else{
            moveInches(0.3, -10);
        }
        moveUsingEncoders(0.3, -ticksPerInch * (40), -ticksPerInch * -(40), -ticksPerInch * -(40), -ticksPerInch * (40));
        stopMotors();
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
        return (8/(1 + Math.pow(e, -1 * (error/65))) - 4);
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
