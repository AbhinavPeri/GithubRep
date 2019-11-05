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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

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

@TeleOp(name="OpenCV", group="Iterative Opmode")
//@Disabled
public class OpenCV extends LinearOpMode implements CameraBridgeViewBase.CvCameraViewListener2
{
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor lF = null;
    private DcMotor rF = null;
    private DcMotor lR = null;
    private DcMotor rR = null;
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
    private Scalar scalarHigh1, scalarLow1, scalarLow2, scalarHigh2;
    private double boundRect2Width, boundRect1x, boundRect2x, boundRect2Area;
    private String returnMessage;
    private double screenWidth;



    static {
        try {
            System.loadLibrary("opencv_java3");
        } catch (UnsatisfiedLinkError e) {
            MyOpenCVLoader.loadOpenCV();
            // pass
        }
    }

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        lR  = hardwareMap.get(DcMotor.class, "leftRear");
        lF = hardwareMap.get(DcMotor.class, "leftFront");
        rR  = hardwareMap.get(DcMotor.class, "rightRear");
        rF = hardwareMap.get(DcMotor.class, "rightFront");

        lR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //leftDrive.setDirection(DcMotor.Direction.FORWARD);
        //rightDrive.setDirection(DcMotor.Direction.REVERSE);



        final Activity activity = (Activity) hardwareMap.appContext;
        final OpenCV t = this;
        hierarchy = new Mat();
        scalarLow1 = new Scalar(16, 100, 100);
        scalarHigh1 = new Scalar(30, 255, 255);
        scalarLow2 = new Scalar(0, 0, 0);
        scalarHigh2 = new Scalar(180, 255, 30);
        
        activity.runOnUiThread(new Runnable() {
            @Override
            public void run() {


                activity.getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
                openCVCamera = activity.findViewById(R.id.activity_java_surface_view);
                openCVCamera.setCvCameraViewListener(t);


            }
        });
        waitForStart();

        activity.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                openCVCamera.setVisibility(SurfaceView.VISIBLE);
                openCVCamera.enableView();
            }
        });

        while (opModeIsActive()){

            /*if (centerX < (720 - boundRect2Width)/3.0 + (boundRect2Width/2.0)){
                telemetry.addData("Position", "Right");
            }else if(centerX > ((720 - boundRect2Width)/3.0) + (boundRect2Width/2.0) && centerX < (2 * (720 - boundRect2Width)/3.0) + (boundRect2Width/2.0)){
                telemetry.addData("Position", "Mid");
            }else if(centerX > (2 * (720 - boundRect2Width)/3.0) + (boundRect2Width/2.0)){
                telemetry.addData("Position", "Left");
            }
            telemetry.addData("boundRect2Area", boundRect2Area);
            telemetry.addData("X Position", centerX);
            telemetry.addData("Y Position", centerY); */


            telemetry.update();
        }
        activity.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                openCVCamera.setVisibility(View.GONE);
            }
        });
        openCVCamera.disableView();
        openCVCamera = null;

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

        return detectSkystoneOptimized(inputFrame);
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
        mRgbaT = mRgba.t();
        Core.flip(mRgba.t(), mRgbaT, 1);
        Imgproc.resize(mRgbaT, mRgbaT, mRgba.size());
        return mRgbaT;
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

}
