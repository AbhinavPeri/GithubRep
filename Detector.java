package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.view.SurfaceView;
import android.view.WindowManager;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.lang.annotation.Target;
import java.util.ArrayList;
import java.util.List;
import java.util.Properties;

public class Detector implements CameraBridgeViewBase.CvCameraViewListener2 {

    private Activity activity;
    private Mode mode;
    private CameraBridgeViewBase openCVCamera;
    private Detector t = this;

    private Mat processFrame, releaseFrame, mask, hierarchy, subMat;
    private List<MatOfPoint> contourList = new ArrayList<>();
    private List<MatOfPoint> selectedContours = new ArrayList<>();
    private List<Rect> boundingRectangles = new ArrayList<>();
    private LinearOpMode context;
    private boolean displayScreen;
    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    private Gamepad gamepad;
    private Object object;
    private Object desiredBlueFoundation, desiredRedFoundation, middleSkystone;

    private Scalar blueScalarLow, blueScalarHigh, redScalarLow, redScalarHigh, yellowScalarLow, yellowScalarHigh, blackScalarLow, blackScalarHigh;

    static {
        System.loadLibrary("opencv_java4");
    }

    enum Mode{
        BLUEFOUNDATION, REDFOUNDATION, SKYSTONE
    }


    public Detector(LinearOpMode opMode){
        context = opMode;
        gamepad = context.gamepad1;
        telemetry = context.telemetry;
        hardwareMap = context.hardwareMap;
        activity = (Activity) context.hardwareMap.appContext;
        activity.runOnUiThread(() -> {
            activity.getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
            openCVCamera = activity.findViewById(R.id.activity_java_surface_view);
            openCVCamera.setCvCameraViewListener(t);
            openCVCamera.setCameraPermissionGranted();

        });
        yellowScalarLow = new Scalar(16, 80, 80);
        yellowScalarHigh = new Scalar(30, 255, 255);
        blackScalarLow = new Scalar(0, 0, 0);
        blackScalarHigh = new Scalar(180, 255, 30);
        redScalarLow = new Scalar(0, 70, 50);
        redScalarHigh = new Scalar(10, 255, 255);
        blueScalarLow = new Scalar(100, 150, 0);
        blueScalarHigh = new Scalar(140, 255, 255);
        displayScreen = true;
        mode = Mode.SKYSTONE;
    }

    public Detector(LinearOpMode opMode, String tuningFilePath){
        Properties props = new Properties();
        context = opMode;
        gamepad = context.gamepad1;
        telemetry = context.telemetry;
        hardwareMap = context.hardwareMap;
        activity = (Activity) context.hardwareMap.appContext;
        activity.runOnUiThread(() -> {
            activity.getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
            openCVCamera = activity.findViewById(R.id.activity_java_surface_view);
            openCVCamera.setCvCameraViewListener(t);
            openCVCamera.setCameraPermissionGranted();

        });
        FileUtil.loadPropertyFile(tuningFilePath, props);
        yellowScalarLow = new Scalar(FileUtil.readIntoDoubleArray("yellowScalarLow", props));
        yellowScalarHigh = new Scalar(FileUtil.readIntoDoubleArray("yellowScalarHigh", props));
        blackScalarLow = new Scalar(FileUtil.readIntoDoubleArray("blackScalarLow", props));
        blackScalarHigh = new Scalar(FileUtil.readIntoDoubleArray("blackScalarHigh", props));
        redScalarLow = new Scalar(FileUtil.readIntoDoubleArray("redScalarLow", props));
        redScalarHigh = new Scalar(FileUtil.readIntoDoubleArray("redScalarHigh", props));
        blueScalarLow = new Scalar(FileUtil.readIntoDoubleArray("blueScalarLow", props));
        blueScalarHigh = new Scalar(FileUtil.readIntoDoubleArray("blueScalarHigh", props));
        displayScreen = true;
        mode = Mode.SKYSTONE;
    }

    public Object getObject(){
        return object;
    }

    public void postObjectTelemetryInfo(){
        telemetry.addData("Object", object.getObjectName());
        telemetry.addData("Object Location", "x: " + object.getObjectLocation().x + ", y: " + object.getObjectLocation().y);
        telemetry.addData("Object Width", object.getObjectWidth());
        telemetry.addData("Object Height", object.getObjectHeight());
        telemetry.addData("Object Size", object.getObjectSize());
        telemetry.update();
    }

    public void startCamera(){
        activity.runOnUiThread(() -> {
            if (displayScreen){
                openCVCamera.setVisibility(SurfaceView.VISIBLE);
            }
            openCVCamera.enableView();
        });
    }

    public void closeCamera(){
        activity.runOnUiThread(() -> {
            openCVCamera.setVisibility(SurfaceView.GONE);
            openCVCamera.disableView();
        });

    }

    public void displayImages(boolean bool){
        displayScreen = bool;
    }

    private Mat rotateImage(Mat mat){
        Mat releaseMat = new Mat();
        Core.rotate(mat, releaseMat, Core.ROTATE_90_CLOCKWISE);
        Imgproc.resize(releaseMat, releaseMat, mat.size());
        return releaseMat;
    }

    private void releaseMat(Mat mat){
        if (mat != null){
            mat.release();
        }
    }

    public void setMode(Mode o){
        mode = o;
    }

    private Mat detectSkystone(CameraBridgeViewBase.CvCameraViewFrame inputFrame){
        releaseFrame = inputFrame.rgba();
        Rect boundRect1;
        Imgproc.cvtColor(releaseFrame, processFrame, Imgproc.COLOR_RGBA2BGR);
        Imgproc.cvtColor(processFrame, processFrame, Imgproc.COLOR_BGR2HSV);
        Core.inRange(processFrame, yellowScalarLow, yellowScalarHigh, mask);
        Imgproc.findContours(mask, contourList, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_NONE);
        try {
            selectedContours.add(contourList.get(0));
            for (MatOfPoint contour : contourList){
                if (Imgproc.contourArea(contour) > 10000){
                    if (Imgproc.contourArea(contour) > Imgproc.contourArea(selectedContours.get(0))){
                        selectedContours.set(0, contour);
                    }
                }
            }
            contourList.clear();
            boundingRectangles.add(Imgproc.boundingRect(selectedContours.get(0)));
            boundRect1 = boundingRectangles.get(0);
            subMat = new Mat(boundRect1.width, boundRect1.height, CvType.CV_8U);
            subMat = releaseFrame.submat(boundRect1);
            Imgproc.cvtColor(subMat, processFrame, Imgproc.COLOR_RGBA2BGR);
            Imgproc.cvtColor(processFrame, processFrame, Imgproc.COLOR_BGR2HSV);
            Core.inRange(processFrame, blackScalarLow, blackScalarHigh, mask);
            Imgproc.findContours(mask, contourList, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_NONE);
            subMat.copyTo(releaseFrame.submat(boundRect1));
            selectedContours.add(contourList.get(0));
            for (MatOfPoint contour : contourList){
                if (Imgproc.contourArea(contour) > 5000){
                    if (Imgproc.contourArea(contour) > Imgproc.contourArea(selectedContours.get(1))){
                        selectedContours.set(1, contour);
                    }
                }
            }
            boundRect1 = Imgproc.boundingRect(selectedContours.get(1));
            boundRect1.x = boundingRectangles.get(0).x + boundRect1.x;
            boundRect1.y = boundingRectangles.get(0).y + boundRect1.y;
            Object skystoneImage = new Object(boundRect1, "skystoneImage");
            skystoneImage.drawBoundingRectangle(releaseFrame);
            skystoneImage.drawCenter(releaseFrame);
            object = skystoneImage;
            contourList.clear();
            selectedContours.clear();
            boundingRectangles.clear();
        }catch (Exception e){

        }
        releaseFrame = rotateImage(releaseFrame);
        return releaseFrame;
    }

    private Mat redFoundation(CameraBridgeViewBase.CvCameraViewFrame inputFrame){
        releaseFrame = inputFrame.rgba();
        Imgproc.cvtColor(releaseFrame, processFrame, Imgproc.COLOR_RGBA2BGR);
        Imgproc.cvtColor(processFrame, processFrame, Imgproc.COLOR_BGR2HSV);
        Core.inRange(processFrame, redScalarLow, redScalarHigh, mask);
        Imgproc.findContours(mask, contourList, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_NONE);
        try {
            selectedContours.add(contourList.get(0));
            for (MatOfPoint contour : contourList){
                if (Imgproc.contourArea(contour) > 10000){
                    if (Imgproc.contourArea(contour) > Imgproc.contourArea(selectedContours.get(0))){
                        selectedContours.set(0, contour);
                    }
                }
            }
            contourList.clear();
            boundingRectangles.add(Imgproc.boundingRect(selectedContours.get(0)));
            Object redFoundation = new Object(selectedContours.get(0), "Red Foundation");
            redFoundation.drawCenter(releaseFrame);
            redFoundation.drawBoundingRectangle(releaseFrame);
            object = redFoundation;
            boundingRectangles.clear();
            selectedContours.clear();
        }catch (Exception e){

        }
        return rotateImage(releaseFrame);
    }

    private Mat blueFoundation(CameraBridgeViewBase.CvCameraViewFrame inputFrame){
        releaseFrame = inputFrame.rgba();
        Imgproc.cvtColor(releaseFrame, processFrame, Imgproc.COLOR_RGBA2BGR);
        Imgproc.cvtColor(processFrame, processFrame, Imgproc.COLOR_BGR2HSV);
        Core.inRange(processFrame, blueScalarLow, blueScalarHigh, mask);
        Imgproc.findContours(mask, contourList, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_NONE);
        try {
            selectedContours.add(contourList.get(0));
            for (MatOfPoint contour : contourList){
                if (Imgproc.contourArea(contour) > 10000){
                    if (Imgproc.contourArea(contour) > Imgproc.contourArea(selectedContours.get(0))){
                        selectedContours.set(0, contour);
                    }
                }
            }
            contourList.clear();
            boundingRectangles.add(Imgproc.boundingRect(selectedContours.get(0)));
            Object blueFoundation = new Object(selectedContours.get(0), "Blue Foundation");
            blueFoundation.drawCenter(releaseFrame);
            blueFoundation.drawBoundingRectangle(releaseFrame);
            object = blueFoundation;
            boundingRectangles.clear();
            selectedContours.clear();
        }catch (Exception e){

        }
        return rotateImage(releaseFrame);
    }

    @Override
    public void onCameraViewStarted(int width, int height) {
        releaseFrame = new Mat(height, width, CvType.CV_8U);
        mask = new Mat(height, width, CvType.CV_8U);
        processFrame = new Mat(height, width, CvType.CV_32F);
        hierarchy = new Mat();


    }

    @Override
    public void onCameraViewStopped() {
        releaseMat(releaseFrame);
        releaseMat(processFrame);
        releaseMat(mask);
        releaseMat(hierarchy);
    }

    @Override
    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {
        switch (mode){
            case SKYSTONE:
                return detectSkystone(inputFrame);
            case REDFOUNDATION:
                return redFoundation(inputFrame);
            case BLUEFOUNDATION:
                return blueFoundation(inputFrame);
            default:
                return rotateImage(inputFrame.rgba());
        }
    }



}
