package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.view.SurfaceView;
import android.view.WindowManager;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
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
import java.util.Properties;

@TeleOp(name = "Detector Lighting Tuning", group = "Linear OpMode")
public class CreateTuningFileOpMode extends LinearOpMode implements CameraBridgeViewBase.CvCameraViewListener2{

    private ElapsedTime runtime = new ElapsedTime();
    private Activity activity;
    private CameraBridgeViewBase openCVCamera;
    private CreateTuningFileOpMode t = this;
    private Scalar blueScalarLow, blueScalarHigh, redScalarLow, redScalarHigh, yellowScalarLow, yellowScalarHigh, blackScalarLow, blackScalarHigh, centerScalar;
    private Mat processFrame, colorFrame, mask, hierarchy, subMat;
    private List<MatOfPoint> contourList = new ArrayList<>();
    private List<MatOfPoint> selectedContours = new ArrayList<>();
    private List<Rect> boundingRectangles = new ArrayList<>();
    private boolean moveToNextScalar, grayScaleMode;
    private int scalarValue = 0;
    private int scalarLowOrHigh = 0;
    private Color color;
    private Properties props = new Properties();
    private Controller controller;

    static {
        System.loadLibrary("opencv_java4");
    }

    private enum Color{
        BLUE, RED, BLACK, YELLOW
    }

    @Override
    public void runOpMode() throws InterruptedException {
        runtime.reset();
        controller = new Controller(gamepad1);
        activity = (Activity) hardwareMap.appContext;
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
        waitForStart();
        activity.runOnUiThread(() ->{
            openCVCamera.setVisibility(SurfaceView.VISIBLE);
            openCVCamera.enableView();
        });

        color = Color.YELLOW;
        tuneColor(Color.YELLOW);
        moveToNextScalar = false;
        tuneColor(Color.BLACK);
        moveToNextScalar = false;
        tuneColor(Color.RED);
        moveToNextScalar = false;
        tuneColor(Color.BLUE);
        props.setProperty("yellowScalarLow", yellowScalarLow.val[0] + " " + yellowScalarLow.val[1] + " " + yellowScalarLow.val[2]);
        props.setProperty("yellowSclarHigh", yellowScalarHigh.val[0] + " " + yellowScalarHigh.val[1] + " " + yellowScalarHigh.val[2]);
        props.setProperty("blackScalarLow", blackScalarLow.val[0] + " " + blackScalarLow.val[1] + " " + blackScalarLow.val[2]);
        props.setProperty("blackScalarHigh", blackScalarHigh.val[0] + " " + blackScalarHigh.val[1] + " " + blackScalarHigh.val[2]);
        props.setProperty("redScalarLow", redScalarLow.val[0] + " " + redScalarLow.val[1] + " " + redScalarLow.val[2]);
        props.setProperty("redScalarHigh", redScalarHigh.val[0] + " " + redScalarHigh.val[1] + " " + redScalarHigh.val[2]);
        props.setProperty("blueScalarLow", blueScalarLow.val[0] + " " + blueScalarLow.val[1] + " " + blueScalarLow.val[2]);
        props.setProperty("blueScalarHigh", blueScalarHigh.val[0] + " " + blueScalarHigh.val[1] + " " + blueScalarHigh.val[2]);
        FileUtil.createNewPropertyFile("Detector2.properties", props);



        activity.runOnUiThread(() -> {
            openCVCamera.setVisibility(SurfaceView.GONE);
            openCVCamera.disableView();
        });

    }

    private void tuneColor(Color color){
        String colorString;
        switch (color){
            case RED:
                colorString = "Red";
                break;
            case BLUE:
                colorString = "Blue";
                break;
            case BLACK:
                colorString = "Black";
                break;
            case YELLOW:
                colorString = "Yellow";
                break;
            default:
                throw new IllegalStateException("Unexpected value: " + color);
        }
        while (!moveToNextScalar && !isStopRequested()){
            controller.Update();
            telemetry.addData("Controls", "Use the Right Bumper and the Left Bumper to switch between Scalar values and Dpad Up and Down to increase or decrease the values");
            telemetry.addData("Controls", "Use Dpad Right and Dpad Left to toggle Gray-scale Mode and use (X) and (B) to switch between lower and higher scalars");
            switch (scalarLowOrHigh){
                case 0:
                    telemetry.addData("Current Scalar:", colorString + " Scalar Low");
                    if (gamepad1.dpad_down){
                        selectScalarLower(color).val[scalarValue] -= 0.0003;
                    }
                    if (gamepad1.dpad_up){
                        selectScalarLower(color).val[scalarValue] += 0.0003;
                    }
                    break;
                case 1:
                    telemetry.addData("Current Scalar:", colorString + " Scalar High");
                    if (gamepad1.dpad_down){
                        selectScalarHigher(color).val[scalarValue] -= 0.0003;
                    }
                    if (gamepad1.dpad_up){
                        selectScalarHigher(color).val[scalarValue] += 0.0003;
                    }
                    break;
            }
            switch (scalarValue){
                case 0:
                    telemetry.addData("Current Scalar Value:", "Hue");
                    break;
                case 1:
                    telemetry.addData("Current Scalar Value:", "Saturation");
                    break;
                case 2:
                    telemetry.addData("Current Scalar Value:", "Vue");
                    break;
            }
            if (controller.RightBumper == Controller.ButtonState.JUST_RELEASED){
                scalarValue ++;
            }
            if (scalarValue > 2){
                scalarValue = 0;
            }
            if (controller.LeftBumper == Controller.ButtonState.JUST_RELEASED){
                scalarValue --;
            }
            if (scalarValue < 0){
                scalarValue = 2;
            }
            if (gamepad1.dpad_left){
                grayScaleMode = false;
            }
            if (gamepad1.dpad_right){
                grayScaleMode = true;
            }
            if (gamepad1.x){
                scalarLowOrHigh = 0;
            }
            if (gamepad1.b){
                scalarLowOrHigh = 1;
            }
            if (controller.AState == Controller.ButtonState.JUST_RELEASED){
                moveToNextScalar = true;
            }

            if (controller.YState == Controller.ButtonState.JUST_RELEASED){
                switch (scalarLowOrHigh){
                    case 0:
                        selectScalarLower(color).val[0] = centerScalar.val[0];
                        selectScalarLower(color).val[1] = centerScalar.val[1];
                        selectScalarLower(color).val[2] = centerScalar.val[2];
                        break;
                    case 1:
                        selectScalarHigher(color).val[0] = centerScalar.val[0];
                        selectScalarHigher(color).val[1] = centerScalar.val[1];
                        selectScalarHigher(color).val[2] = centerScalar.val[2];
                        break;
                    default:
                        throw new IllegalStateException("Unexpected value: " + scalarLowOrHigh);
                }
            }

            telemetry.addData("Scalar Low Values:", "Hue: " + selectScalarLower(color).val[0] + " Saturation: " + selectScalarLower(color).val[1] + " Vue: " + selectScalarLower(color).val[2]);
            telemetry.addData("Scalar High Values:", "Hue: " + selectScalarHigher(color).val[0] + " Saturation: " + selectScalarHigher(color).val[1] + " Vue: " + selectScalarHigher(color).val[2]);
            telemetry.addData("Next Color", "Press (A) to go to the next set of Scalars (If you accidentally switch, press the stop button and redo the tuning)");
            telemetry.addData("Runtime", runtime.milliseconds());
            telemetry.update();
            if (selectScalarLower(color).val[0] < 0){
                selectScalarLower(color).val[0] = 0;
            }
            if (selectScalarLower(color).val[0] > 180){
                selectScalarLower(color).val[0] = 180;
            }
            if (selectScalarLower(color).val[1] < 0){
                selectScalarLower(color).val[1] = 0;
            }
            if (selectScalarLower(color).val[1] > 255){
                selectScalarLower(color).val[1] = 255;
            }
            if (selectScalarLower(color).val[2] < 0){
                selectScalarLower(color).val[2] = 0;
            }
            if (selectScalarLower(color).val[2] > 255){
                selectScalarLower(color).val[2] = 255;
            }
            if (selectScalarHigher(color).val[0] < 0){
                selectScalarHigher(color).val[0] = 0;
            }
            if (selectScalarHigher(color).val[0] > 180){
                selectScalarHigher(color).val[0] = 180;
            }
            if (selectScalarHigher(color).val[1] < 0){
                selectScalarHigher(color).val[1] = 0;
            }
            if (selectScalarHigher(color).val[1] > 255){
                selectScalarHigher(color).val[1] = 255;
            }
            if (selectScalarHigher(color).val[2] < 0){
                selectScalarHigher(color).val[2] = 0;
            }
            if (selectScalarHigher(color).val[2] > 255){
                selectScalarHigher(color).val[2] = 255;
            }


        }
        if (isStopRequested()){
            activity.runOnUiThread(() -> {
                openCVCamera.setVisibility(SurfaceView.GONE);
                openCVCamera.disableView();
            });
        }
    }
    private Scalar selectScalarLower(Color color){
        switch (color){
            case RED:
                return redScalarLow;
            case BLUE:
                return blueScalarLow;
            case BLACK:
                return blackScalarLow;
            case YELLOW:
                return yellowScalarLow;
            default:
                throw new IllegalStateException("Unexpected value: " + color);
        }
    }

    private Scalar selectScalarHigher(Color color){
        switch (color){
            case RED:
                return redScalarHigh;
            case BLUE:
                return blueScalarHigh;
            case BLACK:
                return blackScalarHigh;
            case YELLOW:
                return yellowScalarHigh;
            default:
                throw new IllegalStateException("Unexpected value: " + color);
        }
    }

    private Mat detectYellow(CameraBridgeViewBase.CvCameraViewFrame inputFrame){
        colorFrame = inputFrame.rgba();
        Imgproc.cvtColor(colorFrame, processFrame, Imgproc.COLOR_RGBA2BGR);
        Imgproc.cvtColor(processFrame, processFrame, Imgproc.COLOR_BGR2HSV);
        Core.inRange(processFrame, yellowScalarLow, yellowScalarHigh, mask);
        if (grayScaleMode && color == Color.YELLOW){
            contourList.clear();
            selectedContours.clear();
            boundingRectangles.clear();
            return rotateImage(mask);
        }
        Imgproc.findContours(mask, contourList, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_NONE);
        if (!grayScaleMode && color == Color.YELLOW){
            Imgproc.drawContours(colorFrame, contourList, -1, new Scalar(255, 0, 0), 2);
            contourList.clear();
            selectedContours.clear();
            boundingRectangles.clear();
            Imgproc.circle(colorFrame, new Point(colorFrame.width()/2, colorFrame.height()/2), 3, new Scalar (0, 255, 0), 5);
            centerScalar = new Scalar(colorFrame.get(colorFrame.width()/2, colorFrame.height()/2));
            return rotateImage(colorFrame);
        }
        return null;
    }

    private Mat detectBlack(CameraBridgeViewBase.CvCameraViewFrame inputFrame){
        colorFrame = inputFrame.rgba();
        Imgproc.cvtColor(colorFrame, processFrame, Imgproc.COLOR_RGBA2BGR);
        Imgproc.cvtColor(processFrame, processFrame, Imgproc.COLOR_BGR2HSV);
        Core.inRange(processFrame, blackScalarLow, blackScalarHigh, mask);
        if (grayScaleMode && color == Color.BLACK){
            Core.inRange(processFrame, blackScalarLow, blackScalarHigh, mask);
            contourList.clear();
            selectedContours.clear();
            boundingRectangles.clear();
            return rotateImage(mask);
        }
        Imgproc.findContours(mask, contourList, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_NONE);
        if (!grayScaleMode && color == Color.YELLOW){
            Imgproc.drawContours(colorFrame, contourList, -1, new Scalar(255, 0, 0), 2);
            contourList.clear();
            selectedContours.clear();
            boundingRectangles.clear();
            Imgproc.circle(colorFrame, new Point(colorFrame.width()/2, colorFrame.height()/2), 3, new Scalar (0, 255, 0), 5);
            centerScalar = new Scalar(colorFrame.get(colorFrame.width()/2, colorFrame.height()/2));
            return rotateImage(colorFrame);
        }
        return null;
    }

    private Mat detectSkystone(CameraBridgeViewBase.CvCameraViewFrame inputFrame){
        colorFrame = inputFrame.rgba();
        Rect boundRect1;
        Imgproc.cvtColor(colorFrame, processFrame, Imgproc.COLOR_RGBA2BGR);
        Imgproc.cvtColor(processFrame, processFrame, Imgproc.COLOR_BGR2HSV);
        Core.inRange(processFrame, yellowScalarLow, yellowScalarHigh, mask);
        if (grayScaleMode && color == Color.YELLOW){
            contourList.clear();
            selectedContours.clear();
            boundingRectangles.clear();
            return rotateImage(mask);
        }
        if (grayScaleMode && color == Color.BLACK){
            Core.inRange(processFrame, blackScalarLow, blackScalarHigh, mask);
            contourList.clear();
            selectedContours.clear();
            boundingRectangles.clear();
            return rotateImage(mask);
        }
        Imgproc.findContours(mask, contourList, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_NONE);
        if (!grayScaleMode && color == Color.YELLOW){
            Imgproc.drawContours(colorFrame, contourList, -1, new Scalar(255, 0, 0), 2);
            contourList.clear();
            selectedContours.clear();
            boundingRectangles.clear();
            Imgproc.circle(colorFrame, new Point(colorFrame.width()/2, colorFrame.height()/2), 3, new Scalar (0, 255, 0), 5);
            centerScalar = new Scalar(colorFrame.get(colorFrame.width()/2, colorFrame.height()/2));
            return rotateImage(colorFrame);
        }
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
            subMat = colorFrame.submat(boundRect1);
            Imgproc.cvtColor(subMat, processFrame, Imgproc.COLOR_RGBA2BGR);
            Imgproc.cvtColor(processFrame, processFrame, Imgproc.COLOR_BGR2HSV);
            Core.inRange(processFrame, blackScalarLow, blackScalarHigh, mask);
            Imgproc.findContours(mask, contourList, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_NONE);
            subMat.copyTo(colorFrame.submat(boundRect1));
            selectedContours.add(contourList.get(0));
            for (MatOfPoint contour : contourList){
                if (Imgproc.contourArea(contour) > 5000){
                    if (Imgproc.contourArea(contour) > Imgproc.contourArea(selectedContours.get(1))){
                        selectedContours.set(1, contour);
                    }
                }
            }
            boundingRectangles.add(Imgproc.boundingRect(selectedContours.get(1)));
            Object skystoneImage = new Object(selectedContours.get(1), "Skystone Image");
            skystoneImage.drawBoundingRectangle(colorFrame);
            skystoneImage.drawCenter(colorFrame);
            if (!grayScaleMode && color == Color.BLACK){
                Imgproc.drawContours(colorFrame, contourList, -1, new Scalar(255, 0, 0), 2);
            }
            contourList.clear();
            selectedContours.clear();
            boundingRectangles.clear();
        }catch (Exception e){

        }
        Imgproc.circle(colorFrame, new Point(colorFrame.width()/2, colorFrame.height()/2), 3, new Scalar (0, 255, 0), 5);
        centerScalar = new Scalar(colorFrame.get(colorFrame.width()/2, colorFrame.height()/2));
        return rotateImage(colorFrame);
    }

    private Mat redFoundation(CameraBridgeViewBase.CvCameraViewFrame inputFrame){
        colorFrame = inputFrame.rgba();
        Imgproc.cvtColor(colorFrame, processFrame, Imgproc.COLOR_RGBA2BGR);
        Imgproc.cvtColor(processFrame, processFrame, Imgproc.COLOR_BGR2HSV);
        Core.inRange(processFrame, redScalarLow, redScalarHigh, mask);
        if (grayScaleMode && color == Color.RED){
            return mask;
        }
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
            redFoundation.drawCenter(colorFrame);
            redFoundation.drawBoundingRectangle(colorFrame);
            if (!grayScaleMode && color == Color.RED){
                Imgproc.drawContours(colorFrame, contourList, -1, new Scalar(255, 0, 0), 2);
            }
            boundingRectangles.clear();
            selectedContours.clear();
        }catch (Exception e){

        }
        Imgproc.circle(colorFrame, new Point(colorFrame.width()/2, colorFrame.height()/2), 3, new Scalar (0, 255, 0), 5);
        centerScalar = new Scalar(colorFrame.get(colorFrame.width()/2, colorFrame.height()/2));
        return rotateImage(colorFrame);
    }

    private Mat blueFoundation(CameraBridgeViewBase.CvCameraViewFrame inputFrame){
        colorFrame = inputFrame.rgba();
        Imgproc.cvtColor(colorFrame, processFrame, Imgproc.COLOR_RGBA2BGR);
        Imgproc.cvtColor(processFrame, processFrame, Imgproc.COLOR_BGR2HSV);
        Core.inRange(processFrame, blueScalarLow, blueScalarHigh, mask);
        if (grayScaleMode && color == Color.RED){
            return mask;
        }
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
            blueFoundation.drawCenter(colorFrame);
            blueFoundation.drawBoundingRectangle(colorFrame);
            if (!grayScaleMode && color == Color.BLUE){
                Imgproc.drawContours(colorFrame, contourList, -1, new Scalar(255, 0, 0), 2);
            }
            boundingRectangles.clear();
            selectedContours.clear();
        }catch (Exception e){

        }
        Imgproc.circle(colorFrame, new Point(colorFrame.width()/2, colorFrame.height()/2), 3, new Scalar (0, 255, 0), 5);
        centerScalar = new Scalar(colorFrame.get(colorFrame.width()/2, colorFrame.height()/2));
        return rotateImage(colorFrame);
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

    @Override
    public void onCameraViewStarted(int width, int height) {
        colorFrame = new Mat(height, width, CvType.CV_8U);
        mask = new Mat(height, width, CvType.CV_8U);
        processFrame = new Mat(height, width, CvType.CV_32F);
        hierarchy = new Mat();


    }

    @Override
    public void onCameraViewStopped() {
        releaseMat(colorFrame);
        releaseMat(processFrame);
        releaseMat(mask);
        releaseMat(hierarchy);
    }

    @Override
    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {
        switch (color){
            case YELLOW:
                return detectSkystone(inputFrame);
            case BLACK:
                return detectSkystone(inputFrame);
            case RED:
                return redFoundation(inputFrame);
            case BLUE:
                return blueFoundation(inputFrame);
            default: throw new IllegalStateException("Unexpected value: " + color);

        }
    }
}
