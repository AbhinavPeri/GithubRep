package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.view.SurfaceView;
import android.view.WindowManager;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.opencv.android.CameraBridgeViewBase;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

public abstract class OpenCVCamera implements CameraBridgeViewBase.CvCameraViewListener2 {

    private boolean displayScreen = true;
    private OpenCVCamera t = this;
    protected CameraBridgeViewBase openCVCamera;
    protected Activity activity;
    static {
        System.loadLibrary("opencv_java4");
    }

    public OpenCVCamera(OpMode opMode){
        activity = (Activity) opMode.hardwareMap.appContext;
        activity.runOnUiThread(() -> {
            activity.getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
            openCVCamera = activity.findViewById(R.id.activity_java_surface_view);
            openCVCamera.setCvCameraViewListener(t);
            openCVCamera.setVisibility(SurfaceView.VISIBLE);
            openCVCamera.setCameraPermissionGranted();
        });
    }
    public void startCamera(){
        activity.runOnUiThread(() -> {
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
        if (bool) activity.runOnUiThread(() -> {openCVCamera.setVisibility(SurfaceView.VISIBLE);});
        else activity.runOnUiThread(() -> {openCVCamera.setVisibility(SurfaceView.GONE);});

    }

    public Mat rotateImage(Mat mat){
        Mat releaseMat = new Mat();
        Core.rotate(mat, releaseMat, Core.ROTATE_90_CLOCKWISE);
        Imgproc.resize(releaseMat, releaseMat, mat.size());
        return releaseMat;
    }

    public void releaseMat(Mat mat){
        if (mat != null){
            mat.release();
        }
    }
}
