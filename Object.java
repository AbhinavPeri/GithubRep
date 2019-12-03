package org.firstinspires.ftc.teamcode;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class Object {

    private Point objectLocation;
    private Rect boundingRectangle;
    private double objectWidth;
    private double objectHeight;
    private double objectSize;
    private String objectName;


    public Object(MatOfPoint boundingContour, String name){
        objectName = name;
        boundingRectangle = Imgproc.boundingRect(boundingContour);
        objectLocation = new Point(boundingRectangle.x + boundingRectangle.width/2, boundingRectangle.y + boundingRectangle.height/2);
        objectWidth = boundingRectangle.width;
        objectHeight = boundingRectangle.height;
        objectSize = Imgproc.contourArea(boundingContour);
    }

    public void setObjectName(String name){
        objectName = name;
    }

    public String getObjectName(){
        return objectName;
    }

    public void drawBoundingRectangle(Mat image){
        Imgproc.rectangle(image, boundingRectangle, new Scalar(255, 0, 0), 10);
    }

    public void drawBoundingRectangle(Mat image, Scalar color){
        Imgproc.rectangle(image, boundingRectangle, color, 10);
    }

    public void drawCenter(Mat image){
        Imgproc.circle(image, objectLocation, 10, new Scalar(255, 0, 0), 10);
    }

    public void drawCenter(Mat image, Scalar color){
        Imgproc.circle(image, objectLocation, 10, color, 10);
    }

    public Point getObjectLocation(){
        return objectLocation;
    }

    public double getObjectWidth(){
        return objectWidth;
    }

    public double getObjectHeight(){
        return objectHeight;
    }

    public double getObjectSize(){ return objectSize;}
}
