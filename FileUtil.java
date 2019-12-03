package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.Properties;

/**
 * Utility functions for log files.
 */
public class FileUtil {
    private static final File TUNING_FOLDER =
            new File(AppUtil.ROOT_FOLDER + "/OpenCV_Tuning/");

    public static double[] readIntoDoubleArray(String propertyName, Properties props){
        String input;
        try {
            input = (String) props.get(propertyName);
        } catch (Exception e){
            return null;
        }
        String[] doubleStrings = input.split(" ");

        double[] doubles = new double[doubleStrings.length];

        for (int i = 0; i < doubles.length; i++){
            doubles[i] = Double.parseDouble(doubleStrings[i]);

        }
        return doubles;
    }

    public static void loadPropertyFile(String name, Properties props){
        FileInputStream is;
        File file = new File(TUNING_FOLDER + "/" + name);
        Log.i("File info", file.getAbsolutePath());
        try{
            is = new FileInputStream(file);
            props.load(is);
            Log.i("File Loading Progress", "File was loaded");
        }catch (Exception e){
            Log.e("File Loading Progress", "File wasn't loaded");
            Log.e("File Loading Progress", e.toString());
        }
    }


    public static void createNewPropertyFile(String name, Properties props){
        if (!TUNING_FOLDER.exists()){
            TUNING_FOLDER.mkdir();
        }
        File file = new File(TUNING_FOLDER, name);
        try {
            FileOutputStream fileOutputStream = new FileOutputStream(file);
            props.store(fileOutputStream, null);
            fileOutputStream.close();
            Log.i("Tuning Folder location", TUNING_FOLDER.getAbsolutePath());
        } catch (IOException e){
            Log.e("Property file failed", e.getMessage());
        }


    }

    public static void deletePropertyFile(String name){
        File file = new File(TUNING_FOLDER + "/" + name);
        if (file.exists()){
            file.delete();
        }
    }


}
