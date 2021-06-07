package jp.jaxa.iss.kibo.rpc.kibo_RPC_2nd;

import android.graphics.Bitmap;
import android.util.Log;

import com.google.zxing.BinaryBitmap;
import com.google.zxing.ChecksumException;
import com.google.zxing.DecodeHintType;
import com.google.zxing.FormatException;
import com.google.zxing.LuminanceSource;
import com.google.zxing.MultiFormatReader;
import com.google.zxing.NotFoundException;
import com.google.zxing.PlanarYUVLuminanceSource;
import com.google.zxing.RGBLuminanceSource;
import com.google.zxing.Reader;
import com.google.zxing.common.HybridBinarizer;
import com.google.zxing.qrcode.QRCodeReader;

import org.opencv.android.Utils;
import org.opencv.calib3d.Calib3d;
import org.opencv.aruco.Aruco;
import org.opencv.aruco.DetectorParameters;
import org.opencv.aruco.Dictionary;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import static org.opencv.android.Utils.bitmapToMat;
import static org.opencv.android.Utils.matToBitmap;
import static org.opencv.core.CvType.CV_32FC1;
import static org.opencv.core.CvType.CV_64F;

import java.io.ByteArrayOutputStream;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.EnumMap;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import gov.nasa.arc.astrobee.Kinematics;
import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcApi;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

import static android.graphics.Bitmap.createBitmap;
import android.graphics.Bitmap;
/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

/**
 * A(11.21, -9.8, 4.79)
 */
public class YourService extends KiboRpcService {

    static Point a_ = null;

    Quaternion q = new Quaternion(0f, 0f, -0.707f, 0.707f);

    int ap;
    double ax, ay, az;

    @Override
    protected void runPlan1() {

        // write here your plan 1
        api.startMission();
        //move to point A
        Point p1 = new Point(11.21, -10, 5.0f);
        Quaternion q1 = new Quaternion(0f, 0f, -0.707f, 0.71f);
        Log.d("START", "move to a");
        moveTo(p1, q1);

        //show point A info
        Log.d("IMU", api.getRobotKinematics().getOrientation().toString());
        Log.d("position", api.getRobotKinematics().getPosition().toString());

        //read qrcode
        Log.d("START", "read QR");
        api.flashlightControlFront(0.05f);
        getQR();
        //get point A'
        Log.d("FINISH", "getQR success");

        //to A'
        if(ap >= 2 && ap <= 6){
            pattern23456(a_, q);
        }
        else{
            pattern178(a_, q);
        }
        Log.d("IMU", api.getRobotKinematics().getOrientation().toString());
        Log.d("position", api.getRobotKinematics().getPosition().toString());
    }


    private void moveTo(Point p, Quaternion q){
        Point robotPose = null;
        double x = 0, y = 0, z = 0;

        do {
            api.moveTo(p, q, true);
            robotPose = api.getTrustedRobotKinematics().getPosition();
            x = p.getX() - robotPose.getX();
            y = p.getY() - robotPose.getY();
            z = p.getZ() - robotPose.getZ();

        } while (x + y + z > 0.3);
    }

    /**************************************************************************
     *                        To A'
     **************************************************************************/
    public void pattern23456(Point a_, Quaternion q){
        Point z = new Point(11.21, -9.8, a_.getZ());
        api.moveTo(z, q, true);
        api.moveTo(a_, q, true);
    }
    public void pattern178(Point a_, Quaternion q){
        Point z = new Point(11.21f + 0.45f, -10f, a_.getZ());
        api.moveTo(new Point(11.21f + 0.45f, -10f, 4.79f), q, true);
        api.moveTo(z, q, true);
        api.moveTo(a_, q, true);

    }
    public void endGame(){
        Point b = new Point(10.6, -8.0, 4.5);
        api.moveTo(b, q, true);
        api.reportMissionCompletion();
    }

    /**************************************************************************
     *                        read QR
     **************************************************************************/

    public static String readQR(Bitmap bitmap) {
        try {
            int width = bitmap.getWidth();
            int height = bitmap.getHeight();
            int[] pixel = new int[width * height];
            bitmap.getPixels(pixel,0,width,0,0,width,height);
            RGBLuminanceSource rgbLuminanceSource = new RGBLuminanceSource(width,height,pixel);
            BinaryBitmap binaryBitmap = new BinaryBitmap(new HybridBinarizer(rgbLuminanceSource));

            QRCodeReader qrCodeReader = new QRCodeReader();
            com.google.zxing.Result result = qrCodeReader.decode(binaryBitmap);

            if(result.getNumBits() != 0){
                Log.d("FINISH", "readQR success");
            }
            else if(result.getNumBits() == 0){
                Log.d("FAIL", "readQR failed");
            }
            else{
                Log.d("404", "readQR error");
            }
            return result.getText();
        } catch (Exception e) {
            return null;
        }
    }

    private String getQR(){
        Log.d("getQR: ","called");
        waiting();
        String getQRString = readQR(api.getBitmapNavCam());
        if (getQRString == null){
            Log.d("getQR: ","failed");
            readQR(api.getBitmapNavCam());
            if(getQRString != null) {
                sort(getQRString);
                Log.d("Finished", ap + "," + ax + "," + ay + "," + az);
            }

        }else if(getQRString != null){
            try{
                Log.d("getQR: ", getQRString);
                sort(getQRString);
                Log.d("Finished", ap + "," + ax + "," + ay + "," + az);
                api.sendDiscoveredQR(getQRString);
            }catch (Exception e){
                Log.e("getQR error: ","error");
            }
        }
        return getQRString;
    }

    private void waiting(){
        try {
            Thread.sleep(200);
        }catch (Exception e){

        }
    }

    private void sort(String qrcode) {
        String[] splt = qrcode.split("[\"{}:,pxyz]+");
        ap = Integer.parseInt(splt[1]);
        ax = Double.parseDouble(splt[2]);
        ay = Double.parseDouble(splt[3]);
        az = Double.parseDouble(splt[4]);
        a_ = new Point(ax, ay, az);
    }

}
