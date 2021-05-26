package jp.jaxa.iss.kibo.rpc.kibo_RPC_2nd;

import android.graphics.Bitmap;
import android.util.Log;

import com.google.zxing.BinaryBitmap;
import com.google.zxing.LuminanceSource;
import com.google.zxing.MultiFormatReader;
import com.google.zxing.NotFoundException;
import com.google.zxing.PlanarYUVLuminanceSource;
import com.google.zxing.common.HybridBinarizer;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;
/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

/**
 * A(11.21, -9.8, 4.79)
 */
public class YourService extends KiboRpcService {

    static int pattern;
    float test = 0;

    static Point a_ = new Point();
    Quaternion q = new Quaternion();


    @Override
    protected void runPlan1() {
        // write here your plan 1
        api.startMission();
        //move to point A
        Point p1 = new Point(11.21f, -9.8f, 4.79f);
        Quaternion q1 = new Quaternion(0f, 0f, -0.707f, 0.707f);
        api.moveTo(p1, q1, true);
        Log.println(Log.INFO, "IMU", api.getRobotKinematics().getOrientation().toString());
        Log.println(Log.INFO, "position", api.getRobotKinematics().getPosition().toString());
        try {
            QRReader(api.getBitmapNavCam());
            QRReader(api.getBitmapDockCam());
        } catch (NotFoundException e) {
            e.printStackTrace();
        }
        Log.println(Log.INFO,"Point", a_.toString());
        pattern2();
        Log.println(Log.INFO, "IMU", api.getRobotKinematics().getOrientation().toString());
        Log.println(Log.INFO, "position", api.getRobotKinematics().getPosition().toString());
    }

    public void pattern2(){
        Point z = new Point(11.21, -9.8, a_.getZ());
        api.moveTo(z, q, true);
        api.moveTo(a_, q, true);
    }
    public static void QRReader(Bitmap bitmap) throws NotFoundException {
        MultiFormatReader formatReader = new MultiFormatReader();
        //讀取指定的二維碼文件
        byte[] arr = bitmapToArray(bitmap);
        LuminanceSource source = new PlanarYUVLuminanceSource(arr, bitmap.getWidth(), bitmap.getHeight(), 0, 0, bitmap.getWidth(), bitmap.getHeight(), false);

        BinaryBitmap bBitmap = new BinaryBitmap(new HybridBinarizer(source));
        com.google.zxing.Result qrResult = formatReader.decode(bBitmap);
        //輸出相關的二維碼信息
        System.out.println(qrResult.getText());
        String format = qrResult.getText();
        String regex = "-?\\d*\\.?\\d*";
        Pattern p = Pattern.compile(regex);
        Matcher m = p.matcher(format);
        pattern  = Integer.parseInt(m.group(0));
        a_ = new Point(Float.parseFloat(m.group(1)), Float.parseFloat(m.group(2)), Float.parseFloat(m.group(3)));
    }

    public static byte[] bitmapToArray(Bitmap bmp){
        ByteArrayOutputStream stream = new ByteArrayOutputStream();
        bmp.compress(Bitmap.CompressFormat.JPEG, 50, stream);
        byte[] byteArray = stream.toByteArray();
        return byteArray;
    }

    @Override
    protected void runPlan2(){
        // write here your plan 2
    }

    @Override
    protected void runPlan3(){
        // write here your plan 3
    }

}

