package jp.jaxa.iss.kibo.rpc.kibo_RPC_2nd;

import android.graphics.Bitmap;
import android.util.Log;

import com.google.zxing.BinaryBitmap;
import com.google.zxing.DecodeHintType;
import com.google.zxing.LuminanceSource;
import com.google.zxing.MultiFormatReader;
import com.google.zxing.NotFoundException;
import com.google.zxing.PlanarYUVLuminanceSource;
import com.google.zxing.RGBLuminanceSource;
import com.google.zxing.Reader;
import com.google.zxing.common.HybridBinarizer;
import com.google.zxing.qrcode.QRCodeReader;

import org.opencv.calib3d.Calib3d;
import org.opencv.aruco.Aruco;
import org.opencv.aruco.DetectorParameters;
import org.opencv.aruco.Dictionary;
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
import java.util.List;
import java.util.Map;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import gov.nasa.arc.astrobee.Kinematics;
import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
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
    API ourAPI = new API(api);
    static int pattern;
    float test = 0;

    static Point a_ = new Point();

    Quaternion q = new Quaternion();

    @Override
    protected void runPlan1() {
        // write here your plan 1
        api.startMission();
        //move to point A
        Point p1 = new Point(12.0f, -10.5f, 5.5f);
        Quaternion q1 = new Quaternion(0f, 0f, -0.707f, 0.707f);
        ourAPI.moveTo(p1, q1);
        //show point A info
        Log.d("IMU", api.getRobotKinematics().getOrientation().toString());
        Log.d("position", api.getRobotKinematics().getPosition().toString());
        //read qrcode
        getQRcode();

        Log.d("Point A'", a_.toString());
        //to A'
        pattern2();
        Log.d("IMU", api.getRobotKinematics().getOrientation().toString());
        Log.d("position", api.getRobotKinematics().getPosition().toString());
    }

    private String readQR(Bitmap bitmap){
        //scan and read qrcode
        String result = "eeeeeeerrrrrrrrrrrooooooorrrrrrr";
        int s = (int) 0.5;
        int q = (int) 0.5;

        int width = bitmap.getWidth();
        int height = bitmap.getHeight();
        int newWidth = width * s;
        int newHeight = height * q;
        int startX = (width - newWidth) / 2;
        int startY = (height - newHeight) / 2;

        Bitmap bitmapTriming = createBitmap(bitmap, startX, startY, newWidth, newHeight);
        Bitmap bitmapResize = Bitmap.createScaledBitmap(bitmapTriming, (int) (newWidth * s), (int) (newHeight * q), true);
        width = bitmapResize.getWidth();
        height = bitmapResize.getHeight();
        int[] pixels = new int[width * height];
        bitmapResize.getPixels(pixels, 0, width, 0, 0, width, height);

        try {
            LuminanceSource source = new RGBLuminanceSource(width, height, pixels);
            BinaryBitmap binaryBitmap = new BinaryBitmap(new HybridBinarizer(source));
            Reader reader = new QRCodeReader();
            Map<DecodeHintType, Object> tmpHintsMap;
            tmpHintsMap = new EnumMap<DecodeHintType, Object>(DecodeHintType.class);
            tmpHintsMap.put(DecodeHintType.TRY_HARDER, Boolean.TRUE);
            com.google.zxing.Result decodeResult = reader.decode(binaryBitmap, tmpHintsMap);
            result = decodeResult.getText();
            System.out.println(result);
            String regex = "-?\\d*\\.?\\d*";
            Pattern p = Pattern.compile(regex);
            Matcher m = p.matcher(result);
            pattern  = Integer.parseInt(m.group(0));
            a_ = new Point(Float.parseFloat(m.group(1)), Float.parseFloat(m.group(2)), Float.parseFloat(m.group(3)));
        } catch (Exception e) {

        }

        return result;
    }

    private void getQRcode(){
        //Initiating the detection and scanning of QR code
        int loop_count = 0;
        String QRcodeString = "error";
        while(QRcodeString.equals("error") && loop_count < 20){
            Bitmap bitmap;

            api.flashlightControlFront(0.025f);
            try {
                ourAPI.sleep(2000);
            } catch (Exception e) {
            }
            bitmap = api.getBitmapNavCam();
            api.flashlightControlFront(0);

            QRcodeString = readQR(bitmap);
            loop_count ++;
        }
        api.sendDiscoveredQR(QRcodeString);
        String[] strings = QRcodeString.split(",");
    }

    public void pattern2(){
        Point z = new Point(11.21, -9.8, a_.getZ());
        api.moveTo(z, q, true);
        api.moveTo(a_, q, true);
    }
    public static byte[] bitmapToArray(Bitmap bmp){
        ByteArrayOutputStream stream = new ByteArrayOutputStream();
        bmp.compress(Bitmap.CompressFormat.JPEG, 50, stream);
        byte[] byteArray = stream.toByteArray();
        return byteArray;
    }

    //不推間使用
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
        pattern = Integer.parseInt(m.group(0));
        a_ = new Point(Float.parseFloat(m.group(1)), Float.parseFloat(m.group(2)), Float.parseFloat(m.group(3)));
    }

}

