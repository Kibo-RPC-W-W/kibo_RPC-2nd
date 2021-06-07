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

    static Point a_ = null;

    Quaternion q = new Quaternion();

    @Override
    protected void runPlan1() {
        // write here your plan 1
        api.startMission();
        //move to point A
        Point p1 = new Point(11.21f, -10f, 5.0f);
        Quaternion q1 = new Quaternion(0f, 0f, -0.707f, 0.71f);
        Log.d("START", "move to a");
        moveTo(p1, q1);
        //show point A info
        Log.d("IMU", api.getRobotKinematics().getOrientation().toString());
        Log.d("position", api.getRobotKinematics().getPosition().toString());
        //read qrcode
        Log.d("START", "read QR");
//        api.flashlightControlFront(0.05f);
        Bitmap bitmap = null;
        Mat m = null;
        for(int i =0; i < 20; ++i){
            m = api.getMatNavCam();
            if(m != null){
                Log.d("Find", "Find bitmap");
                bitmap = clearCode(m);
                QRReader(bitmap);
                Log.d("End", "QRReader End");
                if (a_ == null) {
                    Log.d("Fail", "Fail to find QR code");
                }else{
                    Log.d("Point A'", a_.toString());
                    break;
                }
                m = null;
                moveTo(api.getTrustedRobotKinematics().getPosition(), randomAngle());
                Log.d("START", "start new pose to get qr code");
            }
        }
        moveTo(p1, q1);
        //to A'
        ToA_.pattern2(a_, q, api);
        Log.d("IMU", api.getRobotKinematics().getOrientation().toString());
        Log.d("position", api.getRobotKinematics().getPosition().toString());
    }
    public Quaternion randomAngle(){
        float rotation = api.getTrustedRobotKinematics().getOrientation().getW();
        rotation += Math.random() * 0.1;
        return new Quaternion(api.getTrustedRobotKinematics().getOrientation().getX(), api.getTrustedRobotKinematics().getOrientation().getY(), api.getTrustedRobotKinematics().getOrientation().getZ(), rotation);
    }
    public Bitmap clearCode(Mat m){
        Mat zoom = m.submat(m.rows() / 2, m.rows(), m.cols() / 2, m.cols());//new Mat((int)(m.rows() / 1.5), (int)(m.cols() / 1.5), m.type());
        Mat zo = new Mat(zoom.rows(), zoom.cols(), zoom.type());
        Mat zl = new Mat(zoom.rows(), zoom.cols(), zoom.type());
        Mat zz = new Mat(zoom.rows(), zoom.cols(), zoom.type());
        Imgproc.medianBlur(zoom, zo, 11);
        Imgproc.medianBlur(zoom, zz, 7);
        Imgproc.medianBlur(zoom, zl, 3);
        Core.addWeighted(zoom, 2.1, zl, -1.2, 0, zoom);
        Core.addWeighted(zoom, 2.1, zo, -0.9, 0, zoom);
        Core.addWeighted(zoom, 2.1, zz, -1.1, 0, zoom);
//        Imgproc.cvtColor(zoom, zoom, Imgproc.COLOR_RGB2GRAY);
        Imgproc.equalizeHist(zoom, zoom);
        Imgproc.threshold(zoom, zoom, 90, 180, Imgproc.THRESH_BINARY);

        Mat mr = new Mat(m.rows(), m.cols(), m.type());
        System.out.println(zoom.size());
        System.out.println(mr.size());
        Imgproc.resize(zoom, mr, mr.size(), 2, 2, 2);
        Imgproc.cvtColor(mr, mr, Imgproc.COLOR_GRAY2RGBA, 4);
        Bitmap r = Bitmap.createBitmap(mr.cols(), mr.rows(), Bitmap.Config.ARGB_8888);
        Utils.matToBitmap(m, r);
        return r;
    }
    public void moveTo(Point p, Quaternion q){
        Point output = null;
        Point robotPose = null;
        double x = 0, y = 0, z = 0;
        double kP = 0.25;
        double kD = 0.001;
        double xo, yo, zo;
        double lx = 0, ly = 0, lz = 0;
        do{
            api.moveTo(p, q, true);
            robotPose = api.getTrustedRobotKinematics().getPosition();
            x = p.getX() - robotPose.getX();
            y = p.getY() - robotPose.getY();
            z = p.getZ() - robotPose.getZ();
            xo = p.getX() + x * kP + (lx - x) * kD;
            yo = p.getY() + y * kP + (ly - y) * kD;
            zo = p.getY() + z * kP + (lz - z) * kD;
            output = new Point(xo, yo, zo);
            lx = x; ly = y; lz = z;
        }while(x + y + z > 0.3);
    }

    public static int[] bitmapToArray(Bitmap bmp){
        ByteArrayOutputStream stream = new ByteArrayOutputStream();
        bmp.compress(Bitmap.CompressFormat.JPEG, 50, stream);
        byte[] byteArray = stream.toByteArray();
        int[] intArray = new int[byteArray.length];
        for(int i = 0; i < byteArray.length; ++i){
            intArray[i] = byteArray[i];
        }
        return intArray;
    }

    //不推薦使用
    public static void QRReader(Bitmap bitmap) {
        Log.d("START", "QRReader");
        QRCodeReader formatReader = new QRCodeReader();
//        Map<DecodeHintType, Object> hints = new HashMap<>();
//        hints.put(DecodeHintType.PURE_BARCODE, Boolean.TRUE);
//        formatReader.setHints(hints);

        int[] arr = bitmapToArray(bitmap);
        LuminanceSource source = new RGBLuminanceSource(bitmap.getWidth(), bitmap.getHeight(), arr);

        BinaryBitmap bBitmap = new BinaryBitmap(new HybridBinarizer(source));
        com.google.zxing.Result qrResult = null;
        try {
            qrResult = formatReader.decode(bBitmap);
        } catch (NotFoundException | ChecksumException | FormatException e) {
            e.printStackTrace();
        }
        //輸出相關的二維碼信息
        Log.d("test", qrResult.getText());
        String format = qrResult.getText();
        String regex = "-?\\d*\\.?\\d*";
        Pattern p = Pattern.compile(regex);
        Matcher m = p.matcher(format);
        if(m.groupCount() < 4){
            return;
        }
        pattern = Integer.parseInt(m.group(0));
        a_ = new Point(Float.parseFloat(m.group(1)), Float.parseFloat(m.group(2)), Float.parseFloat(m.group(3)));

    }

}

