package jp.jaxa.iss.kibo.rpc.kibo_RPC_2nd;

import com.google.zxing.BinaryBitmap;
import com.google.zxing.DecodeHintType;
import com.google.zxing.LuminanceSource;
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

import static java.lang.Thread.sleep;
import static org.opencv.android.Utils.bitmapToMat;
import static org.opencv.android.Utils.matToBitmap;
import static org.opencv.core.CvType.CV_32FC1;
import static org.opencv.core.CvType.CV_64F;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.EnumMap;
import java.util.List;
import java.util.Map;

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


    Point a_ = new Point();
    Quaternion q = new Quaternion();

    @Override
    protected void runPlan1() {
        // write here your plan 1
        api.startMission();
        //move to point A
        moveTo(11.21,-9.8,4.79,0,0,-0.707,0.707);
        //read qr and get info

    }

    public void pattern2() {
        Point z = new Point(11.21, -9.8, a_.getZ());
        api.moveTo(z, q, true);
        api.moveTo(a_, q, true);
    }

    private void moveTo(double px, double py, double pz,
                        double qx, double qy, double qz,
                        double qw) {
        //指定した座標に移動させる
        //Moving and orientating Astrobee according to the coordinates and quaternions inputted above
        final int LOOP_MAX = 20;
        final Point point = new Point(px, py, pz);
        final Quaternion quaternion = new Quaternion((float) qx, (float) qy,
                (float) qz, (float) qw);
        Result result;
        int loop_count = 0;
        do {
            result = api.moveTo(point, quaternion, true);
            Point position = getPositionWrapper(px, py, pz);
            double mpx = position.getX();
            double mpy = position.getY();
            double mpz = position.getZ();
            if ((py <= -9.4) && mpx >= (px - 0.01) && mpx <= (px + 0.01) &&
                    mpy >= (py - 0.01) && mpy <= (py + 0.01) && mpz >= (pz - 0.01) && mpz <= (pz + 0.01)) {
                break;
            } else if (mpy < (py + 0.05)) {
                break;
            } else {
                loop_count++;
            }
        } while (!result.hasSucceeded() && loop_count < LOOP_MAX);
    }

    private Point getPositionWrapper(double dpx, double dpy, double dpz) {
        //acquire real time data for orientation and coordinates
        int timeout_sec = 10;
        //help me here cuz the file says put int inside getTrustedRobotKinematics is ok, but the real situation is forbidden
        Kinematics kinematics = api.getTrustedRobotKinematics(timeout_sec);
        Point ans_point= null;

        if(kinematics != null){
            ans_point = kinematics.getPosition();
        }
        if(ans_point == null){
            ans_point = new Point(dpx,dpy,dpz);
        }

        return ans_point;
    }

    private String readQR(Bitmap bitmap){
        //scan and read qrcode
        String result = "eeeeeeerrrrrrrrrrrooooooorrrrrrr";
        int n = (int) 0.5;
        int m = (int) 0.5;

        int width = bitmap.getWidth();
        int height = bitmap.getHeight();
        int newWidth = width * n;
        int newHeight = height * m;
        int startX = (width - newWidth) / 2;
        int startY = (height - newHeight) / 2;

        Bitmap bitmapTriming = createBitmap(bitmap, startX, startY, newWidth, newHeight);
        Bitmap bitmapResize = Bitmap.createScaledBitmap(bitmapTriming, (int) (newWidth * m), (int) (newHeight * m), true);
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
        } catch (Exception e) {
        }

        return result;
    }

    private void getQRcode(){
        //QRコードへの移動と読み取り
        //Initiating the detection and scanning of QR code
        int loop_count = 0;
        String QRcodeString = "error";
        while(QRcodeString.equals("error") && loop_count < 20){
            Bitmap bitmap;

                api.flashlightControlFront(0.025f);
            try {
                sleep(2000);
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

    private void getAprime(){

    }

    private double getquaw(double qx, double qy, double qz){
        double qw;
        qw = Math.pow(1-qx*qx-qy*qy-qz*qz,0.5);
        return qw;
    }


}