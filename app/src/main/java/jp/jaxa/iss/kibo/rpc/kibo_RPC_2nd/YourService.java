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
        Point p1 = new Point(11.21f, -9.8f, 4.79f);
        Quaternion q1 = new Quaternion(0f, 0f, -0.707f, 0.707f);
        api.moveTo(p1, q1, true);

    }

    public void pattern2(){
        Point z = new Point(11.21, -9.8, a_.getZ());
        api.moveTo(z, q, true);
        api.moveTo(a_, q, true);
    }
    private void moveToWrapper(double pos_x,double pos_y,double pos_z,
                               double qua_x,double qua_y,double qua_z,
                               double qua_w){
        //指定した座標に移動させる
        //Moving and orientating Astrobee according to the coordinates and quaternions inputted above
        final int LOOP_MAX = 20;
        final Point point = new Point(pos_x,pos_y,pos_z);
        final Quaternion quaternion = new Quaternion((float)qua_x,(float)qua_y,
                (float)qua_z,(float)qua_w);
        Result result;
        int loop_count = 0;
        do{
            result = api.moveTo(point,quaternion,true);
            Point position = getPositionWrapper(pos_x,pos_y,pos_z);
            double m_pos_x = position.getX();
            double m_pos_y = position.getY();
            double m_pos_z = position.getZ();
            if ((pos_y <= -9.4) && m_pos_x >= (pos_x - 0.01) && m_pos_x <= (pos_x + 0.01) && m_pos_y >= (pos_y - 0.01) && m_pos_y <= (pos_y + 0.01) && m_pos_z >= (pos_z - 0.01) && m_pos_z <= (pos_z + 0.01)) {
                break;
            }  else if (m_pos_y < (pos_y + 0.05)) {
                break;
            }else{
                loop_count++;
            }
        }while(!result.hasSucceeded() && loop_count < LOOP_MAX);
    }


    private void moveToQR(int QR_num) {
        //指定したQRコードに移動させる
        //Moving Astrobee to the QR code
        double pos_table[][] = {{10.71, -5.59, 4.52, 0, 0, 1, 0},
                {11.16, -7.98, 5.47, 0.5, 0.5, -0.5, 0.5}};
        Point point = new Point(pos_table[QR_num][0], pos_table[QR_num][1], pos_table[QR_num][2]);
        Quaternion quaternion = new Quaternion((float) pos_table[QR_num][3], (float) pos_table[QR_num][4],
                (float) pos_table[QR_num][5], (float) pos_table[QR_num][6]);
        Result result;
        int loop_count = 0;
        final int LOOP_MAX = 20;
        double m_pos_x;
        double m_pos_y;
        double m_pos_z;
        do {
            result = api.moveTo(point,quaternion,true);
            if (QR_num == 0) {
                Point position = getPositionWrapper(10.71, -5.59, 4.52);
                m_pos_x = position.getX();
                m_pos_y = position.getY();
                m_pos_z = position.getZ();
            } else {
                Point position = getPositionWrapper(11.16, -7.98, 5.47);
                m_pos_x = position.getX();
                m_pos_y = position.getY();
                m_pos_z = position.getZ();
            }
            if (QR_num == 0 && m_pos_x >= 10.7 && m_pos_x <= 10.72 && m_pos_y >= -5.6 && m_pos_y <= -5.58 && m_pos_z >= 4.51 && m_pos_z <= 4.53) {
                break;
            }else if (QR_num == 1 && m_pos_x >= 11.15 && m_pos_x <= 11.17 && m_pos_y >= -7.99 && m_pos_y <= -7.97 && m_pos_z >= 5.46 && m_pos_z <= 5.48){
                break;
            } else {
                loop_count++;
            }
        } while (!result.hasSucceeded() && loop_count < LOOP_MAX);
    }


    private void relativeMoveToWrapper(double pos_x,double pos_y,double pos_z,
                                       double qua_x,double qua_y,double qua_z,
                                       double qua_w){
        //指定した相対的な座標に移動させる
        //Move to the coordinates relative to Astrobee (and orientate)
        final int LOOP_MAX = 20;
        final Point point = new Point(pos_x,pos_y,pos_z);
        final Quaternion quaternion = new Quaternion((float)qua_x,(float)qua_y,
                (float)qua_z,(float)qua_w);
        Result result;
        int loop_count = 0;
        do{
            result = api.relativeMoveTo(point,quaternion,true);
            loop_count++;
        }while(!result.hasSucceeded() && loop_count < LOOP_MAX);
    }


    private void sleep(int millis){
        //Astrobeeを停止させる
        //Stopping Astrobee for __ milliseconds
        try {
            Thread.sleep(millis);
        }catch(InterruptedException e){
        }
    }


    private Point getPositionWrapper(double def_pos_x,double def_pos_y,double def_pos_z){
        //現在位置を取得する
        //Acquiring the real-time data for Astrobee’s orientation and coordinates
        int timeout_sec = 10;
        Kinematics kinematics;
        kinematics = api.getTrustedRobotKinematics(timeout_sec);
        Point ans_point = null;
        if(kinematics != null){
            ans_point = kinematics.getPosition();
        }
        if(ans_point == null){
            ans_point = new Point(def_pos_x,def_pos_y,def_pos_z);
        }
        return ans_point;
    }

}

