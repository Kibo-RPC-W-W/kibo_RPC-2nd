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

import org.apache.commons.lang.ObjectUtils;
import org.opencv.android.Utils;
import org.opencv.calib3d.Calib3d;
import org.opencv.aruco.Aruco;
import org.opencv.aruco.DetectorParameters;
import org.opencv.aruco.Dictionary;
import org.opencv.core.Core;
import org.opencv.core.CvType;
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

public class YourService extends KiboRpcService {

    static Point a_ = null;

    Quaternion q = new Quaternion(0f, 0f, -0.707f, 0.707f);

    int ap;
    double ax, ay, az;

    @Override
    protected void runPlan1() {

        // write here your plan 1
        api.startMission();
        Log.d("Info count/priority", Thread.activeCount() + " " + Thread.currentThread().getPriority());
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

        for(int i = 0; i < 10; ++i){
            aimLaser();
        }
    }


    private void moveTo(Point p, Quaternion q){
        Point robotPose = null;
        double x = 0, y = 0, z = 0;

        do {
            api.moveTo(p, q, false);
            robotPose = api.getTrustedRobotKinematics().getPosition();
            x = Math.abs(p.getX() - robotPose.getX());
            y = Math.abs(p.getY() - robotPose.getY());
            z = Math.abs(p.getZ() - robotPose.getZ());

        } while (x + y + z > 0.3);
    }

    /**************************************************************************
     *                        Aim Laser
     **************************************************************************/
    public Mat undistortImg(Mat src)
    {
        Mat dist_Coeff = getDist_coeff();
        Mat cam_Matrix = getCamIntrinsics();
        Mat output = new Mat(src.rows(), src.cols(), src.type());

        Imgproc.undistort(src, output, cam_Matrix, dist_Coeff);
        return output;
    }
    private Mat getCamIntrinsics(){
        //        cam_matrix arr to mat
        Mat cam_Matrix = new Mat();
        double [][] Nav_Intrinsics = api.getNavCamIntrinsics();
        for (int i = 0; i <= 8; ++i)
        {
            int row , col ;

            if(i < 3){
                row = 0;col = i;
            } else if(i<6){
                row = 1;col = i-3;
            } else{
                row = 2;col = i-6;
            }

            cam_Matrix.put(row, col, Nav_Intrinsics[0][i]);
        }
        if(!cam_Matrix.empty()) {
            Log.d("Get Cam_Matrix[status]:", "Acquired");
        }else{
            Log.d("Get Cam_Matrix[status]:", "Not Acquired");
        }
        return cam_Matrix;
    }
    private Mat getDist_coeff(){
        //         dat coefficient arr to mat
        double [][] Nav_Intrinsics = api.getNavCamIntrinsics();
        Mat dist_Coeff = new Mat();
        for(int i = 0; i<=4 ;i++)
        {
            dist_Coeff.put(0,i,Nav_Intrinsics[1][i]);
        }
        Log.d("Get Dist_coeff[status]:","Acquired");
        return dist_Coeff;

    }

    public double[] multiply_mat_vec(double[][] matrix, double[] vector) {
        int rows = matrix.length;
        int columns = matrix[0].length;

        double[] result = new double[rows];

        for (int row = 0; row < rows; row++) {
            double sum = 0;
            for (int column = 0; column < columns; column++) {
                sum += matrix[row][column]
                        * vector[column];
            }
            result[row] = sum;
        }
        return result;
    }

    public double[][] qua_to_rotation_mat(float q0, float q1, float q2, float q3)
    {
        double[][] Rotation_Mat = new double[3][3];

        Rotation_Mat[0][0] =  (2 * (Math.pow((double)q0 , 2) + Math.pow((double)q1 , 2)) - 1 );
        Rotation_Mat[0][1] = (double) 2*(q1*q2 - q0*q3);
        Rotation_Mat[0][2] = (double) 2*(q1*q3 + q0*q2);
        Rotation_Mat[1][0] = 2 * (q1* q2 + q0*q3);
        Rotation_Mat[1][1] = (2 * (Math.pow((double)q0 , 2) + Math.pow((double)q2 , 2)) - 1 );
        Rotation_Mat[1][2] = (double)2 * (q2* q3 - q0*q1);
        Rotation_Mat[2][0] = 2 * (q1* q3 - q0*q2);
        Rotation_Mat[2][1] = 2 * (q2* q3 + q0*q1);
        Rotation_Mat[2][2] = (2 * (Math.pow((double)q0 , 2) + Math.pow((double)q3 , 2)) - 1 );

        return Rotation_Mat;

    }

    public double[] get_midpoint(double[] src1, double[] src2)
    {
        double[] mid_point = new double[3];
        for(int i = 0; i<3; i++)
        {
            mid_point[i] = (src1[i] + src2[i])/2;
        }
        return mid_point;
    }

    public double get_vec_len(double[] vec)
    {
        double len = 0;
        for(int i = 0; i< 3; i++) { len += Math.pow(vec[i],2); }
        len = Math.sqrt(len);
        return len;
    }
    public void to_unit_vector(double[] vec)
    {
        double len = 0;
        for(int i = 0; i< 3; i++) { len += Math.pow(vec[i],2); }
        len = Math.sqrt(len);
        for(int i = 0; i< 3; i++){vec[i] = vec[i] * (1/len); }
    }

    public double dotProduct(double[] a, double[] b) {
        double sum = 0;
        for (int i = 0; i < a.length; i++) {
            sum += a[i] * b[i];
        }
        return sum;
    }

    public void crossProduct(double[] vec_A, double[] vec_B, double[] cross_P)
    {
        cross_P[0] = vec_A[1] * vec_B[2]
                - vec_A[2] * vec_B[1];
        cross_P[1] = vec_A[2] * vec_B[0]
                - vec_A[0] * vec_B[2];
        cross_P[2] = vec_A[0] * vec_B[1]
                - vec_A[1] * vec_B[0];
    }

    public double[] get_angle_info(double[] vec_A, double[] vec_B)
    {
        double cos_theta ;
        double[] info = new double[2];
        cos_theta = dotProduct(vec_A,vec_B)/(get_vec_len(vec_A)*get_vec_len(vec_B));
        double w = Math.sqrt((1 + cos_theta)/2);
        double s = Math.sqrt((1 - cos_theta)/2);

        info[0] = w;
        info[1] = s;
        return info;
    }

    public Quaternion Qua_multiply(Quaternion Qa, Quaternion Qb)
    {

        float w = Qa.getW() * Qb.getW() - Qa.getX() * Qb.getX() - Qa.getY() * Qb.getY() - Qa.getZ()* Qb.getZ();
        float x = Qa.getW() * Qb.getX() + Qa.getX() * Qb.getW() + Qa.getY() * Qb.getZ() - Qa.getZ()* Qb.getY();
        float y = Qa.getW() * Qb.getY() - Qa.getX() * Qb.getZ() + Qa.getY() * Qb.getW() + Qa.getZ()* Qb.getX();
        float z = Qa.getW() * Qb.getZ() + Qa.getX() * Qb.getY() - Qa.getY() * Qb.getX() + Qa.getZ()* Qb.getW();

        Quaternion Qc = new Quaternion(x, y, z, w);
        return Qc;

    }

    public void aimLaser()
    {
//        remember to put in loop
//        Mat Nav_Cam_View = undistortImg(api.getMatNavCam());
        Mat cam_Matrix = getCamIntrinsics();
        Mat dist_Coeff = getDist_coeff();
        Mat Nav_Cam_View = api.getMatNavCam();
        Mat ids = new Mat();
        List<Mat> corners = new ArrayList<>();
        Dictionary AR_Tag_dict = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
//                    get target position in view img
        Aruco.detectMarkers(Nav_Cam_View, AR_Tag_dict, corners, ids);
        //            needs if statement

        if(!corners.isEmpty()) {
            Log.d("AR[status]:", " Detected");
            Log.d("AR[status]", corners.toString());
            Log.d("AR[status]", corners.size() + " ");
            Log.d("AR[status]", ids.dump());
        }else{
            Log.d("AR[status]:", "Detected");
        }

//        aim relative to Nav_cam's point of view

//        int[] ids_sorted = new int[4];
//        List<Mat> corners_sorted = new ArrayList<Mat>();
        Mat[] corners_sorted = new Mat[4];
        //            sort corners 1234
        for(int i = 0; i < 4; ++i)
        {
            int id = (int)ids.get(i, 0)[0];
//            Log.d("Debug", "1");
//            ids_sorted[id-1] = id;
            Mat vec = corners.get(i);
//            Log.d("Debug", "2");

            corners_sorted[id - 1] = vec;
//            Log.d("Debug", "3");

        }

        Log.d("Corners_Sorted:", corners_sorted.toString());

//        pose estimation
        Quaternion cam_orientation = api.getTrustedRobotKinematics().getOrientation();
        Log.d("Current Orientation: ", cam_orientation.toString());
        float cam_qw = cam_orientation.getW();
        float cam_qx = cam_orientation.getX();
        float cam_qy = cam_orientation.getY();
        float cam_qz = cam_orientation.getZ();

        double[][] rotation_abs_to_cam = qua_to_rotation_mat(cam_qw,cam_qx,cam_qy,cam_qz);

        double[] original_dir_i = {0,-1,0};
        double[] original_dir_j = {0,0,-1};
        double[] original_dir_k = {1,0,0};


        double[] cam_dir_k = multiply_mat_vec(rotation_abs_to_cam, original_dir_k);
        double[] cam_dir_i = multiply_mat_vec(rotation_abs_to_cam, original_dir_i);
        double[] cam_dir_j = multiply_mat_vec(rotation_abs_to_cam, original_dir_j);

        to_unit_vector(cam_dir_i);
        to_unit_vector(cam_dir_j);
        to_unit_vector(cam_dir_k);

        Mat rvecs = new Mat(1,3, CvType.CV_64FC(1));
        Mat tvecs =new Mat(1,3, CvType.CV_64FC(1));
//        Mat _obj = new Mat();
        Log.d("AR[status]", "start estimate");
        Aruco.estimatePoseSingleMarkers(Arrays.asList(corners_sorted), 0.05f, cam_Matrix, dist_Coeff, rvecs, tvecs);
        Log.d("AR[status]", "end estimate");
//        maybe, yep here
        double[] p2 = new double[3];
        p2[0] = tvecs.get(1, 0)[0];
        p2[1] = tvecs.get(1, 1)[0];
        p2[2] = tvecs.get(1, 2)[0];
        double[] p4 = new double[3];
        p4[0] = tvecs.get(3, 0)[0];
        p4[1] = tvecs.get(3, 1)[0];
        p4[2] = tvecs.get(3, 2)[0];
        double[] target_vec_cam = get_midpoint(p2, p4);


        double[][] t_mat = new double[3][3];
        t_mat[0] = cam_dir_i; t_mat[1] = cam_dir_j; t_mat[2] = cam_dir_k;
        double[] target_vec_abs = multiply_mat_vec(t_mat, target_vec_cam);

//        get theta between camZ and target_vec_abs
        double[] angle_info = get_angle_info(cam_dir_k, target_vec_abs);
        double w = angle_info[0];
        double s = angle_info[1];

//        cross camZ and target_vec_abs
        double[] Vec_A = new double[3];
        crossProduct(cam_dir_k, target_vec_abs, Vec_A);
        to_unit_vector(Vec_A);
//        get quaternion from cross and theta

        double x = s * Vec_A[0];
        double y = s * Vec_A[1];
        double z = s * Vec_A[2];


        Quaternion relative_target_orientation = new Quaternion((float)x,(float)y,(float)z,(float)w);
//        Log.d("Target", target_orientation.toString());
        Quaternion target_orientation = Qua_multiply(cam_orientation,relative_target_orientation);
//        target_orientation = cam_orientation  relative_target_orientation;
        try {
            Log.d("TARGET QUATERNION[status]:", String.format("%s", target_orientation.toString()));
        }catch (Exception e){
            Log.d("TARGET QUATERNION[status]:", e.toString());
        }
//        turn
        Point goal = new Point(0,0,0);
        api.relativeMoveTo(goal,target_orientation,true);
        api.laserControl(true);
        waiting();
        api.takeSnapshot();
    }
    /**************************************************************************
     *                        To A'  To B
     **************************************************************************/
    public void pattern23456(Point a_, Quaternion q){
        Point z = new Point(11.21, -9.8, a_.getZ());
        moveTo(z, q);
        moveTo(a_, q);
    }
    public void pattern178(Point a_, Quaternion q){
        Point z = new Point(11.52f, -10f, a_.getZ());
        moveTo(new Point(11.52f, -10f, 5f), q);
        moveTo(z, q);
        moveTo(a_, q);

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
        int count = 0;
        String getQRString = null;

        do{
            getQRString = readQR(api.getBitmapNavCam());
            ++count;
        }while(getQRString == null && count < 3);

        if (getQRString == null){
            Log.d("getQR: ","failed");

        }else if(getQRString != null){
            Log.d("getQR: ", getQRString);
            sort(getQRString);
            Log.d("Finished", ap + "," + ax + "," + ay + "," + az);
            api.sendDiscoveredQR(getQRString);

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
