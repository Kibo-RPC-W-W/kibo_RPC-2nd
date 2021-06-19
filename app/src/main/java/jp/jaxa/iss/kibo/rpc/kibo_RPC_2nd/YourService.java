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
import static org.opencv.core.CvType.CV_64FC1;

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
        aimLaser();
        for(int i = 0; i < 10; ++i){
            api.takeSnapshot();
            Log.d("Debug__", "shot");
        }
        endGame();
    }


    private void moveTo(Point p, Quaternion q){
        moveTo(p, q, false);
    }
    private void moveTo(Point p, Quaternion q, boolean direction){
        Point robotPose = null;
        double x = 0, y = 0, z = 0, error = 0, tolerence = 0.3d;
        do {
            error = 0;
            Kinematics kinematics = api.getRobotKinematics();
            api.moveTo(p, q, false);
            robotPose = kinematics.getPosition();
            x = Math.abs(p.getX() - robotPose.getX());
            y = Math.abs(p.getY() - robotPose.getY());
            z = Math.abs(p.getZ() - robotPose.getZ());
            if(direction){
                double w = Math.abs(kinematics.getOrientation().getW() - q.getW());
                error += w;
                tolerence = 0.33d;
            }
        } while (error > tolerence);
    }

    /**************************************************************************
     *                        Aim Laser
     **************************************************************************/
    public Mat undistortImg(Mat src)
    {
        Mat dist_Coeff = getDist_coeff();
        Mat cam_Matrix = getCamIntrinsics();
        Mat output = new Mat(src.rows(), src.cols(), src.type());
        if(src == null){
            Log.d("Debug_undistort", "src == null");
        }
        Imgproc.undistort(src, output, cam_Matrix, dist_Coeff);
        Log.d("Debug_undistort", "pass");
        return output;
    }
    private Mat getCamIntrinsics(){
        //        cam_matrix arr to mat
        Mat cam_Matrix = new Mat(3, 3, CV_64FC1);
        double [] nav_intrinsics = api.getNavCamIntrinsics()[0];
        for(int i = 0; i < 3; ++i){
            for(int j = 0; j < 3; ++j){
                cam_Matrix.put(i, j, nav_intrinsics[i * 3 + j]);
            }
        }
//        for (int i = 0; i <= 8; ++i)
//        {
//            int row, col ;
//
//            if(i < 3){
//                row = 0; col = i;
//            } else if(i<6){
//                row = 1; col = i-3;
//            } else{
//                row = 2; col = i-6;
//            }
//
//            cam_Matrix.put(row, col, Nav_Intrinsics[0][i]);
//        }
//        Log.d("Get Cam_Matrix[status]", Arrays.toString(nav_intrinsics));
//        Log.d("Get Cam_Matrix[status]", cam_Matrix.dump());
//        Log.d("Get Cam_Matrix[status]:","Acquired");
        return cam_Matrix;
    }
    private Mat getDist_coeff(){
        //         dat coefficient arr to mat
        double [] nav_intrinsics = api.getNavCamIntrinsics()[1];
        Mat dist_Coeff = new Mat(1, 4, CV_64FC1);
        dist_Coeff.put(0, 0, nav_intrinsics[0]);
        dist_Coeff.put(0, 1, nav_intrinsics[1]);
        dist_Coeff.put(0, 2, nav_intrinsics[2]);
        dist_Coeff.put(0, 3, nav_intrinsics[3]);
//        Log.d("Get Dist_coeff[status]", Arrays.toString(nav_intrinsics));
//        Log.d("Get Dist_coeff[status]", dist_Coeff.dump());
//        Log.d("Get Dist_coeff[status]:","Acquired");
        return dist_Coeff;

    }

    public double[] multiply_mat_vec(double[][] matrix, double[] vector) {
        int rows = matrix.length;
        int columns = matrix[0].length;
        if(columns != vector.length){
            Log.d("Debug_multiply_mat_vec", "couldn't match");
        }
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
        for(int i = 0; i < 3; i++) { len += Math.pow(vec[i],2); }
        len = Math.sqrt(len);
        for(int i = 0; i < 3; i++){vec[i] = vec[i] * (1.0/len); }
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

    public void aimLaser()
    {
//        remember to put in loop
        Mat Nav_Cam_View = undistortImg(api.getMatNavCam());
        Mat cam_Matrix = getCamIntrinsics();
        Mat dist_Coeff = getDist_coeff();
//        Mat Nav_Cam_View = api.getMatNavCam();
        Mat ids = new Mat();
        List<Mat> corners = new ArrayList<Mat>();
        Dictionary AR_Tag_dict = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
//                    get target position in view img
        Aruco.detectMarkers(Nav_Cam_View, AR_Tag_dict, corners, ids);
        //            needs if statement

        if(corners != null && !corners.isEmpty()) {
            Log.d("AR[status]:", " Detected");
//            Log.d("AR[status]", corners.toString());
//            Log.d("AR[status]", corners.size() + " ");
//            Log.d("AR[status]", ids.dump());
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

//        Log.d("Corners_Sorted:", corners_sorted.toString());

//        pose estimation
        Quaternion cam_orientation = api.getTrustedRobotKinematics().getOrientation();
//        Log.d("Current Orientation: ", cam_orientation.toString());
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

        Mat rvecs = new Mat();
        Mat tvecs =new Mat();
        Mat _obj = new Mat();
//        Log.d("AR[status]", "start estimate");
        Aruco.estimatePoseSingleMarkers(Arrays.asList(corners_sorted), 0.05f, cam_Matrix, dist_Coeff, rvecs, tvecs, _obj);
//        Log.d("AR[status]", "end estimate");
//        maybe, yep here
//        Log.d("AR[status] t", tvecs.dump());
//        Log.d("AR[status] r", rvecs.dump());
//        Log.d("Debug_", String.valueOf(tvecs.get(0, 0)[0]));
//        Log.d("Debug_", String.valueOf(tvecs.get(2, 0)[0]));
        double[] p2 = tvecs.get(0, 0);
        double[] p4 = tvecs.get(2, 0);
//        Log.d("Debug", "1");
        double[] target_vec_cam = get_midpoint(p2, p4);
//        Log.d("Debug", "2");

        double[][] t_mat = new double[3][3];
        t_mat[0] = cam_dir_i; t_mat[1] = cam_dir_j; t_mat[2] = cam_dir_k;
//        Log.d("Debug", "2.5");
        double[] target_vec_abs = multiply_mat_vec(t_mat, target_vec_cam);
//        Log.d("Debug", "3");

//        get theta between camZ and target_vec_abs
        double[] angle_info = get_angle_info(cam_dir_k, target_vec_abs);
//        Log.d("Debug", "3.5");
        double w = angle_info[0];
        double s = angle_info[1];
//        Log.d("Debug", "4");

//        cross camZ and target_vec_abs
        double[] Vec_A = new double[3];
        crossProduct(cam_dir_k, target_vec_abs, Vec_A);
//        Log.d("Debug", "4.5");
        to_unit_vector(Vec_A);
//        Log.d("Debug", "5");
//        get quaternion from cross and theta

        double x = s * Vec_A[0];
        double y = s * Vec_A[1];
        double z = s * Vec_A[2];

        Quaternion target_orientation = new Quaternion((float)x,(float)y,(float)z,(float)w);
        Log.d("Target", target_orientation.toString());
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
    }
    /**************************************************************************
     *                        To A'  To B
     **************************************************************************/
    public void pattern23456(Point a_, Quaternion q){
        Point z = new Point(11.21, -9.8, a_.getZ());
        moveTo(new Point(11.21, -9.8, 4.79), q);
        Log.d("23456", "1");
        moveTo(z, q);
        Log.d("23456", "2");
        moveTo(a_, q);
        Log.d("23456", "3");

    }
    public void pattern178(Point a_, Quaternion q){
        Point z = new Point(11.52f, -10f, a_.getZ());
        moveTo(new Point(11.52f, -10f, 5f), q);
        Log.d("178", "1");
        moveTo(z, q);
        Log.d("178", "2");
        moveTo(a_, q);
        Log.d("178", "3");
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
