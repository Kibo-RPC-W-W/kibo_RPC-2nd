package jp.jaxa.iss.kibo.rpc.kibo_RPC_2nd;

import android.graphics.Bitmap;
import android.util.Log;

import com.google.zxing.BinaryBitmap;
import com.google.zxing.DecodeHintType;
import com.google.zxing.RGBLuminanceSource;
import com.google.zxing.Result;
import com.google.zxing.common.GlobalHistogramBinarizer;
import com.google.zxing.common.HybridBinarizer;
import com.google.zxing.qrcode.QRCodeReader;

import org.opencv.android.Utils;
import org.opencv.aruco.Aruco;
import org.opencv.aruco.Dictionary;

import org.opencv.core.CvException;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import gov.nasa.arc.astrobee.Kinematics;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;

import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

import static org.opencv.core.CvType.CV_64FC1;
import static org.opencv.imgproc.Imgproc.cvtColor;

public class YourService extends KiboRpcService {

    static Point a_ = null;
    static Point b = new Point(10.6, -8.0, 4.5);
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
        else if(ap == 7){
            pattern178(a_, q);
        }else{
            pattern18(a_, q);
        }
        Log.d("IMU", api.getRobotKinematics().getOrientation().toString());
        Log.d("position", api.getRobotKinematics().getPosition().toString());
        laser_Event();
//        api.laserControl(true);
//        api.takeSnapshot();
        Log.d("Debug__", "shot");
        api.laserControl(false);
        endGame();
    }


    private void moveTo(Point p, Quaternion q){
        moveTo(p, q, false);
    }

    private void moveTo(Point p, Quaternion q, boolean direction){
        Point robotPose, output;
        double x, y, z, error, tolerance = 0.33d;
        int time = 0;
        Log.d("start from", api.getRobotKinematics().getPosition().toString());
        do {
            if(time == 0){
                double outputX = p.getX() * 1.006;
                double outputY = p.getY() * 1.007;
                double outputZ = p.getZ() * 1.005;
                output = new Point(outputX, outputY, outputZ);
            }else{
                output = p;
            }
            Log.d("to", output.toString());

            error = 0;
            Kinematics kinematics = api.getRobotKinematics();
            api.moveTo(output, q, true);
            robotPose = kinematics.getPosition();
            x = Math.abs(p.getX() - robotPose.getX());
            y = Math.abs(p.getY() - robotPose.getY());
            z = Math.abs(p.getZ() - robotPose.getZ());
            error += x;
            error += y;
            error += z;
            if(direction){
                double w = Math.abs(kinematics.getOrientation().getW() - q.getW());
                error += w;
                tolerance = 0.35d;
            }
            ++time;
            if(time > 1){
                tolerance *= 1.1;
            }
        } while (error > tolerance && time < 2);
    }

    /**************************************************************************
     *                        Aim Laser
     **************************************************************************/
    public void laser_Event()
    {
        Mat cam_Matrix = getCamIntrinsics();
        Mat dist_Coeff = getDist_coeff();
        aim(cam_Matrix,dist_Coeff);
        api.laserControl(true);
        waiting();
        api.takeSnapshot();

    }

    public void aim(Mat cam_Matrix, Mat dist_Coeff){
        Mat Nav_Cam_View = undistortImg(api.getMatNavCam());
        Mat ids = new Mat();
        Quaternion q = api.getTrustedRobotKinematics().getOrientation();
        Log.d("Current Pose", q.toString());
        List<Mat> corners = new ArrayList<>();
        Dictionary AR_Tag_dict = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
//                    get target position in view img
        Aruco.detectMarkers(Nav_Cam_View, AR_Tag_dict, corners, ids);

        if(!corners.isEmpty()) {
            Log.d("AR[status]:", " Detected");
            Log.d("AR[status]", corners.toString());
            Log.d("AR[status]", corners.size() + " ");
            Log.d("AR[status]", ids.dump());
        }else{
            Log.d("AR[status]:", "Detected");
        }

//        aim relative to Nav_cam's point of view
        Mat[] corners_sorted = new Mat[4];
        //            sort corners 1234
        for(int i = 0; i < 4; ++i)
        {
            int id = (int)ids.get(i, 0)[0];
            Mat vec = corners.get(i);
            corners_sorted[id - 1] = vec;
        }
        Log.d("Corners_Sorted:", corners_sorted.toString());

        Mat rvecs = new Mat(4,3, CvType.CV_64FC(1));
        Mat tvecs =new Mat(4,3, CvType.CV_64FC(1));
        Log.d("AR[status]", "start estimate");
        Aruco.estimatePoseSingleMarkers(Arrays.asList(corners_sorted), 0.05f, cam_Matrix, dist_Coeff, rvecs, tvecs);
        Log.d("AR[status]", "end estimate");
        double[] p2 = tvecs.get(0, 0);
        double[] p4 = tvecs.get(2, 0);
        double[] transVec = get_midpoint(p2, p4);
        double[][] rotationMatrix = {
                {0, 0, 0},
                {0, 0, -1},
                {0, 1, 0}
        };
        transVec = vecRotation(transVec, q);
        transVec = vecRotation(transVec, rotationMatrix);
        q = QuaternionMultiply(q, eulerToQuaternion(new double[]{1, 0, 0}, getRoll(transVec)));
        Log.d("Target pose1", q.toString());
        q = QuaternionMultiply(q, eulerToQuaternion(new double[]{0, 1, 0}, getPitch(transVec)));
        Log.d("Target pose2", q.toString());
        q = QuaternionMultiply(q, eulerToQuaternion(new double[]{0, 0, 1}, getYaw(transVec)));
        Log.d("Target pose3", q.toString());
        api.relativeMoveTo(new Point(), q, true);
    }
    public Quaternion eulerToQuaternion(double[] v, double rad){
        double w = Math.cos(rad/2);
        double s = Math.sin(rad/2);
        return new Quaternion((float)(v[0] * s), (float)(v[1] * s), (float)(v[2] * s), (float)w);
    }
    //繞Z軸
    public double getYaw(double[] v){
        return Math.atan(v[1]/v[0]);
    }
    //繞X軸
    public double getRoll(double[] v){
        return Math.atan(v[1]/v[2]);
    }
    //繞Y軸
    public double getPitch(double[] v){
        return Math.atan(v[0]/v[2]);
    }
    public Quaternion quaInverse(Quaternion q){
        return new Quaternion(-q.getX(), -q.getY(), -q.getZ(), q.getW());
    }

    public double[] vecRotation(double[] vec, Quaternion q){
        return multiply_mat_vec(quaToRotationMatrix(q), vec);
    }

    public double[] vecRotation(double[] vec, double[][] m){
        return multiply_mat_vec(m, vec);
    }
    public void get_undistort_info(Mat cam_Matrix,Mat dist_Coeff, Mat map1, Mat map2,Mat src){

        Mat new_cam_Matrix = Mat.eye(3, 3, cam_Matrix.type());
        Size size = src.size();
        Log.d("Debug", "QQ");

        Imgproc.initUndistortRectifyMap(cam_Matrix, dist_Coeff,
                Mat.eye(3, 3, CvType.CV_64FC(1)), cam_Matrix, size, CvType.CV_32FC1, map1, map2);
        Log.d("Debug map1",map1.dump());
        Log.d("Debug map2",map2.dump());


    }
    public Mat undistortImg(Mat src,Mat map1,Mat map2)
    {
        Mat output = new Mat(src.rows(), src.cols(), src.type());
        Imgproc.remap(src,output,map1,map2,Imgproc.INTER_LINEAR);

        return output;
    }
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
        Log.d("Get cam_Matrix[status]", "Acquired");
        return cam_Matrix;
    }
    private Mat getDist_coeff()
    {
        //         dat coefficient arr to mat
        double [][] Nav_Intrinsics = api.getNavCamIntrinsics();
        Mat dist_Coeff = new Mat(1,4,CvType.CV_64FC(1));
        for(int i = 0; i<=4 ;i++)
        {
            dist_Coeff.put(0,i,Nav_Intrinsics[1][i]);
        }
        Log.d("Get Dist_coeff[status]:","Acquired");
        return dist_Coeff;

    }

    public double[] multiply_mat_vec(double[][] matrix, double[] vector)
    {
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
    public double[][] quaToRotationMatrix(Quaternion q){
        double x = q.getX(), y = q.getY(), z = q.getZ(), w = q.getW();
        double yy = y * y, xx = x * x, zz = z * z;
        double xy = x * y, xw = x* w, xz = x * z, zw = z * w, yw = y * w, yz = y * z;

        double[][] Rotation_Mat = new double[3][3];
        Rotation_Mat[0][0] =  1 - 2*yy - 2*zz;
        Rotation_Mat[0][1] = 2*xy - 2*zw;
        Rotation_Mat[0][2] = 2*xz + 2*yw;
        Rotation_Mat[1][0] = 2*xy + 2*zw;
        Rotation_Mat[1][1] = 1 - 2*xx - 2*zz;
        Rotation_Mat[1][2] = 2*yz - 2*xw;
        Rotation_Mat[2][0] = 2*xz - 2*yw;
        Rotation_Mat[2][1] = 2*yz + 2*xw;
        Rotation_Mat[2][2] = 1 - 2*xx - 2*yy;
        return  Rotation_Mat;
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
        Log.d("cos_theta", String.valueOf(cos_theta));
        double w = Math.sqrt((1 + cos_theta)/2);
        double s = Math.sqrt((1 - cos_theta)/2);
        Log.d("w", String.valueOf(w));
        Log.d("s", String.valueOf(s));

        info[0] = w;
        info[1] = s;
        return info;
    }

    public Quaternion QuaternionMultiply(Quaternion qa, Quaternion qb){
        float x = qa.getX(), y = qa.getY(), z = qa.getZ(), w = qa.getW();
        float xb = qb.getX(), yb = qb.getY(), zb = qb.getZ(), wb = qb.getW();

        float x_ = xb*w - xb*z + xb*y + xb*x;
        float y_ = yb*z + yb*w - yb*x + yb*y;
        float z_ = -zb*y + zb*x + zb*w + zb*z;
        float w_ = -wb*x -wb*y - wb*z + wb*w;
        return new Quaternion(x_, y_, z_, w_);
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


    /**************************************************************************
     *                        To A'  To B
     **************************************************************************/
    public void pattern23456(Point a_, Quaternion q){
        Point z = new Point(10.9, -9.8, a_.getZ());

        moveTo(new Point(10.9, -9.8, 4.79), q);
        Log.d("23456", "1");
        moveTo(z, q);
        Log.d("23456", "2");
        moveTo(a_, q);
        Log.d("23456", "3");

    }
    public void pattern178(Point a_, Quaternion q){
        Point z = new Point(11.47f, -10f, a_.getZ());

        moveTo(new Point(11.47f, -10f, 5f), q);
        Log.d("178", "1");
        moveTo(z, q);
        Log.d("178", "2");
        moveTo(a_, q);
        Log.d("178", "3");
    }
    public void pattern18(Point a_, Quaternion q){
        Point step1 = new Point(a_.getX(), a_.getY(), 5);
        moveTo(step1, q);
        moveTo(a_, q);
    }
    public void specificMoveTo(Point p, Quaternion q, boolean ax, boolean ay, boolean az){
        Point robotPose, output;
        double error, tolerance = 0.11d;
        int time = 0;
        Log.d("start from", api.getRobotKinematics().getPosition().toString());
        do {
            output = p;
            Log.d("to", output.toString());

            error = 0;
            Kinematics kinematics = api.getRobotKinematics();
            api.moveTo(output, q, true);
            robotPose = kinematics.getPosition();
            if(ax)
                error += Math.abs(p.getX() - robotPose.getX());

            if(ay)
                error += Math.abs(p.getY() - robotPose.getY());

            if(az)
                error += Math.abs(p.getZ() - robotPose.getZ());

        } while (error > tolerance && time < 2);
    }
    public void endGame(){
        if(ap <= 4 || ap == 8){
            end12348();
        }else if(ap == 7){
            end7();
        }else if(ap >= 5 && ap <= 6){
            end56();
        }
//        moveTo(b, q);
        api.reportMissionCompletion();
    }
    public void end12348(){
        Point curr = api.getTrustedRobotKinematics().getPosition();
        Point step1 = new Point(curr.getX(), curr.getY(), b.getZ());
        Point step2 = new Point(10.5, -9.5, b.getZ());
        Point step3 = new Point(10.5, -8.4, b.getZ());
        moveTo(step1, q);
        moveTo(step2, q);
        moveTo(b, q);
    }
    public void end7(){
        Point step1 = new Point(11.47f, -10f, a_.getZ());
        specificMoveTo(step1, q, true, false, false);

        Point curr = api.getTrustedRobotKinematics().getPosition();
        Point step2 = new Point(curr.getX(), curr.getY(), b.getZ() + 0.15);
        Point step3 = new Point(10.5, -9.1, b.getZ());
        moveTo(step2, q);
        moveTo(step3, q);
        moveTo(b, q);
    }
    public void end56(){
        Point curr = api.getTrustedRobotKinematics().getPosition();
        Point step1 = new Point(10.5, -9.5, curr.getZ());
        Point step2 = new Point(10.5, -8.4, b.getZ());
        moveTo(step1, q);
        moveTo(b, q);
    }

    /**************************************************************************
     *                        read QR
     **************************************************************************/

    public static String readQR(Bitmap bitmap) {
        try {
            int width = bitmap.getWidth();
            int height = bitmap.getHeight();
            int[] pixel = new int[(width / 2) * (height / 2)];
            bitmap.getPixels(pixel,0, width / 2,width / 4,height / 4, width / 2, height / 2);
            RGBLuminanceSource rgbLuminanceSource = new RGBLuminanceSource(width / 2,height / 2, pixel);
            BinaryBitmap binaryBitmap = new BinaryBitmap(new GlobalHistogramBinarizer(rgbLuminanceSource));


            Log.d("Time", "before create instance");
            QRCodeReader qrCodeReader = new QRCodeReader();
            Log.d("Time", "after create instance");
            Log.d("Time", "before decode");
            Result result = qrCodeReader.decode(binaryBitmap);
            Log.d("Time", "after decode");
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

    private void getQR(){
        Log.d("getQR: ","called");
        waiting();
        int count = 0;
        String getQRString;

        do{
            getQRString = readQR(api.getBitmapNavCam());
            ++count;
            Log.d("ReadQR", "~");
        }while(getQRString == null && count < 3);

        if (getQRString == null){
            Log.d("getQR: ","failed");

        }else if(getQRString != null){
            Log.d("getQR: ", getQRString);
            sort(getQRString);
            Log.d("Finished", ap + "," + ax + "," + ay + "," + az);
            api.sendDiscoveredQR(getQRString);
        }
    }

    private void waiting(){
        try {
            Thread.sleep(200);
        }catch (Exception ignored){

        }
    }

    private void sort(String qrcode) {
        String[] spilt = qrcode.split("[\"{}:,pxyz]+");
        ap = Integer.parseInt(spilt[1]);
        ax = Double.parseDouble(spilt[2]);
        ay = Double.parseDouble(spilt[3]);
        az = Double.parseDouble(spilt[4]);
        a_ = new Point(ax, ay, az);
    }

    private Mat cropicture(Mat crop){
        Mat image_original = crop;
        Point p1 = new Point(crop.width()/2,crop.height()/3,0);
        Point p2 = new Point((crop.width()/3)*2,(crop.height()/3),0);
        Point p3 = new Point(crop.width()/2,(crop.height()/3)*2,0);
        Point p4 = new Point((crop.width()/3)*2,(crop.height()/3)*2,0);
        Rect rectCrop = new Rect((int)(p1.getX()), (int)(p1.getY()) ,(int)((p4.getX()-p1.getX())+1), (int)((p4.getY()-p1.getY())+1));
        Mat image_output= image_original.submat(rectCrop);
        return image_output;
    }

    public Bitmap convertMat2Bitmap (Mat img) {
        int width = img.width();
        int hight = img.height();

        Bitmap bmp;
        bmp = Bitmap.createBitmap(width, hight, Bitmap.Config.ARGB_8888);
        Mat tmp;
        tmp = img.channels()==1? new Mat(width, hight, CvType.CV_8UC1, new Scalar(1)): new Mat(width, hight, CvType.CV_8UC3, new Scalar(3));
        try {
            if (img.channels()==3)
                cvtColor(img, tmp, Imgproc.COLOR_RGB2BGRA);
            else if (img.channels()==1)
                cvtColor(img, tmp, Imgproc.COLOR_GRAY2RGBA);
            Utils.matToBitmap(tmp, bmp);
        }
        catch (CvException e){
            Log.d("Expection",e.getMessage());
        }
        return bmp;
    }

}
