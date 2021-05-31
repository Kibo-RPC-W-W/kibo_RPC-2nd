package jp.jaxa.iss.kibo.rpc.defaultapk;
import android.util.Log;
import org.opencv.aruco.Aruco;
import org.opencv.aruco.Dictionary;
import org.opencv.core.Mat;

import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;



/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

public class YourService extends KiboRpcService {

    @Override
    protected void runPlan1()
    {
        // write here your plan 1
        api.startMission();



    }


    @Override
    protected void runPlan2(){
        // write here your plan 2
    }

    @Override
    protected void runPlan3(){
        // write here your plan 3
    }


    public Mat[] undistortImg(Mat src)
    {
        double [][] Nav_Intrinsics = api.getNavCamIntrinsics();
        Mat cam_Matrix = new Mat();
        Mat dist_Coeff = new Mat();
        Mat output = new Mat();
        Mat[] output_arr = new Mat[3];

//        cam_matrix & dat coefficient arr to mat
        for (int i = 0; i <= 8; i++)
        {
            int row , col ;
            if(i < 3){row = 0;col = i;}
            else if(i<6){row = 1;col = i-3;}
            else{ row = 2;col = i-6;}
            cam_Matrix.put(row, col, Nav_Intrinsics[0][i]);
        }

        for(int i = 0; i<=4 ;i++)
        {
            dist_Coeff.put(0,i,Nav_Intrinsics[1][i]);
        }

        Imgproc.undistort(src, output, cam_Matrix, dist_Coeff);

        output_arr[0] = output;
        output_arr[1]  = cam_Matrix;
        output_arr[2] = dist_Coeff;
        return output_arr;
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

    public void aimLaser()
    {
//        remember to put in loop
        Mat Nav_Cam_View = undistortImg(api.getMatNavCam())[0];
        Mat cam_Matrix = undistortImg(api.getMatNavCam())[1];
        Mat dist_Coeff = undistortImg(api.getMatNavCam())[2];
        Mat ids = new Mat();
        List<Mat> corners = new ArrayList<>();
        Dictionary AR_Tag_dict = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
//                    get target position in view img
        try
        {
            Aruco.detectMarkers(Nav_Cam_View, AR_Tag_dict, corners, ids);
            //            needs if statement
            Log.d("AR[status]:", " Detected");
        }
        catch (Exception e)
        {
            Log.d("AR[status]:", " Undetected");
        }
//        aim relative to Nav_cam's point of view

//        int[] ids_sorted = new int[4];
        List<Mat> corners_sorted = new ArrayList<>();

        //            sort corners 1234
        for(int i = 0; i<4; i++)
        {
            int id = (int)ids.get(0,i)[0];
//            ids_sorted[id-1] = id;
            Mat vec = corners.get(i);
            corners_sorted.add(id-1,vec);
        }

//        pose estimation
        Quaternion cam_orientation = api.getImu().getOrientation();
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
        Aruco.estimatePoseSingleMarkers(corners_sorted,(float)0.05,cam_Matrix,dist_Coeff, rvecs,tvecs );

//        maybe
        double[] p2 =  tvecs.get(0,1);
        double[] p4 =  tvecs.get(0,3);
        double[] target_vec_cam = get_midpoint(p2,p4);


        double[][] t_mat = new double[3][3];
        t_mat[0] = cam_dir_i; t_mat[1] = cam_dir_j; t_mat[2] = cam_dir_k;
        double[] target_vec_abs = multiply_mat_vec(t_mat, target_vec_cam);

//        get theta between camZ and target_vec_abs
        double[] angle_info = get_angle_info(cam_dir_k,target_vec_abs);
        double w = angle_info[0];
        double s = angle_info[1];

//        cross camZ and target_vec_abs
        double[] Vec_A = new double[3];
        crossProduct(cam_dir_k,target_vec_abs,Vec_A);
        to_unit_vector(Vec_A);
//        get quaternion from cross and theta

        double x = s*Vec_A[0];
        double y = s*Vec_A[1];
        double z = s*Vec_A[2];

        Quaternion target_orientation = new Quaternion((float)x,(float)y,(float)z,(float)w);
//        turn
        Point goal = new Point(0,0,0);
        api.relativeMoveTo(goal,target_orientation,true);
        api.laserControl(true);
    }

//    public boolean evaluate_accuracy()
//    {
//        boolean retry = false;
//
//        return retry;
//    }
//
}

