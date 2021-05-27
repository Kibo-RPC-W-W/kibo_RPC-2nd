package jp.jaxa.iss.kibo.rpc.defaultapk;

import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

import static org.opencv.core.CvType.CV_64F;
import static org.opencv.core.CvType.CV_8UC1;

/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

public class YourService extends KiboRpcService {
    @Override

    protected void runPlan1()
    {
        // write here your plan 1
        int pattern = getPattern();
        api.startMission();

        moveToA_prime(pattern);






    }


    @Override
    protected void runPlan2(){
        // write here your plan 2
    }

    @Override
    protected void runPlan3(){
        // write here your plan 3
    }

    public int getPattern(){
        return 0;
    }
    private void moveToPoint(double pos_x, double pos_y, double pos_z,
                             double orient_x, double orient_y, double orient_z,
                             double orient_w)
    {
        final int LOOP_MAX = 3;
        final Point point = new Point(pos_x, pos_y, pos_z);
        final Quaternion orientation = new Quaternion((float)orient_x, (float)orient_y,
                (float)orient_z, (float)orient_w);

        Result result = api.moveTo(point, orientation, true);

        int loopCounter = 0;
        while(!result.hasSucceeded() || loopCounter < LOOP_MAX){
            result = api.moveTo(point, orientation, true);
            ++loopCounter;
        }
    }


    public void moveToA_prime(int pattern)
    {


    }


    public Mat undistortImg(Mat src)
    {
        double [][] Nav_Intrinsics = api.getNavCamIntrinsics();
        Mat cam_Matrix = new Mat();
        Mat dist_Coeff = new Mat();
        Mat output = new Mat();

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
        return output;
    }

    public void aimLaser()
    {

    }
}

