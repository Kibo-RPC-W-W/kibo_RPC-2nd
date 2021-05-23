package jp.jaxa.iss.kibo.rpc.defaultapk;

import gov.nasa.arc.astrobee.Result;
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
    public void moveTo(double x_org, double y_org, double z_org, double x_des, double y_des, double z_des)
    {
        double dx = x_des-x_org;
        double dy = y_des-y_org;
        double dz = z_des-z_org;
        double magnitude = Math.sqrt((dx*dx)+(dy*dy)+(dz*dz));
        double x_unit = dx/magnitude;
        double y_unit = dy/magnitude;
        double z_unit = dz/magnitude;

        double matrix[][] =
                {
                        {1, 0, 0},
                        {x_unit, y_unit, z_unit}
                };

        double x = matrix[0][1]*matrix[1][2] - matrix[1][1]*matrix[0][2];
        double y = matrix[0][2]*matrix[1][0] - matrix[1][2]*matrix[0][0];
        double z = matrix[0][0]*matrix[1][1] - matrix[1][0]*matrix[0][1];
        double i = matrix[1][0]-matrix[0][0];
        double j = matrix[1][1]-matrix[0][1];
        double k = matrix[1][2]-matrix[0][2];
        double q = Math.sqrt(x*x + y*y + z*z);
        double p = Math.sqrt(i*i + j*j + k*k);
        double theta = Math.acos((2 - p*p) / 2);

        double a = Math.sin(theta/2)*x/q;
        double b = Math.sin(theta/2)*y/q;
        double c = Math.sin(theta/2)*z/q;
        double w = Math.cos(theta/2);

        double pitch = -Math.atan((2 * (a*w + b*c)) / (w*w - a*a - b*b + c*c));
        double roll = -Math.asin(2 * (a*c - b*w));
        double yaw = Math.atan((2 * (c*w + a*b)) / (w*w + a*a - b*b - c*c));
        double sx = (0.103 * Math.cos(roll + 0.279) / Math.cos(1.57080 + yaw));
        double sy = (0.103 * Math.sin(roll + 0.279) / Math.cos(pitch));

        moveToPoint((float)x_org - (float)sx, (float)y_org, (float)z_org + (float)sy, (float)a, (float)b, (float)c, (float)w);
    }

    public void moveToA_prime(int pattern)
    {


    }



    public void aimLaser()
    {

    }
}

