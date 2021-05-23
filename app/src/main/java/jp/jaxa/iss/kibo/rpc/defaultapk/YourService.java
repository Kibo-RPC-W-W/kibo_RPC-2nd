package jp.jaxa.iss.kibo.rpc.defaultapk;

import com.google.zxing.EncodeHintType;

import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;
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
        api.moveTo(new Point(11.21, -9.8, 4.79), new Quaternion(0, 0, 0.707f, -0.707f), true);
    }

    public void pattern2(){
        Point z = new Point(11.21, -9.8, a_.getZ());
        api.moveTo(z, q, true);
        api.moveTo(a_, q, true);

    }

    @Override
    protected void runPlan2(){
        // write here your plan 2
    }

    @Override
    protected void runPlan3(){
        // write here your plan 3
    }

}

