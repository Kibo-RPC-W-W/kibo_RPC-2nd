package jp.jaxa.iss.kibo.rpc.kibo_RPC_2nd;

import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcApi;

public class ToA_ {
    public static void pattern2(Point a_, Quaternion q, KiboRpcApi api){
        Point z = new Point(11.21, -9.8, a_.getZ());
        api.moveTo(z, q, true);
        api.moveTo(a_, q, true);
    }
}
