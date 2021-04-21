package jp.jaxa.iss.kibo.rpc.defaultapk;

import android.graphics.Bitmap;

import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

public class YourService extends KiboRpcService {
    @Override
    protected void runPlan1(){
        //Start
        api.startMission();

        //Move to first pos
        moveTo(11.21, -9.8, 4.79, 0, 0, 0, 1, true);
        //(read QR, move to Point-A’, read AR and aim the target)
        takeBitmapNavCamPicture();
        //Move to pos 2

        //Stop
        api.reportMissionCompletion();
    }

    @Override
    protected void runPlan2(){
        // write here your plan 2
    }

    @Override
    protected void runPlan3(){
        // write here your plan 3
    }

    private void moveTo(double pointX, double pointY, double pointZ, float quaternionX, float quaternionY, float quaternionZ, float quaternionW, boolean printRobotPos){
        Point point = new Point(pointX, pointY, pointZ);
        Quaternion quaternion = new Quaternion(quaternionX, quaternionY, quaternionZ, quaternionW);
        api.moveTo(point, quaternion, printRobotPos);
    }

    private void takeBitmapNavCamPicture(){
        Bitmap image = api.getBitmapNavCam();
        String content = readQR(image);
        api.sendDiscoveredQR(content); // send the content of QR code for judge.
    }

    private String readQR(Bitmap image){
        String code = null;//Noting yet for now
        return code;
    }

}

