package jp.jaxa.iss.kibo.rpc.defaultapk;

import android.graphics.Bitmap;

import org.json.*;

import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

public class YourService extends KiboRpcService {

    @Override
    protected void runPlan1(){

        // A
        Point pointA = new Point(11.21f, -9.8f, 4.79f);
        Quaternion quaternionA = new Quaternion(0f, 0f, -0.707f, -0.707f);

        // B
        Point pointB = new Point(10.6f, -8.0f, 4.5f);
        Quaternion quaternionB = new Quaternion(0f, 0f, -0.707f, -0.707f);

        Quaternion quaternionBInverted = new Quaternion(0f, 0f, 0.707f, 0.707f);

        // Start
        api.startMission();

        // Move to first pos (Step 1)
        moveTo(pointA, quaternionA, 5, true);

        // (read QR, move to Point-A’, read AR and aim the target) (Step 2)
        Bitmap image = api.getBitmapNavCam();
        String content = readQR(image);
        api.sendDiscoveredQR(content);

        float[] infoOfAPrime = parseQRCodeContent(content);


        float tx = infoOfAPrime[1], ty = infoOfAPrime[2], tz = infoOfAPrime[3];


        float[][] keepOutZonesNumbers = {
                {}, //0
                {1,5,9,10,13,14,15,16}, // 1
                {9,10,11,12,13,14,15,16}, // 2
                {4,8,11,12,13,14,15,16},
                {3,4,7,8,11,12,15,16},
                {1,2,3,4,7,8,12,16},
                {1,2,3,4,5,6,7,8},
                {1,2,3,4,5,6,9,13},
                {1,2,5,7,9,10,13,14}
        };

        float[][] keepOutZones1Min = {
                {}, // 0
                {tx-0.3f,ty,tz-0.3f}, // 1
                {tx-0.075f,ty,tz-0.3f}, // 2
                {tx,ty,tz-0.3f},
                {tx+0.075f,ty,tz-0.3f},
                {tx-0.3f,ty,tz-0.075f},
                {tx-0.075f,ty,tz-0.075f},
                {tx,ty,tz-0.075f},
                {tx,ty,tz-0.075f},
                {tx-0.3f,ty,tz},
                {tx-0.075f,ty,tz},
                {tx,ty,tz},
                {tx+0.075f,ty,tz},
                {tx-0.3f,ty,tz+0.075f},
                {tx-0.075f,ty,tz+0.075f},
                {tx,ty,tz+0.075f},
                {tx+0.075f,ty,tz+0.075f},
        };

        float[][] keepOutZone1Max = {
                {}, // 0
                {tx-0.075f,ty+1.785f,tz-0.075f}, // 1
                {tx,ty+1.785f,tz-0.075f},
                {tx+0.075f,ty+1.785f,tz-0.075f},
                {tx+0.3f,ty+1.785f,tz-0.075f},
                {tx-0.075f,ty+1.785f,tz},
                {tx,ty+1.785f,tz},
                {tx+0.075f,ty+1.785f,tz},
                {tx+0.3f,ty+1.785f,tz},
                {tx-0.075f,ty+1.785f,tz+0.075f},
                {tx,ty+1.785f,tz+0.075f},
                {tx+0.075f,ty+1.785f,tz+0.075f},
                {tx+0.3f,ty+1.785f,tz+0.075f},
                {tx-0.075f,ty+1.785f,tz+0.3f},
                {tx,ty+1.785f,tz+0.3f},
                {tx+0.075f,ty+1.785f,tz+0.3f},
                {tx+0.3f,ty+1.785f,tz+0.3f},
        };


        // A'
        Point pointAPrime = new Point(infoOfAPrime[1], infoOfAPrime[2],infoOfAPrime[3]);
        Quaternion quaternionAPrime = new Quaternion(infoOfAPrime[4], infoOfAPrime[5], infoOfAPrime[6], infoOfAPrime[7]);

        // Move to A'
        moveTo(pointAPrime, quaternionAPrime, 5, true);

        // Turn on/off flashlight
        openFlashlight(0.25f, 1000);

        // B
        moveTo(pointB, quaternionBInverted, 5, true); // <----- ?
        moveTo(pointB, quaternionB, 5, true);

        // Finished
        tryToReportMissionCompletion(10, 50L);
    }



    @Override
    protected void runPlan2(){ }

    @Override
    protected void runPlan3(){ }

    /**
     * Moves the robot to the point and aligns in to the quaternion.
     * @param point The point to move to.
     * @param quaternion The rotation to align to.
     * @param attempts The number of attempts.
     * @param printRobotPosition Whether to print the robot's position.
     */
    private void moveTo(Point point, Quaternion quaternion, int attempts, boolean printRobotPosition){
        int iterations = 0;
        Result result = api.moveTo(point, quaternion, printRobotPosition);
        ++iterations;
        while (!result.hasSucceeded() && iterations < attempts){
            result = api.moveTo(point, quaternion, printRobotPosition);
            ++iterations;
        }
    }


    private void moveTo(double pointX, double pointY, double pointZ, float quaternionX, float quaternionY, float quaternionZ, float quaternionW, boolean printRobotPos){
        Point point = new Point(pointX, pointY, pointZ);
        Quaternion quaternion = new Quaternion(quaternionX, quaternionY, quaternionZ, quaternionW);
        api.moveTo(point, quaternion, printRobotPos);
    }


    private String readQR(Bitmap image){
        String code = null;//Noting yet for now
        return code;
    }

    /**
     * Tries to report mission completion.
     * @param attempts The number of attempts before giving up. Note that if reporting is successful,
     *                 the method "gives up" because there's no point in reporting "Mission Complete" twice.
     * @param delay The wait time between the attempts.
     */
    private void tryToReportMissionCompletion(int attempts, long delay){

        for (int a = 0; a < attempts; a++){

            // Delay
            try {
                Thread.sleep(delay);
            } catch (InterruptedException exception) {
                exception.printStackTrace();
            }

            // Report
            if (api.reportMissionCompletion()){
                return;
            }

        }

    }

    /**
     * Parses a JSON String.
     * @param content The JSON String.
     * @return        A float array containing the KOZ pattern, the position and the orientation of Point A'
     */
    private float[] parseQRCodeContent(String content){

        float[] toReturn = new float[8];
        int p = 0;
        double xAsDouble = 0, yAsDouble = 0, zAsDouble = 0;
        try {
            JSONObject jsonObject = new JSONObject(content);
                p = jsonObject.getInt("p");
                xAsDouble = jsonObject.getDouble("x");
                yAsDouble = jsonObject.getDouble("y");
                zAsDouble = jsonObject.getDouble("z");
        } catch (JSONException exception){
            exception.printStackTrace();
        }

        // bruh
        toReturn[0] = p;
        toReturn[1] = (float) xAsDouble;
        toReturn[2] = (float) yAsDouble;
        toReturn[3] = (float) zAsDouble;
        toReturn[4] = 0;
        toReturn[5] = 0;
        toReturn[6] = -0.707f;
        toReturn[7] = -0.707f;

        return toReturn;
    }

    /**
     * Tries to openFlashlight.
     * @param brightness flash light brightness between 0 - 1.
     * @param flashlightMS The wait time before trun off flashlight.
     */

    public void openFlashlight(float brightness, long flashlightMS){
        // Turn on flashlight
        api.flashlightControlFront(brightness);

        try {
            Thread.sleep(flashlightMS);
        } catch (InterruptedException exception){
            exception.printStackTrace();
        }

        // Turn off flashlight
        api.flashlightControlFront(0f);
    }

}

