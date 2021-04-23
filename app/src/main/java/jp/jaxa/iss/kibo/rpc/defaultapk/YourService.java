package jp.jaxa.iss.kibo.rpc.defaultapk;

import android.graphics.Bitmap;
import android.util.Log;

import com.google.zxing.BinaryBitmap;
import com.google.zxing.RGBLuminanceSource;
import com.google.zxing.common.HybridBinarizer;
import com.google.zxing.qrcode.QRCodeReader;
import com.google.zxing.qrcode.encoder.QRCode;

import org.json.*;

import java.time.LocalDateTime;

import gov.nasa.arc.astrobee.Result;

import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

public class YourService extends KiboRpcService {

    private final long MILLISECONDS_IN_A_SECOND = 1000;
    long timeStarted;

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

        Log.d("", "Starting Mission...");
        timeStarted = System.currentTimeMillis();
        api.startMission();
        Log.d(getElapsedTime(), "Started Mission!");



        // Move to first pos (Step 1)
        moveTo(pointA, quaternionA, 5, true);


        // Turn on/off flashlight
        openFlashlight(0.25f, 1000);

        // (read QR, move to Point-A’, read AR and aim the target) (Step 2)
        Bitmap image = api.getBitmapNavCam();
        String content = readQRCode(image, 5);
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


        int keepOutAreaNumber = (int) infoOfAPrime[0];

        for (int i = 0; i < 8; i++){

        }


        // A'
        Point pointAPrime = new Point(infoOfAPrime[1], infoOfAPrime[2],infoOfAPrime[3]);
        Quaternion quaternionAPrime = new Quaternion(infoOfAPrime[4], infoOfAPrime[5], infoOfAPrime[6], infoOfAPrime[7]);

        // Move to A'
        moveTo(pointAPrime, quaternionAPrime, 5, true);



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
        long startTime = System.currentTimeMillis();
        
        Log.d(getElapsedTime(), "moveTo() called!, attempting " + attempts + " times to move from current position to " + point + " and aligning to " + quaternion + ".");
        
        int iterations = 0;
        
        Log.d(getElapsedTime(), "Attempt " + (iterations+1) + " of " + attempts + " to move from current position to " + point + " and aligning to " + quaternion + ".");
        
        Result result = api.moveTo(point, quaternion, printRobotPosition);
        ++iterations;
        
        while (!result.hasSucceeded() && iterations < attempts){
            Log.d(getElapsedTime(), "Attempt " + (iterations+1) + " of " + attempts + " to move from current position to " + point + " and aligning to " + quaternion + ".");
            result = api.moveTo(point, quaternion, printRobotPosition);
            ++iterations;
        }
        
        if (result.hasSucceeded()){
            Log.d(getElapsedTime(), "Successfully moved to " + point + " and aligned to " + quaternion + " in " + (iterations+1) + " attempts taking "+ calculateTime(startTime) + "!");
        } else {
            Log.d(getElapsedTime(), "Failed to move to " + point + " and aligning to " + quaternion + "!");
        }
    }
    
    private String readQRCode(Bitmap image, int attempts){

        Log.d(getElapsedTime(), "readQRCode() called!");
        
        com.google.zxing.Result result = null;

        int[] intArray = new int[image.getWidth() * image.getHeight()];

        image.getPixels(intArray, 0, image.getWidth(), 0, 0 , image.getWidth(), image.getHeight());
        com.google.zxing.BinaryBitmap binaryBitmap = new BinaryBitmap(new HybridBinarizer(new RGBLuminanceSource(image.getWidth(), image.getHeight(), intArray)));

        try {
            result = new QRCodeReader().decode(binaryBitmap);
        } catch (Exception exception){
            exception.printStackTrace();
        }

        return result.getText();
    }

    /**
     * Tries to report mission completion.
     * @param attempts The number of attempts before giving up. Note that if reporting is successful,
     *                 the method "gives up" because there's no point in reporting "Mission Complete" twice.
     * @param delay The wait time between the attempts.
     */
    private void tryToReportMissionCompletion(int attempts, long delay){

        Log.d(getElapsedTime(), "tryToReportMissionCompletion() called!, attempting " + attempts + "times to report mission completion while waiting " + delay + "milliseconds between each call.");

        for (int a = 0; a < attempts; a++){

            // Delay
            try {
                Thread.sleep(delay);
            } catch (InterruptedException exception) {
                exception.printStackTrace();
            }

            // Report
            if (api.reportMissionCompletion()){
                Log.d(getElapsedTime(), "Successfully reported mission completion after " + a+1 + " times!");
                return;
            }

        }

        Log.d(getElapsedTime(), "Failed to report mission completion in " + attempts + "attempts!");

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

        Log.d(getElapsedTime(), "openFlashlight() called!, setting the front flashlight brightness to " + brightness + " for " + flashlightMS + "milliseconds.");

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


    /**
     * Gets the time elapsed since starting the mission.
     * @return The time passed since starting the mission.
     */
    private String getElapsedTime(){
        return ((System.currentTimeMillis() - timeStarted) / MILLISECONDS_IN_A_SECOND) + "s";
    }

    /**
     * TODO: fill this
     * @param startTime
     * @return
     */
    private String calculateTime(long startTime){
        return ((System.currentTimeMillis() - startTime) / MILLISECONDS_IN_A_SECOND) + "s";
    }
}

