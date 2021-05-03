package jp.jaxa.iss.kibo.rpc.defaultapk;

import android.graphics.Bitmap;
import android.util.Log;

import com.google.zxing.BinaryBitmap;
import com.google.zxing.LuminanceSource;
import com.google.zxing.RGBLuminanceSource;
import com.google.zxing.common.HybridBinarizer;
import com.google.zxing.qrcode.QRCodeReader;

import org.json.*;

import java.io.PrintWriter;
import java.io.StringWriter;

import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

public class YourService extends KiboRpcService {

    private final long MILLISECONDS_IN_A_SECOND = 1000;
    private long timeStarted;

    @Override
    protected void runPlan1(){

        // A
        Point pointA = new Point(11.21f, -9.8f, 4.79f);
        Quaternion quaternionA = new Quaternion(0f, 0f, -0.707f, 0.707f);

        // B
        Point pointB = new Point(10.6f, -8.0f, 4.5f);
        Quaternion quaternionB = new Quaternion(0f, 0f, -0.707f, 0.707f);

        Quaternion quaternionBInverted = new Quaternion(0f, 0f, 0.707f, -0.707f);

        // Start

        Log.d("", "Starting Mission...");
        timeStarted = System.currentTimeMillis();
        api.startMission();
        Log.d("Te = " + getElapsedTime() , "Started Mission!");



        // Move to first pos (Step 1)
        moveTo(pointA, quaternionA, 5, true);


        // Turn on/off flashlight
        logMessage("Turning on flashlight...");
        api.flashlightControlFront(0.375f);
        api.flashlightControlBack(0.375f);


        try {
            Thread.sleep(2000); // shouldnt take more than 2 seconds
        } catch (InterruptedException exception){
            StringWriter stringWriter = new StringWriter();
            exception.printStackTrace(new PrintWriter(stringWriter));
            logException(stringWriter.toString());
        }

        // (read QR, move to Point-A’, read AR and aim the target) (Step 2)
        Bitmap image = api.getBitmapNavCam();
        String content = readQRCode(image, 5);
        api.sendDiscoveredQR(content);

        try {
            Thread.sleep(1000);
        } catch (InterruptedException exception){
            StringWriter stringWriter = new StringWriter();
            exception.printStackTrace(new PrintWriter(stringWriter));
            logException(stringWriter.toString());
        }

        // Turn off flashlight
        logMessage("Turning off flashlight...");
        api.flashlightControlFront(0f);
        api.flashlightControlBack(0f);



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
        
        logMessage("moveTo() called!, attempting " + attempts + " times to move from current position to " + point + " and aligning to " + quaternion + ".");
        
        int iterations = 0;
        
        logMessage("Attempt " + (iterations+1) + " of " + attempts + " to move from current position to " + point + " and aligning to " + quaternion + ".");
        
        Result result = api.moveTo(point, quaternion, printRobotPosition);
        ++iterations;
        
        while (!result.hasSucceeded() && iterations < attempts){
            logMessage("Attempt " + (iterations+1) + " of " + attempts + " to move from current position to " + point + " and aligning to " + quaternion + ".");
            result = api.moveTo(point, quaternion, printRobotPosition);
            ++iterations;
        }
        
        if (result.hasSucceeded()){
            logMessage("Successfully moved to " + point + " and aligned to " + quaternion + " in " + (iterations+1) + " attempt(s) taking "+ calculateTime(startTime) + "!");
        } else {
            logMessage("Failed to move to " + point + " and aligning to " + quaternion + "!");
        }
    }
    
    private String readQRCode(Bitmap image, int attempts){

        int iterations = 0;
        logMessage("readQRCode() called!");
        
        com.google.zxing.Result result = null;

        while (result == null && iterations < attempts){
            int[] array = new int[image.getWidth() * image.getHeight()];

            image.getPixels(array, 0, image.getWidth(), 0, 0 , image.getWidth(), image.getHeight());

            LuminanceSource luminanceSource = new RGBLuminanceSource(image.getWidth(), image.getHeight(), array);
            BinaryBitmap binaryBitmap = new BinaryBitmap(new HybridBinarizer(luminanceSource));

            try {
                result = new QRCodeReader().decode(binaryBitmap);
                logMessage("QR code successfully read in " + (iterations+1) + "attempt(s)! The QR Code scanned returned this: " + result);
                return result.getText();
            } catch (Exception exception){
                StringWriter stringWriter = new StringWriter();
                exception.printStackTrace(new PrintWriter(stringWriter));
                logException(stringWriter.toString());
            }

            ++iterations;
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

        logMessage("tryToReportMissionCompletion() called!, attempting " + attempts + "times to report mission completion while waiting " + delay + "milliseconds between each call.");

        for (int a = 0; a < attempts; a++){

            // Delay
            try {
                Thread.sleep(delay);
            } catch (InterruptedException exception) {
                StringWriter stringWriter = new StringWriter();
                exception.printStackTrace(new PrintWriter(stringWriter));
                logException(stringWriter.toString());
            }

            // Report
            if (api.reportMissionCompletion()){
                logMessage("Successfully reported mission completion after " + a+1 + " attempt(s)!");
                return;
            }

        }

        logMessage("Failed to report mission completion in " + attempts + "attempt(s)!");

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
            StringWriter stringWriter = new StringWriter();
            exception.printStackTrace(new PrintWriter(stringWriter));
            logException(stringWriter.toString());
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

        logMessage("openFlashlight() called!, setting the front flashlight brightness to " + brightness + " for " + flashlightMS + "milliseconds.");

        // Turn on flashlight

    }


    /**
     * Gets the time elapsed since starting the mission.
     * @return The time passed since starting the mission.
     */
    private String getElapsedTime(){
        return Long.toString(((System.currentTimeMillis() - timeStarted) / MILLISECONDS_IN_A_SECOND));
    }

    /**
     * TODO: fill this
     * @param startTime
     * @return
     */
    private String calculateTime(long startTime){
        return Long.toString(((System.currentTimeMillis() - startTime) / MILLISECONDS_IN_A_SECOND));
    }

    /**
     * Logs a message.
     * @param message The message
     */
    private void logMessage(String message){
        Log.d("Te = " + getElapsedTime() + "s", message);
    }

    /**
     * Logs an exception.
     * @param message The exception's message.
     */
    private void logException(String message){
        Log.e("[!!!] Te = " + getElapsedTime() + "s", message);
    }

    private Bitmap cropImage(Bitmap image, int x, int y, int w, int h){
        return Bitmap.createBitmap(image, x, y, w, h);
    }
}

