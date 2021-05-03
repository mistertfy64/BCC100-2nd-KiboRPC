package jp.jaxa.iss.kibo.rpc.defaultapk;

import android.graphics.Bitmap;
import android.graphics.Color;
import android.os.StrictMode;
import android.os.SystemClock;
import android.util.Base64;
import android.util.Log;

import com.google.common.base.Splitter;
import com.google.common.collect.ArrayTable;
import com.google.zxing.BinaryBitmap;
import com.google.zxing.LuminanceSource;
import com.google.zxing.MultiFormatReader;
import com.google.zxing.NotFoundException;
import com.google.zxing.RGBLuminanceSource;
import com.google.zxing.client.j2se.BufferedImageLuminanceSource;
import com.google.zxing.common.HybridBinarizer;
import com.google.zxing.qrcode.QRCodeReader;
import com.google.zxing.qrcode.encoder.QRCode;

import org.apache.commons.codec.binary.StringUtils;
import org.json.*;

import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.IOException;
import java.io.PrintWriter;
import java.io.StringWriter;
import java.time.LocalDateTime;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.stream.IntStream;

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
    int timesCalled = 0;
    float tx = 0, ty = 0, tz = 0;
    float offset = 0.9f;

    @Override
    protected void runPlan1() {

        Log.d("INFO", "Te = xs = Info [!] = Concerning, [!!] = Warning, [!!!] = Error");

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
        Log.d("Te = " + getElapsedTime(), "Started Mission!");



        // Move to first pos (Step 1)
        moveTo(pointA, quaternionA, 5, true);

        // Turn on/off flashlight
        logMessage("Turning on flashlight...");
        api.flashlightControlFront(0.555f);
        api.flashlightControlBack(0.555f);

        // (read QR, move to Point-A’, read AR and aim the target) (Step 2)
        Bitmap image = cropImage(api.getBitmapNavCam(), 589,573,852-589,836-573);
        String content = readQRCode(image, 2);

        if (content == null){
            moveTo(pointA, quaternionA, 5, true); // REMOVE THIS IF IT DOESN'T WORK
            image = cropImage(api.getBitmapNavCam(), 589,573,852-589,836-573);
            content = readQRCode(image, 5);
            api.sendDiscoveredQR(content);
        } else {
            api.sendDiscoveredQR(content);
        }


        try {
            Thread.sleep(100);
        } catch (InterruptedException exception) {
            StringWriter stringWriter = new StringWriter();
            exception.printStackTrace(new PrintWriter(stringWriter));
            logException(stringWriter.toString());
        }

        // Turn off flashlight
        logMessage("Turning off flashlight...");
        api.flashlightControlFront(0f);
        api.flashlightControlBack(0f);


        float[] infoOfAPrime = parseQRCodeContent(content);


        tx = infoOfAPrime[1];
        ty = infoOfAPrime[2];
        tz = infoOfAPrime[3];


        float[][] keepOutZonesNumbers = {
                {}, //0
                {1, 5, 9, 10, 13, 14, 15, 16}, // 1
                {9, 10, 11, 12, 13, 14, 15, 16}, // 2
                {4, 8, 11, 12, 13, 14, 15, 16},
                {3, 4, 7, 8, 11, 12, 15, 16},
                {1, 2, 3, 4, 7, 8, 12, 16},
                {1, 2, 3, 4, 5, 6, 7, 8},
                {1, 2, 3, 4, 5, 6, 9, 13},
                {1, 2, 5, 7, 9, 10, 13, 14}
        };

        float[][] keepOutZones1Min = {
                {}, // 0
                {tx - 0.3f, ty, tz - 0.3f}, // 1
                {tx - 0.075f, ty, tz - 0.3f}, // 2
                {tx, ty, tz - 0.3f},
                {tx + 0.075f, ty, tz - 0.3f},
                {tx - 0.3f, ty, tz - 0.075f},
                {tx - 0.075f, ty, tz - 0.075f},
                {tx, ty, tz - 0.075f},
                {tx, ty, tz - 0.075f},
                {tx - 0.3f, ty, tz},
                {tx - 0.075f, ty, tz},
                {tx, ty, tz},
                {tx + 0.075f, ty, tz},
                {tx - 0.3f, ty, tz + 0.075f},
                {tx - 0.075f, ty, tz + 0.075f},
                {tx, ty, tz + 0.075f},
                {tx + 0.075f, ty, tz + 0.075f},
        };

        float[][] keepOutZones1Max = {
                {}, // 0
                {tx - 0.075f, ty + 1.785f, tz - 0.075f}, // 1
                {tx, ty + 1.785f, tz - 0.075f},
                {tx + 0.075f, ty + 1.785f, tz - 0.075f},
                {tx + 0.3f, ty + 1.785f, tz - 0.075f},
                {tx - 0.075f, ty + 1.785f, tz},
                {tx, ty + 1.785f, tz},
                {tx + 0.075f, ty + 1.785f, tz},
                {tx + 0.3f, ty + 1.785f, tz},
                {tx - 0.075f, ty + 1.785f, tz + 0.075f},
                {tx, ty + 1.785f, tz + 0.075f},
                {tx + 0.075f, ty + 1.785f, tz + 0.075f},
                {tx + 0.3f, ty + 1.785f, tz + 0.075f},
                {tx - 0.075f, ty + 1.785f, tz + 0.3f},
                {tx, ty + 1.785f, tz + 0.3f},
                {tx + 0.075f, ty + 1.785f, tz + 0.3f},
                {tx + 0.3f, ty + 1.785f, tz + 0.3f},
        };


        float keepOutAreaPattern = (float) infoOfAPrime[0];

        /*
        int[] activeKeepOutZones = new int[8];

        ArrayList<ArrayList<ArrayList<Float>>> areasToAvoid = new ArrayList<>();


        for (int i = 0; i <= 8; i++) {
            if (i + 1 == infoOfAPrime[0]) {
                for (int j = 0; j < 8; j++) {
                    activeKeepOutZones[i] = (int) keepOutZonesNumbers[i + 1][j];
                }
            }
        }

        for (int i = 1; i <= 16; i++) {
            for (int j = 1; j <= 8; j++) {
                if (activeKeepOutZones[j] == i) {
                    ArrayList<ArrayList<Float>> boundary = new ArrayList<>();

                    ArrayList<Float> boundaryMin = new ArrayList<>();
                    for (int k = 0; k < 3; k++) {
                        boundaryMin.set(k, keepOutZones1Min[i][k]);
                    }

                    ArrayList<Float> boundaryMax = new ArrayList<>();
                    for (int k = 0; k < 3; k++) {
                        boundaryMax.set(k, keepOutZones1Max[i][k]);
                    }

                    boundary.add(boundaryMin);
                    boundary.add(boundaryMax);

                    areasToAvoid.add(boundary);
                }
            }
        }
        */

        // A'
        Point pointAPrime = new Point(infoOfAPrime[1], infoOfAPrime[2], infoOfAPrime[3]);
        Quaternion quaternionAPrime = new Quaternion(infoOfAPrime[4], infoOfAPrime[5], infoOfAPrime[6], infoOfAPrime[7]);

        // A''
        Point pointAPrimePrime = calculatePointAPrimePrime((int) infoOfAPrime[0]);
        Quaternion quaternionAPrimePrime = calculateQuaternionAPrimePrime((int) infoOfAPrime[0]);

        if ((1 <= keepOutAreaPattern && keepOutAreaPattern <= 4) || keepOutAreaPattern == 8){
            // Pattern is 1, 2, 3, 4 or 8
            // Move to A''
            moveTo(pointAPrimePrime, quaternionAPrimePrime, 5, true);
        } else {
            // Pattern is 5, 6 or 7

        }



        // Turn on laser
        api.laserControl(true);

        api.takeSnapshot();

        api.laserControl(false);

        // B
        moveTo(pointB, quaternionBInverted, 5, true); // <----- ?
        moveTo(pointB, quaternionB, 5, true);

        // Finished
        tryToReportMissionCompletion(10, 50L);
    }


    @Override
    protected void runPlan2() {
    }

    @Override
    protected void runPlan3() {
    }

    /**
     * Moves the robot to the point and aligns in to the quaternion.
     *
     * @param point              The point to move to.
     * @param quaternion         The rotation to align to.
     * @param attempts           The number of attempts.
     * @param printRobotPosition Whether to print the robot's position.
     */
    private void moveTo(Point point, Quaternion quaternion, int attempts, boolean printRobotPosition) {
        long startTime = System.currentTimeMillis();

        logMessage("moveTo() called!, attempting " + attempts + " times to move from current position to " + point + " and aligning to " + quaternion + ".");

        int iterations = 0;

        logMessage("Attempt " + (iterations + 1) + " of " + attempts + " to move from current position to " + point + " and aligning to " + quaternion + ".");

        Result result = api.moveTo(point, quaternion, printRobotPosition);
        ++iterations;

        while (!result.hasSucceeded() && iterations < attempts) {
            logMessage("Attempt " + (iterations + 1) + " of " + attempts + " to move from current position to " + point + " and aligning to " + quaternion + ".");
            result = api.moveTo(point, quaternion, printRobotPosition);
            ++iterations;
        }

        if (result.hasSucceeded()) {
            logMessage("Successfully moved to " + point + " and aligned to " + quaternion + " in " + (iterations + 1) + " attempt(s) taking " + calculateTime(startTime) + "!");
        } else {
            logMessage("Failed to move to " + point + " and aligning to " + quaternion + "!");
        }
    }

    private String readQRCode(Bitmap image, int attempts) {

        timesCalled++;

        int iterations = 0;
        logMessage("readQRCode() called!");

        com.google.zxing.Result result = null;

        /*
        if (timesCalled == 1){
            logMessage("(FIRST ITERATION) Original Image's Base64 encoding is: ");
            logMessage("================================================================================= START OF ORIGINAL IMAGE BASE64 ENCODING TEXT (1st) =================================================================================");
            logBase64Data(encodeImageToBase64(image), 4000);
            logMessage("================================================================================== END OF ORIGINAL IMAGE BASE64 ENCODING TEXT (1st)  =================================================================================");
        } else {
            logMessage("(SECOND ITERATION) Original Image's Base64 encoding is: ");
            logMessage("================================================================================= START OF ORIGINAL IMAGE BASE64 ENCODING TEXT (2nd) =================================================================================");
            logBase64Data(encodeImageToBase64(image), 4000);
            logMessage("================================================================================== END OF ORIGINAL IMAGE BASE64 ENCODING TEXT  (2nd) =================================================================================");
        }
        */

        while (result == null && iterations < attempts) {

            int[] array = new int[image.getWidth() * image.getHeight()];
            image.getPixels(array, 0, image.getWidth(), 0, 0, image.getWidth(), image.getHeight());
            BinaryBitmap imageToRead = new BinaryBitmap(new HybridBinarizer(new RGBLuminanceSource(image.getWidth(), image.getHeight(), array)));

            try {
                result = new QRCodeReader().decode(imageToRead);
                logMessage("QR code successfully read in " + (iterations + 1) + " attempt(s)! The QR Code scanned returned this: " + result);
                return result.getText();
            } catch (Exception exception) {
                StringWriter stringWriter = new StringWriter();
                exception.printStackTrace(new PrintWriter(stringWriter));
                logException(stringWriter.toString());
            }

            ++iterations;
        }

        logException("Unable to read QR Code in " + attempts + " attempts! Returning null...");
        return null;

    }

    /**
     * Tries to report mission completion.
     *
     * @param attempts The number of attempts before giving up. Note that if reporting is successful,
     *                 the method "gives up" because there's no point in reporting "Mission Complete" twice.
     * @param delay    The wait time between the attempts.
     */
    private void tryToReportMissionCompletion(float attempts, long delay) {

        logMessage("tryToReportMissionCompletion() called!, attempting " + attempts + "times to report mission completion while waiting " + delay + "milliseconds between each call.");

        for (int a = 0; a < attempts; a++) {

            // Delay
            try {
                Thread.sleep(delay);
            } catch (InterruptedException exception) {
                StringWriter stringWriter = new StringWriter();
                exception.printStackTrace(new PrintWriter(stringWriter));
                logException(stringWriter.toString());
            }

            // Report
            boolean reportMissionCompletionSuccess = api.reportMissionCompletion();
            if (reportMissionCompletionSuccess) {
                logMessage("Successfully reported mission completion after " + (a + 1) + " attempt(s)!");
                return;
            }

        }

        logMessage("Failed to report mission completion in " + attempts + "attempt(s)!");

    }

    /**
     * Parses a JSON String.
     *
     * @param content The JSON String.
     * @return A float array containing the KOZ pattern, the position and the orientation of Point A'
     */
    private float[] parseQRCodeContent(String content) {

        double p = 0, xAsDouble = 0, yAsDouble = 0, zAsDouble = 0;
        try {
            JSONObject jsonObject = new JSONObject(content);
            p = jsonObject.getDouble("p");
            xAsDouble = jsonObject.getDouble("x");
            yAsDouble = jsonObject.getDouble("y");
            zAsDouble = jsonObject.getDouble("z");
        } catch (JSONException exception) {
            StringWriter stringWriter = new StringWriter();
            exception.printStackTrace(new PrintWriter(stringWriter));
            logException(stringWriter.toString());
        }

        return new float[]{(float) p, (float) xAsDouble, (float) yAsDouble, (float) zAsDouble, 0f, 0f, -0.707f, 0.707f};
    }

    /**
     * Gets the time elapsed since starting the mission.
     *
     * @return The time passed since starting the mission.
     */
    private String getElapsedTime() {
        return Long.toString(((System.currentTimeMillis() - timeStarted) / MILLISECONDS_IN_A_SECOND));
    }

    /**
     * TODO: fill this
     *
     * @param startTime
     * @return
     */
    private String calculateTime(long startTime) {
        return Long.toString(((System.currentTimeMillis() - startTime) / MILLISECONDS_IN_A_SECOND));
    }

    /**
     * Logs a message.
     *
     * @param message The message
     */
    private void logMessage(String message) {
        Log.d("Te = " + getElapsedTime() + "s", message);
    }

    /**
     * Logs an exception.
     *
     * @param message The exception's message.
     */
    private void logException(String message) {
        Log.e("[!!!] Te = " + getElapsedTime() + "s", message);
    }

    private Bitmap cropImage(Bitmap image, int x, int y, int w, int h) {
        return Bitmap.createBitmap(image, x, y, w, h);
    }

    private Bitmap cropImage(Bitmap image, float percentX, float percentY, int targetWidth, int targetHeight) {
        int wi = image.getWidth(), hi = image.getHeight();
        return Bitmap.createBitmap(image, (int) percentX * wi, (int) percentY * hi, targetWidth, targetHeight);
    }

    private Bitmap cropImage(Bitmap image, float percentageX1, float percentageY1, float targetX2Percentage, float targetY2Percentage) {
        int wi = image.getWidth(), hi = image.getHeight();
        // ax-bx == (a-b)x
        return Bitmap.createBitmap(image, (int) percentageX1 * wi, (int) percentageY1 * hi, (int) (targetX2Percentage - percentageX1) * wi, (int) (targetY2Percentage - percentageY1) * hi);
    }


    /**
     * TODO: Fill this but its probably useless lol
     * Checks if a point (px, py, pz) is in a cube.
     *
     * @param px
     * @param py
     * @param pz
     * @param cx1
     * @param cy1
     * @param cz1
     * @param cx2
     * @param cy2
     * @param cz2
     * @return
     */
    private boolean checkIfPointIsInCube(float px, float py, float pz, float cx1, float cy1, float cz1, float cx2, float cy2, float cz2) {
        return cx1 <= px && px <= cx2 && cy1 <= py && py <= cy2 && cz1 <= pz && pz <= cz2;
    }

    /**
     * TODO: Fill this but its probably useless lol
     * Check if a line passes through a cube
     *
     * @param lx1
     * @param ly1
     * @param lz1
     * @param lx2
     * @param ly2
     * @param lz2
     * @param cx1
     * @param cy1
     * @param cz1
     * @param cx2
     * @param cy2
     * @param cz2
     * @param ix
     * @param iy
     * @param iz
     * @return
     */
    private boolean checkIfLinePassesThroughCube(float lx1, float ly1, float lz1, float lx2, float ly2, float lz2, float cx1, float cy1, float cz1, float cx2, float cy2, float cz2, float ix, float iy, float iz) {
        float x = lx2, y = ly2, z = lz2;
        while (lx1 <= x && ly1 <= y && lz1 <= z) {
            x -= ix;
            y -= iy;
            z -= iz;
            if (checkIfPointIsInCube(x, y, z, cx1, cy1, cz1, cx2, cy2, cz2)) {
                return true;
            }
        }
        return false;
    }


    private Point calculatePointAPrimePrime(int keepOutAreaPattern){
        float x = 0, y = 0, z = 0;
        // x
        if (keepOutAreaPattern == 1 || keepOutAreaPattern == 8){
            x = tx+offset;
        } else if (keepOutAreaPattern == 3|| keepOutAreaPattern == 4){
            x = tx-offset;
        } else if (keepOutAreaPattern == 2){
            x = tx;
        }
        // y
        y = ty;
        // z
        if ((1 <= keepOutAreaPattern && keepOutAreaPattern <= 4) || keepOutAreaPattern == 8){
            z = tz+offset;
        }
        return new Point(x, y, z);
    }

    private Quaternion calculateQuaternionAPrimePrime(int keepOutAreaPattern){
        float o1 = 0.108f, o2 = 0.418f, o3 = 0.570f, o4 = 0.699f, e1 = 0.000f, e2 = 0.500f, e3 = 0.707f;
        float[][] possibleQuaternions = new float[][]{{o1, -1*o2, o3, o4}, {e1, e3, e3, e1}, {-1*o1, o2, o3, o4}, {e2, e2, -1*e2, e2}, {o2, o1, -1*o4, o3}, {e1, e1, -1*e3, e3}, {-1*o2, -1*o1, -1*o4, o3}, {e2, -1*e2, -1*e2,- 1*e2}};
        return new Quaternion(possibleQuaternions[keepOutAreaPattern-1][0], possibleQuaternions[keepOutAreaPattern-1][1], possibleQuaternions[keepOutAreaPattern-1][2], possibleQuaternions[keepOutAreaPattern-1][3]);
    }



    private Point calculatePointAPrimePrimePrime(int keepOutAreaPattern, float[] infoOfAPrime) {
        float x = infoOfAPrime[1], y = infoOfAPrime[2], z = infoOfAPrime[3];
        switch (keepOutAreaPattern) {
            case 1: {
                return new Point(x + 0.085f, y, z + 0.085f);
            }
            case 2: {
                return new Point(x, y, z + 0.085f);
            }
            case 3: {
                return new Point(x - 0.085f, y, z + 0.085f);
            }
            case 4: {
                return new Point(x - 0.085f, y, z);
            }
            case 5: {
                return new Point(x - 0.085f, y, z - 0.085f);
            }
            case 6: {
                return new Point(x, y, z - 0.085f);
            }
            case 7: {
                return new Point(x + 0.085f, y, z - 0.085f);
            }
            case 8: {
                return new Point(x + 0.085f, y, z);
            }
        }
        return new Point(x, y, z);
    }

    private void logBase64Data(String base64Data, int interval) {
        int i = 0;

        while (base64Data.length() > interval) {
            logMessage(base64Data.substring(0, interval));
            base64Data = base64Data.substring(interval);
            i++;

            if (i % 100 == 0) {
                SystemClock.sleep(5);
            }
        }
        logMessage(base64Data);
    }


    private String encodeImageToBase64(Bitmap image){
        ByteArrayOutputStream byteArrayOutputStream = new ByteArrayOutputStream();
        image.compress(Bitmap.CompressFormat.PNG, 100, byteArrayOutputStream);
        byte[] byteArray = byteArrayOutputStream .toByteArray();
        String encoded = Base64.encodeToString(byteArray, Base64.DEFAULT);
        return encoded;
    }

}

