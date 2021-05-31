package jp.jaxa.iss.kibo.rpc.defaultapk;

import android.graphics.Bitmap;
import android.graphics.Matrix;
import android.os.SystemClock;
import android.util.Base64;
import android.util.Log;

import com.google.zxing.BinaryBitmap;
import com.google.zxing.RGBLuminanceSource;
import com.google.zxing.common.HybridBinarizer;
import com.google.zxing.qrcode.QRCodeReader;

import org.json.JSONException;
import org.json.JSONObject;

import java.io.ByteArrayOutputStream;
import java.io.PrintWriter;
import java.io.StringWriter;

import gov.nasa.arc.astrobee.Kinematics;
import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

public class YourService extends KiboRpcService {


    // FEEL FREE TO EDIT

    // Whether to send log messages. Set it to true to enable logging, which takes more time. Set it to false to disable logging, which takes less time. Default value is true.
    final boolean LOG_MESSAGES = true;
    // The offset coefficient.
    final int DEFAULT_OFFSET_COEFFICIENT = 3;
    // The offset.
    final float OFFSET = 0.4270f;


    // DO NOT EDIT

    final long MILLISECONDS_IN_A_SECOND = 1000;
    long timeStarted;
    int timesCalled = 0;
    float tx = 0, ty = 0, tz = 0;
    float[] infoOfAPrime;
    final float PI = 3.1415f;
    float distanceFromTargetToRobot = 0;


    @Override
    protected void runPlan1() {

        Log.d("INFO", "Te = xs = Info [!] = Concerning, [!!] = Warning, [!!!] = Error");

        // A
        Point pointA = new Point(11.21f, -9.8f, 4.79f);
        Quaternion quaternionA = new Quaternion(0f, 0f, -0.707f, 0.707f);

        // B
        Point pointB = new Point(10.6f, -8.0f, 4.5f);
        Quaternion quaternionB = new Quaternion(0f, 0f, -0.707f, 0.707f);



        timeStarted = System.currentTimeMillis();
        api.startMission();




        // Move to first pos (Step 1)
        moveAndAlignTo(pointA, quaternionA, 5, true);

        // Turn on/off flashlight
        logMessage("Turning on flashlight...");
        api.flashlightControlFront(0.555f);
        api.flashlightControlBack(0.555f);

        // (read QR, move to Point-A’, read AR and aim the target) (Step 2)
        Bitmap image = cropImage(api.getBitmapNavCam(), 589,573,852-589,836-573);
        String content = readQRCode(image, 2);

        if (content.equals("")){
            moveAndAlignTo(pointA, quaternionA, 5, true); // REMOVE THIS IF IT DOESN'T WORK
            image = cropImage(api.getBitmapNavCam(), 589,573,852-589,836-573);
            content = readQRCode(image, 5);
            if (content.equals("")) {
                moveAndAlignTo(pointA, quaternionA, 5, true); // REMOVE THIS IF IT DOESN'T WORK
                image = cropImage(api.getBitmapNavCam(), 589, 573, 852 - 589, 836 - 573);
                content = readQRCode(image, 5);
                api.sendDiscoveredQR(content);
            } else {
                api.sendDiscoveredQR(content);
            }
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


        infoOfAPrime = parseQRCodeContent(content);

        int keepOutAreaPattern = (int) infoOfAPrime[0];
        tx = infoOfAPrime[1];
        ty = infoOfAPrime[2];
        tz = infoOfAPrime[3];

        /*
        // A'
        Point pointAPrime = new Point(infoOfAPrime[1], infoOfAPrime[2], infoOfAPrime[3]);
        Quaternion quaternionAPrime = new Quaternion(infoOfAPrime[4], infoOfAPrime[5], infoOfAPrime[6], infoOfAPrime[7]);

        // A''
        Point pointAPrimePrime = calculatePointAPrimePrime((int) infoOfAPrime[0]);
        Quaternion quaternionAPrimePrime = calculateQuaternionAPrimePrime((int) infoOfAPrime[0]);

        */

        float initialDistanceFromTargetToRobot = (float) Math.abs(pointA.getZ() - infoOfAPrime[3]);
        distanceFromTargetToRobot = initialDistanceFromTargetToRobot / 4;


        moveAndAlignTo(new Point(tx, ty, tz+distanceFromTargetToRobot), api.getRobotKinematics().getOrientation(), 5, true);
        moveAndAlignToTarget((int) infoOfAPrime[0]);

        /*
        if ((1 <= keepOutAreaPattern && keepOutAreaPattern <= 4) || keepOutAreaPattern == 8) {
            // Pattern is 1, 3, 4 or 8
            // Move and align to A''
            moveAndAlignTo(pointAPrimePrime, quaternionAPrimePrime, 5, true);



        } else {

            moveAndAlignTo(api.getRobotKinematics().getPosition(), quaternionAPrimePrime, 5, true);
            // Pattern is 5, 6 or 7

        }*/



        // Turn on laser
        api.laserControl(true);

        api.takeSnapshot();

        api.laserControl(false);

        // B
        moveAndAlignTo(pointB, quaternionB, 5, true);

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
     * @return                   true if the robot can successfully move and align to the position in the specified number of attempts, false if otherwise.
     */
    private boolean moveAndAlignTo(Point point, Quaternion quaternion, int attempts, boolean printRobotPosition) {

        long startTime = System.currentTimeMillis();

        logMessage("[METHOD INVOCATION] moveAndAlignTo() called! The robot will now attempt " + attempts + " times to move to " + point + " and rotate to " + quaternion + ".");

        int iterations = 0;


        while (iterations < attempts) {
            logMessage("[ATTEMPT " + (iterations+1) + "/" + attempts + "] Attempt " + (iterations+1) + " of " + attempts + " to move from the robot's current position to " + point + " and aligning to " + quaternion + ".");
            Result result = api.moveTo(point, quaternion, printRobotPosition);
            ++iterations;

            if (result.hasSucceeded()) {
                logMessage("[PASS] Successfully moved to " + point + " and aligned to " + quaternion + " in " + (iterations + 1) + " attempt(s) taking " + calculateTime(startTime) + " seconds!");
                return true;
            }
        }


        logMessage("[FAIL] Failed to move to " + point + " and aligning to " + quaternion + "!");
        return false;

    }

    private String readQRCode(Bitmap image, int attempts) {

        timesCalled++;

        int iterations = 0;
        logMessage("[METHOD INVOCATION] readQRCode() called! The robot will not attempt " + attempts + " times to read a visible QR code.");

        com.google.zxing.Result result = null;

        while (result == null && iterations < attempts) {

            int[] array = new int[image.getWidth() * image.getHeight()];
            image.getPixels(array, 0, image.getWidth(), 0, 0, image.getWidth(), image.getHeight());
            BinaryBitmap imageToRead = new BinaryBitmap(new HybridBinarizer(new RGBLuminanceSource(image.getWidth(), image.getHeight(), array)));

            try {
                result = new QRCodeReader().decode(imageToRead);
                logMessage("[PASS] QR code successfully read in " + (iterations + 1) + " attempt(s)! The QR Code scanned returned this: " + result);
                return result.getText();
            } catch (Exception exception) {
                StringWriter stringWriter = new StringWriter();
                exception.printStackTrace(new PrintWriter(stringWriter));
                logException(stringWriter.toString());
            }

            ++iterations;
        }

        logException("[FAIL] Unable to read QR Code in " + attempts + " attempts! Returning an empty string...");
        return "";

    }

    /**
     * Tries to report mission completion.
     *
     * @param attempts The number of attempts before giving up. Note that if reporting is successful,
     *                 the method "gives up" because there's no point in reporting "Mission Complete" twice.
     * @param delay    The wait time between the attempts.
     */
    private void tryToReportMissionCompletion(float attempts, long delay) {

        logMessage("[METHOD INVOCATION] tryToReportMissionCompletion() called!, The robot will now attempt " + attempts + "times to report mission completion while waiting " + delay + "milliseconds between each call.");

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
                logMessage("[PASS] Successfully reported mission completion after " + (a + 1) + " attempt(s)!");
                return;
            }

        }

        logMessage("[FAIL] Failed to report mission completion in " + attempts + "attempt(s)!");

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
        if (LOG_MESSAGES){
            Log.d("Te = " + getElapsedTime() + "s", message);
        }
    }

    /**
     * Logs an exception.
     *
     * @param message The exception's message.
     */
    private void logException(String message) {
        if (LOG_MESSAGES){
            Log.e("[!!!] Te = " + getElapsedTime() + "s", message);
        }
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
        Kinematics kinematics = api.getRobotKinematics();
        float cx = (float) kinematics.getPosition().getX();
        float cy = (float) kinematics.getPosition().getY();
        float cz = (float) kinematics.getPosition().getZ();

        // tx = target x, ty = target y, tz = target z, mo = DEFAULT_OFFSET_COEFFICIENT*OFFSET
        // cx, cy, cz = ROBOT'S current position

        // KOZP1 = (tx+DEFAULT_OFFSET_COEFFICIENT*OFFSET, ty+DEFAULT_OFFSET_COEFFICIENT*OFFSET, tz)
        // KOZP2 = (tx, ty+DEFAULT_OFFSET_COEFFICIENT*OFFSET, tz)
        // KOZP3 = (tx-DEFAULT_OFFSET_COEFFICIENT*OFFSET, ty+DEFAULT_OFFSET_COEFFICIENT*OFFSET, tz)
        // KOZP4 = (tx, ty+DEFAULT_OFFSET_COEFFICIENT*OFFSET, tz)
        // KOZP5 = (tx, ty, tz)
        // KOZP6 = (tx, ty, tz)
        // KOZP7 = (tx, ty, tz)
        // KOZP8 = (tx, ty+DEFAULT_OFFSET_COEFFICIENT*OFFSET, tz)

        // x is lower <-------------------> x is higher

/*
        switch (keepOutAreaPattern){
            case 1:{
                return new Point(tx+OFFSET, ty, tz-OFFSET);
            }
            case 2:{
                return new Point(tx, ty, tz-OFFSET);
            }
            case 3:{
                return new Point(tx, ty, tz-OFFSET);
            }
            case 4:{
                return new Point(tx+OFFSET, ty, tz-OFFSET);
            }
            case 5:{
                break;
            }
            case 6:{
                break;
            }
            case 7:{
                break;
            }
            case 8:{
                return new Point(tx+OFFSET, ty, tz-OFFSET);
            }

        }
        */


        return rotateRobotByPoint(new Point(11.21f, -9.8f, 4.79f), keepOutAreaPattern > 1 ? convertDegreesToRadians((keepOutAreaPattern-2)*45) : convertDegreesToRadians(315), infoOfAPrime[1], infoOfAPrime[3]);
    }


    // TODO: FIX THIS
    private Quaternion calculateQuaternionAPrimePrime(int keepOutAreaPattern) {

        Quaternion startQuaternion = new Quaternion(0f, 0f, -0.707f, 0.707f);
        switch (keepOutAreaPattern) {
            case 1: {
                return rotateQuaternionByQuaternion(startQuaternion, calculateQuaternionFromAngles(convertDegreesToRadians(15f), convertDegreesToRadians(-42.5f), convertDegreesToRadians(0f)));
            }
            case 2: {
                return rotateQuaternionByQuaternion(startQuaternion, calculateQuaternionFromAngles(convertDegreesToRadians(0f), convertDegreesToRadians(-42.5f), convertDegreesToRadians(0f)));
            }
            case 3: {
                return rotateQuaternionByQuaternion(startQuaternion, calculateQuaternionFromAngles(convertDegreesToRadians(0f), convertDegreesToRadians(-42.5f), convertDegreesToRadians(-15f)));
            }
            case 4: {
                return rotateQuaternionByQuaternion(startQuaternion, calculateQuaternionFromAngles(convertDegreesToRadians(0f), convertDegreesToRadians(0f), convertDegreesToRadians(90f)));
            }
            case 5: {
                break;
            }
            case 6: {
                break;
            }
            case 7: {
                break;
            }
            case 8: {
                return rotateQuaternionByQuaternion(startQuaternion, calculateQuaternionFromAngles(convertDegreesToRadians(0f), convertDegreesToRadians(-42.5f), convertDegreesToRadians(-90f)));
            }
        }
        // at least it tried so we have a 0.00000000000000000000000001% chance of getting a non-zero score but this shouldnt happen
        return new Quaternion(0f, 0f, -0.707f, 0.707f);
    }


    private Quaternion rotateQuaternionByMatrix(Quaternion startQuaternion, Matrix matrix){
        return new Quaternion();
    }

    /**
     * Copied from https://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/code/index.htm
     * @param q1
     * @param q2
     * @return
     */
    private Quaternion rotateQuaternionByQuaternion(Quaternion q1, Quaternion q2){
        float x =  q1.getX() * q2.getW()+ q1.getY() * q2.getZ() - q1.getZ() * q2.getY() + q1.getW() * q2.getX();
        float y = -q1.getX() * q2.getZ() + q1.getY() * q2.getW() + q1.getZ() * q2.getX() + q1.getW() * q2.getY();
        float z =  q1.getX() * q2.getY() - q1.getY() * q2.getX() + q1.getZ() * q2.getW() + q1.getW() * q2.getZ();
        float w = -q1.getX() * q2.getX() - q1.getY() * q2.getY() - q1.getZ() * q2.getZ() + q1.getW() * q2.getW();
        return new Quaternion(x, y, z, w);
    }



    private float convertRadiansToDegrees(float radians){
        return radians*180f/PI;
    }

    private float convertDegreesToRadians(float degrees){
        return degrees*PI/180f;
    }

    /**
     * Converts radian angles to a quaternion.
     * DO NOT CALL THIS METHOD DIRECTLY!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
     * DEGREES ARE IN RADIANS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
     * Stolen from Wikipedia.
     * @param yaw Z
     * @param pitch Y
     * @param roll X
     * @return
     */
    private Quaternion calculateQuaternionFromAngles(float yaw, float pitch, float roll){
        // Abbreviations for the various angular functions
        float cy = (float) Math.cos(yaw * 0.5);
        float sy = (float) Math.sin(yaw * 0.5);
        float cp = (float) Math.cos(pitch * 0.5);
        float sp = (float) Math.sin(pitch * 0.5);
        float cr = (float) Math.cos(roll * 0.5);
        float sr = (float) Math.sin(roll * 0.5);
        
        float w = cr * cp * cy + sr * sp * sy;
        float x = sr * cp * cy - cr * sp * sy;
        float y = cr * sp * cy + sr * cp * sy;
        float z = cr * cp * sy - sr * sp * cy;
            
        return new Quaternion(x,y,z,w);
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

    /**
     * Not going to implement 3D.
     * https://math.stackexchange.com/questions/270194/how-to-find-the-vertices-angle-after-rotation
     * @param startingPoint
     * @param centerX
     * @param centerZ
     * @return
     */
    private Point rotateRobotByPoint(Point startingPoint, float angle, float centerX, float centerZ){
        float currentX = (float) startingPoint.getX();
        float currentZ = (float) startingPoint.getZ();
        return new Point((float) (currentX-centerX)*Math.cos(angle)-(currentZ-centerZ)*Math.sin(angle)+centerX, (float) startingPoint.getY(),(float) (currentX-centerX)*Math.sin(angle)+(currentZ-centerZ)*Math.cos(angle)+centerZ);
    }

    private void moveAndAlignToTarget(int keepOutAreaPattern){
        if (3 <= keepOutAreaPattern && keepOutAreaPattern <= 6){
            for (int i = keepOutAreaPattern; i <= keepOutAreaPattern; i++){
                if (i == 3){
                    moveAndAlignTo(new Point(api.getRobotKinematics().getPosition().getX()-distanceFromTargetToRobot, api.getRobotKinematics().getPosition().getY(), api.getRobotKinematics().getPosition().getZ()), api.getRobotKinematics().getOrientation(), 5, true);
                } else if (i == 6){
                    moveAndAlignTo(new Point(api.getRobotKinematics().getPosition().getX()+distanceFromTargetToRobot, api.getRobotKinematics().getPosition().getY(), api.getRobotKinematics().getPosition().getZ()), api.getRobotKinematics().getOrientation(), 5, true);
                } else {
                    moveAndAlignTo(new Point(api.getRobotKinematics().getPosition().getX(), api.getRobotKinematics().getPosition().getY(), api.getRobotKinematics().getPosition().getZ()+distanceFromTargetToRobot), api.getRobotKinematics().getOrientation(), 5, true);
                }
            }
        } else if (keepOutAreaPattern == 1 ||  keepOutAreaPattern >= 7) {
            for (int i = keepOutAreaPattern == 1 ? 9: keepOutAreaPattern; i >= keepOutAreaPattern; i--){
                if (i==9){
                    moveAndAlignTo(new Point(api.getRobotKinematics().getPosition().getX()+distanceFromTargetToRobot, api.getRobotKinematics().getPosition().getY(), api.getRobotKinematics().getPosition().getZ()), api.getRobotKinematics().getOrientation(), 5, true);
                } else {
                    moveAndAlignTo(new Point(api.getRobotKinematics().getPosition().getX(), api.getRobotKinematics().getPosition().getY(), api.getRobotKinematics().getPosition().getZ()+distanceFromTargetToRobot), api.getRobotKinematics().getOrientation(), 5, true);
                }
            }
        }
        moveAndAlignTo(api.getRobotKinematics().getPosition(), calculateQuaternionAPrimePrime(keepOutAreaPattern), 5, true);
    }


}

