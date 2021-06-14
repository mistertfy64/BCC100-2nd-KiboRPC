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
import org.opencv.android.OpenCVLoader;
import org.opencv.aruco.*;
import org.opencv.core.Mat;


import java.io.ByteArrayOutputStream;
import java.io.PrintWriter;
import java.io.StringWriter;
import java.lang.Math;
import java.util.ArrayList;
import java.util.List;

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
    // The offset coefficient. Default value is 3.
    final float DEFAULT_OFFSET_COEFFICIENT = 2.25f;
    // The offset. Default value is 0.275f.
    final float OFFSET = 0.275f;
    // Whether to use the "preset location". Default value is true.
    final boolean FORCE_USE_PRESETS = true;
    // ???
    final float ANGLE_CONSTANT_1 = 9f;
    final float ANGLE_CONSTANT_2 = 9f;
    
    // DO NOT EDIT

    final long MILLISECONDS_IN_A_SECOND = 1000;
    long timeStarted; // what even is this for?
    int timesCalled = 0;
    float papx = 0, papy = 0, papz = 0;
    float[] infoOfAPrime = new float[]{-1, 0, 0, 0, 0, 0, 0, 0};
    final float PI = 3.1415f;
    float distanceFromQRCodeToRobot = 0;
    int keepOutAreaPattern = 0;
    Point QRCodePosition;
    float differenceOfQRCodeAndARTagXCoordinates = 0f;
    float differenceOfQRCodeAndARTagZCoordinates = 0f;
    Quaternion initialQuaternion = new Quaternion(0f, 0f, -0.707f, 0.707f);

    @Override
    protected void runPlan1() {

        Log.d("INFO", "Te = xs = Info [!] = Concerning, [!!] = Warning, [!!!] = Error");




        api.startMission();
        timeStarted = System.currentTimeMillis();

        if(OpenCVLoader.initDebug()){
            logMessage("Succesfully loaded OpenCV");
        } else {
            logException("Failed to load openCV");
        }



        // A
        Point pointA = new Point(11.21f, -9.8f, 4.79f);
        Quaternion quaternionA = new Quaternion(0f, 0f, -0.707f, 0.707f);

        // B
        Point pointB = new Point(10.6f, -8.0f, 4.5f);
        Quaternion quaternionB = new Quaternion(0f, 0f, -0.707f, 0.707f);


        // Move to first pos (Step 1)
        moveAndAlignTo(pointA, quaternionA, 5, true);

        // Turn on/off flashlight
        changeBrightnessOfBothFlashlights(0.555f, 5);

        // (read QR, move to Point-A’, read AR and aim the QRCode) (Step 2)
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

        changeBrightnessOfBothFlashlights(0f, 5);


        infoOfAPrime = parseQRCodeContent(content);

        keepOutAreaPattern = (int) infoOfAPrime[0];
        papx = infoOfAPrime[1];
        papy = infoOfAPrime[2];
        papz = infoOfAPrime[3];

        float initialDistanceFromQRCodeToRobot = (float) Math.abs(pointA.getZ() - infoOfAPrime[3]);
        distanceFromQRCodeToRobot = initialDistanceFromQRCodeToRobot;
        differenceOfQRCodeAndARTagXCoordinates = Math.abs(papx-11.21f);
        differenceOfQRCodeAndARTagZCoordinates = Math.abs(papz-4.79f);



        moveAndAlignTo(new Point(papx, papy, papz-OFFSET), api.getRobotKinematics().getOrientation(), 5, true);
        moveToPointAPrime();

        changeBrightnessOfBothFlashlights(0.555f, 5);

        double[] positions = detectArUcoMarker(api.getMatNavCam());
        QRCodePosition = calculatePointOfCenterOfQRCode();

        changeBrightnessOfBothFlashlights(0f, 5);


        calibrateAstrobee();

        moveAndAlignTo(getConfidentPosition(), calculateOptimalQuaternion(), 5, true);

        // Turn on laser
        api.laserControl(true);

        logMessage("Taking snapshots...");
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
                logMessage("[PASS] Successfully moved to " + point + " and aligned to " + quaternion + " in " + iterations + " attempt(s) taking " + calculateTime(startTime) + " seconds!");
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
    
    private void moveToPointAPrime(){
        /* OLD
        switch (keepOutAreaPattern){
            case 1:{
                moveAndAlignTo(new Point(papx, papy, papz-OFFSET), api.getRobotKinematics().getOrientation(), 5, true);
                moveAndAlignTo(new Point(papx, papy, papz), api.getRobotKinematics().getOrientation(), 5, true);
                moveAndAlignTo(new Point(papx, papy, papz+OFFSET), api.getRobotKinematics().getOrientation(), 5, true);
                break;
            }
            case 2:{
                moveAndAlignTo(new Point(papx-OFFSET, papy, papz-OFFSET), api.getRobotKinematics().getOrientation(), 5, true);
                break;
            }
            case 3:{
                moveAndAlignTo(new Point(papx-OFFSET, papy, papz-OFFSET), api.getRobotKinematics().getOrientation(), 5, true);
                break;
            }
            case 4:{
                moveAndAlignTo(new Point(papx-OFFSET, papy, papz-OFFSET), api.getRobotKinematics().getOrientation(), 5, true);
                break;
            }
            case 5:{
                moveAndAlignTo(new Point(papx-OFFSET, papy, papz), api.getRobotKinematics().getOrientation(), 5, true);
                break;
            }
            case 6:{
                moveAndAlignTo(new Point(papx, papy, papz), api.getRobotKinematics().getOrientation(), 5, true);
                break;
            }
            case 7:{
                moveAndAlignTo(new Point(papx, papy, papz-OFFSET), api.getRobotKinematics().getOrientation(), 5, true);
                moveAndAlignTo(new Point(papx, papy, papz), api.getRobotKinematics().getOrientation(), 5, true);
                break;
            }
            case 8:{
                moveAndAlignTo(new Point(papx, papy, papz-OFFSET), api.getRobotKinematics().getOrientation(), 5, true);
                moveAndAlignTo(new Point(papx, papy, papz), api.getRobotKinematics().getOrientation(), 5, true);
                break;
            }
        }
        */
        // NEW
        moveAndAlignTo(new Point(papx, papy, papz-differenceOfQRCodeAndARTagZCoordinates/2), initialQuaternion, 5, true);
        switch (keepOutAreaPattern){
            case 1:
            case 8:{
                moveAndAlignTo(new Point(papx+differenceOfQRCodeAndARTagXCoordinates, papy, papz-differenceOfQRCodeAndARTagZCoordinates/2), initialQuaternion, 5, true);
                moveAndAlignTo(new Point(papx+differenceOfQRCodeAndARTagXCoordinates, papy, papz), initialQuaternion, 5, true);
                break;
            }
            case 2:
            case 3:
            case 4: {
                moveAndAlignTo(new Point(papx-differenceOfQRCodeAndARTagXCoordinates, papy, papz-differenceOfQRCodeAndARTagZCoordinates/2), initialQuaternion, 5, true);
                moveAndAlignTo(new Point(papx-differenceOfQRCodeAndARTagXCoordinates, papy, papz), initialQuaternion, 5, true);
                break;
            }
            case 5:
            case 6: {
                moveAndAlignTo(new Point(papx-differenceOfQRCodeAndARTagXCoordinates*DEFAULT_OFFSET_COEFFICIENT*1.25f, papy, papz-differenceOfQRCodeAndARTagZCoordinates/2f), initialQuaternion, 5, true);
                moveAndAlignTo(new Point(papx-differenceOfQRCodeAndARTagXCoordinates*DEFAULT_OFFSET_COEFFICIENT*1.25f, papy, papz), initialQuaternion, 5, true);
                break;
            }
            case 7: {
            }
        }

        moveAndAlignTo(new Point(papx, papy, papz), api.getRobotKinematics().getOrientation(), 5, true);

    }


    private void moveAndAlignToFinish(){
        switch (keepOutAreaPattern){
            case 1:
            case 2:
            case 3:{

                break;
            }
            case 4:{
                break;
            }
            case 5:
            case 6:
            case 7:{
                break;
            }
            case 8:{
                break;
            }
            default: {
                break;
            }
        }
    }

    private Point getConfidentPosition(){
        Kinematics kinematics = api.getTrustedRobotKinematics();
        if (kinematics == null) {
            return api.getRobotKinematics().getPosition();
        }
        return kinematics.getPosition();
    }

    private Quaternion getConfidentQuaternion(){
        Kinematics kinematics = api.getTrustedRobotKinematics();
        if (kinematics == null) {
            return api.getRobotKinematics().getOrientation();
        }
        return kinematics.getOrientation();
    }

    private Quaternion calculateOptimalQuaternion(){
        Quaternion startQuaternion = new Quaternion(0f, 0f, -0.707f, 0.707f);
        switch (keepOutAreaPattern){
            case 1:
            case 8: {
                return rotateQuaternionByQuaternion(startQuaternion, calculateQuaternionFromAngles(-1*(float) Math.toRadians(ANGLE_CONSTANT_1),  -1* (float) Math.toRadians(ANGLE_CONSTANT_2), 0f));
            }
            case 2:
            case 3:
            case 4: {
                return rotateQuaternionByQuaternion(startQuaternion, calculateQuaternionFromAngles((float) Math.toRadians(ANGLE_CONSTANT_1),  -1* (float) Math.toRadians(ANGLE_CONSTANT_2), 0f));
            }
            case 5:
            case 6: {
                return rotateQuaternionByQuaternion(startQuaternion, calculateQuaternionFromAngles((float) Math.toRadians(ANGLE_CONSTANT_1),   (float) Math.toRadians(ANGLE_CONSTANT_2), 0f));
            }
            case 7: {
                return rotateQuaternionByQuaternion(startQuaternion, calculateQuaternionFromAngles(-1*(float) Math.toRadians(ANGLE_CONSTANT_1),  (float) Math.toRadians(ANGLE_CONSTANT_2), 0f));
            }
        }
        return new Quaternion(0f, 0f, -0.707f, 0.707f);
    }
    


    private double[] detectArUcoMarker(Mat mat){

        List<Mat> corners = new ArrayList<>();
        Mat ids = new Mat();

        Aruco.detectMarkers(mat, Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250), corners, ids);
        
        //TODO: If this doesn't work calculate the center and then use that.
        int[][] markerPositions = calculatePositionOfPoints(corners.get(0).get(0,0), corners.get(1).get(0,0), corners.get(2).get(0,0), corners.get(3).get(0,0));
        // logMessage(markerPositions.toString());
        int[] wantedMarkerPosition = new int[2];
        int markerNumber = 1;

        switch (keepOutAreaPattern){
            case 1:
            case 8: {
                wantedMarkerPosition[0] = 1;
                wantedMarkerPosition[1] = 1;
                break;
            }
            case 2:
            case 3:
            case 4: {
                wantedMarkerPosition[0] = 0;
                wantedMarkerPosition[1] = 1;
                break;
            }
            case 5:
            case 6:{
                wantedMarkerPosition[0] = 0;
                wantedMarkerPosition[1] = 0;
                break;
            }
            case 7:{
                wantedMarkerPosition[0] = 1;
                wantedMarkerPosition[1] = 0;
                break;
            }
            default: {
                wantedMarkerPosition[0] = 0;
                wantedMarkerPosition[1] = 1;
                break;
            }
        }

        for (int i = 0; i < 4; i++){
            if (markerPositions[i][0]==wantedMarkerPosition[0]&&markerPositions[i][1]==wantedMarkerPosition[1]){
                markerNumber = i;
                break;
            }
        }


        return calculateCenterFromCorners(corners.get(markerNumber).get(0, 0), corners.get(markerNumber).get(0, 1), corners.get(markerNumber).get(0, 2), corners.get(markerNumber).get(0, 3));


    }

    private void changeBrightnessOfBothFlashlights(float brightness, int attempts){
        Result frontResult = null, backResult = null;
        logMessage("[METHOD INVOCATION] changeBrightnessOfBothFlashlights() called! Attempting " + attempts + " times to change the brightness of both flashlights to " + brightness + "f.");
        int iterations = 0;
        while ((frontResult == null || backResult == null) && iterations < attempts){
            frontResult = api.flashlightControlFront(brightness);
            backResult = api.flashlightControlBack(brightness);
            iterations++;
        }
        logMessage((frontResult==null||backResult==null)?"[FAIL] Failed to change the brightness of both flashlights to " + brightness + "f.":"[PASS] Successfully changed the brightness of both flashlights to " + brightness + "f.");
    }


    /**
     * C3 --- C2
     * |       |
     * |       |
     * |       |
     * C0 --- C1
     * @param c00
     * @param c01
     * @param c02
     * @param c03
     * @return
     */
    private double[] calculateCenterFromCorners(double[] c00, double[] c01, double[] c02, double[] c03){
        return new double[]{(c03[0] + c01[0]) / 2, (c03[1] + c01[1]) / 2 };
    }

    /**
     * Index:
     * 0, 1 --- 1, 1
     * |          |
     * |          |
     * |          |
     * 0, 0 --- 1, 0
     * @return
     */
    private int[][] calculatePositionOfPoints(double[] c1, double[] c2, double[] c3, double[] c4){
        double[][] px = {{c1[0], 1.0}, {c2[0], 2.0}, {c3[0], 3.0}, {c4[0], 4.0}};
        double[][] py = {{c1[1], 1.0}, {c2[1], 2.0}, {c3[1], 3.0}, {c4[1], 4.0}};
        double[][] pxnew = sortArrayBasedOnFirstElement(px);
        double[][] pynew = sortArrayBasedOnFirstElement(py);
        int[][] result = new int[4][2];

        // x
        for (int i = 0; i < 2; i++){
            result[convertDoubleToIndex(pxnew[i][1])][0] = 0;
        }
        for (int i = 2; i < 4; i++){
            result[convertDoubleToIndex(pxnew[i][1])][0] = 1;
        }
        // y
        for (int i = 0; i < 2; i++){
            result[convertDoubleToIndex(pynew[i][1])][1] = 0;
        }
        for (int i = 2; i < 4; i++){
            result[convertDoubleToIndex(pynew[i][1])][1] = 1;
        }
        return result;
    }

    private int convertDoubleToIndex(double whatToConvert){
        if (whatToConvert == 1.0){
            return 0;
        } else if (whatToConvert == 2.0){
            return 1;
        } else if (whatToConvert == 3.0){
            return 2;
        } else if (whatToConvert == 4.0){
            return 3;
        } else {
            return 0;
        }
    }


    private double[][] sortArrayBasedOnFirstElement(double[][] whatToSort){
        double[][] currentArray = whatToSort;
        // bubble sort because i cant think of a better sorting algorithm lol
        for (int i = 0; i < whatToSort.length; i++){
            for (int j = 0; j < whatToSort.length; j++){
                if (i==j){ continue; }
                if (currentArray[i][0] > currentArray[j][0]){
                    double[] tempArray = currentArray[j];
                    currentArray[j] = currentArray[i];
                    currentArray[i] = tempArray;
                }
            }
        }
        return currentArray;
    }


    // 0.01 units <---> 6.4 mat pixels
    // 1 units <---> 640 mat pixels

    private double convertMatPixelsToSimulatorUnits(double pixels){
        return (pixels/6.4d)/100d;
    }

    private double convertSimulatorUnitsToMatPixels(double units){
        return (units*6.4d*100d);
    }

    private Point calculatePointOfCenterOfQRCode(){
        Point position = new Point(infoOfAPrime[1], infoOfAPrime[2], infoOfAPrime[3]);
        float px = (float) position.getX(), py = (float) position.getY(), pz = (float) position.getZ();
        switch(keepOutAreaPattern){
            case 1:{
                px -= 0.1125f;
                pz += 0.04f;
                break;
            }
            case 2:{
                px += 0.1125f;
                pz += 0.04f;
                break;
            }
            case 3:{
                px += 0.1125f;
                pz += 0.04f;
                break;
            }
            case 4:{
                px += 0.1125f;
                pz += 0.04f;
                break;
            }
            case 5:{
                px += 0.1125f;
                pz -= 0.04f;
                break;
            }
            case 6:{
                px += 0.1125f;
                pz -= 0.04f;
                break;
            }
            case 7:{
                px -= 0.1125f;
                pz -= 0.04f;
                break;
            }
            case 8:{
                px -= 0.1125f;
                pz += 0.04f;
                break;
            }

        }
        return new Point(px, py, pz);
    }

    private void calibrateAstrobee(){
        
        moveAndAlignTo(new Point(papx, papy, papz), getConfidentQuaternion(), 5, true);
    }

}

