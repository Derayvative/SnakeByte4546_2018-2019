package org.firstinspires.ftc.teamcode;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

public class GoldDetectorVuforia extends LinearOpMode {

    VuforiaLocalizer vuforia;
    Bitmap image1;
    Bitmap image2;
    Bitmap image3;

    public GoldDetectorVuforia() {
        /*To access the image: you need to iterate through the images of the frame object:*/

        //getImage();
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        params.vuforiaLicenseKey = "AQ1iIdT/////AAAAGZ0U6OKRfU8tpKf9LKl/7DM85y3Wp791rb6q3WwHfYaY53vqKSjAO8wU2FgulWnDt6gLqu9hB33z1reejMz/NyfL8u11QZlMIbimmnP/v4hvoXZWu0p62V9eMG3R2PQ3Z7rZ0qK8HwsQYE/0jmBhTy0D17M4fWpNW64QQnMJqFxq/N1BXm32PEInYDHBYs7WUrHL5oa9xeSSurxUq/TqDpeJwQM+1/GYppdAqzbcM1gi3yzU7JDLdNtOZ6+lbi5uXlU++GnFvQaEXL9uVcnTwMEgBhBng6oOEVoEDXiSUBuZHuMRGZmHfVXSNE3m1UXWyEdPTlMRI5vfEwfsBHmQTmvYr/jJjng3+tBpu85Q1ivo";
        params.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        vuforia = ClassFactory.getInstance().createVuforia(params);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true); //enables RGB565 format for the image
        vuforia.setFrameQueueCapacity(4); //tells VuforiaLocalizer to only store one frame at a time
    }

    public GoldDetectorVuforia(boolean backCam) {
        //getImage();
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        params.vuforiaLicenseKey = "AQ1iIdT/////AAAAGZ0U6OKRfU8tpKf9LKl/7DM85y3Wp791rb6q3WwHfYaY53vqKSjAO8wU2FgulWnDt6gLqu9hB33z1reejMz/NyfL8u11QZlMIbimmnP/v4hvoXZWu0p62V9eMG3R2PQ3Z7rZ0qK8HwsQYE/0jmBhTy0D17M4fWpNW64QQnMJqFxq/N1BXm32PEInYDHBYs7WUrHL5oa9xeSSurxUq/TqDpeJwQM+1/GYppdAqzbcM1gi3yzU7JDLdNtOZ6+lbi5uXlU++GnFvQaEXL9uVcnTwMEgBhBng6oOEVoEDXiSUBuZHuMRGZmHfVXSNE3m1UXWyEdPTlMRI5vfEwfsBHmQTmvYr/jJjng3+tBpu85Q1ivo";
        if (backCam) {
            params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        } else {
            params.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        }
        vuforia = ClassFactory.getInstance().createVuforia(params);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true); //enables RGB565 format for the image
        vuforia.setFrameQueueCapacity(4); //tells VuforiaLocalizer to only store one frame at a time
    }

    public Bitmap getImage() throws InterruptedException {


        /*To access the image: you need to iterate through the images of the frame object:*/

        //getImage();


        VuforiaLocalizer.CloseableFrame frame = vuforia.getFrameQueue().take();
        long numImages = frame.getNumImages();
        Image rgb = null;
        for (int i = 0; i < numImages; i++) {
            Image img = frame.getImage(i);
            int fmt = img.getFormat();
            if (fmt == PIXEL_FORMAT.RGB565) {
                rgb = frame.getImage(i);
                break;
            }
        }

        Bitmap bm = Bitmap.createBitmap(rgb.getWidth(), rgb.getHeight(), Bitmap.Config.RGB_565);
        bm.copyPixelsFromBuffer(rgb.getPixels());

        return bm;
    }

    public double findCentralYellowness(Bitmap bm_img) throws InterruptedException {

        Color cur_color = null;
        int cur_color_int, rgb[] = new int[3];
        float hsv[] = new float[3];
        int hueMax = 0;
        double whiteScore = 0;

        int width = bm_img.getWidth();
        int height = bm_img.getHeight();

        //double centerX = width / 2;
        double centerX = 565;
        double centerY = 363;
        //double centerY = height * 3.0 / 4;

        double count = 0;

        double distance = 0;

        for (int i = 0; i < width; i = i + 4) {
            for (int j = (int) (height * 1.0 / 3.0); j < (int) (height * 2.0 / 3.0); j = j + 4) {

                cur_color_int = bm_img.getPixel(i, j);

                rgb[0] = cur_color.red(cur_color_int);
                rgb[1] = cur_color.green(cur_color_int);
                rgb[2] = cur_color.blue(cur_color_int);

                Color.RGBToHSV(rgb[0], rgb[1], rgb[2], hsv);

                double hue = hsv[0];
                double sat = hsv[1];

                if (hue <= 50 && hue >= 30 && sat >= 0.80) {
                    count++;
                    distance += getDistance(i, centerX, j, centerY);
                }
            }
        }

       /*
       try {
           telemetry.addData("Central Tendency", distance / count);
           telemetry.addData("Count", count);
           telemetry.addData("Selection Index", Math.log10(distance/count/count));
           //sleep(2000);
       }
       catch (ArithmeticException e){
           telemetry.addData("POints", "None Found");
       }
       telemetry.update();
       */

        double score = distance / count / count;

        if (Double.isNaN(score) || count == 0) {
            return 999999;
        }

        //Score ranges from small decimals (e.g. 0.0001) to thousands, so logarithmic function
        //makes the scores slightly easier to digest, but keeps the comparisons the same
        return Math.log10(distance / count / count);
    }


    public double[] findCentralYellownessExpanded(Bitmap bm_img) throws InterruptedException {

        Color cur_color = null;
        int cur_color_int, rgb[] = new int[3];
        float hsv[] = new float[3];
        int hueMax = 0;
        double whiteScore = 0;

        int width = bm_img.getWidth();
        int height = bm_img.getHeight();

        //double centerX = width / 2;
        double centerX = 565;
        double centerY = 363;

        double averageX = 0;
        double averageY = 0;
        //double centerY = height * 3.0 / 4;

        double hueAvg = 0;
        double satAvg = 0;
        double valAvg = 0;

        double count = 0;

        double distance = 0;

        double xTotal = 0;
        double yTotal = 0;

        for (int i = (int) (width * 0.25); i < (int) (width * 0.75); i = i + 4) {
            for (int j = (int) (height * 1.0 / 3); j < height; j = j + 4) {

                cur_color_int = bm_img.getPixel(i, j);

                rgb[0] = cur_color.red(cur_color_int);
                rgb[1] = cur_color.green(cur_color_int);
                rgb[2] = cur_color.blue(cur_color_int);

                Color.RGBToHSV(rgb[0], rgb[1], rgb[2], hsv);

                double hue = hsv[0];
                double sat = hsv[1];

                //if (hue <= 50 && hue >= 30 && sat >= 0.8){
                //if (hue <= 60 && hue >= 10 && sat >= 0.70) {
                if (sat <= 0.25) {

                } else {
                    count++;
                    distance += getDistance(i, centerX, j, centerY);
                    averageX += i;
                    averageY += j;
                    hueAvg += hue;
                    satAvg += sat;
                }


                yTotal++;
            }
            //xTotal++;
        }

       /*
       try {
           telemetry.addData("Central Tendency", distance / count);
           telemetry.addData("Count", count);
           telemetry.addData("Selection Index", Math.log10(distance/count/count));
           //sleep(2000);
       }
       catch (ArithmeticException e){
           telemetry.addData("POints", "None Found");
       }
       telemetry.update();
       */

        double score = distance / count / count;

        double[] data = {score, count, count / yTotal, hueAvg / count, satAvg / count, averageX / count, averageY / count};


        //Score ranges from small decimals (e.g. 0.0001) to thousands, so logarithmic function
        //makes the scores slightly easier to digest, but keeps the comparisons the same
        return data;
    }

    public double[] findCentralYellownessCIELABTest(Bitmap bm_img) throws InterruptedException {

        Color cur_color = null;
        double cur_color_int, rgb[] = new double[3];
        float hsv[] = new float[3];
        int hueMax = 0;
        double whiteScore = 0;

        int width = bm_img.getWidth();
        int height = bm_img.getHeight();

        //double centerX = width / 2;
        double centerX = 565;
        double centerY = 363;
        //double centerY = height * 3.0 / 4;

        double count = 0;

        double distance = 0;
        //2.2.3.
        //160.100.20


        double[] finalRGB = {0, 0, 0, 9999, 9999, 9999, -9999, -9999, -9999};

        for (int i = (int) (width * 0.25); i < (int) (width * 0.75); i = i + 4) {
            for (int j = 0; j < height / 2.0; j = j + 4) {

                cur_color_int = bm_img.getPixel(i, j);

                rgb[0] = cur_color.red((int) cur_color_int);
                rgb[1] = cur_color.green((int) cur_color_int);
                rgb[2] = cur_color.blue((int) cur_color_int);

                rgb = ColorSpaceConvertor.RGVtoCIELAB(rgb);

                count++;

                finalRGB[0] += rgb[0];
                finalRGB[1] += rgb[1];
                finalRGB[2] += rgb[2];

                if (rgb[0] < finalRGB[3]) {
                    finalRGB[3] = rgb[0];
                }
                if (rgb[1] < finalRGB[4]) {
                    finalRGB[4] = rgb[1];
                }
                if (rgb[2] < finalRGB[5]) {
                    finalRGB[5] = rgb[2];
                }
                if (rgb[0] > finalRGB[6]) {
                    finalRGB[6] = rgb[0];
                }
                if (rgb[1] > finalRGB[7]) {
                    finalRGB[7] = rgb[1];
                }
                if (rgb[2] > finalRGB[8]) {
                    finalRGB[8] = rgb[2];
                }
            }
        }

       /*
       try {
           telemetry.addData("Central Tendency", distance / count);
           telemetry.addData("Count", count);
           telemetry.addData("Selection Index", Math.log10(distance/count/count));
           //sleep(2000);
       }
       catch (ArithmeticException e){
           telemetry.addData("POints", "None Found");
       }
       telemetry.update();
       */

        finalRGB[0] /= count;
        finalRGB[1] /= count;
        finalRGB[2] /= count;
        //Score ranges from small decimals (e.g. 0.0001) to thousands, so logarithmic function
        //makes the scores slightly easier to digest, but keeps the comparisons the same
        return finalRGB;
    }


    public double[] findCentralYellownessCIELAB(Bitmap bm_img) throws InterruptedException {

        Color cur_color = null;
        double cur_color_int, rgb[] = new double[3];
        float hsv[] = new float[3];
        int hueMax = 0;
        double whiteScore = 0;

        int width = bm_img.getWidth();
        int height = bm_img.getHeight();


        //double centerX = width / 2;
        double centerX = 0;
        double centerY = 0;
        //double centerY = height * 3.0 / 4;
        double[] gold = {209, 90, 160};
        double[] background = {300, 13, 31};
        double[] redTape = {300, 12, 20};
        //double[] white = {300, 12, 20};


        double yCount = 0;
        double wCount = 0;

        double total = 0;

        double distance = 0;

        double[] finalRGB = new double[3];

        double[] tally = new double[4];

        double[] sample = new double[3];

        double lowestScore = 9999999;

        double threshold = 50.0;

        for (int i = 0; i < (int) (width); i = i + 5) {
            for (int j = (int) (height / 3.0); j < height * 2.0 / 3.0; j = j + 5) {

                cur_color_int = bm_img.getPixel(i, j);

                rgb[0] = cur_color.red((int) cur_color_int);
                rgb[1] = cur_color.green((int) cur_color_int);
                rgb[2] = cur_color.blue((int) cur_color_int);

                double[] CIELAB = ColorSpaceConvertor.RGVtoCIELAB(rgb);

                double[] v1 = {CIELAB[0], CIELAB[1], CIELAB[2]};

                double diffGold = findVectorDifference(v1, gold);
                double diffBack = findVectorDifference(v1, background);
                double diffRedTape = findVectorDifference(v1, redTape);
                //double diffWhite = findVectorDifference(v1, white);

                double[] scores = {diffGold, diffBack, diffRedTape};

                int lowestIndex = findSmallestIndexOfArray(scores);

                if (scores[lowestIndex] < threshold / 2 && lowestIndex == 0) {
                    //tally[lowestIndex] += 20 * (threshold - scores[lowestIndex]) / threshold;
                    tally[lowestIndex] += 50;
                    //sample[0] += v1[0];
                    //sample[1] += v1[1];
                    //sample[2] += v1[2];
                    centerX += 10 * i;
                    centerY += 10 * j;
                    yCount += 10;
                    //wCount++;
                }

                if (scores[lowestIndex] < threshold && lowestIndex == 0) {
                    //tally[lowestIndex] += 20 * (threshold - scores[lowestIndex]) / threshold;
                    tally[lowestIndex] += 20;
                    //sample[0] += v1[0];
                    //sample[1] += v1[1];
                    //sample[2] += v1[2];
                    centerX += 10 * i;
                    centerY += 10 * j;
                    yCount += 10;
                    //wCount++;
                }
                if (scores[lowestIndex] < 5 * threshold && lowestIndex == 0) {
                    //tally[lowestIndex] += (5 * threshold - scores[lowestIndex]) / (5 * threshold);
                    tally[lowestIndex]++;
                    //sample[0] += v1[0];
                    //sample[1] += v1[1];
                    //sample[2] += v1[2];
                    centerX += i;
                    centerY += j;
                    yCount++;
                    //wCount++;
                }
                if (scores[lowestIndex] < threshold && lowestIndex == 2) {
                    //tally[lowestIndex] += 20 * (threshold - scores[lowestIndex]) / threshold;
                    tally[lowestIndex] += 20;
                    //sample[0] += v1[0];
                    //sample[1] += v1[1];
                    //sample[2] += v1[2];
                    centerX += i;
                    centerY += j;
                    yCount++;
                    //wCount++;
                }
                if (scores[lowestIndex] < 5 * threshold && lowestIndex == 2) {
                    //tally[lowestIndex] += (5 * threshold - scores[lowestIndex]) / (5 * threshold);
                    tally[lowestIndex] += 1;
                    //sample[0] += v1[0];
                    //sample[1] += v1[1];
                    //sample[2] += v1[2];
                    centerX += i;
                    centerY += j;
                    yCount++;
                    //wCount++;
                }


            }
        }

       /*
       try {
           telemetry.addData("Central Tendency", distance / count);
           telemetry.addData("Count", count);
           telemetry.addData("Selection Index", Math.log10(distance/count/count));
           //sleep(2000);
       }
       catch (ArithmeticException e){
           telemetry.addData("POints", "None Found");
       }
       telemetry.update();
       */


        //Score ranges from small decimals (e.g. 0.0001) to thousands, so logarithmic function
        //makes the scores slightly easier to digest, but keeps the comparisons the same
        //String[] array = {"Yellow: " + yCount, "White: " + wCount};
        //sample[0] /= wCount;
        //sample[1] /= wCount;
        //sample[2] /= wCount;
        //tally[3] = lowestScore;
        tally[0] *= -1;
        return tally;
    }


    public double[] findCentralYellownessCIELABThreeLocations(Bitmap bm_img) throws InterruptedException {

        Color cur_color = null;
        double cur_color_int, rgb[] = new double[3];
        float hsv[] = new float[3];
        int hueMax = 0;
        double whiteScore = 0;

        int width = bm_img.getWidth();
        int height = bm_img.getHeight();


        //double centerX = width / 2;
        double centerX = 0;
        double centerY = 0;
        //double centerY = height * 3.0 / 4;
        double[] gold = {209, 90, 160};
        double[] background = {300, 13, 31};
        double[] redTape = {250, 300, 240};
        //double[] white = {300, 12, 20};


        double yCount = 0;
        double wCount = 0;

        double total = 0;

        double distance = 0;

        double[] finalRGB = new double[3];

        double[] tally = new double[3];

        double[] sample = new double[3];

        double lowestScore = 9999999;

        double threshold = 30.0;

        for (int i = 0; i < (int) (width); i = i + 4) {
            for (int j = (int) (0); j < height; j = j + 4) {

                cur_color_int = bm_img.getPixel(i, j);

                rgb[0] = cur_color.red((int) cur_color_int);
                rgb[1] = cur_color.green((int) cur_color_int);
                rgb[2] = cur_color.blue((int) cur_color_int);

                double[] CIELAB = ColorSpaceConvertor.RGVtoCIELAB(rgb);

                double[] v1 = {CIELAB[0], CIELAB[1], CIELAB[2]};

                double diffGold = findVectorDifference(v1, gold);
                double diffBack = findVectorDifference(v1, background);
                double diffRedTape = findVectorDifference(v1, redTape);
                //double diffWhite = findVectorDifference(v1, white);

                double[] scores = {diffGold, diffBack, diffRedTape};

                int lowestIndex = findSmallestIndexOfArray(scores);

                if (scores[lowestIndex] < threshold && lowestIndex == 0) {
                    if (i < width / 3) {
                        tally[0] += 10 * (threshold - scores[lowestIndex]) / threshold;
                    } else if (i < width * 2.0 / 3.0) {
                        tally[1] += 10 * (threshold - scores[lowestIndex]) / threshold;
                    } else {
                        tally[2] += 10 * (threshold - scores[lowestIndex]) / threshold;
                    }
                }

                if (scores[lowestIndex] < 5 * threshold && lowestIndex == 0) {
                    if (i < width / 3) {
                        tally[0] += (5 * threshold - scores[lowestIndex]) / (5 * threshold);
                    } else if (i < width * 2.0 / 3.0) {
                        tally[1] += (5 * threshold - scores[lowestIndex]) / (5 * threshold);
                    } else {
                        tally[2] += (5 * threshold - scores[lowestIndex]) / (5 * threshold);
                    }
                }


            }
        }

       /*
       try {
           telemetry.addData("Central Tendency", distance / count);
           telemetry.addData("Count", count);
           telemetry.addData("Selection Index", Math.log10(distance/count/count));
           //sleep(2000);
       }
       catch (ArithmeticException e){
           telemetry.addData("POints", "None Found");
       }
       telemetry.update();
       */


        //Score ranges from small decimals (e.g. 0.0001) to thousands, so logarithmic function
        //makes the scores slightly easier to digest, but keeps the comparisons the same
        //String[] array = {"Yellow: " + yCount, "White: " + wCount};
        //sample[0] /= wCount;
        //sample[1] /= wCount;
        //sample[2] /= wCount;
        //tally[1] = centerX / yCount;
        //tally[2] = centerY / yCount;
        //tally[3] = lowestScore;
        return tally;
    }

    public double[] findCentralYellownessCIELABTwoLocations(Bitmap bm_img) throws InterruptedException {

        Color cur_color = null;
        double cur_color_int, rgb[] = new double[3];
        float hsv[] = new float[3];
        int hueMax = 0;
        double whiteScore = 0;

        int width = bm_img.getWidth();
        int height = bm_img.getHeight();


        //double centerX = width / 2;
        double centerX = 0;
        double centerY = 0;
        //double centerY = height * 3.0 / 4;
        double[] gold = {209, 90, 160};
        double[] background = {300, 13, 31};
        double[] redTape = {250, 300, 240};
        //double[] white = {300, 12, 20};


        double yCount = 0;
        double wCount = 0;

        double total = 0;

        double distance = 0;

        double[] finalRGB = new double[3];

        double[] tally = new double[2];

        double[] sample = new double[3];

        double lowestScore = 9999999;

        double threshold = 50.0;

        for (int i = 0; i < (int) (width) ; i = i + 4) {
            for (int j = (int) (0); j < height ; j = j + 4) {

                cur_color_int = bm_img.getPixel(i, j);

                rgb[0] = cur_color.red((int) cur_color_int);
                rgb[1] = cur_color.green((int) cur_color_int);
                rgb[2] = cur_color.blue((int) cur_color_int);

                double[] CIELAB = ColorSpaceConvertor.RGVtoCIELAB(rgb);

                double[] v1 = {CIELAB[0], CIELAB[1], CIELAB[2]};

                double diffGold = findVectorDifference(v1, gold);
                double diffBack = findVectorDifference(v1, background);
                double diffRedTape = findVectorDifference(v1, redTape);
                //double diffWhite = findVectorDifference(v1, white);

                double[] scores = {diffGold, diffBack, diffRedTape};

                int lowestIndex = findSmallestIndexOfArray(scores);

                if (scores[lowestIndex] < threshold && lowestIndex == 0) {
                    if (i < width / 2) {
                        tally[0] += 10 * (threshold - scores[lowestIndex]) / threshold;
                    } else {
                        tally[1] += 10 * (threshold - scores[lowestIndex]) / threshold;
                    }
                }

                if (scores[lowestIndex] < 5 * threshold && lowestIndex == 0) {
                    if (i < width / 2) {
                        tally[0] += (5 * threshold - scores[lowestIndex]) / (5 * threshold);
                    } else {
                        tally[1] += (5 * threshold - scores[lowestIndex]) / (5 * threshold);
                    }
                }


            }
        }

       /*
       try {
           telemetry.addData("Central Tendency", distance / count);
           telemetry.addData("Count", count);
           telemetry.addData("Selection Index", Math.log10(distance/count/count));
           //sleep(2000);
       }
       catch (ArithmeticException e){
           telemetry.addData("POints", "None Found");
       }
       telemetry.update();
       */


        //Score ranges from small decimals (e.g. 0.0001) to thousands, so logarithmic function
        //makes the scores slightly easier to digest, but keeps the comparisons the same
        //String[] array = {"Yellow: " + yCount, "White: " + wCount};
        //sample[0] /= wCount;
        //sample[1] /= wCount;
        //sample[2] /= wCount;
        //tally[1] = centerX / yCount;
        //tally[2] = centerY / yCount;
        //tally[3] = lowestScore;
        return tally;
    }

    public int findSmallestIndexOfArray(double[] array) throws InterruptedException{
        int smallestIndex = -1;
        double smallest = Double.MAX_VALUE;
        for (int i = 0; i < array.length; i++) {
            if (array[i] < smallest) {
                smallest = array[i];
                smallestIndex = i;
            }
        }
        return smallestIndex;
    }

    public double getDistance(double x1, double x2, double y1, double y2) throws InterruptedException {
        return Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2));
    }

    public double findVectorDifference(double[] vector1, double[] vector2) throws InterruptedException {
        double sum = 0;
        if (vector1.length != vector2.length) {
            return -1;
        }
        for (int i = 0; i < vector1.length; i++) {
            sum += (Math.pow(vector1[i] - vector2[i], 2));
        }
        return Math.sqrt(sum);
    }

    //LinearOpMode is an abstract class that requires the implementation of runOpMode; however, this
    //a class meant to hold Vuforia methods and not meant to be run from the RC.
    //It is important to extend LinearOpMode since it gives us the opModeIsActive() method.
    @Override
    public void runOpMode() throws InterruptedException {
        throw new UnsupportedOperationException();
    }

    public double[] getAverageHSV(Bitmap bm_img) throws InterruptedException {
        Color cur_color = null;
        int cur_color_int, rgb[] = new int[3];
        float hsv[] = new float[3];
        int hueMax = 0;
        double whiteScore = 0;

        int width = bm_img.getWidth();
        int height = bm_img.getHeight();

        double centerX = 0;
        double centerY = 0;

        double count = 0;

        double distance = 0;

        double[] hsvSample = {0, 0, 0};
        for (int i = (int) (width * 0.25); i < (int) (width * 0.75); i = i + 4) {
            for (int j = (int) (height * 1.0 / 3); j < height; j = j + 4) {
                cur_color_int = bm_img.getPixel(i, j);
                rgb[0] = cur_color.red(cur_color_int);
                rgb[1] = cur_color.green(cur_color_int);
                rgb[2] = cur_color.blue(cur_color_int);

                Color.RGBToHSV(rgb[0], rgb[1], rgb[2], hsv);

                double hue = hsv[0];
                double sat = hsv[1];
                double val = hsv[2];

                hsvSample[0] += hue;
                hsvSample[1] += sat;
                hsvSample[2] += val;
                count++;


            }
        }

        hsvSample[0] = hsvSample[0] / count;
        hsvSample[1] = hsvSample[1] / count;
        hsvSample[2] = hsvSample[2] / count;

        return hsvSample;
    }

    public double getCentralWhite(Bitmap bm_img) throws InterruptedException {
        Color cur_color = null;
        int cur_color_int, rgb[] = new int[3];
        float hsv[] = new float[3];
        int hueMax = 0;
        double whiteScore = 0;

        int width = bm_img.getWidth();
        int height = bm_img.getHeight();

        double centerX = 0;
        double centerY = 0;

        double count = 0;

        double distance = 0;

        for (int i = (int) (width * 0.25); i < (int) (width * 0.75); i = i + 4) {
            for (int j = (int) (height * 1.0 / 3); j < height; j = j + 4) {
                cur_color_int = bm_img.getPixel(i, j);
                rgb[0] = cur_color.red(cur_color_int);
                rgb[1] = cur_color.green(cur_color_int);
                rgb[2] = cur_color.blue(cur_color_int);

                Color.RGBToHSV(rgb[0], rgb[1], rgb[2], hsv);

                double hue = hsv[0];
                double sat = hsv[1];

                if (sat <= 0.15 && sat >= 0.04 && !(hue >= 20 && hue <= 80)) {
                    count++;
                    centerX += i;
                    centerY += j;
                }


            }
        }

        return count;
    }

    public String getCentralYellow(Bitmap bm_img) throws InterruptedException {

        Color cur_color = null;
        int cur_color_int, rgb[] = new int[3];
        float hsv[] = new float[3];
        int hueMax = 0;
        double whiteScore = 0;

        int width = bm_img.getWidth();
        int height = bm_img.getHeight();

        double centerX = 0;
        double centerY = 0;

        double count = 0;

        double distance = 0;

        for (int i = (int) (width * 0.25); i < (int) (width * 0.75); i = i + 4) {
            for (int j = (int) (height * 1.0 / 3); j < height; j = j + 4) {
                cur_color_int = bm_img.getPixel(i, j);
                rgb[0] = cur_color.red(cur_color_int);
                rgb[1] = cur_color.green(cur_color_int);
                rgb[2] = cur_color.blue(cur_color_int);

                Color.RGBToHSV(rgb[0], rgb[1], rgb[2], hsv);

                double hue = hsv[0];
                double sat = hsv[1];

                if (hue <= 50 && hue >= 30 && sat >= 0.80) {
                    count++;
                    centerX += i;
                    centerY += j;
                }


            }
        }

        return getDistance(centerX / count, 565, centerY / count, 365) + " " + centerX / count + " " + centerY / count;

    }

    public String getBestThreePlaceMatch(double[] scores) throws InterruptedException {
        if (scores.length != 3) {
            return null;
        }
        if (scores[0] > scores[1] && scores[0] > scores[2]) {
            return "RIGHT";
        } else if (scores[1] > scores[2] && scores[1] > scores[0]) {
            return "MIDDLE";
        } else if (scores[2] > scores[1] && scores[2] > scores[0]) {
            return "LEFT";
        }
        return "TIE";
    }

    public String getBestThreePlaceMatchExtended(double[] scores) throws InterruptedException {
        if (scores.length != 3) {
            return null;
        }
        if (scores[0] > scores[1] && scores[0] > scores[2]) {
            return "RIGHT " + ((scores[0] / (scores[1] + scores[2] + scores[0])) * 100);
        } else if (scores[1] > scores[2] && scores[1] > scores[0]) {
            return "MIDDLE " + ((scores[1] / (scores[0] + scores[2] + scores[1])) * 100);
        } else if (scores[2] > scores[1] && scores[2] > scores[0]) {
            return "LEFT " + ((scores[2] / (scores[1] + scores[0] + scores[2])) * 100);
        }
        return "TIE";
    }

    public double findCosineSimilarityOfVectors(double[] v1, double[] v2) throws InterruptedException {
        if (v1.length != v2.length) {
            return -1;
        }
        double a = 0;
        double b = 0;
        double c = 0;
        for (int i = 0; i < v1.length; i++) {
            a += v1[i] * v1[i];
            b += Math.pow(v1[i], 2);
            c += Math.pow(v2[i], 2);
        }
        Math.sqrt(b);
        Math.sqrt(c);
        return a / (b * c);
    }

    public double[] getCIELABOfMiddlePoint(Bitmap bm_img) throws InterruptedException {
        Color cur_color = null;
        double cur_color_int, rgb[] = new double[3];
        float hsv[] = new float[3];
        int hueMax = 0;
        double whiteScore = 0;

        int width = bm_img.getWidth();
        int height = bm_img.getHeight();

        double centerX = 0;
        double centerY = 0;

        double count = 0;

        double distance = 0;

       /*
       for (int i = (int)(width * 0.25); i < (int)(width * 0.75); i = i + 4){
           for (int j = (int)(height * 1.0/3); j < height; j = j + 4){
           */
        cur_color_int = bm_img.getPixel(width / 2, height / 2);
        rgb[0] = cur_color.red((int) cur_color_int);
        rgb[1] = cur_color.green((int) cur_color_int);
        rgb[2] = cur_color.blue((int) cur_color_int);

        double[] CIELAB = ColorSpaceConvertor.RGVtoCIELAB(rgb);

        double hue = hsv[0];
        double sat = hsv[1];
       /*
               if (hue <= 50 && hue >= 30 && sat >= 0.80){
                   count++;
                   centerX += i;
                   centerY += j;
               }


           }
       }
       */

        return CIELAB;

    }

    public double[] classifyAsGoldOrMineral(Bitmap bm_img) throws InterruptedException {
        Color cur_color = null;
        double cur_color_int, rgb[] = new double[3];
        float hsv[] = new float[3];
        int hueMax = 0;
        double whiteScore = 0;

        int width = bm_img.getWidth();
        int height = bm_img.getHeight();

        double centerX = 0;
        double centerY = 0;

        double count = 0;

        double distance = 0;


        //70 -40 40 Cube
        //20 -80 -80 Sphere
        double[] gold = {70, -40, 40};
        double[] mineral = {20, -80, -80};

        double[] scores = {0, 0, 0, 0};
        int side = 0;
        for (int i = 0; i < (int) (width) ; i = i + 4) {
            for (int j = 0; j < height ; j = j + 4) {
                cur_color_int = bm_img.getPixel(i, j);

                rgb[0] = cur_color.red((int) cur_color_int);
                rgb[1] = cur_color.green((int) cur_color_int);
                rgb[2] = cur_color.blue((int) cur_color_int);


                double[] CIELAB = ColorSpaceConvertor.RGVtoCIELAB(rgb);

                double v1 = findVectorDifference(CIELAB, gold);
                double v2 = findVectorDifference(CIELAB, mineral);

                if (i >= width / 2){
                    side = 1;
                }
                if (v1 < v2 && v1 < 5){
                    scores[0 + side * 2] += 100;
                }
                else if (v1 < v2 && v1 < 15){
                    scores[0 + side * 2] += 10;
                }
                else if (v1 < v2 && v1 < 50){
                    scores[0 + side * 2]++;
                }
                if (v1 > v2 && v2 < 2){
                    scores[1 + side * 2] += 100;
                }
                else if (v1 > v2 && v2 < 5){
                    scores[1 + side * 2] += 10;
                }
                else if (v1 > v2 && v2 < 10){
                    scores[1 + side * 2]++;
                }

            }
        }


        return scores;

    }

    public String determineLocationOfGold(Bitmap bm) throws InterruptedException{
        double[] scores = classifyAsGoldOrMineral(bm);
        if (scores[0] < 100 && scores[2] < 100){
            return "RIGHT";
        }
        if (scores[0] > 300 && scores[0] > scores[2]){
            return "MIDDLE";
        }
        if (scores[2] > 300 && scores[0] < scores[2]){
            return "LEFT";
        }
        if (Math.abs(scores[2] - scores[0]) < 123){
            return "RIGHT";
        }
        if (scores[2] > scores[0]){
            return "LEFT";
        }
        if (scores[0] > scores[2]){
            return "MIDDLE";
        }
        return "UNKNOWN";
    }
}



