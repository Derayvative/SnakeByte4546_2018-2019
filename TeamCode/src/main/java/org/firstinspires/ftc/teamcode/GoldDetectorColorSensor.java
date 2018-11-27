package org.firstinspires.ftc.teamcode;

import android.content.Context;
import android.graphics.Color;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.io.IOException;
import java.io.OutputStreamWriter;
import java.util.ArrayList;
import java.util.Arrays;

import static java.lang.Double.isNaN;


public class GoldDetectorColorSensor {
    //Color Sensor attached to the side of the drivetrain used to analyze the minerals + gold
    ColorSensor goldDetector;
    DistanceSensor goldDistanceSensor;
    float[] MineralHSV;
    float[] GoldHSV;

    public GoldDetectorColorSensor(ColorSensor cs, DistanceSensor csDistance) throws InterruptedException {
        goldDetector = cs;
        goldDistanceSensor = csDistance;
    }

    public boolean isObjectInRange(){
        double distance = goldDistanceSensor.getDistance(DistanceUnit.CM);
       if ((distance < 10.5) && (!isNaN(distance))){
           return true;
       }
       return false;
    }

    public String identifyObject() throws InterruptedException{
        if (isObjectInRange()){
            double[] RGB = {goldDetector.red(), goldDetector.green(), goldDetector.blue()};
            ColorSpaceConvertor.capRGB(RGB);
            double[] CIELAB = ColorSpaceConvertor.RGVtoCIELAB(RGB);

            double GoldSimilarity =  ColorSpaceConvertor.CalculateCIELABSimilarity
            (CIELAB, RobotConstants.GOLD_CIELAB_VALUES_CLOSE);

            double MineralSimilarity = ColorSpaceConvertor.CalculateCIELABSimilarity
            (CIELAB, RobotConstants.MINERAL_CIELAB_VALUES_CLOSE);


            if (GoldSimilarity < MineralSimilarity){
                return "GOLD_CLOSE";
            }
            else if (MineralSimilarity  < GoldSimilarity){
                return "MINERAL_CLOSE";
            }
        }
        return "NOTFOUND";
    }

    public double getRange() throws InterruptedException{
        return goldDistanceSensor.getDistance(DistanceUnit.CM);
    }

    private int[] analyzeSample() throws InterruptedException{
        //Index 0 of array represents red, index 1 represents green value, index 2 represents blue
        int[] rgb = new int [3];

        rgb[0] = goldDetector.red();
        rgb[1] = goldDetector.blue();
        rgb[2] = goldDetector.green();

        return rgb;
    }

    public double[] collectYellowSample() throws InterruptedException{

        ArrayList<Integer> Alpha = new ArrayList<>();
        ArrayList<Integer> Red = new ArrayList<>();
        ArrayList<Integer> Blue = new ArrayList<>();
        ArrayList<Integer> Green = new ArrayList<>();

        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (timer.milliseconds() < 15000){
            Alpha.add(goldDetector.alpha());
            Red.add(goldDetector.red());
            Blue.add(goldDetector.blue());
            Green.add(goldDetector.green());
        }

        double[] RGBAvg = {getAvg(Alpha),getAvg(Red), getAvg(Blue), getAvg(Green), getSTDDev(getAvg(Alpha),Alpha), getSTDDev(getAvg(Red), Red), getSTDDev(getAvg(Blue), Blue), getSTDDev(getAvg(Green), Green),};
        return RGBAvg;
    }



    public double getAvg(ArrayList<Integer> a){
        double count = 0;
        int total = 0;
        for (int e : a){
            total += e;
            count++;
        }
        return total/count;
    }

    public double getSTDDev(double avg, ArrayList<Integer> a){
        double count = 0;
        double total = 0;
        for (int e : a){
            total += (Math.pow(e - avg, 2));
            count++;
        }
        return Math.sqrt(total/count);
    }



    public float[] getHSVArray(int r, int g, int b) throws InterruptedException{
        float[] hsv = new float[3];
        Color.RGBToHSV(r, g, b, hsv);
        hsv[0] /= 360.0;
        return hsv;
    }

    public double findHSVAvg() throws InterruptedException{
        int count = 0;
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        double h = 0;
        double s = 0;
        double v = 0;
        while (timer.milliseconds() < 15000){
            h += getHSVArray(goldDetector.red(),goldDetector.blue(),goldDetector.green())[0];
            s += getHSVArray(goldDetector.red(),goldDetector.blue(),goldDetector.green())[1];
            v += getHSVArray(goldDetector.red(),goldDetector.blue(),goldDetector.green())[0];
        }
        return 0;
    }

    public String identifyGoldOrMineral(float[] hsv) throws InterruptedException{
        double vectorSimilarityMineral = Math.sqrt(Math.pow(MineralHSV[0] - hsv[0],2)+ Math.pow(MineralHSV[1] - hsv[1],2) + Math.pow(MineralHSV[2] - hsv[2],2));
        double vectorSimilarityGold = Math.sqrt(Math.pow(GoldHSV[0] - hsv[0],2)+ Math.pow(GoldHSV[1] - hsv[1],2) + Math.pow(GoldHSV[2] - hsv[2],2));
        if (vectorSimilarityGold > vectorSimilarityMineral){
            return "Mineral";
        }
        else if (vectorSimilarityMineral > vectorSimilarityGold){
            return "Gold";
        }
        return null;

    }

    public double identifyLeastSimilarSample(double[] sample1, double[] sample2, double[] sample3){

        double similarity12 = ColorSpaceConvertor.CalculateCIELABSimilarity(sample1, sample2);
        double similarity23 = ColorSpaceConvertor.CalculateCIELABSimilarity(sample3, sample2);
        double similarity13 = ColorSpaceConvertor.CalculateCIELABSimilarity(sample1, sample3);

        if (similarity12 < similarity23 && similarity12 < similarity13){
            return 3;
        }
        if (similarity23 < similarity13 && similarity23 < similarity12){
            return 1;
        }
        if (similarity13 < similarity23 && similarity13 < similarity12){
            return 2;
        }
        return 0;

    }








}
