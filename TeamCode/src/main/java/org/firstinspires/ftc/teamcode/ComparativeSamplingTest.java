package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.Arrays;

//@Autonomous
public class ComparativeSamplingTest extends AutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        GoldDetectorColorSensor GD = new GoldDetectorColorSensor(CS, DS);
        waitForStart();
        sleep(5000);
        double[] RGB1 = {CS.red(), CS.green(), CS.blue()};
        ColorSpaceConvertor.capRGB(RGB1);
        double[] sample1 = ColorSpaceConvertor.RGVtoCIELAB(RGB1);
        telemetry.addData("1", "Recorded");
        telemetry.update();
        sleep(4000);
        double[] RGB2 = {CS.red(), CS.green(), CS.blue()};
        ColorSpaceConvertor.capRGB(RGB2);
        double[] sample2 = ColorSpaceConvertor.RGVtoCIELAB(RGB2);
        telemetry.addData("2", "Recorded");
        telemetry.update();
        sleep(4000);
        double[] RGB3 = {CS.red(), CS.green(), CS.blue()};
        ColorSpaceConvertor.capRGB(RGB3);
        double[] sample3 = ColorSpaceConvertor.RGVtoCIELAB(RGB3);
        telemetry.addData("3", "Recorded");
        telemetry.update();
        sleep(2000);
        telemetry.addData("Results", GD.identifyLeastSimilarSample(sample1,sample2,sample3));
        telemetry.addData("1", Arrays.toString(sample1));
        telemetry.addData("2", Arrays.toString(sample2));
        telemetry.addData("3", Arrays.toString(sample3));
        telemetry.addData("12", ColorSpaceConvertor.CalculateCIELABSimilarity(sample1, sample2));
        telemetry.addData("13", ColorSpaceConvertor.CalculateCIELABSimilarity(sample1, sample3));
        telemetry.addData("23", ColorSpaceConvertor.CalculateCIELABSimilarity(sample3, sample2));
        telemetry.update();
        sleep(15000);


    }
}
