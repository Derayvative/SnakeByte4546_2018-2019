package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import java.util.ArrayList;
import java.util.Arrays;

@Autonomous
public class ColorSensorTest extends LinearOpMode {
    ColorSensor CS;
    DistanceSensor DS;

    @Override
    public void runOpMode() throws InterruptedException {
        CS = hardwareMap.colorSensor.get("goldDetector");
        DS = hardwareMap.get(DistanceSensor.class, "goldDetector");
        GoldDetectorColorSensor goldFinder = new GoldDetectorColorSensor(CS, DS);

        waitForStart();

        while (opModeIsActive()) {
            //test
            /*
            double[] RGB = {CS.red(), CS.green(), CS.blue()};
            ColorSpaceConvertor.capRGB(RGB);
            double[] CIELAB = ColorSpaceConvertor.RGVtoCIELAB(RGB);
            telemetry.addData("Gold Similarity", ColorSpaceConvertor.CalculateCIELABSimilarity(CIELAB, RobotConstants.GOLD_CIELAB_VALUES_CLOSE));
            telemetry.addData("Mineral Similarity", ColorSpaceConvertor.CalculateCIELABSimilarity(CIELAB, RobotConstants.MINERAL_CIELAB_VALUES_CLOSE));
            telemetry.update();
            sleep(1000);
            */
            double[] RGB = {CS.red(), CS.green(), CS.blue()};
            ColorSpaceConvertor.capRGB(RGB);
            double[] CIELAB = ColorSpaceConvertor.RGVtoCIELAB(RGB);
            telemetry.addData("CIELAB", Arrays.toString(CIELAB));
            telemetry.addData("Gold Similarity", ColorSpaceConvertor.CalculateCIELABSimilarity(CIELAB, RobotConstants.GOLD_CIELAB_VALUES_CLOSE));
            telemetry.addData("Mineral Similarity", ColorSpaceConvertor.CalculateCIELABSimilarity(CIELAB, RobotConstants.MINERAL_CIELAB_VALUES_CLOSE));
            telemetry.addData("Gold Similarity Far", ColorSpaceConvertor.CalculateCIELABSimilarity(CIELAB, RobotConstants.GOLD_CIELAB_VALUES_FAR));
            telemetry.addData("Mineral Similarity Far", ColorSpaceConvertor.CalculateCIELABSimilarity(CIELAB, RobotConstants.MINERAL_CIELAB_VALUES_FAR));
            telemetry.addData("Range", goldFinder.getRange());
            if (goldFinder.getRange() <= 6.5){
                telemetry.addData("OBJECT", goldFinder.identifyObject());
            }
            telemetry.update();
            sleep(300);
        }
    }
}
