package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

//@Autonomous
public class TestAuto extends AutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        GoldDetectorColorSensor GD = new GoldDetectorColorSensor(CS,DS);
        setPower(0.2);
        sleep(800);
        setZero();
        pRightTurn(89);
        sleep(200);
        double startAngle = getFunctionalGyroYaw();
        sleep(300);
        setZero();
        setPower(0.15);
        String objectSeen = "";
        while (!objectSeen.equals("GOLD_CLOSE") && !objectSeen.equals("GOLD_FAR") && opModeIsActive()){
            setPowerAndTurn(0.15, simpleStraighten(startAngle));
            double[] RGB = {CS.red(), CS.green(), CS.blue()};
            ColorSpaceConvertor.capRGB(RGB);
            double[] CIELAB = ColorSpaceConvertor.RGVtoCIELAB(RGB);
            //objectSeen = GD.identifyObject();
            if (GD.isObjectInRange()){
                setZero();
                sleep(300);
                objectSeen = GD.identifyObject();
                setPower(0.15);
                sleep(100);
            }
            telemetry.addData("Gold Similarity", ColorSpaceConvertor.CalculateCIELABSimilarity(CIELAB, RobotConstants.GOLD_CIELAB_VALUES_CLOSE));
            telemetry.addData("Mineral Similarity", ColorSpaceConvertor.CalculateCIELABSimilarity(CIELAB, RobotConstants.MINERAL_CIELAB_VALUES_CLOSE));
            telemetry.addData("Gold Similarity Far", ColorSpaceConvertor.CalculateCIELABSimilarity(CIELAB, RobotConstants.GOLD_CIELAB_VALUES_FAR));
            telemetry.addData("Mineral Similarity Far", ColorSpaceConvertor.CalculateCIELABSimilarity(CIELAB, RobotConstants.MINERAL_CIELAB_VALUES_FAR));
            telemetry.addData("OBJECT", objectSeen);
            telemetry.update();
            idle();
        }
        setZero();

    }
}
