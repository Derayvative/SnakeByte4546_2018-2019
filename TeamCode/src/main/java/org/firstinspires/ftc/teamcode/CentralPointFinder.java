package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.ArrayList;
import java.util.Arrays;

//@Autonomous
public class CentralPointFinder extends AutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        //GoldDetectorVuforia GDV = new GoldDetectorVuforia(false);
        initialize();
        String target = "";
        //waitForStart();
        while (!isStarted()){
            //sleep(100);
            //Bitmap bm = GDV.getImage();
            //70 -40 40 Cube
            //20 -80 -80 Sphere
            //double[] array = GDV.classifyAsGoldOrMineral(bm);
            //double[] array = GDV.getCIELABOfMiddlePoint(bm);
            //telemetry.addData("Scores", Arrays.toString(array));
            //telemetry.addData("ID", GDV.determineLocationOfGold(bm));
            /*
            tallyVisionResults(GDV.determineLocationOfGold(bm));
            telemetry.addData("RESULTS", Arrays.toString(vision));
            telemetry.addData("Vision", selectTarget());
            target = selectTarget();
            telemetry.addData("Gyro", getFunctionalGyroYaw());
            //telemetry.addData("Best Match", GDV.getBestThreePlaceMatchExtended(array));
            //telemetry.addData("Num POints", array[3]);
            telemetry.update();
        }
        if (target.equals("LEFT")){
            turnToPosition(-23);
            turnToPosition(-23);
            setPower(0.2);
            sleep(1500);
            moveToRangeBasic(15);
            sleep(500);
            turnToPosition(45);
            turnToPosition(45);
            moveToRangeBasic(20, 45);
            dropTeamMarker();
            turnToPosition(55);
            setPower(-0.2);
            sleep(10000);
        }
        else if (target.equals("MIDDLE")){

            turnToPosition(15);
            turnToPosition(15);

            moveTime(2500, .35);
            //moveToRangePIStraightenToStartAngle(20.0);
            //scoreMarker();
            moveToRangeBasic(23);
            dropTeamMarker();
            //pRightTurn(55);
            //pMoveBackward(10000);
            turnToPosition(75);

            //sleep(1000);

            setPower(-0.45);
            sleep(3000);
            if (getFunctionalGyroYaw() > 60 || getFunctionalGyroYaw() < 30) {
                turnToPosition(55);
            }
            setPower(-0.45);
            sleep(7000);

        }
        else if (target.equals("RIGHT")){
            turnToPosition(0);
            turnToPosition(40);
            turnToPosition(40);
            setPower(-0.2);
            sleep(3000);
            moveToRangeBasic(12);
            turnToPosition(-45);
            turnToPosition(-45);
            moveToRangeBasic(12);
            turnToPosition(48);
            dropTeamMarker();

            sleep(200);
            glideAgainstWallMovingBack();

            /*
            turnToPosition(0);
            turnToPosition(50);
            turnToPosition(50);
            //urnToPositionPI(25);


            setPower(0.37);
            sleep(2500);
            moveToRangeBasic(12);
            turnToPosition(-45);
            turnToPosition(-45);

            //moveToRangePIStraightenToStartAngle(15);
            //scoreMarker();
            moveToRangeBasic(12);
            turnToPosition(48);
            //sleep(1000);
            //setPower(-0.5);
            //sleep(5000);

            //sleep(1000);
            dropTeamMarker();

            sleep(200);
            setPower(-0.45);
            sleep(10000);
            */

        }

    }
}
