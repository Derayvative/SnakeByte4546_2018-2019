package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class AutonomousTeamPark extends AutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

    }

    /*
    GoldDetectorVuforia GDV = new GoldDetectorVuforia();
    Bitmap image1;
    Bitmap image2;
    Bitmap image3;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();

        turnToPositionPI(-55);

        image1 = GDV.getImage();
        telemetry.addData("Hello","Hello");
        telemetry.update();

        sleep(300);
        double scoreR = GDV.findCentralYellowness(image1);
        turnToPosition(-85);
        sleep(300);
        image2 = GDV.getImage();
        double scoreM = GDV.findCentralYellowness(image2);
        turnToPositionPI(-115);
        sleep(300);
        image3 = GDV.getImage();
        double scoreL = GDV.findCentralYellowness(image3);
        telemetry.addData("L", scoreL);
        telemetry.addData("M", scoreM);
        telemetry.addData("R", scoreR);
        telemetry.update();
        sleep(1000);
        if (scoreL < scoreM && scoreL < scoreR){
            pRightTurn(73);
            telemetry.addData("SetUp", "GMM");
            telemetry.update();

            //setPower(0.5);
            //sleep(10000);

            setPower(0.5);
            sleep(1500);
            //moveToRange(5);
            turnToPosition(40);
            moveToRangePIStraightenToStartAngle(20);
            setTeamMarker();
            sleep(1000);
            dropTeamMarker();
            sleep(1000);
            setTeamMarker();
            turnToPosition(-60);
            glideAgainstWallMovingBack();
        }
        if (scoreM < scoreL && scoreM < scoreR) {
            telemetry.addData("SetUp", "MGM");
            telemetry.update();
            pRightTurn(113);
            moveTime(2000, .5);
            moveToRangePIStraightenToStartAngle(20.0);
            setTeamMarker();
            sleep(1000);
            dropTeamMarker();
            sleep(1000);
            setTeamMarker();
            //pMoveBackward(10000);
            turnToPosition(-60);
            glideAgainstWallMovingBack();
        }
        if (scoreR < scoreL && scoreR < scoreM){
            telemetry.addData("SetUp", "MMG");
            telemetry.update();
            pRightTurn(145);
            setPower(0.5);
            sleep(1500);
            moveToRangePIStraightenToStartAngle(15);
            turnToPosition(-45);
            moveToRangePIStraightenToStartAngle(15);
            setTeamMarker();
            sleep(1000);
            dropTeamMarker();
            sleep(1000);
            setTeamMarker();
            moveToRangePIStraightenToStartAngle(10);
            turnToPosition(-60);
            glideAgainstWallMovingBack();
        }
    }
    */
}



