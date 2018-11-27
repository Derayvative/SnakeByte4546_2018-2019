package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class AutonomousDepot extends AutoOpMode {
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

        //pLeftTurn(55);
        turnToPosition(-65);
        //turnToPosition(-65);
        turnToPosition(-65);

        image1 = GDV.getImage();
        telemetry.addData("Hello","Hello");
        telemetry.update();

        sleep(300);
        double scoreR = GDV.findCentralYellowness(image1);
        //pLeftTurn(25);
        turnToPosition(-90);
        turnToPosition(-90);
        //turnToPosition(-90);
        sleep(300);
        image2 = GDV.getImage();
        double scoreM = GDV.findCentralYellowness(image2);
        //turnToPosition(-120);
        turnToPosition(-120);
        turnToPosition(-120);
        //pLeftTurn(25);
        sleep(300);
        image3 = GDV.getImage();
        double scoreL = GDV.findCentralYellowness(image3);
        telemetry.addData("L", scoreL);
        telemetry.addData("M", scoreM);
        telemetry.addData("R", scoreR);
        telemetry.update();
        //sleep(1000);
        if (scoreL < scoreM && scoreL < scoreR){
            //PIRightTurn(73);
            // turnToPosition(-25);
            turnToPosition(-25);
            telemetry.addData("SetUp", "GMM");
            telemetry.update();

            //setPower(0.5);
            //sleep(10000);

            //setPower(0.5);
            setPower(0.35);
            sleep(1500);
            moveToRangeBasic(15);
            turnToPosition(40);
            moveToRangeBasic(20);
            //scoreMarker();
            turnToPosition(55);

            //sleep(1000);
            dropTeamMarker();

            //setPower(-0.4);
            //sleep(10000);
            sleep(200);
            glideAgainstWallMovingBack();
        }
        if (scoreM < scoreL && scoreM < scoreR) {
            telemetry.addData("SetUp", "MGM");
            telemetry.update();
            //PIRightTurn(113);
            turnToPosition(15);
            turnToPosition(15);

            moveTime(2000, .35);
            //moveToRangePIStraightenToStartAngle(20.0);
            //scoreMarker();
            moveToRangeBasic(23);
            pRightTurn(55);
            //pMoveBackward(10000);
            turnToPosition(55);

            //sleep(1000);
            dropTeamMarker();

            sleep(200);
            glideAgainstWallMovingBack();
        }
        if (scoreR < scoreL && scoreR < scoreM){
            telemetry.addData("SetUp", "MMG");
            telemetry.update();
            //PIRightTurn(145);
            turnToPosition(0);
            turnToPosition(40);
            turnToPosition(40);
            //urnToPositionPI(25);

            setPower(0.4);
            sleep(2000);
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
            glideAgainstWallMovingBack();
        }

    }
    */
}


