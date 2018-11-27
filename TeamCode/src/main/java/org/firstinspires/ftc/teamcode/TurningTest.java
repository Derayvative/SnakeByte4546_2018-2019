package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

@Autonomous
public class TurningTest extends AutoOpMode {
    VuforiaLocalizer vuforia;
    Bitmap image1;
    Bitmap image2;
    Bitmap image3;

    @Override
    public void runOpMode() throws InterruptedException {
        //vuforiaInit();
        initialize();
        waitForStart();
        /*
        precisionTurnToPosition(150.625);
        moveToRangeBasic(14.0);
        precisionTurnToPosition(224.9375);
        moveToRangeBasic(8.0);
        precisionTurnToPosition(248.4375);
    }*//*
        precisionTurnToPosition(-140.0);
        moveToRangeBasic(14.0);
        precisionTurnToPosition(-226.4375);
        moveToRangeBasic(12.0);
        turnToPosition(-110.1875);
        */

        precisionTurnToPosition(178.375);
        moveToRangeBasic(31.0);
        turnToPosition(251.374);
        setPower(-0.5);
        sleep(3000);
        turnToPosition(230);
        sleep(7000);
    }
}
