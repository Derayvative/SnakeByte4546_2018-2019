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
        moveBackwardEncoderSingle(0.2, 27);
    }
}
