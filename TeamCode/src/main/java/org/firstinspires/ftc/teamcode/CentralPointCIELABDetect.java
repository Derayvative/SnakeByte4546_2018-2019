package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.Arrays;

@Autonomous
public class CentralPointCIELABDetect extends AutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        //GoldDetectorVuforia GDV = new GoldDetectorVuforia(true);
        initialize();
        //
        // waitForStart();

        raiseTeamMarker();
        waitForStart();
        lowerTeamMarker();
        sleep(2000);
    }
}
