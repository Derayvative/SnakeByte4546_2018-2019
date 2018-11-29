package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.Arrays;

@Autonomous
public class CentralPointCIELABDetect extends AutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        //GoldDetectorVuforia GDV = new GoldDetectorVuforia(true);

        waitForStart();

        while (opModeIsActive()){
          //  Bitmap bm = GDV.getImage();

            //Watermelon
            //Pumpkin
            //telemetry.addData("Mid", Arrays.toString(GDV.getCIELABOfMiddlePoint(bm)));
            telemetry.update();
        }
    }
}
