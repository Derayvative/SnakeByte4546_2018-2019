package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

@Autonomous
public class AutonomousSettingsToggle extends AutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        double delayMS = 0;
        int targetOption = 0;
        boolean prevGPUp = false;
        boolean prevGPDown = false;
        boolean prevAPress = false;
        String filename = "AutonomousOptions.txt";
        File file = AppUtil.getInstance().getSettingsFile(filename);
        String settings = "";
        String[] visionTarget = {"Use Sensor", "Left", "Middle", "Right"};
        waitForStart();
        while (opModeIsActive()){
            if (gamepad1.dpad_up && !prevGPUp){
                delayMS += 1000;
                prevGPUp = true;
            }
            else if (!gamepad1.dpad_up){
                prevGPUp = false;
            }
            if (gamepad1.dpad_down && !prevGPDown){
                if (!(delayMS < 1000)){
                    delayMS -= 1000;
                }
                prevGPDown = true;
            }
            else if (!gamepad1.dpad_down){
                prevGPDown = false;
            }
            /*
            if (gamepad1.a && !prevAPress){
                prevAPress = true;
                targetOption++;
            }
            else if(!gamepad1.a){
                prevAPress = false;
            }
            */

            telemetry.addData(prevGPUp + "", prevGPDown + "");
            telemetry.addData("Delay", delayMS + " ms");
            /*
            telemetry.addData("Vision target", visionTarget[targetOption % 4]);
            */

            settings = delayMS + "";
            if (gamepad1.a){
                ReadWriteFile.writeFile(file, settings);
                telemetry.addData("Settings", "Overwritten");
            }

            telemetry.update();

        }


    }
}
