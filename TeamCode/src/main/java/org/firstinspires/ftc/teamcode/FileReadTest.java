package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.Scanner;
import java.util.concurrent.locks.ReadWriteLock;

//@Autonomous
public class FileReadTest extends AutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        String filename = "AutonomousOptions.txt";
        File file = AppUtil.getInstance().getSettingsFile(filename);
        String fileText = ReadWriteFile.readFile(file);
        Scanner reader = new Scanner(fileText);
        String delay = reader.next();
        telemetry.addData("Delay", delay);
        telemetry.update();
        sleep(5000);
    }
}
