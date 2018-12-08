package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.ArrayList;

//@Autonomous
public class RangeSensorTest extends AutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        ElapsedTime timer = new ElapsedTime();
        String output = "";
        ArrayList<Double> ranges = new ArrayList<>();
        ArrayList<Double> times = new ArrayList<>();
        LinearRegressionTest l = null;
        double predicted = 0;
        double[] read = new double[3];
        timer.reset();
        ranges.add(0.0);
        times.add(0.0);;
        while (opModeIsActive()){


            if (ranges.size() != 0 && l != null){
                do{
                    predicted = l.predict(timer.milliseconds());
                    read = getReasonableRangeTest();
                } while (Math.abs(read[0] - predicted) > 200);

                ranges.add(read[0]);
                times.add(timer.milliseconds());


                telemetry.addData("Predicted", predicted);
            }
            l = new LinearRegressionTest(times, ranges);
            telemetry.addData("Read", read[0]);
            telemetry.addData("Diff", read[1]);
            telemetry.addData("Prev", read[2]);
            double percentError = Math.abs(predicted - read[0]) / read[0] * 100;
            telemetry.addData("% Error of Guess", percentError);
            telemetry.addData("Regression", l.toString());
            //telemetry.addData("Predicted", l.predict(timer.milliseconds()));
            telemetry.update();
            output += ("Reading: " + read[0] + "\t Predicted: " + predicted  + "\t % Error:" + percentError + "\n");
            String filename = "RangeOutputTest.txt";
            File file = AppUtil.getInstance().getSettingsFile(filename);
            ReadWriteFile.writeFile(file, output);
            sleep(100);
        }
    }
}
