package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.ArrayList;
//@TeleOp
public class EasyAutoCreator extends AutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        int index = 0;
        initialize();
        double timeOfPress = 0;
        ArrayList<Integer> methodSequence = new ArrayList<>();
        ArrayList<Double> methodParameters = new ArrayList<>();
        String[] possibleMethods = {"turnToPosition()", "precisionTurnToPosition()", "moveToRangeBasic()"};
        waitForStart();
        while (opModeIsActive() && !gamepad1.right_bumper){
            while (index < 0){
                index+=3;
            }
            telemetry.addData("Method Selected", possibleMethods[index % 3]);
            telemetry.addData("Yaw", getFunctionalGyroYaw());
            telemetry.addData("Range", getRangeReading());
            telemetry.addData("Press A to add method", "Ok");
            telemetry.addData("Press B to remove Method", "Ok");
            telemetry.addData("Press right bumper to activate run mode", "Ok");
            if (gamepad1.dpad_up && System.currentTimeMillis() - timeOfPress > 250){
                index++;
                timeOfPress = System.currentTimeMillis();
            }
            if (gamepad1.dpad_down && System.currentTimeMillis() - timeOfPress > 250){
                index--;
                timeOfPress = System.currentTimeMillis();
            }
            if (gamepad1.a && System.currentTimeMillis() - timeOfPress > 250){

                timeOfPress = System.currentTimeMillis();
                methodSequence.add(index % 3);
               if (index % 3 < 2){
                   methodParameters.add(getFunctionalGyroYaw());
               }
               else{
                   methodParameters.add(getRangeReading());
               }
            }
            if (gamepad1.b && System.currentTimeMillis() - timeOfPress > 250){
                methodParameters.remove(methodParameters.size() - 1);
                methodSequence.remove(methodSequence.size() - 1);
                timeOfPress = System.currentTimeMillis();

            }
            int count = 0;
            String allCode = "";
            for (int method : methodSequence){
                String code = possibleMethods[method].substring(0, possibleMethods[method].length() - 1) + methodParameters.get(count) + ")";
                telemetry.addData("Method", code);
                count++;
                allCode += (code + ";\n");
            }
            String filename = "RecentAuto.txt";
            File file = AppUtil.getInstance().getSettingsFile(filename);
            ReadWriteFile.writeFile(file, allCode);
            telemetry.update();
        }
        int count = 0;
        telemetry.addData("Exiting", "Creation Mode");
        telemetry.update();
        //initialize();
        //sleep(3000);
        /*
        for (int method : methodSequence){
            if (method == 0){

                turnToPosition(methodParameters.get(count));
                sleep(300);

            }
            if (method == 1){

                precisionTurnToPosition(methodParameters.get(count));
                sleep(300);

            }
            if (method == 2){

                moveToRangeBasic(methodParameters.get(count));
                sleep(300);

            }
            count++;
            */
       // }
    }

}
