package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ReadWriteFile;

import java.io.File;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Scanner;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import static org.firstinspires.ftc.teamcode.RobotConstants.HANG_SERVO_DOWN_POSITION;
import static org.firstinspires.ftc.teamcode.RobotConstants.HANG_SERVO_UP_POSITION;

/**
 * This 2018-2019 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the gold and silver minerals.
 * <p>
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 * <p>
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name = "123MATTRUNTHISAUTONOMOUS")

public class GoldDetectorTensorFlow extends AutoOpMode {
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY = "AXFo96T/////AAABmRzLk+Bot0LXr+j5jBrnBrRkeWb2d4sU3DIruHS37eyy2kGXLhDHifADYmZecTiiPh+BX0eGSA3vIm1tWNg0iskgm1+CL9CbiyFI0IKkje9dWDtoJPtvA6UwDHuSPV/H7rSEkhlrhsmmoY18a9Ahdu76900ohPoU53ENOwOFpexQOLsnNr1hIHlX8uORREAdrfdwJH9aK9Tj4LAVMPAsrRgItHs8J+NZMKrGKiBDy+YeRAlwXz/4uzXO64WR5nUsqB778k3FtGmp2C0HoSf9oQ5X9yU4iZehbLOGEfezLSufi/J9leG8ceDS3PIykPNQo8UaiKNssdNA4weqlVkbwTYXqrM6JEOKhfIpGD+ppPW1";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;
    private VuforiaLocalizer vuforiaCam;
    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    private String[] recentResults = new String[10];

    @Override
    public void runOpMode() throws InterruptedException {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initialize();
        hangServo.setPosition(HANG_SERVO_UP_POSITION);


        int goldMineralX = -1;
        String target = "Unknown";
        String filename = "AutonomousOptions.txt";
        File file = AppUtil.getInstance().getSettingsFile(filename);
        String fileText = ReadWriteFile.readFile(file);
        Scanner reader = new Scanner(fileText);
        double delay = reader.nextDouble();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        //waitForStart();


        /** Wait for the game to begin */
        //telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();


        if (!isStarted            /** Activate Tensor Flow Object Detection. */
                ()) {
            telemetry.addData("Delay", delay);
            if (tfod != null) {
                tfod.activate();
            }

            while (!isStarted()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    sleep(100);
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        int goldCount = 0;
                        int silverCount = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                goldCount++;
                                telemetry.addData("Gold " + goldCount + " Coordinates: ", (recognition.getLeft() + recognition.getRight()) / 2.0 + " " + (recognition.getTop() + recognition.getBottom()) / 2.0);
                            } else {
                                silverCount++;
                                telemetry.addData("Silver " + silverCount + " Coordinates: ", (recognition.getLeft() + recognition.getRight()) / 2.0 + " " + (recognition.getTop() + recognition.getBottom()) / 2.0);
                            }
                        }

                        ArrayList<Integer> removedInts = new ArrayList<>();
                        for (int i = 0; i < updatedRecognitions.size(); i++){
                            Recognition rec = updatedRecognitions.get(i);
                            for (int j = 0; j < updatedRecognitions.size(); j++){
                                Recognition rec2 = updatedRecognitions.get(j);
                                if (Math.abs((rec.getLeft() + rec.getRight())/2 - (rec2.getLeft() + rec.getRight())/2) < 150){
                                    if (rec.getConfidence() < rec2.getConfidence()){
                                        if (!removedInts.contains(i)){
                                            removedInts.add(i);
                                        }
                                    }
                                    if (rec.getConfidence() > rec2.getConfidence()){
                                        if (!removedInts.contains(j)){
                                            removedInts.add(j);
                                        }
                                    }
                                }
                            }
                        }
                        Collections.sort(removedInts, Collections.reverseOrder());
                        for (int i : removedInts){
                            updatedRecognitions.remove(i);
                        }

                        if (updatedRecognitions.size() == 3) {
                            goldMineralX = -1;
                            int silverMineral1X = -1;
                            int silverMineral2X = -1;
                            telemetry.addData("# Object Detected", updatedRecognitions.size());
                            for (Recognition recognition : updatedRecognitions) {
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                    goldMineralX = (int) recognition.getLeft();
                                } else if (silverMineral1X == -1) {
                                    silverMineral1X = (int) recognition.getLeft();
                                } else {
                                    silverMineral2X = (int) recognition.getLeft();
                                }
                            }
                            if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                                if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                                    telemetry.addData("Gold Mineral Position", "Left");
                                    shiftArrayDown(recentResults, "L");
                                } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                    telemetry.addData("Gold Mineral Position", "Right");
                                    shiftArrayDown(recentResults, "R");
                                } else {
                                    telemetry.addData("Gold Mineral Position", "Middle");
                                    shiftArrayDown(recentResults, "M");
                                }
                            }
                        }
                        //telemetry.update();

                        if (updatedRecognitions.size() == 2) {
                            goldMineralX = -1;
                            int silverMineral1X = -1;
                            int silverMineral2X = -1;
                            telemetry.addData("# Object Detected", updatedRecognitions.size());
                            for (Recognition recognition : updatedRecognitions) {
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                    goldMineralX = (int) recognition.getLeft();
                                } else if (silverMineral1X == -1) {
                                    silverMineral1X = (int) recognition.getLeft();
                                } else {
                                    silverMineral2X = (int) recognition.getLeft();
                                }
                            }
                            if (goldMineralX == -1) {
                                telemetry.addData("Gold Mineral Position", "Right");
                                target = "Right";
                                shiftArrayDown(recentResults, "R");
                            }
                            else if (goldMineralX < silverMineral1X) {
                                telemetry.addData("Gold Mineral Position", "Left");
                                target = "Left";
                                shiftArrayDown(recentResults, "L");
                            } else {
                                telemetry.addData("Gold Mineral Position", "Middle");
                                target = "Middle";
                                shiftArrayDown(recentResults, "M");
                            }
                        }
                        if (updatedRecognitions.size() == 1){
                            goldMineralX = -1;
                            int silverMineral1X = -1;
                            telemetry.addData("# Object Detected", updatedRecognitions.size());
                            for (Recognition recognition : updatedRecognitions) {
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                    goldMineralX = (int) (recognition.getLeft() + recognition.getRight() / 2);
                                }
                            }
                            if (goldMineralX < 500 && goldMineralX != -1){
                                telemetry.addData("Gold Mineral Position", "Left");
                                target = "Left";
                                shiftArrayDown(recentResults, "L");
                            }
                            if (goldMineralX > 500 && goldMineralX != -1){
                                telemetry.addData("Gold Mineral Position", "Middle");
                                target = "Middle";
                                shiftArrayDown(recentResults, "M");
                            }
                            if (goldMineralX == -1){
                                telemetry.addData("Gold Mineral Position", "Right (Sketch)");
                                target = "Right";
                                shiftArrayDown(recentResults, "R");
                            }
                        }
                    }
                    
                }
                telemetry.addData("Recent results", Arrays.toString(recentResults));
                target = getTarget(recentResults);
                telemetry.addData("Gold X-Position", goldMineralX);
                telemetry.addData("Target", target);
                telemetry.update();
            }
        }
        if (tfod != null) {
            tfod.shutdown();
        }
        waitForStart();
        hangServo.setPosition(HANG_SERVO_DOWN_POSITION);
        sleep(1500);
        moveBackwardEncoderSingle(0.2, 27);
        sleep(500);
        powerLiftUpP(2000);
        PEncoderSetPowerBackward(15);
        powerLiftDownP(7000);
        powerLiftUpP(900);
        //sleep((int)delay);
        if (target.equals("Right")){

            precisionTurnToPosition(-147.0);

            PEncoderSetPowerForward(1050);

            moveToRangeBasic(14.0, -153);
            precisionTurnToPosition(-225);
            PEncoderSetPowerForward(600);
            moveToRangeBasic(12.0, -225);
            dropTeamMarker();
            precisionTurnToPosition(-120);

            setPower(-0.5);
            sleep(1500);
            if (Math.abs(getFunctionalGyroYaw() - (-135)) > 5){
                turnToPosition(-135);
            }
            //middleIntake.setPower(-0.75);
            //outerIntake.setPower(1);
            setPower(-0.5);
            sleep(1000);
            middleIntake.setPower(0);
            outerIntake.setPower(0);
            sleep(5000);
        }
        if (target.equals("Middle")){
            precisionTurnToPosition(180);

            PEncoderSetPowerForward(4500);

            //moveToRangeBasic(20.0, 180);
            dropTeamMarker();
            precisionTurnToPosition(245);
            setPower(-0.5);
            sleep(1500);
            if (Math.abs(getFunctionalGyroYaw() - (225)) > 5){
                turnToPosition(225);
            }
            //middleIntake.setPower(-0.75);
            //outerIntake.setPower(-1);
            setPower(-0.5);
            sleep(1000);
            middleIntake.setPower(0);
            outerIntake.setPower(0);
            sleep(5000);
        }
        if (target.equals("Left")){
            precisionTurnToPosition(147);
            PEncoderSetPowerForward(1050);

            moveToRangeBasic(14.0, 153);
            precisionTurnToPosition(225);
            PEncoderSetPowerForward(600);
            moveToRangeBasic(12.0, 225);
            dropTeamMarker();
            precisionTurnToPosition(245);
            setPower(-0.5);
            sleep(1500);
            if (Math.abs(getFunctionalGyroYaw() - (225)) > 5){
                turnToPosition(225);
            }
            //middleIntake.setPower(-0.75);
            //outerIntake.setPower(1);
            setPower(-0.5);
            sleep(1000);
            middleIntake.setPower(0);
            outerIntake.setPower(0);
            sleep(5000);
        }

    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);


        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);

    }

    public void initVufPhoneCamera() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforiaCam = ClassFactory.getInstance().createVuforia(parameters);



        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    public void initTfodPhoneCamera() {
        tfod.deactivate();
        tfod.shutdown();
        sleep(2000);
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
}
