package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

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
//@TeleOp(name = "Gold Detector Tensor Flow Test Versatile", group = "Concept")

public class TensorTest extends AutoOpMode {
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

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() throws InterruptedException {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();

        String target = "Unknown";

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();


        if (!isStarted()) {
            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }

            while (!isStarted()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        for (Recognition recognition : updatedRecognitions) {
                            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                telemetry.addData("Gold Coordinates: ", (recognition.getLeft() + recognition.getRight()) / 2.0 + " " + (recognition.getTop() + recognition.getBottom()) / 2.0);
                            } else {
                                telemetry.addData("Silver Coordinates: ", (recognition.getLeft() + recognition.getRight()) / 2.0 + " " + (recognition.getTop() + recognition.getBottom()) / 2.0);
                            }
                        }
                        if (updatedRecognitions.size() > 3){
                            int goldMineralX = -1;
                            int silverMineral1X = -1;
                            int silverMineral2X = -1;
                            double highestGoldCertainty = -1;
                            double highestSilverCertainty = -1;
                            double highestSilverX = -1;
                            double secondHighestSilverCertainty = -1;
                            double secondHighestSilverX = -1;
                            for (Recognition recognition : updatedRecognitions) {
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL) && (recognition.getConfidence() * recognition.getHeight() > highestGoldCertainty)) {
                                    goldMineralX = (int) recognition.getLeft();
                                    highestGoldCertainty = recognition.getConfidence() * recognition.getHeight();
                                } else {
                                    double certainty = recognition.getHeight() * recognition.getConfidence();
                                    if (certainty > secondHighestSilverCertainty){
                                        if (certainty > highestSilverCertainty){
                                            secondHighestSilverCertainty = highestSilverCertainty;
                                            secondHighestSilverX = highestSilverX;
                                            highestSilverCertainty = certainty;
                                            highestSilverX = recognition.getLeft();
                                        }
                                        else{
                                            secondHighestSilverCertainty = certainty;
                                            secondHighestSilverX = recognition.getLeft();
                                        }
                                    }
                                }
                            }
                            if (highestSilverX < secondHighestSilverX){
                                silverMineral1X = (int) highestSilverX;
                                silverMineral2X = (int) secondHighestSilverX;
                            }
                            else{
                                silverMineral2X = (int) highestSilverX;
                                silverMineral1X = (int) secondHighestSilverX;
                            }
                            telemetry.addData("Best GOld", goldMineralX);
                            telemetry.addData("Silver 1", silverMineral1X);
                            telemetry.addData("Silver 2", silverMineral2X);

                            if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                                if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                                    telemetry.addData("Gold Mineral Position", "Left");
                                } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                    telemetry.addData("Gold Mineral Position", "Right");
                                } else {
                                    telemetry.addData("Gold Mineral Position", "Center");
                                }
                            }
                        }
                        if (updatedRecognitions.size() == 3) {
                            int goldMineralX = -1;
                            int silverMineral1X = -1;
                            int silverMineral2X = -1;
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
                                } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                    telemetry.addData("Gold Mineral Position", "Right");
                                } else {
                                    telemetry.addData("Gold Mineral Position", "Center");
                                }
                            }
                        }
                        telemetry.update();

                        if (updatedRecognitions.size() == 2) {
                            int goldMineralX = -1;
                            int silverMineral1X = -1;
                            int silverMineral2X = -1;
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
                            }
                            else if (goldMineralX < silverMineral1X) {
                                telemetry.addData("Gold Mineral Position", "Left");
                                target = "Left";
                            } else {
                                telemetry.addData("Gold Mineral Position", "Middle");
                                target = "Middle";
                            }
                            telemetry.update();
                        }
                    }
                }
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }
        waitForStart();
        /*
        if (target.equals("Right")){
            precisionTurnToPosition(-150);
        }
        if (target.equals("Middle")){
            precisionTurnToPosition(-180);
        }
        if (target.equals("Left")){
            precisionTurnToPosition(150);
        }
        */

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
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

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
}
