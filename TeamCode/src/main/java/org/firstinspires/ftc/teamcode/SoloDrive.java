package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static java.lang.Thread.sleep;
import static org.firstinspires.ftc.teamcode.RobotConstants.GATE_DOWN_POSITION;
import static org.firstinspires.ftc.teamcode.RobotConstants.GATE_UP_POSITION;
import static org.firstinspires.ftc.teamcode.RobotConstants.HANG_SERVO_DOWN_POSITION;
import static org.firstinspires.ftc.teamcode.RobotConstants.HANG_SERVO_UP_POSITION;
import static org.firstinspires.ftc.teamcode.RobotConstants.TEAM_MARKER_DOWN_POSITION;
import static org.firstinspires.ftc.teamcode.RobotConstants.TEAM_MARKER_UP_POSITION;


@TeleOp
public class SoloDrive extends OpMode{

    // ======= instance variables: =======

    private double halfSpeed;

    // DcMotors - Drive-train


    DcMotor FL;
    DcMotor FR;
    DcMotor BL;
    DcMotor BR;

    DcMotor middleIntake;
    DcMotor outerIntake;

    DcMotor lift;
    DcMotor lift2;

    Servo TeamMarker;

    //Servo basketServo;
    Servo gateServo;
    Servo hangServo;

    boolean basketServoPositionDown = true;
    boolean gateServoPositionDown = true;
    boolean hangServoPositionDown = true;
    boolean timerStarted = false;
    int currSeconds = 0;
    int currMinutes = 0;

    double servoPos;

    //Time-related variables
    ElapsedTime time;
    double mostRecentBPress;
    double mostRecentAPress;
    double mostRecentDPadUpPress;
    boolean xLastPressed = false;
    boolean hangMode;

    double upLifThreshold;
    boolean liftStartedMoving;
    int encGoal;
    int startEnc;
    int liftShift = 0;

    int prevEnc;
    int currEnc = -5;

    //Constants




    // DcMotors - Intake
    // (9/18: needs hardware-map on phone)
    DcMotor IT;

    @Override


    // =======Initialization: Hardware Mapping, variable setting =======
    // functions upon initialization

    public void init() {

        // dynamic variables:


        halfSpeed = 1;

        //Drive-train
        FL = hardwareMap.dcMotor.get("FL");
        FR = hardwareMap.dcMotor.get("FR");
        BL = hardwareMap.dcMotor.get("BL");
        BR = hardwareMap.dcMotor.get("BR");
        middleIntake = hardwareMap.dcMotor.get("middleIntake");
        outerIntake = hardwareMap.dcMotor.get("outerIntake");
        lift = hardwareMap.dcMotor.get("lift");
        lift2 = hardwareMap.dcMotor.get("lift2");

        //TeamMarker = hardwareMap.servo.get("TeamMarker");

        //basketServo = hardwareMap.servo.get("basketServo");
        gateServo = hardwareMap.servo.get("gateServo");
        hangServo = hardwareMap.servo.get("hangServo");

        //basketServo.setPosition(TEAM_MARKER_DOWN_POSITION);
        gateServo.setPosition(GATE_UP_POSITION);
        hangServo.setPosition(HANG_SERVO_DOWN_POSITION);
        hangServoPositionDown = true;

        //Intake
        //Commented out until Trollbot has an intake
        //IT = hardwareMap.dcMotor.get("IT");
        //TeamMarker.setPosition(TEAM_MARKER_UP_POSITION);

        //Time-Related Variables
        time = new ElapsedTime();
        time.reset();
        mostRecentBPress = 0;
        mostRecentAPress = 0;
        mostRecentDPadUpPress = 0;
        upLifThreshold = 0.1;
        hangMode = false;
        liftStartedMoving = false;


    }
    @Override
    public void start(){
        time.reset();
    }

    @Override

    // ======= Controls (as Tele-Op is running) =======

    public void loop() {



        // Drive-train:
        int timeTotal = (int)time.seconds();
        currMinutes = (int)(2.0 - timeTotal / 60.0);
        if (currMinutes < 0){
            currMinutes = 0;
        }
        currSeconds = (120 - timeTotal) % 60;
        if (currSeconds < 0){
            currSeconds = 0;
        }

        if (currSeconds < 10){
            telemetry.addData("Current Time", currMinutes + ":" + "0" + currSeconds);
        }
        else{
            telemetry.addData("Current Time", currMinutes + ":" + currSeconds);
        }


        if (timeTotal > 120){
            telemetry.addData("Current Phase", "Game Finished");
        }
        else if (timeTotal > 90){
            telemetry.addData("Current Phase", "End Game");
        }
        else{
            telemetry.addData("Current Phase", "TeleOp");
        }

        if (Math.abs(gamepad1.left_stick_y) > .1) {
            //telemetry.addData("Left Power", gamepad1.left_stick_y * halfSpeed);
            FL.setPower(gamepad1.left_stick_y * halfSpeed);
            BL.setPower(gamepad1.left_stick_y * halfSpeed);
        } else {
            //telemetry.addData("Left Power", 0);
            FL.setPower(0);
            BL.setPower(0);
        }


        if (Math.abs(gamepad1.right_stick_y) > .1) {
            //telemetry.addData("Right Power", gamepad1.right_stick_y * halfSpeed);
            FR.setPower(-gamepad1.right_stick_y * halfSpeed);
            BR.setPower(-gamepad1.right_stick_y * halfSpeed);
        } else {
            //telemetry.addData("Right Power", 0);
            FR.setPower(0);
            BR.setPower(0);
        }

        /*
        if (time.milliseconds() - mostRecentBPress > 250) {
            if (gamepad1.b && (halfSpeed == 1)) {
                halfSpeed = 0.5;
            } else if ((gamepad1.b) && (halfSpeed == 0.5)) {
                halfSpeed = 1;
            }
            mostRecentBPress = time.milliseconds();
        }

        if (halfSpeed == 0.5){
            telemetry.addData("Half Speed (B)", "True");
        }
        else{
            telemetry.addData("Half Speed (B)", "False");
        }
        */

        if (gamepad1.right_trigger > 0.1){
            middleIntake.setPower(-1);
            outerIntake.setPower(-1);
            if (!gateServoPositionDown) {
                gateServoPositionDown = true;
                gateServo.setPosition(GATE_UP_POSITION);
            }
            //if (!basketServoPositionDown){
            //    basketServoPositionDown = true;
            //    basketServo.setPosition(TEAM_MARKER_DOWN_POSITION);
            //}

        }
        else if (gamepad1.right_bumper){
            middleIntake.setPower(-1);
            outerIntake.setPower(0);
        }
        else if (gamepad1.left_trigger > 0.1){
            middleIntake.setPower(1);
            outerIntake.setPower(1);
            //  if (!gateServoPositionDown){

            //}
        }
        else if (gamepad1.left_bumper){
            middleIntake.setPower(1);
            outerIntake.setPower(0);
        }

        /*
        else if (gamepad1.right_trigger > 0.1){
            outerIntake.setPower(gamepad1.right_trigger / 10.0 + 0.1);
        }
        else if (gamepad1.left_trigger > 0.1){
            outerIntake.setPower(-gamepad1.left_trigger / 10.0 - 0.1);
        }
        */
        else{
            middleIntake.setPower(0);
            outerIntake.setPower(0);
        }

        if (gamepad1.dpad_up){
            lift.setPower(1);
            lift2.setPower(-1);
        }
        else if (gamepad1.dpad_down){
            lift.setPower(-1);
            lift2.setPower(1);
        }
        else{
            lift.setPower(0);
            lift2.setPower(0);
        }
        /*
        if (gamepad2.a && basketServoPositionDown && time.milliseconds() - mostRecentAPress > 250){
            basketServoPositionDown = false;
            /*
            for (double i = 0.9; i >= 0.4; i -= 0.1){
                basketServo.setPosition(i);
                try {
                    Thread.sleep(100);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }

            basketServo.setPosition(TEAM_MARKER_UP_POSITION);
            mostRecentAPress = time.milliseconds();
        }
        else if (gamepad2.a && !basketServoPositionDown && time.milliseconds() - mostRecentAPress > 250){
            basketServoPositionDown = true;
            basketServo.setPosition(TEAM_MARKER_DOWN_POSITION);
            mostRecentAPress = time.milliseconds();
        }
        */

        if (gamepad1.b && gateServoPositionDown && time.milliseconds() - mostRecentAPress > 250 && !gamepad2.start){
            gateServoPositionDown = false;
            gateServo.setPosition(GATE_DOWN_POSITION);
            mostRecentAPress = time.milliseconds();
        }
        else if (gamepad1.b && !gateServoPositionDown && time.milliseconds() - mostRecentAPress > 250 && !gamepad2.start){
            gateServoPositionDown = true;
            gateServo.setPosition(GATE_UP_POSITION);
            mostRecentAPress = time.milliseconds();
        }

        if (gamepad1.x && !xLastPressed) {
            if (hangServoPositionDown) {
                hangServo.setPosition(HANG_SERVO_DOWN_POSITION);
                hangServoPositionDown = false;
            }
            else {
                hangServo.setPosition(HANG_SERVO_UP_POSITION);
                hangServoPositionDown = true;
            }
            xLastPressed = true;

        }
        else if (!gamepad1.x) {
            xLastPressed = false;
        }

        if (gamepad1.left_stick_button){
            initiatePrecisionLift(8000);
        }
        else if (gamepad1.right_stick_button){
            initiatePrecisionLift(0);
        }
        updateLiftPower();
        /*
        if (gamepad2.y){
            basketServoPositionDown = true;
            basketServo.setPosition(TEAM_MARKER_DOWN_POSITION);
            gateServoPositionDown = true;
            gateServo.setPosition(GATE_UP_POSITION);
        }

        if (gamepad2.x){
            gateServoPositionDown = false;
            gateServo.setPosition(GATE_DOWN_POSITION);
            try {
                Thread.sleep(300);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            basketServoPositionDown = false;
    z        basketServo.setPosition(TEAM_MARKER_UP_POSITION);

        }
        */

        /*
        if (gamepad1.dpad_down && time.milliseconds() - mostRecentDPadUpPress > 250 && !hangMode){
            hangMode = true;
            upLifThreshold = 1;
            mostRecentDPadUpPress = time.milliseconds();
        }
        else if (gamepad1.dpad_down && time.milliseconds() - mostRecentDPadUpPress > 250 && hangMode){
            hangMode = false;
            upLifThreshold = 0.1;
            mostRecentDPadUpPress = time.milliseconds();

        }

        if (hangMode){
            telemetry.addData("Restricted Hang Mode", "enabled");
        }
        if (!hangMode){
            telemetry.addData("Restricted Hang Mode", "disabled");
        }
        */

        telemetry.addData("Gate Down", gateServoPositionDown);
        telemetry.addData("Basket Down", basketServoPositionDown);
        telemetry.addData("GP1", gamepad1.toString().substring(gamepad1.toString().indexOf("lx")));
        telemetry.addData("GP2", gamepad2.toString().substring(gamepad2.toString().indexOf("lx")));
        telemetry.addData("Lift Encoder", lift.getCurrentPosition());


        // Intake:

        /*
        if (gamepad1.right_trigger > 0.1) {
            IT.setPower(gamepad1.right_trigger);
        } else if (gamepad1.left_trigger > 0.1){
            IT.setPower(-gamepad1.left_trigger);
        }
        else{
            IT.setPower(0);
        }
        */

        // Half-speed:



    } // end of loop

    public void liftToHeight(int encoderGoal) throws InterruptedException {
        startEnc = getLiftEncoder();
        boolean underTarget;
        if (Math.abs(getLiftEncoder()) < encoderGoal) {
            underTarget = true;
        } else {
            underTarget = false;
        }
        encGoal = encoderGoal;
        liftStartedMoving = true;
    }

    public void updateLiftPower() {

        if ((Math.abs(Math.abs(getLiftEncoder()) - encGoal)) > 10  && liftStartedMoving){

            int error = Math.abs(encGoal - Math.abs(getLiftEncoder()));
            double power = 0.13 + error * 0.35 / 200.0;
            prevEnc = currEnc;
            currEnc = getLiftEncoder();
            if (prevEnc - currEnc == 0){
                power = 0;
                liftStartedMoving = false;
                if (getLiftEncoder() < 150){
                    liftShift = getLiftEncoder();
                }
            }
            telemetry.addData(getLiftEncoder() + " " + encGoal, "Yo");
            if (Math.abs(getLiftEncoder()) > encGoal){
                power *= -1;
                telemetry.addData("Over", "Over");
            }
            lift.setPower(power);
            lift2.setPower(-power);
            telemetry.addData("Error", error);
            telemetry.addData("Distance", Math.abs(getLiftEncoder() - startEnc));
            telemetry.addData("Power", 0.13 + error * 0.35 / 200.0);
            telemetry.update();
        }
        else{
            lift.setPower(0);
            lift2.setPower(0);
            liftStartedMoving = false;
        }
    }



    public void initiatePrecisionLift(int encoderGoal){
        startEnc = getLiftEncoder();
        boolean underTarget;
        if (Math.abs(getLiftEncoder()) < encoderGoal) {
            underTarget = true;
        } else {
            underTarget = false;
        }
        encGoal = encoderGoal;
        liftStartedMoving = true;
    }

    public int getLiftEncoder(){
        return lift.getCurrentPosition() - liftShift;
    }





}

