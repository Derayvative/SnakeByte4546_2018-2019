package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static java.lang.Thread.sleep;
import static org.firstinspires.ftc.teamcode.RobotConstants.TEAM_MARKER_DOWN_POSITION;
import static org.firstinspires.ftc.teamcode.RobotConstants.TEAM_MARKER_UP_POSITION;


@TeleOp
public class TeleOpMode extends OpMode{

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

    Servo basketServo;
    Servo gateServo;

    boolean basketServoPositionDown = true;
    boolean gateServoPositionDown = true;

    double servoPos;

    //Time-related variables
    ElapsedTime time;
    double mostRecentBPress;
    double mostRecentAPress;

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

        basketServo = hardwareMap.servo.get("basketServo");
        gateServo = hardwareMap.servo.get("gateServo");

        basketServo.setPosition(0.9);
        gateServo.setPosition(0.65);

        //Intake
        //Commented out until Trollbot has an intake
        //IT = hardwareMap.dcMotor.get("IT");
        //TeamMarker.setPosition(TEAM_MARKER_UP_POSITION);

        //Time-Related Variables
        time = new ElapsedTime();
        time.reset();
        mostRecentBPress = 0;
        mostRecentAPress = 0;


    }

    @Override

    // ======= Controls (as Tele-Op is running) =======

    public void loop() {



        // Drive-train:

        if (Math.abs(gamepad1.left_stick_y) > .1) {
            telemetry.addData("Left Power", gamepad1.left_stick_y * halfSpeed);
            FL.setPower(gamepad1.left_stick_y * halfSpeed);
            BL.setPower(gamepad1.left_stick_y * halfSpeed);
        } else {
            telemetry.addData("Left Power", 0);
            FL.setPower(0);
            BL.setPower(0);
        }


        if (Math.abs(gamepad1.right_stick_y) > .1) {
            telemetry.addData("Right Power", gamepad1.right_stick_y * halfSpeed);
            FR.setPower(-gamepad1.right_stick_y * halfSpeed);
            BR.setPower(-gamepad1.right_stick_y * halfSpeed);
        } else {
            telemetry.addData("Right Power", 0);
            FR.setPower(0);
            BR.setPower(0);
        }

        if (time.milliseconds() - mostRecentBPress > 250) {
            if (gamepad1.b && (halfSpeed == 1)) {
                halfSpeed = 0.5;
            } else if ((gamepad1.b) && (halfSpeed == 0.5)) {
                halfSpeed = 1;
            }
            mostRecentBPress = time.milliseconds();
        }

        if (halfSpeed == 0.5){
            telemetry.addData("Half Speed", "True");
        }
        else{
            telemetry.addData("Half Speed", "False");
        }

        if (gamepad2.right_trigger > 0.1){
            middleIntake.setPower(-1);
            outerIntake.setPower(-1);
        }
        else if (gamepad2.right_bumper){
            middleIntake.setPower(-1);
            outerIntake.setPower(0);
        }
        else if (gamepad2.left_trigger > 0.1){
            middleIntake.setPower(1);
            outerIntake.setPower(1);
        }
        else if (gamepad2.left_bumper){
            middleIntake.setPower(1);
            outerIntake.setPower(0);
        }
        else{
            middleIntake.setPower(0);
            outerIntake.setPower(0);
        }

        if (gamepad2.left_stick_y > 0.1){
            lift.setPower(gamepad2.left_stick_y);
            lift2.setPower(-gamepad2.left_stick_y);
        }
        else if (gamepad2.left_stick_y < -0.1){
            lift.setPower(gamepad2.left_stick_y);
            lift2.setPower(-gamepad2.left_stick_y);
        }
        else{
            lift.setPower(0);
            lift2.setPower(0);
        }

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
            */
            basketServo.setPosition(0.4);
            mostRecentAPress = time.milliseconds();
        }
        else if (gamepad2.a && !basketServoPositionDown && time.milliseconds() - mostRecentAPress > 250){
            basketServoPositionDown = true;
            basketServo.setPosition(0.9);
            mostRecentAPress = time.milliseconds();
        }

        if (gamepad2.b && gateServoPositionDown && time.milliseconds() - mostRecentAPress > 250){
            gateServoPositionDown = false;
            gateServo.setPosition(0.25);
            mostRecentAPress = time.milliseconds();
        }
        else if (gamepad2.b && !gateServoPositionDown && time.milliseconds() - mostRecentAPress > 250){
            gateServoPositionDown = true;
            gateServo.setPosition(0.65);
            mostRecentAPress = time.milliseconds();
        }
        if (gamepad2.y){
            basketServoPositionDown = true;
            basketServo.setPosition(0.9);
            gateServoPositionDown = true;
            gateServo.setPosition(0.65);
        }
        if (gamepad2.x){
            gateServoPositionDown = false;
            gateServo.setPosition(0.25);
            try {
                Thread.sleep(300);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            basketServoPositionDown = false;
            basketServo.setPosition(0.4);

        }

        telemetry.addData("Gate Down", gateServoPositionDown);
        telemetry.addData("Basket Down", basketServoPositionDown);

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




}

