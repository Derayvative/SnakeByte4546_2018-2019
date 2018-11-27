package org.firstinspires.ftc.teamcode;

//This file contains constants used for the Autonomous and TeleOp
//This is easy to use since if we change it in one file, it changes it in all file
//Class is final, meaning it cannot be extended
public final class RobotConstants {

    //Final variables cannot be changed
    public static final double TEAM_MARKER_DOWN_POSITION = 0.7;

    public static final double TEAM_MARKER_UP_POSITION = 0.1;

    //Color Sensor Constants

    //Arrays contain L, A, B values
    //public static final double[] GOLD_CIELAB_VALUES_CLOSE = {190, 80, 120};
    public static final double[] GOLD_CIELAB_VALUES_CLOSE = {90, 21, 30};


    //public static final double[] MINERAL_CIELAB_VALUES_CLOSE = {200, 4, 32};
    public static final double[] MINERAL_CIELAB_VALUES_CLOSE = {115, -2, 34};

    public static final double[] GOLD_CIELAB_VALUES_FAR = {80, 30, 60};

    public static final double[] MINERAL_CIELAB_VALUES_FAR = {200, 200, 200};

    public static final int BUTTON_DELAY_TOGGLE = 250;

}
