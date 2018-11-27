    package org.firstinspires.ftc.teamcode;

//Basic tool for experimentation with color sensor and potential CV
//Meant to convert RGB color space into CIELAB color space
// (although functionality for other color spaces can be added)
//The problem with RGB color space is that it is difficult to
//tell how similar 2 colors are based off of their RGB values
//CIELAB color space more matches how the human eye detects
//similarities between colors

    import android.opengl.Matrix;

    //In CIELAB color space, L represents how white or black an object is
//a represents how red or green and object is and b represents blue or yellow
//A in-depth summary of the CIELAB color system can be found here: https://www.colourphil.co.uk/lab_lch_colour_space.shtml
public final class ColorSpaceConvertor {


    //No Argument private constructor prevents class from being initialized
    //To use methods, type ColorSpaceConvert.methodName()
    private ColorSpaceConvertor(){

    }

    //Colors RGB to XYZ color space (math and numbers here aren't really important to understand)
    //XYZ is basically the "Grand Central Station" of color space. It's easier to go
    // RGB  -> XYZ -> Other Color Space than RGB -> Other Color Space
    public static double[] RGBtoXYZ(double[] RGB) throws InterruptedException{
        double r = RGB[0]/255;
        double g = RGB[1]/255;
        double b = RGB[2]/255;


        if (r > 0.04045){
            r = Math.pow(((r + 0.055 ) / 1.055), 2.4);
        }
        else{
            r /= 12.92;
        }

        if (b > 0.04045){
            b = Math.pow(((b + 0.055 ) / 1.055), 2.4);
        }
        else{
            b /= 12.92;
        }
        if (g > 0.04045){
            g = Math.pow(((g + 0.055 ) / 1.055), 2.4);
        }

        else{
            g /= 12.92;
        }

        r *= 100;
        g *= 100;
        b *= 100;

        double X = r * 0.4124 + g * 0.3576 + b * 0.1805;
        double Y = r * 0.2126 + g * 0.7152 + b * 0.0722;
        double Z = r * 0.0193 + g * 0.1192 + b * 0.9505;

        double[] XYZ = {X,Y,Z};
        return XYZ;
    }



    //CIELAB is a way of identifying colors that is extremely suitable
    //for comparing the similarity between 2 colors
    public static double[] XYZtoCIELAB(double[] XYZ) throws InterruptedException{
        double X = XYZ[0];
        double Y = XYZ[1];
        double Z = XYZ[2];

        X /= 0.95047;
        Y /= 1.000;
        Z /= 1.08883;

        if ( X > 0.008856 ) {
            X = Math.pow(X,1.0/3);
        }
        else{
            X = ( 7.787 * X ) + ( 16.0 / 116 );
        }
        if ( Y > 0.008856 ){
            Y = Math.pow(Y, 1.0/3);
        }
        else{
            Y = ( 7.787 * Y ) + ( 16.0 / 116 );
        }
        if ( Z > 0.008856 ) {
            Z = Math.pow(Z, 1.0/3);
        }
        else{
            Z = ( 7.787 * Z ) + ( 16.0 / 116 );
        }
        double CIE_L = ( 116 * Y ) - 16;
        double CIE_a = 500 * (X - Y);
        double CIE_b = 200 * (Y - Z);

        double[] CIELAB = {CIE_L * 100.0 / 360.0, CIE_a * 200.0 / 360.0 - 100, CIE_b * 200.0 / 360.0 - 100};

        return CIELAB;
    }

    public static double[] RGVtoCIELAB(double[] RGB) throws InterruptedException{
        return XYZtoCIELAB(RGBtoXYZ(RGB));
    }

    //CIELAB Color Space allows the perceivable difference to by calculated
    //by finding (essentially) the vector difference between 2 CIELAB representations
    //A difference value of ~5 is considered a noticeable difference by an average person
    //A difference value of ~2.3 is considered noticeable for perceptive individuals
    public static double CalculateCIELABSimilarity(double[] CIELAB1, double[] CIELAB2){
        return Math.sqrt(Math.pow(CIELAB2[0] - CIELAB1[0],2) + Math.pow(CIELAB2[1] - 4*CIELAB1[1],2) + Math.pow(CIELAB2[2] - CIELAB1[2],2));
    }

    //Sometimes RGB color sensor is weird and returns values > 255
    public static double[] capRGB(double[] RGB) throws InterruptedException{
        if (RGB.length != 3){
            return RGB;
        }
        if (RGB[0] > 255){
            RGB[0] = 255;
        }
        if (RGB[1] > 255){
            RGB[1] = 255;
        }
        if (RGB[2] > 255){
            RGB[2] = 255;
        }
        return RGB;
    }



}
