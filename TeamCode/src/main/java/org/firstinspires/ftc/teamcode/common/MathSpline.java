package org.firstinspires.ftc.teamcode.common;

import static java.lang.Math.*;

public class MathSpline {
    double alpha = 0; //final x
    double beta = 0; //final y
    double theta = 0; //angle of arc
    double dLeft = 0; //arc length left
    double dRight = 0; //arc length right
    double radius = 0; //radius of robot's COR
    double pLeft = 0; //power of left wheel
    double pRight = 0; //power of right wheel

    Constants constants = new Constants();
    AbsPose absPose = new AbsPose();

    //Variables for Math
    double midD = constants.horizontalDistanceOdo;
    double insideAcos = 0;
    double radiusLeft = 0;
    double radiusRight = 0;

    //Input the Final Position

    /* ---Implement once finsihed dev AbsPose----
    public void setFinalPose(double xPose, double yPose){
        absPose.setAbsDistance(xPose,yPose);

        alpha = absPose.returnDistanceX();
        beta = absPose.returnDistanceY();
    }
     */

    public void setFinalPose(double xPose, double yPose){
        alpha = xPose;
        beta = yPose;
    }

    //Calculate the needed variables
    public double returnLDistance(){
        //radius
        radius = ((alpha * alpha) + (beta * beta)) / (2 * alpha);

        //left radius
        if (alpha > 0)
            radiusLeft =  (radius * 0.9) + midD;
        else {
            radiusLeft = (radius * 0.85) - midD;
        }
        //theta
        insideAcos = (-(alpha * alpha) - (beta * beta)) / (2 * radius * radius);
        double inside = insideAcos + 1;

        theta = Math.acos(inside);

        dLeft = radiusLeft * theta;

        return dLeft; //dLeft
    }
    public double returnRDistance(){
        //radius
        radius = ((alpha * alpha ) + (beta * beta )) / (2 * alpha );

        //right radius
        if (alpha > 0)
            radiusRight = (radius * 0.85) - midD;
        else{
            radiusRight = (radius * 0.9) + midD;
        }

        //theta
        insideAcos = (-(alpha * alpha) - (beta * beta)) / (2 * radius * radius);
        double inside = insideAcos + 1;

        theta = Math.acos(inside);

        dRight = radiusRight * theta;

        return dRight; //dRight
    }

    public double returnLPower(){
        dLeft = returnLDistance();
        dRight = returnRDistance();

        if (dLeft > dRight)
            pLeft = 1;
        else
            pLeft = dLeft/dRight;

        return pLeft;
    }
    public double returnRPower(){
        dLeft = returnLDistance();
        dRight = returnRDistance();


        if (dRight > dLeft)
            pRight = 1;
        else
            pRight = dRight/dLeft;

        return pRight;
    }
}
