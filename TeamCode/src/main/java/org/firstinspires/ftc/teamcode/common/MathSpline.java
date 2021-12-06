package org.firstinspires.ftc.teamcode.common;

public class MathSpline {
    double alpha = 0; //final x
    double beta = 0; //final y
    double theta = 0; //angle of arc
    double dLeft = 0; //arc length left
    double dRight = 0; //arc length right
    double radius = 0; //radius of robot's COM
    double pLeft = 0; //power of left wheel
    double pRight = 0; //power of right wheel

    Constants constants = new Constants();

    //Variables for Math
    double midD = constants.horizontalDistanceOdo;
    double insideAcos = 0;
    double radiusLeft = 0;
    double radiusRight = 0;

    //Input the Final Position
    public void setFinalPose(double xPose, double yPose){
        alpha = xPose;
        beta = yPose;
    }

    //Calculate the needed variables
    public double returnRadius(){
        radius = ((alpha * alpha * beta) + (beta * beta * beta)) / (2 * alpha * beta);
        return radius;
    }
    public double returnLeftRadius(){
        radius = returnRadius();

        if (alpha > 0)
            radiusLeft = radius + midD;
        else
            radiusLeft = radius - midD;

        return radiusLeft;
    }
    public double returnRightRadius(){
        radius = returnRadius();

        if (alpha > 0)
            radiusRight = radius - midD;
        else
            radiusRight = radius + midD;

        return radiusRight;
    }
    public double returnTheta(){
        radius = returnRadius();

        insideAcos = (-(alpha * alpha) - (beta * beta)) / (2 * radius);
        theta = Math.acos(insideAcos + 1);

        return theta;
    }

    //Return Left and Right Distance for each "Side"
    public double returnLDistance(){
        //radius
        radius = ((alpha * alpha * beta) + (beta * beta * beta)) / (2 * alpha * beta);

        //left radius
          if (alpha > 0)
            radiusLeft = radius + midD;
         else
            radiusLeft = radius - midD;

        //theta
        insideAcos = (-(alpha * alpha) - (beta * beta)) / (2 * radius * radius); //extra radius
        double inside = insideAcos + 1;
        theta = Math.acos(inside);

        dLeft = returnLeftRadius() * returnTheta();

        return dLeft; //dLeft
    }
    public double returnRDistance(){
        //radius
        radius = ((alpha * alpha * beta) + (beta * beta * beta)) / (2 * alpha * beta);

        //left radius
         if (alpha > 0)
            radiusRight = radius - midD;
         else
            radiusRight = radius + midD;


        //theta
        insideAcos = (-(alpha * alpha) - (beta * beta)) / (2 * radius * radius); //extra radius
        double inside = insideAcos + 1;
        theta = Math.acos(inside);

        dRight = returnRightRadius() * returnTheta();

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
