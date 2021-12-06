package org.firstinspires.ftc.teamcode.common;

public class MathSpline {
    private double alpha; //final x
    private double beta; //final y
    private double theta; //angle of arc
    private double dLeft; //arc length left
    private double dRight; //arc length right
    private double radius; //radius of robot's COM
    private double pLeft; //power of left wheel
    private double pRight; //power of right wheel

    Constants constants = new Constants();

    //Variables for Math
    private final double midD = constants.horizontalDistanceOdo;
    private double insideAcos;
    private double radiusLeft;
    private double radiusRight;

    //Input the Final Position
    public void setFinalPose(double xPose, double yPose){
        alpha = xPose;
        beta = yPose;
    }

    
    //Calculate the needed variables
    public double returnRadius(){
        radius = ((Math.pow(alpha,2) * beta) + Math.pow(beta,3)) / (2 * alpha * beta);
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

        insideAcos = (Math.pow(alpha,2) + Math.pow(beta,2)) / (2 * radius);
        theta = Math.acos(-insideAcos + 1);

        return theta;
    }

    //Return Left and Right Distance for each "Side"
    public double returnLDistance(){
        dLeft = returnLeftRadius() * returnTheta();

        return dLeft;
    }
    public double returnRDistance(){
        dRight = returnRightRadius() * returnTheta();

        return dRight;
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
