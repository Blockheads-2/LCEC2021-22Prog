package org.firstinspires.ftc.teamcode.common;


public class MathConstHead {
    double psi = 0; //final x
    double omega = 0; //final y
    double distance = 0;
    double angle = 0;

    double psiSquared = 0;
    double omegaSquared = 0;
    double psiOverOmega = 0;

    Constants constants = new Constants();
    AbsPose absPose = new AbsPose();


    //Input the Final Position

    /* ----Implement once finish abs pose----
    public void setFinalPose(double xPose, double yPose){
        absPose.setAbsDistance(xPose,yPose);

        psi = absPose.returnDistanceX();
        omega = absPose.returnDistanceY();
    }
     */

    public void setFinalPose(double xPose, double yPose){
        psi = xPose;
        omega = yPose;
    }

    public double returnDistance(){
        psiSquared = psi * psi;
        omegaSquared = omega * omega;

        double addPoses = psiSquared + omegaSquared;

        distance = Math.pow(addPoses,0.5);

        return distance;
    }

    public double returnAngle(){
        psiOverOmega = omega/psi;

        angle = Math.atan(psiOverOmega);

        return angle;
    }
}
