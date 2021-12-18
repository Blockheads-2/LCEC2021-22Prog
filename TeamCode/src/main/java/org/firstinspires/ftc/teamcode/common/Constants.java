package org.firstinspires.ftc.teamcode.common;

import org.checkerframework.checker.units.qual.C;

public class Constants {

    //Elevator Motor Positions
    public int elevatorPositionDown = 0; //Rest
    public int elevatorPositionTop = 3410; //Extended
    public int elevatorPositionMid = 2700;
    public int elevatorPositionBottom = 1500;
    public int elevatorAcrossDrop = 2500;

    //Capping Servo Positions
    public double capStart = 0.6; //Straight Up Vertical
    public double capPickUp = 0.12; //Touching Floor / Pick Up Capstone
    public double capDrop = 0.38;
    public double capAlmostDrop = 0.44;

    //Drive Train Constants
    public double maxVelocityDT = 2700;
    public double clicksPerInch = 45.285;

    //Degrees Per Inch Auto
    public double degree = 23.47/90;

    //Distance Between Odo and Center
    public double horizontalDistanceOdo = 6.25;
    public double midDistanceOdo = 3.0;
}
