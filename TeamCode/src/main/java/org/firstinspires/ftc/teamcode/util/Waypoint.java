package org.firstinspires.ftc.teamcode.util;

/**
 * Simple object to be used by {@link Path} that contains the basic data for a bots position.
 * @author YesItIsEvan
 */
public class Waypoint {

    double X = 0;
    double Y = 0;
    double degrees = 0;
    double minimumTime = 0;

    public Waypoint(double x , double y , double heading_degrees){
        X=x;
        Y=y;
        degrees = heading_degrees;
    }
    public Waypoint(double x , double y , double heading_degrees, double minTime){
        X=x;
        Y=y;
        degrees = heading_degrees;
        minimumTime = minTime;
    }

    public double getX(){
        return X;
    }

    public double getY(){
        return Y;
    }

    public double getHeading(){
        return degrees;
    }
}
