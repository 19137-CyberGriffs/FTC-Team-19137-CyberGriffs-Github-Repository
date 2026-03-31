package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.Odometry;

/**
 * An object used by {@link Odometry} to create virtual points on the play field that can be used to get important relative information from
 * @author YesItIsEvan
 */
public class PointOfInterest {
    private String name;
    private Pose2D pose;

    public PointOfInterest(String name,Pose2D pose) {
        this.name = name;
        this.pose = pose;
    }

    public Pose2D getPose() {return pose;}

    public String getName() {return name;}

    public double distanceFrom(Pose2D pose, DistanceUnit distanceUnit) {
        double difX = this.pose.getX(distanceUnit)-pose.getX(distanceUnit);
        double difY = this.pose.getY(distanceUnit)-pose.getY(distanceUnit);
        return Math.sqrt(difX*difX+difY*difY);
    }

    public double relativeAngleTo(Pose2D pose, DistanceUnit distanceUnit, AngleUnit angleUnit) {
        double difX = pose.getX(distanceUnit)-this.pose.getX(distanceUnit);
        double difY = pose.getY(distanceUnit)-this.pose.getY(distanceUnit);
        if(difY > 0)
            return Math.asin(difX/difY);
        else
            if(difX > 0)
                return Math.asin(difX/difY) - Math.PI/2.0;
            else
                return Math.asin(difX/difY) + Math.PI/2.0;
    }
}
