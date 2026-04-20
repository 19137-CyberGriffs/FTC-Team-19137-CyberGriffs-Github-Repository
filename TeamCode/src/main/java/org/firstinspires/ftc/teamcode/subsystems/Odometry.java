package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.util.PointOfInterest;

import java.util.NoSuchElementException;

public class Odometry {

    private GoBildaPinpointDriver pinpoint;
    private Pose2D roboPosition;
    private int fullRobotRotations = 0;
    private PointOfInterest[] POIs = new PointOfInterest[0];

    public enum Alliance {BLUE, RED};
    Alliance alliance = Alliance.BLUE;

    //Deprecated
    private double basketX;
    private double basketY;

    public Odometry(Pose2D initialPos, HardwareMap hardwareMap, Alliance alliance){
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.resetPosAndIMU();
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.setOffsets(-84,-168,DistanceUnit.MM);
        pinpoint.setPosition(new Pose2D(DistanceUnit.METER, initialPos.getY(DistanceUnit.METER),initialPos.getX(DistanceUnit.METER),AngleUnit.DEGREES,initialPos.getHeading(AngleUnit.DEGREES)));
    }

    /**
     * Resets odometry and its readings, setting them to zero.
     * <p>
     * THE BOT MUST BE ABSOLUTELY STILL. if the bot moves then everything is screwed. Odometry will freak out, and you would wish that drift was the worst of your worries.
     */
    public void reset(){
        pinpoint.resetPosAndIMU();
    }

    //===Methodology===//

    /**
     * Updates the robots position using odometry
     */
    public void updatePosition(){
        pinpoint.update();// If your questioning this logic, just know that odometry only records heading between -180 and 180 degrees
        if(Math.abs(pinpoint.getHeading(AngleUnit.DEGREES)-roboPosition.getHeading(AngleUnit.DEGREES))>180)
            fullRobotRotations -= (int) (pinpoint.getHeading(AngleUnit.DEGREES)/Math.abs(pinpoint.getHeading(AngleUnit.DEGREES)));
        roboPosition = new Pose2D(DistanceUnit.METER,pinpoint.getPosY(DistanceUnit.METER),pinpoint.getPosX(DistanceUnit.METER),AngleUnit.DEGREES,(360*(fullRobotRotations)+pinpoint.getHeading(AngleUnit.DEGREES)));
    }

    /**
     * @return The position of the robot during the last update
     */
    public Pose2D getRoboPosition(){
        return roboPosition;
    }

    /**
     * Adds a new {@link PointOfInterest} for {@link Odometry} to keep track of in its POI list.
     * @param newPoint the new Point of Interest
     */
    public void addPOI(PointOfInterest newPoint){
        PointOfInterest[] newPOIs = new PointOfInterest[POIs.length+1];
        for(int i=0;i<POIs.length;i++){newPOIs[i] = POIs[i];}
        newPOIs[POIs.length] = newPoint;
        POIs = newPOIs;
    }

    /**
     * Calculates the distance from the center of the robot to the Point of Interest referenced.
     * @param name The name of the Point of Interest
     * @param distanceUnit The unit of the value returned
     * @return The distance from the bot to the POI
     */
    public double getDistanceFromPOI(String name, DistanceUnit distanceUnit){
        for(PointOfInterest point: POIs){
            if(point.getName().equals(name))
                return point.distanceFrom(roboPosition,distanceUnit);
        }
        throw new NoSuchElementException("Referred to Point of Interest does not not exist");
    }

    /**
     * In use for the 2025-2026 Season
     * @return Distance from the robot to the goal
     * @deprecated Was a hard coded version of the newer points of interest. However, it stays here for future references and learning oportunities.
     * @author YesItIsEvan
     */
    public double getDistanceFromGoal(){
        roboPosition = new Pose2D(DistanceUnit.METER,pinpoint.getPosX(DistanceUnit.METER),pinpoint.getPosY(DistanceUnit.METER),AngleUnit.DEGREES,pinpoint.getHeading(AngleUnit.DEGREES));
        double initX = 3.105;	   //Initial x distance from goal
        double initY = 2.04;	  //Initial y distance from goal

        double xComponent = roboPosition.getX(DistanceUnit.METER) - basketX;//double xComponent = initX - roboPosition.getX(DistanceUnit.METER);
        double yComponent = roboPosition.getY(DistanceUnit.METER) - basketY;//double yComponent = initY - roboPosition.getY(DistanceUnit.METER);
        return Math.sqrt(Math.pow(xComponent, 2) + Math.pow(yComponent, 2));
    }

    /**
     * In use for the 2025-2026 Season
     * @param turretCurrentPos
     * @return The amount the turret needs to rotate to face the basket
     * @deprecated Was a Band-Aid fix and doesn't reflect the goals of this class. However, it stays here for future references and learning oportunities.
     * @author YesItIsEvan
     */
    public double getBasketAngle(int turretCurrentPos){
        double turretAngle = roboPosition.getHeading(AngleUnit.DEGREES);//-(turretCurrentPos*(90/1000.0));
        double basketAngle = Math.toDegrees(Math.asin((roboPosition.getY(DistanceUnit.METER)-basketY)/getDistanceFromGoal()));
        return basketAngle + turretAngle;
    }
}