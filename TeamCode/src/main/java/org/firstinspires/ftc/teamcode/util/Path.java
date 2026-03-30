package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * Path is a class that organizes {@link Waypoint} objects into a single easy to manage object that will give
 * suggestions for drive variables and gives access to information such as where the bot currently is on its mission.
 * @author YesItIsEvan
 */
public class Path {

    private VoltageSensor voltage;
    private double powerCorrectionCoe;

    private Pose2D robotCurrent;
    private Pose2D robotTarget;

    public enum traceMode {STOP_WHEN_COMPLETE, ROUND_LOOP, REVERSING_LOOP};
    private traceMode mode = traceMode.STOP_WHEN_COMPLETE;

    private enum Order {FORWARD, REVERSE};
    private Order order = Order.FORWARD;

    private boolean stop = false;

    private Waypoint[] waypoints = new Waypoint[0];
    private int currentWaypoint = 0;
    private double runTime;
    private double startTime;

    private double cartesianTolerance = 0.05;
    private double headingTolerance = 1;

    private double suggestedDrive = 0;
    private double suggestedStrafe = 0;
    private double suggestedTurn = 0;

    private double powerCoe = 0.6;
    private double driveTuneCoe = 0.8;
    private double strafeTuneCoe = 1.4;
    private double turnTuneCoe = 0.47;

    public enum Ahead {SLOW, STANDARD, FLANK};// Subnautica reference hehe
    private Ahead speedMode = Ahead.STANDARD;

    public Path(double currentRunTime_s, HardwareMap hardwareMap){
        robotCurrent = new Pose2D(DistanceUnit.METER,0,0, AngleUnit.DEGREES,0);
        voltage = hardwareMap.get(VoltageSensor.class, "Control Hub");
        runTime = currentRunTime_s;
    }

    public Path(Pose2D current, double currentRunTime_s, HardwareMap hardwareMap){
        robotCurrent = current;
        voltage = hardwareMap.get(VoltageSensor.class, "Control Hub");
        runTime = currentRunTime_s;
    }

    /**
     * Updates Path with the current robot position to determine if the robot has reached the waypoint and can proceed to the next.
     * This also updates the suggested drive parameters necessary for reaching the waypoint.
     * @param robotPos The current position of the robot
     * @param currentRunTime The current runtime in seconds
     */
    public void update(Pose2D robotPos,double currentRunTime){

        robotCurrent = robotPos;
        robotTarget = new Pose2D(DistanceUnit.METER, waypoints[currentWaypoint].getX(), waypoints[currentWaypoint].getY(),AngleUnit.DEGREES, waypoints[currentWaypoint].getHeading());

        if(robotCurrent.getX(DistanceUnit.METER) > robotTarget.getX(DistanceUnit.METER)-cartesianTolerance && robotCurrent.getX(DistanceUnit.METER) < robotTarget.getX(DistanceUnit.METER)+cartesianTolerance) {
            if (robotCurrent.getY(DistanceUnit.METER) > robotTarget.getY(DistanceUnit.METER) - cartesianTolerance && robotCurrent.getY(DistanceUnit.METER) < robotTarget.getY(DistanceUnit.METER) + cartesianTolerance)
                if (robotCurrent.getHeading(AngleUnit.DEGREES) > robotTarget.getHeading(AngleUnit.DEGREES) - headingTolerance && robotCurrent.getHeading(AngleUnit.DEGREES) < robotTarget.getHeading(AngleUnit.DEGREES) + headingTolerance)
                    if(runTime >= waypoints[currentWaypoint].minimumTime){
                        if(order == Order.FORWARD)
                            currentWaypoint++;
                        else
                            currentWaypoint--;
                        startTime = currentRunTime;
                    }
        }
        runTime = currentRunTime - startTime;

        if(currentWaypoint == waypoints.length || currentWaypoint == -1)
            switch(mode){
                case STOP_WHEN_COMPLETE:
                    stop = true;
                    currentWaypoint--;
                    break;
                case ROUND_LOOP:
                    currentWaypoint = 0;
                    break;
                case REVERSING_LOOP:
                    if(order == Order.FORWARD) {
                        order = Order.REVERSE;
                        currentWaypoint = waypoints.length - 1;
                    }
                    else {
                        order = Order.FORWARD;
                        currentWaypoint = 0;
                    }
            }

        switch(speedMode){
            case SLOW:
                setDriveTune(0.4,0.65,0.35);
                break;
            case STANDARD:
                setDriveTune(0.7,1,0.5);
                break;
            case FLANK:
                setDriveTune(1.7,2,1);
        }

        suggestedDrive = (sigmoid(robotTarget.getY(DistanceUnit.METER)-robotCurrent.getY(DistanceUnit.METER)) * Math.sin(robotCurrent.getHeading(AngleUnit.RADIANS))) + (sigmoid(robotTarget.getX(DistanceUnit.METER)-robotCurrent.getX(DistanceUnit.METER)) * Math.cos(robotCurrent.getHeading(AngleUnit.RADIANS)));
        suggestedStrafe = (sigmoid(robotTarget.getX(DistanceUnit.METER)-robotCurrent.getX(DistanceUnit.METER)) * Math.sin(robotCurrent.getHeading(AngleUnit.RADIANS))) - (sigmoid(robotTarget.getY(DistanceUnit.METER)-robotCurrent.getY(DistanceUnit.METER)) * Math.cos(robotCurrent.getHeading(AngleUnit.RADIANS)));
        suggestedTurn = sigmoid(robotCurrent.getHeading(AngleUnit.RADIANS)-robotTarget.getHeading(AngleUnit.RADIANS));

        powerCorrectionCoe = 12/voltage.getVoltage();

        suggestedDrive *= driveTuneCoe * powerCorrectionCoe * powerCoe;
        suggestedStrafe *= strafeTuneCoe * powerCorrectionCoe * powerCoe;
        suggestedTurn *= turnTuneCoe * powerCorrectionCoe * powerCoe;
    }

    /**
     * Appends a new waypoint at the end of the path
     * <p>
     * Note: Use during initialization
     * @param newWaypoint
     */
    public void addWaypoint(Waypoint newWaypoint){
        Waypoint[] newWaypoints = new Waypoint[waypoints.length+1];
        for(int i=0;i<waypoints.length;i++){newWaypoints[i] = waypoints[i];}
        newWaypoints[waypoints.length] = newWaypoint;
        waypoints = newWaypoints;
    }

    /**
     * Sets how the bot will follow the path.
     * Round loops will have the bot go back to the first waypoint once it has reached the last waypoint,
     * meanwhile reversing loops will have the bot reverse the order in which it follows the path.
     * @param followMode
     */
    public void setTraceMode(traceMode followMode){
        mode = followMode;
    }

    /**
     * Sets the waypoint tolerance of the bot when getting to a target waypoint. If the bot is within these
     * parameters away from a target waypoint then the bot will proceed to the next waypoint. This prevents the bot
     * from getting stuck trying to achieve an exact position.
     * <p>
     * Default: 5 cm, 1 degree
     * </p>
     * @param cm
     * @param degrees
     */
    public void setTolerance(double cm, double degrees){
        cartesianTolerance = cm/100;
        headingTolerance = degrees;
    }

    /**
     * Sets tuning coefficients for suggested drive. Tuning coefficients make sure that the
     * amount of influence that each drive component has respect on another. This makes the bot drive in predictable
     * and straight lines towards its waypoint.
     * <p>
     * Suggestion: This method is not really needed as whenever changing speed modes, the tuning variables already get changed
     * accordingly, so do not use this method unless absolutely necessary
     * @param drive
     * @param strafe
     * @param turn
     */
    public void setDriveTune(double drive, double strafe, double turn){
        driveTuneCoe = drive;
        strafeTuneCoe = strafe;
        turnTuneCoe = turn;
    }

    /**
     * Sets the speed mode of the bot and will adjust the drive tuning accordingly. This also impacts which
     * function Path will use to determine drive power so it is suggested to use this method instead of {@code Path.setPower()}.
     * <p>
     * @param ahead
     */
    public void setSpeedMode(Ahead ahead){
        speedMode = ahead;
    }

    /**
     * Will set the coefficient that determines the bots overall drive power.
     * <p>
     *     NOT TO BE USED TO SET OVERALL SPEED!
     * @param power
     */
    public void setPower(double power){
        powerCoe = power;
    }
    public double getSuggestedDrive(){
        return suggestedDrive;
    }

    public double getSuggestedStrafe() {
        return suggestedStrafe;
    }

    public double getSuggestedTurn() {
        return suggestedTurn;
    }

    /**
     *
     * @return The index of the waypoint the bot is currently going to inside the waypoint array that {@link Path} uses.
     */
    public int getCurrentWaypoint(){
        return currentWaypoint;
    }

    public boolean isRunning(){
        return !stop;
    }

    private double sigmoid(double in){
        switch (speedMode) {
            case SLOW:
                return ((2 / (1 + Math.pow(Math.E, -50 * in))) - 1);
            case FLANK:
                return ((2 / (1 + Math.pow(Math.E, -9 * in))) - 1);
            default:
                return ((2 / (1 + Math.pow(Math.E, -28 * in))) - 1);
        }
    }
}
