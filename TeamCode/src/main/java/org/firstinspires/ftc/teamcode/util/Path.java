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

    /**
     * Modes for how the bot will follow the waypoints
     */
    public enum traceMode {
        /**
         * Straight forward, the bot will stop once it reaches the final waypoint
         */
        STOP_WHEN_COMPLETE,
        /**
         * Once the bot reaches its final waypoint, it will go directly back to the first waypoint
         */
        ROUND_LOOP,
        /**
         * Once the bot reaches its final waypoint, it will follow the path in reverse order and then return to a regular order once it reaches the first waypoint again
         */
        REVERSING_LOOP
    };
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

    /**
     * Speed modes for the robot that are referred to in the same way as the speed modes for the cyclops in Subnautica.
     */
    public enum Ahead {
        /**
         * A slow and controllable speed for careful maneuvers
         */
        SLOW,
        /**
         * Whenever the bot must move at a reasonable pace without too much risk in the event of an error
         */
        STANDARD,
        /**
         * WE ARE NO LONGER COMPROMISING AND ARE GOING FULL {@literal F@&KIN} THROTTLE, HOPE YOU BROUGHT A FULLY CHARGED BATTERY
         */
        FLANK
    };
    private Ahead speedMode = Ahead.STANDARD;

    /**
     * Forms a new path for you to forge. Use the {@code Path.addWaypoint(Waypoint)} method to add new points along the path.
     * @param currentRunTime_s The current runtime of the Op-mode
     * @param hardwareMap Hardware map for the voltage sensor to give you better drive suggestions
     */
    public Path(double currentRunTime_s, HardwareMap hardwareMap){
        robotCurrent = new Pose2D(DistanceUnit.METER,0,0, AngleUnit.DEGREES,0);
        voltage = hardwareMap.get(VoltageSensor.class, "Control Hub");
        runTime = currentRunTime_s;
    }

    /**
     * Forms a new path for you to forge. Use the {@code Path.addWaypoint(Waypoint)} method to add new points along the path.
     * @param current The current Pose2D of the bot for presetting off of (0,0)
     * @param currentRunTime_s The current runtime of the Op-mode
     * @param hardwareMap Hardware map for the voltage sensor to give you better drive suggestions
     */
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
     * @param newWaypoint A new instance of the {@link Waypoint} class for {@link Path} to organize alongside the current waypoints.
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
     * @param followMode The trace mode defined by the {@link traceMode} enum
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
     * @param cm Coordinate tolerance in centimeters
     * @param degrees Rotational tolerance in degrees
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
     * @param drive Drive (forward and backward) tuning coefficient
     * @param strafe Strafe (Crab left and right) tuning coefficient
     * @param turn Turn (Rotate left and right) tuning coefficient
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
     * @param ahead The speed mode defined by the {@link Ahead} enum
     */
    public void setSpeedMode(Ahead ahead){
        speedMode = ahead;
    }

    /**
     * Will set the coefficient that determines the bots overall drive power.
     * <p>
     *     NOT TO BE USED TO SET OVERALL SPEED!
     * @param power The power multiplier between (0,1)
     */
    public void setPower(double power){
        powerCoe = power;
    }

    /**
     * Gives a suggested drive power after considering the distance to the waypoint, drive power, and speed mode
     * @return Suggested drive power
     */
    public double getSuggestedDrive(){
        return suggestedDrive;
    }

    /**
     * Gives a suggested strafe power after considering the distance to the waypoint, drive power, and speed mode
     * @return Suggested strafe power
     */
    public double getSuggestedStrafe() {
        return suggestedStrafe;
    }

    /**
     * Gives a suggested turn power after considering the distance to the waypoint, drive power, and speed mode
     * @return Suggested turning power
     */
    public double getSuggestedTurn() {
        return suggestedTurn;
    }

    /**
     * Gives an integer value representing the index of the waypoint that the bot is currently targeting.
     * @return The index of the bot's current waypoint inside the waypoint array that {@link Path} stores.
     */
    public int getCurrentWaypoint(){
        return currentWaypoint;
    }

    /**
     * If the bot is set to STOP_WHEN_COMPLETE then this condition will turn false once the bot has reached its final waypoint
     * @return whether the bot is still running the mission
     */
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
