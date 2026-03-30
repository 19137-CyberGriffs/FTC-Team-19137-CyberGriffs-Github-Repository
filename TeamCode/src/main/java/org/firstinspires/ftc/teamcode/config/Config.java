package org.firstinspires.ftc.teamcode.config;

import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;

/*
    The purpose of this class is to have
    all the relevant parameters in one place
 */

public class Config {

    public static class Drivetrain{

	   public final static double STRAFE_DEADZONE = 0.05;
	   public final static double SLOW_SPEED_MULTIPLIER = 0.4;
	   public final static double GLOBAL_SPEED_MULTIPLIER = .8;

	   public final static double DX_DRIVE  = 0.9;
	   public final static double DX_STRAFE = 0.9;
	   public final static double DX_TURN   = 0.8;
	   public final static double DX_DRIVE_SCALE  = 1 - DX_DRIVE;
	   public final static double DX_STRAFE_SCALE = 1 - DX_STRAFE;
	   public final static double DX_TURN_SCALE   = 1 - DX_TURN;

	   public final static double LATERAL_GAIN = 1.142;

    }

    public static class Intake{

	   public final static int ARM_INTAKE_POS = 1170;
	   public final static int ARM_RELEASE_POS = 0;
	   public final static int ARM_ELEVATOR_MOVING_POS = 300;
	   public final static int ARM_HOVER_POS = 1100;
	   public final static int ARM_LOW_ASCENT = 500;
	   public final static int ARM_PUSH = 1800;
	   public final static int ARM_BACK_OUT = 800;

	   public final static double ARM_MAX_POWER = 0.8;
	   public final static double ARM_MIN_POWER = 0.2;

	   public final static double WRIST_INTAKE_POS = 0.73;
	   public final static double WRIST_RELEASE_POS = 0.8;
	   public final static double WRIST_PUSH_POS = 0.83;

	   public final static double CLAW_RIGHT_OPEN_POS = 0.3;
	   public final static double CLAW_RIGHT_CLOSE_POS = 0.52;

	   public final static double CLAW_LEFT_OPEN_POS = 0.7;
	   public final static double CLAW_LEFT_CLOSE_POS = 0.48;

	   public final static double RADIUS_HORIZONTAL = 0.8;
	   public final static double RADIUS_VERTICAL = 0.45;

	   public final static double RADIUS_MAX = 0.5;
	   public final static double RADIUS_MIN = 0;

    }

    public static class Elevator{

	   public final static int ELEVATOR_HIGH = 4200;
	   public final static int ELEVATOR_MID = 2000;
	   public final static int ELEVATOR_LOW = 50;
	   public final static int ELEVATOR_ASCENT = 3400;

	   public final static double ELEVATOR_POWER = 1;

	   public final static double TRAY_INTAKE_POS = 0.47;
	   public final static double TRAY_RELEASE_POS = 0.65; // ++ to push back
	   public final static double TRAY_INIT_POS = 0.3;
    }

    public static class Autonomous{

	   public final static double DRIVE_BY_ENCODER_MAX_POWER = 0.5;
	   public final static double DRIVE_BY_ENCODER_MIN_POWER = 0.1;

	   public final static double MAX_TURN_POWER = 0.6;
	   public final static double MIN_TURN_POWER = 0.15;
	   public final static double ANGLE_TOLERANCE = 0.1;
	   public final static double KP_CONSTANT = 0.01;

    }

    public static class Vision{
	   /* Blue HSV detection range */
	   public final static Scalar BLUE_LOWER_HSV = new Scalar(90,100,50);
	   public final static Scalar BLUE_UPPER_HSV = new Scalar(150,255,255);

	   /* Red HSV detection range */
	   // TODO: Change this values for Red
	   public final static Scalar RED_LOWER_HSV = new Scalar(0,0,0);
	   public final static Scalar RED_UPPER_HSV = new Scalar(0,0,0);

	   /* Yellow HSV detection range */
	   // TODO: Change this values for Yellow
	   public final static Scalar YEL_LOWER_HSV = new Scalar(0,0,0);
	   public final static Scalar YEL_UPPER_HSV = new Scalar(0,0,0);

	   public final static Point TOP_LEFT_CORNER =	new Point(100, 100);
	   public final static Point BOTTOM_RIGHT_CORNER = new Point(200,200);
	   public final static Rect TARGET_AREA = new Rect(TOP_LEFT_CORNER, BOTTOM_RIGHT_CORNER);
    }
}
