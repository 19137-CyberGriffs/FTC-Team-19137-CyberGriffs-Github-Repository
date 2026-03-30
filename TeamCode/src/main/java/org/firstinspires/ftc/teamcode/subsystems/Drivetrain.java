package org.firstinspires.ftc.teamcode.subsystems;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.*;
import static org.firstinspires.ftc.teamcode.config.Config.Drivetrain.*;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.config.Config;

/**
 * Wraps the motors controlling movement into an easy-to-use drivetrain object.
 * @author Sunny-19-19
 */
public class Drivetrain {

    // DECLARE OBJECTS
    private final DcMotorEx frontLeft;
    private final DcMotorEx frontRight;
    private final DcMotorEx backLeft;
    private final DcMotorEx backRight;
    private final BNO055IMU imu; // Inertial Measurement Unit
    private double headingOffset = 0;

    // LINEAR OP MODE
    // Used in autonomous so that the robot doesn't move
    // If the user has pressed STOP
    private LinearOpMode autonomousOpMode;


    // IGNORE (For motion smoothing)
    private boolean smoothMotion;
    private boolean slowMotion;
    private boolean fieldCentricControl;

    double driveSmoothed, strafeSmoothed, turnSmoothed;
    double drivePrev = 0, strafePrev = 0, turnPrev = 0;



    //  -- CONSTRUCTOR --  //
    public Drivetrain(HardwareMap hardwareMap) {


	   // Assign each motor variable its respective Motor from the hardwareMap
	   // Refer to the readme (1.1) for more info about the hardwareMap
	   frontLeft = (DcMotorEx) hardwareMap.get(DcMotor.class, "frontLeft");
	   frontRight = (DcMotorEx) hardwareMap.get(DcMotor.class, "frontRight");
	   backLeft = (DcMotorEx) hardwareMap.get(DcMotor.class, "backLeft");
	   backRight = (DcMotorEx) hardwareMap.get(DcMotor.class, "backRight");

	   // Set the direction of the motors (FORWARD, REVERSE)
	   // To test direction: disable the encoder, set all motor's power to 1.
	   // All motors should move forward with respect to the robot.
	   backLeft.setDirection(DcMotor.Direction.FORWARD);
	   frontLeft.setDirection(DcMotor.Direction.FORWARD);
	   frontRight.setDirection(DcMotor.Direction.REVERSE);
	   backRight.setDirection(DcMotor.Direction.REVERSE);

	   // The ZeroPowerBehavior refers to how the motor will behave when the power
	   // is set to 0, BRAKE: Motor will resist movement. FLOAT: Motor will move freely.
	   frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
	   frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
	   backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
	   backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

	   // Initialize IMU
	   imu = hardwareMap.get(BNO055IMU.class, "imu");
	   BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
	   parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
	   imu.initialize(parameters);

	   // Set motion adjustment
	   smoothMotion = false;
	   slowMotion = false;

	   // Reset all the encoders in case any involuntary movement happened during setup.
	   // readme (1.3)
	   resetEncoders();
    }

    // Drive the drivetrain using power for each variable ranging from -1 to 1
    // Range function constrains a variable to a certain range. eg, Range.clip(2, -1, 1) returns 1.
    public void drive(double drive, double strafe, double turn){
	   drive  = drive  * GLOBAL_SPEED_MULTIPLIER;
	   strafe = strafe * GLOBAL_SPEED_MULTIPLIER;
	   turn   = turn   * GLOBAL_SPEED_MULTIPLIER;

	   if (strafe > -STRAFE_DEADZONE && strafe < STRAFE_DEADZONE){
		  strafe = 0;  // If Strafe is between DeadZone nothing happens
	   }
	   else{
		  // Compact Version of: if strafe > 0 subtract deadZone else add deadZone
		  strafe = strafe > 0 ? strafe - STRAFE_DEADZONE : strafe + STRAFE_DEADZONE;
	   }

	   double fDrive  = drive;
	   double fStrafe = strafe;
	   double fTurn   = turn;

	   if(fieldCentricControl){
		  // Adjust the inputs for field-centric control
		  double heading = Math.toRadians(getHeading()); // Convert IMU angle to radians
		  double temp = drive * Math.cos(heading) - strafe * Math.sin(heading);
		  fStrafe = drive * Math.sin(heading) + strafe * Math.cos(heading);
		  fDrive = temp;
	   }


	   if(smoothMotion){
		  driveSmoothed = (drive  * DX_DRIVE)  + (drivePrev  * DX_DRIVE_SCALE);
		  strafeSmoothed =(strafe * DX_STRAFE) + (strafePrev * DX_STRAFE_SCALE);
		  turnSmoothed =  (turn   * DX_TURN)   + (turnPrev   * DX_TURN_SCALE);

		  drivePrev  = driveSmoothed;
		  strafePrev = strafeSmoothed;
		  turnPrev   = turnSmoothed;

		  fDrive = driveSmoothed;
		  fStrafe = strafeSmoothed;
		  fTurn = turnSmoothed;
	   }

	   if(slowMotion){
		  fDrive *= SLOW_SPEED_MULTIPLIER;
		  fStrafe *= SLOW_SPEED_MULTIPLIER;
		  fTurn *= SLOW_SPEED_MULTIPLIER;
	   }


	   // E-Brake (Stop if no input is received)
	   driveSmoothed  = drive  == 0 ? 0 : driveSmoothed;
	   strafeSmoothed = strafe == 0 ? 0 : strafeSmoothed;
	   turnSmoothed   = turn   == 0 ? 0 : turnSmoothed;


	   // This is the standard mecanum wheels control system
	   // DO NOT MODIFY
	   // Refer to readme (1.2)
	   frontLeft. setPower(Range.clip(fDrive - fStrafe + fTurn, -1, 1));
	   frontRight.setPower(Range.clip(fDrive + fStrafe - fTurn, -1, 1));
	   backLeft.  setPower(Range.clip(fDrive + fStrafe + fTurn, -1, 1));
	   backRight. setPower(Range.clip(fDrive - fStrafe - fTurn, -1, 1));


    }


    // Drives the robot using the encoder. Refer to the readme for more info about encoders
    // The arguments "drive" and "strafe" should be in millimeters
    // readme (1.4)

	/**
	 * Okay, danny did not make this documentation. This method is cooked AF. Feel free to try and fix it, but good luck.
	 * @deprecated What the hell is going on
	 * @author Sunny-19-19
	 */
    public void driveByEncoder(double drive, double strafe, double turn) {


	   // Apply Config multiplier
	   double maxPower = Config.Autonomous.DRIVE_BY_ENCODER_MAX_POWER;
	   double minPower = Config.Autonomous.DRIVE_BY_ENCODER_MIN_POWER;


	   // Convert inputs to encoder ticks
	   drive = millisToTicks(drive);
	   strafe = millisToTicks(strafe);
	   turn = millisToTicks(turn);


	   // Calculate target positions (standard mecanum calculations)
	   int targetFrontLeft = (int) (drive + strafe + turn);
	   int targetFrontRight = (int) (drive - strafe - turn);
	   int targetBackLeft = (int) (drive - strafe + turn);
	   int targetBackRight = (int) (drive + strafe - turn);


	   // Set an initial low power to avoid start kick
	   setPower(minPower);


	   // Reset Encoders
	   resetEncoders();
	   setMode(RUN_TO_POSITION);


	   // Set motor target positions after setting initial power
	   frontLeft.setTargetPosition(targetFrontLeft);
	   frontRight.setTargetPosition(targetFrontRight);
	   backLeft.setTargetPosition(targetBackLeft);
	   backRight.setTargetPosition(targetBackRight);


	   setMode(RUN_TO_POSITION);


	   // Loop until progress reaches or exceeds 1.0 (100% progress)
	   while (autonomousOpMode.opModeIsActive()) {
		  // Calculate progress using absolute values
		  int currentFrontLeft = frontLeft.getCurrentPosition();
		  double progress = Math.min(1.0, Math.abs((double) currentFrontLeft / targetFrontLeft));

		  // Break the loop if progress is 1.0 or greater
		  if (progress >= 1.0) {
			 break;
		  }

		  // Power Curve Function (X=0 to X=1) (progress is X)
		  double skewedProgress = Math.sin(progress * Math.PI);

		  // Calculate power dynamically
		  double power = maxPower * skewedProgress;
		  power = Math.max(power, minPower); // Ensure power is at least minPower

		  // Set motor power
		  frontLeft.setPower(power);
		  frontRight.setPower(power);
		  backLeft.setPower(power);
		  backRight.setPower(power);

		  // Update telemetry
		  autonomousOpMode.telemetry.addData("Progress", progress);
		  autonomousOpMode.telemetry.addData("Power", power);
		  autonomousOpMode.telemetry.update();

		  // Small delay to prevent busy-waiting
		  autonomousOpMode.sleep(10);
	   }

	   resetEncoders();
    }


    // Sets the heading of the motor using the IMU
    public void setHeading(double targetAngle) {
	   // Normalize the target angle to be within -180 to 180 degrees
	   targetAngle = normalizeAngle(targetAngle);

	   // Define constants for turning
	   final double maxTurnPower = Config.Autonomous.MAX_TURN_POWER; // Maximum power during turn
	   final double minTurnPower = Config.Autonomous.MIN_TURN_POWER; // Minimum power to ensure movement
	   final double tolerance =    Config.Autonomous.ANGLE_TOLERANCE; // Degrees within target to stop
	   final double kP =	  Config.Autonomous.KP_CONSTANT; // Proportional control constant

	   // Set motors to RUN_WITHOUT_ENCODER for turning because
	   // The system uses the IMU rather than the encoders
	   setMode(RUN_WITHOUT_ENCODER);

	   while (autonomousOpMode.opModeIsActive()) {
		  // Get the current heading
		  double currentAngle = getHeading();

		  // Calculate the shortest path to the target angle
		  double error = normalizeAngle(targetAngle - currentAngle);

		  // Break the loop if the error is within tolerance
		  if (Math.abs(error) <= tolerance) {
			 break;
		  }

		  // Calculate the power using proportional control
		  double power = kP * error;
		  power = Range.clip(power, -maxTurnPower, maxTurnPower);

		  // Ensure minimum power
		  if (Math.abs(power) < minTurnPower) {
			 power = Math.copySign(minTurnPower, power);
		  }

		  // Set motor power for turning
		  frontLeft.setPower(-power);
		  frontRight.setPower(power);
		  backLeft.setPower(-power);
		  backRight.setPower(power);

		  // Telemetry for debugging
		  autonomousOpMode.telemetry.addData("Target Angle", targetAngle);
		  autonomousOpMode.telemetry.addData("Current Angle", currentAngle);
		  autonomousOpMode.telemetry.addData("Error", error);
		  autonomousOpMode.telemetry.addData("Power", power);
		  autonomousOpMode.telemetry.update();

		  // Small delay to prevent busy-waiting
		  autonomousOpMode.sleep(10);
	   }

	   // Stop the motors
	   setPower(0);

	   // Restore the motor mode to RUN_USING_ENCODER
	   setMode(RUN_USING_ENCODER);
	   setMode(RUN_TO_POSITION);
    }

    // Setup routine for OpMode
    public void initOpMode(){
	   // Disable the encoder since we use power during teleOp
	   setMode(RUN_WITHOUT_ENCODER);
	   // Set power to zero to prevent movement for whatever reason
	   setPower(0);
    }

    // Setup routine for Autonomous
    public void initAuton(LinearOpMode linearOpMode){

	   autonomousOpMode = linearOpMode;

	   // Target Position Must be set before setting mode RUN_TO_POSITION
	   // readme (1.3)
	   frontLeft. setTargetPosition(0);
	   frontRight.setTargetPosition(0);
	   backLeft.  setTargetPosition(0);
	   backRight. setTargetPosition(0);

	   resetEncoders();

	   setMode(RUN_USING_ENCODER);
	   setMode(RUN_TO_POSITION);
	   setPower(0.5);

    }

    // Logs orientation and other data
    public void addGyroTelemetry(Telemetry telemetry){
	   // Read IMU data
	   Orientation orientation;
	   orientation = imu.getAngularOrientation();

	   // Add Data to telemetry object (passed from the OP Mode)
	   telemetry.addData("Heading", orientation.firstAngle);
	   telemetry.addData("Roll",    orientation.secondAngle); // Hopefully always 0
	   telemetry.addData("Pitch",   orientation.thirdAngle);  // Hopefully always 0
	   telemetry.addData("Offset",  headingOffset);
    }

    // Set the mode of the motors
    // readme (1.5)
    private void setMode(DcMotor.RunMode mode){
	   frontLeft. setMode(mode);
	   frontRight.setMode(mode);
	   backLeft.  setMode(mode);
	   backRight. setMode(mode);
    }

    // Sets the power for all the motors power ranges from -1 to 1
    // Generally this would be used only in autonomous
    public void setPower(double power){
	   frontLeft. setPower(Range.clip(power, -1,1));
	   frontRight.setPower(Range.clip(power, -1,1));
	   backLeft.  setPower(Range.clip(power, -1,1));
	   backRight. setPower(Range.clip(power, -1,1));
    }

    // Enable/Disable oriented drive
    public void setFieldCentricControl(boolean active){
	   fieldCentricControl = active;
    }

    // Enable/Disable smooth motion
    public void setSmoothMotion(boolean active){
	   smoothMotion = active;
    }

    // Enable/Disable slow motion
    public void setSlowMotion(boolean active){
	   slowMotion = active;
    }

    // Reset heading
    public void resetHeading(){
	   headingOffset = imu.getAngularOrientation().firstAngle;
    }

    // Constrain the angle to -180 to 180
    private double normalizeAngle(double angle) {
	   while (angle > 180) angle -= 360;
	   while (angle < -180) angle += 360;
	   return angle;
    }

    // Resets the encoders back to Zero
    private void resetEncoders(){
	   setMode(STOP_AND_RESET_ENCODER);
    }

    // Gets the heading, in Degrees
    public double getHeading() {
	   return imu.getAngularOrientation().firstAngle - headingOffset;
    }

    // Converts millimeters to encoder ticks, readme (1.6)
    private int millisToTicks(double millis){
	   // Ticks = millis * (1/((Math.PI*WHEEL_DIAMETER)/TICKS_PER_REVOLUTION))
	   // Use the formula above to recalculate this constant
	   // I use a constant because it uses less CPU
	   return (int)((millis)*(1.64572332501));
    }

}