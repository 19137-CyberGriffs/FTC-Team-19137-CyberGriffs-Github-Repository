/*
Copyright 2026 FIRST Tech Challenge Team 19137

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import static com.qualcomm.robotcore.hardware.Servo.Direction.*;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.Odometry;
import org.firstinspires.ftc.teamcode.util.Path;
import org.firstinspires.ftc.teamcode.util.Waypoint;

import java.util.*;

@Autonomous(name="DecodeAuto")
public class DecodeAuto extends LinearOpMode {

	DecodeTeleOp.Alliance alliance =  DecodeTeleOp.Alliance.BLUE;

	double drive;
	double strafe;
	double turn;
	double powerCoe = 1;

	Drivetrain drivetrain;
	HuskyLens huskyLens;
	double distanceToGoal;

	int magRotationCount = 0;
	int turretTargetPosition = 0;
	boolean liftUp = false;
	boolean runIntake = false;

	HuskyLens.Block[] visibleTags;
	HuskyLens.Block aprilTag;

	double motorPowerCorrectionMultiplier;

	double period = 1000/16.0;
	double beginTime = getRuntime()*1000;
	double currentTime;

	int previousFlywheelPos = 0;
	int currentFlywheelPos = 0;
	double flywheelRPM;
	double targetFlywheelTPS;
	double[] flywheelRPMOverTime = new double[10];

	double timeInRoute = 0;
	boolean robotReachedTarget = false;
	int currentTarget = 0;
	boolean stop = false;

	@Override
	public void runOpMode() throws InterruptedException{

		//Drivetrain Constructors
		Path path = new Path(getRuntime(),hardwareMap);
		drivetrain = new Drivetrain(hardwareMap);
		drivetrain.initOpMode();
		drivetrain.setSmoothMotion(true);

		//Flywheel Constructors
		DcMotorEx flywheelMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "motor"); //Flywheel Motor
		flywheelMotor.setDirection(DcMotor.Direction.REVERSE);
		flywheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); //The flywheel automatically brakes if there isn't power

		//TurretYaw Constructors
		DcMotor turretYaw = hardwareMap.get(DcMotor.class, "turretYaw");   //Turret Rotation Motor
		turretYaw.setTargetPosition(turretTargetPosition);
		turretYaw.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);// Make sure robot turret is at home position when initializing (indicated with mark on the front)
		turretYaw.setMode(DcMotor.RunMode.RUN_TO_POSITION);

		//IntakeMotor Constructors
		DcMotor intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
		intakeMotor.setDirection(DcMotor.Direction.FORWARD);
		intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		//Servo Constructors
		Servo magServo = hardwareMap.get(Servo.class, "magServo"); // Magazine Servo
		magServo.setDirection(FORWARD);

		//Flap Constructor
		Servo hood = hardwareMap.get(Servo.class, "turretFlap");	//Turret flap servo
		hood.setDirection(FORWARD); //Turret is programmed to position, not continuous power (like a motor), instead its like

		//Delivery Method Constructors
		Servo lift = hardwareMap.get(Servo.class, "liftServo");
		lift.setDirection(FORWARD);

		//Sensor Constructors
		VoltageSensor voltage = hardwareMap.get(VoltageSensor.class, "Control Hub");

		//Camera Constructor
		huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
		huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

		//Initializes the position of components
		Odometry odometry = new Odometry(hardwareMap,alliance);
		magServo.setPosition(.127);
		lift.setPosition(0.012);
		turretYaw.setPower(0.9);
		flywheelMotor.setPower(0);
		flywheelMotor.setVelocity(0);
		hood.scaleRange(0,0.6);

		//Patrolling Atkinson's room
		path.setTraceMode(Path.traceMode.ROUND_LOOP);
		path.setTolerance(2,1);
		path.addWaypoint(new Waypoint(0,0,0,5));//Home
		path.addWaypoint(new Waypoint(0.1,0.05,0));
		path.addWaypoint(new Waypoint(0.1,0.05,0));
		path.addWaypoint(new Waypoint(0.8,0.1,0));
		path.addWaypoint(new Waypoint(0.8,0.5,0));
		path.addWaypoint(new Waypoint(1,0.8,0));
		path.addWaypoint(new Waypoint(6,0.8,0));
		path.addWaypoint(new Waypoint(7.4,0.7,90));
		path.addWaypoint(new Waypoint(7.4,7,90));
		path.addWaypoint(new Waypoint(7.4,0.7,90));
		path.addWaypoint(new Waypoint(6,0.8,0));
		path.addWaypoint(new Waypoint(0.8,0.8,0));
		path.addWaypoint(new Waypoint(0.8,0.1,0));
		path.addWaypoint(new Waypoint(0,0,0));//Home
		path.addWaypoint(new Waypoint(-1,-2,0,6));//Forced home
		path.addWaypoint(new Waypoint(-1,-2,0,5));//Reset

		telemetry.addLine("Welcome Aboard Captain,");
		telemetry.addData("All Systems", "Online");
		telemetry.update();
		// Wait for the game to start (driver presses PLAY)
		waitForStart();

		// run until the end of the match (driver presses STOP)
		while (opModeIsActive() && path.isRunning()) {

			// Voltage Correction
			motorPowerCorrectionMultiplier = (12/voltage.getVoltage());

			// Odometry
			odometry.updatePosition();
			path.update(odometry.getRoboPosition(),getRuntime());
			turretTargetPosition = (int)((1000/90.0)*odometry.getBasketAngle(turretYaw.getCurrentPosition())+0.5);
			distanceToGoal = odometry.getDistanceFromGoal();

			// Drivetrain
			drivetrain.drive(path.getSuggestedDrive(), path.getSuggestedStrafe(), path.getSuggestedTurn());

			// Actions to take while reaching waypoint
			runIntake = false;
			liftUp = false;
			path.setPower(0.7);
			path.setSpeedMode(Path.Ahead.SLOW);
			switch (path.getCurrentWaypoint()) {
				case 6: case 8: case 11:
                    path.setSpeedMode(Path.Ahead.STANDARD);
					break;
                case 14:
					path.setPower(0.4);
					break;
			}

			// Flywheel
			currentFlywheelPos = flywheelMotor.getCurrentPosition();
			flywheelRPM = (currentFlywheelPos-previousFlywheelPos)*(1/period)*(60000/28.0);
			previousFlywheelPos = flywheelMotor.getCurrentPosition();

			for(int i=1;i<flywheelRPMOverTime.length;i++)
				flywheelRPMOverTime[i] = flywheelRPMOverTime[i-1];
			flywheelRPMOverTime[0] = flywheelRPM;

			flywheelMotor.setPower(0.0001*Math.abs(targetFlywheelTPS -Arrays.stream(flywheelRPMOverTime).average().getAsDouble())*motorPowerCorrectionMultiplier);
			targetFlywheelTPS = ((81.81818 * Math.pow(distanceToGoal,2) + 191.45455 * distanceToGoal + 1793.63636)*(28/60.0));
			flywheelMotor.setVelocity(0);//targetFlywheelTPS);

			// Camera
			visibleTags = huskyLens.blocks();
			for (HuskyLens.Block aprilTag : visibleTags) {
				if(aprilTag.id == 5 && alliance == DecodeTeleOp.Alliance.RED) {
					turretTargetPosition = turretYaw.getCurrentPosition() + (int)(1.1*(aprilTag.x - 160)+0.5);
				}
				if(aprilTag.id == 1 && alliance == DecodeTeleOp.Alliance.BLUE) {
					turretTargetPosition = turretYaw.getCurrentPosition() + (int)(1.1*(aprilTag.x - 160)+0.5);
				}
			}

			// Turret Limits
			turretTargetPosition = 0;

			if(turretTargetPosition > 1000)
				turretTargetPosition = 1000;
			if(turretTargetPosition < -1000)
				turretTargetPosition = -1000;

			turretYaw.setTargetPosition(turretTargetPosition);

			// Magazine Positions
			if(liftUp) {
				magServo.setPosition(.1248+(magRotationCount*.073));
				runIntake = false;
			}
			else if(runIntake)
				magServo.setPosition(.1099 + (magRotationCount * .073));
			else
				magServo.setPosition(.1248+(magRotationCount*.073));

			//Intake
			if(runIntake)
				intakeMotor.setPower(0.9);

			// Lift
			if(liftUp)
				lift.setPosition(0.35);
			else
				lift.setPosition(0.012);

			// Hood
			hood.setPosition(distanceToGoal*0.2);

			// Telemetry
			telemetry.addData("Current Target ID", path.getCurrentWaypoint());
			telemetry.addData("Heading (degrees)", odometry.getRoboPosition().getHeading(AngleUnit.DEGREES));
			telemetry.addData("X (meters)", odometry.getRoboPosition().getX(DistanceUnit.METER));
			telemetry.addData("Y (meters)", odometry.getRoboPosition().getY(DistanceUnit.METER));
			telemetry.addData("drive", path.getSuggestedDrive());
			telemetry.addData("strafe", path.getSuggestedStrafe());
			telemetry.addData("turn", path.getSuggestedTurn());

			telemetry.update();

			// Thread Control
			updateCurrentTime();
			long sleepTime = Math.round(period - (currentTime - beginTime));
			if (sleepTime > 0) sleep(sleepTime);
			beginTime = getRuntime()*1000;
			timeInRoute += period;
		}
	}
	public void updateCurrentTime(){
		currentTime = getRuntime()*1000;
	}
	public double sigmoid(double in){
		return ((1.6/(1+Math.pow(1000000000,-in)))-0.80)+0.2*(in/Math.abs(in));
	}
}
