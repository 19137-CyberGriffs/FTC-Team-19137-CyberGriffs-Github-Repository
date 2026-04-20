package org.firstinspires.ftc.teamcode.sampledata;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import static com.qualcomm.robotcore.hardware.Servo.Direction.*;

import org.firstinspires.ftc.teamcode.config.ControllerManager;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Odometry;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.dfrobot.HuskyLens;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import org.firstinspires.ftc.robotcore.external.JavaUtil;

import java.util.Arrays;

/**
 * @deprecated
 * @author YesItIsEvan
 */
@TeleOp(name = "DecodeTeleOp")
public class DecodeTeleOp extends LinearOpMode {


    //===OBJECT AND VARIABLE DECLARATION===//
    public enum Alliance {BLUE, RED};
    Alliance alliance = Alliance.BLUE;

    Drivetrain drivetrain;
    ControllerManager controllers;
    HuskyLens huskyLens;
    GoBildaPinpointDriver pinpoint;
    Pose2D roboPosition;
    double distanceToGoal;

    double drive;
    double strafe;
    double turn = 0;

    private int magRotationCount = 0;
    boolean rightBumperUsed = false;
    boolean leftBumperUsed = false;

    int turretTargetPosition = 0;

    HuskyLens.Block[] visibleTags;
    HuskyLens.Block aprilTag;

    double motorPowerCorrectionMultiplier;

    private NormalizedColorSensor testColor;
    double hue;
    private int greenHue = 160;
    private int purpleHue = 225;
    private int lightHue = 0;

    double period = 1000/16.0;
    double beginTime = getRuntime()*1000;
    double currentTime;

    int previousFlywheelPos = 0;
    int currentFlywheelPos = 0;
    double flywheelRPM;
    double targetFlywheelTPS;
    double[] flywheelRPMOverTime = new double[10];

    double liftWait = 0;
    double magToIntakeWait = 0;
    double intakeWait = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        //===CONSTRUCTOR SECTION===//
	   /*
		Each of these lines of code calls upon an imported  class to construct various
		mechanical objects, such as the flywheel motor or the intake motor using the
		DcMotor class.

		In case you don't understand what a constructor does, take an introductory
		Java programming class, or refer to README - [02] for more information on
		what this section actually accomplishes.
	   */

        //Drivetrain Constructors
        drivetrain = new Drivetrain(hardwareMap);
        drivetrain.initOpMode(); //Initializes the drivetrain and allows for user input to start upon opmode beginning
        controllers = new ControllerManager(gamepad1, gamepad2);



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
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        //Camera Constructor
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        //Initializes the position of components
        Odometry odometryStuff = new Odometry(hardwareMap,alliance);
        magServo.setPosition(.127);
        lift.setPosition(0.012);
        turretYaw.setPower(0.1);
        flywheelMotor.setPower(0);
        flywheelMotor.setVelocity(0);
        hood.scaleRange(0,0.6);

        testColor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");

        waitForStart();

        turretYaw.setPower(0.9);
        controllers.opModeInitRumble();

        while (opModeIsActive()){  //Loop while teleop is active, DO NOT EXIT

            //uses battery voltage to adjust motor power for consistent power output
            motorPowerCorrectionMultiplier = (12/voltage.getVoltage());


            //===COLOR SENSOR===//
            NormalizedRGBA colors = testColor.getNormalizedColors();
            hue = JavaUtil.colorToHue(colors.toColor());

            //TODO: Finish magazine class, it incorporates color sensor, makes easier
            //TODO: Use above conditional statements to use color sensor properly

            //===ODOMETRY===//
            odometryStuff.updatePosition();
            distanceToGoal = odometryStuff.getDistanceFromGoal();
            if(!gamepad1.left_bumper)
                turretTargetPosition = (int)((1000/90.0)*odometryStuff.getBasketAngle(turretYaw.getCurrentPosition())+0.5);



            //===DRIVETRAIN===//
            drivetrain.setSmoothMotion(false);
            drivetrain.setFieldCentricControl(false); //No global control (stinky)

            if(controllers.resetHeading()){
                drivetrain.resetHeading();
            }

            double boostCoe = ((double)gamepad1.left_trigger*0.5)+1;

            drive  = -0.8*controllers.drive()*boostCoe; // Drive is negative because a stick's Y-axis ranges from -1 (up stick) to 1 (down stick)
            strafe = 1.5*controllers.strafe()*boostCoe;
            turn  = 0.5*controllers.turn()*boostCoe;//smoothing only the turn, this helps give time for the turret to follow the target while turning

            drivetrain.drive(drive,strafe,turn);// Send the inputs to the drivetrain after all post processing conditions have been checked



            //===CAMERA===//
            //===================================================================================//
            //  April tag IDs are based on the order of tags learned by the camera during setup  //
            //	 The following list shows which april tag each ID corresponds to		    //
            //																   //
            //    Camera ID#				  FTC ID#			  Motif/Basket		 //
            //	   1					    20				   Blue			 //
            //	   2						   21				   GPP			  //
            //	   3					    22				   PGP			  //
            //	   4					    23				   PPG			  //
            //	   5					    24				   Red			  //
            //===================================================================================//

            visibleTags = huskyLens.blocks();
            for (HuskyLens.Block aprilTag : visibleTags) {
                if(aprilTag.id == 5 && alliance == Alliance.RED) {
                    turretTargetPosition = turretYaw.getCurrentPosition() + (int)(1.1*(aprilTag.x - 160)+0.5);
                }
                if(aprilTag.id == 1 && alliance == Alliance.BLUE) {
                    turretTargetPosition = turretYaw.getCurrentPosition() + (int)(1.1*(aprilTag.x - 160)+0.5);
                }
            }

		  /*
				The BallisticsModel class is the physics engine, accounts for air drag and velocity components using RK4. Its method, simulateShot,
				returns a boolean dependent on whether or not the given variables would allow for the ball to make it inside the bin at the given distance with respect
				to the robot.

				ShotSolution contains the speed and angle variables used in the optimizer. In other words, they are the values that are finalized
				when the physics is done calculating. These values are used below in conjunction with the power correction to then create the most optimal speed
				and extension angle possible using the object of reference, "solution".

				TurretOptimizer loops through every possible speed first, then loops through the other angle option. It simulates each step of speed and angle option
				with the BallisticsModel class to find the best results. Null is returned if a combination is not found, and a ShotSolution object is created
				if there is a successful combination of values. This object uses the combination to set the speed and angle variables aforementioned above.

				Below, if the second player engages the right trigger, the bot computes the distance to the goal and the vertical height of the bin. From there,
				it optimizes with those values using the optimizer, sets the optimal values to the shot solution, then uses the solution's values to then set
				the position of the extender flap and the speed of the flywheel.
	   */

            //===FLYWHEEL CONTROL===//

            currentFlywheelPos = flywheelMotor.getCurrentPosition();
            flywheelRPM = (currentFlywheelPos-previousFlywheelPos)*(1/period)*(60000/28.0);
            previousFlywheelPos = flywheelMotor.getCurrentPosition();

            for(int i=1;i<flywheelRPMOverTime.length;i++)
                flywheelRPMOverTime[i] = flywheelRPMOverTime[i-1];
            flywheelRPMOverTime[0] = flywheelRPM;

            flywheelMotor.setPower(0.0001*Math.abs(targetFlywheelTPS -Arrays.stream(flywheelRPMOverTime).average().getAsDouble())*motorPowerCorrectionMultiplier);
            targetFlywheelTPS = ((81.81818 * Math.pow(distanceToGoal,2) + 191.45455 * distanceToGoal + 1963.63636)*(28/60.0));
            flywheelMotor.setVelocity(targetFlywheelTPS);

            if(flywheelMotor.getVelocity() <= targetFlywheelTPS-50*(60/28.0) || flywheelMotor.getVelocity() >= targetFlywheelTPS+50*(60/28.0))
                gamepad2.rumble(10);
            //flywheelMotor.setVelocity(3270*(28/60.0)); //TESTER

            //===TURRET CHECK===//
            if(turretTargetPosition > 1000)//limitations
                turretTargetPosition = 1000;
            if(turretTargetPosition < -1000)
                turretTargetPosition = -1000;

            turretYaw.setTargetPosition(turretTargetPosition);



            //===LIFT CONTROLS===//
            if(gamepad2.right_trigger > 0.2 && liftWait <= 0) {
                magToIntakeWait = 700;
                intakeWait = 700;
                lift.setPosition(0.35);
            }
            else
                lift.setPosition(0.012);



            //===TURRET FLAP CONTROLS===//
            if(gamepad1.dpad_up){
                hood.setPosition(1);
            }
            else if(gamepad1.dpad_down){
                hood.setPosition(0.0);
            }
            else{
                hood.setPosition(distanceToGoal*0.2);
            }



            //===MAGAZINE CONTROLS===//
            if (gamepad2.left_bumper && !leftBumperUsed && magToIntakeWait <= 0){
                intakeWait = 820;
                liftWait = 850;
                magRotationCount--;
                leftBumperUsed = true;
            }

            if (gamepad2.right_bumper && !rightBumperUsed && magToIntakeWait <= 0){
                intakeWait = 820;
                liftWait = 850;
                magRotationCount++;
                rightBumperUsed = true;
            }

            if(magRotationCount < 0)
                magRotationCount = 0;
            if(magRotationCount > 2)
                magRotationCount = 2;

            if(gamepad1.right_trigger>0.2 && !gamepad2.dpad_up && magToIntakeWait <= 0) {
                liftWait = 500;
                magServo.setPosition(.1099 + (magRotationCount * .073));
            }
            else
                magServo.setPosition(.1248+(magRotationCount*.073));

            if(magServo.getPosition() != (.1099 + magRotationCount * .073))
                intakeWait = 450;

            if(!gamepad2.left_bumper)
                leftBumperUsed = false;
            if(!gamepad2.right_bumper)
                rightBumperUsed = false;



            //===INTAKE CONTROLS===//
            if (gamepad1.right_trigger>0.2 && !gamepad2.dpad_up && intakeWait <= 0){
                intakeMotor.setPower(motorPowerCorrectionMultiplier*0.9);
            }
            else{
                intakeMotor.setPower(0);
            }



            //===AUTOMATIC FIRING SYSTEM===//
		  /*
			 The Automatic Firing System (A.F.S.) coordinates a single human input to
			 automatically fire the entire contents of the magazine upon request.

			 It is a unique ability to the CyberGriffs team, and has the possibility to shave
			 many seconds of human input off of a match.
		   */

            //===TELEMETRY===//
            //Color
            telemetry.addData("Light Detected", ((OpticalDistanceSensor) testColor).getLightDetected());
            telemetry.addData("Hue: ", hue);
            if ((hue >= (greenHue - 50)) && (hue <= (greenHue + 50)))
                telemetry.addData("Current Color: ", "Green");
            else if ((hue >= (purpleHue - 50)) && (hue <= (purpleHue + 50)))
                telemetry.addData("Current Color: ", "Purple");
            else
                telemetry.addData("Current Color: ", "None Detected");

            //Flywheel
            telemetry.addData("Flywheel RPM (smoothed)", Arrays.stream(flywheelRPMOverTime).average().getAsDouble());
            telemetry.addData("Flywheel Target RPM", targetFlywheelTPS *(60/28.0));

            //Turret
            telemetry.addData("Yaw Position", turretYaw.getCurrentPosition());

            //Magazine
            telemetry.addData("Magazine Rotation Position Value", magRotationCount);

            //Odometry
            telemetry.addData("X coordinate (meters)", odometryStuff.getRoboPosition().getX(DistanceUnit.METER));
            telemetry.addData("Y coordinate (meters)", odometryStuff.getRoboPosition().getY(DistanceUnit.METER));
            telemetry.addData("Heading angle (DEGREES)", odometryStuff.getRoboPosition().getHeading(AngleUnit.DEGREES));
            telemetry.addData("Distance from Goal", Math.round(odometryStuff.getDistanceFromGoal()*10000)/10000.0 + "meter(s)");

            //Camera



            //===THREAD CONTROL===//

            liftWait -= period;
            magToIntakeWait -= period;
            intakeWait -= period;
            updateCurrentTime();
            long sleepTime = Math.round(period - (currentTime - beginTime));
            if (sleepTime > 0) sleep(sleepTime);

            //System Monitor
            updateCurrentTime();
            telemetry.addData("CPS", (int)(1000/(currentTime - beginTime)+0.5));

            telemetry.update();
            beginTime = getRuntime()*1000;
        }
    }
    public void updateCurrentTime(){
        currentTime = getRuntime()*1000;
    }
}
