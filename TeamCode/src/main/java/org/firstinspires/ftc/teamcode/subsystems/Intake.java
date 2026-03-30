package org.firstinspires.ftc.teamcode.subsystems;

import static com.qualcomm.robotcore.hardware.Servo.Direction.*;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.Config;

public class Intake {

    // Declare objects
    private final DcMotor intakeArm;
    private final Servo intakeWrist;
    private final Servo intakeRadius; // As in the bone
    private final Servo intakeClawLeft;
    private final Servo intakeClawRight;

    volatile private ArmPosition armPosition = null;
    private ClawPosition clawPosition = ClawPosition.CLAW_CLOSE;
    private IntakeMode intakeMode = IntakeMode.HORIZONTAL;


    //-- CONSTRUCTOR --//
    public Intake(HardwareMap hardwareMap){

	   // Initialize all objects
	   intakeArm =		 hardwareMap.get(DcMotor.class, "intakeArm");
	   intakeWrist =	  hardwareMap.get(Servo.class, "intakeWrist");
	   intakeRadius =    hardwareMap.get(Servo.class,"intakeRadius");
	   intakeClawLeft =  hardwareMap.get(Servo.class, "intakeClawLeft");
	   intakeClawRight = hardwareMap.get(Servo.class, "intakeClawRight");

	   // Setup the Servo's directions
	   intakeWrist.setDirection(REVERSE);
	   intakeClawLeft.setDirection(FORWARD);
	   intakeClawRight.setDirection(FORWARD);

	   // Setup the Arm motor
	   intakeArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
	   intakeArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
	   intakeArm.setTargetPosition(0);
	   intakeArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
	   intakeArm.setPower(Config.Intake.ARM_MAX_POWER);

    }


    public void setIntakeMode(IntakeMode mode){
	   switch (mode){
		  case VERTICAL:
			 this.intakeMode = IntakeMode.VERTICAL;
			 setArmPosition(armPosition);
			 break;
		  case HORIZONTAL:
			 this.intakeMode = IntakeMode.HORIZONTAL;
			 setArmPosition(armPosition);
			 break;
	   }
    }


    public void setCustomRadiusPosition(double pos){

	   double mappedPos = mapRange(pos, -1,1,0,1);

	   if(mappedPos > Config.Intake.RADIUS_MAX){
		  mappedPos = Config.Intake.RADIUS_MAX;
	   }
	   else if(mappedPos < Config.Intake.RADIUS_MIN){
		  mappedPos = Config.Intake.RADIUS_MIN;
	   }

	   intakeRadius.setPosition(mappedPos);

    }


    public void setArmPosition(ArmPosition pos){

	   armPosition = pos;

	   switch (pos){

		  case ARM_INTAKE:
		  	intakeArm.setTargetPosition(Config.Intake.ARM_INTAKE_POS);
			 intakeWrist.setPosition(Config.Intake.WRIST_INTAKE_POS);
			 if(intakeMode == IntakeMode.HORIZONTAL){
				intakeRadius.setPosition(Config.Intake.RADIUS_HORIZONTAL);
			 }
			 else{
				intakeRadius.setPosition(Config.Intake.RADIUS_VERTICAL);
			 }
			 break;

		  case ARM_RELEASE:
		  	 intakeArm.setPower(Config.Intake.ARM_MAX_POWER);
			 intakeArm.setTargetPosition(Config.Intake.ARM_RELEASE_POS);
			 intakeWrist.setPosition(Config.Intake.WRIST_RELEASE_POS);
			 intakeRadius.setPosition(Config.Intake.RADIUS_VERTICAL);
			 break;

		  case ARM_HOVER:
		  	 intakeArm.setTargetPosition(Config.Intake.ARM_HOVER_POS);
			 intakeWrist.setPosition(Config.Intake.WRIST_INTAKE_POS);
			 if(intakeMode == IntakeMode.HORIZONTAL){
				intakeRadius.setPosition(Config.Intake.RADIUS_HORIZONTAL);
			 }
			 else{
				intakeRadius.setPosition(Config.Intake.RADIUS_VERTICAL);
			 }
			 break;

		  case ARM_ELEVATOR_MOVING:
		  	 intakeArm.setPower(Config.Intake.ARM_MAX_POWER);
			 intakeArm.setTargetPosition(Config.Intake.ARM_ELEVATOR_MOVING_POS);
			 break;

		  case ARM_LOW_ASCENT:
			 intakeArm.setTargetPosition(Config.Intake.ARM_LOW_ASCENT);
			 break;

		  case ARM_PUSH:
			 intakeArm.setTargetPosition(Config.Intake.ARM_PUSH);
			 intakeWrist.setPosition(Config.Intake.WRIST_PUSH_POS);
			 break;

		  case ARM_BACK_OUT:
			 intakeArm.setTargetPosition(Config.Intake.ARM_BACK_OUT);
			 intakeWrist.setPosition(Config.Intake.WRIST_INTAKE_POS);
			 break;

	   }

	   intakeArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);


    }


    public void setClawPosition(ClawPosition pos){

	   switch (pos){
		  case CLAW_OPEN:
			 intakeClawRight.setPosition(Config.Intake.CLAW_RIGHT_OPEN_POS);
			 intakeClawLeft.setPosition(Config.Intake.CLAW_LEFT_OPEN_POS);
			 break;
		  case CLAW_CLOSE:
			 intakeClawRight.setPosition(Config.Intake.CLAW_RIGHT_CLOSE_POS);
			 intakeClawLeft.setPosition(Config.Intake.CLAW_LEFT_CLOSE_POS);
			 break;
	   }
	   clawPosition = pos;
    }


    private void curveArmPosition(ArmPosition targetPos, int target){

	   new Thread(
			 () -> {

				intakeArm.setTargetPosition(target);
				intakeArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

				while (armPosition == targetPos) {

				    int currentPos = intakeArm.getCurrentPosition();
				    double progress = Math.min(1.0, Math.abs((double) currentPos / target));

				    if (progress >= 1.0) {
					   break;
				    }

				    // Power Curve Function (X=0 to X=1) (progress is X)
				    double skewedProgress = Math.sin(progress * Math.PI);

				    // Calculate power dynamically
				    double power = Config.Intake.ARM_MAX_POWER * skewedProgress;
				    power = Math.max(power, Config.Intake.ARM_MIN_POWER);

				    intakeArm.setPower(power);

				}
			 }
	   ).start();

    }


    public static double mapRange(double value, double fromLow, double fromHigh, double toLow, double toHigh) {
	   return (value - fromLow) / (fromHigh - fromLow) * (toHigh - toLow) + toLow;
    }


    public void addIntakeTelemetry(Telemetry telemetry){
	   telemetry.addData("Arm Position", armPosition);
	   telemetry.addData("Claw Position", clawPosition);
	   telemetry.addData("Intake Mode", intakeMode);
    }


    public ArmPosition getArmPosition(){
	   return armPosition;
    }

    public ClawPosition getClawPosition(){
	   return clawPosition;
    }

    public IntakeMode getIntakeMode(){
	   return intakeMode;
    }


    public enum ClawPosition{
	   CLAW_CLOSE,
	   CLAW_OPEN
    }

    public enum ArmPosition{
	   ARM_INTAKE,
	   ARM_RELEASE,
	   ARM_HOVER,
	   ARM_ELEVATOR_MOVING,
	   ARM_LOW_ASCENT,
	   ARM_PUSH,
	   ARM_BACK_OUT
    }

    public enum IntakeMode{
	   HORIZONTAL,
	   VERTICAL
    }

}
