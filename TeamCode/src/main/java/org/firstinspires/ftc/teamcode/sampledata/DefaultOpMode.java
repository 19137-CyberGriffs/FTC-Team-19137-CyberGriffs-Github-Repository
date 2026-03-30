
package org.firstinspires.ftc.teamcode.sampledata;

import static org.firstinspires.ftc.teamcode.subsystems.Elevator.ElevatorPosition.*;
import static org.firstinspires.ftc.teamcode.subsystems.Elevator.TrayPosition.*;
import static org.firstinspires.ftc.teamcode.subsystems.Intake.ArmPosition.*;
import static org.firstinspires.ftc.teamcode.subsystems.Intake.ClawPosition.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.ControllerManager;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

//TODO: Does jackson even read my todos?

@TeleOp(name = "Default - OpMode")
public class DefaultOpMode extends LinearOpMode {

    // Declare Objects and variables
    Drivetrain drivetrain;
    Intake intake;
    Elevator elevator;

    boolean buttonState = false;
    boolean actionToggle = true;
    boolean isHanging = false;

    ControllerManager controllers;
    ElapsedTime timer; // The timer has no real significance other than showing the time

    @Override
    public void runOpMode() throws InterruptedException {

	   // Initialize Drivetrain
	   drivetrain = new Drivetrain(hardwareMap);
	   drivetrain.initOpMode();

	   // Initialize Elevator
	   elevator = new Elevator(hardwareMap);

	   // Initialize Scooper
	   intake = new Intake(hardwareMap);

	   // Initialize Controllers
	   controllers = new ControllerManager(gamepad1, gamepad2);
	   timer = new ElapsedTime();

	   // End game warning
	   boolean endGameWarning = false;

	   // Initial Setup
	   elevator.setTrayPosition(TRAY_INIT);
	   intake.setClawPosition(CLAW_CLOSE);
	   intake.setArmPosition(ARM_RELEASE);


	   //-- TELEMETRY --//

	   // Keep updating telemetry while waiting for start (To check if the camera is working)
	   while(!isStarted() && !isStopRequested()){

		  telemetry.addData("Hey Griffs, It's me, Nightmare 👻", "");
		  telemetry.addData("Status", "Initialized");
		  telemetry.update();

	   }

	   // Wait for the User to press Play in the DriverHub
	   waitForStart();

	   elevator.setTrayPosition(TRAY_INTAKE);

	   // Start the Timer
	   timer.startTime();

	   // Rumble to indicate the controller is active
	   controllers.opModeInitRumble();

	   // Begin the control loop
	   while(opModeIsActive()){
	   	
	   		

		  //-- DRIVETRAIN --//
		  drivetrain.setSmoothMotion(false);
		  drivetrain.setFieldCentricControl(false);

		  // Get motion inputs from the sticks
		  double drive  =-controllers.drive(); // Drive is negative because a stick's Y-axis ranges from -1 (up stick) to 1 (down stick)
		  double strafe = controllers.strafe();
		  double turn   = controllers.turn();

		  // Check if slow mode is active or elevator is up
		  if(controllers.slowMode() || elevator.getElevatorPosition() == ELEVATOR_HIGH){
			 drivetrain.setSlowMotion(true);
		  }
		  else{
			 drivetrain.setSlowMotion(false);
		  }

		  if(controllers.resetHeading()){
			 drivetrain.resetHeading();
		  }

		  // Check if the user is holding raw input (no orientation)
		  drivetrain.setFieldCentricControl(!controllers.rawInput());

		  // Send the inputs to the drivetrain after all post processing conditions have been checked
		  drivetrain.drive(drive,strafe,turn);


		  //-- INTAKE ARM--//
		  if(controllers.arm_release()){
			 if( !buttonState){
				if(actionToggle){
				    intake.setArmPosition(ARM_BACK_OUT);
				}
				else{
				    intake.setArmPosition(ARM_RELEASE);
				}
				actionToggle = false;
			 }
			 buttonState = true;
		  }
		  else{
			 buttonState = false;
		  }

		  if(controllers.arm_intake()){
			 intake.setArmPosition(ARM_HOVER);
			 intake.setClawPosition(CLAW_OPEN);
			 actionToggle = true;
		  }


		  //-- INTAKE CLAW --//

		  if(controllers.intake_close()){
			 if(intake.getArmPosition() == ARM_HOVER){
				intake.setArmPosition(ARM_INTAKE);
			 }
			 intake.setClawPosition(CLAW_CLOSE);
		  }
		  if(controllers.intake_release()){
			 intake.setClawPosition(CLAW_OPEN);

			 if(intake.getArmPosition() == ARM_INTAKE){
				intake.setArmPosition(ARM_HOVER);
			 }
		  }


		  //-- ELEVATOR --//
		  if (controllers.elevator_high())
		  {
			 elevator.setElevatorPosition(ELEVATOR_HIGH);
			 intake.setArmPosition(ARM_ELEVATOR_MOVING);
			 actionToggle = false;
		  }
		  else if (controllers.elevator_low())
		  {
			 elevator.setElevatorPosition(ELEVATOR_LOW);
			 elevator.setTrayPosition(TRAY_INTAKE);
		  }


		  //-- TRAY --//
		  if(!isHanging){
			 if(controllers.tray_release()){
				elevator.setTrayPosition(TRAY_RELEASE);
			 }
			 else{
				elevator.setTrayPosition(TRAY_INTAKE);
			 }
		  }

		  //-- ASCENT --//
		  if(controllers.ascent_prepare()){
			 isHanging = true;
			 elevator.setElevatorPosition(ELEVATOR_MID);
			 elevator.setTrayPosition(TRAY_INIT);
		  }
		  if(controllers.ascent_execute()){
			 elevator.setElevatorPosition(ELEVATOR_LOW);
			 elevator.setTrayPosition(TRAY_INIT);
			 sleep(1000);
			 intake.setArmPosition(ARM_RELEASE);
			 sleep(100000);
		  }
		  if(gamepad1.a){
			 intake.setArmPosition(ARM_PUSH);
		  }


		  //-- SET INTAKE MODE --//
		  if(controllers.intake_mode_horizontal()){
			 intake.setIntakeMode(Intake.IntakeMode.HORIZONTAL);
		  }
		  if(controllers.intake_mode_vertical()){
			 intake.setIntakeMode(Intake.IntakeMode.VERTICAL);
		  }


		  //-- TELEMETRY --//
		  telemetry.addData("Elapsed Time", timer.time());
		  telemetry.addData("Drive  ", drive);
		  telemetry.addData("Strafe ", strafe);
		  telemetry.addData("Turn   ", turn);

		  drivetrain.addGyroTelemetry(telemetry);
		  intake.addIntakeTelemetry(telemetry);
		  elevator.addElevatorTelemetry(telemetry);

		  telemetry.update();

		  // End Game Warning Rumble
		  if(timer.time() > 90 && !endGameWarning){
			 controllers.endGameRumble();
			 endGameWarning = true;
		  }
	   }
    }
}
