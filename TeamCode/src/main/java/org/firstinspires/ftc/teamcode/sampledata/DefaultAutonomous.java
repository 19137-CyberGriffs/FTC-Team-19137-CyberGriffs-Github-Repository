package org.firstinspires.ftc.teamcode.sampledata;

import static org.firstinspires.ftc.teamcode.subsystems.Elevator.ElevatorPosition.*;
import static org.firstinspires.ftc.teamcode.subsystems.Elevator.TrayPosition.*;
import static org.firstinspires.ftc.teamcode.subsystems.Intake.ArmPosition.*;
import static org.firstinspires.ftc.teamcode.subsystems.Intake.ClawPosition.*;



import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.util.ParallelMotion;
//import org.firstinspires.ftc.teamcode.vision.AprilTagManager;

/**
 * @deprecated
 * @author Sunny-19-19
 * @author JacksonClack4
 */
@Autonomous(name = "Two Sample - Autonomous")
public class DefaultAutonomous extends LinearOpMode {

    Drivetrain drivetrain;
    Elevator elevator;
    Intake intake;
    //AprilTagManager aprilTag;
    ElapsedTime timer;

    @Override
    public void runOpMode() throws InterruptedException {

	   drivetrain = new Drivetrain(hardwareMap);
	   elevator = new Elevator(hardwareMap);
	   intake = new Intake(hardwareMap);
	   //aprilTag = new AprilTagManager(hardwareMap);
	   timer = new ElapsedTime();

	   drivetrain.initAuton(this);
	   elevator.setTrayPosition(TRAY_INIT);
	   intake.setClawPosition(CLAW_CLOSE);

	   waitForStart();
	   
	   //release starting sample into tray
	   elevator.setTrayPosition(TRAY_RELEASE);
	   sleep(200);
	   intake.setClawPosition(CLAW_OPEN);
	   sleep(100);
	   
	   //movement to release starting sample
	   drivetrain.driveByEncoder(380,250,0);
	   intake.setArmPosition(ARM_ELEVATOR_MOVING);
	   
	   
	   //move elevator to release starting sample
	   new ParallelMotion(
			 () -> {
				drivetrain.setHeading(45);
			 },
			 () -> {
				elevator.setElevatorPosition(ELEVATOR_HIGH);
			 }
	   ).start();
	   
	   //release starting sample into basket
	   drivetrain.driveByEncoder(125,0,0);
	   sleep(500);
		elevator.setTrayPosition(TRAY_INTAKE);
		drivetrain.driveByEncoder(-250,0,0);
		
		
		//start movement to 1st sample
		new ParallelMotion(
			 () -> {
				drivetrain.setHeading(90);
			 },
			 () -> {
				elevator.setElevatorPosition(ELEVATOR_LOW);
			 }
	   ).start();
		
		//set intake modes to obtain 1st sample
		intake.setIntakeMode(Intake.IntakeMode.VERTICAL);
		intake.setArmPosition(ARM_INTAKE);
		intake.setClawPosition(CLAW_OPEN);

		drivetrain.driveByEncoder(30,0,0);
		
		sleep(300);
		intake.setClawPosition(CLAW_CLOSE);
		sleep(400);
		
		//set tray and arm values to obtain 1st sample
		elevator.setTrayPosition(TRAY_RELEASE);
		intake.setArmPosition(ARM_RELEASE);
		drivetrain.driveByEncoder(90,0,0);
		
		drivetrain.setHeading(40);
		
		//release 1st sample in tray
		sleep(600);
		intake.setClawPosition(CLAW_OPEN);
		sleep(200);
		
		
		//move to release 1st sample
		new ParallelMotion(
			 () -> {
				drivetrain.driveByEncoder(205,110,0);
				drivetrain.driveByEncoder(-30,0,0);
			 },
			 () -> {
			 	intake.setArmPosition(ARM_ELEVATOR_MOVING);
				elevator.setElevatorPosition(ELEVATOR_HIGH);
			 }
	   ).start();
	   
	   
	   //release 1st sample
	   elevator.setTrayPosition(TRAY_INTAKE);
	   sleep(400);
	   
	   //back up from 1st sample
	   drivetrain.driveByEncoder(-70,0,0);
	   
	   
	   
	   //Start movement to 2nd sample
	   
	   //start movement to 2nd sample
		new ParallelMotion(
			 () -> {
				drivetrain.setHeading(90);
			 },
			 () -> {
				elevator.setElevatorPosition(ELEVATOR_LOW);
			 }
	   ).start();
	   
	   sleep(500);
	   
	   intake.setArmPosition(ARM_RELEASE);
	   
	/*	//set tray and arm values to obtain 1st sample
		elevator.setTrayPosition(TRAY_RELEASE);
		intake.setArmPosition(ARM_RELEASE);
		drivetrain.driveByEncoder(90,0,0);
		
		drivetrain.setHeading(40);
		
		//release 1st sample in tray
		sleep(200);
		intake.setClawPosition(CLAW_OPEN);
		sleep(200);
		/*
		
		//move to release 2nd sample
		new ParallelMotion(
			 () -> {
				drivetrain.driveByEncoder(140,50,0);
				drivetrain.driveByEncoder(-30,0,0);
			 },
			 () -> {
			 	intake.setArmPosition(ARM_ELEVATOR_MOVING);
				elevator.setElevatorPosition(ELEVATOR_HIGH);
			 }
	   ).start();
	   
	   
	   //release 2nd sample
	   sleep(800);
	   elevator.setTrayPosition(TRAY_INTAKE);
	   sleep(400);
	   
	   //back up from 2nd sample
	   drivetrain.driveByEncoder(-70,0,0);
		
	   elevator.setElevatorPosition(ELEVATOR_ZERO);
	   sleep(1000);
	   intake.setArmPosition(ARM_RELEASE);
	   //*/
	   

	   // Check how long auton is
	   telemetry.addData("Elapsed Time", timer.time());
	   telemetry.update();
	   sleep(2000);

    }
}
