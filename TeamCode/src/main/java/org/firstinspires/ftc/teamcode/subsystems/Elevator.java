package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.Elevator.ElevatorPosition.*;
import static org.firstinspires.ftc.teamcode.subsystems.Elevator.TrayPosition.*;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.Config;

public class Elevator {

    //cheeseburger

    private final DcMotor elevatorLeft;
    private final DcMotor elevatorRight;
    private final Servo elevatorServo;
    private ElevatorPosition currentElevatorPosition = ELEVATOR_LOW;

    private TrayPosition currentTrayPosition = TRAY_INTAKE;

    public Elevator (HardwareMap hardwareMap) {
	   elevatorLeft = hardwareMap.get(DcMotor.class, "elevatorLeft");
	   elevatorRight = hardwareMap.get(DcMotor.class, "elevatorRight");
	   elevatorServo = hardwareMap.get(Servo.class, "elevatorServo");

	   elevatorLeft.setDirection(DcMotor.Direction.FORWARD);
	   elevatorRight.setDirection(DcMotor.Direction.REVERSE);
	   elevatorServo.setDirection(Servo.Direction.FORWARD);

	   elevatorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
	   elevatorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

	   elevatorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
	   elevatorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

	   elevatorLeft.setTargetPosition(0);
	   elevatorRight.setTargetPosition(0);

	   elevatorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
	   elevatorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

	   elevatorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
	   elevatorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

	   elevatorLeft.setPower(Config.Elevator.ELEVATOR_POWER);
	   elevatorRight.setPower(Config.Elevator.ELEVATOR_POWER);
    }

    public void setElevatorPosition(ElevatorPosition epos) {
	   switch (epos) {
		  case ELEVATOR_LOW: {
			 elevatorLeft.setTargetPosition(Config.Elevator.ELEVATOR_LOW);
			 elevatorRight.setTargetPosition(Config.Elevator.ELEVATOR_LOW);
			 currentElevatorPosition = ELEVATOR_LOW;
			 break;
		  }
		  case ELEVATOR_MID: {
			 elevatorLeft.setTargetPosition(Config.Elevator.ELEVATOR_MID);
			 elevatorRight.setTargetPosition(Config.Elevator.ELEVATOR_MID);
			 currentElevatorPosition = ELEVATOR_MID;
			 break;
		  }
		  case ELEVATOR_HIGH: {
			 elevatorLeft.setTargetPosition(Config.Elevator.ELEVATOR_HIGH);
			 elevatorRight.setTargetPosition(Config.Elevator.ELEVATOR_HIGH);
			 currentElevatorPosition = ELEVATOR_HIGH;
			 break;
		  }
		  case ELEVATOR_ZERO:{
			 elevatorLeft.setTargetPosition(0);
			 elevatorRight.setTargetPosition(0);
			 currentElevatorPosition = ELEVATOR_ZERO;
			 break;
		  }
		  case ELEVATOR_ASCENT:{
			 elevatorLeft.setTargetPosition(Config.Elevator.ELEVATOR_ASCENT);
			 elevatorRight.setTargetPosition(Config.Elevator.ELEVATOR_ASCENT);
			 currentElevatorPosition = ELEVATOR_ASCENT;
		  }
	   }

    }

    public void setTrayPosition(TrayPosition tpos) {
	   switch (tpos) {
		  case TRAY_INTAKE: {
			 elevatorServo.setPosition(Config.Elevator.TRAY_INTAKE_POS);
			 currentTrayPosition = TRAY_INTAKE;
			 break;
		  }
		  case TRAY_RELEASE: {
			 elevatorServo.setPosition(Config.Elevator.TRAY_RELEASE_POS);
			 currentTrayPosition = TRAY_RELEASE;
			 break;
		  }
		  case TRAY_INIT: {
			 elevatorServo.setPosition(Config.Elevator.TRAY_INIT_POS);
			 currentTrayPosition = TRAY_INIT;
		  }
	   }
    }

    public boolean isBusy(){
	   return elevatorLeft.isBusy() || elevatorRight.isBusy();
    }

    public ElevatorPosition getElevatorPosition(){
	   return currentElevatorPosition;
    }

    public TrayPosition getTrayPosition(){
	   return currentTrayPosition;
    }

    public void addElevatorTelemetry(Telemetry telemetry)
    {
	   telemetry.addData("Elevator Position", currentElevatorPosition);
	   telemetry.addData("Tray Position", currentTrayPosition);
    }

    public void resetEncoders(){
	   elevatorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
	   elevatorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

	   elevatorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
	   elevatorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

	   elevatorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
	   elevatorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public enum ElevatorPosition
    {
	   ELEVATOR_HIGH,
	   ELEVATOR_MID,
	   ELEVATOR_LOW,
	   ELEVATOR_ZERO,
	   ELEVATOR_ASCENT,
    }
    public enum TrayPosition
    {
	   TRAY_INTAKE,
	   TRAY_RELEASE,
	   TRAY_INIT
    }

}
