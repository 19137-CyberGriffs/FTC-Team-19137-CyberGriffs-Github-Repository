    package org.firstinspires.ftc.teamcode.config;

    import com.qualcomm.robotcore.hardware.Gamepad;

    /*

	   The purpose of this class it to handle the controller configuration in
	   one place. For example, if I want to change INTAKE from left_bumper
	   right_bumper, I don't have to edit every single OpMode to make that change.
	   Instead, I can instance this class in every OpMode and then have all the
	   settings in one place.
	   Also, Rumble only works with PS4/PS5/XBOX controllers, not with the F310.

	   CLASS INSTRUCTIONS

	   If you want to change what player controls a certain action
	   go to the command you want to change and change player1.[input] to player2.[input]

	   If you want to change the input for the command itself
	   go to the command and change player1.[CHANGE_THIS]

	   The list of Controller inputs is as follow:

	*/

    public class ControllerManager {

	   private final Gamepad player1;
	   private final Gamepad player2;

	   // Rumble effects to indicate Control and End Game
	   private final Gamepad.RumbleEffect initRumble;
	   private final Gamepad.RumbleEffect endGameRumble;

	   //-- CONSTRUCTOR --//
	   public ControllerManager(Gamepad gamepad1, Gamepad gamepad2){

		  // We get the Gamepad Objects from the LinearOpMode from where this class is instanced
		  this.player1 = gamepad1;
		  this.player2 = gamepad2;

		  // Each Step sets the power of the vibration motor for a specified amount of time in ms
		  initRumble = new Gamepad.RumbleEffect.Builder()
				.addStep(1,1,300)
				.build();

		  endGameRumble = new Gamepad.RumbleEffect.Builder()
				.addStep(1,1,600)
				.build();
	   }


	   // SPINNER CONTROLS
	   public boolean intake_close(){
		  return player2.right_bumper;
	   }
	   public boolean intake_release(){
		  return player2.left_bumper;
	   }

	   // ARM CONTROLS
	   public boolean arm_intake(){
		  return player2.a;
	   }
	   public boolean arm_release(){
		  return player2.y;
	   }

	   // ELEVATOR CONTROLS
	   public boolean elevator_high(){
		  return player2.dpad_up;
	   }
	   public boolean elevator_mid() {return false;}
	   public boolean elevator_low(){
		  return player2.dpad_down;
	   }

	   // TRAY CONTROLS
	   public boolean tray_intake(){
		  return player2.left_trigger >= 0.2;}
	   public boolean tray_release(){
		  return player2.left_trigger < 0.2;}

	   // DRIVETRAIN CONTROLS
	   public float drive(){
		  return player1.left_stick_y;
	   }
	   public float strafe(){
		  return player1.left_stick_x;
	   }
	   public float turn(){
		  return player1.right_stick_x;
	   }

	   // MISC
	   public boolean slowMode(){
		  return player1.right_trigger > 0.2;
	   }
	   public boolean rawInput(){
		  return player1.left_trigger > 0.2;
	   }
	   
	   
	   public boolean ascent_prepare(){
		  return false;
	   }
	   public boolean ascent_execute(){
		  return false;
	   }

	   public boolean intake_mode_horizontal(){
		  return player1.right_bumper;
	   }
	   public boolean intake_mode_vertical(){
		  return player1.left_bumper;
	   }


	   public double customRadiusPos(){
		  return player2.left_stick_y;
	   }
	   public boolean resetIntake(){
		  return player1.right_stick_button;
	   }
	   public boolean resetHeading(){
		  return player1.right_stick_button && player1.left_stick_button;
	   }

	   // When called, rumbles the controller to indicate control
	   public void opModeInitRumble(){
		  player1.runRumbleEffect(initRumble);
		  player2.runRumbleEffect(initRumble);
	   }

	   // When called, rumbles the controller to indicate End Game
	   public void endGameRumble(){
		  player1.runRumbleEffect(endGameRumble);
		  player2.runRumbleEffect(endGameRumble);
	   }
    }
