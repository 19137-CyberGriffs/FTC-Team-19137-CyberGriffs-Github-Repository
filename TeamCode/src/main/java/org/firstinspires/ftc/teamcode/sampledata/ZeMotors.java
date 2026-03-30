package org.firstinspires.ftc.teamcode.sampledata;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "You Drive")
public class ZeMotors extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {


	   // Intake (obviously)
	   DcMotor intake = hardwareMap.get(DcMotor.class, "intake");

	   // Gecko Wheel setup
	   DcMotor geckoWheel = hardwareMap.get(DcMotor.class, "geckoWheel");

	   // Rhino wheel setup
	   DcMotor flywheel = hardwareMap.get(DcMotor.class, "flywheel");

	   waitForStart();
	   while (this.opModeIsActive()) {

		  // Turn on intake
		  if (gamepad1.right_trigger >= 0.5) {

			 intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
			 intake.setPower(1);
		  }
		  else {
			 intake.setPower(0);
		  }

		  // If statement for launcher activation
		  if (gamepad1.left_trigger >= 0.5) {

			 intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
			 geckoWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
			 flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

			 flywheel.setPower(0.5);
			 geckoWheel.setPower(1);
			 intake.setPower(1);

		  }
		  else {
			 flywheel.setPower(0);
			 geckoWheel.setPower(0);
			 intake.setPower(0);
		  }

		  // Cycle artifacts through the "system"
		  if (gamepad1.y) {
			 geckoWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
			 intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
			 intake.setPower(0.4);
			 geckoWheel.setPower(0.4);
		  }
		  else {
			 intake.setPower(0);
			 geckoWheel.setPower(0);
		  }

		  // this was a suggestion from laffiyette (weston lafollette) that we add in case
		  // the robot breaks again and we're a push bot. It basically allows us to eject balls
		  // from the ramp area.
		  if (gamepad1.b) {
			 intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
			 geckoWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
			 intake.setPower(-0.5);
			 geckoWheel.setPower(-0.5);
		  }

		  else {
			 intake.setPower(0);
			 geckoWheel.setPower(0);
		  }

	   }

    }

}
