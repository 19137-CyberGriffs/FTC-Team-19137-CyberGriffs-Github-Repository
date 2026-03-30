package org.firstinspires.ftc.teamcode.sunny;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.concurrent.TimeUnit;

public class MotorWrapper {

    private DcMotor motor;
    Telemetry telemetry;

    private int lastPosition;
    private long lastTime;
    private final double ticksPerRev = 28;
    private double rpm = 0;

    public MotorWrapper(DcMotor motor, Telemetry telemetry){

	   this.telemetry = telemetry;
	   this.motor = motor;

	   lastPosition = motor.getCurrentPosition();
	   lastTime = System.nanoTime();

	   // Create BackgroundTask to get rpm updates at a constant rate
	   BackgroundTask rpmUpdater = new BackgroundTask(
			 this::updateRPM,	 // The task that will run at a constant rate
			 40,			    // Time between updates
			 TimeUnit.MILLISECONDS // Set the time units to milliseconds
	   );
	   // Start the BackgroundTask (.shutdown() to stop it)
	   rpmUpdater.start();

    }

    public void reset(){
	   lastPosition = motor.getCurrentPosition();
	   lastTime = System.nanoTime();
    }

    private double lastError = 0;
    private double integral = 0;
    private double power = 0;

    public void setRPM(double targetRPM) {
	   double Kp = 0.3;
	   double Ki = 0.001;
	   double Kd = 1;

	   double error = targetRPM - getRPM();
	   integral += error;
	   integral = Math.max(-30, Math.min(30, integral)); // Prevent windup
	   double derivative = error - lastError;

	   double newPower = (Kp * error) + (Ki * integral) + (Kd * derivative);
	   newPower = Math.max(0, Math.min(1, newPower)); // Ensure power only moves forward

	   double smoothingFactor = 0.1;
	   power = (power * (1 - smoothingFactor)) + (newPower * smoothingFactor); // Smooth transitions

	   motor.setPower(power);
	   lastError = error;

	   try{
		  Thread.sleep(20);
	   } catch (Exception e){
		  System.out.println(e.getMessage());
	   }
    }


    public double getRPM() {
	   return rpm;
    }

    public int getTickChange(){

	   int currentPosition = motor.getCurrentPosition();
	   int ticks = currentPosition - lastPosition;

	   lastPosition = currentPosition;

	  return ticks;

    }

    // Updates the rpm variable
    // DO NOT call this method, it is exclusive to the background task
    private void updateRPM(){

	   long currentTime = System.nanoTime();
	   int currentPosition = motor.getCurrentPosition();

	   long deltaTime = currentTime - lastTime; // nanoseconds
	   int deltaTicks = currentPosition - lastPosition;

	   lastTime = currentTime;
	   lastPosition = currentPosition;

	   // Prevent division by zero & noise when motor stops
	   // if (deltaTime == 0 || Math.abs(deltaTicks) < 2) return 0.0;

	   rpm = (deltaTicks / ticksPerRev) * 60.0 / (deltaTime / 1e9);

    }

}

