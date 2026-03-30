package org.firstinspires.ftc.teamcode.sunny;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;

public class HardwareInitializer {

    HardwareMap hardwareMap;
    Telemetry telemetry;

    public HardwareInitializer(HardwareMap hardwareMap, Telemetry telemetry){
	   this.hardwareMap = hardwareMap;
	   this.telemetry = telemetry;
    }


    // Prints all devices connected the Control/Expansion Hub
    public void printDevices(){
	   telemetry.addData("Devices Found", hardwareMap.size());
	   telemetry.addData("____________________", "");

	   for(HardwareDevice device: hardwareMap){
		  telemetry.addData("Device", device.getDeviceName());
	   }
	   telemetry.update();
    }


    // Prints all devices connected to the Control/Expansion hub of a certain type
    public void printDevices(Class<? extends HardwareDevice> type){

	   // Get devices of type
	   List<? extends HardwareDevice> devices = hardwareMap.getAll(type);

	   telemetry.addData("Devices of type [" + type.getSimpleName() + "] Found", devices.size());
	   telemetry.addData("___________________", "");

	   for (HardwareDevice device : devices) {
		  telemetry.addData("Device", device.getDeviceName() + " " + hardwareMap.getNamesOf(device));
	   }
	   telemetry.update();
    }

    // Print all devices with a unique name
    public void printNamedDevices() {
	   telemetry.addData("Named Devices Found", "");
	   telemetry.addData("___________________", "");

	   List<String> assignedNames = new ArrayList<>();
	   Map<String, HardwareDevice> deviceMap = new HashMap<>();

	   int count = 0;
	   for (HardwareDevice device : hardwareMap) {
		  Set<String> names = hardwareMap.getNamesOf(device);
		  if (names.isEmpty()) continue; // Skip unnamed devices

		  for (String name : names) {
			 assignedNames.add(name); // Store just the name
			 deviceMap.put(name, device); // Map name to device
			 count++;
		  }
	   }

	   // Sort device names alphabetically
	   Collections.sort(assignedNames);

	   for (String name : assignedNames) {
		  HardwareDevice device = deviceMap.get(name);

		  if (device instanceof VoltageSensor) continue;
		  if (device instanceof ServoController)continue;
		  assert device != null;
		  if (device.getDeviceName().contains("Expansion Hub Portal")) continue;

		  telemetry.addData(device.getDeviceName(), name);
	   }

	   telemetry.addData("Total Named Devices", count);
	   telemetry.update();
    }



    public void testDevicesUsingTelemetry(Gamepad gamepad, LinearOpMode opMode){

	   boolean madeSelection = true;
	   boolean lastButtonState = false;
	   int selection = 0;
	   int menuMaxOption = 3;
	   String[] mainMenu = {"Motors", "Servos", "Digital", "Analog", "I2C"};

	   while(opMode.opModeIsActive()){

		  if(madeSelection){

			 madeSelection = false;

			 for(int x = 0; x <= menuMaxOption; x++){

				if(selection == x){
				    telemetry.addData("", mainMenu[x]);
				}
				telemetry.addData("", mainMenu[x]);
			 }
			 
			 telemetry.update();

		  }

		  if(gamepad.dpad_down && !lastButtonState){
			 lastButtonState = true;
			 selection--;
			 if(selection < 0){
				selection = 0;
			 }
		  }

		  if(gamepad.dpad_up && !lastButtonState){
			 lastButtonState = true;
			 selection++;
			 if(selection > menuMaxOption){
				selection = menuMaxOption;
			 }
		  }

		  if(gamepad.a){
			 madeSelection = true;
		  }

	   }

    }




}
