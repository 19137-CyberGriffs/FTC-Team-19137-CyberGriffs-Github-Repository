package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class AprilTagManager {

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;
    private AprilTagDetection desiredTag;
    private final int desiredTagID = 13;

    public AprilTagManager(HardwareMap hardwareMap){

	   aprilTagProcessor = new AprilTagProcessor.Builder()
			 .setOutputUnits(DistanceUnit.MM, AngleUnit.DEGREES)
			 .setDrawCubeProjection(true)
			 .setDrawAxes(true)
			 .build();
	   visionPortal = new VisionPortal.Builder()
			 .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
			 .addProcessor(aprilTagProcessor)
			 .build();

    }

    public boolean isDetectingTarget(){
	   return !aprilTagProcessor.getDetections().isEmpty();
    }

    public double getZ(){

	   List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
	   desiredTag = null;
	   for (AprilTagDetection tag: detections){
		  if(tag.id == desiredTagID){
			 desiredTag = tag;
		  }
	   }

	   if(desiredTag != null){
		  return desiredTag.ftcPose.z;
	   }else{
		  return -1;
	   }
    }

    public double getX(){

	   List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
	   desiredTag = null;
	   for (AprilTagDetection tag: detections){
		  if(tag.id == desiredTagID){
			 desiredTag = tag;
		  }
	   }

	   if(desiredTag != null){
		  return desiredTag.ftcPose.x;
	   }else{
		  return -1;
	   }
    }

    public double getY(){

	   List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
	   desiredTag = null;
	   for (AprilTagDetection tag: detections){
		  if(tag.id == desiredTagID){
			 desiredTag = tag;
		  }
	   }

	   if(desiredTag != null){
		  return desiredTag.ftcPose.y;
	   }else{
		  return -1;
	   }
    }

    public double getBearing(){

	   List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
	   desiredTag = null;
	   for (AprilTagDetection tag: detections){
		  if(tag.id == desiredTagID){
			 desiredTag = tag;
		  }
	   }

	   if(desiredTag != null){
		  return desiredTag.ftcPose.bearing;
	   }else{
		  return -1;
	   }
    }

    public double getElevation() {

	   List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
	   desiredTag = null;
	   for (AprilTagDetection tag : detections) {
		  if (tag.id == desiredTagID) {
			 desiredTag = tag;
		  }
	   }

	   if (desiredTag != null) {
		  return desiredTag.ftcPose.elevation;
	   } else {
		  return -1;
	   }
    }

    public double getRange() {

	   List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
	   desiredTag = null;
	   for (AprilTagDetection tag : detections) {
		  if (tag.id == desiredTagID) {
			 desiredTag = tag;
		  }
	   }

	   if (desiredTag != null) {
		  return desiredTag.ftcPose.range;
	   } else {
		  return -1;
	   }
    }

    public double getYaw() {

	   List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
	   desiredTag = null;
	   for (AprilTagDetection tag : detections) {
		  if (tag.id == desiredTagID) {
			 desiredTag = tag;
		  }
	   }

	   if (desiredTag != null) {
		  return desiredTag.ftcPose.yaw;
	   } else {
		  return -1;
	   }
    }

    public double getRoll() {

	   List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
	   desiredTag = null;
	   for (AprilTagDetection tag : detections) {
		  if (tag.id == desiredTagID) {
			 desiredTag = tag;
		  }
	   }

	   if (desiredTag != null) {
		  return desiredTag.ftcPose.roll;
	   } else {
		  return -1;
	   }
    }

    public double getPitch() {

	   List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
	   desiredTag = null;
	   for (AprilTagDetection tag : detections) {
		  if (tag.id == desiredTagID) {
			 desiredTag = tag;
		  }
	   }

	   if (desiredTag != null) {
		  return desiredTag.ftcPose.pitch;
	   } else {
		  return -1;
	   }
    }


}
