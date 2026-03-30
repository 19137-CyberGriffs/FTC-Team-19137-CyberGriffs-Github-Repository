package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvPipeline;

public class VisionManager {

    OpenCvCamera webcam;
    OpenCvPipeline pipeline;

    public VisionManager(HardwareMap hardwareMap, OpenCvPipeline pipeline){

	   // Get the camera Id from the hardwareMap
	   int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
			 "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

	   // Using the ID we got previously initialize the webcam
	   webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

	   // Set the pipeline. Refer to readme about what a pipeline is.
	   this.pipeline = pipeline;
	   webcam.setPipeline(this.pipeline);

	   // Open the webcam device
	   webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
		  @Override
		  public void onOpened() {
			 // Start streaming at 320x240 resolution
			 webcam.startStreaming(320, 240);
		  }

		  @Override
		  public void onError(int errorCode) {
			 // TODO: ADD ERROR HANDLING
		  }
	   });
    }

    // Add Vision telemetry
    public void addVisionTelemetry(Telemetry telemetry){

	   telemetry.addData("FPS", webcam.getCurrentPipelineMaxFps());
	   telemetry.addData("PipelineMs",webcam.getPipelineTimeMs());
	   telemetry.addData("Overhead", webcam.getOverheadTimeMs());

	   if(pipeline instanceof ColorDetectionPipeline){
		  ColorDetectionPipeline colorDetectionPipeline = (ColorDetectionPipeline) pipeline;
		  Point contourCenter = colorDetectionPipeline.getLargestContourCenter();
		  telemetry.addData("X", contourCenter.x);
		  telemetry.addData("Y", contourCenter.y);
		  telemetry.addData("Area", colorDetectionPipeline.getLargestContourArea());
	   }
    }
}
