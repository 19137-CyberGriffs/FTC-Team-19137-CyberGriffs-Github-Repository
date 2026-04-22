package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import static com.qualcomm.robotcore.hardware.Servo.Direction.*;

import org.firstinspires.ftc.teamcode.config.ControllerManager;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Odometry;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.dfrobot.HuskyLens;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import org.firstinspires.ftc.robotcore.external.JavaUtil;

import java.util.Arrays;

@TeleOp(name = "BasicTeleOP")
public class BasicTeleOp extends LinearOpMode {

    Drivetrain drivetrain;
    ControllerManager controllers;
    GoBildaPinpointDriver pinpoint;

    double drive;
    double strafe;
    double turn = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        //Drivetrain Constructors
        drivetrain = new Drivetrain(hardwareMap);
        drivetrain.initOpMode(); //Initializes the drivetrain and allows for user input to start upon opmode beginning
        controllers = new ControllerManager(gamepad1, gamepad2);
        drivetrain.setSmoothMotion(false);
        drivetrain.setFieldCentricControl(false);


        //Initialization Complete
        telemetry.addLine("Welcome Aboard Captain");
        telemetry.addData("All Systems","Online");
        while (opModeIsActive()) {  //Loop while teleop is active, DO NOT EXIT

            drive  = controllers.drive(); // Drive is negative because a stick's Y-axis ranges from -1 (up stick) to 1 (down stick)
            strafe = controllers.strafe();
            turn  = controllers.turn();//smoothing only the turn, this helps give time for the turret to follow the target while turning

            drivetrain.drive(drive,strafe,turn);// Send the inputs to the drivetrain after all post-processing conditions have been checked
        }
    }
}
