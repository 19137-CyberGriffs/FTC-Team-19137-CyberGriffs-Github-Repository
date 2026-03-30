/*
Copyright 2026 FIRST Tech Challenge Team 19137

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.Odometry;
import org.firstinspires.ftc.teamcode.util.Path;
import org.firstinspires.ftc.teamcode.util.Waypoint;

@Autonomous(name="BasicAuto")
public class BasicAuto extends LinearOpMode {

    DecodeTeleOp.Alliance alliance =  DecodeTeleOp.Alliance.BLUE;

    Drivetrain drivetrain;
    Odometry odometry;
    Path path;

    double powerCorrectionCoe;

    double period = 1000/16.0;
    double beginTime = getRuntime()*1000;
    double currentTime;

    @Override
    public void runOpMode() throws InterruptedException{

        //Drivetrain Constructors
        drivetrain = new Drivetrain(hardwareMap);
        drivetrain.initOpMode();
        drivetrain.setSmoothMotion(true);

        //Odometry Constructors
        odometry = new Odometry(new Pose2D(DistanceUnit.METER,0,0,AngleUnit.DEGREES,0), hardwareMap);

        //Path Constructors
        path = new Path(getRuntime(),hardwareMap);
        path.setTraceMode(Path.traceMode.STOP_WHEN_COMPLETE);
        path.setTolerance(5,1);

        //*   insert constructors for motors, servos, camera, or others here   *//

        // Tuning/Default sequence
        path.addWaypoint(new Waypoint(0,0,0));
        path.addWaypoint(new Waypoint(1,1,90));
        path.addWaypoint(new Waypoint(1,0,-90));
        path.addWaypoint(new Waypoint(0.5,0.5,-45));
        path.addWaypoint(new Waypoint(0,1,0));
        path.addWaypoint(new Waypoint(0,0,0));

        //Confirming initialization
        telemetry.addLine("Welcome Aboard Captain,");
        telemetry.addData("All Systems", "Online");//Subnautica hehe
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive() && path.isRunning()) {

            //===Odometry===//
            odometry.updatePosition();
            path.update(odometry.getRoboPosition(),getRuntime());

            //===Drivetrain===//
            drivetrain.drive(path.getSuggestedDrive(), path.getSuggestedStrafe(), path.getSuggestedTurn());

            //===Actions to take while reaching current waypoint===//

            // Defaults
            path.setPower(0.7);
            path.setSpeedMode(Path.Ahead.STANDARD);

            // Waypoint specific conditions
            switch (path.getCurrentWaypoint()) {
                case 4: case 5:
                    path.setSpeedMode(Path.Ahead.SLOW);
                    break;
                case 3:
                    path.setSpeedMode(Path.Ahead.FLANK);
            }

            //*   insert other systems here   *//

            //===Telemetry===//
            telemetry.addData("Current Waypoint ID", path.getCurrentWaypoint());
            telemetry.addData("Heading (degrees)", odometry.getRoboPosition().getHeading(AngleUnit.DEGREES));
            telemetry.addData("X (meters)", odometry.getRoboPosition().getX(DistanceUnit.METER));
            telemetry.addData("Y (meters)", odometry.getRoboPosition().getY(DistanceUnit.METER));
            telemetry.addData("drive", path.getSuggestedDrive());
            telemetry.addData("strafe", path.getSuggestedStrafe());
            telemetry.addData("turn", path.getSuggestedTurn());

            telemetry.update();

            //===Thread Control===//
            currentTime = getRuntime()*1000;
            long sleepTime = Math.round(period - (currentTime - beginTime));
            if (sleepTime > 0) sleep(sleepTime);
            beginTime = getRuntime()*1000;
        }
    }
}
