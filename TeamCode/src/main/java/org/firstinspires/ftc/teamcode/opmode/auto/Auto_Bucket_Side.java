package org.firstinspires.ftc.teamcode.opmode.example;



import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.*;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;
import org.firstinspires.ftc.teamcode.config.subsystem.ClawSubsystem;

import java.util.concurrent.TimeUnit;

/**
 * This is an example auto that showcases movement and control of three servos autonomously.
 * It is able to detect the team element using a huskylens and then use that information to go to the correct spike mark and backdrop position.
 * There are examples of different ways to build paths.
 * A custom action system have been created that can be based on time, position, or other factors.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 9/8/2024
 */

@Autonomous(name = "Auto Bucket Side - Pedro Pathing", group = "Examples")
public class Auto_Bucket_Side extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState, actionState, clawState;
    private String navigation;
    public ClawSubsystem claw;


    /** Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Centerstage, this would be blue far side/red human player station.)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y. **/
    //Start Pose
    private Pose startPose = new Pose(9, 96, Point.CARTESIAN);
    private Pose scoringBucket = new Pose(14, 129, Point.CARTESIAN);
    int scoringBucketHeading = 135;

    private Pose sample1 = new Pose(36.500, 121.500, Point.CARTESIAN);

    private Pose sample2 = new Pose(36.570, 131.890, Point.CARTESIAN);

    private Pose sample3 = new Pose(45.562, 132.969, Point.CARTESIAN);
    private PathChain startToBucket;
    private PathChain bucketToSample1;

    private PathChain bucketToSample2;
    private PathChain bucketToSample3;

    private PathChain sample1toBucket;
    private PathChain sample2toBucket;
    private PathChain sample3toBucket;



            /*
    private Pose startPose = new Pose(9, 96, 90);
    //Spike mark locations
    private Pose scoringBucket = new Pose(14, 129, Math.toRadians(135));

    private Point startToBucketControlPoint = new Point(23.5, 119, Point.CARTESIAN);
    private Pose MiddleSpikeMark = new Pose(59, 94.5, Math.toRadians(270));
    private Pose RightSpikeMark = new Pose(52, 82.75, Math.toRadians(270));
    //Backdrop zone locations
    private Pose LeftBackdrop = new Pose(44, 121.75, Math.toRadians(270));
    private Pose MiddleBackdrop = new Pose(49.5, 121.75, Math.toRadians(270));
    private Pose RightBackdrop = new Pose(58, 121.25, Math.toRadians(270));
    private Pose WhiteBackdrop = new Pose(40, 122.25, Math.toRadians(270));

    // Poses and Paths for Purple and Yellow
    private Pose spikeMarkGoalPose, initialBackdropGoalPose, firstCycleStackPose, firstCycleBackdropGoalPose, secondCycleStackPose, secondCycleBackdropGoalPose;
    private Path scoreSpikeMark, initialScoreOnBackdrop, scoreSpikeMarkChosen;

    // White Stack Cycle Poses + Path Chains
    private Pose TopTruss = new Pose(28, 84, Math.toRadians(270));
    private Pose BottomTruss = new Pose(28, 36, Math.toRadians(270));
    private Pose Stack = new Pose(46, 11.5, Math.toRadians(270));
    private PathChain cycleStackTo, cycleStackBack, cycleStackToBezier;
    */

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {

        /** There are two major types of paths components: BezierCurves and BezierLines.
         *    * BezierCurves are curved, and require > 3 points. There are the start and end points, and the control points.
         *    - Control points manipulate the curve between the start and end points.
         *    - A good visualizer for this is [this](https://www.desmos.com/calculator/3so1zx0hcd).
         *    * BezierLines are straight, and require 2 points. There are the start and end points. **/

        /** This is a path chain, defined on line 66
         * It, well, chains multiple paths together. Here we use a constant heading from the board to the stack.
         * On line 97, we set the Linear Interpolation,
         * which means that Pedro will slowly change the heading of the robot from the startHeading to the endHeading over the course of the entire path */
        startToBucket = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(startPose),
                                new Point(23.500, 119.000, Point.CARTESIAN), //control point
                                new Point(scoringBucket)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(scoringBucketHeading)) // COULD BE cosntantlinearheading instead of linear not sure if it will work r not
                .setPathEndTimeoutConstraint(0)
                .build();

        bucketToSample1 = follower.pathBuilder()
                .addPath(
                        // Line 2
                        new BezierLine(
                                new Point(scoringBucket),
                                new Point(sample1)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(scoringBucketHeading), Math.toRadians(0))
                .setPathEndTimeoutConstraint(0)
                .build();

        sample1toBucket = follower.pathBuilder()
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(sample1),
                                new Point(scoringBucket)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(scoringBucketHeading))
                .setPathEndTimeoutConstraint(0)
                .build();

        bucketToSample2 = follower.pathBuilder()
                .addPath(
                        // Line 4
                        new BezierLine(
                                new Point(scoringBucket),
                                new Point(sample2)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(scoringBucketHeading), Math.toRadians(0))
                .setPathEndTimeoutConstraint(0)
                .build();

        sample2toBucket = follower.pathBuilder()
                .addPath(
                        // Line 5
                        new BezierLine(
                                new Point(sample2),
                                new Point(scoringBucket)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(scoringBucketHeading))
                .setPathEndTimeoutConstraint(0)
                .build();

        bucketToSample3 = follower.pathBuilder()
                .addPath(
                        // Line 6
                        new BezierLine(
                                new Point(scoringBucket),
                                new Point(sample3)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(scoringBucketHeading), Math.toRadians(90))
                .setPathEndTimeoutConstraint(0)
                .build();

        sample3toBucket = follower.pathBuilder()
                .addPath(
                        // Line 7
                        new BezierLine(
                                new Point(sample3),
                                new Point(scoringBucket)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(scoringBucketHeading))
                .setPathEndTimeoutConstraint(0)
                .build();
    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() function on line 193)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. **/
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 10:
                follower.followPath(scoreSpikeMark);
                setPathState(11);
                break;
            case 11:
                if (pathTimer.getElapsedTimeSeconds() > 2.6) {
                    claw.openLClaw();
                    setPathState(12);
                }
                break;
            case 12:
                if(follower.getPose().getY() > 120) {
                    setActionState(1);
                    follower.followPath(initialScoreOnBackdrop);
                    setPathState(13);
                }
                break;
        }
    }

    /** This switch is called continuously and runs the necessary actions, when finished, it will set the state to -1.
     * (Therefore, it will not run the action continuously) **/
    public void autonomousActionUpdate() {
        switch (actionState) {
            case 0:
                setClawState(0);
                setActionState(-1);
                break;
            case 1:
                setClawState(1);
                setActionState(-1);
                break;
        }
    }

    /** This switch is called continuously and runs the claw actions, when finished, it will set the state to -1.
     * (Therefore, it will not run the action continuously) **/
    public void clawUpdate() {
        switch (clawState) {
            case 0:
                claw.groundClaw();
                claw.closeClaws();
                setClawState(-1);
                break;
            case 1:
                claw.scoringClaw();
                setClawState(-1);
                break;
        }
    }


    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
        autonomousPathUpdate();
    }

    public void setActionState(int aState) {
        actionState = aState;
        pathTimer.resetTimer();
        autonomousActionUpdate();
    }

    public void setClawState(int cState) {
        clawState = cState;
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the actions and movement of the robot
        follower.update();
        autonomousPathUpdate();
        autonomousActionUpdate();
        clawUpdate();

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();

        opmodeTimer.resetTimer();

        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        claw = new ClawSubsystem(hardwareMap);

    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {
        //PUT INITIALIZATION STUFF HERE:

        // After 4 Seconds, Robot Initialization is complete
        if (opmodeTimer.getElapsedTimeSeconds() > 4) {
            telemetry.addData("Init", "Finished");
        }
    }

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        claw.closeClaws();
        setBackdropGoalPose();
        buildPaths();
        opmodeTimer.resetTimer();
        setPathState(10);
        setActionState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }
}