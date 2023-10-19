package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;

@Autonomous(name = "Main LEFT Auto (1+2)")
public class MainAuto1 extends OpMode {

    AprilTag pipeline = null;
    int detectionID;

    SampleMecanumDrive drive = null;
    Pose2d startPose = null;
    Pose2d postPreScorePose = null;
    Pose2d poseEstimate = null;
    Pose2d grabConePose = null;

    Trajectory trajTest = null;
    TrajectorySequence trajSeqPreScore;
    TrajectorySequence trajCycle;
    Trajectory traj0;
    Trajectory traj2;

    enum State {
        TRAJPREPLACED,
        TRAJCYCLE,
        TRAJCYCLE1,
        PARK,
        IDLE
    }

    State currentstate = State.IDLE;

    Pose2d testStartPose = null;
    Pose2d postScorePose = null;

    boolean liftUpdate = true;

    boolean coneGrabbed = false;

    Robot robot = new Robot();

    @Override
    public void init() {
        OpenCvWebcam webcam;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        pipeline = new AprilTag();
        webcam.setPipeline(pipeline);

        Robot robot = new Robot();

        int[] IDsofInterest = {0, 1, 2};
        int detectionID = 0;
        boolean tagFound = false;



        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
        startPose = new Pose2d(-36, -64.5, Math.toRadians(90));
        postPreScorePose = new Pose2d(-36, -12, Math.toRadians(90));
        grabConePose = new Pose2d(-56, -12, Math.toRadians(180));

        testStartPose = new Pose2d(0, 0, Math.toRadians(0));
        postScorePose = new Pose2d(-36, -12, Math.toRadians(90));
        //Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));

        drive = new SampleMecanumDrive(hardwareMap);
        //robot.init(hardwareMap, telemetry);
        drive.setPoseEstimate(startPose);

        trajSeqPreScore = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(.5)
                //.forward(8)
                //.turn(Math.toRadians(-90))
                //.lineTo(new Vector2d(-34, -2))
                .lineTo(new Vector2d(-34, -8.5))
                .turn(Math.toRadians(-43))
                /*.addTemporalMarker(9, () -> {
                    robot.lift(.6, 4200);
                    robot.liftPower(.0275);
                })*/
                //.lineTo(new Vector2d(-66, -38))
                .forward(6)
                .addDisplacementMarker(robot::openClaw)
                .back(6)
                /*.addTemporalMarker(13, () -> {
                    robot.openClaw();
                })*/
                .lineToLinearHeading(postPreScorePose)
                //.addDisplacementMarker(() -> drive.followTrajectorySequence(trajCycle))
                //.addDisplacementMarker(() ->)
                .build();

        trajCycle = drive.trajectorySequenceBuilder(postPreScorePose)
                .lineToLinearHeading(grabConePose)
                .waitSeconds(.2)
                .forward(6)
                .addTemporalMarker(2.9, () -> {
                    robot.closeClaw();
                })
                .addTemporalMarker(3.3, () -> {
                    coneGrabbed = true;
                })
                .waitSeconds(.5)
                //code to set lift to correct height and grab cone
                .lineToLinearHeading(new Pose2d(-34, -8.5, Math.toRadians(43)))
                //code to set lift to correct height
                .waitSeconds(.2)
                .forward(5)
                .addTemporalMarker(6, () -> {coneGrabbed = false;})
                .addDisplacementMarker(robot::openClaw)
                .waitSeconds(0.2)
                //code to drop cone
                .lineToLinearHeading(postPreScorePose)
                .build();
        traj0 = drive.trajectoryBuilder(postScorePose)
                .strafeLeft(30)
                .build();
        traj2 = drive.trajectoryBuilder(postScorePose)
                .strafeRight(30)
                .build();

         /*trajTest = drive.trajectoryBuilder(testStartPose)
                .strafeLeft(24)
                .build();*/

        //drive.followTrajectoryAsync(trajTest);
        drive.followTrajectorySequenceAsync(trajSeqPreScore);
        drive = new SampleMecanumDrive(hardwareMap);
        robot.init(hardwareMap, telemetry);
    }

    @Override
    public void init_loop() {
        robot.init(hardwareMap, telemetry);
        ArrayList<AprilTagDetection> detections = pipeline.getLatestDetections();
        for (AprilTagDetection detection : detections) {
            detectionID = detection.id;
            telemetry.addData("DetectionID", detectionID);
            telemetry.addData("detection.id", detection.id);
            telemetry.update();
            sleep(50);
        }

    }

    @Override
    public void start() {
        if(detectionID == 0) {}
        if(detectionID == 1) {}
        if(detectionID == 2) {}
        drive.setPoseEstimate(startPose);
        currentstate = State.TRAJPREPLACED;
        drive.followTrajectorySequenceAsync(trajSeqPreScore);
        //drive.followTrajectorySequenceAsync(trajCycle);
        robot.closeClaw();

    }

    @Override
    public void loop() {

        switch(currentstate){
            case TRAJPREPLACED:
                telemetry.addData("Flag 1: ", "TRAJPREPLACED");
                telemetry.update();
                if(!drive.isBusy()) {
                    telemetry.addData("FLag 2: ", "drive is not busy, TRAJPREPLACED");
                    telemetry.update();
                    currentstate = State.TRAJCYCLE;
                    drive.followTrajectorySequenceAsync(trajCycle);
                }
                break;

            case TRAJCYCLE:
                telemetry.addData("Flag 3: ", "TRAJCYCLE");
                telemetry.update();
                if(!drive.isBusy()){
                    telemetry.addData("Flag 4: ", "drive is not busy, TRAJCYCLE");
                    telemetry.update();
                    currentstate = State.TRAJCYCLE1;
                    drive.followTrajectorySequenceAsync(trajCycle);
                }
                break;

            case TRAJCYCLE1:
                if(!drive.isBusy()){
                    if(!drive.isBusy()){
                        robot.lift(.8, 500);
                        liftUpdate = false;
                        if(detectionID == 0){
                            drive.followTrajectoryAsync(traj0);
                        }
                        if(detectionID == 2){
                            drive.followTrajectoryAsync(traj2);
                        }
                        currentstate = State.PARK;
                    }
                }

            case PARK:
                if(!drive.isBusy()){
                    currentstate = State.IDLE;
                }

            case IDLE:
                telemetry.addData("Flag 5: ", "IDLE");
                telemetry.update();
                break;
        }

        drive.update();
        poseEstimate = drive.getPoseEstimate();
        telemetry.addData("Pose X", poseEstimate.getX());
        telemetry.addData("Pose Y", poseEstimate.getY());
        telemetry.addData("Heading", poseEstimate.getHeading());

        poseEstimate = drive.getPoseEstimate();
        if(liftUpdate){
            robot.liftUpdateLeft(poseEstimate, coneGrabbed);
        }
    }
}
