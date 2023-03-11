package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.components.MecanumDriveA;
import org.firstinspires.ftc.teamcode.components.Grabber;
import org.firstinspires.ftc.teamcode.components.Catcher;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.ArrayList;

@Autonomous
public class AutoDrivetrainTest extends LinearOpMode {
    private MecanumDriveA ADrive;
    private Grabber grabber;
    private Catcher catcher;
    private ElapsedTime timer = null;
    private Trajectory traj;

    private static double TILE_LENGTH = 23.5;

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    //tag IDs of the sleeve
    int Left = 17;
    int Middle = 18;
    int Right = 19;
    private int positionToPark = 0;

    AprilTagDetection tagOfInterest = null;

    public void runOpMode() {
        ADrive = new MecanumDriveA(hardwareMap);
        ADrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        grabber = new Grabber(hardwareMap);

        catcher = new Catcher(hardwareMap);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        waitForStart();

        if (opModeIsActive()) {
//            Trajectory traj = ADrive.trajectoryBuilder(new Pose2d())
//                    .strafeLeft(4)
//                    .build();
//            ADrive.followTrajectory(traj);
//            ADrive.turn(Math.toRadians(90));
//            Pose2d startPose = new Pose2d(-4, 0, Math.toRadians(90));
//            ADrive.setPoseEstimate(startPose);

            while (opModeIsActive() && positionToPark == 0) {
                ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

                if(currentDetections.size() != 0) {
                    boolean tagFound = false;
                    for(AprilTagDetection tag : currentDetections)
                    {
                        if(tag.id == Left || tag.id == Middle || tag.id == Right)
                        {
                            tagOfInterest = tag;
                            tagFound = true;
                            positionToPark = tag.id - 16;
                            break;
                        }
                    }

                    if(tagFound)
                    {
                        telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                        tagToTelemetry(tagOfInterest);
                    }
                    else
                    {
                        telemetry.addLine("Don't see tag of interest :(");

                        if(tagOfInterest == null)
                        {
                            telemetry.addLine("(The tag has never been seen)");
                        }
                        else
                        {
                            telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                            tagToTelemetry(tagOfInterest);
                        }
                    }

                    telemetry.update();

                }
            }

            if (positionToPark > 0) {
                if (positionToPark == 1) {
                    Trajectory traj1 = ADrive.trajectoryBuilder(new Pose2d())
                            .forward(TILE_LENGTH)
                            .build();
                    Trajectory traj2 = ADrive.trajectoryBuilder(traj1.end())
                            .strafeLeft(TILE_LENGTH)
                            .build();
                    ADrive.followTrajectory(traj1);
                    ADrive.followTrajectory(traj2);
                } else if (positionToPark == 3) {
                    Trajectory traj1 = ADrive.trajectoryBuilder(new Pose2d())
                            .forward(TILE_LENGTH)
                            .build();
                    Trajectory traj2 = ADrive.trajectoryBuilder(traj1.end())
                            .strafeRight(TILE_LENGTH)
                            .build();
                    ADrive.followTrajectory(traj1);
                    ADrive.followTrajectory(traj2);
                } else {
                    Trajectory traj1 = ADrive.trajectoryBuilder(new Pose2d())
                            .forward(TILE_LENGTH)
                            .build();
                    ADrive.followTrajectory(traj1);
                }
            }

            // Go to startpoint
//            traj = ADrive.trajectoryBuilder(new Pose2d())
//                    .lineToSplineHeading(new Pose2d(TILE_LENGTH*2,0,Math.toRadians(90)))
//                    .build();
//            ADrive.followTrajectory(traj);
//
//            // First setpoint
//            grabber.moveDown(490);
//            timer = new ElapsedTime();
//            while (opModeIsActive() && timer.seconds() <= 0.3) {
//                grabber.update();
//            }
//            timer = null;
//
//            // Drive to cone
//            traj = ADrive.trajectoryBuilder(new Pose2d())
//                    .forward(5)
//                    .build();
//            ADrive.followTrajectory(traj);
//
//            // Close claw
//            grabber.closeClaw();
//
//            // Auto contract
//            grabber.autoContract();
//            timer = new ElapsedTime();
//            while (opModeIsActive() && timer.seconds() <= 0.8) {
//                grabber.update();
//            }
//
//            // Drive back to junction
//            traj = ADrive.trajectoryBuilder(new Pose2d())
//                    .back(5)
//                    .build();
//            ADrive.followTrajectory(traj);
        }
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}


