package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.components.MecanumDriveA;

@Autonomous
public class AutoVideo extends LinearOpMode {
    private MecanumDriveA ADrive;
    private Trajectory traj;

    private static double TILE_LENGTH = 23.5;


    public void runOpMode() {
        ADrive = new MecanumDriveA(hardwareMap);
        ADrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        if (opModeIsActive()) {
            Trajectory traj = ADrive.trajectoryBuilder(new Pose2d())
                    .lineToSplineHeading(new Pose2d(TILE_LENGTH * 2, 0, Math.toRadians(-90)))
                    .build();
            ADrive.followTrajectory(traj);
        }
    }
}