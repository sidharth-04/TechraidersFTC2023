package org.firstinspires.ftc.teamcode.testing;

import com.arcrobotics.ftclib.controller.wpilibcontroller.ArmFeedforward;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.teamcode.components.MecanumDriveT;
import org.firstinspires.ftc.teamcode.components.Grabber;
import org.firstinspires.ftc.teamcode.components.Catcher;

import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class TestBed extends LinearOpMode {
    private MecanumDriveT TDrive;
    private Grabber grabber;
    private Catcher catcher;

    @Override
    public void runOpMode() {
        // Drivetrain Variables
        TDrive = new MecanumDriveT(hardwareMap);
        grabber = new Grabber(hardwareMap);
        catcher = new Catcher(hardwareMap);
        double x;
        double y;
        double rx;

        // Tuning Variables
        // Wait for player to press start
        waitForStart();

        grabber.init();
        catcher.init();

        while (opModeIsActive()) {
            // ----------------------------------
            // DRIVETRAIN
            // ----------------------------------
            x = gamepad1.left_stick_x;
            y = -gamepad1.left_stick_y;
            rx = gamepad1.right_stick_x;
            TDrive.update(x, y, rx);

            // ----------------------------------
            // GRABBER
            // ----------------------------------
            // Code for rotating the grabber
            if (grabber.notInAuto()) {
                if (gamepad1.a) {
                    grabber.moveDown(580);
                }
                if (gamepad1.y) {
                    grabber.moveUp(80);
                }
                if (gamepad1.start) {
                    grabber.autoContract();
                }

                // Claw catching
                if(gamepad1.right_bumper) {
                    grabber.openClaw();
                } else if (gamepad1.left_bumper) {
                    grabber.closeClaw();
                }

                // Code for controlling the grabber lift
                if (gamepad1.right_trigger > 0) {
                    grabber.extendLift();
                    telemetry.addData("grabberLift", grabber.getPos());
                    telemetry.update();
                }
                else if (gamepad1.left_trigger > 0) {
                    grabber.contractLift();
                    telemetry.addData("grabberLift", grabber.getPos());
                    telemetry.update();
                } else {
                    grabber.stationLift();
                }
            }
            grabber.update();

            // ----------------------------------
            // CATCHER
            // ----------------------------------
            // Code for rotating the catcher
            if (gamepad2.right_stick_x > 0.0) {
                catcher.increaseRotation();
            } else if (gamepad2.right_stick_x < 0.0) {
                catcher.decreaseRotation();
            }
            if (gamepad2.a) {
                catcher.setEasyCatch();
            }
            if (gamepad2.b) {
                catcher.setTeleDrop();

            }

            // Code for flipping the cone
            if (gamepad2.right_bumper) {
                catcher.dumpCone();
            } else if (gamepad2.left_bumper || catcher.getFlipPos() == 0) {
                catcher.resetCatcher();
            }

            // Extend contract lift
            if (gamepad2.right_trigger > 0) {
                catcher.extendLift();
            }
            else if (gamepad2.left_trigger > 0) {
                catcher.contractLift();
            } else {
                catcher.stationLift();
            }
        }
    }
}