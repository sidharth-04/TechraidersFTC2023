package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.components.MecanumDriveT;
import org.firstinspires.ftc.teamcode.components.Grabber;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp
public class TestBed extends LinearOpMode {
    private MecanumDriveT TDrive;
    private Grabber grabber;

    private DcMotorEx grabberRotator;
    private DcMotor grabberLift;
    private Servo grabberClawServo;

    private DcMotor catcherLift;
    private Servo rotateLiftServo;
    private Servo flipConeServo;

    public static double rotationServoPos = 0.5;
    public static double flipConeServPos = 1;

    @Override
    public void runOpMode() {
        // Drivetrain Variables
        TDrive = new MecanumDriveT(hardwareMap);
        grabber = new Grabber(hardwareMap);
        double x;
        double y;
        double rx;

        // HD Hex Motors
        grabberRotator = hardwareMap.get(DcMotorEx.class, "grabberRotatorMotor");
        grabberLift = hardwareMap.get(DcMotor.class, "grabberLiftMotor");
        catcherLift = hardwareMap.get(DcMotor.class, "catcherLiftMotor");

        // Servo Motors
        grabberClawServo = hardwareMap.get(Servo.class, "grabberClawServo");
        flipConeServo = hardwareMap.get(Servo.class, "flipConeServo");
        rotateLiftServo = hardwareMap.get(Servo.class, "rotateLiftServo");

        // Tuning Variables
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Wait for player to press start
        waitForStart();

        grabberClawServo.setPosition(0.7);
        flipConeServo.setPosition(flipConeServPos);
        rotateLiftServo.setPosition(rotationServoPos);

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
                    grabber.moveUp(90);
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
                }
                else if (gamepad1.left_trigger > 0) {
                    grabber.contractLift();
                } else {
                    grabber.stationLift();
                }
            }
            grabber.update();
            telemetry.addData("graber pos", grabber.getPos());
            telemetry.update();

            // ----------------------------------
            // CATCHER
            // ----------------------------------
            // Code for rotating the catcher
            if (gamepad2.dpad_right) {
                rotationServoPos += 0.01;
                if (rotationServoPos >= 1) {
                    rotationServoPos = 1;
                }
            } else if (gamepad2.dpad_left) {
                rotationServoPos -= 0.01;
                if (rotationServoPos <= 0) {
                    rotationServoPos = 0;
                }
            }
            if (gamepad2.a) {
                rotationServoPos = 0.4;
            }
            rotateLiftServo.setPosition(rotationServoPos);

            // Code for flipping the cone
            if (gamepad2.right_bumper) {
                flipConeServPos = 0;
            } else if (gamepad2.left_bumper || flipConeServo.getPosition() == 0) {
                flipConeServPos = 0.9;
            }
            flipConeServo.setPosition(flipConeServPos);

//            if (gamepad2.right_trigger > 0) {
//                catcherLift.setPower(0.6);
//            } else if (gamepad2.left_trigger > 0) {
//                catcherLift.setPower(-0.6);
//            } else {
//                catcherLift.setPower(0);
//            }

            if (gamepad2.right_trigger > 0) {
                if (catcherLift.getCurrentPosition() <= 4950) {
                    catcherLift.setPower(0.8);
                    telemetry.addData("WARNING WARNING",catcherLift.getCurrentPosition() );
                    telemetry.update();
                } else {
                    catcherLift.setPower(0);
                    telemetry.addData("WARNING WARNING", "You've reached Max!!!!" );
                    telemetry.update();
                }
            }
            else if (gamepad2.left_trigger > 0) {
                if (catcherLift.getCurrentPosition() > 0) {
                    catcherLift.setPower(-0.8);
                    telemetry.addData("WARNING WARNING",catcherLift.getCurrentPosition());
                    telemetry.update();
                } else {
                    catcherLift.setPower(0);
                    telemetry.addData("WARNING WARNING", "You've reached Min!!!!" );
                    telemetry.update();
                }
            } else {
                catcherLift.setPower(0);
            }
        }
    }
}