package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.components.MecanumDriveT;

import com.arcrobotics.ftclib.controller.wpilibcontroller.ArmFeedforward;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp
public class TestBed extends LinearOpMode {
    private MecanumDriveT TDrive;

    private DcMotorEx grabberRotator;
    private DcMotor grabberLift;
    private Servo grabberClawServo;

    private DcMotor catcherLift;
    private Servo rotateLiftServo;
    private Servo flipConeServo;

    private double rotationServoPos = 0.5;

    // Variables to tune
    private ArmFeedforward feedforward;
    private PIDController controller;
    public static double kS = 0, kCos = 0.1, kV = 0.6, kA = 0;
    public static double feedforwardTarget = 0.0;
    public static double feedforwardVel = 0.8;
    public static double feedforwardAcc = 0;
    private static double p = 0.008, i = 0, d = 0.0002, f = 0.1;
    public static double ticks_in_degrees = (28 * 45.0/125.0) / 360.0;
    private int grabberState = 0;
    private int direction = 0;
    private int downCheckPoint = 450;
    private int upCheckPoint = 250;

    @Override
    public void runOpMode() {
        // Drivetrain Variables
        TDrive = new MecanumDriveT(hardwareMap);
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
        feedforward = new ArmFeedforward(kS, kCos, kV, kA);
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Wait for player to press start
        waitForStart();

        grabberClawServo.setPosition(0.7);
        flipConeServo.setPosition(1);
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
            if(gamepad1.a) {
                // Coming down
                grabberState = 1;
                direction = 1;
                kV = 0.6;
                kA = 0.0;
                feedforwardVel = 0.8;
                feedforwardAcc = 0;
                feedforward = new ArmFeedforward(kS, kCos, kV, kA);
            }
            if(gamepad1.y) {
                // Coming up
                grabberState = 1;
                direction = -1;
                kV = -0.6;
                kA = 0.05;
                feedforwardVel = 0.8;
                feedforwardAcc = 0;
                feedforward = new ArmFeedforward(kS, kCos, kV, kA);
            }

            //Feedforward
            if (grabberState == 1) {
                grabberRotator.setPower(feedforward.calculate(Math.toRadians(feedforwardTarget), feedforwardVel, feedforwardAcc));
                if (direction == 1 && grabberRotator.getCurrentPosition() >= downCheckPoint) {
                    kV = -0.6;
                    kA = 0;
                    feedforwardVel = 0.4;
                    feedforwardAcc = 0.06;
                    feedforward = new ArmFeedforward(kS, kCos, kV, kA);
                    if (grabberRotator.getCurrentPosition() >= 500) {
                        grabberState = 0;
                        direction = 0;
                    }
                } else if (direction == -1 && grabberRotator.getCurrentPosition() <= upCheckPoint) {
                    grabberState = 2;
                }
            }

            // PID
            if (grabberState == 2) {
                int armPos = grabberRotator.getCurrentPosition();
                int pidTarget = 90;
                double pid = controller.calculate(armPos, pidTarget);
                double ff = Math.cos(Math.toRadians(pidTarget / ticks_in_degrees)) * f;
                double power = pid + ff;
                grabberRotator.setPower(power);
                telemetry.addData("measured pos", armPos);
                telemetry.addData("target pos", pidTarget);
                telemetry.update();
            }

            // Claw catching
            if(gamepad1.right_bumper) {
                grabberClawServo.setPosition(0.7);
            }
            else if (gamepad1.left_bumper) {
                grabberClawServo.setPosition(0.4);
            }

            // Code for controlling the grabber lift
//            if (gamepad1.right_trigger > 0) {
//                grabberLift.setPower(0.6);
//            } else if (gamepad1.left_trigger > 0) {
//                grabberLift.setPower(-0.6);
//            } else {
//                grabberLift.setPower(0);
//            }

            if (gamepad1.right_trigger > 0) {
                if (grabberLift.getCurrentPosition() <= 3000) {
                    grabberLift.setPower(0.8);
                    telemetry.addData("WARNING WARNING",grabberLift.getCurrentPosition() );
                    telemetry.update();
                } else {
                    grabberLift.setPower(0);
                    telemetry.addData("WARNING WARNING", "You've reached Max!!!!" );
                    telemetry.update();
                }
            }
            else if (gamepad1.left_trigger > 0) {
                if (grabberLift.getCurrentPosition() > 0) {
                    grabberLift.setPower(-0.8);
                    telemetry.addData("WARNING WARNING",grabberLift.getCurrentPosition() );
                    telemetry.update();
                } else {
                    grabberLift.setPower(0);
                    telemetry.addData("WARNING WARNING", "You've reached Min!!!!" );
                    telemetry.update();
                }
            } else {
                grabberLift.setPower(0);
            }
            telemetry.addData("grabber lift pos", grabberLift.getCurrentPosition());

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
            rotateLiftServo.setPosition(rotationServoPos);

            // Code for flipping the cone
            if (gamepad2.right_bumper) {
                flipConeServo.setPosition(0);
            } else if (gamepad2.left_bumper) {
                flipConeServo.setPosition(0.95);
            }

//            if (gamepad2.right_trigger > 0) {
//                catcherLift.setPower(0.6);
//            } else if (gamepad2.left_trigger > 0) {
//                catcherLift.setPower(-0.6);
//            } else {
//                catcherLift.setPower(0);
//            }

            if (gamepad2.right_trigger > 0) {
                if (catcherLift.getCurrentPosition() <= 4950) {
                    catcherLift.setPower(0.6);
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
                    catcherLift.setPower(-0.6);
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
            telemetry.addData("catcher lift pos", catcherLift.getCurrentPosition());

            // Update Telemetry
            telemetry.update();
        }
    }
}