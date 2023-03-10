package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
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
    private CRServo grabberClawServo;
    private DcMotor catcherLift;
    private CRServo rotateLiftServo;
    private CRServo flipConeServo;

    // Variables to tune
    private ArmFeedforward feedforward;
    private PIDController controller;
    public static double kS = 0, kCos = 0.1, kV = 0.1, kA = 0;
    public static double feedforwardTarget = 0.0;
    public static double feedforwardVel = 0.8;
    public static double feedforwardAcc = 0;
    private static double p = 0.008, i = 0, d = 0.0002, f = 0.1;
    public static double ticks_in_degrees = (28 * 45.0/125.0) / 360.0;
    private int grabberState = 0;
    private int direction = 0;
    private int downCheckPoint = 350;
    private int upCheckPoint = 250;
    private ElapsedTime bufferTimer = null;

    @Override
    public void runOpMode() {
        // Drivetrain Variables
        TDrive = new MecanumDriveT(hardwareMap);
        double x;
        double y;
        double rx;

        // Teleop Variables
        double rotationServoPower = 0;

        // HD Hex Motors
        grabberRotator = hardwareMap.get(DcMotorEx.class, "grabberRotatorMotor");
        grabberLift = hardwareMap.get(DcMotor.class, "grabberLiftMotor");
        catcherLift = hardwareMap.get(DcMotor.class, "catcherLiftMotor");

        // Servo Motors
        flipConeServo = hardwareMap.get(CRServo.class, "flipConeServo");
        rotateLiftServo = hardwareMap.get(CRServo.class, "rotateLiftServo");
        grabberClawServo = hardwareMap.get(CRServo.class, "grabberClawServo");

        // Tuning Variables
        feedforward = new ArmFeedforward(kS, kCos, kV, kA);
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Wait for player to press start
        waitForStart();

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
            }
            if(gamepad1.y) {
                // Coming up
                grabberState = 1;
                direction = -1;
                kV = -0.6;
            }

            //Feedforward
            if (grabberState == 1) {
                bufferTimer = null;
                feedforward = new ArmFeedforward(kS, kCos, kV, kA);
                grabberRotator.setPower(feedforward.calculate(Math.toRadians(feedforwardTarget), feedforwardVel, feedforwardAcc));
                if (direction == 1 && grabberRotator.getCurrentPosition() >= downCheckPoint) {
                    grabberState = 2;
                } else if (direction == -1 && grabberRotator.getCurrentPosition() <= upCheckPoint) {
                    grabberState = 2;
                }
            }

            // PID
            if (grabberState == 2) {
                int armPos = grabberRotator.getCurrentPosition();
                int pidTarget = 300;
                if (direction == 1) {
                    if (bufferTimer == null) {
                        pidTarget = 400;
                        if (armPos - pidTarget <= 25) {
                            bufferTimer = new ElapsedTime();
                        }
                    } else {
                        if (bufferTimer.seconds() > 1.2) {
                            pidTarget = 580;
                        }
                    }
                } else {
                    pidTarget = 90;
                }
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
                grabberClawServo.setPower(0.6);
            }
            else if (gamepad1.left_bumper) {
                grabberClawServo.setPower(-0.6);
            } else {
                grabberClawServo.setPower(0.0);
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
                    grabberLift.setPower(0.6);
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
                    grabberLift.setPower(-0.6);
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
            rotationServoPower = -gamepad2.right_stick_x / 2;
            rotateLiftServo.setPower(rotationServoPower);

            // Code for flipping the cone
            if (gamepad2.right_bumper){
                flipConeServo.setPower(0.6);
            } else if(gamepad2.left_bumper){
                flipConeServo.setPower(-0.6);
            } else {
                flipConeServo.setPower(0);
            }

//            if (gamepad2.right_trigger > 0) {
//                catcherLift.setPower(0.6);
//            } else if (gamepad2.left_trigger > 0) {
//                catcherLift.setPower(-0.6);
//            } else {
//                catcherLift.setPower(0);
//            }

            if (gamepad2.right_trigger > 0) {
                if (catcherLift.getCurrentPosition() <= 4450) {
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
