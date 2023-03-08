package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.components.MecanumDriveT;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;

@Config
@TeleOp
public class TestBed extends LinearOpMode {
    private MecanumDriveT TDrive;

    private DcMotor grabberRotator;
    private DcMotor grabberLift;
    private CRServo grabberClawServo;

    private DcMotor catcherLift;
    private CRServo rotateLiftServo;
    private CRServo flipConeServo;

    // Variables to tune

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
        grabberRotator = hardwareMap.get(DcMotor.class, "grabberRotatorMotor");
        grabberLift = hardwareMap.get(DcMotor.class, "grabberLiftMotor");
        catcherLift = hardwareMap.get(DcMotor.class, "catcherLiftMotor");

        // Servo Motors
        flipConeServo = hardwareMap.get(CRServo.class, "flipConeServo");
        rotateLiftServo = hardwareMap.get(CRServo.class, "rotateLiftServo");
        grabberClawServo = hardwareMap.get(CRServo.class, "grabberClawServo");

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
            // Code for catching the cone
            if (gamepad1.a){
                grabberClawServo.setPower(0.6);
            } else if(gamepad1.x){
                grabberClawServo.setPower(-0.6);
            } else {
                grabberClawServo.setPower(0);
            }

            // Code for rotating the grabber
            if(gamepad1.right_bumper) {
                grabberRotator.setPower(-0.4);
            }
            else if (gamepad1.left_bumper) {
                grabberRotator.setPower(0.4);
            } else {
                grabberRotator.setPower(0.0);
            }
             telemetry.addData("grabber rotator pos", grabberRotator.getCurrentPosition());

            // Code for controlling the grabber lift
            if (gamepad1.right_trigger > 0){
                grabberLift.setPower(0.6);
            } else if(gamepad1.left_trigger > 0){
                grabberLift.setPower(-0.6);
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
//            if (gamepad2.right_bumper){
//                flipConeServo.setPower(0.6);
//            } else if(gamepad2.left_bumper){
//                flipConeServo.setPower(-0.6);
//            } else {
//                flipConeServo.setPower(0);
//            }

            // Code for controlling the catcher lift
            if (gamepad2.right_trigger > 0){
                catcherLift.setPower(0.6);
            } else if(gamepad2.left_trigger > 0){
                catcherLift.setPower(-0.6);
            } else {
                catcherLift.setPower(0);
            }
            telemetry.addData("catcher lift pos", catcherLift.getCurrentPosition());

            // Update Telemetry
            telemetry.update();
        }
    }
}
