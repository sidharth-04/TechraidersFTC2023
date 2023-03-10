package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.components.MecanumDriveT;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.components.Grabber;

@TeleOp
public class GrabberTest extends LinearOpMode {
    private Grabber grabber;

    @Override
    public void runOpMode() {
        grabber = new Grabber(hardwareMap);
        double x;
        double y;
        double rx;
        double robotAngle;

        waitForStart();

        while (opModeIsActive()) {
            x = gamepad1.left_stick_x;
            y = -gamepad1.left_stick_y;
            rx = gamepad1.right_stick_x;

            grabber.update();
        }
    }
}
