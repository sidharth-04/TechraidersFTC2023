package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.components.MecanumDriveT;

@TeleOp
public class TeleopDrivetrainTest extends LinearOpMode {
    private MecanumDriveT TDrive;

    @Override
    public void runOpMode() {
        TDrive = new MecanumDriveT(hardwareMap);
        double x;
        double y;
        double rx;
        double robotAngle;

        waitForStart();

        while (opModeIsActive()) {
            x = gamepad1.left_stick_x;
            y = -gamepad1.left_stick_y;
            rx = gamepad1.right_stick_x;

            TDrive.update(x, y, rx);
        }
    }
}
