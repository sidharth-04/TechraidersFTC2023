package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;


public class MecanumDriveT {
    private boolean fieldCentric = true;
    private DcMotorEx frontLeft, backLeft, backRight, frontRight;
    private IMU imu;

    public MecanumDriveT(HardwareMap hardwareMap) {
        // Setup up imu
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                DriveConstants.LOGO_FACING_DIR, DriveConstants.USB_FACING_DIR));
        imu.initialize(parameters);

        // Setup motors
        frontLeft = hardwareMap.get(DcMotorEx.class, "B0");
        frontRight = hardwareMap.get(DcMotorEx.class,"A0");
        backRight = hardwareMap.get(DcMotorEx.class,"A1");
        backLeft = hardwareMap.get(DcMotorEx.class,"B1");

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setFieldCentric(boolean fieldCentric) {
        this.fieldCentric = fieldCentric;
    }

    public void update(double x, double y, double rx) {
        double robotAngle = this.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double frontLeftPower;
        double backLeftPower;
        double frontRightPower;
        double backRightPower;

        if (this.fieldCentric) {
            // Calculate the rotated x and y
            double rotX = x * Math.cos(-robotAngle) - y * Math.sin(-robotAngle);
            double rotY = x * Math.sin(-robotAngle) + y * Math.cos(-robotAngle);

            frontLeftPower = (rotY*Math.abs(rotY) + rotX*Math.abs(rotX) + rx);
            backLeftPower = (rotY*Math.abs(rotY) - rotX*Math.abs(rotX) + rx);
            frontRightPower = (rotY*Math.abs(rotY)  - rotX*Math.abs(rotX) - rx);
            backRightPower = (rotY*Math.abs(rotY)  + rotX*Math.abs(rotX) - rx);
        } else {
            frontLeftPower = (y*Math.abs(y) + x*Math.abs(x) + rx);
            backLeftPower = (y*Math.abs(y) - x*Math.abs(x) + rx);
            frontRightPower = (y*Math.abs(y)  - x*Math.abs(x) - rx);
            backRightPower = (y*Math.abs(y)  + x*Math.abs(x) - rx);
        }

        this.frontLeft.setPower(frontLeftPower);
        this.backLeft.setPower(backLeftPower);
        this.frontRight.setPower(frontRightPower);
        this.backRight.setPower(backRightPower);
    }
}
