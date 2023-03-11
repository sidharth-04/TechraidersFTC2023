package org.firstinspires.ftc.teamcode.components;

import com.arcrobotics.ftclib.controller.wpilibcontroller.ArmFeedforward;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Catcher {
    // Actuators
    private DcMotorEx catcherLift;
    private Servo flipConeServo;
    private Servo rotateLiftServo;

    private static double rotationServoPos = 0.5;
    private static double flipConeServoPos = 1;

    public Catcher(HardwareMap hardwareMap) {
        catcherLift = hardwareMap.get(DcMotorEx.class, "catcherLiftMotor");
        flipConeServo = hardwareMap.get(Servo.class, "flipConeServo");
        rotateLiftServo = hardwareMap.get(Servo.class, "rotateLiftServo");
    }

    public void init() {
//        flipConeServo.setPosition(flipConeServoPos);
//        rotateLiftServo.setPosition(rotationServoPos);
    }

    public void increaseRotation() {
        rotationServoPos += 0.01;
        if (rotationServoPos >= 1) {
            rotationServoPos = 1;
        }
        rotateLiftServo.setPosition(rotationServoPos);
    }
    public void decreaseRotation() {
        rotationServoPos -= 0.01;
        if (rotationServoPos <= 0) {
            rotationServoPos = 0;
        }
        rotateLiftServo.setPosition(rotationServoPos);
    }

    public void dumpCone() {
        flipConeServoPos = 0;
        flipConeServo.setPosition(flipConeServoPos);
    }
    public void resetCatcher() {
        flipConeServoPos = 0.9;
        flipConeServo.setPosition(flipConeServoPos);
    }
    public double getFlipPos() {
        return flipConeServo.getPosition();
    }

    public void setEasyCatch() {
        rotationServoPos = 0.4;
        rotateLiftServo.setPosition(rotationServoPos);
    }
    public void setAutoDrop() {
        rotationServoPos = 0.34;
        rotateLiftServo.setPosition(rotationServoPos);
    }
    public void setTeleDrop() {
        rotationServoPos = 0.67;
        rotateLiftServo.setPosition(rotationServoPos);
    }

    public void extendLift() {
        if (catcherLift.getCurrentPosition() <= 4800) {
            catcherLift.setPower(0.8);
        } else {
            catcherLift.setPower(0);
        }
    }
    public void contractLift() {
        if (catcherLift.getCurrentPosition() >= 0) {
            catcherLift.setPower(-0.8);
        } else {
            catcherLift.setPower(0);
        }
    }
    public void stationLift() {
        catcherLift.setPower(0);
    }
}
