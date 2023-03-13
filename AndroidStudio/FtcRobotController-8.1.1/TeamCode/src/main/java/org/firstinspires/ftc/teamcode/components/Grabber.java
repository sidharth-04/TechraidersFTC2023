package org.firstinspires.ftc.teamcode.components;

import com.arcrobotics.ftclib.controller.wpilibcontroller.ArmFeedforward;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;

public class Grabber {
    // Actuators
    private DcMotorEx grabberRotator;
    private DcMotorEx grabberLift;
    private Servo claw;

    // Feedforward
    private static double kS = 0, kCos = 0.1, kV = 0.6, kA = 0;
    private ArmFeedforward feedforward = new ArmFeedforward(kS, kCos, kV, kA);
    private static double feedforwardTarget = 0.0;
    private static double feedforwardVel = 0.8;
    private static double feedforwardAcc = 0;

    // PID
    private static double p = 0.008, i = 0, d = 0.0002, f = 0.1;
    private PIDController controller = new PIDController(p, i, d);
    private static double ticks_in_degrees = (28 * 45.0/125.0) / 360.0;
    private static int holdValue = 0;

    // Control Switching
    private int grabberState = 0;
    private int direction = 0;
    private int downCheckPoint = 450;
    private int upCheckPoint = 250;

    // AutoContract Feature
    private int autoContractState = 0;
    private ElapsedTime stateTimer = null;

    public Grabber(HardwareMap hardwareMap) {
        this.grabberRotator = hardwareMap.get(DcMotorEx.class, "grabberRotatorMotor");
        this.grabberLift = hardwareMap.get(DcMotorEx.class, "grabberLiftMotor");
        this.claw = hardwareMap.get(Servo.class, "grabberClawServo");

        this.claw.setPosition(0.7);
    }

    public void init() {
//        claw.setPosition(0.7);
    }

    public void autoContract() {
        if (grabberState == 1) {
            return;
        }
        if (grabberRotator.getCurrentPosition() < upCheckPoint) {
            return;
        }
        autoContractState = 1;
        moveUp(80);
    }
    public boolean notInAuto() {
        if (autoContractState == 0) {
            return true;
        }
        return false;
    }

    public void moveDown(int setpoint) {
        if (grabberState == 1) {
            return;
        }
        // Failsafe if already down
        if (grabberRotator.getCurrentPosition() > downCheckPoint) {
            return;
        }
        grabberState = 1;
        direction = 1;
        kV = 0.6;
        kA = 0.0;
        feedforwardVel = 0.8;
        feedforwardAcc = 0;
        feedforward = new ArmFeedforward(kS, kCos, kV, kA);
        holdValue = setpoint;
    }

    public void moveUp(int setpoint) {
        if (grabberState == 1) {
            return;
        }
        // Failsafe if already up
        if (grabberRotator.getCurrentPosition() < upCheckPoint) {
            return;
        }
        if (grabberLift.getCurrentPosition() > 200) {
            return;
        }
        grabberState = 1;
        direction = -1;
        kV = -0.6;
        kA = 0.0;
        feedforwardVel = 1;
        feedforwardAcc = 0;
        feedforward = new ArmFeedforward(kS, kCos, kV, kA);
        holdValue = setpoint;
    }

    public int getPos() {
        return grabberLift.getCurrentPosition();
    }
    public void setPower(double power) {
        grabberRotator.setPower(power);
    }
    public boolean atSetpoint() {
        if (Math.abs(grabberRotator.getCurrentPosition() - holdValue) < 10) {
            return true;
        }
        return false;
    }

    public void openClaw() {
        claw.setPosition(0.7);
    }
    public void closeClaw() {
        claw.setPosition(0.4);
    }

    public void extendLift() {
        if (grabberLift.getCurrentPosition() <= 2050) {
            grabberLift.setPower(1);
        } else {
            grabberLift.setPower(0);
        }
    }
    public void contractLift() {
        if (grabberLift.getCurrentPosition() > 0) {
            grabberLift.setPower(-1);
        } else {
            grabberLift.setPower(0);
        }
    }
    public void stationLift() {
        grabberLift.setPower(0);
    }

    public void update() {
        if (grabberState == 1) {
            grabberRotator.setPower(feedforward.calculate(Math.toRadians(feedforwardTarget), feedforwardVel, feedforwardAcc));
            if (direction == 1 && grabberRotator.getCurrentPosition() >= downCheckPoint) {
                kV = -0.6;
                kA = 0;
                feedforwardVel = 0.4;
                feedforwardAcc = 0.06;
                feedforward = new ArmFeedforward(kS, kCos, kV, kA);
                if (grabberRotator.getCurrentPosition() >= 500) {
                    grabberState = 2;
                }
            } else if (direction == -1 && grabberRotator.getCurrentPosition() <= upCheckPoint) {
                grabberState = 2;
            }
        }

        if (grabberState == 2) {
            int armPos = grabberRotator.getCurrentPosition();
            int pidTarget = holdValue;
            double pid = controller.calculate(armPos, pidTarget);
            double ff = Math.cos(Math.toRadians(pidTarget / ticks_in_degrees)) * f;
            double power = pid + ff;
            grabberRotator.setPower(power);
        }

        if (autoContractState == 1) {
            if (Math.abs(grabberRotator.getCurrentPosition() - 80) <= 10) {
                stateTimer = new ElapsedTime();
                autoContractState += 1;
            }
        }
        else if (autoContractState == 2) {
            if (stateTimer.seconds() > 1) {
                openClaw();
                stateTimer = new ElapsedTime();
                autoContractState += 1;
            }
        } else if (autoContractState == 3) {
            if (stateTimer.seconds() > 1) {
                moveDown(580);
                stateTimer = null;
                autoContractState += 1;
            }
        } else if (Math.abs(grabberRotator.getCurrentPosition() - 580) <= 10) {
            autoContractState = 0;
        }
    }
}
