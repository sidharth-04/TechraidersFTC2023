package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;

public class Grabber {
    // Actuators
    private DcMotorEx grabberRotator;
    private DcMotorEx grabberLift;
    private CRServo claw;

    private PIDController rotatorController;
    public static double rp = 0.0, ri = 0.0, rd = 0.0, rf = 0.0;
    public static int target = 0;

    // States and Finite State Machine (FSM)
    private enum FSM {
        FREE_CONTROL,
        FLIP_CONE,
        RESETTING
    }
    private FSM FSMState;

    // Endpoints (EP) for grabberRotator
    private double rotatorEndpoints[] = new double[6];
    private double rotEPT = rotatorEndpoints[0];
    private double rotEPB = rotatorEndpoints[5];

    // Endpoints (EP) for grabberLift
    private double liftEPMin;
    private double liftEPMax;

    private final double ticks_in_degrees = 700 / 180.0;

    public Grabber(HardwareMap hardwareMap, String opmode) {
        this.grabberRotator = hardwareMap.get(DcMotorEx.class, "grabberRotatorMotor");
        this.grabberLift = hardwareMap.get(DcMotorEx.class, "grabberLiftMotor");
        this.claw = hardwareMap.get(CRServo.class, "clawServo");

        this.setState(FSM.FREE_CONTROL);
    }

    private void setState(FSM state) {
        this.FSMState = state;
    }

    public String getState() {
        switch (this.FSMState) {
            case RESETTING:
                return "resetting";
            case FREE_CONTROL:
                return "free_contro";
            case FLIP_CONE:
                return "flip_cone";
        }
        return "not found";
    }

    // Signals that can be sent to the system
    public void flipConeSignal() {
        this.setState(FSM.FLIP_CONE);
    }

//    public void initiateGodModeSignal() {
//        this.setState(FSM.GOD_MODE);
//    }

    public void switchToFreeSignal() {
        this.setState(FSM.FREE_CONTROL);
    }

    public void resetGrabberSignal() {
        this.setState((FSM.RESETTING));
    }

    // Functions
    public void activateClaw(int pos) {
        return;
    }

    public void rotateGrabberToEP(int ep) {
        return;
    }

    public void slideLift(double power) {
        return;
    }

    public void setLiftToEP(int ep) {
        return;
    }

    private void flipCone() {

    }
    private void reset() {

    }

    public void update() {
        switch (this.FSMState) {
            case FREE_CONTROL:
                return;
            case RESETTING:
                reset();
            case FLIP_CONE:
                flipCone();
        }
    }
}
