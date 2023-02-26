package ftcAdventure;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.Range;  

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.lang.Thread;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;

@TeleOp
public class DriveTrainAuto extends LinearOpMode {
    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backRight;
    private DcMotorEx backLeft;

    // Target in encoder ticks for each motor
    double target = 1000;
    
    // Pid controllers set with arbitrary values, use at your own risk
    PIDController frontLeftController = new PIDController(0.5,0.1,0.2);
    PIDController frontRightController = new PIDController(0.5,0.1,0.2);
    PIDController backRightController = new PIDController(0.5,0.1,0.2);
    PIDController backLeftController = new PIDController(0.5,0.1,0.2);
    
    @Override
    public void runOpMode() {
        // Drivetrain Motors
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

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    
        
        // Wait for player to press start
        waitForStart();
        
        while (opModeIsActive()) {
            // ----------------------------------
            // DRIVETRAIN
            // ----------------------------------
            // Actuate the motors
            frontRight.setPower(frontRightController.output(target,frontRight.getCurrentPosition()));
            backRight.setPower(backRightController.output(target,backRight.getCurrentPosition()));
            backLeft.setPower(backLeftController.output(target,backLeft.getCurrentPosition()));
            backRight.setPower(backRightController.output(target,backRight.getCurrentPosition()));

            telemetry.addData("current backleft", backLeft.getPower());
            telemetry.addData("current frontleft", frontLeft.getPower());
            telemetry.addData("current backright", backRight.getPower());
            telemetry.addData("current frontright", frontRight.getPower());
            // telemetry.update();
        }
    }
}