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
public class DriveTrainSimplified extends LinearOpMode {
    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backRight;
    private DcMotorEx backLeft;
    public BNO055IMU imu;

    Orientation angles;
    Acceleration gravity;
    
    @Override
    public void runOpMode() {
        // Drivercentric Variables
        double gamepadX;
        double gamepadY;
        double gamepadT;
        double robotAngle;
        double xRotated;
        double yRotated;
        
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

        // frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        // Drivercentric Initialization
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        
        composeTelemetryMadarchod();
        
        // Wait for player to press start
        waitForStart();

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        
        while (opModeIsActive()) {
            // ----------------------------------
            // DRIVETRAIN
            // ----------------------------------
            // Get the coordinates of the joystick in a x-y axis
            gamepadX = gamepad1.left_stick_x;
            gamepadY = -gamepad1.left_stick_y;
            gamepadT = (Math.atan2(gamepadYCoordinate, gamepadXCoordinate)/Math.PI)*180;
           
            // Angle from the imu (internal angle of the robot)
            robotAngle = getAngle();
        
            // Calculate the rotated x and y
            double x_rotated = x * Math.cos(robotAngle) - y * Math.sin(robotAngle);
            double y_rotated = x * Math.sin(robotAngle) + y * Math.cos(robotAngle);
            
            // Turn the robot to the  right and to the left based on newly computed x and y controls
            frontRight.setPower(x_rotated - y_rotated - gamepadT);
            backRight.setPower(x_rotated + y_rotated - gamepadT);
            frontLeft.setPower(x_rotated + y_rotated + gamepadT);
            backLeft.setPower(x_rotated - y_rotated + gamepadT);
            telemetry.addData("current backleft", backLeft.getPower());
            telemetry.addData("current frontleft", frontLeft.getPower());
            telemetry.addData("current backright", backRight.getPower());
            telemetry.addData("current frontright", frontRight.getPower());
            // telemetry.update();
        }
    }
    
    // Utility Functions
    void composeTelemetryMadarchod() {
        telemetry.addAction(new Runnable() {
            @Override
            public void run() {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
            }
        });
    }

    public double getAngle() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }
}