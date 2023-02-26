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
    // Drivetrain Motors
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
        double x;
        double y;
        double t;
        double robotAngle;

        // Target in encoder ticks for each motor
        double xTarget = 100;
        double yTarget = 100;
        double thetaTarget = Math.PI / 2;
        
        // Pid controllers set with arbitrary values, use at your own risk
        PIDController controllerX = new PIDController(0.5,0.1,0.2);
        PIDController controllerY = new PIDController(0.5,0.1,0.2);
        PIDController controllerTheta = new PIDController(0.5,0.1,0.2,true);

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
            // Angle from the imu (internal angle of the robot)
            robotAngle = getAngle();
            xRobotPosition = getRobotX();
            yRobotPosition = getRobotY();

            x = xControl.calculate(xTarget, xRobotPosition); 
            y = yControl.calculate(yTarget, yRobotPosition);
            t = thetaControl.calculate(thetaTarget, robotAngle);
        
            // Calculate the rotated x and y
            double rotX = x * Math.cos(-robotAngle) - y * Math.sin(-robotAngle);
            double rotY = x * Math.sin(-robotAngle) + y * Math.cos(-robotAngle);

            double frontLeftPower = (rotY + rotX + t);
            double backLeftPower = (rotY - rotX + t);
            double frontRightPower = (rotY - rotX - t);
            double backRightPower = (rotY + rotX - t);

            // Actuate the motors
            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);
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
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle * Math.PI / 180;
    }

    public double getRobotX() {
        return 0;
    }
    public double getRobotY() {
        return 0;
    }
}