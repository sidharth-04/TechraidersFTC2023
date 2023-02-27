package ftcAdventure;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;

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
public class DriveTrainCode extends LinearOpMode {
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backRight;
    private DcMotor backLeft;
    private DcMotor grabberRotator;
    private DcMotor grabberLift;
    private DcMotor catcherLift;
    
    private CRServo flipConeServo;
    private CRServo rotateLiftServo;
    
    public BNO055IMU imu;

    Orientation angles;
    Acceleration gravity;
    
    @Override
    public void runOpMode() {
        // Drivetrain Variables
        double driveTurn;
        double driveVertical;
        double driveHorizontal;
        
        // Drivercentric Variables
        double gamepadXCoordinate;
        double gamepadYCoordinate;
        double gamepadHypot;
        double gamepadDegree;
        double robotDegree;
        double movementDegree;
        double gamepadXControl;
        double gamepadYControl;
        
        // Teleop Variables
        double rotationServoPower = 0;
        
        // Drivetrain Motors
        frontLeft = hardwareMap.dcMotor.get("B0");
        frontRight = hardwareMap.dcMotor.get("A0");
        backRight = hardwareMap.dcMotor.get("A1");
        backLeft = hardwareMap.dcMotor.get("B1");
        
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        
        // HD Hex Motors
        grabberRotator = hardwareMap.get(DcMotor.class, "grabberRotatorMotor");
        // grabberRotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        grabberLift = hardwareMap.get(DcMotor.class, "grabberLiftMotor");
        // grabberLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        catcherLift = hardwareMap.get(DcMotor.class, "catcherLiftMotor");
        // catcherLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        // Servo Motors
        flipConeServo = hardwareMap.get(CRServo.class, "flipConeServo");
        rotateLiftServo = hardwareMap.get(CRServo.class, "rotateLiftServo");
        
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
        
        // grabberRotator.setTargetPosition(300);
        // grabberRotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // grabberLift.setTargetPosition(300);
        // grabberLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // catcherLift.setTargetPosition(300);
        // catcherLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // grabberRotator.setVelocity(200);
        // grabberLift.setVelocity(200);
        // catcherLift.setVelocity(200);

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        
        while (opModeIsActive()) {
            // ----------------------------------
            // DRIVETRAIN
            // ----------------------------------
            driveTurn = -gamepad1.right_stick_x;
            
            // Get the coordinates of the joystick in a x-y axis
            gamepadXCoordinate = gamepad1.left_stick_x; 
            gamepadYCoordinate = -gamepad1.left_stick_y; 
            // Get the value of the hypotenuse of the x and y coordinates in the code
            gamepadHypot = Range.clip(Math.hypot(gamepadXCoordinate, gamepadYCoordinate), 0, 1);
            // Get the degree of the gamepad based of the hypotenuse
            gamepadDegree = (Math.atan2(gamepadYCoordinate, gamepadXCoordinate)/Math.PI)*180;
           
            // Angle from the imu (internal angle of the robot)
            robotDegree = getAngle();
            telemetry.addData("angle", robotDegree );
            telemetry.update();
            
            // Final degree the robot will move in
            movementDegree = gamepadDegree - robotDegree;
        
            // Gives the x length, where to move in the x direction
            gamepadXControl = Math.cos(Math.toRadians(movementDegree)) * gamepadHypot;
            gamepadYControl = Math.sin(Math.toRadians(movementDegree)) * gamepadHypot;
            
            // Turn the robot to the  right and to the left based on newly computed x and y controls
            frontRight.setPower(gamepadYControl * Math.abs(gamepadYControl) - gamepadXControl * Math.abs(gamepadXControl) + driveTurn);
            backRight.setPower(gamepadYControl * Math.abs(gamepadYControl) + gamepadXControl * Math.abs(gamepadXControl) + driveTurn);
            frontLeft.setPower(gamepadYControl * Math.abs(gamepadYControl) + gamepadXControl * Math.abs(gamepadXControl) - driveTurn);
            backLeft.setPower(gamepadYControl * Math.abs(gamepadYControl) - gamepadXControl * Math.abs(gamepadXControl) - driveTurn);
            telemetry.addData("current backleft", backLeft.getPower());
            telemetry.addData("current frontleft", frontLeft.getPower());
            telemetry.addData("current backright", backRight.getPower());
            telemetry.addData("current frontright", frontRight.getPower());
            // telemetry.update();
            
            // ----------------------------------
            // GRABBER
            // ----------------------------------
            // Code for rotating the grabber
            if(gamepad1.right_bumper) {
                grabberRotator.setPower(-0.4);
            }
            else if (gamepad1.left_bumper) {
                grabberRotator.setPower(0.4);
            } else {
                grabberRotator.setPower(0.0);
            }
            // telemetry.addData("encoder pos", grabberRotator.getCurrentPosition());
            // telemetry.update();
            
            // Code for controlling the grabber lift
            if (gamepad1.right_trigger > 0){
                grabberLift.setPower(0.8);
            } else if(gamepad1.left_trigger > 0){
                grabberLift.setPower(1);
            } else {
                grabberLift.setPower(0);
            }
            
            // ----------------------------------
            // CATCHER
            // ----------------------------------
            // Code for rotating the catcher
            rotationServoPower = -gamepad2.right_stick_x / 2;
            rotateLiftServo.setPower(rotationServoPower);
            
            // Code for flipping the cone
            if (gamepad2.right_bumper){
                flipConeServo.setPower(0.6);
            } else if(gamepad2.left_bumper){
                flipConeServo.setPower(-0.6);
            } else {
                flipConeServo.setPower(0);
            }
            
            // Code for controlling the catcher lift
            if (gamepad2.right_trigger > 0){
                catcherLift.setPower(1);
            } else if(gamepad2.left_trigger > 0){
                catcherLift.setPower(1);
            } else {
                catcherLift.setPower(0);
            }
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
    
    public void sleep(int millis) {
        try {
            Thread.sleep(millis);
        } catch (Exception e) {}
    }
    
    public double getAngle() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }
}
    