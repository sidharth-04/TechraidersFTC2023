package ftcAdventure;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
    
    public BNO055IMU imu;

    Orientation angles;
    Acceleration gravity;
    
    @Override
    public void runOpMode() {
      
      
      double driveTurn;
      double driveVertical;
      double driveHorizontal;
        
//linked to the double value produced by the movement of the robot
        double gamepadXCoordinate;
        double gamepadYCoordinate;

        double gamepadHypot;
        double gamepadDegree;
        double robotDegree;
        double movementDegree;
        double gamepadXControl;
        double gamepadYControl;
      
      
        frontLeft = hardwareMap.dcMotor.get("B0");
        frontRight = hardwareMap.dcMotor.get("A0");
        backRight = hardwareMap.dcMotor.get("A1");
        backLeft = hardwareMap.dcMotor.get("B1");
        
        
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        
        //DRIVER CENTRIC CODE
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
        waitForStart();

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        
        // TELEOP VARIABLES 
        boolean goingDown = false; 
        boolean initialGrab = true; 
        boolean loadingSequence =  true;
        
        
        // END 
         while (opModeIsActive()) {
          driveTurn = -gamepad1.right_stick_x;
         // acc to the paper sirf yeh controls chahiye
         //according to the paper, only these controls were needed

        //gets the coordinates of the joystick in a x-y axis
            gamepadXCoordinate = gamepad1.left_stick_x; 
            gamepadYCoordinate = -gamepad1.left_stick_y; 
            
        //gets the value of the hypotenuse of the x and y coordinates in the code
            gamepadHypot = Range.clip(Math.hypot(gamepadXCoordinate, gamepadYCoordinate), 0, 1);
        
        //gives the degree of the gamepad based of the hypotenuse
            gamepadDegree = (Math.atan2(gamepadYCoordinate, gamepadXCoordinate)/Math.PI)*180;
        //contains the angle from the imu (internal angle of the robot)
            robotDegree = getAngle();
        //updates the angle data
            telemetry.addData("angle", robotDegree );
            telemetry.update();
        //final degree the robot will have to move in
            movementDegree = gamepadDegree - robotDegree;
        //gives you the x length, where to move in the x direction
            gamepadXControl = Math.cos(Math.toRadians(movementDegree)) * gamepadHypot;
            gamepadYControl = Math.sin(Math.toRadians(movementDegree)) * gamepadHypot;
            
        //turns the robot to the  right and to the left using the rightstick in gamepad 1 
            frontRight.setPower(gamepadYControl * Math.abs(gamepadYControl) - gamepadXControl * Math.abs(gamepadXControl) + driveTurn);
            backRight.setPower(gamepadYControl * Math.abs(gamepadYControl) + gamepadXControl * Math.abs(gamepadXControl) + driveTurn);
            frontLeft.setPower(gamepadYControl * Math.abs(gamepadYControl) + gamepadXControl * Math.abs(gamepadXControl) - driveTurn);
            backLeft.setPower(gamepadYControl * Math.abs(gamepadYControl) - gamepadXControl * Math.abs(gamepadXControl) - driveTurn);
             telemetry.addData("current backleft", backLeft.getPower());
             telemetry.addData("current frontleft", frontLeft.getPower());
             telemetry.addData("current backright", backRight.getPower());
             telemetry.addData("current frontright", frontRight.getPower());
              
            //   telemetry.update();
            }
            
                
            
           
               // going up 
            // if (gamepad2.right_bumper && currentLinearLiftPosition <= maxUpLinearLift  &&  goingDown == false) { // linear lift adjusts to the correct max height
            //     linearLift.setPower(1);
            //     liftRunning = true;
            // }
            //went up, now going down
              // else {
              //   topPusher.setPosition(1);
              //   sleep(5000);
              //                     topPusher.setPosition(0);

              //   goingDown = true;
              //   goingDown = true;
              //   telemetry.addData("Status NNN", "Should go down");
                // telemetry.update();
            // }
            
            
            //   if (gamepad2.right_bumper && currentLinearLiftPosition >= maxDownLinearLift  &&  goingDown == true) { // linear lift adjusts to the correct max height
            //     linearLift.setPower(-1);
            //     liftRunning = true;
            // }
            
            //     else    {
            //     goingDown = false;
            //     telemetry.addData("Status NNN", "Should go UP");
                // telemetry.update();
            // }
            // else if (gamepad2.right_bumper && currentLinearLiftPosition >= -140 && goingUp == true && goingDown == true ) {
            //     linearLift.setPower(-1);
            //     liftRunning = true;
            //     goingDown = true;
            // }
            // else if (currentLinearLiftPosition < maxDownLinearLift) {
            //     goingDown = false;
            //     linearLift.setPower(0);}
            // else {
            //     linearLift.setPower(0);
            //     // telemetry.addData("Status", linearLift.getCurrentPosition());
            //     liftRunning = false;
            // }

                
    }
            
            // telemetry.addData("current Voltage", "");

            // telemetry.update();
            
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

    //allows us to quickly get our z angle
    public double getAngle() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }
    

}
    