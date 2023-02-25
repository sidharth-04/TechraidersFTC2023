/*
Copyright 2023 FIRST Tech Challenge Team 2108

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package ftcAdventure;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Remove a @Disabled the on the next line or two (if present) to add this opmode to the Driver Station OpMode list,
 * or add a @Disabled annotation to prevent this OpMode from being added to the Driver Station
 */
@TeleOp

public class TestServo extends LinearOpMode {
    private Blinker control_Hub;
    private DcMotor hdmotor0;
    private Gyroscope imu;
    private CRServo servo0;


    @Override
    public void runOpMode() {
        //declaration of hardware
        hdmotor0 = hardwareMap.get(DcMotor.class, "hdmotor0");
        servo0 = hardwareMap.get(CRServo.class, "servo0");
        
                double servoPower = 0; 

        //considering that we need only buttons (right and left trigger for the motor)
        //and the right analogue stick for the servo, controlling the lift at the back
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.update();
            
        //declaring the variable for servo power
        servoPower = this.gamepad1.right_stick_x;
        
        //declaring variable for servo
         servo0.setPower(servoPower);
        // servo0.setPower(0.5);
   
            if(gamepad1.left_trigger>0.0)
        {
            hdmotor0.setPower(0.4);
            
        }
        else if(gamepad1.right_trigger>0.0)
        {
            hdmotor0.setPower(-0.4);
        }
        else        {
            hdmotor0.setPower(0.0);
         }
         
        telemetry.addData("Claw Test", hdmotor0.getCurrentPosition());
       // telemetry.addData("Servo Test", servo0.getPosition());

        telemetry.update();
        
        }
    }
}
