//----------------------------------------------------------------------------------------------------------------------------------------------
//Linear lift going up
//----------------------------------------------------------------------------------------------------------------------------------------------

if (gamepad2.right_trigger && CatcherLiftMotor.getCurrentPosition() <= 1490) {  //put the maximum value for the lift from telemetry here
    CatcherLiftMotor.setPower(0.6);
        //liftRunning = true;
    telemetry.addData("WARNING WARNING",CatcherLiftMotor.getCurrentPosition() );
    telemetry.update();
}

else if (gamepad2.right_trigger && CatcherLiftMotor.getCurrentPosition() >= 1490) {
    CatcherLiftMotor.setPower(0);
        //initialGrab = false; //add this part to the left bumper
    telemetry.addData("WARNING WARNING", "You've reached Max!!!!" );
    telemetry.update();
        //goingDown = true;
}
 
//----------------------------------------------------------------------------------------------------------------------------------------------
// Linear lift to go down
//----------------------------------------------------------------------------------------------------------------------------------------------

    if (gamepad2.left_trigger && CatcherLiftMotor.getCurrentPosition() > 0) { 
    grabberArm.setPosition(0);
    CatcherLiftMotor.setPower(-0.6);
        //liftRunning = true;
    telemetry.addData("WARNING WARNONG",CatcherLiftMotor.getCurrentPosition() );
    telemetry.update();
}
else if (gamepad2.left_trigger && CatcherLiftMotor.getCurrentPosition() < 0 && goingDown == true) {
    CatcherLiftMotor.setPower(0);
        // initialGrab = false;
    telemetry.addData("WARNING WARNONG", "You've reached Max!!!!");
    telemetry.update();
  }