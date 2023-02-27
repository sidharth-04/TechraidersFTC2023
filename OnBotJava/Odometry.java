class Odometry {
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private double seperation;
    private double seperationLength;
    private double ticksPerMeter;
    private double gearReduction;
    private double wheelRadius;

    private double x;
    private double y;
    private double theta;

    public Odometry(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
    }

    public void initialize(double seperation, double seperationLength, double ticksPerRev, double gearReduction, double wheelRadius) {
        this.seperation = seperation;
        this.seperationLength = seperationLength;
        this.ticksPerRev = ticksPerRev;
        this.gearReduction = gearReduction;
        this.wheelRadius = wheelRadius;
    }

    private double encoderTicksToInches(double ticks) {
        return this.wheelRadius * 2 * Math.PI * this.gearReduction * ticks / this.ticksPerRev;
    }

    public double getNewPosition(double time) {
        this.time = time;
        double deltaTime = this.time - this.lastTime;

        double frontLeftTravel = this.encoderTicksToInches(this.frontLeft.getCurrentPosition() - oldPosition);
        double frontRightTravel = this.encoderTicksToInches(this.frontRight.getCurrentPosition());
        double rearLeftTravel = this.encoderTicksToInches(this.backLeft.getCurrentPosition());
        double rearRightTravel = this.encoderTicksToInches(this.backRight.getCurrentPosition());

        double deltaXTravel = (frontLeftTravel + frontRightTravel + rearLeftTravel + rearRightTravel) / 4.0;
        double deltaYTravel = (-frontLeftTravel + frontRightTravel + rearLeftTravel - rearRightTravel) / 4.0;
        double deltaTheta = (-frontLeftTravel + frontRightTravel - rearLeftTravel + rearRightTravel) / (2 * (this.wheelSeparation + this.wheelSeparationLength));

        this.x += deltaXTravel*Math.cos(this.theta) - deltaYTravel*Math.sin(this.theta);
        this.y += deltaYTravel*Math.cos(this.theta) + deltaXTravel*Math.sin(this.theta);
        this.theta = (this.theta + deltaTheta) % (2*pi);

        this.lastTime = time;

        return 
    }
}