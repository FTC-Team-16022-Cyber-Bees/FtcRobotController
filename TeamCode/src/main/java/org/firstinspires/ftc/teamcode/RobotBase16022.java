package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Disabled
public abstract class RobotBase16022 extends LinearOpMode {

    //Define all variables for hardware here
    public DcMotor Left_Front;
    public DcMotor Right_Front;
    public DcMotor Right_Back;
    public DcMotor Left_Back;

    //Define variables for servo starting positions, drive speeds, turn speeds,
    //distances in autonomous for encoders, etc.

    //Define motor counts for autonomous encoder based driving
    //NOTE:  Encoder counts are not yet updated to reflect Neverest 20 Motors or Gobilda
    //static final double     COUNTS_PER_MOTOR_REV    = 1120.0 ;    // eg: AndyMark NeverRest40 Motor Encoder
    static final double     COUNTS_PER_MOTOR_REV    = 537.6;    // For AndyMark Neverest20 Orbital
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // Geared up/down or 1:1
    static final double     WHEEL_DIAMETER_INCHES   = 3.937;    // Wheel circumference is 100mm for Gobilda
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION)/(WHEEL_DIAMETER_INCHES * 3.1415);

    HardwareMap hwMap;

    public ElapsedTime runtime = new ElapsedTime();

    protected void teleOpInitialize() {

        // define and initialize drive train motors
        Left_Front = hardwareMap.dcMotor.get("Left_Front");
        Right_Front = hardwareMap.dcMotor.get("Right_Front");
        Right_Back = hardwareMap.dcMotor.get("Right_Back");
        Left_Back = hardwareMap.dcMotor.get("Left_Back");

        // set direction of motors considering bevel gears and side of drivetrain
        Left_Front.setDirection(DcMotor.Direction.REVERSE);
        Right_Front.setDirection(DcMotor.Direction.FORWARD);
        Left_Back.setDirection(DcMotor.Direction.REVERSE);
        Right_Back.setDirection(DcMotor.Direction.FORWARD);

        //For TeleOp, we are not likely to need encoders on the drivetrain motors,
        //so we reset them post Autonomous
        Left_Front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Right_Front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Left_Back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Right_Back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //We follow up the stop and reset with turning off encoders (for TeleOp)
        Left_Front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Right_Front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Left_Back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Right_Back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Left_Front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Right_Front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Left_Back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Right_Back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Left_Front.setPower(0);
        Right_Front.setPower(0);
        Left_Back.setPower(0);
        Right_Back.setPower(0);

        //If we add other hardware (servo's, motors, sensors) that gets used in TeleOp
        //we would define them here including directions for motors, setting motors to
        //use encoders or run to position, etc

    }

    public void encoderDrive(double speed, double leftInches1, double leftInches2, double rightInches1,double rightInches2, double timeoutS) {
        int new_tLeftTarget;
        int new_tRightTarget;
        int new_bLeftTarget;
        int new_bRightTarget;

        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        // encoderDrive(drive_Speed, Front Left, Front Right, Rear Left, Rear Right, timeout)
        //encoderDrive(DRIVE_SPEED,  48,  48, 48, 48, 1.0);  // Drive Straight
        //encoderDrive(DRIVE_SPEED, -24, -24, -24, -24, 1.0);  // drive reverse
        //encoderDrive(DRIVE_SPEED, 12, -12, 12, -12, 1.0); // Turn Right
        //encoderDrive(DRIVE_SPEED, -12, 12, -12, 12, 1.0); //Turn Left
        //encoderDrive(TURN_SPEED,   12, 12, -12, -12, 1.0);  // DO NOT USE
        //encoderDrive(DRIVE_SPEED, 12, 12, -12, -12, 1.0);//Strafe Left
        //encoderDrive(DRIVE_SPEED, -2, 2, -2, 2, 1.0);//Go Forward
        // Ensure that the opmode is still active

        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            new_tLeftTarget = Left_Front.getCurrentPosition() + (int) (leftInches1 * COUNTS_PER_INCH);
            new_tRightTarget = Right_Front.getCurrentPosition() + (int) (rightInches1 * COUNTS_PER_INCH);
            new_bLeftTarget = Left_Back.getCurrentPosition() + (int) (leftInches2 * COUNTS_PER_INCH);
            new_bRightTarget = Right_Back.getCurrentPosition() + (int) (rightInches2 * COUNTS_PER_INCH);
            Left_Front.setTargetPosition(new_tLeftTarget);
            Right_Front.setTargetPosition(new_tRightTarget);
            Left_Back.setTargetPosition(new_bLeftTarget);
            Right_Back.setTargetPosition(new_bRightTarget);

            // Turn On RUN_TO_POSITION
            Left_Front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Right_Front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Left_Back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Right_Back.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            Left_Front.setPower(speed);
            Right_Front.setPower(speed);
            Left_Back.setPower(speed);
            Right_Back.setPower(speed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (Left_Front.isBusy() && Right_Front.isBusy() && Left_Back.isBusy() && Right_Back.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d :%7d :%7d", new_tLeftTarget, new_tRightTarget, new_bLeftTarget, new_bRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d :%7d :%7d",
                        Left_Front.getCurrentPosition(),
                        Right_Front.getCurrentPosition(),
                        Left_Back.getCurrentPosition(),
                        Right_Back.getCurrentPosition());
                telemetry.update();

            }

            // Stop all motion;
            Left_Front.setPower(0);
            Right_Front.setPower(0);
            Left_Back.setPower(0);
            Right_Back.setPower(0);

            // Turn off RUN_TO_POSITION
            Left_Front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Right_Front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Left_Back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Right_Back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    public void strafeLeft(double speed, double distance){
            encoderDrive(speed, -distance, distance, -distance, distance, 3.0);
    }
    public void strafeRight(double speed, double distance){
            encoderDrive(speed, distance, -distance, distance, -distance, 3.0);
    }
    public void driveForward(double speed, double distance){
            encoderDrive(speed, distance, distance, distance, distance, 3.0);
    }
    public void driveReverse(double speed, double distance){
            encoderDrive(speed, -distance, -distance, -distance, -distance, 4.0);
    }
    public void turnLeft(double speed, double distance){
            encoderDrive(speed, -distance, -distance, distance, distance, 3.0); //45 is 90 degrees
    }
    public void turnRight(double speed, double distance){
            encoderDrive(speed, distance, distance, -distance, -distance, 3.0);
    }

    public void drive( double lf, double rf, double lr, double rr1){

        Left_Front.setPower(lf);
        Right_Front.setPower(rf);
        Left_Back.setPower(lr);
        Right_Back.setPower(rr1);

    }

    public double scaleInput(double dVal)  {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                    0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);
        if (index < 0) {
            index = -index;
        } else if (index > 16) {
            index = 16;
        }

        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        return dScale;
    }

    //Sample of code that 'nudges' robot forward.  Can be changed to nudge in any direction

    public void triggerForward(){
        double frontRight = Range.clip(gamepad1.right_trigger,-1,1);
        double rearRight = Range.clip(gamepad1.right_trigger,-1,1);
        double frontLeft = Range.clip(gamepad1.right_trigger,-1,1);
        double rearLeft = Range.clip(gamepad1.right_trigger,-1,1);

        frontRight = (float) scaleInput(frontRight);
        rearRight = (float) scaleInput(rearRight);
        frontLeft = (float) scaleInput(frontLeft);
        rearLeft = (float) scaleInput(rearLeft);

        Left_Front.setPower(frontRight);
        Left_Back.setPower(rearLeft);
        Right_Back.setPower(rearRight);
        Left_Front.setPower(frontLeft);
    }
}
