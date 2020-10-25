package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
public abstract class Skystone_Auto_Core extends LinearOpMode {

    SkystoneRobotConfig robot = new SkystoneRobotConfig();

    ElapsedTime runtime = new ElapsedTime();

    // ********************** //
    //For this function, we pass the motors identified in the robot config, set them for encoder
    //usage, take the distance required, and then translate it to motor encoder driven power
    // ********************** //


    // Rewrote this function to handle forward and back only

    /*
    public void StraightPDrive (double distance) {

        // Note reverse distance is determined by giving a negative distance, not a speed
        // Power for RUN TO POSITION is always positive

        int new_distanceTarget;
        double startDirection = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
                AngleUnit.DEGREES).firstAngle;
        double minPower = 0.3;
        double maxPower = 0.7;


        // Trying PController set up around IMU
        PController pControllerDrive = new PController (0.02);
        pControllerDrive.setInputRange(20,40);
        pControllerDrive.setSetPoint(startDirection);
        pControllerDrive.setOutputRange(minPower, maxPower);

        robot.fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        new_distanceTarget = (int) (distance * robot.COUNT_PER_INCH);

        robot.fl.setTargetPosition(new_distanceTarget);
        robot.fr.setTargetPosition(new_distanceTarget);
        robot.bl.setTargetPosition(new_distanceTarget);
        robot.br.setTargetPosition(new_distanceTarget);

        // Turn on RUN_TO_POSITION

        robot.fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.br.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Changing motor settings for each of four motors to run using encoders
        //We would change this if we used odometry wheels

        while (opModeIsActive()) {

            // Reset the timeout time and start motion
            // Timeout is provided as a safety precaution in case robot hits obstacle and cannot
            // complete path; this would lead to motor burnout if the motors kept trying to turn
            double currentDirection = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
                    AngleUnit.DEGREES).firstAngle;
            telemetry.addData("Current Angle", currentDirection);

            if (currentDirection > startDirection) {
                // Turn Left
                robot.br.setPower(minPower+pControllerDrive.getComputedOutput(currentDirection));
                robot.fr.setPower(minPower+pControllerDrive.getComputedOutput(currentDirection));
                robot.bl.setPower(minPower);
                robot.fl.setPower(minPower);

            } else if (currentDirection < startDirection) {
                // Turn Right
                robot.bl.setPower(minPower+pControllerDrive.getComputedOutput(currentDirection));
                robot.fl.setPower(minPower+pControllerDrive.getComputedOutput(currentDirection));
                robot.br.setPower(minPower);
                robot.fr.setPower(minPower);
            } else {
                //On target - keep going straight
                robot.fl.setPower(minPower);
                robot.fr.setPower(minPower);
                robot.bl.setPower(minPower);
                robot.br.setPower(minPower);
            }

            while (this.opModeIsActive() && robot.fl.isBusy() && robot.fr.isBusy()
                    && robot.bl.isBusy() && robot.br.isBusy()) {
                //Do nothing.  We are waiting for the motors to report that they are done
                telemetry.addData("Target", "Running to %7d",
                        new_distanceTarget);
                telemetry.addData("Current", "Running at %7d :%7d :%7d :%7d",
                        robot.fl.getCurrentPosition(),
                        robot.fr.getCurrentPosition(),
                        robot.bl.getCurrentPosition(),
                        robot.br.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motors
            robot.fl.setPower(0);
            robot.fr.setPower(0);
            robot.bl.setPower(0);
            robot.br.setPower(0);

            // Turn off RUN TO POSITION
            robot.fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // Optional pause after each move
            // sleep (250);
        }
    }*/

    public void EncoderDrive (double speed,
                              double frontLeftDistance,
                              double frontRightDistance,
                              double backLeftDistance,
                              double backRightDistance) {

        // Note reverse distance is determined by giving a negative distance, not a speed
        // Power for RUN TO POSITION is always positive

        int new_flTarget, new_frTarget, new_blTarget, new_brTarget;

        robot.fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        new_flTarget = (int) (frontLeftDistance * robot.COUNT_PER_INCH);
        new_frTarget = (int) (frontRightDistance * robot.COUNT_PER_INCH);
        new_blTarget = (int) (backLeftDistance * robot.COUNT_PER_INCH);
        new_brTarget = (int) (backRightDistance * robot.COUNT_PER_INCH);

        robot.fl.setTargetPosition(new_flTarget);
        robot.fr.setTargetPosition(new_frTarget);
        robot.bl.setTargetPosition(new_blTarget);
        robot.br.setTargetPosition(new_brTarget);

        // Turn on RUN_TO_POSITION

        robot.fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.br.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Changing motor settings for each of four motors to run using encoders
        //We would change this if we used odometry wheels

        while (opModeIsActive()) {

            // Reset the timeout time and start motion
            // Timeout is provided as a safety precaution in case robot hits obstacle and cannot
            // complete path; this would lead to motor burnout if the motors kept trying to turn

            runtime.reset();
            robot.fl.setPower(speed);
            robot.fr.setPower(speed);
            robot.bl.setPower(speed);
            robot.br.setPower(speed);

            while (this.opModeIsActive() && robot.fl.isBusy() && robot.fr.isBusy()
                    && robot.bl.isBusy() && robot.br.isBusy()) {
                //Do nothing.  We are waiting for the motors to report that they are done
                telemetry.addData("Target", "Running to %7d :%7d :%7d :%7d",
                        new_flTarget, new_frTarget, new_blTarget, new_brTarget);
                telemetry.addData("Current", "Running at %7d :%7d :%7d :%7d",
                        robot.fl.getCurrentPosition(),
                        robot.fr.getCurrentPosition(),
                        robot.bl.getCurrentPosition(),
                        robot.br.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motors
            robot.fl.setPower(0);
            robot.fr.setPower(0);
            robot.bl.setPower(0);
            robot.br.setPower(0);

            // Turn off RUN TO POSITION
            robot.fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // Optional pause after each move
            // sleep (250);
        }
    }

    public void StrafeLeft (double speed, double distance) {
        EncoderDrive(speed, -distance, distance, distance, -distance);
    }

    public void StrafeRight (double speed, double distance) {
        EncoderDrive(speed, distance, -distance, -distance, distance);
    }

    public void DriveForward (double speed, double distance) {
        EncoderDrive(speed, distance, distance, distance, distance);
    }

    public void DriveBackward (double speed, double distance) {
        EncoderDrive(speed, -distance, -distance, -distance, -distance);
    }

    public void TurnLeft (double speed, double distance) {
        EncoderDrive(speed, -distance, distance, -distance, distance);
    }

    public void TurnRight (double speed, double distance) {
        EncoderDrive(speed, distance, -distance, distance, -distance);
    }


    /*public void initializeIMU() {

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        // Reduces time lag on gyro calls by eliminating calls to other sensors
        // parameters.mode = BNO055IMU.SensorMode.GYRONLY;
        robot.imu.initialize(parameters);

        if (!robot.imu.isGyroCalibrated()) {
            this.sleep(50);
        }
    }

     */

    /*
    public void turnRobotInDegrees(double angle, double power) {

        double startAngle = robot.imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        double robotTurnedAngle = 0;


        robot.fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        robot.fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        while (this.opModeIsActive() && Math.abs(robotTurnedAngle) < Math.abs(angle)) {
            robotTurnedAngle = robot.imu.getAngularOrientation(AxesReference.INTRINSIC,
                    AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - startAngle;

            telemetry.addData("Starting angle", startAngle);
            telemetry.addData("Robot Turned Angle", robotTurnedAngle);
            telemetry.addData("Passed Angle", angle);
            telemetry.update();
            sleep(100);

            if (robotTurnedAngle > 180) {
                robotTurnedAngle = robotTurnedAngle - 360;
            }
            if (robotTurnedAngle < -180) {
                robotTurnedAngle = robotTurnedAngle + 360;
            }

            if (angle < 0) {
                robot.fl.setPower(-power);
                robot.bl.setPower(-power);
                robot.fr.setPower(power);
                robot.br.setPower(power);
            } else {
                robot.fl.setPower(power);
                robot.bl.setPower(power);
                robot.fr.setPower(-power);
                robot.br.setPower(-power);
            }
        }
    }

     */

    /*public void turnDegreesPController(double angle, PController pController) {

        double accumulatedTurn = 0;
        double startAngle = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
                AngleUnit.DEGREES).firstAngle;
        double powerToMotors = 0;

        robot.fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Reset the set point based on the angle
        pController.setSetPoint(Math.abs(angle));

        //**Possibly redundant, but trying to see if it reduces excess turn
        robot.bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        do {
            accumulatedTurn = startAngle - robot.imu.getAngularOrientation(AxesReference.EXTRINSIC,
                        AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

            // accumulatedTurn variable keeps track of the total amount turned so far.
            if (accumulatedTurn < -180)
                accumulatedTurn += 360;
            else if (accumulatedTurn > 180)
                accumulatedTurn -= 360;

            powerToMotors = pController.minOutput + pController.getComputedOutput(accumulatedTurn);

            if (angle < 0) {
                // turn left - modify ordering if your motors behave differently
                robot.fl.setPower(-powerToMotors);
                robot.bl.setPower(-powerToMotors);
                robot.fr.setPower(powerToMotors);
                robot.br.setPower(powerToMotors);
            } else {
                // turn right - modify ordering if your motors behave differently
                robot.fl.setPower(powerToMotors);
                robot.bl.setPower(powerToMotors);
                robot.fr.setPower(-powerToMotors);
                robot.br.setPower(-powerToMotors);
            }
        } while(opModeIsActive() && !pController.hasPControllerReachedTarget());

        // set power to zero and apply brakes
        Robot_Stop();
        // sleep (500); // wait for half a second so the robot stops all motion

    }

     */

    // False tells the robot to latch the foundation; true says to unlatch

    public void LatchFoundation (boolean latchPos) {

        double backLeftLatchServoMax        = 0.9;
        double backLeftLatchServoMin        = 0.45;
        double backRightLatchServoMax       = 0.9;
        double backRightLatchServoMin       = 0.45;

        if (latchPos) {
            robot.backRightLatchServo.setPosition(backRightLatchServoMax);
            robot.backLeftLatchServo.setPosition(backLeftLatchServoMax);
        } else {
            robot.backRightLatchServo.setPosition((backRightLatchServoMin));
            robot.backLeftLatchServo.setPosition(backLeftLatchServoMin);
        }

    }

    // This function assigns power and movement type (e.g. strafe) to wheels
    // Movement must be stopped by another function
    public void Robot_Move ( int moveFlag, double speed) {
        if (moveFlag == 1) {
            // Strafe left
            robot.fl.setPower(-speed);
            robot.fr.setPower(speed);
            robot.bl.setPower(speed);
            robot.br.setPower(-speed);
        }
        if (moveFlag == 2) {
            // Strafe right
            robot.fl.setPower(speed);
            robot.fr.setPower(-speed);
            robot.bl.setPower(-speed);
            robot.br.setPower(speed);
        }
        if (moveFlag == 3) {
            // Forward
            robot.fl.setPower(speed);
            robot.fr.setPower(speed);
            robot.bl.setPower(speed);
            robot.br.setPower(speed);
        }
        if (moveFlag == 4 ) {
            // Backward
            robot.fl.setPower(speed);
            robot.fr.setPower(speed);
            robot.bl.setPower(speed);
            robot.br.setPower(speed);
        }
    }

    public void Robot_Stop () {

        robot.fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.fl.setPower(0);
        robot.fr.setPower(0);
        robot.bl.setPower(0);
        robot.br.setPower(0);
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

        double dScale;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        return dScale;
    }
}
