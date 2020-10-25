package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Disabled
@TeleOp
public class UltimateGoal_TeleOp_V1 extends Skystone_Auto_Core {

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        double fwdBackPower, strafePower, turnPower, maxPower;

        robot.fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.leftLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Robot speed modifier - value that power is divided by on Mecanum drive
        double speedModifier = 1.5;

        // Servo position supporting variables

        double tamperServoMin = 0;   // Top of motion
        double tamperServoMax = .5;  // Bottom of motion
        double swingerServoMax = .97;  // Side facing block
        double swingerServoMin = .3;   // Inside to robot
        double gripperServoMax = .8;  // Top of motion
        double gripperServoMin = .2;  // Bottom of motion
        double tampGripCombo = 0; //Variable to track time between tamp and grip
        boolean tamperDown = false;
        boolean gripperClosed = false;
        double timeBetweenTamperAndGripper = 1.2;
        double bumpRightLastTime = 0;
        double bumpLeftLastTime = 0;
        boolean rightBumperPosFlag = false;
        boolean leftBumperPosFlag = false;
        double secondsBetweenLatchPress = 0.5;
        double backLeftLatchServoMax = 0.9;  // Bottom
        double backLeftLatchServoMin = 0.35;  // Top
        double backRightLatchServoMax = 0.9; // Bottom
        double backRightLatchServoMin = 0.35;  // Top
        boolean capstoneScoreFlag = false;
        double secondsBetweenCapstoneFlip = 1;
        double lastCapstoneFlip = 0;
        double capstoneServoMax = 1;
        double capstoneServoMin = 0.55;
        boolean tamperPosFlag = false;
        double tamperLastTime = 0;
        double secondsBetweenTamperSwing = 0.5;
        double secondsBetweenGripper = 0.5;
        boolean gripperPosFlag = true;
        double gripperLastTime = 0;
        double swingerLastTime = 0;
        boolean swingerPosFlag = false;
        double secondsBetweenSwinger = 0.5;
        boolean blockDetectorFlag = false;
        double backUpDistance = 15;  // centimeters from platform that 'x' reacts to
        double minSensorDistance = 3.5;   // centimeters from platform where auto-backup stops
        double robotAlignmentSpeed = -0.3; // speed at which robot backs up to align to foundation
        boolean leftFirst = false;
        boolean rightFirst = false;
        double nudgeDistance = 1;
        int distance = 0;
        double nudgeSpeed = 0.3;
        boolean bottomLimit = false;
        double newSwingPosition = -1100;
        boolean blockDraggerUp = false;

        // ************ //
        // Initialize and position all servo's in optimal position
        // Need to change for holding in intake arms for tamper servo
        // ************ //

        robot.gripperServo.setPosition(gripperServoMax);
        robot.tamperServo.setPosition(tamperServoMin);
        robot.swingerServo.setPosition(swingerServoMax);

        // Set dragger positions to upright to avoid interfering in teleOp
        robot.rightAutoDraggerServo.setPosition(.3);
        robot.leftAutoDraggerServo.setPosition(.05);

        robot.backLeftLatchServo.setPosition(backLeftLatchServoMin);
        robot.backRightLatchServo.setPosition(backRightLatchServoMin);

        //Do not use waitForStart() since we have Motorola E4 phones.
        //They have a known bug to time out when using Waitforstart()

        //waitForStart();
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("status", "waiting for start command...");
            telemetry.update();
        }

        while (opModeIsActive()) {

            // Mecanum code:  uses left joystick for forward and backward motion (Y axis), right joystick
            // for turning, and left joystick (X axis) for strafing

            fwdBackPower = Range.clip(-gamepad1.left_stick_y, -1, 1);
            strafePower = Range.clip(gamepad1.left_stick_x, -1, 1);
            turnPower = Range.clip(gamepad1.right_stick_x, -1, 1);

            fwdBackPower = ((float) scaleInput(fwdBackPower)) / speedModifier;
            strafePower = ((float) scaleInput(strafePower)) / speedModifier;
            turnPower = ((float) scaleInput(turnPower)) / speedModifier;

            double frontLeftVal = fwdBackPower + turnPower + strafePower;
            double frontRightVal = fwdBackPower - turnPower - strafePower;
            double backLeftVal = fwdBackPower + turnPower - strafePower;
            double backRightVal = fwdBackPower - turnPower + strafePower;

            maxPower = Math.abs(frontLeftVal);
            if (Math.abs(frontRightVal) > maxPower) {
                maxPower = Math.abs(frontRightVal);
            }
            if (Math.abs(backLeftVal) > maxPower) {
                maxPower = Math.abs(backLeftVal);
            }
            if (Math.abs(backRightVal) > maxPower) {
                maxPower = Math.abs(backRightVal);
            }

            if (maxPower > 1) {
                frontLeftVal = frontLeftVal / maxPower;
                frontRightVal = frontRightVal / maxPower;
                backLeftVal = backLeftVal / maxPower;
                backRightVal = backRightVal / maxPower;
            }

            robot.fl.setPower(frontLeftVal);
            robot.fr.setPower(frontRightVal);
            robot.bl.setPower(backLeftVal);
            robot.br.setPower(backRightVal);

            // ********************************* //
            // Intake Motor Code - controls intakes
            // Assign intake motors to left and right trigger, where right trigger is forward and
            // left trigger is back; adds protection in case both triggers are pressed at same time
            // ********************************* //

            if ((gamepad1.right_trigger > 0) && (gamepad1.left_trigger == 0)) {
                robot.rightIntakeMotor.setPower(-1);
                robot.leftIntakeMotor.setPower(-1);
            } else if ((gamepad1.left_trigger > 0) && (gamepad1.right_trigger == 0)) {
                robot.rightIntakeMotor.setPower(1);
                robot.leftIntakeMotor.setPower(1);
            } else if ((gamepad1.right_trigger > 0) && (gamepad1.left_trigger > 0)) {
                robot.rightIntakeMotor.setPower(-1);
                robot.leftIntakeMotor.setPower(-1);
            } else {
                robot.rightIntakeMotor.setPower(0);
                robot.leftIntakeMotor.setPower(0);
            }

            // ****************************** //
            // Servo Controls - Latches on Driver Gamepad
            // Using bumper press on driver gamepad to toggle latches that drag foundation
            // First press sets at max position.  This starts 1/2 second wait time until another press
            // will work.  It also triggers a flag that tells the next press to take servo to other
            // position.
            // ****************************** //

            if (gamepad1.right_bumper) {
                if ((getRuntime() - bumpRightLastTime) > secondsBetweenLatchPress) {
                    bumpRightLastTime = getRuntime();
                    if (!rightBumperPosFlag) {
                        robot.backRightLatchServo.setPosition(backRightLatchServoMax);
                        robot.backLeftLatchServo.setPosition(backLeftLatchServoMax);
                        rightBumperPosFlag = true;
                    } else {
                        robot.backRightLatchServo.setPosition(backRightLatchServoMin);
                        robot.backLeftLatchServo.setPosition(backLeftLatchServoMin);
                        rightBumperPosFlag = false;
                    }
                }
            }
            /*
            if (gamepad1.left_bumper) {
                if ((getRuntime() - bumpLeftLastTime) > secondsBetweenLatchPress) {
                    bumpLeftLastTime = getRuntime();
                    if (!leftBumperPosFlag) {
                        robot.backLeftLatchServo.setPosition(backLeftLatchServoMax);
                        leftBumperPosFlag = true;
                    } else {
                        robot.backLeftLatchServo.setPosition(backLeftLatchServoMin);
                        leftBumperPosFlag = false;
                    }
                }
            }

             */

            // Code for testing block dragger positioning

            /*
            if ((gamepad1.a) && (blockDraggerUp)) {
                robot.leftAutoDraggerServo.setPosition(.75);  // Bottom of range and grabbing block
                robot.rightAutoDraggerServo.setPosition(.7);
                blockDraggerUp = false;
            } else if ((gamepad1.a) && (!blockDraggerUp)) {
                robot.leftAutoDraggerServo.setPosition(.35);
                robot.rightAutoDraggerServo.setPosition(.3);
                blockDraggerUp = true;
            }*/


            robot.tapeMeasure.setPower((gamepad2.left_stick_y)*0.85);

            // ******************************** //
            // Tamper and Gripper Servo Controls - Gamepad 2
            // The following code blocks tie the tamper and gripper to a single press of the
            // left and right gripper respectively.  Each press has a small wait period, after
            // which it allows the same press to reverse the current position
            // They both use a time check (getRunTime()) versus the last recorded time check
            // and compare that to a variable that is desired time lag between presses
            // ******************************** //

            if (gamepad2.left_bumper) {
                if ((getRuntime() - tamperLastTime) > secondsBetweenTamperSwing) {
                    tamperLastTime = getRuntime();
                    if (!tamperPosFlag) {
                        robot.tamperServo.setPosition(tamperServoMax);
                        tamperPosFlag = true;
                    } else {
                        robot.tamperServo.setPosition(tamperServoMin);
                        tamperPosFlag = false;
                    }
                }
            }

            if (gamepad2.right_bumper) {
                if ((getRuntime() - gripperLastTime) > secondsBetweenGripper) {
                    gripperLastTime = getRuntime();
                    if (!gripperPosFlag) {
                        robot.gripperServo.setPosition(gripperServoMax);  // Top of Motion
                        gripperPosFlag = true;
                    } else {
                        robot.gripperServo.setPosition(gripperServoMin);  // Bottom of Motion
                        gripperPosFlag = false;
                    }
                }
            }

            // ****************************** //
            // Automated color block tamping and gripping:
            // Attempting code to color sense that block is in basket, tamp, and then grip the block
            // Code must allow time for tamp to happen and then grip without using sleep
            // Tamp should only happen once per block, and immediately untamp
            // ****************************** //
/*
            if ((robot.blockDetector.green() > 10000) && (robot.blockDetector.alpha() > 21000)) {
                blockDetectorFlag = true;
            } else {
                blockDetectorFlag = false;
                tamperDown = false;
                gripperClosed = false;
            }

            if (blockDetectorFlag) {
                telemetry.addData("Tamper Servo Position", robot.tamperServo.getPosition());
                if ((tamperServoMax > robot.tamperServo.getPosition()) && (!tamperDown)) {
                    robot.tamperServo.setPosition(tamperServoMax);
                    tamperDown = true;
                    sleep(100);
                }
                if ((gripperServoMin < robot.gripperServo.getPosition()) && (!gripperClosed)) {
                    robot.gripperServo.setPosition(gripperServoMin);
                    sleep (100);
                    gripperClosed = true;
                }
            }
*/
            // ***************** //
            // This code toggles the swinger servo position from over the robot to scoring position
            // ***************** //

            // Set the swinger position minimum height to clear the lift
            // Base level starts at 1100 and adds encoder values every time lift gets to bottom limit sensor


            if (!robot.magneticBottomLimit.getState() && (bottomLimit)) {
                newSwingPosition = robot.rightLiftMotor.getCurrentPosition() + newSwingPosition;
                bottomLimit = false;  // Ensures it only does this once
            } else if (robot.magneticBottomLimit.getState()){
                bottomLimit = true;
            }


            if (gamepad2.x) {
                if ((getRuntime() - swingerLastTime) > secondsBetweenSwinger) {
                    swingerLastTime = getRuntime();

                    // don't let the arm swing around unless the position is greater than
                    // the last recorded minimum height to clear the lift

                    if (robot.rightLiftMotor.getCurrentPosition() < newSwingPosition) {
                        if (!swingerPosFlag) {
                            robot.swingerServo.setPosition(swingerServoMax);
                            swingerPosFlag = true;
                        } else if (swingerPosFlag) {
                            robot.swingerServo.setPosition(swingerServoMin);
                            swingerPosFlag = false;
                        }
                    }

                }
            }

            ///////////////////////////
            // Adding a set of code to move the lift arm to set positions tied to the number of
            // blocks currently stacked.  The minimum will be enough to clear the robot, swing, and
            // lower.  Still may release manually.
            // Logic:
            // a) Gunner presses a button for stacking first block
            // b) Lift raises to present position designed to clear robot (while motor is busy)
            // c) Swing arm swings to scoring position
            // d) Lower arm
            // e) Release grip
            // f) <Multiple button presses move the lift one block per press, if pressed in rapid>
            // g) Pressing y puts the lift back into 'stowed' position
            ///////////////////////////

            /*
            if (gamepad2.x) {
                robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.liftMotor.setTargetPosition(1500);
                robot.liftMotor.setPower(0.4); // Needs to be verified
                while (robot.liftMotor.isBusy()){

                }
                robot.swingerServo.setPosition(swingerServoMin);
                robot.liftMotor.setTargetPosition(0);
                robot.liftMotor.setPower(0.3);
                while (robot.liftMotor.isBusy()){
                    // Intentionally blank
                }
                robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            if (gamepad2.y) {
                robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.liftMotor.setTargetPosition(1000);
                while (robot.liftMotor.isBusy()){

                }
                robot.swingerServo.setPosition(swingerServoMin);
                robot.liftMotor.setTargetPosition(0);
                while (robot.liftMotor.isBusy()){
                    // Intentionally blank
                }
                robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

             */


            // Lift motor checks for limit sensors at top and bottom of lift.  Stops motion if it
            // "sees" sensors

            // Changed the lower speed to be less than the up speed to compensate marginally for gravity
            // The y stick value is divided by 2 on the way down but only 1.5 on the way up.
            // The -0.01 lift power is designed to hold the lift in place.

            if ((gamepad2.right_stick_y < 0) && (robot.magneticTopLimit.getState())) {
                robot.rightLiftMotor.setPower(gamepad2.right_stick_y / 1.5);
                robot.leftLiftMotor.setPower(gamepad2.right_stick_y / 1.5);
            } else {
                if ((gamepad2.right_stick_y > 0) && (robot.magneticBottomLimit.getState())) {
                    robot.rightLiftMotor.setPower(gamepad2.right_stick_y / 2);
                    robot.leftLiftMotor.setPower(gamepad2.right_stick_y / 2);
                } else if ((gamepad2.right_stick_y == 0) && (!robot.magneticBottomLimit.getState())) {
                    robot.rightLiftMotor.setPower(0);
                    robot.leftLiftMotor.setPower(0);
                } else {
                    // Need a little power to keep lift from slipping - BRAKE is not enough on its own
                    robot.rightLiftMotor.setPower(-0.002);
                    robot.leftLiftMotor.setPower(-0.002);
                }
            }

            // This code is the capstone flipper, which works by pushing the dpad up or down.
            // If the flipper has already flipped, the flag is set and it unflips based on timing
            if ((gamepad2.dpad_up) && (!gripperPosFlag)) {
                robot.capstoneScoreServo.setPosition(capstoneServoMax);
            }

            if (gamepad2.dpad_down) {
                robot.capstoneScoreServo.setPosition(capstoneServoMin);
            }
            /*
            if (gamepad2.b) {
                robot.leftAutoDraggerServo.setPosition(0.42);  // Bottom
            }

            if (gamepad2.y) {
                robot.leftAutoDraggerServo.setPosition(.05);  // Top
            }

             */
            //TELEMETRY TESTING AREA

            //Testing readouts of sensors to verify
            // Testing ultrasonic sensors

            telemetry.addData("Left Ultrasonic", robot.leftUltrasonic.rawUltrasonic());
            telemetry.addData("Left Distance", robot.leftUltrasonic.rawOptical());
            telemetry.addData("Left Inches", robot.leftUltrasonic.getDistance(DistanceUnit.INCH));
            telemetry.addData("Right Ultrasonic", robot.rightUltrasonic.rawUltrasonic());
            telemetry.addData("Right Distance", robot.rightUltrasonic.rawOptical());
            telemetry.addData("Right Inches", robot.rightUltrasonic.getDistance(DistanceUnit.INCH));
            telemetry.addData("Front Ultrasonic", robot.frontUltrasonic.rawUltrasonic());
            telemetry.addData("Front Distance", robot.frontUltrasonic.rawOptical());
            telemetry.addData("Front Inches", robot.frontUltrasonic.getDistance(DistanceUnit.INCH));



            //  Testing color sensors

            // telemetry.addData("Block Distance (cm)",  //Detects the Lexan
            //        String.format(Locale.US, "%.02f", robot.blockDetectorDistance.getDistance(DistanceUnit.CM)));
            // telemetry.addData("Line Color Sensor Blue", robot.lineDetector.blue());
            // telemetry.addData( "Line Color Sensor Red", robot.lineDetector.red());
            // telemetry.addData("Line Color Sensor Green", robot.lineDetector.green());
            // telemetry.addData("Block Color Sensor Blue", robot.blockDetector.blue());
            // telemetry.addData("Block Color Sensor Red", robot.blockDetector.red());
            // telemetry.addData("Block Color Sensor Green", robot.blockDetector.green());
            // telemetry.addData("Block Color Sensor Alpha", robot.blockDetector.alpha());

            telemetry.addData("Right Distance Sensor",
                    robot.rightRearDistanceSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("Blue Underneath sensor", robot.lineDetector.blue());
            telemetry.addData("Left Distance Sensor",
                    robot.leftRearDistanceSensor.getDistance(DistanceUnit.CM));
/*
            telemetry.addData ("Front Left Motor Encoder Count", robot.fl.getCurrentPosition());
            telemetry.addData ("Front Right Motor Encoder Count", robot.fr.getCurrentPosition());
            telemetry.addData ("Back Left Motor Encoder Count", robot.bl.getCurrentPosition());
            telemetry.addData ("Back Right Motor Encoder Count", robot.br.getCurrentPosition());
            telemetry.addData ("Front Left Motor Encoder Count", robot.fl.getPower());
            telemetry.addData ("Front Right Motor Encoder Count", robot.fr.getPower());
            telemetry.addData ("Back Left Motor Encoder Count", robot.bl.getPower());
            telemetry.addData ("Back Right Motor Encoder Count", robot.br.getPower());
            telemetry.addData("Tape Measure Power", -gamepad2.left_stick_y);

 */


            //telemetry.addData ("Right Lift Motor", robot.rightLiftMotor.getCurrentPosition());
            //telemetry.addData ("Left Lift Motor", robot.leftLiftMotor.getCurrentPosition());
            //telemetry.addData("Evolving Bottom Starting Position", newSwingPosition);

            // Testing Motor Values
            /*
            telemetry.addData("Front Right Value:", frontRightVal);
            telemetry.addData("Front Left Value:", frontLeftVal);
            telemetry.addData("Back Left Value:", backLeftVal);
            telemetry.addData("Back Right Value:", backRightVal);
            telemetry.update();
            */
            // Testing Limit Switches
/*
            if (robot.magneticTopLimit.getState()) {
                telemetry.addData("Magnetic Top Limit", "Does not see a Magnet");
            } else {
                telemetry.addData("Magnetic Top Limit", "Sees a Magnet");
            }

            if (robot.magneticBottomLimit.getState()) {
                telemetry.addData("Magnetic Bottom Limit", "Does not see a Magnet");
            } else {
                telemetry.addData("Magnetic Bottom Limit", "Sees a Magnet");
            }
*/
            telemetry.update();

            // Nudge Left
            if (gamepad1.b) {
                robot.fl.setPower(-nudgeSpeed);
                robot.fr.setPower(nudgeSpeed);
                robot.bl.setPower(nudgeSpeed);
                robot.br.setPower(-nudgeSpeed);
                sleep(100);
                Robot_Stop();
            }

                //Nudge Right
            if (gamepad1.x) {
                robot.fl.setPower(nudgeSpeed);
                robot.fr.setPower(-nudgeSpeed);
                robot.bl.setPower(-nudgeSpeed);
                robot.br.setPower(nudgeSpeed);
                sleep(100);
                Robot_Stop();
            }

            // Nudge Forward
            if (gamepad1.y) {
                robot.fl.setPower(nudgeSpeed);
                robot.fr.setPower(nudgeSpeed);
                robot.bl.setPower(nudgeSpeed);
                robot.br.setPower(nudgeSpeed);
                sleep(100);
                Robot_Stop();
            }

            //Nudge Backwards
            if (gamepad1.a) {
                robot.fl.setPower(-nudgeSpeed);
                robot.fr.setPower(-nudgeSpeed);
                robot.bl.setPower(-nudgeSpeed);
                robot.br.setPower(-nudgeSpeed);
                sleep(100);
                Robot_Stop();
            }



            /*if (gamepad1.y) {
                if ((robot.leftRearDistanceSensor.getDistance(DistanceUnit.CM) <= backUpDistance)
                        && (robot.rightRearDistanceSensor.getDistance(DistanceUnit.CM) <= backUpDistance)) {
                    if ((robot.leftRearDistanceSensor.getDistance(DistanceUnit.CM)) >
                            ((robot.rightRearDistanceSensor.getDistance(DistanceUnit.CM)))) {
                        leftFirst = true;
                    }
                    // Start on whichever wheel is further away.
                    if (leftFirst) {
                        while ((robot.leftRearDistanceSensor.getDistance(DistanceUnit.CM) <= backUpDistance)
                                && (robot.leftRearDistanceSensor.getDistance(DistanceUnit.CM) >= minSensorDistance)) {
                            robot.fl.setPower(0);
                            robot.bl.setPower(0);
                            while (robot.leftRearDistanceSensor.getDistance(DistanceUnit.CM) >= minSensorDistance) {
                                robot.fl.setPower(robotAlignmentSpeed);
                                robot.bl.setPower(robotAlignmentSpeed);
                            }
                            robot.fl.setPower(0);
                            robot.bl.setPower(0);
                        }
                        if ((robot.rightRearDistanceSensor.getDistance(DistanceUnit.CM) <= backUpDistance)
                                && (robot.rightRearDistanceSensor.getDistance(DistanceUnit.CM) >= minSensorDistance)) {
                            robot.fr.setPower(0);
                            robot.br.setPower(0);
                            while (robot.rightRearDistanceSensor.getDistance(DistanceUnit.CM) >= minSensorDistance) {
                                robot.fr.setPower(robotAlignmentSpeed);
                                robot.br.setPower(robotAlignmentSpeed);
                            }
                            robot.fr.setPower(0);
                            robot.br.setPower(0);
                        }
                        leftFirst = false;
                    } else {
                        if ((robot.rightRearDistanceSensor.getDistance(DistanceUnit.CM) <= backUpDistance)
                                && (robot.rightRearDistanceSensor.getDistance(DistanceUnit.CM) >= minSensorDistance)) {
                            robot.fr.setPower(0);
                            robot.br.setPower(0);
                            while (robot.rightRearDistanceSensor.getDistance(DistanceUnit.CM) >= minSensorDistance) {
                                robot.fr.setPower(robotAlignmentSpeed);
                                robot.br.setPower(robotAlignmentSpeed);
                            }
                            robot.fr.setPower(0);
                            robot.br.setPower(0);
                        }

                        while ((robot.leftRearDistanceSensor.getDistance(DistanceUnit.CM) <= backUpDistance)
                                && (robot.leftRearDistanceSensor.getDistance(DistanceUnit.CM) >= minSensorDistance)) {
                            robot.fl.setPower(0);
                            robot.bl.setPower(0);
                            while (robot.leftRearDistanceSensor.getDistance(DistanceUnit.CM) >= minSensorDistance) {
                                robot.fl.setPower(robotAlignmentSpeed);
                                robot.bl.setPower(robotAlignmentSpeed);
                            }
                            robot.fl.setPower(0);
                            robot.bl.setPower(0);
                        }
                    }
                    /* sleep(100);
                    // Might need touch sensor for full verification in auto - distance sensors struggle
                    // with short distance detection
                    if ((robot.leftRearDistanceSensor.getDistance(DistanceUnit.CM) <= 4 )
                            && (robot.rightRearDistanceSensor.getDistance(DistanceUnit.CM) <= 4)) {
                        if (!leftBumperPosFlag) {
                            robot.backLeftLatchServo.setPosition(backLeftLatchServoMax);
                            leftBumperPosFlag = true;
                        }
                        if (!rightBumperPosFlag) {
                            robot.backRightLatchServo.setPosition(backRightLatchServoMax);
                            rightBumperPosFlag = true;
                        }
                    }
                }
            }*/
        }
    }
}
