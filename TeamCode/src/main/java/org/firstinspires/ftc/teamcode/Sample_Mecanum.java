package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Disabled
@TeleOp
public class Sample_Mecanum extends LinearOpMode {

    private DcMotorEx Front_Right;
    private DcMotorEx Front_Left;
    private DcMotorEx Back_Right;
    private DcMotorEx Back_Left;

    @Override
    public void runOpMode() throws InterruptedException {

        double fwdBackPower, strafePower, turnPower, maxPower;

        // Assigning the motors on the robot to the variables we've created to operate
        // the motors
        Back_Right = hardwareMap.get(DcMotorEx.class, "Back_Right");
        Back_Left = hardwareMap.get(DcMotorEx.class, "Back_Left");
        Front_Right = hardwareMap.get(DcMotorEx.class, "Front_Right");
        Front_Left = hardwareMap.get(DcMotorEx.class, "Front_Left");

        // Set directions of motors for left and right side of drivetrain
        Front_Left.setDirection(DcMotor.Direction.REVERSE);
        Back_Left.setDirection(DcMotor.Direction.REVERSE);
        Front_Right.setDirection(DcMotor.Direction.FORWARD);
        Back_Right.setDirection(DcMotor.Direction.FORWARD);

        Front_Left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Front_Right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Back_Left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Back_Right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Do not use waitForStart() since we have Motorola E4 phones.
        //They have a known bug to time out when using Waitforstart()

        //waitForStart();
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("status", "waiting for start command...");
            telemetry.update();
        }

        while (opModeIsActive()) {

            fwdBackPower = -gamepad1.left_stick_y;
            strafePower = gamepad1.left_stick_x;
            turnPower = gamepad1.right_stick_x;

            double FrontLeftVal = fwdBackPower + turnPower + strafePower;
            double FrontRightVal = fwdBackPower - turnPower - strafePower;
            double BackLeftVal = fwdBackPower + turnPower - strafePower;
            double BackRightVal = fwdBackPower - turnPower + strafePower;

            maxPower = Math.abs(FrontLeftVal);
            if(Math.abs(FrontRightVal) > maxPower) {maxPower = Math.abs(FrontRightVal);}
            if(Math.abs(BackLeftVal) > maxPower) {maxPower = Math.abs(BackLeftVal);}
            if(Math.abs(BackRightVal) > maxPower) {maxPower = Math.abs(BackRightVal);}

            if (maxPower > 1) {
                FrontLeftVal = FrontLeftVal/maxPower;
                FrontRightVal = FrontRightVal/maxPower;
                BackLeftVal = BackLeftVal/maxPower;
                BackRightVal = BackRightVal/maxPower;
            }

            // Set power to wheels using mecanum power formula above
            Front_Left.setPower(FrontLeftVal);
            Front_Right.setPower(FrontRightVal);
            Back_Left.setPower(BackLeftVal);
            Back_Right.setPower(BackRightVal);

            //Added read out of motor values to control screen on driver station
            telemetry.addData("Left Joystick Y", gamepad1.left_stick_y);
            telemetry.addData("Left Joystick X", gamepad1.left_stick_x);
            telemetry.addData("Right Joystick Y", gamepad1.right_stick_y);
            telemetry.addData("Right Joystick X", gamepad1.right_stick_x);

            telemetry.addData("Front Right Input", FrontLeftVal);
            telemetry.addData("Front Left Input", FrontRightVal);
            telemetry.addData("Back Right Input", BackLeftVal);
            telemetry.addData("Back Left Input", BackRightVal);

            telemetry.update();
        }
    }
}
