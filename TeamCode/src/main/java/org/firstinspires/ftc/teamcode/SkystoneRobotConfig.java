package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

// import org.openftc.easyopencv.OpenCvInternalCamera;

@Disabled
public class SkystoneRobotConfig {

    static final double COUNTS_PER_MOTOR_REV    = 537.6;
    static final double DRIVE_GEAR_REDUCTION    = 1;
    static final double WHEEL_DIAMETER_INCHES   = 3.94;
    static final double COUNT_PER_INCH          = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION)/
            (WHEEL_DIAMETER_INCHES * 3.1415);

    // drivetrain motors
    public DcMotorEx fr;
    public DcMotorEx fl;
    public DcMotorEx br;
    public DcMotorEx bl;

    public DigitalChannel magneticBottomLimit;
    public DigitalChannel magneticTopLimit;

    public ModernRoboticsI2cRangeSensor leftUltrasonic;
    public ModernRoboticsI2cRangeSensor rightUltrasonic;
    public ModernRoboticsI2cRangeSensor frontUltrasonic;

    public ModernRoboticsI2cGyro gyro;

    public RevColorSensorV3 blockDetector;
    public DistanceSensor blockDetectorDistance;
    public RevColorSensorV3 lineDetector;
    public DistanceSensor lineDetectorDistance;

    public Servo leftAutoDraggerServo, rightAutoDraggerServo, tamperServo, backLeftLatchServo, backRightLatchServo,  capstoneScoreServo, gripperServo, swingerServo;

    public CRServo tapeMeasure;

    public DcMotor rightLiftMotor, rightIntakeMotor, leftIntakeMotor, leftLiftMotor;

    // public BNO055IMU imu;

    public DistanceSensor rightRearDistanceSensor;  // Lift side
    public DistanceSensor leftRearDistanceSensor;  // Phone side

    //public OpenCvInternalCamera phoneCam;

    //VuforiaLocalizer vuforia;

    HardwareMap hwMap           = null;

    public SkystoneRobotConfig() {

    }

    public void init(HardwareMap ahwMap) {
        // Save reference hardware map
        hwMap = ahwMap;

        // Assigning the motors on the robot to the variables we've created to operate
        // the motors
        br = hwMap.get(DcMotorEx.class, "BackRight");
        bl = hwMap.get(DcMotorEx.class, "BackLeft");
        fr = hwMap.get(DcMotorEx.class, "FrontRight");
        fl = hwMap.get(DcMotorEx.class, "FrontLeft");

        // Set directions of motors for left and right side of drivetrain
        // Reversed these on 12/8 - may need to switch back
        fl.setDirection(DcMotor.Direction.FORWARD);
        bl.setDirection(DcMotor.Direction.FORWARD);
        fr.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.REVERSE);

        // Set motors to brake when no power is applied
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Set all motors to zero power at the start
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);

        // Initialize non drivetrain motors
        rightLiftMotor = hwMap.get(DcMotor.class, "RightLift");
        leftLiftMotor = hwMap.get(DcMotor.class, "LeftLift");
        rightIntakeMotor = hwMap.get(DcMotor.class,"RightIntake");
        leftIntakeMotor = hwMap.get(DcMotor.class,"LeftIntake");

        leftIntakeMotor.setDirection(DcMotor.Direction.FORWARD);
        rightIntakeMotor.setDirection(DcMotor.Direction.REVERSE);

        // Lift Motors are on reverse sides; Lift2 is on the swing side
        rightLiftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLiftMotor.setDirection(DcMotor.Direction.REVERSE );
        leftLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize our gyroscope
        //imu = hwMap.get(BNO055IMU.class,"imu");

        // Initialize alternative modern robotics gyro
        gyro = hwMap.get(ModernRoboticsI2cGyro.class, "Gyro");

        // Initialize all of our servos
        leftAutoDraggerServo = hwMap.get(Servo.class,"LeftAutoDraggerServo");
        rightAutoDraggerServo = hwMap.get(Servo.class,"RightAutoDraggerServo");
        tamperServo = hwMap.get(Servo.class,"Tamper");
        capstoneScoreServo = hwMap.get(Servo.class,"CapstoneScorer");
        backLeftLatchServo = hwMap.get(Servo.class,"BackLeftLatch");
        backRightLatchServo = hwMap.get(Servo.class,"BackRightLatch");
        gripperServo = hwMap.get(Servo.class,"Gripper");
        swingerServo = hwMap.get(Servo.class,"Swinger");

        // Initializing our color sensors & their integrated distance sensor
        // Distance sensors detect when an object is near to draw a color reading
        blockDetector = hwMap.get(RevColorSensorV3.class,"BlockDetector");
        blockDetectorDistance = hwMap.get(DistanceSensor.class, "BlockDetector");
        lineDetector = hwMap.get(RevColorSensorV3.class,"LineDetector");
        lineDetectorDistance = hwMap.get(DistanceSensor.class, "LineDetector");

        // Initializing our limit switches
        magneticBottomLimit = hwMap.get(DigitalChannel.class,"MagneticBottomLimit");
        magneticTopLimit = hwMap.get(DigitalChannel.class,"MagneticTopLimit");

        magneticBottomLimit.setMode(DigitalChannel.Mode.INPUT);
        magneticTopLimit.setMode(DigitalChannel.Mode.INPUT);

        // Initialize our ultrasonic sensors
        leftUltrasonic = hwMap.get(ModernRoboticsI2cRangeSensor.class,"LeftUltrasonic");
        rightUltrasonic = hwMap.get(ModernRoboticsI2cRangeSensor.class,"RightUltrasonic");
        frontUltrasonic = hwMap.get(ModernRoboticsI2cRangeSensor.class, "Front_Ultrasonic");

        // Change direction of back right latch
        backRightLatchServo.setDirection(Servo.Direction.REVERSE);

        // Change direction of front right block dragger
        rightAutoDraggerServo.setDirection(Servo.Direction.REVERSE);

        // Get rear distance sensors
        rightRearDistanceSensor = hwMap.get(DistanceSensor.class, "RightSideDistanceSensor");
        leftRearDistanceSensor = hwMap.get(DistanceSensor.class, "LeftSideDistanceSensor");

        tapeMeasure = hwMap.crservo.get("tapeMeasure");
    }

}
