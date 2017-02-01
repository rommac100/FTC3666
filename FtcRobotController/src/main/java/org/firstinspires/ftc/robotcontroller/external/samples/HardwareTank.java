package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a K9 robot.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * 
 *
 *
 * Note: the configuration of the servos is such that:
 *   As the arm servo approaches 0, the arm position moves up (away from the floor).
 *   As the claw servo approaches 0, the claw opens up (drops the game element).
 */
public class HardwareTank
{
    /* Public OpMode members. */
    public DcMotor  leftMotor   = null;
    public DcMotor  rightMotor  = null;
    public DcMotor  linearSlide = null;
    public DcMotor  spin1Motor = null;
    public DcMotor  spin2Motor = null;
    public DcMotor  flyWheelMotor1 = null;
    public DcMotor  flyWheelMotor2 = null;
    public Servo    beaconServo = null;
    public Servo    flagServo = null;
    public DeviceInterfaceModule device = null;
    public ColorSensor colourSensor = null;
    public final double distancePerRev = 18.84;
    public final double ticksPerInch = 53.4776;
    public double leftDrivePower;       //power level for left side drive train motor
    public double rightDrivePower;      //power level for right side drive train motor
    public double innerIntakePower;     //power level for the inner intake
    public double outerIntakePower;     //power level for the outer intake
    public double systemFlyPower;       //current power level for fly motors
    public double marvinPos = .5;
    public double defaultFlyPower = .7;
    public double linearSlidePower;
    public double liveFlyPowerSetting = defaultFlyPower;
    public int maxSlideHeight = 1000;   //In theory this is low eneugh of a end height that we will have no problems in the short run, and can fine tune further from here.
                                        //should be less than one rotation right?
    public int ledChannel = 5;

    public double minBangValue = 0;// for bangbang for the flywheels
    public double maxBangValue = 1; // for bangbang for the flywheels

    /* Local OpMode members. */
    HardwareMap hwMap  = null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareTank() {
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // save reference to HW Map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftMotor   = hwMap.dcMotor.get("left_drive");
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightMotor  = hwMap.dcMotor.get("right_drive");
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        spin1Motor = hwMap.dcMotor.get("spin1");
        spin2Motor = hwMap.dcMotor.get("spin2");
        flyWheelMotor1 = hwMap.dcMotor.get("fly1");
        flyWheelMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flyWheelMotor2 = hwMap.dcMotor.get("fly2");
        flyWheelMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flyWheelMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flyWheelMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        spin1Motor.setDirection(DcMotor.Direction.REVERSE);
        spin2Motor.setDirection(DcMotor.Direction.REVERSE);
        flyWheelMotor1.setDirection(DcMotor.Direction.REVERSE);
        device = hwMap.deviceInterfaceModule.get("deviceINT");
        colourSensor = hwMap.colorSensor.get("colour_sensor");
        // Set all motors to zero power
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        spin1Motor.setPower(0);
        spin2Motor.setPower(0);
        flyWheelMotor1.setPower(0);
        flyWheelMotor2.setPower(0);

        flyWheelMotor1.setMaxSpeed(1200);
        flyWheelMotor2.setMaxSpeed(1200);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.


        // Define and initialize ALL installed servos.

    }


    public void drive(int direction, double power, int ticks, ElapsedTime runtime, LinearOpMode linear)
    {
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftMotor.setTargetPosition(ticks);
        rightMotor.setTargetPosition(ticks);
        double timeTemp = runtime.seconds()+10;
        switch (direction)
        {
            case 0:
                leftMotor.setPower(power);
                rightMotor.setPower(power);
                while(leftMotor.isBusy() && rightMotor.isBusy())
                {

                }
                leftMotor.setPower(0);
                rightMotor.setPower(0);

                leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                break;
            case 1:
                leftMotor.setPower(power);
                rightMotor.setPower(-1*power);
                break;
            case 2:
                leftMotor.setPower(-1*power);
                rightMotor.setPower(-1*power);
                while(leftMotor.isBusy() && rightMotor.isBusy() && linear.opModeIsActive() && runtime.seconds() < timeTemp)
                {

                }

                leftMotor.setPower(0);
                rightMotor.setPower(0);

                leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                break;
            case 3:
                leftMotor.setPower(-1*power);
                leftMotor.setPower(1*power);
                break;


        }
    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     */

    public void waitForTick(long periodMs) {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}
