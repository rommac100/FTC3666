package org.firstinspires.ftc.robotcontroller.external.samples;

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
    public DcMotor  spin1Motor = null;
    public DcMotor  spin2Motor = null;
    public DcMotor  flyWheelMotor1 = null;
    public DcMotor  flyWheelMotor2 = null;
    public Servo    beaconServo = null;
    public DeviceInterfaceModule device = null;
    public final double distancePerRev = 18.84;
    public final double ticksPerInch = 53.4776;
    public double leftDrivePower;      //power level for left side drive train motor
    public double rightDrivePower;     //power level for right side drive train motor
    public double innerIntakePower;    //power level for the inner intake
    public double outerIntakePower;    //power level for the outer intake
    public double systemFlyPower;
    public double marvinPos = .5;


    /* Local OpMode members. */
    HardwareMap hwMap  = null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareTank() {
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // save reference to HW Mapff
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
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        spin1Motor.setDirection(DcMotor.Direction.REVERSE);
        spin2Motor.setDirection(DcMotor.Direction.REVERSE);
        flyWheelMotor1.setDirection(DcMotor.Direction.REVERSE);
        beaconServo = hwMap.servo.get("beaconS");
        device = hwMap.deviceInterfaceModule.get("deviceINT");

        // Set all motors to zero power
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        spin1Motor.setPower(0);
        spin2Motor.setPower(0);
        flyWheelMotor1.setPower(0);
        flyWheelMotor2.setPower(0);
        beaconServo.setPosition(.5);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.


        // Define and initialize ALL installed servos.

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
