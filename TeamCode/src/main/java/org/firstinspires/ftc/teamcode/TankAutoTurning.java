
package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwareTank;

import java.util.Locale;


/*
This auto drives forward, fires two projectiles, and then goes turns and attempts to capture the beacon. Afterwards it attempts to park and knock off the capball.
 */

@Autonomous(name="Tank: AutotomousTurning", group="Tank")
public class TankAutoTurning extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareTank robot   = new HardwareTank();
    private ElapsedTime     runtime = new ElapsedTime();

    private double centerQ = 0;

    public int distance(double dis)
    {
        return (int)(dis*robot.ticksPerInch);
    }

    public enum DIRECTION {
        FORWARD(+0.3), REVERSE(.25), Clockwise(.25), Counter_Clockwise(-.25);
        public final double value;

        DIRECTION(double value) {
            this.value = value;
        }
    }

    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    private int fEncoder = 0;
    private int fLastEncoder = 0;

    private long fVelocityTime = 0;
    private long fLastVelocityTime = 0;

    private double motorOut = 0.0;
    private double fTarget = 7.5e-7;
    private double fVelocity = 0.0;
    private double fError = 0.0;
    private double fLastError = 0.0;
    private double tolerance = 0.5e-7;


    public void drive(DIRECTION direction, int ticks) {
        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        double timeTemp = runtime.seconds() + 10;
        switch (direction) {
            case FORWARD:
                robot.leftMotor.setTargetPosition(ticks);
                robot.rightMotor.setTargetPosition(ticks);
                robot.leftMotor.setPower(DIRECTION.FORWARD.value);
                robot.rightMotor.setPower(DIRECTION.FORWARD.value);
                while (robot.leftMotor.isBusy() && robot.rightMotor.isBusy() && opModeIsActive()) {

                }
                robot.leftMotor.setPower(0);
                robot.rightMotor.setPower(0);

                robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                break;
            case REVERSE:
                robot.leftMotor.setTargetPosition(-ticks);
                robot.rightMotor.setTargetPosition(-ticks);

                robot.leftMotor.setPower(DIRECTION.FORWARD.value);
                robot.rightMotor.setPower(DIRECTION.FORWARD.value);
                while (robot.leftMotor.isBusy() && robot.rightMotor.isBusy() && opModeIsActive()) {
                    telemetry.addData("motorLeft Pos", robot.leftMotor.getCurrentPosition());
                    telemetry.update();
                }

                robot.leftMotor.setPower(0);
                robot.rightMotor.setPower(0);

                robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                break;
        }
    }

    public double[] getAngles() {
        Quaternion quatAngles = imu.getQuaternionOrientation();

        double w = quatAngles.w;
        double x = quatAngles.x;
        double y = quatAngles.y;
        double z = quatAngles.z;

        // for the Adafruit IMU, yaw and roll are switched
        double roll = Math.atan2( 2*(w*x + y*z) , 1 - 2*(x*x + y*y) ) * 180.0 / Math.PI;
        double pitch = Math.asin( 2*(w*y - x*z) ) * 180.0 / Math.PI;
        double yaw = Math.atan2( 2*(w*z + x*y), 1 - 2*(y*y + z*z) ) * 180.0 / Math.PI;

        return new double[]{yaw, pitch, roll};
    }

    public void calibrateIMU()
    {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.useExternalCrystal = true;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.pitchMode = BNO055IMU.PitchMode.WINDOWS;
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu.initialize(parameters);
    }

    public double getHeading() {return getAngles()[0];}

    public boolean turningDriveBoolean(double power, int angle, float angleDesired)
    {
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        boolean temp = true;

        if (angle < 0)
        {
            robot.leftDrivePower = -power;
            robot.rightDrivePower = power;

            if (getHeading() < angleDesired)
            {
                robot.leftDrivePower = 0;
                robot.rightDrivePower = 0;
                temp = false;

            }
            robot.leftMotor.setPower(robot.leftDrivePower);
            robot.rightMotor.setPower(robot.rightDrivePower);
        }
        else if (angle > 0)
        {
            robot.leftDrivePower = power;
            robot.rightDrivePower = -power;

            if (getHeading() > angleDesired)
            {
             robot.leftDrivePower = 0;
                robot.rightDrivePower =0;
                temp = false;
            }
            robot.leftMotor.setPower(robot.leftDrivePower);
            robot.rightMotor.setPower(robot.rightDrivePower);
        }
        return temp;
    }

    private void setFPower(double power) {
        robot.flyWheelMotor1.setPower(power);
        robot.flyWheelMotor2.setPower(power);
    }

    public void bangBang() {
        fVelocityTime = System.nanoTime();
        fEncoder = robot.flyWheelMotor1.getCurrentPosition();
        fVelocity = (double) (fEncoder - fLastEncoder) / (fVelocityTime - fLastVelocityTime);
        if (fVelocity >= (fTarget + tolerance)) {
            setFPower(.35);
        } else if (fVelocity < (fTarget - tolerance)) {
            setFPower(.44);
        }

        fLastEncoder = fEncoder;
        fLastVelocityTime = fVelocityTime;
    }

    public double colourSensorCheck(String teamColour)
    {
        int red = robot.colourSensor.red();
        int blue = robot.colourSensor.blue();

        int x = 0;


        //averaging loop
        telemetry.addData("redVal", red);
        telemetry.addData("blueVal", blue);

        telemetry.update();

        //Different conditionals relating the colour sensor output + team Alliance colour.
        if (teamColour.equals("blue")) {
            if ((red) > blue) {
                return .2;
            }
            else if (red < blue)
            {
                return .9;
            }
            else
            {
                return colourSensorCheck("blue");
            }
        }
        else if (teamColour.equals("red"))
        {
            if ((red) > blue)
            {
                return .2;
            }
            else if (red < blue)
            {
                return .9;
            }
            else {
                return colourSensorCheck("blue");
            }
        }
    return .5;
    }

    public void flyWheelShooter(double duration) {

        while (runtime.seconds() < duration && opModeIsActive()) {
            bangBang();
            sleep(500);
            robot.spin1Motor.setPower(.4);
            bangBang();
        }

        setFPower(0);
        robot.spin1Motor.setPower(0);

    }

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.device.setDigitalChannelMode(0, DigitalChannelController.Mode.OUTPUT);

        robot.device.setDigitalChannelState(0, false);

        robot.leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");
        telemetry.addData("Alliance Colour", "Red or Blue");
        telemetry.update();


        imu = hardwareMap.get(BNO055IMU.class, "imu");
        calibrateIMU();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        centerQ= robot.device.getAnalogInputVoltage(4);


        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);
        runtime.reset();
        composeTelemetry();

        float angleDesired = 0;
        float angleDesired2 = 0;


        boolean driveForward1 = true;
        boolean flywheels = false;
        boolean turnDrive1 = false;
        boolean driveForward2 = false;
        boolean driveForward3 = false;
        boolean turnDrive2 = false;
        boolean colourSensorGo = false;

        //used for the flyWheels
        double desiredTime = 0;

        while (opModeIsActive()) {

            if (driveForward1)
            {
                drive(DIRECTION.FORWARD, distance(22));

                flywheels = true;
                driveForward1=false;

            }
            else if (flywheels)
            {
                flyWheelShooter(10);
                    turnDrive1 = true;
                    flywheels = false;
            }
            else if (turnDrive1)
            {
                if (angleDesired == 0)
                {
                    angleDesired = 35;
                }
                turnDrive1 = turningDriveBoolean(.1,  35, angleDesired);
                driveForward2 = !turnDrive1;
            }
            else if (driveForward2)
            {
                telemetry.addData("drive2", "");
                telemetry.update();
                drive(DIRECTION.FORWARD, distance(43));
                turnDrive2 = true;
                driveForward2=false;
            }
            else if (turnDrive2)
            {
                if (angleDesired2 == 0)
                {
                    angleDesired2 = -70;
                }
                turnDrive2 = turningDriveBoolean(.1,  -80, angleDesired2);
                if (!turnDrive2)
                {
                driveForward3 = true;
                }

                turnDrive2 = true;
                driveForward3 = true;
            }
            else if (driveForward3)
            {

                drive(DIRECTION.REVERSE,distance(29));
                colourSensorGo = true;
                driveForward3 = false;

            }

            else if (colourSensorGo)
            {
                drive(DIRECTION.REVERSE, distance(5));
                drive(DIRECTION.FORWARD, distance(40));
                colourSensorGo = false;
            }

            telemetry.update();
            }
    }
    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
            gravity  = imu.getGravity();
        }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel*gravity.xAccel
                                        + gravity.yAccel*gravity.yAccel
                                        + gravity.zAccel*gravity.zAccel));
                    }
                });
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

}


