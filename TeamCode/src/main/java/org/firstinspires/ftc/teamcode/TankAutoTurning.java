/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwareTank;

import java.util.Locale;

/**
 * This file illustrates the concept of driving a path based on time.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backwards for 1 Second
 *   - Stop and close the claw.
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@Autonomous(name="Tank: AutotomousTurning", group="Tank")
public class TankAutoTurning extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareTank robot   = new HardwareTank();
    private ElapsedTime     runtime = new ElapsedTime();

    private double centerQ = 0;

    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    public int distance(double dis)
    {
        return (int)(dis*robot.ticksPerInch);
    }


    //0 = forward, 2 = reverse
    public void drive(int direction, double power, int ticks)
    {
        telemetry.addData("Drive", direction);
        telemetry.update();
        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.leftMotor.setTargetPosition(ticks);
        robot.rightMotor.setTargetPosition(ticks);
        double timeTemp = runtime.seconds()+10;


        if (direction ==0)
        {
            robot.leftMotor.setPower(power);
            robot.rightMotor.setPower(power);
            while(robot.leftMotor.isBusy() && opModeIsActive())
            {

            }
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);

            robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        else if (direction == 2)
        {
            telemetry.addData("Case 2", "");
            telemetry.update();
            robot.leftMotor.setTargetPosition(-ticks);
            robot.rightMotor.setTargetPosition(-ticks);
            robot.leftMotor.setPower(power);
            robot.rightMotor.setPower(power);

            while(robot.leftMotor.isBusy() && opModeIsActive())
            {
                telemetry.addData("motorLeft Pwr", robot.leftMotor.getPower());
            }

            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);

            robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

    }

        public void betterTurn(double power, float angleDesired)
        {
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            boolean temp = true;
            float angle;
            int interval = 3;
            double temptime=1;
            while(temp)
            {

                angle = AngleUnit.DEGREES.fromUnit(angles.angleUnit,angles.firstAngle);
                if (angle < angleDesired)
                {
                    robot.leftDrivePower = -power;
                    robot.rightDrivePower = power;


                    robot.leftMotor.setPower(robot.leftDrivePower);
                    robot.rightMotor.setPower(robot.rightDrivePower);
                }
                else if (angle > angleDesired)
                {
                    robot.leftDrivePower = power;
                    robot.rightDrivePower = -power;


                    robot.leftMotor.setPower(robot.leftDrivePower);
                    robot.rightMotor.setPower(robot.rightDrivePower);
                }

            }

        }


    public boolean turningDriveBoolean(double power, int angle, float angleDesired)
    {
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        boolean temp = true;

        if (angle < 0)
        {
            robot.leftDrivePower = -power;
            robot.rightDrivePower = power;

            if (AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle) < angleDesired)
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

            if (AngleUnit.DEGREES.fromUnit(angles.angleUnit,angles.firstAngle) > angleDesired)
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

    public void turningDrive(double power, int angle)
    {
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        float angleDesired = AngleUnit.DEGREES.fromUnit(angles.angleUnit,angles.firstAngle)+angle;
        if (angle < 0) {
            robot.leftDrivePower = power;
            robot.rightDrivePower = -power;


                if (AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle) < angleDesired) {
                    robot.leftDrivePower = 0;
                    robot.rightDrivePower = 0;
                }
                robot.leftMotor.setPower(robot.leftDrivePower);
                robot.rightMotor.setPower(robot.rightDrivePower);

                telemetry.update();
            }

        else
        {
            robot.leftDrivePower = -power;
            robot.rightDrivePower = power;

                if (AngleUnit.DEGREES.fromUnit(angles.angleUnit,angles.firstAngle) > angleDesired)
                {
                    robot.leftDrivePower = 0;
                    robot.rightDrivePower = 0;
                }
                robot.leftMotor.setPower(robot.leftDrivePower);
                robot.rightMotor.setPower(robot.rightDrivePower);
                telemetry.update();


        }
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public double colourSensorCheck(String teamColour)
    {
        int red = robot.colourSensor.red();
        int blue = robot.colourSensor.blue();

        int x = 0;
        int blueAvg = 0;

        //averaging loop
        while (opModeIsActive() && x <= 3)
        {
            blueAvg += blue;
            sleep(20);
            blue = robot.colourSensor.blue();
            x++;
        }
        x= 0;

        int redAvg = 0;
        //averaging loop
        while (opModeIsActive() && x <=3)
        {
            redAvg += red;
            sleep(20);
            red = robot.colourSensor.red();
            x++;
        }

        //Different conditionals relating the colour sensor output + team Alliance colour.
        if (teamColour.equals("blue")) {
            if ((redAvg / 3) > blueAvg / 3) {
                return .2;
            }
            else if (redAvg /3 > blueAvg /3 )
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
            if ((redAvg/3) > blueAvg/3)
            {
                return .9;
            }
            else if (redAvg/3 < blueAvg/3)
            {
                return .2;
            }
            else {
                return colourSensorCheck("blue");
            }
        }
    return .5;
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

        robot.leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");
        telemetry.addData("Alliance Colour", "Red or Blue");
        telemetry.update();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        angles   = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        centerQ= robot.device.getAnalogInputVoltage(4);
        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way

        //Step 1: drive forward one foam Pad
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
                drive(0, .25, distance(22));

                flywheels = true;
                driveForward1=false;

            }
            else if (flywheels)
            {
                if (desiredTime == 0)
                {
                    desiredTime = runtime.seconds() + 10;
                }
                robot.flyWheelMotor1.setPower(robot.defaultFlyPower);
                robot.flyWheelMotor2.setPower(robot.defaultFlyPower);

                if (runtime.seconds() > desiredTime -8)
                {
                    robot.spin1Motor.setPower(.4);
                }
                if (runtime.seconds() >= desiredTime)
                {
                    robot.flyWheelMotor1.setPower(0);
                    robot.flyWheelMotor2.setPower(0);

                    robot.spin1Motor.setPower(0);

                    turnDrive1 = true;
                    flywheels = false;
                }
            }
            else if (turnDrive1)
            {
                if (angleDesired == 0)
                {
                    angleDesired = 5;
                }
                turnDrive1 = turningDriveBoolean(.1,  5, angleDesired);
                driveForward2 = !turnDrive1;
            }
            else if (driveForward2)
            {
                telemetry.addData("drive2", "");
                telemetry.update();
                drive(0,.25, distance(40));
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

            }
            else if (driveForward3)
            {
                drive (2,.25,distance(27));

                driveForward3 = false;
                colourSensorGo = true;
            }

            else if (colourSensorGo)
            {
                robot.beaconServo.setPosition(colourSensorCheck("red"));
                drive(0,.1,distance(2));
                drive(2, .1, distance(3));
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


