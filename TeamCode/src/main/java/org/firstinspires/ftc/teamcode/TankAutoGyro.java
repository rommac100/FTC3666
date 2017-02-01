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

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwareTank;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;

import java.util.Locale;

import static android.R.attr.angle;

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
@Autonomous(name="Tank: AutotomousGyroTurning", group="Tank")
public class TankAutoGyro extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareTank robot   = new HardwareTank();
    private ElapsedTime     runtime = new ElapsedTime();

    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    boolean firstAngle = true;
    boolean secondAngle = true;

    public int distance(double dis)
    {
        return (int)(dis*robot.ticksPerInch);
    }

    public enum DIRECTION {
        FORWARD (+0.25), REVERSE (.25), Clockwise (.25), Counter_Clockwise(-.25);
        public final double value;
        DIRECTION (double value) {this.value = value;}
    }
    public enum Direction {
        FORWARD (+1.0), REVERSE (+1.0), Clockwise (-1.0), Counter_Clockwise(+1.0);
        public final double value;
        Direction (double value) {this.value = value;}
    }


    //0 = forward, 2 = reverse
    public void drive(DIRECTION direction, int ticks)
    {
        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.leftMotor.setTargetPosition(ticks);
        robot.rightMotor.setTargetPosition(ticks);
        double timeTemp = runtime.seconds()+10;
        switch (direction)
        {
            case FORWARD:
                robot.leftMotor.setPower(DIRECTION.FORWARD.value);
                robot.rightMotor.setPower(DIRECTION.FORWARD.value);
                while(robot.leftMotor.isBusy() && robot.rightMotor.isBusy())
                {

                }
                robot.leftMotor.setPower(0);
                robot.rightMotor.setPower(0);

                robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                break;
            case REVERSE:
                robot.leftMotor.setTargetPosition(-ticks);
                robot.rightMotor.setTargetPosition(-ticks);

                robot.leftMotor.setPower(DIRECTION.REVERSE.value);
                robot.rightMotor.setPower(DIRECTION.REVERSE.value);
                while(robot.leftMotor.isBusy() && robot.rightMotor.isBusy() && opModeIsActive())
                {
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
    public double adjustAngle(double angle)
    {
        while (angle > 180) angle -= 360;
        while (angle <= -180) angle += 360;
        return angle;
    }

    public void driveTrain(double leftPow, double rightPow)
    {
        robot.leftMotor.setPower(leftPow);
        robot.rightMotor.setPower(rightPow);
    }


    public void turnP(double degrees, Direction direction, double timeout, double speed, double kp) {
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double targetAngle = adjustAngle(getHeading() + direction.value * degrees);
        double error;
        double power;

        do {
            error = adjustAngle(targetAngle - getHeading());
            power = kp * error;
            power = Range.clip(power, -speed, +speed);
            driveTrain(-power, power);
            idle();
        } while (opModeIsActive() && Math.abs(error) > .5);
        driveTrain(0,0);
    }
    public void turnDrive(double angle)
    {
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DIRECTION direction = DIRECTION.Clockwise;

        double desiredAngle = getHeading() + angle;


        desiredAngle = adjustAngle(desiredAngle);
        if (desiredAngle > 0)
        {
            direction = DIRECTION.Clockwise;
        }
        else if (desiredAngle < 0)
        {
            direction = DIRECTION.Counter_Clockwise;
        }
        telemetry.addData("desiredAngle", desiredAngle);
        switch (direction)
        {
            case Clockwise:
                robot.leftDrivePower = DIRECTION.Clockwise.value;
                robot.rightDrivePower = -robot.leftDrivePower;

                while (opModeIsActive() && getHeading() < desiredAngle)
                {
                    telemetry.update();
                    if (desiredAngle - getHeading() > 10 && desiredAngle - getHeading() < 20)
                    {
                        robot.leftDrivePower = robot.leftDrivePower/2;
                        robot.rightDrivePower = -robot.rightDrivePower;
                    }
                    robot.leftMotor.setPower(robot.leftDrivePower);
                    robot.rightMotor.setPower(robot.rightDrivePower);
                }
                robot.rightMotor.setPower(0);
                robot.leftMotor.setPower(0);
                break;
            case Counter_Clockwise:
                robot.rightDrivePower = DIRECTION.Counter_Clockwise.value;
                robot.leftDrivePower = robot.rightDrivePower;

                while (opModeIsActive() && getHeading() > desiredAngle)
                {
                    telemetry.update();
                    robot.leftMotor.setPower(robot.leftDrivePower);
                    robot.rightMotor.setPower(robot.rightDrivePower);
                }




        }
    }

    public boolean turningDriveBoolean(double power, int angle, float angleDesired) {
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        boolean temp = true;

        if (angle < 0) {
            robot.leftDrivePower = power;
            robot.rightDrivePower = -power;

            if (AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle) < angleDesired) {
                robot.leftDrivePower = 0;
                robot.rightDrivePower = 0;

                temp = false;

            }
            robot.leftMotor.setPower(robot.leftDrivePower);
            robot.rightMotor.setPower(robot.rightDrivePower);
        } else if (angle > 0) {
            robot.leftDrivePower = -power;
            robot.rightDrivePower = power;

            if (AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle) > angleDesired) {
                robot.leftDrivePower = 0;
                robot.rightDrivePower = 0;
                temp = false;
            }
            robot.leftMotor.setPower(robot.leftDrivePower);
            robot.rightMotor.setPower(robot.rightDrivePower);
        }
        return temp;
    }

    public double getHeading() {return getAngles()[0];}

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



        imu = hardwareMap.get(BNO055IMU.class, "imu");
        calibrateIMU();

        double tempTime = 0;
        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        //Step 1: drive forward one foam Pad
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);
        runtime.reset();
        composeTelemetry();



        //telemetry.addData("Angle",angleDesired);

        float angleDesired = 0;
        float angleDesired2 = 0;


        boolean driveForward1 = true;
        boolean flywheels = false;
        boolean turnDrive1 = false;
        boolean driveForward2 = false;
        boolean turnDrive2 = false;

        //used for the flyWheels
        double desiredTime = 0;

        while (opModeIsActive()) {
            //sleep(2000);
            //sleep(1000);

            //first conditional is relating to the inital movement


            if (driveForward1)
            {
                drive(DIRECTION.FORWARD, distance(22));

                turnDrive1 = true;
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

                if (runtime.seconds() > desiredTime -2)
                {
                    robot.spin2Motor.setPower(.4);
                }
                if (runtime.seconds() >= desiredTime)
                {
                    robot.flyWheelMotor1.setPower(0);
                    robot.flyWheelMotor2.setPower(0);

                    robot.spin2Motor.setPower(0);

                    turnDrive1 = true;
                    flywheels = false;
                }
            }
            else if (turnDrive1)
            {
                //turnDrive(73);
               turnP(90,Direction.Clockwise, 10, .2, 1);
                turnDrive1 =false;
            }

/*
            if (runtime.seconds() < 5) {
                drive(2, .25, distance(22));
            }
            //second conditional is relating to the firing of the particles
            else if (runtime.seconds() < 12)
            {
                if (runtime.seconds() > 7)
                {
                    robot.flyWheelMotor1.setPower(robot.defaultFlyPower);
                    robot.flyWheelMotor2.setPower(robot.defaultFlyPower);
                }
                else if (runtime.seconds() < 11)
                {
                    robot.spin2Motor.setPower(.4);
                }
                else if (runtime.seconds() >= 11)
                {
                    robot.spin2Motor.setPower(0);
                    robot.flyWheelMotor1.setPower(0);
                    robot.flyWheelMotor2.setPower(0);
                }

            }
            else if (runtime.seconds() < 15)
            {
                turningDrive(.1, 45);
            }
            else if (runtime.seconds() < 18)
            {
                drive(2, .25, distance(22));
            }
            else if (runtime.seconds() < 21)
            {
                turningDrive(.1, 45);
            }




*/
            //turningDrive(.1, 90);
            //telemetry.addData("Angle", angleDesired);
/*
            if (!firstMove)
            {
                drive(2, .25, distance(22));
                flyWheel = false;
                firstMove = true;

            }
            else if (!flyWheel)
            {
                if (tempTime == 0)
                {
                    tempTime = runtime.seconds() + 8;
                }

                if (runtime.seconds() < tempTime)
                {
                    robot.flyWheelMotor1.setPower(0.7);
                    robot.flyWheelMotor2.setPower(0.7);

                    if (runtime.seconds() > tempTime + 3)
                    {
                        robot.spin2Motor.setPower(.4);
                    }
                }
                robot.spin1Motor.setPower(0);
                robot.spin2Motor.setPower(0);
                robot.flyWheelMotor1.setPower(0);
                robot.flyWheelMotor2.setPower(0);
                firstTurn = false;
                flyWheel = true;
            }
            else if (!firstTurn) {
                if (AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle) < angleDesired) {
                    robot.leftDrivePower = 0;
                    robot.rightDrivePower = 0;
                    secondMove = false;
                    firstTurn = true;
                }

                robot.rightMotor.setPower(robot.rightDrivePower);
                robot.leftMotor.setPower(robot.leftDrivePower);
            }
            */
            /*
            else if (!secondMove)
            {
                drive(2, .25, distance(22));
                secondTurn =false;
                secondMove = true;
            }
            else if (!secondTurn)
            {
                robot.leftDrivePower = .1;
                robot.rightDrivePower = -.1;
                if (AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle) > angleDesired2) {
                    robot.leftDrivePower = 0;
                    robot.rightDrivePower = 0;
                    secondTurn = true;
                }
                robot.rightMotor.setPower(robot.rightDrivePower);
                robot.leftMotor.setPower(robot.leftDrivePower);

            }
            */
            telemetry.update();
        }








        /*
        drive(2, .25, distance(22));
        sleep(1000);


            double tempTime = runtime.seconds() + 8;
            while (runtime.seconds() < tempTime) {
                if (runtime.seconds() < tempTime - 3) {

                    robot.flyWheelMotor1.setPower(.7);
                    robot.flyWheelMotor2.setPower(.7);
                }



                //robot.spin1Motor.setPower(.8);
                robot.spin2Motor.setPower(.4);
            }
            robot.spin1Motor.setPower(0);
            robot.spin2Motor.setPower(0);
            robot.flyWheelMotor1.setPower(0);
            robot.flyWheelMotor2.setPower(0);

            turningDrive(-.5, distance(100));
*/
        //runtime.reset();
        //sleep(2000);
        //drive(2, 1, distance(35));


        // Step 4:  Stop and close the claw



        //sleep(1000);
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

