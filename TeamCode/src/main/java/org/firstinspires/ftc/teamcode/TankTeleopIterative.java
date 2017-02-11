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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwareTank;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Hardware Tank: TankTeleOpIterative", group="Iterative Opmode")  // @Autonomous(...) is the other common choice
public class TankTeleopIterative extends OpMode
{
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    HardwareTank robot = new HardwareTank();

    private double maxDriveTrain;
    private double maxIntakeSystem;
    private boolean direction = true; // true equals normal direction
    private boolean drift = true;
    private double halfSpeed = 1;       //current speed reduction coefficient.  1 at normal power.
    private double flyWheelDelta = .1;
    private double flyWheelPower = robot.defaultFlyPower;

    @Override
    public void init() {
        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.addData("left",  robot.leftDrivePower);
        telemetry.addData("right", robot.rightDrivePower);
        telemetry.addData("liveFly",robot.liveFlyPowerSetting);
        telemetry.addData("FlyWheel2", robot.flyWheelMotor2.getPower());
        telemetry.addData("flyWheel1", robot.flyWheelMotor1.getPower());
        telemetry.addData("flyWheelSysPower", robot.systemFlyPower);
        telemetry.addData("spin1Motor", robot.spin1Motor.getPower());
        telemetry.addData("spin2Motor", robot.spin2Motor.getPower());



        robot.leftDrivePower = 0;
        robot.rightDrivePower = 0;
        robot.innerIntakePower = 0;
        robot.outerIntakePower = 0;
        robot.systemFlyPower = robot.defaultFlyPower;
        robot.marvinPos = .5;

        //Having flywheels using PID instead just power.
        //Having flywheels using PID instead just power.
        robot.flyWheelMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.flyWheelMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        //Linear Slide Movement Configuration, Currently Sketchy
        //Drive Train Joystick Declaration
        robot.leftDrivePower  = gamepad1.left_stick_y;
        robot.rightDrivePower = gamepad1.right_stick_y;

    }

    public double bangBang(double ticksPerSecond)
    {
        return 0;
    }

    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
        robot.flyWheelMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.flyWheelMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.beaconServo.setPosition(robot.marvinPos);
        robot.flyWheelMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.flyWheelMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */


    public void getMaxSpeed()
    {
        robot.flyWheelMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.flyWheelMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.flyWheelMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.flyWheelMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.flyWheelMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.flyWheelMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double tempTime = runtime.seconds();
        tempTime+=30;
        while(runtime.seconds() < tempTime)
        {
            robot.flyWheelMotor1.setPower(1.00);
            robot.flyWheelMotor2.setPower(1.00);
        }

        robot.flyWheelMotor1.setPower(0.0);
        robot.flyWheelMotor2.setPower(0.0);

        int flyWheelPos1 = robot.flyWheelMotor1.getCurrentPosition();
        int flyWheelPos2 = robot.flyWheelMotor2.getCurrentPosition();

        flyWheelPos1 = flyWheelPos1/30;
        flyWheelPos2 = flyWheelPos2/30;

        telemetry.addData("flyWheelPos1",flyWheelPos1);
        telemetry.addData("flyWheelPos2",flyWheelPos2);

        telemetry.update();

    }

    @Override
    public void loop() {

        telemetry.addData("Status", "Initialized");
        telemetry.addData("left",  robot.leftDrivePower);
        telemetry.addData("right", robot.rightDrivePower);
        telemetry.addData("liveFly",robot.liveFlyPowerSetting);
        telemetry.addData("FlyWheel2", robot.flyWheelMotor2.getPower());
        telemetry.addData("flyWheel1", robot.flyWheelMotor1.getPower());
        telemetry.addData("flyWheelSysPower", robot.systemFlyPower);
        telemetry.addData("spin1Motor", robot.spin1Motor.getPower());
        telemetry.addData("spin2Motor", robot.spin2Motor.getPower());
        //Intake System Joystick Declaration
        robot.innerIntakePower = gamepad2.right_stick_y;
        robot.outerIntakePower = gamepad2.left_stick_y;

        robot.leftDrivePower  = gamepad1.left_stick_y;
        robot.rightDrivePower = gamepad1.right_stick_y;


        //Flywheel Conditionals, allows the variability of power/speed
         if (gamepad2.dpad_right) {
            robot.systemFlyPower = robot.defaultFlyPower;
        } else if (gamepad2.right_trigger > 0) {
            robot.flyWheelMotor1.setPower(robot.systemFlyPower);
            robot.flyWheelMotor2.setPower(robot.systemFlyPower);
        }
         else if (gamepad2.dpad_left)
         {
             robot.systemFlyPower = .4;
         }
         else if (gamepad2.dpad_down)
         {
             robot.systemFlyPower = .45;
         }
         else if (gamepad2.dpad_up)
         {
             robot.systemFlyPower = .55;
         }
         else if (robot.innerIntakePower > 0)
         {
             robot.flyWheelMotor1.setPower(-.2);
             robot.flyWheelMotor2.setPower(-.2);
         }
         else {
            robot.flyWheelMotor1.setPower(0);
            robot.flyWheelMotor2.setPower(0);
        }







        //Slowing down speed for the Capturing of Beacons
        if (gamepad2.left_bumper) {
            halfSpeed = .5;
        } else if (gamepad2.left_trigger > 0.25) {
            halfSpeed = .25;
        } else {
            halfSpeed = 1;
        }

        //Controlling of Drift - DcMotor Braking configuration
        if (gamepad1.y && drift) {
            robot.leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            drift = false;
        } else if (gamepad1.y && !drift) {
            robot.leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            robot.rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            drift = true;
        }

        //Marvin Servo Control, using 180 degree Servo on the Intake side of the Robot
        if (gamepad1.left_trigger > 0) {
            robot.marvinPos = .9;
        } else if (gamepad1.right_trigger > 0) {
            robot.marvinPos = .2;
        }

        //Normalization of Intake System values, since it is driven by joysticks
        maxIntakeSystem = Math.max(Math.abs(robot.innerIntakePower), Math.abs(robot.outerIntakePower));
        if (maxIntakeSystem > 1.0) {
            robot.innerIntakePower /= maxIntakeSystem;
            robot.outerIntakePower /= maxIntakeSystem;
        }



        //Normalization of the Drive Train Values, since it is also driven by joysticks
        maxDriveTrain = Math.max(Math.abs(robot.leftDrivePower), Math.abs(robot.rightDrivePower));

        if (maxDriveTrain > 1.0) {
            robot.leftDrivePower /= maxDriveTrain;
            robot.rightDrivePower /= maxDriveTrain;
        }

        if (gamepad1.right_bumper && direction || gamepad1.left_bumper && direction)
        {
            robot.leftDrivePower  = gamepad1.left_stick_y *-1;
            robot.rightDrivePower = gamepad1.right_stick_y *-1;
            direction = false;
        }
        else if (gamepad1.right_bumper && !direction || gamepad1.left_bumper && !direction)
        {
            robot.leftDrivePower  = gamepad1.left_stick_y;
            robot.rightDrivePower = gamepad1.right_stick_y;
            direction = true;
        }

        robot.leftMotor.setPower(robot.leftDrivePower*halfSpeed);
        robot.rightMotor.setPower(robot.rightDrivePower*halfSpeed);

        robot.spin1Motor.setPower(robot.innerIntakePower);
        robot.spin2Motor.setPower(robot.outerIntakePower);

        robot.beaconServo.setPosition(robot.marvinPos);
    telemetry.update();


    }

    @Override
    public void stop() {
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);

        robot.spin1Motor.setPower(0);
        robot.spin2Motor.setPower(0);

        robot.flyWheelMotor1.setPower(0);
        robot.flyWheelMotor2.setPower(0);
        robot.flyWheelMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.flyWheelMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

}
