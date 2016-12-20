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

import com.qualcomm.ftccommon.Device;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwareTank;




@TeleOp(name="Tank: Teleop", group="Tank")
public class TankTeleop extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareTank robot           = new HardwareTank();   // Use a Pushbot's hardware
                                                               // could also use HardwarePushbotMatrix class.
                     // sets rate to move servo
    private double winchDelta = .1;
    @Override
    public void runOpMode() {
        double max;
        double max2;
        boolean direction = true; // true equals normal direction
        boolean drift = true;
        //current position for the beacon servo
        double halfSpeed = 1;       //current speed reduction coefficient.  1 at normal power.

        robot.leftDrivePower = 0;
        robot.rightDrivePower = 0;
        robot.innerIntakePower = 0;
        robot.outerIntakePower = 0;
        robot.systemFlyPower = robot.defaultFlyPower;
        robot.marvinPos = .5;


        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        robot.leftDrivePower  = gamepad1.left_stick_y;
        robot.rightDrivePower = gamepad1.right_stick_y;


        robot.flyWheelMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.flyWheelMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            robot.innerIntakePower = gamepad2.right_stick_y;
            robot.outerIntakePower = gamepad2.left_stick_y;

            robot.leftDrivePower  = gamepad1.left_stick_y;
            robot.rightDrivePower = gamepad1.right_stick_y;


            if(gamepad2.y)
            {
                if(robot.linearSlide.getCurrentPosition() < robot.maxSlideHeight)
                {
                    robot.linearSlidePower = 0.15;
                }
            }
            else if(gamepad2.a)
            {
                if(robot.linearSlide.getCurrentPosition() > 0 )
                {
                    robot.linearSlidePower = -0.15;
                }
            }

            /*
            //this part is kinda sketch
            else if(gamepad2.x)
            {
                robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.linearSlide.setTargetPosition(0);
            }
            if(robot.linearSlide.getMode() = "RUN_TO_POSITION")
            {
                if(!robot.linearSlide.isBusy())
                {
                    robot.linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
            }
            //end sketchiness
            */



            if(gamepad2.dpad_down)
            {
                robot.liveFlyPowerSetting -= .05;
            }
            else if(gamepad2.dpad_up)
            {
                robot.liveFlyPowerSetting += 0.01;
            }
            else if(gamepad2.dpad_right)
            {
                robot.liveFlyPowerSetting = robot.defaultFlyPower;
            }
            else if(gamepad2.dpad_right&&gamepad2.dpad_up)
            {
                robot.liveFlyPowerSetting = robot.defaultFlyPower+0.2;
            }
            else if(gamepad2.dpad_right&&gamepad2.dpad_down)
            {
                robot.liveFlyPowerSetting = robot.defaultFlyPower-0.2;
            }


            //defining slower speeds for the triggering of beacons
            if (gamepad2.left_bumper)
            {
                halfSpeed = .5;
            }

            else if (gamepad2.left_trigger > 0.25)
            {
                halfSpeed = .25;
            }
            else

            {
                halfSpeed= 1;
            }


            //drift control
            if (gamepad1.y && drift)
            {
                robot.leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                drift = false;
            }
            else if (gamepad1.y && !drift)
            {
                robot.leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                robot.rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                drift = true;
            }

            //flywheel options
            if (gamepad2.right_trigger > 0)
            {
                robot.flyWheelMotor1.setPower(robot.systemFlyPower);
                robot.flyWheelMotor2.setPower(robot.systemFlyPower);
            }
            else
            {
                robot.flyWheelMotor1.setPower(0);
                robot.flyWheelMotor2.setPower(0);
            }


            //marvin servo
            if (gamepad1.left_trigger > 0)
            {
                robot.marvinPos = .2;
            }
            else if (gamepad1.right_trigger > 0)
            {
                robot.marvinPos =.9;
            }

            // Normalize the values so neither exceed +/- 1.0
           // max = Math.max(Math.abs(left), Math.abs(right));
            max2 = Math.max(Math.abs(robot.innerIntakePower), Math.abs(robot.outerIntakePower));
            if (max2 > 1.0)
            {
                robot.innerIntakePower /=max2;
                robot.outerIntakePower /= max2;
            }


            max = Math.max(Math.abs(robot.leftDrivePower), Math.abs(robot.rightDrivePower));
            if (max > 1.0)
            {
                robot.innerIntakePower /= max;
                robot.outerIntakePower /= max;

            }
            robot.leftMotor.setPower(robot.leftDrivePower*halfSpeed);
            robot.rightMotor.setPower(robot.rightDrivePower*halfSpeed);

            robot.spin1Motor.setPower(robot.innerIntakePower);
            robot.spin2Motor.setPower(robot.outerIntakePower);

            robot.beaconServo.setPosition(robot.marvinPos);

            // Send telemetry message to signify robot running;
            telemetry.addData("left",  "%.2f", robot.leftDrivePower);
            telemetry.addData("right", "%.2f", robot.rightDrivePower);
            telemetry.addData("beaconServo" , "", robot.beaconServo.getPosition());
            telemetry.addData("liveFly","",robot.liveFlyPowerSetting);
            telemetry.addData("FlyWheel2",  "power", robot.flyWheelMotor2.getPower());
            telemetry.addData("flyWheel1", "power", robot.flyWheelMotor1.getPower());
            telemetry.addData("spin1Motor", "power", robot.spin1Motor.getPower());
            telemetry.addData("spin2Motor", "power", robot.spin2Motor.getPower());
            telemetry.update();

            // Pause for metronome tick.  40 mS each cycle = update 25 times a second.
            robot.waitForTick(40);
        }
    }
}
