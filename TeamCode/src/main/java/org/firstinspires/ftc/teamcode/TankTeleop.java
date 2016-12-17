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

/**
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a POV Game style Teleop for a PushBot
 * In this mode the left stick moves the robot FWD and back, th e Right stick turns left and right.
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Tank: Teleop", group="Tank")
public class TankTeleop extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareTank robot           = new HardwareTank();   // Use a Pushbot's hardware
                                                               // could also use HardwarePushbotMatrix class.
                     // sets rate to move servo
    private double winchDelta = .1;
    @Override
    public void runOpMode() {
        double left;
        double right;
        double spin1;
        double spin2;
        double max;
        double max2;
        double max3;
        boolean direction = true; // true equals normal direction
        boolean drift = true;
        double marvinPos = .5;
        double halfSpeed = 1;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        left  = gamepad1.left_stick_y;
        right = gamepad1.right_stick_y;
        robot.flyWheelMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.flyWheelMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            spin1 = gamepad2.left_stick_y;
            spin2 = gamepad2.right_stick_y;

            left  = gamepad1.left_stick_y;
            right = gamepad1.right_stick_y;
            //if (robot.device.getDigitalInputStateByte() == 1)

            //definining the front direction
            if (gamepad2.left_bumper)
            {
                halfSpeed = .5;
            }

            else if (gamepad2.left_trigger == 1)
            {
                halfSpeed = .25;
            }
            else
            {
                halfSpeed= 1;
            }

            if (gamepad1.y && drift)
            {
                robot.leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
            else if (gamepad1.y && !drift)
            {
                robot.leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                robot.rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }

            //flywheel options
            if (gamepad2.right_trigger > 0)
            {
                robot.flyWheelMotor1.setPower(.7);
                robot.flyWheelMotor2.setPower(.7);
            }
            else
            {
                robot.flyWheelMotor1.setPower(0);
                robot.flyWheelMotor2.setPower(0);
            }

            if (gamepad1.left_trigger > 0)
            {
                marvinPos = .2;
            }
            else if (gamepad1.right_trigger > 0)
            {
                marvinPos =.9;
            }

            // Normalize the values so neither exceed +/- 1.0
           // max = Math.max(Math.abs(left), Math.abs(right));
            max2 = Math.max(Math.abs(spin1), Math.abs(spin2));
            if (max2 > 1.0)
            {
                spin1 /=max2;
                spin2 /= max2;
            }


            max = Math.max(Math.abs(left), Math.abs(right));
            if (max > 1.0)
            {
                left /= max;
                right /= max;

            }
            robot.leftMotor.setPower(left*halfSpeed);
            robot.rightMotor.setPower(right*halfSpeed);

            robot.spin1Motor.setPower(spin1);
            robot.spin2Motor.setPower(spin2);

            robot.beaconServo.setPosition(marvinPos);

            // Send telemetry message to signify robot running;
            telemetry.addData("left",  "%.2f", left);
            telemetry.addData("right", "%.2f", right);
            telemetry.addData("beaconServo" , "", robot.beaconServo.getPosition());
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
