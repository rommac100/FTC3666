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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwareTank;

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

@Autonomous(name="Tank: AutotomousWaitCapBall", group="Tank")
public class TankAutoWaitCapBall extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareTank robot   = new HardwareTank();
    private ElapsedTime     runtime = new ElapsedTime();

    public int distance(double dis)
    {
        return (int)(dis*robot.ticksPerInch);
    }


    //0 = forward, 2 = reverse
    public void drive(int direction, double power, int ticks)
    {
        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.leftMotor.setTargetPosition(ticks);
        robot.rightMotor.setTargetPosition(ticks);
        switch (direction)
        {
            case 0:
                robot.leftMotor.setPower(power);
                robot.rightMotor.setPower(power);
                while(robot.leftMotor.isBusy() && robot.rightMotor.isBusy())
                {


                }
                robot.leftMotor.setPower(0);
                robot.rightMotor.setPower(0);

                robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                break;
            case 1:
                robot.leftMotor.setPower(power);
                robot.rightMotor.setPower(-1*power);
                break;
            case 2:
                robot.leftMotor.setPower(-1*power);
                robot.rightMotor.setPower(-1*power);
                double temp = runtime.seconds()+8;
                while(robot.leftMotor.isBusy() && robot.rightMotor.isBusy() && opModeIsActive() && runtime.seconds() < temp)
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
            case 3:
                robot.leftMotor.setPower(-1*power);
                robot.leftMotor.setPower(1*power);
                break;


        }
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

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way

        sleep(10000);
        //Step 1: drive forward one foam Pad
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);
        runtime.reset();
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

            //sleep(2000);
            drive(2, 1, distance(35));


            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);
            // Step 4:  Stop and close the claw


            telemetry.addData("Path", "Complete");
            telemetry.update();
            //sleep(1000);
    }
}
