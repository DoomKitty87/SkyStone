/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This file is an edited version of the sample OpMode, designed to run a four-wheel swerve drive design.
 */

@TeleOp(name="Basic: Iterative OpMode", group="Iterative Opmode")
//Disabled
public class OpMode_Swerve_Experimental extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    // The DcMotor "one"s are in front, twos in back (mainly just used for sliding).
    private DcMotor leftDriveone = null;
    private DcMotor rightDriveone = null;
    private DcMotor leftDrivetwo = null;
    private DcMotor rightDrivetwo = null;
    Servo servoleftOne;
    double servoleftOnePos = 0.5;
    Servo servoleftTwo;
    double servoleftTwoPos = 0.5;
    Servo servorightOne;
    double servorightOnePos = 0.5;
    Servo servorightTwo;
    double servorightTwoPos = 0.5;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDriveone  = hardwareMap.get(DcMotor.class, "left_drive_one");
        rightDriveone = hardwareMap.get(DcMotor.class, "right_drive_one");
        leftDrivetwo = hardwareMap.get(DcMotor.class, "left_drive_two");
        rightDrivetwo = hardwareMap.get(DcMotor.class, "right_drive_two");
        servoleftOne = hardwareMap.servo.get("servoleftOne");
       servoleftTwo = hardwareMap.servo.get("servoleftTwo");
        servorightOne = hardwareMap.servo.get("servorightOne");
        servorightTwo = hardwareMap.servo.get("servorightTwo");
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDriveone.setDirection(DcMotor.Direction.FORWARD);
        rightDriveone.setDirection(DcMotor.Direction.REVERSE);
        leftDrivetwo.setDirection(DcMotor.Direction.FORWARD);
        rightDrivetwo.setDirection(DcMotor.Direction.REVERSE);
        // As stated with motors, servo one is in front and two is in back, from bird's eye view.
        servoleftOne.setPosition(servoleftOnePos);
       servoleftTwo.setPosition(servoleftTwoPos);
        servorightOne.setPosition(servorightOnePos);
        servorightTwo.setPosition(servorightTwoPos);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    public double map(double map_input) {
        double input_start = -1.0;
        double input_end = 1.0;
        double output_start = 0.0;
        double output_end = 1.0;
        double input_range = input_end - input_start;
        double output_range = output_end - output_start;
        double input = map_input;
        double x = input - input_start;
        double y = (output_range / input_range) * x;
        double output = output_start + y;
        return output;
    }
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPowerone;
        double rightPowerone;
        double leftPowertwo;
        double rightPowertwo;

        //This is experimental code for swerve drive.
        //Basic swerve drive could use a system of simply if (turnx >= 0) then leftDriveOne.setDirection(DcMotor.setDirection.FORWARD);,
        //and opposite with negative.
        //Advanced (which would be easier to pilot) requires a combination of x and y to approximate where joystick is, then
        //move pivots to that direction.
        double drivey = -gamepad1.left_stick_y;
        double drivex = gamepad1.left_stick_x;
        double turnx  =  gamepad1.right_stick_x;
        double turny  = -gamepad1.right_stick_y;

        //servoleftOnePos = Range.clip(turnx +  .5, 0, 1.0);
        //IMPORTANT: Need to figure out math to derive for motors of swerving, power level and direction.
        //If possible, still try to use motors over servos, but likely not.
        /*
        *For reference: https://www.youtube.com/watch?v=idizm--Qwlc
        * Seems like a good design
        *Speculation: (if you have ideas write here)
        *Left stick controls movement, right is turning? might have to rename variables, but more intuitive to drive.
        * I found 360 degree servos with angular feedback, so they can be used.
         */
        // This is some math to map the controller input to the servos.
        // Updated to use subroutine to map, just call map(map_input).
        // change input to something else based on joysticks. this is a crude form of map, but
        //I'm not skilled enough to do something better. Mainly I just named things stupidly.
        //Added math to convert to degrees and whatnot
        double servopos = 0;
        double leftonedrivespeed = 0;
        double rightonedrivespeed = 0;
        double lefttwodrivespeed = 0;
        double righttwodrivespeed = 0;
        // drivemodes: 1 = l stick directs robot, speed is controlled by displacement from center (joystick). Right stick turns.
        //2 = 1 stick directs robot, speed is controlled by y on right stick. Right stick turns.
        //3 =
        int drivemode = 1;
        if (gamepad1.a) {
             drivemode = 1;
        }
        if (gamepad1.x) {
            drivemode = 2;
        }
        if (gamepad1.y) {
            drivemode = 3;
        }
        if (gamepad1.b) {
            drivemode = 4;
        }
        if (drivemode == 1) {
            servopos = Math.toDegrees(Math.atan2(drivex, drivey)) / 360;
            leftonedrivespeed = (Math.abs(gamepad1.left_stick_x) + Math.abs(-gamepad1.left_stick_y)) / 2;
            lefttwodrivespeed = (Math.abs(gamepad1.left_stick_x) + Math.abs(-gamepad1.left_stick_y)) / 2;
            righttwodrivespeed = (Math.abs(gamepad1.left_stick_x) + Math.abs(-gamepad1.left_stick_y)) / 2;
            rightonedrivespeed = (Math.abs(gamepad1.left_stick_x) + Math.abs(-gamepad1.left_stick_y)) / 2;
            //drivespeed should be equal to displacement of joystick from center. I'm not sure if this will work. It might. We will have to test with hardware to find out.
            turnx = gamepad1.left_stick_x;
        }
        if (drivemode == 2) {
            servoleftTwoPos = Math.toDegrees(Math.atan2(drivex, drivey)) / 360;
            servorightTwoPos = Math.toDegrees(Math.atan2(drivex, drivey)) / 360;
            servorightOnePos = Math.toDegrees(Math.atan2(drivex, drivey)) / 360;
            servoleftOnePos = Math.toDegrees(Math.atan2(drivex, drivey)) / 360;
            leftonedrivespeed = -gamepad1.right_stick_y;
            lefttwodrivespeed = -gamepad1.right_stick_y;
            rightonedrivespeed = -gamepad1.right_stick_y;
            righttwodrivespeed = -gamepad1.right_stick_y;
            turnx = gamepad1.left_stick_y;
        }
        //I think we should discuss what the following two should be, based on what we want in terms of control.
        if (drivemode == 3) {
            servoleftOnePos = Math.toDegrees(Math.atan2(drivex, drivey)) / 360;
            servorightOnePos = Math.toDegrees(Math.atan2(drivex, drivey)) / 360;
            servoleftTwoPos = Math.toDegrees(Math.atan2(turnx, turny)) / 360;
            servorightTwoPos = Math.toDegrees(Math.atan2(turnx, turny)) / 360;
            leftonedrivespeed = (Math.abs(gamepad1.left_stick_x) + (Math.abs(-gamepad1.left_stick_y))) / 2;
            lefttwodrivespeed = (Math.abs(gamepad1.right_stick_x) + (Math.abs(-gamepad1.right_stick_y))) / 2;
            rightonedrivespeed = (Math.abs(gamepad1.left_stick_x) + (Math.abs(-gamepad1.left_stick_y))) / 2;
            righttwodrivespeed = (Math.abs(gamepad1.right_stick_x) + (Math.abs(-gamepad1.right_stick_y))) / 2;
            turnx = 0;
        }
        if (drivemode == 4) {
            
        }
        //servopos = Math.toDegrees(Math.atan2(drivex, drivey)) / 360;
        //drivespeed = -gamepad1.right_stick_y;
        // Change numbertomap to whatever it needs to be for correct angular orientation.
        //These following lines may need to change, based on driving modes.
        //servoleftOnePos = servopos;
        //servoleftTwoPos = servopos;
        //servorightOnePos = servopos;
        //servorightTwoPos = servopos;
        //All the power stuff is irrelevant for this drive, as there needs to be a different joystick setup.
        //Maybe. I think it will depend on driving mode.
        //Jk I was wrong with the above stuff. I just want to keep the comments for engineering notebook.
        //Turnx should be set to zero in drivemodes where it is not used.
        leftPowerone  = Range.clip(leftonedrivespeed + turnx,-1.0, 1.0) ;
        rightPowerone  = Range.clip(rightonedrivespeed - turnx, -1.0, 1.0) ;
        leftPowertwo  = Range.clip(lefttwodrivespeed + turnx, -1.0, 1.0);
        rightPowertwo  = Range.clip(righttwodrivespeed - turnx, -1.0, 1.0);



        // Send calculated power to wheels and swerve module servos
        leftDriveone.setPower(leftPowerone);
        rightDriveone.setPower(rightPowerone);
        leftDrivetwo.setPower(leftPowertwo);
        rightDrivetwo.setPower(rightPowertwo);
        servoleftOne.setPosition(servoleftOnePos);
        servoleftTwo.setPosition(servoleftTwoPos);
        servorightOne.setPosition(servorightOnePos);
        servorightTwo.setPosition(servorightTwoPos);
        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPowerone, rightPowerone, leftPowertwo, rightPowertwo);
        telemetry.addData("Servos", "left (%.2f), right (%.2f)", servoleftOnePos, servorightOnePos, servoleftTwoPos, servorightTwoPos);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
