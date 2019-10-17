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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Autonomous_Insentive", group="Linear Opmode")
//@Disabled

public class Autonomous_Insentive extends LinearOpMode {

    private Servo SL_PULL = null;
    private DcMotor BL_DRIVE = null;
    private DcMotor BR_DRIVE = null;
    private DcMotor FL_DRIVE = null;
    private DcMotor FR_DRIVE = null;
    private DcMotor R_INTAKE = null;
    private DcMotor L_INTAKE = null;
    private DigitalChannel IT = null;

    public void runOpMode() {

        SL_PULL = hardwareMap.get(Servo.class, "SL_PULL");
        BL_DRIVE = hardwareMap.get(DcMotor.class, "BL_DRIVE");
        BR_DRIVE = hardwareMap.get(DcMotor.class, "BR_DRIVE");
        FL_DRIVE = hardwareMap.get(DcMotor.class, "FL_DRIVE");
        FR_DRIVE = hardwareMap.get(DcMotor.class, "FR_DRIVE");
        R_INTAKE = hardwareMap.get(DcMotor.class, "RI");
        L_INTAKE = hardwareMap.get(DcMotor.class, "LI");
        IT = hardwareMap.get(DigitalChannel.class, "IT");

        IT.setMode(DigitalChannel.Mode.INPUT);
        R_INTAKE.setDirection(DcMotor.Direction.FORWARD);
        L_INTAKE.setDirection(DcMotor.Direction.REVERSE);
        BL_DRIVE.setDirection(DcMotor.Direction.FORWARD);
        BR_DRIVE.setDirection(DcMotor.Direction.REVERSE);
        FL_DRIVE.setDirection(DcMotor.Direction.FORWARD);
        FR_DRIVE.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        if (opModeIsActive()) {

            resetEncoders();
            servoPos("UP",0);
            encoderDrive(26,1,2500,1000,"DOWN");
            encoderDrive(-15,1,2500,0,"DOWN");
            encoderTurn(14.1,1,2500,"RIGHT");
            encoderDrive(65,1,4000,1000,"UP");
            encoderDrive(-50,1,4500,0,"UP");
            encoderTurn(14.1,1,2500,"LEFT");

            killBot(); // Stop Program!

        }

        while (opModeIsActive()) {
            //This is a backup stop loop if the robot does not stop after the end of the autonomous code.
            requestOpModeStop(); // You've Gone Too Far! Stop The Machine!
            stop(); // Redundancy, Redundancy, Redundancy...
            }
        }

        public void resetEncoders(){
            BL_DRIVE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BR_DRIVE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FL_DRIVE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FR_DRIVE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        public void killBot(){
            BL_DRIVE.setPower(0);
            BR_DRIVE.setPower(0);
            FL_DRIVE.setPower(0);
            FR_DRIVE.setPower(0);

            requestOpModeStop();
            stop();
        }

        public void encoderDrive(double Inches, double Speed, int SleepTimeA, int SleepTimeB, String ServoPos){

            double Diameter = 11.21;
            double EncoderTurns = 288;
            double DesiredPos = Inches * EncoderTurns / Diameter;

            BL_DRIVE.setTargetPosition((int) DesiredPos);
            BR_DRIVE.setTargetPosition((int) DesiredPos);
            FL_DRIVE.setTargetPosition((int) DesiredPos);
            FR_DRIVE.setTargetPosition((int) DesiredPos);

            BL_DRIVE.setPower(Speed);
            BR_DRIVE.setPower(Speed);
            FL_DRIVE.setPower(Speed);
            FR_DRIVE.setPower(Speed);

            BL_DRIVE.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BR_DRIVE.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FL_DRIVE.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FR_DRIVE.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            /*TODO
                Get servo position and return out of function when at "DOWN" pos rather than using 2 sleep times
             */

            sleep(SleepTimeA);
            servoPos(ServoPos, 0);
            sleep(SleepTimeB);

            resetEncoders();
        }
    public void encoderTurn(double Inches, double Speed, int SleepTime, String Direction){

        //12in = 90 degrees

        double Diameter = 11.21;
        double EncoderTurns = 288;
        double DesiredPos = Inches * EncoderTurns / Diameter;

        if (Direction == "RIGHT") {
            BL_DRIVE.setTargetPosition((int) DesiredPos);
            BR_DRIVE.setTargetPosition((int) -DesiredPos);
            FL_DRIVE.setTargetPosition((int) DesiredPos);
            FR_DRIVE.setTargetPosition((int) -DesiredPos);
        }
        else if (Direction == "LEFT") {
            BL_DRIVE.setTargetPosition((int) -DesiredPos);
            BR_DRIVE.setTargetPosition((int) DesiredPos);
            FL_DRIVE.setTargetPosition((int) -DesiredPos);
            FR_DRIVE.setTargetPosition((int) DesiredPos);
        }
        else {
            BL_DRIVE.setTargetPosition(0);
            BR_DRIVE.setTargetPosition(0);
            FL_DRIVE.setTargetPosition(0);
            FR_DRIVE.setTargetPosition(0);
        }

        BL_DRIVE.setPower(Speed);
        BR_DRIVE.setPower(Speed);
        FL_DRIVE.setPower(Speed);
        FR_DRIVE.setPower(Speed);

        BL_DRIVE.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR_DRIVE.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FL_DRIVE.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR_DRIVE.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        sleep(SleepTime);

        resetEncoders();
    }

        public void servoPos(String position, int SleepTime){

            sleep(SleepTime);

            if (position == "UP"){
                SL_PULL.setPosition(180);
            }
            else if (position == "DOWN"){
                SL_PULL.setPosition(0);
            }
            else {
                SL_PULL.setPosition(0);
            }
        }
        public void intake(double speed){
            R_INTAKE.setPower(speed);
            L_INTAKE.setPower(speed);
        }

    }



