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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@TeleOp(name="Basic: Linear OpMode", group="Linear Opmode")
//@Disabled

public class BasicOpMode_Linear extends LinearOpMode {

    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Count";

    private static final String VUFORIA_KEY =
            "Ade+9uD/////AAABma6ja62RQUgPiQ0Q5nG2sMcX8DLBpOO89TP2QHCUcEfMKa6XuJyS3BhaRszQY9wHoyu3VFsM7RJUs7Uqpsv892Y3XKLXkYvvquPuqXyjG41OivqL6XA/jTp+sChq9T0KuNooX2CldeaVlfKOaV/cwB8lC97iIgWilZEqj6dEUpWjg/wlEpKdrvILvoBGqIZ4q/ZN3fm/785FD24Dt5WOWaShDT3iQ2eIO72yYG7AQ9aPzAgyruWwGHUOxKG7ocGyrSPFo5iTGgp3rUZaa+YWJEmoeViGj4tNXPJ5DUPeQhQaFNnNtVh681OSxDKlLFBEblihe1Y2r4fTu/OKYntUToWCQLSfejBn2tcvSGRWvobt";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    private DistanceSensor BK_DIST = null;
    private DistanceSensor R_DIST = null;
    private Servo SL_PULL = null;
    private DcMotor BL_DRIVE = null;
    private DcMotor BR_DRIVE = null;
    private DcMotor FL_DRIVE = null;
    private DcMotor FR_DRIVE = null;
    private DcMotor R_INTAKE = null;
    private DcMotor L_INTAKE = null;
    private DigitalChannel IT = null;
    private DcMotor LL = null;
    private DcMotor A = null ;
    private DcMotor E = null ;
    private Servo Claw = null ;




    public void runOpMode() {

        BK_DIST = hardwareMap.get(DistanceSensor.class, "BK_DIST");
        R_DIST = hardwareMap.get(DistanceSensor.class, "R_DIST");
        SL_PULL = hardwareMap.get(Servo.class, "SL_PULL");
        BL_DRIVE = hardwareMap.get(DcMotor.class, "BL_DRIVE");
        BR_DRIVE = hardwareMap.get(DcMotor.class, "BR_DRIVE");
        FL_DRIVE = hardwareMap.get(DcMotor.class, "FL_DRIVE");
        FR_DRIVE = hardwareMap.get(DcMotor.class, "FR_DRIVE");
        R_INTAKE = hardwareMap.get(DcMotor.class, "RI");
        L_INTAKE = hardwareMap.get(DcMotor.class, "LI");
        IT = hardwareMap.get(DigitalChannel.class, "IT");
        LL = hardwareMap.get(DcMotor.class, "LL");
        //A  = hardwareMap.get(DcMotor.class,"A");//
        E = hardwareMap.get(DcMotor.class,"E");
        Claw = hardwareMap.get(Servo.class,"C");

        IT.setMode(DigitalChannel.Mode.INPUT);
        R_INTAKE.setDirection(DcMotor.Direction.FORWARD);
        L_INTAKE.setDirection(DcMotor.Direction.REVERSE);
        BL_DRIVE.setDirection(DcMotor.Direction.FORWARD);
        BR_DRIVE.setDirection(DcMotor.Direction.REVERSE);
        FL_DRIVE.setDirection(DcMotor.Direction.FORWARD);
        FR_DRIVE.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        while (opModeIsActive()) {

            Motor_power motorPower = new Motor_power();

            motorPower = drive(-gamepad1.left_stick_y, motorPower);
            motorPower = drive_turn(-gamepad1.right_stick_x,gamepad1.right_stick_x, motorPower);
            setMotorPower(motorPower);
            intake(gamepad1.right_bumper,gamepad1.left_bumper,IT.getState());
            servoCalc(gamepad1.y, gamepad1.a);
            servocalc2(gamepad2.b,gamepad2.x);
            Lift(gamepad1.dpad_up, gamepad2.dpad_down);
            Extension(gamepad2.left_bumper,gamepad2.right_bumper);
            telemetryUpdae();


                }
        }
        public void servocalc2 (boolean grab, boolean drop ){
        if (grab){
            servoPos2("grab");
        }
        else if (drop){
            servoPos2("drop");
            }
        }



    public void servoCalc(boolean UP, boolean DOWN) {
        if (UP) {
            servoPos("UP");
        } else if (DOWN) {
            servoPos("DOWN");
        }

    }

    public void Lift(boolean up, boolean down) {
        if (up == true) {
            LL.setPower(1);

        } else if (down == true) {
            LL.setPower(-1);

        } else {
            LL.setPower(0);

        }
    }
    public void Extension  (boolean out , boolean in) {
        if (out == true){
            E.setPower(-1);

        }
        else if (in == true && out == false){
            E.setPower(1);

        }
        else{
            E.setPower(0);


        }
    }
    public void servoPos2(String position){
    if (position == "grab"){
        Claw.setPosition(30);
    }
        else if (position == "drop"){
            Claw.setPosition(0);


    }
    }


    public void servoPos(String position) {
        if (position == "UP") {
            SL_PULL.setPosition(0);
           // A .setPower(.25);
           // sleep(2000);
            //A .setPower(0);
        } else if (position == "DOWN") {
            SL_PULL.setPosition(180);

           // A .setPower(-.3);
            //sleep(800);
            //A.setPower(0);
        };
        }


    public Motor_power drive_turn(float right ,float left, Motor_power motorPower ){
        if(right > .5) {
            right = (float) 0.5;
        }
        if(left > .5) {
            left = (float) 0.5;
        }
        if(motorPower == null) {
            motorPower = new Motor_power();
        }
        motorPower.BL += left;
        motorPower.BR += right;
        motorPower.FL += left;
        motorPower.FR += right;

        return motorPower;
    }

    public Motor_power drive(float Power, Motor_power motorPower) {
        if(Power > .5) {
            Power = (float) 1;
        }
        if(motorPower == null) {
            motorPower = new Motor_power();
        }
        motorPower.BL += Power;
        motorPower.BR += Power;
        motorPower.FL += Power;
        motorPower.FR += Power;

        return motorPower;
    }

    public void setMotorPower(Motor_power motorPower) {
        BL_DRIVE.setPower(motorPower.BL);
        BR_DRIVE.setPower(motorPower.BR);
        FL_DRIVE.setPower(motorPower.FL);
        FR_DRIVE.setPower(motorPower.FR);
    }

    /*public void intakeCalc(float in, float ) {
        R_INTAKE.setPower(in);
        L_INTAKE.setPower(out);
    }

     */




    public void intake(boolean IN,boolean OUT, boolean IT) {
        if ( IN == true ) {
            L_INTAKE.setPower(1);
            R_INTAKE.setPower(1);
        }
        else if ( OUT == true && IT == true){
            L_INTAKE.setPower(-1);
            R_INTAKE.setPower(-1);



        }
        else {
            L_INTAKE.setPower(0);
            R_INTAKE.setPower(0);
        }
    }
    public void telemetryUpdae(){
        telemetry.addData("IT", IT.getState());
        telemetry.addData("BK_DIST", BK_DIST.getDistance(DistanceUnit.INCH));
        telemetry.addData("BK_DIST", R_DIST.getDistance(DistanceUnit.INCH));
        telemetry.update();
    }

    class Motor_power {
        public float FR,FL,BR,BL = (float) 0.0;
    }
}

