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
 * OR TORT (INCLUDING NEGLIGENCnearOpMode {

    private static final String TFOD_MODEL_ASE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous(name="Autonomous_Insentive", group="Linear Opmode")
//@Disabled

public class Autonomous_Insentive extends LinearOpMode {

    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Count";

    private static final String VUFORIA_KEY =
            "Ade+9uD/////AAABma6ja62RQUgPiQ0Q5nG2sMcX8DLBpOO89TP2QHCUcEfMKa6XuJyS3BhaRszQY9wHoyu3VFsM7RJUs7Uqpsv892Y3XKLXkYvvquPuqXyjG41OivqL6XA/jTp+sChq9T0KuNooX2CldeaVlfKOaV/cwB8lC97iIgWilZEqj6dEUpWjg/wlEpKdrvILvoBGqIZ4q/ZN3fm/785FD24Dt5WOWaShDT3iQ2eIO72yYG7AQ9aPzAgyruWwGHUOxKG7ocGyrSPFo5iTGgp3rUZaa+YWJEmoeViGj4tNXPJ5DUPeQhQaFNnNtVh681OSxDKlLFBEblihe1Y2r4fTu/OKYntUToWCQLSfejBn2tcvSGRWvobt";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    private Servo SL_PULL = null;
    private DcMotor BL_DRIVE = null;
    private DcMotor BR_DRIVE = null;
    private DcMotor FL_DRIVE = null;
    private DcMotor FR_DRIVE = null;
    private DcMotor R_INTAKE = null;
    private DcMotor L_INTAKE = null;
    private DigitalChannel IT = null;

    public int Count = 0;
    public int Skystone = 0;
    public double left = 0;
    public double right = 0;

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

        Initialize();
        waitForStart();

        if (opModeIsActive()) {

            resetEncoders();
            servoPos("UP",0);
            encoderDrive(14,1,1400,100,"UP", 1);
            encoderTurn(1,1,10,"LEFT");
            vuforia();
            encoderTurn(1,1,20,"RIGHT");
            vuforia();
            encoderTurn(1,1,10,"LEFT");
            vuforia();

            telemetry.addData("Skystone", Skystone);
            telemetry.update();

            if(Skystone == 1){

                telemetry.addData("left", left);
                telemetry.update();
                sleep(5000);

                if (left < 40){
                    Left();
                }
                else if ((left > 50 && left < 275) || (left - right) > 100){
                    Middle();
                }
                else if (left > 275){
                    Right();
                }
                else {
                    Middle();
                }
            }

            sleep(5000);

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

        public void stopWheels(){
            BL_DRIVE.setPower(0);
            BR_DRIVE.setPower(0);
            FL_DRIVE.setPower(0);
            FR_DRIVE.setPower(0);
        }

        public void killBot(){
            BL_DRIVE.setPower(0);
            BR_DRIVE.setPower(0);
            FL_DRIVE.setPower(0);
            FR_DRIVE.setPower(0);

            if (tfod != null) {
                tfod.shutdown();
            }

            requestOpModeStop();
            stop();
        }

        public void Left(){

            telemetry.addData("Left", true);
            telemetry.addData("Middle", false);
            telemetry.addData("Right", false);

            telemetry.update();

            encoderDrive(11.5,1,2500,1000,"DOWN", 0);
            encoderDrive(-15,1,2500,0,"DOWN", 0);
            encoderTurn(14.1,1,2500,"RIGHT");
            encoderDrive(65,1,4000,1000,"UP", 0);
            encoderDrive(-50,1,4500,0,"UP", 0);
            encoderTurn(14.1,1,2500,"LEFT");
        }

        public void Middle(){

            telemetry.addData("Left", false);
            telemetry.addData("Middle", true);
            telemetry.addData("Right", false);

            telemetry.update();

            encoderTurn(10.1,1,2500,"RIGHT");
            encoderTurn(10.1,1,2500,"LEFT");

            encoderDrive(11.5,1,2500,1000,"DOWN", 0);
            encoderDrive(-15,1,2500,0,"DOWN", 0);
            encoderTurn(14.1,1,2500,"RIGHT");
            encoderDrive(65,1,4000,1000,"UP", 0);
            encoderDrive(-50,1,4500,0,"UP", 0);
            encoderTurn(14.1,1,2500,"LEFT");

        }

        public void Right(){

            telemetry.addData("Left", false);
            telemetry.addData("Middle", false);
            telemetry.addData("Right", true);

            telemetry.update();

            encoderDrive(11.5,1,2500,1000,"DOWN", 0);
            encoderDrive(-15,1,2500,0,"DOWN", 0);
            encoderTurn(14.1,1,2500,"RIGHT");
            encoderDrive(65,1,4000,1000,"UP", 0);
            encoderDrive(-50,1,4500,0,"UP", 0);
            encoderTurn(14.1,1,2500,"LEFT");

        }

        public void vuforia(){
            while (Count < 1 && Skystone == 0){
                tensorFlow();
            }
        }

        public void encoderDrive(double Inches, double Speed, int SleepTimeA, int SleepTimeB, String ServoPos, int Vuforia){

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
               Tensor flow can see the location of the skystone given that we create a range for the skystone.left value;
               */

            sleep(SleepTimeA);
            servoPos(ServoPos, 0);
            if (Vuforia == 1){
                tensorFlow();
            }
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
        }
        else if (Direction == "LEFT") {
            BL_DRIVE.setTargetPosition((int) -DesiredPos);
            BR_DRIVE.setTargetPosition((int) DesiredPos);
        }
        else {
            BL_DRIVE.setTargetPosition(0);
            BR_DRIVE.setTargetPosition(0);
        }

        BL_DRIVE.setPower(Speed);
        BR_DRIVE.setPower(Speed);

        BL_DRIVE.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR_DRIVE.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FL_DRIVE.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR_DRIVE.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        sleep(SleepTime);

        resetEncoders();
    }

        public void Initialize(){

            initVuforia();
            initTfod();

        if (tfod != null) {
            tfod.activate();
        }

    }

        public void tensorFlow(){

            if (tfod != null) {
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    Count = updatedRecognitions.size();

                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData("SkystoneLabel", recognition.getLabel());
                        telemetry.update();
                        if (recognition.getLabel() == "Count"){
                            Skystone = 1;
                            left = recognition.getLeft();
                            right = recognition.getRight();
                        }
                    }
                }
            }
        }

    private void initVuforia() {

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {

        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
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



