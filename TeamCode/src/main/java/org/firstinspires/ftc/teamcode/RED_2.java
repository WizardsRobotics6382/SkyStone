package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;



import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;


    /**
     * Created by maryjaneb  on 11/13/2016.
     *
     * nerverest ticks
     * 60 1680
     * 40 1120
     * 20 560
     *
     * monitor: 640 x 480
     *
     */
    @Autonomous(name= "opencvSkystoneDetector", group="Sky autonomous")
//@Disabled
    public class RED_2 extends LinearOpMode {
        public DcMotor BL_DRIVE = null;
        public DcMotor BR_DRIVE = null;
        public DcMotor FL_DRIVE = null;
        private Servo SL_PULL = null;
        public DcMotor FR_DRIVE = null;
        public DcMotor A = null;

        private ElapsedTime runtime = new ElapsedTime();


        //0 means skystone, 1 means yellow stone
        //-1 for debug, but we can keep it like this because if it works, it should change to either 0 or 255
        private static int valMid = -1;
        private static int valLeft = -1;
        private static int valRight = -1;

        private static float rectHeight = .6f / 8f;
        private static float rectWidth = 1.5f / 8f;

        private static float offsetX = 0f / 8f;//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
        private static float offsetY = 0f / 8f;//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive

        private static float[] midPos = {4f / 8f + offsetX, 4f / 8f + offsetY};//0 = col, 1 = row
        private static float[] leftPos = {2f / 8f + offsetX, 4f / 8f + offsetY};
        private static float[] rightPos = {6f / 8f + offsetX, 4f / 8f + offsetY};
        //moves all rectangles right or left by amount. units are in ratio to monitor

        private final int rows = 640;
        private final int cols = 480;

        OpenCvCamera webcam;

        @Override
        public void runOpMode() {


            BL_DRIVE = hardwareMap.get(DcMotor.class, "BL_DRIVE");
            BR_DRIVE = hardwareMap.get(DcMotor.class, "BR_DRIVE");
            FL_DRIVE = hardwareMap.get(DcMotor.class, "FL_DRIVE");
            FR_DRIVE = hardwareMap.get(DcMotor.class, "FR_DRIVE");
            SL_PULL = hardwareMap.get(Servo.class, "SL_PULL");
            A = hardwareMap.get(DcMotor.class, "A");


            waitForStart();
            runtime.reset();
            while (opModeIsActive()) {


                /**
                 * RIGHT
                 */

                if (valLeft == 255 && valMid == 255 && valRight == 0) {
                    sleep(200);
                    sleep(200);
                    encoderDrive(40, .5, 100, 100);
                    encoderTurn(16.25, 1, 100, "RIGHT");
                    encoderDrive(24, .5, 2000, 2000);
                    A.setPower(-1);
                    sleep(1000);
                    A.setPower(0);
                    encoderDrive(-20, 1, 2000, 2000);
                    encoderTurn(12, 3, 1000, "RIGHT");
                    encoderDrive(40, 6, 1000, 1000);
                    A.setPower(1);
                    sleep(1000);
                    A.setPower(0);


                    encoderDrive(-68, 6, 4000, 5000);
                    encoderTurn(12, 6, 1000, "LEFT");
                    encoderDrive(11, 4, 1000, 1000);
                    A.setPower(-.25);
                    sleep(1000);
                    A.setPower(0);
                    encoderDrive(-11, 3, 1000, 1000);
                    encoderTurn(12, 4, 1000, "RIGHT");
                    encoderDrive(60, 4, 1000, 1000);
                    encoderDrive(-8, 3, 1000, 1000);


                    killBot();


                }
                /**
                 *Left
                 */
                else if (valLeft == 0 && valMid == 255 && valRight == 255) {
                    sleep(200);
                    encoderDrive(40, .5, 100, 100);

                    sleep(2000);
                    encoderTurn(11.25, 1, 100, "LEFT");
                    encoderDrive(24, .5, 2000, 1000);
                    A.setPower(-.25);
                    sleep(1000);
                    A.setPower(0);
                    encoderDrive(-20, 1, 2000, 2000);


                    encoderTurn(18, 3, 2000, "RIGHT");
                    encoderDrive(40, 6, 2000, 1000);
                    A.setPower(.25);
                    sleep(2000);
                    A.setPower(0);


                    encoderDrive(-60, 6, 5000, 1000);
                    encoderTurn(14, 6, 1000, "LEFT");
                    encoderDrive(11, 4, 1000, 1000);
                    A.setPower(-.25);
                    sleep(1000);
                    A.setPower(0);
                    encoderDrive(-11, 3, 1000, 1000);
                    encoderTurn(18, 4, 1000, "RIGHT");
                    encoderDrive(58, 4, 1000, 1000);
                    encoderDrive(-8, 3, 1000, 1000);


                }
                /**
                 MIDDLE
                 */
                else if (valLeft == 255 && valMid == 0 && valRight == 255) {
                    sleep(2000);

                    encoderDrive(40, .5, 100, 100);

                    sleep(2000);
                    encoderTurn(5, 1, 100, "RIGHT");
                    encoderDrive(24, .5, 2000, 5000);
                    A.setPower(-.25);
                    sleep(1000);
                    A.setPower(0);
                    encoderDrive(-20, 1, 2000, 5000);
                    encoderTurn(35, .55, 1000, "RIGHT");
                    encoderDrive(40, 6, 100, 5000);
                    A.setPower(.25);
                    sleep(2000);
                    A.setPower(0);
                    encoderDrive(-84, 1, 1000, 1000);
                    encoderTurn(35, .5, 1000, "RIGHT");
                    killBot();

                    ;


                }

            }

        }

        public void encoderDrive(double Inches, double Speed, int SleepTimeA, int SleepTimeB) {

            double Diameter = 11.21;
            double EncoderTurns = 288;
            double DesiredPos = Inches * EncoderTurns / Diameter;

            BL_DRIVE.setTargetPosition((int) DesiredPos);
            BR_DRIVE.setTargetPosition((int) -DesiredPos);
            FL_DRIVE.setTargetPosition((int) DesiredPos);
            FR_DRIVE.setTargetPosition((int) -DesiredPos);

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
            //servoPos(ServoPos, 0);


            sleep(SleepTimeB);

            resetEncoders();
        }

        public void encoderTurn(double Inches, double Speed, int SleepTime, String Direction) {


            //12in = 90 degrees

            double Diameter = 11.21;
            double EncoderTurns = 288;
            double DesiredPos = Inches * EncoderTurns / Diameter;

            if (Direction == "RIGHT") {
                BL_DRIVE.setTargetPosition((int) DesiredPos);
                BR_DRIVE.setTargetPosition((int) DesiredPos);
                FL_DRIVE.setTargetPosition((int) DesiredPos);
                FR_DRIVE.setTargetPosition((int) DesiredPos);
            } else if (Direction == "LEFT") {
                BL_DRIVE.setTargetPosition((int) -DesiredPos);
                BR_DRIVE.setTargetPosition((int) -DesiredPos);
                FL_DRIVE.setTargetPosition((int) -DesiredPos);
                FR_DRIVE.setTargetPosition((int) -DesiredPos);
            } else {
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

        public void resetEncoders() {
            BL_DRIVE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BR_DRIVE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FL_DRIVE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FR_DRIVE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        void killBot() {
            BL_DRIVE.setPower(0);
            BR_DRIVE.setPower(0);
            FL_DRIVE.setPower(0);
            FR_DRIVE.setPower(0);
            webcam.closeCameraDevice();
        }

        public void servoPos(String position, int SleepTime) {

            sleep(SleepTime);

            if (position == "UP") {
                A.setPower(-1);
                sleep(500);
                A.setPower(0);
            } else if (position == "DOWN") {
                A.setPower(.25);
                sleep(1000);
                A.setPower(0);
            } else if (position == "MIDDLE") {
                A.setPower(1);
                sleep(1000);
                A.setPower(1);
            } else {
                A.setPower(0);
            }
        }
    }











