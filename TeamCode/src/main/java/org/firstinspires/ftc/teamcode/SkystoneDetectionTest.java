package org.firstinspires.ftc.teamcode;

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
public class SkystoneDetectionTest extends LinearOpMode {
    public DcMotor BL_DRIVE = null;
    public DcMotor BR_DRIVE = null;
    public DcMotor FL_DRIVE = null;
    public DcMotor FR_DRIVE = null;
    private ElapsedTime runtime = new ElapsedTime();
    private Servo SL_PULL = null;

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
    public void runOpMode() throws InterruptedException {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = new OpenCvWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        BL_DRIVE = hardwareMap.get(DcMotor.class, "BL");
        BR_DRIVE = hardwareMap.get(DcMotor.class, "BR");
        FL_DRIVE = hardwareMap.get(DcMotor.class, "FL");
        FR_DRIVE = hardwareMap.get(DcMotor.class, "FR");


        webcam.openCameraDevice();//open camera
        webcam.setPipeline(new StageSwitchingPipeline());//different stages
        webcam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);//display on RC
        //width, height
        //width = height in this case, because camera is in portrait mode.

        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
            telemetry.addData("Values", valLeft + "   " + valMid + "   " + valRight);
            telemetry.addData("Height", rows);
            telemetry.addData("Width", cols);

            telemetry.update();
            sleep(100);
            //call movement functions
//            strafe(0.4, 200);
//            moveDistance(0.4, 700);

            /**
             * vvv Program vvv
             */

            if(valLeft == 255 && valMid == 0 && valRight == 0){
                sleep(200);
                encoderDrive(11.8,1,100,100,"DOWN");
                encoderTurn(10,1,100,"RIGHT");


            }
            if(valLeft == 0 && valMid == 255 && valRight == 0){
                sleep(200);
                encoderDrive(11,1,100,100,"UP");
                encoderTurn(10,1,100,"LEFT");


            }
            if(valLeft == 0 && valMid == 0 && valRight == 255 ){
                sleep(200);
                encoderDrive(11,1,100,100,"UP");
                encoderTurn(10,1,100,"RIGHT");

            }

        }

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
               Tensor flow can see the location of the skystone given that we create a range for the skystone.left value;
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
    public void resetEncoders() {
        BL_DRIVE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR_DRIVE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL_DRIVE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR_DRIVE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void servoPos(String position, int SleepTime){

        sleep(SleepTime);

        if (position == "UP"){
            SL_PULL.setPosition(180);
        }
        else if (position == "DOWN"){
            SL_PULL.setPosition(0);
        }
        else if (position == "MIDDLE"){
            SL_PULL.setPosition(90);
        }
        else {
            SL_PULL.setPosition(0);
        }
    }


        //detection pipeline
        static class StageSwitchingPipeline extends OpenCvPipeline {
            Mat yCbCrChan2Mat = new Mat();
            Mat thresholdMat = new Mat();
            Mat all = new Mat();
            List<MatOfPoint> contoursList = new ArrayList<>();

            enum Stage {//color difference. greyscale
                detection,//includes outlines
                THRESHOLD,//b&w
                RAW_IMAGE,//displays raw view
            }

            private Stage stageToRenderToViewport = Stage.detection;
            private Stage[] stages = Stage.values();

            @Override
            public void onViewportTapped() {
                /*
                 * Note that this method is invoked from the UI thread
                 * so whatever we do here, we must do quickly.
                 */

                int currentStageNum = stageToRenderToViewport.ordinal();

                int nextStageNum = currentStageNum + 1;

                if (nextStageNum >= stages.length) {
                    nextStageNum = 0;
                }

                stageToRenderToViewport = stages[nextStageNum];
            }

            @Override
            public Mat processFrame(Mat input) {
                contoursList.clear();
                /*
                 * This pipeline finds the contours of yellow blobs such as the Gold Mineral
                 * from the Rover Ruckus game.
                 */

                //color diff cb.
                //lower cb = more blue = skystone = white
                //higher cb = less blue = yellow stone = grey
                Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);//converts rgb to ycrcb
                Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);//takes cb difference and stores

                //b&w
                Imgproc.threshold(yCbCrChan2Mat, thresholdMat, 102, 255, Imgproc.THRESH_BINARY_INV);

                //outline/contour
                Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
                yCbCrChan2Mat.copyTo(all);//copies mat object
                //Imgproc.drawContours(all, contoursList, -1, new Scalar(255, 0, 0), 3, 8);//draws blue contours


                //get values from frame
                double[] pixMid = thresholdMat.get((int) (input.rows() * midPos[1]), (int) (input.cols() * midPos[0]));//gets value at circle
                valMid = (int) pixMid[0];

                double[] pixLeft = thresholdMat.get((int) (input.rows() * leftPos[1]), (int) (input.cols() * leftPos[0]));//gets value at circle
                valLeft = (int) pixLeft[0];

                double[] pixRight = thresholdMat.get((int) (input.rows() * rightPos[1]), (int) (input.cols() * rightPos[0]));//gets value at circle
                valRight = (int) pixRight[0];

                //create three points
                Point pointMid = new Point((int) (input.cols() * midPos[0]), (int) (input.rows() * midPos[1]));
                Point pointLeft = new Point((int) (input.cols() * leftPos[0]), (int) (input.rows() * leftPos[1]));
                Point pointRight = new Point((int) (input.cols() * rightPos[0]), (int) (input.rows() * rightPos[1]));

                //draw circles on those points
                Imgproc.circle(all, pointMid, 5, new Scalar(255, 0, 0), 1);//draws circle
                Imgproc.circle(all, pointLeft, 5, new Scalar(255, 0, 0), 1);//draws circle
                Imgproc.circle(all, pointRight, 5, new Scalar(255, 0, 0), 1);//draws circle

                //draw 3 rectangles
                Imgproc.rectangle(//1-3
                        all,
                        new Point(
                                input.cols() * (leftPos[0] - rectWidth / 2),
                                input.rows() * (leftPos[1] - rectHeight / 2)),
                        new Point(
                                input.cols() * (leftPos[0] + rectWidth / 2),
                                input.rows() * (leftPos[1] + rectHeight / 2)),
                        new Scalar(0, 255, 0), 3);
                Imgproc.rectangle(//3-5
                        all,
                        new Point(
                                input.cols() * (midPos[0] - rectWidth / 2),
                                input.rows() * (midPos[1] - rectHeight / 2)),
                        new Point(
                                input.cols() * (midPos[0] + rectWidth / 2),
                                input.rows() * (midPos[1] + rectHeight / 2)),
                        new Scalar(0, 255, 0), 3);
                Imgproc.rectangle(//5-7
                        all,
                        new Point(
                                input.cols() * (rightPos[0] - rectWidth / 2),
                                input.rows() * (rightPos[1] - rectHeight / 2)),
                        new Point(
                                input.cols() * (rightPos[0] + rectWidth / 2),
                                input.rows() * (rightPos[1] + rectHeight / 2)),
                        new Scalar(0, 255, 0), 3);

                switch (stageToRenderToViewport) {
                    case THRESHOLD: {
                        return thresholdMat;
                    }

                    case detection: {
                        return all;
                    }

                    case RAW_IMAGE: {
                        return input;
                    }

                    default: {
                        return input;
                    }

                }
            }
        }
    }