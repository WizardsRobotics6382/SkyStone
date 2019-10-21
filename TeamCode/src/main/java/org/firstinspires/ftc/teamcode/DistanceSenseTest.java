package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="Autonomous_Insentive", group="Linear Opmode")
//@Disabled

public class DistanceSenseTest extends LinearOpMode {


    private Servo SL_PULL = null;
    private DcMotor BL_DRIVE = null;
    private DcMotor BR_DRIVE = null;
    private DcMotor FL_DRIVE = null;
    private DcMotor FR_DRIVE = null;
    private DcMotor R_INTAKE = null;
    private DcMotor L_INTAKE = null;
    private DigitalChannel IT = null;
    private DistanceSensor BK_DIST = null;
    private DistanceSensor R_DIST = null;

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

        IT.setMode(DigitalChannel.Mode.INPUT);
        R_INTAKE.setDirection(DcMotor.Direction.FORWARD);
        L_INTAKE.setDirection(DcMotor.Direction.REVERSE);
        BL_DRIVE.setDirection(DcMotor.Direction.FORWARD);
        BR_DRIVE.setDirection(DcMotor.Direction.REVERSE);
        FL_DRIVE.setDirection(DcMotor.Direction.FORWARD);
        FR_DRIVE.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        if (opModeIsActive()){

        distanceDriveBackwards(5,5);

        }
    }

    public void drive(double leftPower, double rightPower){
        BL_DRIVE.setPower(leftPower);
        BR_DRIVE.setPower(rightPower);
        FL_DRIVE.setPower(leftPower);
        FR_DRIVE.setPower(rightPower);
    }

    public void distanceDriveBackwards(double stopDist, double targetDist){

        while(BK_DIST.getDistance(DistanceUnit.INCH) < stopDist){
            if(R_DIST.getDistance(DistanceUnit.INCH) > targetDist){
                drive(0.9,1);
            }
            if(R_DIST.getDistance(DistanceUnit.INCH) < targetDist){
                drive(1,0.9);
            }
        }
    }
}
