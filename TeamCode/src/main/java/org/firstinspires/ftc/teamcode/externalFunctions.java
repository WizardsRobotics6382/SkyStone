package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public final class externalFunctions extends LinearOpMode {

    public DcMotor BL_DRIVE = hardwareMap.get(DcMotor.class, "BL_DRIVE");
    public DcMotor BR_DRIVE = hardwareMap.get(DcMotor.class, "BR_DRIVE");
    public DcMotor FL_DRIVE = hardwareMap.get(DcMotor.class, "FL_DRIVE");
    public DcMotor FR_DRIVE = hardwareMap.get(DcMotor.class, "FR_DRIVE");
    public DcMotor R_INTAKE = hardwareMap.get(DcMotor.class, "RI");
    public DcMotor L_INTAKE = hardwareMap.get(DcMotor.class, "LI");
    public DigitalChannel IT = hardwareMap.get(DigitalChannel.class, "IT");

    public void configure() {

        IT.setMode(DigitalChannel.Mode.INPUT);
        R_INTAKE.setDirection(DcMotor.Direction.FORWARD);
        L_INTAKE.setDirection(DcMotor.Direction.REVERSE);
        BL_DRIVE.setDirection(DcMotor.Direction.FORWARD);
        BR_DRIVE.setDirection(DcMotor.Direction.REVERSE);
        FL_DRIVE.setDirection(DcMotor.Direction.FORWARD);
        FR_DRIVE.setDirection(DcMotor.Direction.REVERSE);

    }
    public void drive ( float leftPower, float rightPower){
        BL_DRIVE.setPower(leftPower);
        BR_DRIVE.setPower(rightPower);
        FL_DRIVE.setPower(leftPower);
        FR_DRIVE.setPower(rightPower);
    }
    public void intakeCalc(boolean buttonIn, boolean buttonOut){
        if (buttonIn == true || (buttonOut == true)) {
            if (buttonOut){
                intake(1);
            }
            else if (buttonIn && IT.getState() == true){
                intake(-1);
            }
            else{
                intake(0);
            }
        }
        else {
            intake(0);
        }
    }
    public void intake(double speed){
        R_INTAKE.setPower(speed);
        L_INTAKE.setPower(speed);
    }

    @Override
    public void runOpMode() throws InterruptedException {

    }
}
