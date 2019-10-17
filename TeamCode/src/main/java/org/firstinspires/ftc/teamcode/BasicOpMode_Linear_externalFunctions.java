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
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name="ExternalFunc", group="Linear Opmode")
//@Disabled

public class BasicOpMode_Linear_externalFunctions extends LinearOpMode {

    private DcMotor BL_DRIVE = null;
    private DcMotor BR_DRIVE = null;
    private DcMotor FL_DRIVE = null;
    private DcMotor FR_DRIVE = null;
    private DcMotor R_INTAKE = null;
    private DcMotor L_INTAKE = null;
    private DigitalChannel IT = null;

    public void runOpMode() {

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

        externalFunctions func = new externalFunctions();
        func.configure();

        waitForStart();

        while (opModeIsActive()) {

            func.drive(-gamepad1.left_stick_y, -gamepad1.right_stick_y);
            func.intakeCalc(gamepad1.right_bumper, gamepad1.left_bumper);
        }
    }
}
