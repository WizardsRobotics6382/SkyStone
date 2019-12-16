package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="TestLift", group="Linear Opmode")
//@Disabled
public class test extends LinearOpMode {
    public DcMotor LL = null;

    public void runOpMode() {
        LL = hardwareMap.get(DcMotor.class,"LL");

        waitForStart();

        while (opModeIsActive()) {

            Lift(gamepad1.dpad_up, gamepad1.dpad_down);

        }
    }

    public void Lift(boolean Up, boolean Down){
            if(Up == true){
                LL.setPower(1);
            }
            else if(Down == true){
                LL.setPower(-0.5);
            }
            else{
                LL.setPower(0);
            }
        }
}
