package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="TeleOp_InfoEd", group="CNUPlus")
public class TeleOp_InfoEd<usbManager> extends LinearOpMode {

    HardwareInfoEd robot = new HardwareInfoEd();
    double coef       = 1;
    double leftPower  = 0;
    double rightPower = 0;
    double drive, turn;


    @Override
    public void runOpMode() {
        robot.init(hardwareMap);


        telemetry.addData("Say", "Hello Driver");
        telemetry.update();

        robot.motorL.   setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorR.   setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.drill.    setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.motorL.   setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorR.   setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.drill.    setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        waitForStart();
        while (opModeIsActive()) {

            if (gamepad1.b)
                robot.drill.setPower(1);

            else if (gamepad1.x)
                robot.drill.setPower(-1);

            else
                robot.drill.setPower(0);

            
            if (gamepad1.dpad_up && robot.motorLift.getCurrentPosition()<0)
                robot.motorLift.setPower(0.7);

            else if (gamepad1.dpad_down  &&  robot.motorLift.getCurrentPosition()>-30000)
                robot.motorLift.setPower(-0.7);

            else
                robot.motorLift.setPower(0);


            if (gamepad1.left_trigger > 0 || gamepad1.right_trigger > 0)
                coef = 1 - (gamepad1.left_trigger + gamepad1.right_trigger)/2;
            else
                coef = 1;

            drive = -gamepad1.left_stick_y * coef;
            turn  =  gamepad1.left_stick_x * coef;

            leftPower  = Range.clip (drive + turn,-1,1);
            rightPower = Range.clip (drive - turn,-1,1);

            robot.motorR.setPower (rightPower);
            robot.motorL.setPower (leftPower);


        }
    }
}
