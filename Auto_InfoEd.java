package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="Auto_InfoEd", group="CNUPlus")
public class Auto_InfoEd extends LinearOpMode {

    HardwareInfoEd robot = new HardwareInfoEd();
    private ElapsedTime runtime = new ElapsedTime();
    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;
    static final double     LIFT_GEAR_REDUCTION     = 0.5;
    static final double     LEAD_SIZE               = 0.8;
    static final double     LIFT_COUNTS_PER_CM      = (COUNTS_PER_MOTOR_REV * LIFT_GEAR_REDUCTION)/ LEAD_SIZE ;
    static final double     WHEEL_DIAMETER_CM       = 12.7 ;
    static final double     COUNTS_PER_CM           = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_CM * Math.PI);
    static final double     HEADING_THRESHOLD       = 1 ;
    static final double     P_TURN_COEFF            = 0.1;
    static final double     P_DRIVE_COEFF           = 0.025;
    float[] hsvValues = {0F, 0F, 0F};

    final double SCALE_FACTOR = 255;


    boolean bLedOn = true;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        telemetry.addData(">", "Calibrating Gyro");
        telemetry.update();

        robot.gyro.calibrate();

        while (!isStopRequested() && robot.gyro.isCalibrating())  {
            sleep(50);
            idle();
        }

        telemetry.addData(">", "Robot Ready.");
        telemetry.update();


        robot.sensorColor.enableLed(bLedOn);
        Color.RGBToHSV((int) (robot.sensorColor.red() * SCALE_FACTOR),
                (int) (robot.sensorColor.green() * SCALE_FACTOR),
                (int) (robot.sensorColor.blue() * SCALE_FACTOR),
                hsvValues);

        //Color.RGBToHSV(robot.sensorColor.red() * SCALE_FACTOR, robot.sensorColor.green() * SCALE_FACTOR, robot.sensorColor.blue() * SCALE_FACTOR, hsvValues);

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
            loop();
            drive(500,0,0.7);
            drill();
        }
    }
    public void drive (double distance, double angle, double speed){

        int     newLeftTarget;
        int     newRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        if (opModeIsActive()){

            moveCounts =(int)(COUNTS_PER_CM * distance);

            newLeftTarget  = robot.motorL.getCurrentPosition() + moveCounts;
            newRightTarget = robot.motorR.getCurrentPosition() + moveCounts;

            robot.motorL.setTargetPosition(newLeftTarget);
            robot.motorR.setTargetPosition(newRightTarget);

            robot.motorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.motorL.setPower(speed);
            robot.motorR.setPower(speed);

            while (opModeIsActive() &&
                    (robot.motorL.isBusy() && robot.motorR.isBusy())) {

                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                robot.motorL.setPower(leftSpeed);
                robot.motorR.setPower(rightSpeed);

                if (robot.sensorDistance.getDistance(DistanceUnit.CM) < 60) {

                    robot.motorL.setPower(0);
                    robot.motorR.setPower(0);

                    robot.motorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.motorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                    gyroTurn(1, 90);

                }

                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
                telemetry.addData("Actual",  "%7d:%7d",      robot.motorL.getCurrentPosition(),
                        robot.motorR.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                telemetry.update();
            }


        }
    }

    public void gyroTurn (double speed, double angle) {

        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            telemetry.update();
        }
    }

    public void gyroHold( double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }

        robot.motorL.setPower(0);
        robot.motorR.setPower(0);
    }

    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double   leftSpeed;
        double   rightSpeed;

        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        robot.motorL.setPower(leftSpeed);
        robot.motorR.setPower(rightSpeed);

        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    public double getError(double targetAngle) {

        double robotError;

        robotError = targetAngle - robot.gyro.getIntegratedZValue();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    public void drill (){
        sleep (100);
        robot.motorLift.setTargetPosition((int)(robot.motorLift.getCurrentPosition() - (LIFT_COUNTS_PER_CM * 35)));

        robot.motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.motorLift.setPower(-0.7);
        robot.drill.    setPower(1);

        while (robot.motorLift.isBusy()){
            telemetry.addData("Say","Drilling...");
            telemetry.update();
        }

        robot.motorLift.setPower(0);
        robot.drill.    setPower(0);

        robot.motorLift.setMode          (DcMotor.RunMode.RUN_USING_ENCODER);

        robot.motorLift.setTargetPosition((int)(robot.motorLift.getCurrentPosition() + (LIFT_COUNTS_PER_CM * 35)));

        robot.motorLift.setMode          (DcMotor.RunMode.RUN_TO_POSITION);

        robot.motorLift.setPower(0.7);
        robot.drill.    setPower(-1);

        while (robot.motorLift.isBusy()){
            telemetry.addData("Say","Resetting...");
            telemetry.update();
        }

        robot.motorLift.setPower(0);
        robot.drill.    setPower(0);

        robot.motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
