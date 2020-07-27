package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;

public class HardwareInfoEd
{

    public DcMotor motorL                              = null;
    public DcMotor motorR                              = null;
    public DcMotor drill                               = null;
    public DcMotor motorLift                           = null;
    public ModernRoboticsI2cRangeSensor sensorDistance = null;
    public ColorSensor sensorColor                     = null;
    public ModernRoboticsI2cGyro gyro                  = null;

    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    public HardwareInfoEd(){

    }

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        motorL         = hwMap.get(DcMotor.class, "motorL");
        motorR         = hwMap.get(DcMotor.class, "motorR");
        drill          = hwMap.get(DcMotor.class, "drill");
        motorLift      = hwMap.get(DcMotor.class, "motorlift");
        sensorDistance = hwMap.get(ModernRoboticsI2cRangeSensor.class, "senzorDistanta");
        sensorColor    = hwMap.get(ColorSensor.class, "senzorCuloare");
        gyro           = hwMap.get(ModernRoboticsI2cGyro.class, "gyro");

        motorL.    setDirection(DcMotor.Direction.REVERSE);
        motorR.    setDirection(DcMotor.Direction.FORWARD);
        drill.     setDirection(DcMotor.Direction.REVERSE);
        motorLift. setDirection(DcMotor.Direction.FORWARD);

        motorL.   setPower(0);
        motorR.   setPower(0);
        drill.    setPower(0);
        motorLift.setPower(0);

        motorL.   setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorR.   setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drill.    setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void waitForTick(long periodMs) {

        long  remaining = periodMs - (long)period.milliseconds();

        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
        period.reset();
    }

}