package org.firstinspires.ftc.teamcode.AutoClasses;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.AbsoluteAnalogEncoder;

import java.util.Map;

public class ArmA {

    public ServoImplEx armServo;

    private final double arm_deposit_position = 1.0;
    private final double arm_idle_position = 0.0;

    private Telemetry telemetry;

    public AbsoluteAnalogEncoder encoder;
    private final double leftServoLowerPWMLimit = 930;
    private final double  leftServoHigherPWMLimit= 2520;

    private final double rightServoPWMHigherLimit = 2300;
    private final double rightServoPWMLowerLimit = 535;

    private final double rightDepositPosition = 0;
    private final double leftDepositPosition = 1.0;
    private final double rightIdlePosition = 1.0;
    private final double leftIdlePosition = 0.0;
    private final double encoderOffset = 0;

    public ArmA(HardwareMap hwMap, Telemetry telemetry){
        this.telemetry = telemetry;

        armServo = hwMap.get(ServoImplEx.class, "armServo");

        armServo.setPwmRange(new PwmControl.PwmRange(1080,2050));

    }
    public void armToDepositPosition(){
        telemetry.addData("Arm Position Called", "Deposit");
        armServo.setPosition(0.04);
    }
    public void armToIdlePosition(){
        telemetry.addData("Arm Position Called", "Idle");
        armServo.setPosition(0.97);
    }
}
