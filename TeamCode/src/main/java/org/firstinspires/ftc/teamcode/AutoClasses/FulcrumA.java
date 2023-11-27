package org.firstinspires.ftc.teamcode.AutoClasses;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Constants;

import java.util.Map;

public class FulcrumA {
    private Telemetry telemetry;
    private ServoImplEx fulcrumServo;
    private final double down = 1.0;
    private final double up = 0;
    private final double lowerPWMLimit = 352;
    private final double upperPWMLimit = 1070;
    public ElapsedTime fulcrumCycleTime = new ElapsedTime();
;
    public FulcrumA(HardwareMap hwMap, Telemetry telemetry){
        this.telemetry = telemetry;
        fulcrumServo = hwMap.get(ServoImplEx.class, "fulcrumServo");
        fulcrumServo.setPwmRange(new PwmControl.PwmRange(lowerPWMLimit,upperPWMLimit));

    }

    private void fulcrumDown(){
        fulcrumServo.setPosition(down);
    }

    private void fulcrumUp(){
        fulcrumServo.setPosition(up);
    }
}
