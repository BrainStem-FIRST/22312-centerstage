package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.nio.file.attribute.UserDefinedFileAttributeView;
import java.util.Map;

public class Hanging {

    public final String HANGING_SYSTEM_NAME = "HOPPER_SYSTEM_NAME";

    public final String SERVO_NOT_RELEASED = "SERVO_NOT_RELEASED";

    public final String SERVO_RELEASED = "SERVO_RELEASED";

    private Telemetry telemetry;

    private Map stateMap;

    private ServoImplEx hangingServo;
    public DcMotorEx hangingMotor;

    private double servoNotReleasedPosition = 0;
    private double servoReleasedPosition = 1;

    private int lowerPWMLimitServo = 591;
    private int higherPWMLimitServo = 1600;
    public Hanging(HardwareMap hwMap, Telemetry telemetry, Map stateMap){
        this.telemetry = telemetry;
        this.stateMap = stateMap;

        hangingMotor = hwMap.get(DcMotorEx.class, "hangingMotor");
        hangingServo = hwMap.get(ServoImplEx.class, "hangingServo");

        hangingServo.setPwmRange(new PwmControl.PwmRange(lowerPWMLimitServo, higherPWMLimitServo));

        hangingMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hangingMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setState(){
        String servoState = (String) stateMap.get(HANGING_SYSTEM_NAME);

        switch(servoState){
            case SERVO_NOT_RELEASED:{
                toNotReleased();
                break;
            }
            case SERVO_RELEASED:{
                toReleased();
                break;
            }
        }
    }

    private void toNotReleased(){
        hangingServo.setPosition(servoNotReleasedPosition);
    }

    private void toReleased(){
        hangingServo.setPosition(servoReleasedPosition);
    }
}
