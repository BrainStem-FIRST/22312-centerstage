package org.firstinspires.ftc.teamcode.AutoClasses;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Map;

public class HopperA {
    private Telemetry telemetry;
    private NormalizedColorSensor colorSensor1;
    private NormalizedColorSensor colorSensor2;

    ElapsedTime waitTime = new ElapsedTime();

    public HopperA(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        colorSensor1 = hwMap.get(NormalizedColorSensor.class, "bottomHopperColorSensor");
        colorSensor2 = hwMap.get(NormalizedColorSensor.class, "upperHopperColorSensor");

    }
}