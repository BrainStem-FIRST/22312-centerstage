package org.firstinspires.ftc.teamcode.AutoClasses;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Config
@Autonomous(name="Robot: Red-Left SAFE (no cycle)", group="Robot")
public class AutoRLSafe extends AutoAbstractRL {
    @Override
    public Boolean cycleAllowed() {return false;}
}
