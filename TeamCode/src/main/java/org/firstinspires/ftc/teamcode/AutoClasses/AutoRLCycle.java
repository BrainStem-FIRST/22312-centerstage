package org.firstinspires.ftc.teamcode.AutoClasses;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Config
@Autonomous(name="Robot: Red-Left WITH CYCLE", group="Robot")
public class AutoRLCycle extends AutoAbstractRL {
    @Override
    public Boolean cycleAllowed() {return true;}
}
