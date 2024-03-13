package org.firstinspires.ftc.teamcode.AutoClasses;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Config
@Autonomous(name="Robot: Red-Right SAFE (no cycle)", group="Robot")
public class AutoRRSafe extends AutoAbstractRR {
    @Override
    public Boolean cycleAllowed() {return false;}
}
