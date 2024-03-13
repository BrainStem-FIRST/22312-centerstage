package org.firstinspires.ftc.teamcode.AutoClasses;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Config
@Autonomous(name="Robot: Red-Right WITH CYCLE", group="Robot")
public class AutoRRCycle extends AutoAbstractRR {
    @Override
    public Boolean cycleAllowed() {return true;}
}
