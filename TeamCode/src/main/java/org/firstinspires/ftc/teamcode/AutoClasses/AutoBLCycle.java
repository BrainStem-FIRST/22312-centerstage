package org.firstinspires.ftc.teamcode.AutoClasses;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Config
@Autonomous(name="Robot: Blue-Left WITH CYCLE", group="Robot")
public class AutoBLCycle extends AutoAbstractBL {
    @Override
    public Boolean cycleAllowed() {return true;}
}
