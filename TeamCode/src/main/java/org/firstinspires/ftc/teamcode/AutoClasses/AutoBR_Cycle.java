package org.firstinspires.ftc.teamcode.AutoClasses;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Config
@Autonomous(name="Robot: Blue-Right WITH CYCLE", group="Robot")
public class AutoBR_Cycle extends AutoAbstractBR {
    @Override
    public Boolean cycleAllowed() {return true;}
}
