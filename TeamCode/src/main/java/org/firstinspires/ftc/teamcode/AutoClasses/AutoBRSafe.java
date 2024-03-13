package org.firstinspires.ftc.teamcode.AutoClasses;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Config
@Autonomous(name="Robot: Blue-Right SAFE (no cycle)", group="Robot")
public class AutoBRSafe extends AutoAbstractBR {
    @Override
    public Boolean cycleAllowed() {return false;}
}
