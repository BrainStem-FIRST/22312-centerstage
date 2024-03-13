package org.firstinspires.ftc.teamcode.AutoClasses;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Config
@Autonomous(name="Robot: Blue-Left SAFE (no cycle)", group="Robot")
public class AutoBLSafe extends AutoAbstractBL {
    @Override
    public Boolean cycleAllowed() {return false;}
}
