package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.HashMap;
import java.util.Map;

@TeleOp(name="Robot: TeleTesting", group="Robot")
public class TeleTEsting extends LinearOpMode {
    @Override
    public void runOpMode() {
        ElapsedTime initTimer = new ElapsedTime();
        Constants constants = new Constants();
        Map<String, String> stateMap = new HashMap<String, String>() {{ }};
        waitForStart();

//        robot.lift.raiseHeightTo(800);
//        sleep(500);
//        robot.arm.armToIdlePosition();
//        sleep(200);
//        robot.lift.raiseHeightTo(0);
//        isReset = true;

        while(!isStopRequested()) {
        }
        }

}
