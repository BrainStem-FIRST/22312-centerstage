package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

public abstract class Vision extends LinearOpMode {
    public Vision(HardwareMap hardwareMap) {
        HuskyLens huskyLens;
    }

    public Action detectAprilTag() {
        return new Action() {
            while(opModeIsActive()){

                final int READ_PERIOD = 1;
                HuskyLens huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
//
//                HuskyLens.Block[] blocks = huskyLens.blocks();
//                telemetry.addData("Block count", blocks.length);
//                for (int i = 0; i < blocks.length; i++) {
//                    telemetry.addData("Block", blocks[i].toString());
//                }
//
//                telemetry.update();
                public void runOpMode(){
                    huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");

                    Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);

                    rateLimit.expire();

                    if (!huskyLens.knock()) {
                        telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
                    } else {
                        telemetry.addData(">>", "Press start to continue");
                    }
                    huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

                    telemetry.update();
                    waitForStart();

                    /*
                     * Looking for AprilTags per the call to selectAlgorithm() above.  A handy grid
                     * for testing may be found at https://wiki.dfrobot.com/HUSKYLENS_V1.0_SKU_SEN0305_SEN0336#target_20.
                     *
                     * Note again that the device only recognizes the 36h11 family of tags out of the box.
                     */
                    while (opModeIsActive()) {
                        if (!rateLimit.hasExpired()) {
                            continue;
                        }
                        rateLimit.reset();

                        HuskyLens.Block[] blocks = huskyLens.blocks();
                        telemetry.addData("Block count", blocks.length);
                        for (int i = 0; i < blocks.length; i++) {
                            telemetry.addData("Block", blocks[i].toString());
                        }

                        telemetry.update();
                    }
                }
            }
        };
    }
}