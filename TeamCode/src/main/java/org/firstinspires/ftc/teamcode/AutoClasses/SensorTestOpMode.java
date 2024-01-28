package org.firstinspires.ftc.teamcode.AutoClasses;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
@TeleOp(name = "Sensor tests", group = "Robot")
public class SensorTestOpMode extends LinearOpMode { //TODO: abstract class? idk it was the only thing that got rid of the errors

    @Override
    public void runOpMode() {
        BrainSTEMRobotA robot = new BrainSTEMRobotA(hardwareMap, telemetry);

        /////////////////////////////////////////////
        //               HUSKYLENS                 //
        /////////////////////////////////////////////

        HuskyLens.Block[] blocks;   // recognized objects will be added to this array
        int targetAprilTagNum = readPropPosition(robot);

        // Start is handled within readPropPosition
//        waitForStart();

        // Switch to tag recognition mode
        robot.huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        while (opModeIsActive()) {
            /********************************************
            //            AprilTag RECOGNITION         //
            ********************************************/
            blocks = robot.huskyLens.blocks();
            telemetry.addData("Block count", blocks.length);

            for (int i = 0; i < blocks.length; i++) {
                telemetry.addData("Block", blocks[i].toString());
            }


            /********************************************
            //              COLOR SENSORS              //
            ********************************************/

            // read current color values
            NormalizedRGBA currentColor = robot.colorSensor.getNormalizedColors();
            telemetry.addData("gain", robot.colorSensor.getGain());

            final float[] hsvValues = new float[3];
            Color.colorToHSV(currentColor.toColor(), hsvValues);

            telemetry.addLine()
                    .addData("Red", "%.3f", currentColor.red)
                    .addData("Green", "%.3f", currentColor.green)
                    .addData("Blue", "%.3f", currentColor.blue);
            telemetry.addLine()
                    .addData("Hue", "%.3f", hsvValues[0])
                    .addData("Saturation", "%.3f", hsvValues[1])
                    .addData("Value", "%.3f", hsvValues[2]);

            telemetry.addData("Alpha", "%.3f", currentColor.alpha);


            /********************************************
             //            DISTANCE SENSORS             //
             ********************************************/

            // Distance sensors
            // Rev 2m Distance Sensor measurement range: 5 to 200cm with 1mm resolution
            DistanceSensor sensorDistanceLeft, sensorDistanceRight;
            sensorDistanceLeft = hardwareMap.get(DistanceSensor.class, "leftDistanceSensor");
            sensorDistanceRight = hardwareMap.get(DistanceSensor.class, "rightDistanceSensor");

            telemetry.addData("left sensor distance", sensorDistanceLeft.getDistance(DistanceUnit.MM));
            telemetry.addData("right sensor distance", sensorDistanceRight.getDistance(DistanceUnit.MM));

            telemetry.update();

        }
    }

    int readPropPosition(BrainSTEMRobotA robot) {
        HuskyLens.Block[] blocks;   // recognized objects will be added to this array
        int targetTagNum = -1;
        AutoAbstractOpMode.Alliance alliance = AutoAbstractOpMode.Alliance.RED;

        // find prop and target tag before START
        robot.huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);

        while (!isStarted() && !isStopRequested()) {

            // Read the scene
            blocks = robot.huskyLens.blocks();
            telemetry.addData("amount of blocks seen ", blocks.length);

            if (blocks.length != 0) {
                for (int i = 0; i < blocks.length; i++) {
                    telemetry.addData("Block", blocks[i].toString());

                    if (blocks[i].id == 1) {    // Look only for Red
                        telemetry.addLine("Red detected.");
                        alliance = AutoAbstractOpMode.Alliance.RED;
                    }
                    else if (blocks[i].id == 2) {
                        telemetry.addLine("Blue detected.");
                        alliance = AutoAbstractOpMode.Alliance.BLUE;
                    }

                    targetTagNum = getTargetTag(blocks[i], alliance);
                    telemetry.addData("Found target prop: ", targetTagNum);
                }
            }
            else {
                telemetry.addLine("Don't see the prop :(");

                if (targetTagNum == -1) {
                    telemetry.addLine("(The prop has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the prop before");
                    telemetry.addData("which was: ", targetTagNum);
                }

                sleep(20);
            }
            telemetry.update();
        } // while

//        // return if start is given
//        if(targetTagNum == -1) {
//            // No prop was detected by the time of Start, return a default value
//            // Default is Right
//            targetTagNum = (alliance()== AutoAbstractOpMode.Alliance.BLUE) ? 3 : 6;
//        }

        return targetTagNum;
    }

    int getTargetTag(HuskyLens.Block block, AutoAbstractOpMode.Alliance a) {

        int propPos;

        // for test purposes, return a known value
        // delete this segment when team prop is available
        //        return 1;
        if (block.x < 110) {
            // Prop is on left
            propPos = (a == AutoAbstractOpMode.Alliance.BLUE) ? 1 : 4;
        } else if (block.x > 210) {
            // prop is on right
            propPos = (a == AutoAbstractOpMode.Alliance.BLUE) ? 3 : 6;
        } else {
            // prop is on center
            propPos = (a == AutoAbstractOpMode.Alliance.BLUE) ? 2 : 5;
        }

        return propPos;

    }
}