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
public class SensorTestOpMode extends LinearOpMode {

    public void runOpMode() {

        BrainSTEMRobotA robot = new BrainSTEMRobotA(hardwareMap, telemetry);

        waitForStart();

        while (opModeIsActive()) {
        /*
        /********************************************
        //               HUSKYLENS                 //
        ********************************************

        HuskyLens.Block[] blocks;   // recognized objects will be added to this array
        int targetAprilTagNum = readPropPosition(robot);

        // Change recognition mode to AprilTags before the While Loop
        robot.huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
        sleep(100);

        telemetry.addData("target tag: ", targetAprilTagNum);
        telemetry.update();

        //////////////////////////////////////////////////////////
        //           FINAL APPROACH USING SENSORS               //
        //////////////////////////////////////////////////////////
        int targetBlockPos = -1; // The block of interest within the blocks array.

        blocks = robot.huskyLens.blocks();
        telemetry.addData("Block count", blocks.length);

        for (int i = 0; i < blocks.length; i++) {
            telemetry.addData("Block", blocks[i].toString());

            if (blocks[i].id == targetAprilTagNum) {
                targetBlockPos = i;
                telemetry.addData("block seen (target): ", blocks[i].id);
            }
        }

        telemetry.addData("block of interest is in slot", targetBlockPos);

         */


            /********************************************
             //              COLOR SENSORS              //
             ********************************************/

            // read current color values
            NormalizedRGBA currentColor = robot.colorSensor.getNormalizedColors();
            telemetry.addData("gain", robot.colorSensor.getGain());
//        telemetry.addData("alliance", alliance());

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

            //TODO: add saturation and hue telemetry


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

/*
    int readPropPosition(BrainSTEMRobotA robot) {
        HuskyLens.Block[] blocks;   // recognized objects will be added to this array
        int targetTagNum = -1;

        // find prop and target tag before START
        robot.huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);

        while (!isStarted() && !isStopRequested()) {

            // Read the scene
            blocks = robot.huskyLens.blocks();
            telemetry.addData("amount of blocks", blocks.length);

            if (blocks.length != 0) {
                if (alliance() == AutoAbstractOpMode.Alliance.RED) {
                    for (int i = 0; i < blocks.length; i++) {
                        telemetry.addData("Block", blocks[i].toString());

                        if (blocks[i].id == 1) {    // Look only for Red
                            targetTagNum = getTargetTag(blocks[i]);
                            telemetry.addData("Found target prop: ", targetTagNum);
                        }
                    }
                } else if (alliance() == AutoAbstractOpMode.Alliance.BLUE) {
                    for (int i = 0; i < blocks.length; i++) {
                        telemetry.addData("Block", blocks[i].toString());

                        if (blocks[i].id == 2) {    // Look only for Blue
                            targetTagNum = getTargetTag(blocks[i]);
                            telemetry.addData("Found target prop: ", targetTagNum);
                        }
                    }
                }
            } else {
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

        // return if start is given
        if(targetTagNum == -1) {
            // No prop was detected by the time of Start, return a default value
            // Default is Right
            targetTagNum = (alliance()== AutoAbstractOpMode.Alliance.BLUE) ? 3 : 6;
        }

        return targetTagNum;
    }

    int getTargetTag(HuskyLens.Block block) {

        int propPos;
        AutoAbstractOpMode.Alliance a = alliance();

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

 */
    }
}