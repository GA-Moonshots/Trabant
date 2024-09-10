package org.firstinspires.ftc.teamcode.util.testmodes;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.RoadRunnerDrive;
import org.firstinspires.ftc.teamcode.util.sensors.ThreeDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.util.OdometryConfig;


public final class ManualFeedbackTuner extends LinearOpMode {
    public static double DISTANCE = 64;

    @Override
    public void runOpMode() throws InterruptedException {
        if (TuningOpModes.DRIVE_CLASS.equals(RoadRunnerDrive.class)) {
            RoadRunnerDrive drive = new RoadRunnerDrive(hardwareMap, new Pose2d(0, 0, 0));
            
            if (drive.localizer instanceof ThreeDeadWheelLocalizer) {
                if (OdometryConfig.perpXTicks == 0 && OdometryConfig.par0YTicks == 0 && OdometryConfig.par1YTicks == 1) {
                    throw new RuntimeException("Odometry wheel locations not set! Run AngularRampLogger to tune them.");
                }
            }
            waitForStart();

            while (opModeIsActive()) {
                Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(0, 0, 0))
                            .lineToX(DISTANCE)
                            .lineToX(0)
                            .build());
            }
        } else {
            throw new RuntimeException();
        }
    }
}
