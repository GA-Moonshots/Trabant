package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.autnomous.RoadRunnerDrive;

/**
 * We extend RoadRunner's mecanum drive. That file needs our motor instantiations
 */
public class MecanumDrive extends RoadRunnerDrive {

    public MecanumDrive(HardwareMap hardwareMap, Pose2d pose) {
        super(hardwareMap, pose);
    }
}


