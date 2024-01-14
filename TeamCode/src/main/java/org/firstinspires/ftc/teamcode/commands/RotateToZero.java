package org.firstinspires.ftc.teamcode.commands;


import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Rotation2d;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.Trabant;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;

/**
 * Piggy-backs off of RotateByDegree and just calculates the change needed to face our zero angle
 */
public class RotateToZero extends RotateByDegree {

    public RotateToZero(Trabant robot, double timeout) {
        super(robot, robot.drive.getAngleDifferenceFromZero(), timeout);
    }

    /**
     * In case there's a difference in time from constructor and initialization, we update
     * the necessary change before we build the rotation
     */
    @Override
    public void initialize() {
        deltaAng = robot.drive.getAngleDifferenceFromZero();
        super.initialize();
    }

}
