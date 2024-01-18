package org.firstinspires.ftc.teamcode.commands;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Rotation2d;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.Trabant;
import org.firstinspires.ftc.teamcode.subsystems.DarkDrive;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;

/**
 * Uses RoadRunner to project a new heading, build an action, then run within the execute function.
 */
public class RotateByDegree extends CommandBase {
    private final MecanumDrive drive;
    protected final Trabant robot;
    protected final double timeout;
    /**
     * Our RoadRunner element
     */
    protected Action action;
    protected Timing.Timer timer;
    protected double deltaAng;
    protected boolean finished = false;


    public RotateByDegree(Trabant robot, double deltaAng, double timeout) {
        this.robot = robot;
        this.drive = robot.drive;
        this.deltaAng = deltaAng;
        this.timeout = timeout;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        timer.start(); // timeout protection
        Rotation2d current = drive.pose.heading;
        double newAngleRadians = current.toDouble() + Math.toRadians(deltaAng); // set our target
        Rotation2d newHeading = Rotation2d.fromDouble(newAngleRadians); // Create new Rotation2d
        action = drive.actionBuilder(drive.pose)
            .turnTo(newHeading)
            .build();
    }

    @Override
    public void execute() {
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("x", drive.pose.position.x);
        packet.put("y", drive.pose.position.y);
        packet.put("heading", drive.pose.heading.toDouble());

        // Use the telemetryPacket with the action's run method:
        finished = !action.run(packet);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    @Override
    public boolean isFinished() {
        return finished || timer.elapsedTime() > timeout;
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }

}
