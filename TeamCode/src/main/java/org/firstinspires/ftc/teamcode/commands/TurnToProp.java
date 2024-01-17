package org.firstinspires.ftc.teamcode.commands;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Rotation2d;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.Trabant;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;

public class TurnToProp extends CommandBase {

    private final MecanumDrive drive;
    private final Trabant robot;
    protected final double timeout;
    /**
     * Our RoadRunner element
     */
    protected Action action;
    protected Timing.Timer timer;
    protected boolean finished = false;


    public TurnToProp(Trabant robot, double timeout) {
        this.robot = robot;
        this.drive = robot.drive;
        this.timeout = timeout;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        timer.start();
        Rotation2d current = drive.pose.heading;
        Rotation2d newHeading;

        if(robot.targetApril == Trabant.AprilTagToAlign.CENTER){
            // Check rear distance sensor and cast a new pose and create the action to get to
            // the correct distance
            action = drive.actionBuilder(drive.pose)
                    .build();
        } else {
            double newAngleRadians;
            if(robot.targetApril == Trabant.AprilTagToAlign.LEFT) {
                newAngleRadians = current.toDouble() - Math.PI / 2; // Add 90 degrees in radians
            }else{
                newAngleRadians = current.toDouble() + Math.PI / 2; // Add 90 degrees in radians
            }
            newHeading = Rotation2d.fromDouble(newAngleRadians); // Create new Rotation2d
            action = drive.actionBuilder(drive.pose)
                    .turnTo(newHeading)
                    .build();
        }
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
