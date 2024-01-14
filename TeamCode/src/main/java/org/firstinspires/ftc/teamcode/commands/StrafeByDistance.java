package org.firstinspires.ftc.teamcode.commands;


import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.Trabant;
import org.firstinspires.ftc.teamcode.autnomous.RoadRunnerDrive;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.DistanceSensor;

public class StrafeByDistance extends CommandBase {

    protected final MecanumDrive drive;
    protected final Trabant robot;
    private final DistanceSensor leftDistance, rightDistance;
    private final double deltaX, deltaY;
    protected final double timeout;
    /**
     * Our RoadRunner element
     */
    protected Action action;
    protected Timing.Timer timer;
    protected boolean finished = false;


    public StrafeByDistance(Trabant robot, double deltaX, double deltaY, double timeout) {
        this.robot = robot;
        this.drive = robot.drive;
        this.leftDistance = drive.leftDistance;
        this.rightDistance = drive.rightDistance;
        this.deltaX = deltaX;
        this.deltaY = deltaY;
        this.timeout = timeout;

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        timer.start();

        RoadRunnerDrive d = new RoadRunnerDrive(robot.opMode.hardwareMap, drive.pose);
        Vector2d current = drive.pose.position;
        Vector2d targetPosition = new Vector2d(current.x + deltaX, current.y + deltaY);

        action = drive.actionBuilder(drive.pose)
                .strafeTo(targetPosition)
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
