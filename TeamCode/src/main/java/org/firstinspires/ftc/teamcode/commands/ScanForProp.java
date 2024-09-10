package org.firstinspires.ftc.teamcode.commands;


import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.teamcode.Trabant;
import org.firstinspires.ftc.teamcode.util.sensors.DistanceSensor;

/**
 * Slight tweak to the AutoStrafeCommand
 */
public class ScanForProp extends StrafeByDistance {
    // INSTANCE VARIABLES
    private final DistanceSensor leftDistance;
    private final DistanceSensor rightDistance;
    private boolean propOnLeft = false;
    private boolean propOnRight = false;

    public ScanForProp(Trabant robot, double deltaX, double deltaY, double timeout) {
        super(robot, deltaX, deltaY, timeout);
        leftDistance = robot.drive.leftDistance;
        rightDistance = robot.drive.rightDistance;
    }

    @Override
    public void execute() {
        // check left
        if(leftDistance.doubleCheckDistance() <= 10) {
            // trigger exit from command
            propOnLeft = true;
            // store target location on robot
            robot.targetApril = Trabant.AprilTagToAlign.LEFT;
        // check right
        } else if(rightDistance.doubleCheckDistance() <= 10){
            // trigger exit from command
            propOnRight = true;
            // store target location on robot
            robot.targetApril = Trabant.AprilTagToAlign.RIGHT;
        // keep driving?
        } else {
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("x", drive.pose.position.x);
            packet.put("y", drive.pose.position.y);
            packet.put("heading", drive.pose.heading.toDouble());
            // Use the telemetryPacket with the action's run method:
            finished = !action.run(packet);
        }
        // if done, conclude it's the middle
        if(finished && !propOnLeft && !propOnRight){
            robot.targetApril = Trabant.AprilTagToAlign.CENTER;
        }
    }

    @Override
    public boolean isFinished() {
        return finished || propOnLeft || propOnRight || timer.elapsedTime() > timeout;
    }

}
