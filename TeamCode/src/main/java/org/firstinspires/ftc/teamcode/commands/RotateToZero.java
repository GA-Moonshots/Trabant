package org.firstinspires.ftc.teamcode.commands;


import org.firstinspires.ftc.teamcode.Trabant;

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
