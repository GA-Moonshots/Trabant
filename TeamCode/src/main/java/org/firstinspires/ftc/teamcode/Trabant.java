package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.autnomous.RoadRunnerDrive;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.Robot;

/**
 * Primary robot file. This is where hardware and inputs are assembled
 */
public class Trabant extends Robot {

    // SUBSYSTEMS
    public MecanumDrive drive;

    /**
     * This guides the state of the robot, like whether or not to listen to the controller
     */
    public enum OpModeType {
        TELEOP, AUTO
    }

    public Trabant(OpModeType type) {
        if (type == OpModeType.TELEOP) {
            initTele();
        } else {
            initAuto();
        }
    }

    /**
     * Initialize teleop or autonomous, depending on which enum option was passed to the constructor
     */
    public void initTele() {
        // initialize teleop-specific scheduler
    }

    public void initAuto() {
        // initialize auto-specific scheduler
    }
}
