package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.Trabant;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * The primary operation file for the teleOp phase of the match
 */
@TeleOp(name="Trabant")
public class DriveyMcDriverson extends CommandOpMode {

    /**
     * Autonomous is over. It's time to drive. Forget RoadRunner's need to track our position
     */
    @Override
    public void initialize() {
        /*
         We build our robot. From here on out, we don't need this file. When we build the robot,
         all of our buttons are bound to commands and this class's parent, CommandOpMode, will
         continuously run any scheduled commands. We now slide into the WPILib style.
         */
        Robot m_robot = new Trabant(Trabant.OpModeType.TELEOP, this);
    }

}
