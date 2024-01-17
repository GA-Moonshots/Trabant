package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Trabant;

/**
 * The primary operation file for the teleOp phase of the match
 */
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Autonomous - Primary")
public class AutoMcAutty extends CommandOpMode {

    /**
     * Set up robot such that it asks the player what our starting position is and kicks off
     * a FTCLib-style RoadRunner.
     */
    @Override
    public void initialize() {
        /*
         We build our robot. From here on out, we don't need this file. When we build the robot,
         all of our buttons are bound to commands and this class's parent, CommandOpMode, will
         continuously run any scheduled commands. We now slide into the WPILib style.
         */
        Robot m_robot = new Trabant(Trabant.OpModeType.AUTO, this);
    }

}
