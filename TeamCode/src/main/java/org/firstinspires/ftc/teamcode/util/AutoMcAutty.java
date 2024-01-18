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
        // QUERY USER TO DETERMINE OUR STARTING COORDINATES
        boolean isLeft = false;
        boolean isRed = false;
        boolean goForBoard = false;
        boolean gotoOpposite = false;
        // give player time to enter selection
        while(opModeInInit()) {
            // press X for blue and B for red
            if (gamepad1.x)
                isRed = false;
            else if (gamepad1.b && !gamepad1.start)
                isRed = true;
            // press dpad LEFT for left and RIGHT for right
            if (gamepad1.dpad_left)
                isLeft = true;
            else if (gamepad1.dpad_right)
                isLeft = false;
            // press Y to go for 45 and A just to park
            if (gamepad1.y)
                goForBoard = true;
            else if (gamepad1.a && !gamepad1.start)
                goForBoard = false;
            // DISPLAY SELECTION
            telemetry.addData("Position", "%s Team, %s Side", isRed ? "Red" : "Blue",
                    isLeft ? "Left" : "Right");
            telemetry.addData("Target Points", "%s", goForBoard ? "45" : "25");
            telemetry.update();
        }
        /*
         We build our robot. From here on out, we don't need this file. When we build the robot,
         all of our buttons are bound to commands and this class's parent, CommandOpMode, will
         continuously run any scheduled commands. We now slide into the WPILib style.
         We pass in our autonomous config variables, which signals to the robot we want to be in
         autonomous mode instead of in teleop mode, which would take no params besides this.
         */
        Robot m_robot = new Trabant(this, isRed, isLeft, goForBoard);
    }

}
