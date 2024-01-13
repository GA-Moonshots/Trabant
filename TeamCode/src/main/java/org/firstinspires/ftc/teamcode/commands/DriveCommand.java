package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Trabant;
import org.firstinspires.ftc.teamcode.autnomous.RoadRunnerDrive;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;

/**
 * MecanumDrive's default command, this is how we listen to the controller
 */
public class DriveCommand extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final MecanumDrive drive;

    /**
     * Store a reference to our robot
     */
    private final Trabant robot;

    /**
     * Establishes our drive command with references and registering our required subsystem
     *
     * @param robot The robot instantiating this drive command
     */
    public DriveCommand(Trabant robot) {
        this.robot = robot;
        this.drive = robot.drive;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(robot.drive);
    }

    @Override
    public void initialize() {
        // Do you need to set up anything when the command gets first added to the scheduler?
    }

    @Override
    public void execute() {
        // -- PLAYER 1 --


        // -- PLAYER 2 --

    }

    /**
     * This function never ends
     * @return false
     */
    public boolean isFinished() { return false; }

    @Override
    public void end(boolean interrupted){
        drive.stop();
    }
}
