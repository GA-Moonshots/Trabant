package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Trabant;
import org.firstinspires.ftc.teamcode.autnomous.RoadRunnerDrive;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;

/**
 * An example command that uses an example subsystem.
 */
public class DriveCommand extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final MecanumDrive drive;
    private final Trabant robot;

    /**
     * Creates a new ExampleCommand.
     *
     * @param robot The robot file instantiating this drive command
     */
    public DriveCommand(Trabant robot) {
        this.robot = robot;
        this.drive = robot.drive;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(robot.drive);
    }

    @Override
    public void initialize() {
        // Do you need to set up anything in the
    }

    @Override
    public void end(boolean interupted){

    }
}
