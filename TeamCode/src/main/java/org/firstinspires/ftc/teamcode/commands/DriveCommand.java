package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.config.HardwareNames;
import org.firstinspires.ftc.teamcode.Trabant;
import org.firstinspires.ftc.teamcode.subsystems.DarkDrive;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;

/**
 * MecanumDrive's default teleOp command, this is default way the drive listens to the controller
 */
public class DriveCommand extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final MecanumDrive drive;
    private GamepadEx player1;

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
        player1 = robot.player1;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(robot.drive);
    }

    @Override
    public void initialize() {
        // Do you need to set up anything when the command gets first added to the scheduler?
    }

    /**
     * Helper function for applying dead zone
     */
    private double applyDeadZone(double input) {
        return Math.abs(input) <= HardwareNames.INPUT_THRESHOLD ? 0.0d : input;
    }

    @Override
    public void execute() {

        // you can go digging for the opMode controller
        double speedMod = robot.opMode.gamepad1.right_bumper ? 0.2 : 1; // slow mode

        double forward = -applyDeadZone(player1.getLeftY());
        double strafe = applyDeadZone(player1.getLeftX());
        double turn = applyDeadZone(player1.getRightX());

        // Drive the robot with adjusted inputs:
        drive.drive(forward * speedMod, strafe * speedMod, turn * speedMod);

    }

    /**
     * This command never ends
     * @return false
     */
    @Override
    public boolean isFinished() { return false; }

    @Override
    public void end(boolean interrupted){
        drive.stop();
    }
}
