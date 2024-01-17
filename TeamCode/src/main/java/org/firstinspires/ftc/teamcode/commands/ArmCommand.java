package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.Trabant;
import org.firstinspires.ftc.teamcode.config.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;

/**
 * Arm's default teleOp command, this is default way the arm monitors to controller axis
 */
public class ArmCommand extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Arm arm;
    private GamepadEx player2;

    /**
     * Store a reference to our robot
     */
    private final Trabant robot;

    /**
     * Establishes our drive command with references and registering our required subsystem
     *
     * @param robot The robot instantiating this drive command
     */
    public ArmCommand(Trabant robot) {
        this.robot = robot;
        this.arm = robot.arm;
        player2 = robot.player2;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(robot.arm);
    }

    @Override
    public void initialize() {
        // Do you need to set up anything when the command gets first added to the scheduler?
    }

    /**
     * Helper function for applying dead zone
     */
    private double applyDeadZone(double input) {
        return Math.abs(input) <= Constants.INPUT_THRESHOLD ? 0.0d : input;
    }

    @Override
    public void execute() {

        double armRotate = applyDeadZone(player2.getLeftY());
        double elevatorStr = applyDeadZone(player2.getRightY());

        arm.setMotorPower(armRotate);

        // POST DATA
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Arm Pos", arm.getMotorPosition());
        packet.put("Arm Power", arm.getMotorPower());
        FtcDashboard.getInstance().sendTelemetryPacket(packet);

    }

    /**
     * This command never ends
     * @return false
     */
    @Override
    public boolean isFinished() { return false; }

    @Override
    public void end(boolean interrupted){
        arm.stop();
    }
}
