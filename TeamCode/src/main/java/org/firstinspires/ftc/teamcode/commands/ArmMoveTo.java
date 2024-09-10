package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.HardwareNames;
import org.firstinspires.ftc.teamcode.Trabant;
import org.firstinspires.ftc.teamcode.subsystems.Arm;

/**
 * The arm motor's default teleOp command, this is default way the shoulder listens to the controller
 * Servo functions are implemented in the subsystem class
 */
public class ArmMoveTo extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Arm arm;
    private final GamepadEx player2;
    public enum ArmPosition {
        HIGH_DROP, LOW_DROP, GROUND
    }
    public ArmPosition requestedPos;
    public final Telemetry telemetry;
    public int target;

    /**
     * Store a reference to our robot
     */
    private final Trabant robot;

    /**
     * Links references to handy instance variables and requests the required subsystem
     * Instantaneous servo controls are found within the subsystem class
     * @param robot The robot instantiating this drive command
     */
    public ArmMoveTo(Trabant robot, ArmPosition requestedPos) {
        this.robot = robot;
        this.telemetry = robot.opMode.telemetry;
        this.arm = robot.arm;
        player2 = robot.player2;

        this.requestedPos = requestedPos;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(robot.arm);
    }

    @Override
    public void initialize() {
        // CALCULATE TARGET AND SET POSITION
        if(requestedPos == ArmPosition.GROUND){
            target = HardwareNames.ARM_DOWN_POSITION;
        }else{
            target = HardwareNames.ARM_UP_POSITION;
        }

        arm.motor.setTargetPosition(target);
    }

    @Override
    public void execute() {
        arm.motor.set(HardwareNames.ARM_MOTOR_STRENGTH);
        // POST DATA
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Arm Pos", arm.getMotorPosition());
        packet.put("Arm Power", arm.getMotorPower());
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    /**
     * Default commands never end, so we'll just return false here
     * @return false
     */
    @Override
    public boolean isFinished() { return arm.motor.atTargetPosition(); }

    @Override
    public void end(boolean interrupted){
        arm.stop();
    }
}
