package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autnomous.RoadRunnerDrive;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.Robot;

/**
 * Primary robot file. This is where hardware and inputs are assembled
 */
public class Trabant extends Robot {

    // INSTANCE VARIABLES
    public LinearOpMode opMode;
    public GamepadEx player1;
    public GamepadEx player2;
    // SUBSYSTEMS
    public MecanumDrive drive;


    /**
     * This guides the state of the robot, like whether or not to listen to the controller
     */
    public enum OpModeType {
        TELEOP, AUTO
    }

    /**
     * Robot factory
     * @param type
     * @param opMode
     */
    public Trabant(OpModeType type, LinearOpMode opMode) {
        this.opMode = opMode;
        player1 = new GamepadEx(opMode.gamepad1);
        player2 = new GamepadEx(opMode.gamepad2);

        // TODO: Use init loop in opMode to query user for starting position
        drive = new MecanumDrive(this, new Pose2d(0, 0, 0));
        register(drive);

        if (type == OpModeType.TELEOP) {
            initTele();
        } else {
            initAuto();
        }
    }

    /**
     * Set teleOp's default commands and player control bindings
     */
    public void initTele() {
        drive.setDefaultCommand(new DriveCommand(this));

        // PLAYER 1


    }

    /**
     * Query user for starting position and call the corresponding commands
     */
    public void initAuto() {

    }
}
