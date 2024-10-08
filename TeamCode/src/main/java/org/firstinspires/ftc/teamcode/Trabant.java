package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.ArmMoveTo;
import org.firstinspires.ftc.teamcode.commands.RotateToZero;
import org.firstinspires.ftc.teamcode.commands.StrafeByDistance;
import org.firstinspires.ftc.teamcode.commands.Drive;
import org.firstinspires.ftc.teamcode.commands.ScanForProp;
import org.firstinspires.ftc.teamcode.commands.TurnToProp;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Mecanum;
import org.firstinspires.ftc.teamcode.util.Robot;


/**
 * Primary robot file. This is where hardware and inputs are assembled
 */
public class Trabant extends Robot {

    // INSTANCE VARIABLES
    public LinearOpMode opMode;
    public Telemetry telemetry;
    public GamepadEx player1;
    public GamepadEx player2;
    public enum AprilTagToAlign {
        LEFT, CENTER, RIGHT, NONE
    }
    public AprilTagToAlign targetApril = AprilTagToAlign.NONE;

    // SUBSYSTEMS
    public Mecanum drive;
    public Arm arm;


    /**
     * This guides the state of the robot, like whether or not to listen to the controller
     */
    public enum OpModeType {
        TELEOP, AUTO
    }

    /**
     * Welcome to the Command pattern. Here we assemble the robot and kick-off the command
     * @param opMode The selected operation mode
     */
    public Trabant(LinearOpMode opMode) {
        this.opMode = opMode;
        // DISABLE FTC DASHBOARD FOR COMPETITION
        telemetry = new MultipleTelemetry(opMode.telemetry, FtcDashboard.getInstance().getTelemetry());
        player1 = new GamepadEx(opMode.gamepad1);
        player2 = new GamepadEx(opMode.gamepad2);

        initTele();
    }

    public Trabant(LinearOpMode opMode, boolean isRed, boolean isLeft, boolean goForBoard) {
        this.opMode = opMode;
        // DISABLE FTC DASHBOARD FOR COMPETITION
        telemetry = new MultipleTelemetry(opMode.telemetry, FtcDashboard.getInstance().getTelemetry());
        initAuto(isRed, isLeft, goForBoard);
    }

    /**
     * Set teleOp's default commands and player control bindings
     */
    public void initTele() {
        // throw-away pose because we're not localizing anymore
        drive = new Mecanum(this, new Pose2d(0,0,0));
        register(drive);
        drive.setDefaultCommand(new Drive(this));
        // start arm
        arm = new Arm(this);
        register(arm);

        /*
                .__                                      ____
        ______  |  |  _____   ___.__.  ____ _______     /_   |
        \____ \ |  |  \__  \ <   |  |_/ __ \\_  __ \     |   |
        |  |_> >|  |__ / __ \_\___  |\  ___/ |  | \/     |   |
        |   __/ |____/(____  // ____| \___  >|__|        |___|
        |__|               \/ \/          \/
        */
        Button aButtonP1 = new GamepadButton(player1, GamepadKeys.Button.A);
        aButtonP1.whenPressed(new InstantCommand(() -> {
            drive.toggleFieldCentric();
        }));

        Button bButtonP1 = new GamepadButton(player1, GamepadKeys.Button.B);
        bButtonP1.whenPressed(new InstantCommand(() -> {
            drive.resetFieldCentricTarget();
        }));

        Button xButtonP1 = new GamepadButton(player1, GamepadKeys.Button.X);
        Button yButtonP1 = new GamepadButton(player1, GamepadKeys.Button.Y);
        Button dPadUpP1 = new GamepadButton(player1, GamepadKeys.Button.DPAD_UP);
        Button dPadDownP1 = new GamepadButton(player1, GamepadKeys.Button.DPAD_DOWN);
        Button dPadLeftP1 = new GamepadButton(player1, GamepadKeys.Button.DPAD_LEFT);
        Button dPadRightP1 = new GamepadButton(player1, GamepadKeys.Button.DPAD_RIGHT);

        /*
                _                                    __
               (_ )                                /'__`\
         _ _    | |    _ _  _   _    __   _ __    (_)  ) )
        ( '_`\  | |  /'_` )( ) ( ) /'__`\( '__)      /' /
        | (_) ) | | ( (_| || (_) |(  ___/| |       /' /( )
        | ,__/'(___)`\__,_)`\__, |`\____)(_)      (_____/'
        | |                ( )_| |
        (_)                `\___/'
         */
        Button aButtonP2 = new GamepadButton(player2, GamepadKeys.Button.A);
        aButtonP2.whenPressed(new InstantCommand(() -> {
            arm.travelMode();
        }));

        Button bButtonP2 = new GamepadButton(player2, GamepadKeys.Button.B);
        bButtonP2.whenPressed(new InstantCommand(() -> {
            arm.toggleOpen();
        }));

        Button xButtonP2 = new GamepadButton(player2, GamepadKeys.Button.X);
        xButtonP2.whenHeld(new ArmMoveTo(this, ArmMoveTo.ArmPosition.GROUND));

        Button yButtonP2 = new GamepadButton(player2, GamepadKeys.Button.Y);
        yButtonP2.whenHeld(new ArmMoveTo(this, ArmMoveTo.ArmPosition.HIGH_DROP));

        Button dPadUpP2 = new GamepadButton(player2, GamepadKeys.Button.DPAD_UP);
        dPadUpP2.whileHeld(new InstantCommand(() -> {
            arm.wristUp();
        }));

        Button dPadDownP2 = new GamepadButton(player2, GamepadKeys.Button.DPAD_DOWN);
        dPadDownP2.whileHeld(new InstantCommand(() -> {
            arm.wristDown();
        }));

        Button dPadLeftP2 = new GamepadButton(player2, GamepadKeys.Button.DPAD_LEFT);
        dPadLeftP2.whileHeld(new InstantCommand(() -> {
            arm.rollNegative();
        }));

        Button dPadRightP2 = new GamepadButton(player2, GamepadKeys.Button.DPAD_RIGHT);
        dPadRightP2.whileHeld(new InstantCommand(() -> {
            arm.rollPositive();
        }));

    }

    /**
     * Query user for starting position and call the corresponding commands
     */
    public void initAuto(boolean isRed, boolean isLeft, boolean goForBoard) {
        // TODO: Calculate pose for each of the four starting positions
        Pose2d start;
        // RED LEFT
        if(isRed && isLeft)
            start = new Pose2d(new Vector2d(0, 0), 0.0);
        // RED RIGHT
        else if(isRed)
            start = new Pose2d(new Vector2d(0, 0), 0.0);
        // BLUE LEFT
        else if(isLeft)
            start = new Pose2d(new Vector2d(0, 0), 0.0);
        // BLUE RIGHT
        else
            start = new Pose2d(new Vector2d(0, 0), 0.0);

        drive = new Mecanum(this, start);
        register(drive);

        // locate prop, drop piece, withdraw and straighten out
        new SequentialCommandGroup(
                new ScanForProp(this,0,20, 4),
                new TurnToProp(this, 1),
                new InstantCommand(() -> {
                    arm.openClaw();
                    opMode.sleep(25);
                    arm.travelMode();
                }),
                new StrafeByDistance(this, 0, -2, 0.5),
                new RotateToZero(this, 2)
        ).schedule();

        // TODO: complete autonomous command sequence

    }
}
