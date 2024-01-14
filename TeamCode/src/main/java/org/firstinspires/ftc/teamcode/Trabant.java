package org.firstinspires.ftc.teamcode;

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
import org.firstinspires.ftc.teamcode.commands.RotateByDegree;
import org.firstinspires.ftc.teamcode.commands.RotateToZero;
import org.firstinspires.ftc.teamcode.commands.StrafeByDistance;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.ScanForProp;
import org.firstinspires.ftc.teamcode.commands.TurnToProp;
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
    public enum AprilTagToAlign {
        LEFT,
        CENTER,
        RIGHT,
        NONE
    }
    public AprilTagToAlign targetApril = AprilTagToAlign.NONE;

    // SUBSYSTEMS
    public MecanumDrive drive;
    public Telemetry telemetry;


    /**
     * This guides the state of the robot, like whether or not to listen to the controller
     */
    public enum OpModeType {
        TELEOP, AUTO
    }

    /**
     * Welcome to the Command pattern. Here we assemble the robot and kick-off the command
     * @param type Select TeleOp or Auto
     * @param opMode The selected operation mode
     */
    public Trabant(OpModeType type, LinearOpMode opMode) {
        this.opMode = opMode;
        this.telemetry = opMode.telemetry;
        player1 = new GamepadEx(opMode.gamepad1);
        player2 = new GamepadEx(opMode.gamepad2);


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
        // throw-away pose because we're not localizing anymore
        drive = new MecanumDrive(this, new Pose2d(0,0,0));
        register(drive);
        drive.setDefaultCommand(new DriveCommand(this));

        // PLAYER 1
        Button aButton = new GamepadButton(player1, GamepadKeys.Button.A);
        aButton.whenPressed(new InstantCommand(() -> {
            drive.toggleFieldCentric();
        }));

        Button bButton = new GamepadButton(player1, GamepadKeys.Button.B);

        Button xButton = new GamepadButton(player1, GamepadKeys.Button.X);

        Button yButton = new GamepadButton(player1, GamepadKeys.Button.Y);
        yButton.whenPressed(new InstantCommand(() -> {
            drive.resetFieldCentricTarget();
        }));
    }

    /**
     * Query user for starting position and call the corresponding commands
     */
    public void initAuto() {

        // QUERY USER TO DETERMINE OUR STARTING COORDINATES
        boolean isLeft = false;
        boolean isRed = false;
        boolean goForBoard = false;
        boolean gotoOpposite = false;
        // give player time to enter selection
        while(opMode.opModeInInit()) {
            // press X for blue and B for red
            if (opMode.gamepad1.x)
                isRed = false;
            else if (opMode.gamepad1.b && !opMode.gamepad1.start)
                isRed = true;
            // press dpad LEFT for left and RIGHT for right
            if (opMode.gamepad1.dpad_left)
                isLeft = true;
            else if (opMode.gamepad1.dpad_right)
                isLeft = false;
            // press Y to go for 45 and A just to park
            if (opMode.gamepad1.y)
                goForBoard = true;
            else if (opMode.gamepad1.a && !opMode.gamepad1.start)
                goForBoard = false;
            // DISPLAY SELECTION
            telemetry.addData("Position", "%s Team, %s Side", isRed ? "Red" : "Blue", isLeft ? "Left" : "Right");
            telemetry.addData("Target Points", "%s", goForBoard ? "45" : "25");
            telemetry.update();
        }

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

        drive = new MecanumDrive(this, start);
        register(drive);

        // locate prop, drop piece, withdraw and straighten out
        new SequentialCommandGroup(
                new ScanForProp(this,0,20, 4),
                new TurnToProp(this, 1),
                // new ReleaseRetractCommand(this),
                new StrafeByDistance(this, 0, -2, 0.5),
                new RotateToZero(this, 2)
        ).schedule();

        // TODO: complete autonomous command sequence

    }
}
