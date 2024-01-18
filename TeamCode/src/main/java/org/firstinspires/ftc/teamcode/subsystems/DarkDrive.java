package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Trabant;
import org.firstinspires.ftc.teamcode.autonomous.Localizer;
import org.firstinspires.ftc.teamcode.sensors.ThreeDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.config.HardwareNames;
import org.firstinspires.ftc.teamcode.config.OdometryConfig;

/**
 * An attempt to build our own RoadRunner mecanum implementation
 */
public class DarkDrive extends SubsystemBase {
    private boolean isFieldCentric = true;
    private double fieldCentricTarget = 0.0;

    private DcMotorEx leftFront, leftBack, rightFront, rightBack;
    private Localizer localizer;
    private IMU imu;

    public DarkDrive(Trabant robot, Pose2d startPose) {
        HardwareMap hardwareMap = robot.opMode.hardwareMap;
        this.leftFront = hardwareMap.get(DcMotorEx.class, HardwareNames.LEFT_FRONT_NAME);
        this.leftBack = hardwareMap.get(DcMotorEx.class, HardwareNames.LEFT_BACK_NAME);
        this.rightFront = hardwareMap.get(DcMotorEx.class, HardwareNames.RIGHT_FRONT_NAME);
        this.rightBack = hardwareMap.get(DcMotorEx.class, HardwareNames.RIGHT_BACK_NAME);

        this.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.localizer = new ThreeDeadWheelLocalizer(hardwareMap);
    }

    public void drive(double forward, double strafe, double turn) {
        Twist2dDual<Time> position = localizer.update();
    }

}
