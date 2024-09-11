package org.firstinspires.ftc.teamcode.util.sensors;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.autonomous.Localizer;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.OdometryConfig;

@Config
public final class ThreeDeadWheelLocalizer implements Localizer {
    public final Encoder par0, par1, perp;

    private int lastPar0Pos, lastPar1Pos, lastPerpPos;

    public ThreeDeadWheelLocalizer(HardwareMap hardwareMap) {
        // TODO: make sure your config has **motors** with these names (or change them)
        //   the encoders should be plugged into the slot matching the named motor
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        par0 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, Constants.LEFT_ODOMETRY_NAME)));
        par1 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, Constants.RIGHT_ODOMETRY_NAME)));
        perp = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, Constants.CENTER_ODOMETRY_NAME)));

        // TODO: reverse encoder directions if needed
        //   par0.setDirection(DcMotorSimple.Direction.REVERSE);

        lastPar0Pos = par0.getPositionAndVelocity().position;
        lastPar1Pos = par1.getPositionAndVelocity().position;
        lastPerpPos = perp.getPositionAndVelocity().position;

    }

    public Twist2dDual<Time> update() {
        PositionVelocityPair par0PosVel = par0.getPositionAndVelocity();
        PositionVelocityPair par1PosVel = par1.getPositionAndVelocity();
        PositionVelocityPair perpPosVel = perp.getPositionAndVelocity();

        int par0PosDelta = par0PosVel.position - lastPar0Pos;
        int par1PosDelta = par1PosVel.position - lastPar1Pos;
        int perpPosDelta = perpPosVel.position - lastPerpPos;

        Twist2dDual<Time> twist = new Twist2dDual<>(
                new Vector2dDual<>(
                        new DualNum<Time>(new double[] {
                                (OdometryConfig.par0YTicks * par1PosDelta - OdometryConfig.par1YTicks * par0PosDelta) / (OdometryConfig.par0YTicks - OdometryConfig.par1YTicks),
                                (OdometryConfig.par0YTicks * par1PosVel.velocity - OdometryConfig.par1YTicks * par0PosVel.velocity) / (OdometryConfig.par0YTicks - OdometryConfig.par1YTicks),
                        }).div(OdometryConfig.ticksPerInch),
                        new DualNum<Time>(new double[] {
                                (OdometryConfig.perpXTicks / (OdometryConfig.par0YTicks - OdometryConfig.par1YTicks) * (par1PosDelta - par0PosDelta) + perpPosDelta),
                                (OdometryConfig.perpXTicks / (OdometryConfig.par0YTicks - OdometryConfig.par1YTicks) * (par1PosVel.velocity - par0PosVel.velocity) + perpPosVel.velocity),
                        }).div(OdometryConfig.ticksPerInch)
                ),
                new DualNum<>(new double[] {
                        (par0PosDelta - par1PosDelta) / (OdometryConfig.par0YTicks - OdometryConfig.par1YTicks),
                        (par0PosVel.velocity - par1PosVel.velocity) / (OdometryConfig.par0YTicks - OdometryConfig.par1YTicks),
                })
        );

        lastPar0Pos = par0PosVel.position;
        lastPar1Pos = par1PosVel.position;
        lastPerpPos = perpPosVel.position;

        FtcDashboard.getInstance().getTelemetry().addData("par0", par0PosVel.position);
        FtcDashboard.getInstance().getTelemetry().addData("par1", par1PosVel.position);
        FtcDashboard.getInstance().getTelemetry().addData("perp", perpPosVel.position);
        FtcDashboard.getInstance().getTelemetry().update();

        return twist;
    }
}
