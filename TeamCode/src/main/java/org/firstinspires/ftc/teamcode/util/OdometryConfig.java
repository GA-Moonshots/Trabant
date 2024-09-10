package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;

@Config
public class OdometryConfig {
    public static double par0YTicks = -4022; // y position of the first parallel encoder (in tick units)
    public static double par1YTicks = 4022; // y position of the second parallel encoder (in tick units)
    public static double perpXTicks = 8466; // x position of the perpendicular encoder (in tick units)

    public static double ticksPerInch = 1058;
}
