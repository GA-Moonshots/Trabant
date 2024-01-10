package org.firstinspires.ftc.teamcode;

public class Trabant extends Robot {
    // enum to specify opmode type
    public enum OpModeType {
        TELEOP, AUTO
    }

    public Trabant() {
        this(OpModeType.TELEOP);
    }

    public Trabant(OpModeType type) {
        if (type == OpModeType.TELEOP) {
            initTele();
        } else {
            initAuto();
        }
    }

    /*
     * Initialize teleop or autonomous, depending on which is used
     */
    public void initTele() {
        // initialize teleop-specific scheduler
    }

    public void initAuto() {
        // initialize auto-specific scheduler
    }
}
