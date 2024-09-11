package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.Trabant;

import java.util.Locale;

public class Arm extends SubsystemBase {
    // SUBSYSTEM ASSETS
    private final Trabant robot;
    private final Servo wristServo, openServo, rollServo;
    public final MotorEx motor;
    // STATE VARIABLES
    private double wristAng = Constants.WRIST_MIN;
    private double rollPos = Constants.ROLL_MAX;
    private boolean isOpen = false;
    private int offset = 0;

    public Arm(Trabant robot) {
        this.robot = robot;
        HardwareMap hardwareMap = robot.opMode.hardwareMap;

        // shoulder motor
        this.motor = new MotorEx(hardwareMap, Constants.ARM_MOTOR_NAME);
        motor.setRunMode(Motor.RunMode.PositionControl);
        motor.setInverted(true);
        motor.setPositionCoefficient(Constants.ARM_MOTOR_kP);
        motor.setPositionTolerance(Constants.ARM_MOTOR_TOLERANCE);

        // servos
        wristServo = hardwareMap.get(Servo.class, Constants.WRIST_SERVO_NAME);
        openServo = hardwareMap.get(Servo.class, Constants.OPEN_SERVO_NAME);
        rollServo = hardwareMap.get(Servo.class, Constants.ROLL_SERVO_NAME);
        wristServo.setPosition(wristAng);
        openServo.setPosition(Constants.CLAW_CLOSED_POS);
        rollServo.setPosition(rollPos);
    }

    // -- SERVO COMMANDS --
    public void travelMode() {
        closeClaw();
        wristServo.setPosition(0);
        wristAng = 0;
    }
    public void wristUp() {
        wristAng -= Constants.WRIST_INC;
        wristAng = Range.clip(wristAng, Constants.WRIST_MIN, Constants.WRIST_MAX);
        wristServo.setPosition(wristAng);
    }
    public void wristTo(double wristToMove) {
        wristToMove = Range.clip(wristToMove, Constants.WRIST_MIN, Constants.WRIST_MAX);
        wristServo.setPosition(wristToMove);
        wristAng = wristToMove;
    }
    public void wristDown() {
        wristAng += Constants.WRIST_INC;
        wristAng = Range.clip(wristAng, Constants.WRIST_MIN, Constants.WRIST_MAX);
        wristServo.setPosition(wristAng);
    }
    public void openClaw() {
        openServo.setPosition(Constants.CLAW_OPEN_POS);
        isOpen = true;
    }
    public void closeClaw() {
        openServo.setPosition(Constants.CLAW_CLOSED_POS);
        isOpen = false;
    }
    public void toggleOpen() {
        if(isOpen) {
            closeClaw();
        } else {
            openClaw();
        }
    }
    public void toggleRoll() {
        if(rollServo.getPosition() >= .5){
            rollServo.setPosition(Constants.ROLL_MIN);
            rollPos = Constants.ROLL_MIN;
        } else{
            rollServo.setPosition(Constants.ROLL_MAX);
            rollPos = Constants.ROLL_MAX;
        }
    }
    public void rollPositive() {
        rollPos +=  Constants.ROLL_INC;
        if(rollPos > Constants.ROLL_MAX) rollPos = Constants.ROLL_MAX;
        rollServo.setPosition(rollPos);
    }
    public void rollNegative() {
        rollPos -=  Constants.ROLL_INC;
        if(rollPos < Constants.ROLL_MIN) rollPos = Constants.ROLL_MIN;
        rollServo.setPosition(rollPos);
    }

    // -- MOTOR UTILITIES --
    public void changeOffset(int delta) {
        offset += delta;
    }
    public void stop(){
        motor.stopMotor();
    }
    public double getMotorPower(){
        return motor.get();
    }
    public double getMotorPosition(){
        return motor.getCurrentPosition();
    }
    public void setMotorPower(double power){
        motor.set(power);
    }
    public double getOffset(){
        return offset;
    }

    @NonNull
    @Override
    public String toString() {
        return String.format(Locale.ENGLISH, "OPR: (%f, %f, %f)",
                openServo.getPosition(), wristServo.getPosition(), rollServo.getPosition());
    }
}
