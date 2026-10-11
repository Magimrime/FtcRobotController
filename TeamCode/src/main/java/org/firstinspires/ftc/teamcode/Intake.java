package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/*
 * The intake at the front of the robot: one motor plus a continuous-rotation servo on each side
 * that help pull balls in. The two feeder servos run the whole match; the motor runs on demand.
 *
 *   mainmotorintake   Expansion Hub motor port 0
 *   leftfeeder        Control Hub servo port 0
 *   rightfeeder       Expansion Hub servo port 0
 *
 * Positive power pulls balls in, negative pushes them back out.
 */
public class Intake {

    public static final String MOTOR_NAME = "mainmotorintake";
    public static final String LEFT_FEEDER_NAME = "leftfeeder";
    public static final String RIGHT_FEEDER_NAME = "rightfeeder";

    // Power levels, 0 to 1.
    public static final double IN_POWER = 1.0;
    public static final double OUT_POWER = 0.7;

    private final DcMotor motor;
    private final CRServo leftFeeder;
    private final CRServo rightFeeder;

    // The last powers sent, for telemetry.
    private double motorPower;
    private double leftFeederPower;
    private double rightFeederPower;

    public Intake(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotor.class, MOTOR_NAME);
        leftFeeder = hardwareMap.get(CRServo.class, LEFT_FEEDER_NAME);
        rightFeeder = hardwareMap.get(CRServo.class, RIGHT_FEEDER_NAME);

        // Positive power must pull balls IN on all three. The two feeders face each other across
        // the intake, so one of them is reversed. Check with the "Robot Hardware Test" utility and
        // flip FORWARD/REVERSE on anything that pushes balls out instead.
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFeeder.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFeeder.setDirection(DcMotorSimple.Direction.REVERSE);

        // FLOAT lets the intake coast to a stop instead of jerking a half-captured ball.
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /** Pulls balls in. */
    public void in() {
        run(IN_POWER);
    }

    /** Pushes balls back out, for clearing a jam or an extra ball. */
    public void out() {
        run(-OUT_POWER);
    }

    /** Stops the motor but keeps both feeders pulling balls in. */
    public void feedersOnly() {
        run(0.0, IN_POWER, IN_POWER);
    }

    public void stop() {
        run(0.0);
    }

    /** Runs the motor and both feeders at the same power: -1 to 1, positive pulls balls in. */
    public void run(double power) {
        run(power, power, power);
    }

    /** Runs each part separately, -1 to 1. The hardware test uses this to check one at a time. */
    public void run(double motorPower, double leftFeederPower, double rightFeederPower) {
        motor.setPower(motorPower);
        leftFeeder.setPower(leftFeederPower);
        rightFeeder.setPower(rightFeederPower);

        this.motorPower = motorPower;
        this.leftFeederPower = leftFeederPower;
        this.rightFeederPower = rightFeederPower;
    }

    /** True while pulling balls in. */
    public boolean isRunningIn() {
        return motorPower > 0.0;
    }

    /** Adds the configured ports, to compare with the list at the top of this file. */
    public void addPortTelemetry(Telemetry telemetry) {
        telemetry.addData("Intake ports", "motor %d   left feeder %d   right feeder %d",
                motor.getPortNumber(), leftFeeder.getPortNumber(), rightFeeder.getPortNumber());
    }

    /** Adds the last powers to telemetry. The caller still calls telemetry.update(). */
    public void addTelemetry(Telemetry telemetry) {
        String state = motorPower > 0.0 ? "In" : motorPower < 0.0 ? "Out"
                : leftFeederPower != 0.0 || rightFeederPower != 0.0 ? "Feeders only" : "Stopped";
        telemetry.addData("Intake", "%s  (motor %4.2f, feeders %4.2f / %4.2f)",
                state, motorPower, leftFeederPower, rightFeederPower);
    }
}
