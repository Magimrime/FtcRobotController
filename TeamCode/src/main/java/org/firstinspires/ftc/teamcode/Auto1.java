package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * Auto1: shoots the 4 pre-loaded balls and nothing else. The robot does not move.
 * The flywheel spins up to 3000 RPM, then the siphon turns at least 8 times to feed the balls.
 * The siphon servo can't count its turns, so it runs for 8 x SECONDS_PER_TURN: time one siphon
 * turn at SIPHON_POWER and put that number here.
 */
@Autonomous(name = "Auto1", group = "Robot", preselectTeleOp = "TeleOp")
public class Auto1 extends LinearOpMode {

    private static final double RPM = 3000.0;
    private static final double SIPHON_TURNS = 8.0;
    private static final double SECONDS_PER_TURN = 2.0;  // measure on the robot
    private static final double SIPHON_POWER = 0.5;      // half speed: steadier shots
    private static final double TICKS_PER_REV = 28.0;    // flywheel motor, same as TeleOp

    @Override
    public void runOpMode() {
        DcMotorEx flywheel = hardwareMap.get(DcMotorEx.class, "turret");
        CRServo siphon = hardwareMap.get(CRServo.class, "servosiphon");
        DcMotor intake = hardwareMap.get(DcMotor.class, "mainmotorintake");
        CRServo leftFeeder = hardwareMap.get(CRServo.class, "leftfeeder");
        CRServo rightFeeder = hardwareMap.get(CRServo.class, "rightfeeder");
        siphon.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFeeder.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setVelocityPIDFCoefficients(40.0, 0.0, 0.0, 32767.0 / (6000.0 * TICKS_PER_REV / 60.0));

        waitForStart();
        flywheel.setVelocity(RPM * TICKS_PER_REV / 60.0);

        // Wait for the flywheel to reach 3000 RPM (2 seconds at most).
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < 2.0 && rpm(flywheel) < RPM - 75.0) {
            idle();
        }

        // Feed: the siphon, plus the intake and feeders, which push the balls up to it.
        siphon.setPower(SIPHON_POWER);
        intake.setPower(1.0);
        leftFeeder.setPower(1.0);
        rightFeeder.setPower(1.0);
        timer.reset();
        while (opModeIsActive() && timer.seconds() < SIPHON_TURNS * SECONDS_PER_TURN) {
            telemetry.addData("Flywheel", "%.0f RPM", rpm(flywheel));
            telemetry.addData("Siphon", "about %.1f of %.0f turns", timer.seconds() / SECONDS_PER_TURN, SIPHON_TURNS);
            telemetry.update();
        }
        // Done. Everything stops when the OpMode ends.
    }

    private static double rpm(DcMotorEx flywheel) {
        return flywheel.getVelocity() * 60.0 / TICKS_PER_REV;
    }
}
