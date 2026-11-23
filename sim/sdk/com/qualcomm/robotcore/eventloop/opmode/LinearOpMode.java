package com.qualcomm.robotcore.eventloop.opmode;

import ftcsim.SimContext;

/**
 * SIMULATOR stand-in for the FTC SDK's LinearOpMode. INIT, START and STOP come from the simulated
 * Driver Station (ftcsim.SimContext): START after the scenario's INIT time, STOP 30 s later.
 * Every call costs a little simulated time, so loops that only check opModeIsActive() still move on.
 */
public abstract class LinearOpMode extends OpMode {

    public abstract void runOpMode() throws InterruptedException;

    public void waitForStart() {
        while (!isStarted() && !isStopRequested()) {
            idle();
        }
    }

    public final boolean opModeIsActive() {
        SimContext c = SimContext.get();
        c.tick(c.scenario.costLoopCheck);
        return c.isStarted() && !c.isStopRequested();
    }

    public final boolean opModeInInit() {
        SimContext c = SimContext.get();
        c.tick(c.scenario.costLoopCheck);
        return !c.isStarted() && !c.isStopRequested();
    }

    public final boolean isStarted() {
        return SimContext.get().isStarted();
    }

    public final boolean isStopRequested() {
        SimContext c = SimContext.get();
        c.tick(c.scenario.costLoopCheck);
        return c.isStopRequested();
    }

    public final void idle() {
        SimContext.get().tick(0.0002);
    }

    public final void sleep(long milliseconds) {
        SimContext.get().sleep(milliseconds / 1000.0);
    }
}
