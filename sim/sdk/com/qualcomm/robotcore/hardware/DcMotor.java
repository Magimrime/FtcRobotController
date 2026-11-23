package com.qualcomm.robotcore.hardware;

/** SIMULATOR stand-in for the FTC SDK interface. */
public interface DcMotor extends DcMotorSimple {
    enum ZeroPowerBehavior { UNKNOWN, BRAKE, FLOAT }

    enum RunMode {
        RUN_WITHOUT_ENCODER, RUN_USING_ENCODER, RUN_TO_POSITION, STOP_AND_RESET_ENCODER;

        public boolean isPIDMode() {
            return this == RUN_USING_ENCODER || this == RUN_TO_POSITION;
        }
    }

    void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior);

    ZeroPowerBehavior getZeroPowerBehavior();

    void setMode(RunMode mode);

    RunMode getMode();

    int getCurrentPosition();

    void setTargetPosition(int position);

    int getTargetPosition();

    boolean isBusy();

    int getPortNumber();

    @Deprecated
    default void setPowerFloat() {
        setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);
        setPower(0);
    }

    default boolean getPowerFloat() {
        return getZeroPowerBehavior() == ZeroPowerBehavior.FLOAT && getPower() == 0;
    }
}
