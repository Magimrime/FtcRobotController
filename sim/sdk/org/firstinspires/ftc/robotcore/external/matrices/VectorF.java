package org.firstinspires.ftc.robotcore.external.matrices;

/** SIMULATOR stand-in for the FTC SDK class. */
public class VectorF {
    protected final float[] data;

    public VectorF(float... data) {
        this.data = data.clone();
    }

    public int length() {
        return data.length;
    }

    public float get(int index) {
        return data[index];
    }

    public void put(int index, float value) {
        data[index] = value;
    }
}
