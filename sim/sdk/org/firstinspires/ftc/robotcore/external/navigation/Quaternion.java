package org.firstinspires.ftc.robotcore.external.navigation;

/** SIMULATOR stand-in for the FTC SDK class. */
public class Quaternion {
    public float w, x, y, z;
    public long acquisitionTime;

    public Quaternion() {
        this(1, 0, 0, 0, 0);
    }

    public Quaternion(float w, float x, float y, float z, long acquisitionTime) {
        this.w = w;
        this.x = x;
        this.y = y;
        this.z = z;
        this.acquisitionTime = acquisitionTime;
    }
}
