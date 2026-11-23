package org.firstinspires.ftc.robotcore.external.matrices;

/** SIMULATOR stand-in for the FTC SDK class: a small row-major float matrix. */
public class MatrixF {
    protected final int rows, cols;
    protected final float[] data;

    public MatrixF(int rows, int cols) {
        this.rows = rows;
        this.cols = cols;
        this.data = new float[rows * cols];
    }

    public int numRows() {
        return rows;
    }

    public int numCols() {
        return cols;
    }

    public float get(int row, int col) {
        return data[row * cols + col];
    }

    public void put(int row, int col, float value) {
        data[row * cols + col] = value;
    }
}
