package com.qualcomm.robotcore.hardware;

import java.util.HashMap;
import java.util.Map;

/**
 * SIMULATOR stand-in for the FTC SDK Gamepad. The simulated driver presses buttons with
 * press("x") (scripted in the scenario); xWasPressed() then returns true once, like the SDK.
 */
public class Gamepad {
    public float left_stick_x, left_stick_y, right_stick_x, right_stick_y, left_trigger, right_trigger;
    public boolean a, b, x, y, dpad_up, dpad_down, dpad_left, dpad_right, left_bumper, right_bumper,
            start, back, guide, left_stick_button, right_stick_button, options, share, ps,
            cross, circle, square, triangle;

    private final Map<String, Boolean> pressed = new HashMap<>();
    private final Map<String, Boolean> released = new HashMap<>();

    /** Simulator: the driver pushes this button down (it stays down until release). */
    public void press(String button) {
        set(button, true);
        pressed.put(button, true);
    }

    /** Simulator: the driver lets go of this button. */
    public void release(String button) {
        set(button, false);
        released.put(button, true);
    }

    private void set(String button, boolean down) {
        switch (button) {
            case "a": a = cross = down; break;
            case "b": b = circle = down; break;
            case "x": x = square = down; break;
            case "y": y = triangle = down; break;
            case "dpad_up": dpad_up = down; break;
            case "dpad_down": dpad_down = down; break;
            case "dpad_left": dpad_left = down; break;
            case "dpad_right": dpad_right = down; break;
            case "left_bumper": left_bumper = down; break;
            case "right_bumper": right_bumper = down; break;
            case "start": start = down; break;
            case "back": back = down; break;
            default: throw new IllegalArgumentException("no gamepad button " + button);
        }
    }

    private boolean take(Map<String, Boolean> m, String button) {
        Boolean v = m.remove(button);
        return v != null && v;
    }

    public boolean aWasPressed() { return take(pressed, "a"); }
    public boolean bWasPressed() { return take(pressed, "b"); }
    public boolean xWasPressed() { return take(pressed, "x"); }
    public boolean yWasPressed() { return take(pressed, "y"); }
    public boolean crossWasPressed() { return take(pressed, "a"); }
    public boolean circleWasPressed() { return take(pressed, "b"); }
    public boolean squareWasPressed() { return take(pressed, "x"); }
    public boolean triangleWasPressed() { return take(pressed, "y"); }
    public boolean dpadUpWasPressed() { return take(pressed, "dpad_up"); }
    public boolean dpadDownWasPressed() { return take(pressed, "dpad_down"); }
    public boolean dpadLeftWasPressed() { return take(pressed, "dpad_left"); }
    public boolean dpadRightWasPressed() { return take(pressed, "dpad_right"); }
    public boolean leftBumperWasPressed() { return take(pressed, "left_bumper"); }
    public boolean rightBumperWasPressed() { return take(pressed, "right_bumper"); }
    public boolean startWasPressed() { return take(pressed, "start"); }
    public boolean backWasPressed() { return take(pressed, "back"); }

    public boolean aWasReleased() { return take(released, "a"); }
    public boolean bWasReleased() { return take(released, "b"); }
    public boolean xWasReleased() { return take(released, "x"); }
    public boolean yWasReleased() { return take(released, "y"); }
    public boolean dpadUpWasReleased() { return take(released, "dpad_up"); }
    public boolean dpadDownWasReleased() { return take(released, "dpad_down"); }
}
