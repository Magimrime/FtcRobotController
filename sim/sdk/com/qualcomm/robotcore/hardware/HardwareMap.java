package com.qualcomm.robotcore.hardware;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * SIMULATOR stand-in for the FTC SDK HardwareMap. Like the real one, get() throws when the name is
 * not in the robot configuration (the OpMode then stops in INIT), tryGet() returns null.
 */
public class HardwareMap {
    private final Map<String, Object> byName = new LinkedHashMap<>();
    private final List<Object> all = new ArrayList<>();

    /** Simulator: put a device in the configuration. */
    public void put(String name, Object device) {
        if (name != null) {
            byName.put(name, device);
        }
        all.add(device);
    }

    public <T> T get(Class<? extends T> classOrInterface, String deviceName) {
        T result = tryGet(classOrInterface, deviceName);
        if (result == null) {
            throw new IllegalArgumentException(String.format("Unable to find a hardware device with name \"%s\" and type %s",
                    deviceName, classOrInterface.getSimpleName()));
        }
        return result;
    }

    @SuppressWarnings("unchecked")
    public <T> T tryGet(Class<? extends T> classOrInterface, String deviceName) {
        ftcsim.SimContext.get().tick(0.0005);
        Object d = byName.get(deviceName);
        if (d != null && classOrInterface.isInstance(d)) {
            return (T) d;
        }
        return null;
    }

    @SuppressWarnings("unchecked")
    public <T> List<T> getAll(Class<? extends T> classOrInterface) {
        List<T> list = new ArrayList<>();
        for (Object d : all) {
            if (classOrInterface.isInstance(d)) {
                list.add((T) d);
            }
        }
        return list;
    }
}
