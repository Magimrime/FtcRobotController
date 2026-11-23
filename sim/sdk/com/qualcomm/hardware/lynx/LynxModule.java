package com.qualcomm.hardware.lynx;

import com.qualcomm.robotcore.hardware.HardwareDevice;

import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Set;

/**
 * SIMULATOR stand-in for a REV hub. It models what bulk caching does to timing and values:
 * OFF = every encoder read is its own command (fresh value), AUTO = one bulk read gives every
 * channel's value at that moment and serves them until a channel is read twice, MANUAL = one bulk
 * read until clearBulkCache().
 */
public class LynxModule implements HardwareDevice {
    public enum BulkCachingMode { OFF, MANUAL, AUTO }

    private final String name;
    private BulkCachingMode mode = BulkCachingMode.OFF;
    private final Map<Object, Runnable> channels = new LinkedHashMap<>();
    private final Set<Object> readSinceBulk = new HashSet<>();
    private boolean haveBulk = false;

    public LynxModule(String name) {
        this.name = name;
    }

    public void setBulkCachingMode(BulkCachingMode mode) {
        this.mode = mode;
        clearBulkCache();
    }

    public BulkCachingMode getBulkCachingMode() {
        return mode;
    }

    public void clearBulkCache() {
        haveBulk = false;
        readSinceBulk.clear();
    }

    /** Simulator: a device on this hub whose value is read through bulk reads. */
    public void simRegister(Object channel, Runnable snapshot) {
        channels.put(channel, snapshot);
    }

    /** Simulator: the code reads this channel. Costs hub time and refreshes the cached value(s) as the real hub would. */
    public void simRead(Object channel) {
        ftcsim.SimContext c = ftcsim.SimContext.get();
        switch (mode) {
            case OFF:
                c.tick(c.scenario.costHubCommand);
                channels.get(channel).run();
                return;
            case MANUAL:
                if (!haveBulk) {
                    bulk(c);
                }
                return;
            default:  // AUTO
                if (!haveBulk || readSinceBulk.contains(channel)) {
                    bulk(c);
                }
                readSinceBulk.add(channel);
        }
    }

    private void bulk(ftcsim.SimContext c) {
        c.tick(c.scenario.costBulkRead);
        for (Runnable r : channels.values()) {
            r.run();
        }
        haveBulk = true;
        readSinceBulk.clear();
    }

    @Override
    public String getDeviceName() {
        return name;
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Lynx;
    }
}
