package ftcsim;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

/** The Driver Station screen: keeps the last frame the OpMode sent, and hands each frame to the context. */
public class SimTelemetry implements Telemetry {
    private final List<String> building = new ArrayList<>();
    private List<String> lastFrame = new ArrayList<>();
    private final SimContext ctx;
    private String captionSeparator = " : ";
    private boolean autoClear = true;
    private int msInterval = 250;

    public SimTelemetry(SimContext ctx) {
        this.ctx = ctx;
    }

    public List<String> lastFrame() {
        return lastFrame;
    }

    private class SimItem implements Item {
        final int index;
        String caption;

        SimItem(String caption, String value) {
            this.caption = caption;
            index = building.size();
            building.add(caption + captionSeparator + value);
        }

        @Override
        public Item setCaption(String caption) {
            this.caption = caption;
            return this;
        }

        @Override
        public String getCaption() {
            return caption;
        }

        @Override
        public Item setValue(Object value) {
            if (index < building.size()) building.set(index, caption + captionSeparator + value);
            return this;
        }

        @Override
        public Item setValue(String format, Object... args) {
            return setValue(String.format(format, args));
        }

        @Override
        public Item addData(String caption, Object value) {
            return SimTelemetry.this.addData(caption, value);
        }

        @Override
        public Item addData(String caption, String format, Object... args) {
            return SimTelemetry.this.addData(caption, format, args);
        }
    }

    private class SimLine implements Line {
        @Override
        public Item addData(String caption, Object value) {
            return SimTelemetry.this.addData(caption, value);
        }

        @Override
        public Item addData(String caption, String format, Object... args) {
            return SimTelemetry.this.addData(caption, format, args);
        }
    }

    @Override
    public Item addData(String caption, String format, Object... args) {
        return new SimItem(caption, String.format(format, args));
    }

    @Override
    public Item addData(String caption, Object value) {
        return new SimItem(caption, String.valueOf(value));
    }

    @Override
    public Line addLine() {
        building.add("");
        return new SimLine();
    }

    @Override
    public Line addLine(String lineCaption) {
        building.add(lineCaption);
        return new SimLine();
    }

    @Override
    public boolean removeItem(Item item) {
        return false;
    }

    @Override
    public boolean removeLine(Line line) {
        return false;
    }

    @Override
    public void clear() {
        building.clear();
    }

    @Override
    public void clearAll() {
        building.clear();
        lastFrame = new ArrayList<>();
    }

    @Override
    public boolean update() {
        ctx.tick(ctx.scenario.costTelemetry);
        lastFrame = new ArrayList<>(building);
        if (autoClear) building.clear();
        ctx.onScreen(lastFrame);
        return true;
    }

    @Override
    public void setDisplayFormat(DisplayFormat displayFormat) {
    }

    @Override
    public int getMsTransmissionInterval() {
        return msInterval;
    }

    @Override
    public void setMsTransmissionInterval(int msTransmissionInterval) {
        msInterval = msTransmissionInterval;
    }

    @Override
    public boolean isAutoClear() {
        return autoClear;
    }

    @Override
    public void setAutoClear(boolean autoClear) {
        this.autoClear = autoClear;
    }

    @Override
    public String getItemSeparator() {
        return " | ";
    }

    @Override
    public void setItemSeparator(String itemSeparator) {
    }

    @Override
    public String getCaptionValueSeparator() {
        return captionSeparator;
    }

    @Override
    public void setCaptionValueSeparator(String captionValueSeparator) {
        this.captionSeparator = captionValueSeparator;
    }
}
