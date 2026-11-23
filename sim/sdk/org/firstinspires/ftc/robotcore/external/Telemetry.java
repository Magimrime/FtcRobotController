package org.firstinspires.ftc.robotcore.external;

/** SIMULATOR stand-in for the FTC SDK interface (the parts the team's OpModes use). */
public interface Telemetry {
    enum DisplayFormat { CLASSIC, MONOSPACE, HTML }

    interface Item {
        Item setCaption(String caption);

        String getCaption();

        Item setValue(Object value);

        Item setValue(String format, Object... args);

        Item addData(String caption, Object value);

        Item addData(String caption, String format, Object... args);
    }

    interface Line {
        Item addData(String caption, Object value);

        Item addData(String caption, String format, Object... args);
    }

    Item addData(String caption, String format, Object... args);

    Item addData(String caption, Object value);

    Line addLine();

    Line addLine(String lineCaption);

    boolean removeItem(Item item);

    boolean removeLine(Line line);

    void clear();

    void clearAll();

    boolean update();

    void setDisplayFormat(DisplayFormat displayFormat);

    int getMsTransmissionInterval();

    void setMsTransmissionInterval(int msTransmissionInterval);

    boolean isAutoClear();

    void setAutoClear(boolean autoClear);

    String getItemSeparator();

    void setItemSeparator(String itemSeparator);

    String getCaptionValueSeparator();

    void setCaptionValueSeparator(String captionValueSeparator);
}
