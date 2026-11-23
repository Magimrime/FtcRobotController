package org.firstinspires.ftc.robotcore.internal.system;

import java.io.File;

/**
 * SIMULATOR stand-in for the FTC SDK class. The robot's folders point into the simulator's own
 * scratch folder (sim/out/robot-data unless the system property ftcsim.robotData says otherwise),
 * so an OpMode that writes a log on the Control Hub writes it there instead; the runner moves it
 * next to that match's results.
 */
public class AppUtil {
    public static final File FIRST_FOLDER = new File(System.getProperty("ftcsim.robotData", "sim/out/robot-data"));
    public static final File ROBOT_DATA_DIR = FIRST_FOLDER;
    public static final File ROBOT_SETTINGS = FIRST_FOLDER;
    public static final File MATCH_LOG_FOLDER = FIRST_FOLDER;

    private static final AppUtil INSTANCE = new AppUtil();

    public static AppUtil getInstance() {
        return INSTANCE;
    }

    public static File getDefContext() {
        return FIRST_FOLDER;
    }
}
