package ftcsim;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

/** A USB webcam in the robot configuration ("Webcam 1"). The picture itself comes from SimVisionPortal. */
public class SimWebcam implements WebcamName {
    public final String name;

    public SimWebcam(String name) {
        this.name = name;
    }

    @Override
    public String getDeviceName() {
        return "Logitech C270 (simulated) " + name;
    }
}
