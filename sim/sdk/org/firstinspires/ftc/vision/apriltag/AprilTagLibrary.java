package org.firstinspires.ftc.vision.apriltag;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;

import java.util.LinkedHashMap;
import java.util.Map;

/** SIMULATOR stand-in for the FTC SDK class: the tags a processor reports (others are ignored). */
public class AprilTagLibrary {
    private final Map<Integer, AprilTagMetadata> tags;

    private AprilTagLibrary(Map<Integer, AprilTagMetadata> tags) {
        this.tags = tags;
    }

    public AprilTagMetadata[] getAllTags() {
        return tags.values().toArray(new AprilTagMetadata[0]);
    }

    public AprilTagMetadata lookupTag(int id) {
        return tags.get(id);
    }

    public static class Builder {
        private final Map<Integer, AprilTagMetadata> tags = new LinkedHashMap<>();
        private boolean allowOverwrite = false;

        public Builder setAllowOverwrite(boolean allow) {
            allowOverwrite = allow;
            return this;
        }

        public Builder addTag(AprilTagMetadata tag) {
            if (tags.containsKey(tag.id) && !allowOverwrite) {
                throw new IllegalArgumentException("You're trying to add a tag with an ID that already exists: " + tag.id);
            }
            tags.put(tag.id, tag);
            return this;
        }

        public Builder addTag(int id, String name, double size, VectorF fieldPosition, DistanceUnit distanceUnit, Quaternion fieldOrientation) {
            return addTag(new AprilTagMetadata(id, name, size, fieldPosition, distanceUnit, fieldOrientation));
        }

        public Builder addTag(int id, String name, double size, DistanceUnit distanceUnit) {
            return addTag(new AprilTagMetadata(id, name, size, distanceUnit));
        }

        public Builder addTags(AprilTagLibrary library) {
            for (AprilTagMetadata t : library.getAllTags()) addTag(t);
            return this;
        }

        public Builder addLibrary(AprilTagLibrary library) {
            return addTags(library);
        }

        public AprilTagLibrary build() {
            return new AprilTagLibrary(new LinkedHashMap<>(tags));
        }
    }
}
