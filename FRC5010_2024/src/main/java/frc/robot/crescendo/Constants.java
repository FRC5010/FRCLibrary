package frc.robot.crescendo;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public final class Constants {
    public static final class Physical {
        public static final double TRACK_WIDTH_INCHES = 16.8;
        public static final double WHEEL_BASE_INCHES = 21.5;
        public static final Transform3d PIVOT_ORIGIN_OFFSET = new Transform3d(new Translation3d(0.0, 0.0, 0.0),
                new Rotation3d());

    }

    public static final class Field {
        public static final Pose3d SPEAKER_POSE = new Pose3d(new Translation3d(Units.inchesToMeters(652.73), Units.inchesToMeters(218.42), 0.0),
                new Rotation3d());
        private static final Transform3d SHOT_OFFSET = new Transform3d(new Translation3d(0.0, Units.inchesToMeters(-9.055), Units.inchesToMeters(80.515)),
                new Rotation3d());
        public static final Pose3d SHOT_POSE = SPEAKER_POSE.plus(SHOT_OFFSET);
    }
}