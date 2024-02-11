package frc.robot.crescendo;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public final class Constants {
    public static final class Physical {
        public static final double TRACK_WIDTH_INCHES = 16.8;
        public static final double WHEEL_BASE_INCHES = 21.5;
        public static final Transform3d PIVOT_ORIGIN_OFFSET = new Transform3d(new Translation3d(0.0, 0.0, 0.0),
                new Rotation3d());

    }

    public static final class Field {
        public static final Transform3d SPEAKER_POSE = new Transform3d(new Translation3d(652.73, 218.42, 0.0),
                new Rotation3d());
        private static final Transform3d SHOT_OFFSET = new Transform3d(new Translation3d(0.0, -9.055, 80.515),
                new Rotation3d());
        public static final Transform3d SHOT_POSE = SPEAKER_POSE.plus(SHOT_OFFSET);
    }
}
