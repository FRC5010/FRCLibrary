package frc.robot.crescendo;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public final class Constants {
        public static final class Physical {
                public static final double TRACK_WIDTH_INCHES = 16.8;
                public static final double WHEEL_BASE_INCHES = 21.5;
                public static final Transform3d PIVOT_ORIGIN_OFFSET = new Transform3d(new Translation3d(0.0, 0.0, 0.65),
                                new Rotation3d());
                public static final double SHOOTER_ANGLE_OFFSET = 52.071; // TODO: Measure this accurately on real robot
                public static final double PIVOT_EXIT_POINT_ANGLE_OFFSET = 145.112; // TODO: Measure this accurately
                public static final double PIVOT_SHOOTER_RADIUS = 0.245; // Meters

        }

        public static final class Field {
                public static final Pose3d SPEAKER_POSE_RED = new Pose3d(
                                new Translation3d(Units.inchesToMeters(0), Units.inchesToMeters(323.25 - 218.42), 0.0),
                                new Rotation3d(0, 0, 180));
                public static final Pose3d SPEAKER_POSE_BLUE = new Pose3d(
                                new Translation3d(Units.inchesToMeters(0), Units.inchesToMeters(218.42), 0.0),
                                new Rotation3d());
                private static final Transform3d BLUE_SHOT_OFFSET = new Transform3d(
                                new Translation3d(Units.inchesToMeters(9.055), 0.0, Units.inchesToMeters(80.515)),
                                new Rotation3d());
                private static final Transform3d RED_SHOT_OFFSET = new Transform3d(
                                new Translation3d(Units.inchesToMeters(9.055), 0.0, Units.inchesToMeters(80.515)),
                                new Rotation3d());
                public static Pose3d BLUE_SHOT_POSE = SPEAKER_POSE_BLUE.plus(BLUE_SHOT_OFFSET);
                public static Pose3d RED_SHOT_POSE = SPEAKER_POSE_RED.plus(RED_SHOT_OFFSET);
        }
}
