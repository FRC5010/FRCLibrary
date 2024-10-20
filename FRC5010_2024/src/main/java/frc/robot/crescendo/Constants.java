package frc.robot.crescendo;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
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
                public static final double SHOOTER_CAM_ANGLE_OFFSET = 64 + 5.630592; // TODO: Measure this accurately on real robot
				public static final double SHOOTER_CAM_LENS_TO_SPLINE = 4.752489;
				public static final double SPLINE_HEIGHT = 20.528445;
				public static final double SPLINE_X = 11.4375;
                public static final double PIVOT_EXIT_POINT_ANGLE_OFFSET = 145.112; // TODO: Measure this accurately
                public static final double PIVOT_SHOOTER_RADIUS = 0.245; // Meters

                public static final double BOTTOM_SHOOTING_SPEED = 4250;
                public static final double TOP_SHOOTING_SPEED = 4250;

                public static final double MANUAL_SHOOTING_SPEED = 4800;
                public static final double AMP_SHOOTING_SPEED = 2000;

                public static final double SHUTTLE_SPEED = 2000;
                public static final double SHUTTLE_SPEED_HIGH = 3000;

                public static final double SUBWOOFER_SHOT = 2500;

        }

        public static final class Field {
                public static final Pose3d SPEAKER_POSE_RED = new Pose3d(
                                new Translation3d(Units.inchesToMeters(652.73), Units.inchesToMeters(218.42), 0.0),
                                new Rotation3d(0, 0, 180));
                public static final Pose3d SPEAKER_POSE_BLUE = new Pose3d(
                                new Translation3d(Units.inchesToMeters(0), Units.inchesToMeters(218.42), 0.0),
                                new Rotation3d());
                private static final Transform3d BLUE_SHOT_OFFSET = new Transform3d(
                                new Translation3d(Units.inchesToMeters(9.055), 0.0, Units.inchesToMeters(80.515)),
                                new Rotation3d());
                private static final Transform3d RED_SHOT_OFFSET = new Transform3d(
                                new Translation3d(Units.inchesToMeters(-9.055), 0.0, Units.inchesToMeters(80.515)), // TEMP COMP UPDATE
                                new Rotation3d());
                // Old forward projected aiming position Units.inchesToMeters(9.055)                
                public static Pose3d BLUE_SHOT_POSE = new Pose3d(
                                new Translation3d(0, Units.inchesToMeters(218.42), 0.0),
                                new Rotation3d());
                // Old forward projected aiming position Units.inchesToMeters(652.73 - 9.055)
                public static Pose3d RED_SHOT_POSE = new Pose3d(
                                new Translation3d(Units.inchesToMeters(652.73), Units.inchesToMeters(218.42), 0.0),
                                new Rotation3d(0, 0, 180));
                public static Pose3d BLUE_SHUTTLE_POSE = new Pose3d(
                                new Translation3d(0, Units.inchesToMeters(295.0), 0.0),
                                new Rotation3d());
                public static Pose3d RED_SHUTTLE_POSE = new Pose3d(
                                new Translation3d(Units.inchesToMeters(652.73), Units.inchesToMeters(295.0), 0.0),
                                new Rotation3d(0, 0, 180));
                


                // Auto Shot Positions (Blue Alliance)
                public static final Pose2d CENTER_STAGE_SHOT_LONG = new Pose2d(4.50, 4.70, new Rotation2d());
        }
}
