package frc.robot.crescendo;

public final class Constants {
    public static final class Physical {
        public static final double TRACK_WIDTH_INCHES = 16.8;
        public static final double WHEEL_BASE_INCHES = 21.5;
        
    }
    public static final class Field {
        public static final double SPEAKER_X = 652.73;
        public static final double SPEAKER_Y = 218.42;
        public static final double SPEAKER_Z = 0.0;

        public static final double SHOT_OFFSET_X = 0.0;
        public static final double SHOT_OFFSET_Y = -9.055;
        public static final double SHOT_OFFSET_Z = 80.515;

        public static final double SHOT_X = SPEAKER_X + SHOT_OFFSET_X;
        public static final double SHOT_Y = SPEAKER_X + SHOT_OFFSET_Y;
        public static final double SHOT_Z = SPEAKER_X + SHOT_OFFSET_Z;

    }
}
