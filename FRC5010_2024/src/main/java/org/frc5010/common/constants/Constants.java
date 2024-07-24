package org.frc5010.common.constants;

import com.pathplanner.lib.util.PIDConstants;

public class Constants {

  public static final class AutonConstants {
    public static final PIDConstants TRANSLATION_PID = new PIDConstants(5, 0, .1);
    public static final PIDConstants ANGLE_PID = new PIDConstants(5, 0, .1);
  }
}
