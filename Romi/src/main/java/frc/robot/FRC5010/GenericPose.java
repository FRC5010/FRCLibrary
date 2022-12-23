// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public abstract class GenericPose extends SubsystemBase {
    public abstract void resetEncoders();
    
      public abstract double getLeftDistance();
    
      public abstract double getRightDistance();
    
      public abstract double getLeftEncoderRate();
    
      public abstract double getRightEncoderRate();
      
      /**
       * The acceleration in the X-axis.
       *
       * @return The acceleration of the Romi along the X-axis in Gs
       */
      public abstract double getAccelX();
    
      /**
       * The acceleration in the Y-axis.
       *
       * @return The acceleration of the Romi along the Y-axis in Gs
       */
      public abstract double getAccelY();
    
      /**
       * The acceleration in the Z-axis.
       *
       * @return The acceleration of the Romi along the Z-axis in Gs
       */
      public abstract double getAccelZ();
    
      /**
       * Current angle of the Romi around the X-axis.
       *
       * @return The current angle of the Romi in degrees
       */
      public abstract double getGyroAngleX();
    
      /**
       * Current angle of the Romi around the Y-axis.
       *
       * @return The current angle of the Romi in degrees
       */
      public abstract double getGyroAngleY();
    
      /**
       * Current angle of the Romi around the Z-axis.
       *
       * @return The current angle of the Romi in degrees
       */
      public abstract double getGyroAngleZ();
    
      public Rotation2d getGyroRotation2d() {
        return new Rotation3d(getGyroAngleX(), getGyroAngleY(), getGyroAngleZ()).toRotation2d();
      }
    
      /** Reset the gyro. */
      public abstract void resetGyro();
}
