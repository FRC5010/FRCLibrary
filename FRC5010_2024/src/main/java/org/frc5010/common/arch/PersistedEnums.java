// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.arch;

import edu.wpi.first.wpilibj.Preferences;

/** Add your docs here. */
public enum PersistedEnums {
  /** Add constants to the enum */
  EXAMPLE_DOUBLE(0.0, Double.class),
  EXAMPLE_BOOLEAN(true, Boolean.class),
  ROBOT_VISUAL_H(60, Integer.class);

  /**
   * ---------------------------------------------------------------------------------------------
   */
  public Object value;

  public String type;

  private PersistedEnums(Object value, Class<?> type) {
    this.value = value;
    this.type = type.getSimpleName();
    initPersistedConstant();
  }

  public void initPersistedConstant() {
    if (!Preferences.containsKey(name())) {
      switch (type) {
        case "Double":
          {
            Preferences.setDouble(name(), (Double) value);
            break;
          }
        case "Float":
          {
            Preferences.setFloat(name(), (Float) value);
            break;
          }
        case "Long":
          {
            Preferences.setLong(name(), (Long) value);
            break;
          }
        case "Integer":
          {
            Preferences.setInt(name(), (Integer) value);
            break;
          }
        case "String":
          {
            Preferences.setString(name(), (String) value);
            break;
          }
        case "Boolean":
          {
            Preferences.setBoolean(name(), (Boolean) value);
            break;
          }
        default:
          {
            Preferences.setString(name(), (String) value.toString());
            break;
          }
      }
    }
  }

  public Double getDouble() {
    return Preferences.getDouble(name(), (Double) value);
  }

  public Float getFloat(String key) {
    return Preferences.getFloat(name(), (Float) value);
  }

  public Boolean getBoolean() {
    return Preferences.getBoolean(name(), (Boolean) value);
  }

  public Integer getInteger() {
    return Preferences.getInt(name(), (Integer) value);
  }

  public Long getLong() {
    return Preferences.getLong(name(), (Long) value);
  }

  public String getString() {
    return Preferences.getString(name(), (String) value);
  }
}
