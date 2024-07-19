// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.arch;

import edu.wpi.first.math.Pair;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/** Add your docs here. */
public class WpiNetworkTableValuesHelper implements WpiHelperInterface {
  protected Map<String, Pair<String, Object>> sendables = new HashMap<>();
  protected Map<String, Pair<String, Object>> persisted = new HashMap<>();
  protected static List<Sendable> registered = new ArrayList<>();

  /**
   * Register a Wpi Helper Interface class
   *
   * @param wpiHelper
   */
  public static void register(Sendable wpiHelper) {
    registered.add(wpiHelper);
  }

  public static void loadRegisteredToNetworkTables() {
    registered.stream().forEach(it -> SmartDashboard.putData(it));
  }
  /**
   * Gets the value of a WPI NT value of the specified type
   *
   * @param key
   * @param type
   * @return
   */
  public Object get(String key, String type) {
    String valueType = this.type(key);
    if (null != valueType) {
      if (type.compareTo(valueType) == 0) {
        return this.get(key);
      } else {
        log_rio(key + " is of type " + valueType + " not of type " + type);
      }
    } else {
      log_rio(key + " was not declared.");
    }
    return null;
  }

  public Double getDouble(String key) {
    return (Double) get(key, Double.class.getSimpleName());
  }

  public Integer getInteger(String key) {
    return (Integer) get(key, Integer.class.getSimpleName());
  }

  public String getString(String key) {
    return (String) get(key, String.class.getSimpleName());
  }

  public Boolean getBoolean(String key) {
    return (Boolean) get(key, Boolean.class.getSimpleName());
  }

  /**
   * Declares a variable on the Display to this class The function must be called during the class
   * constructor before putting the class on Display The overloaded function implementations can be
   * used without needing to specify the type.
   *
   * @param key - String name of the variable being stored. Recommend using a final String constant.
   * @param type - Enumerated type of the values supported by this interface.
   * @param val - The value being stored. apiNote - If the variable has already been declared, an
   *     error will be printed and the value will not be set.
   */
  public void declare(String key, String type, Object val) {
    Pair<String, Object> value = sendables.get(key);
    if (null == value) {
      declare(key, Pair.of(type, val));
    } else {
      if (type.compareTo(value.getFirst()) != 0) {
        log_rio(key + " has already been declared as type " + value.getFirst());
      }
    }
  }

  public void declare(String key, Double val) {
    declare(key, val.getClass().getSimpleName(), val);
  }

  public void declare(String key, Integer val) {
    declare(key, val.getClass().getSimpleName(), val);
  }

  public void declare(String key, String val) {
    declare(key, val.getClass().getSimpleName(), val);
  }

  public void declare(String key, Boolean val) {
    declare(key, val.getClass().getSimpleName(), val);
  }

  public void declareDouble(String key) {
    declare(key, Double.class.getSimpleName(), 0.0);
  }

  public void set(String key, String type, Object val) {
    String valueType = type(key);
    if (null != valueType) {
      if (type.compareTo(valueType) == 0) {
        set(key, val);
      } else {
        log_rio(
            "Error setting " + val + " to " + key + " which is type " + valueType + " not " + type);
      }
    } else {
      log_rio("Error setting " + val + " to " + key + " which has not been declared.");
    }
  }

  public void set(String key, Double val) {
    set(key, val.getClass().getSimpleName(), val);
  }

  public void set(String key, Integer val) {
    set(key, val.getClass().getSimpleName(), val);
  }

  public void set(String key, String val) {
    set(key, val.getClass().getSimpleName(), val);
  }

  public void set(String key, Boolean val) {
    set(key, val.getClass().getSimpleName(), val);
  }

  public Object get(String key) {
    if (Preferences.containsKey(key)) {
      Pair<String, Object> value = persisted.get(key);
      switch (value.getFirst()) {
        case "Double":
          return Double.valueOf(Preferences.getDouble(key, (Double) value.getSecond()));
        case "Integer":
          return Integer.valueOf(Preferences.getInt(key, (Integer) value.getSecond()));
        case "String":
          return Preferences.getString(key, (String) value.getSecond());
        case "Boolean":
          return Boolean.valueOf(Preferences.getBoolean(key, (Boolean) value.getSecond()));
      }
    } else if (sendables.containsKey(key)) {
      return sendables.get(key).getSecond();
    }
    return null;
  }

  public String type(String key) {
    if (Preferences.containsKey(key)) {
      Pair<String, Object> value = persisted.get(key);
      return value.getFirst();
    } else if (sendables.containsKey(key)) {
      return sendables.get(key).getFirst();
    }
    return null;
  }

  public void declare(String key, Pair<String, Object> val) {
    sendables.put(key, val);
  }

  public void persist(String key, Integer val) {
    persist(key, Integer.class.getSimpleName(), val);
  }

  public void persist(String key, Double val) {
    persist(key, Double.class.getSimpleName(), val);
  }

  public void persist(String key, String val) {
    persist(key, String.class.getSimpleName(), val);
  }

  public void persist(String key, Boolean val) {
    persist(key, Boolean.class.getSimpleName(), val);
  }

  public void persist(String key, String type, Object val) {
    Pair<String, Object> value = persisted.get(key);
    if (null == value) {
      persist(key, Pair.of(type, val));
    } else {
      if (type.compareTo(value.getFirst()) != 0) {
        log_rio(key + " has already been persisted as type " + value.getFirst());
      } else {
        log_rio("WARNING: " + key + " has already been persisted.");
      }
    }
  }

  public void persist(String key, Pair<String, Object> val) {
    if (!Preferences.containsKey(key)) {
      switch (val.getFirst()) {
        case "Double":
          {
            Preferences.setDouble(key, (Double) val.getSecond());
            break;
          }
        case "Integer":
          {
            Preferences.setInt(key, (Integer) val.getSecond());
            break;
          }
        case "String":
          {
            Preferences.setString(key, (String) val.getSecond());
            break;
          }
        case "Boolean":
          {
            Preferences.setBoolean(key, (Boolean) val.getSecond());
            break;
          }
      }
    }
  }

  public void set(String key, Object val) {
    if (Preferences.containsKey(key)) {
      Pair<String, Object> value = persisted.get(key);
      switch (value.getFirst()) {
        case "Double":
          Preferences.setDouble(key, (Double) val);
        case "Integer":
          Preferences.setInt(key, (Integer) val);
        case "String":
          Preferences.setString(key, (String) val);
        case "Boolean":
          Preferences.setBoolean(key, (Boolean) val);
      }
      persisted.put(key, Pair.of(value.getFirst(), val));
    } else if (sendables.containsKey(key)) {
      sendables.put(key, Pair.of(sendables.get(key).getFirst(), val));
    }
  }

  public void addToTab(ShuffleboardTab tab, String key) {
    if (Preferences.containsKey(key)) {
      Pair<String, Object> value = persisted.get(key);
      switch (value.getFirst()) {
        case "Double":
          tab.addDouble(
              key, () -> Double.valueOf(Preferences.getDouble(key, (Double) value.getSecond())));
        case "Integer":
          tab.addInteger(
              key, () -> Integer.valueOf(Preferences.getInt(key, (Integer) value.getSecond())));
        case "String":
          tab.addString(key, () -> Preferences.getString(key, (String) value.getSecond()));
        case "Boolean":
          tab.addBoolean(
              key, () -> Boolean.valueOf(Preferences.getBoolean(key, (Boolean) value.getSecond())));
      }
    } else if (sendables.containsKey(key)) {
      Pair<String, Object> value = persisted.get(key);
      switch (value.getFirst()) {
        case "Double":
          tab.addDouble(key, () -> (Double) (sendables.get(key).getSecond()));
        case "Integer":
          tab.addInteger(key, () -> (Integer) (sendables.get(key).getSecond()));
        case "String":
          tab.addString(key, () -> (String) (sendables.get(key).getSecond()));
        case "Boolean":
          tab.addBoolean(key, () -> (Boolean) (sendables.get(key).getSecond()));
      }
    } else {
      System.err.println("Unable to add " + key + " to the tab " + tab);
    }
  }

  public void addToTabList(ShuffleboardTab tab, String list, String key) {
    ShuffleboardLayout layout = tab.getLayout(list);
    if (Preferences.containsKey(key)) {
      Pair<String, Object> value = persisted.get(key);
      switch (value.getFirst()) {
        case "Double":
          layout.addDouble(
              key, () -> Double.valueOf(Preferences.getDouble(key, (Double) value.getSecond())));
        case "Integer":
          layout.addInteger(
              key, () -> Integer.valueOf(Preferences.getInt(key, (Integer) value.getSecond())));
        case "String":
          layout.addString(key, () -> Preferences.getString(key, (String) value.getSecond()));
        case "Boolean":
          layout.addBoolean(
              key, () -> Boolean.valueOf(Preferences.getBoolean(key, (Boolean) value.getSecond())));
      }
    } else if (sendables.containsKey(key)) {
      Pair<String, Object> value = persisted.get(key);
      switch (value.getFirst()) {
        case "Double":
          layout.addDouble(key, () -> (Double) (sendables.get(key).getSecond()));
        case "Integer":
          layout.addInteger(key, () -> (Integer) (sendables.get(key).getSecond()));
        case "String":
          layout.addString(key, () -> (String) (sendables.get(key).getSecond()));
        case "Boolean":
          layout.addBoolean(key, () -> (Boolean) (sendables.get(key).getSecond()));
      }
    } else {
      System.err.println("Unable to add " + key + " to the tab " + tab + " list " + list);
    }
  }

  public void initSendables(SendableBuilder builder, String loggingPrefix) {
    builder.setSmartDashboardType(loggingPrefix);
    for (String key : sendables.keySet()) {
      Pair<String, Object> value = sendables.get(key);
      switch (value.getFirst()) {
        case "Integer":
          builder.addIntegerProperty(
              key,
              () -> (Integer) sendables.get(key).getSecond(),
              (val) -> sendables.put(key, Pair.of(value.getFirst(), val)));
          break;
        case "Double":
          builder.addDoubleProperty(
              key,
              () -> (Double) sendables.get(key).getSecond(),
              (val) -> sendables.put(key, Pair.of(value.getFirst(), val)));
          break;
        case "String":
          builder.addStringProperty(
              key,
              () -> (String) sendables.get(key).getSecond(),
              (val) -> sendables.put(key, Pair.of(value.getFirst(), val)));
          break;
        case "Boolean":
          builder.addBooleanProperty(
              key,
              () -> (Boolean) sendables.get(key).getSecond(),
              (val) -> sendables.put(key, Pair.of(value.getFirst(), val)));
          break;
      }
    }
  }
}
