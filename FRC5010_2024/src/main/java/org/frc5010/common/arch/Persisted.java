// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.arch;

import edu.wpi.first.wpilibj.Preferences;
import org.frc5010.common.constants.GenericPersisted;

/**
 * Persisted - handles boilerplate code for persisted constant values
 *
 * <ul>
 *   <li><b>Example</b>
 *   <li>First, set the name as a 'public static final String' in a Constants.java file.
 *       <ul>
 *         <li><code>public static final String varName = "varName";</code>
 *       </ul>
 *   <li>Then, in your initializing class, use:
 *       <ul>
 *         <li><code>
 *             private static Persisted&lt;Integer&gt; varName = new Persisted&lt;&gt;(Constants.varName, 60);
 *             </code>
 *       </ul>
 *   <li>Next, if you need the value somewhere else without knowing the value, just tell it the type
 *       <ul>
 *         <li><code>
 *             private static Persisted&lt;Integer&gt; varName = new Persisted&lt;&gt;(Constants.varName, Integer.class);
 *             </code>
 *       </ul>
 *   <li>Finally, whereever you need the value in your class, use:
 *       <ul>
 *         <li><code>varName.getInteger();</code>
 *       </ul>
 * </ul>
 */
public class Persisted<T> extends GenericPersisted {
  protected T defaultValue;

  /**
   * Persisted constructor used for initiailizing with default value
   *
   * @param name Ex: <code>static final String variableName = "variableName";</code>
   * @param defaultValue Initial value to set if not already stored
   */
  public Persisted(String name, T defaultValue) {
    super(name, defaultValue.getClass().getSimpleName());
    this.defaultValue = defaultValue;
    if (!Preferences.containsKey(name)) {
      init();
    } else {
      this.defaultValue = initGet();
    }
  }

  /**
   * Persisted constructor used when only retrieving values in additional classes The type needs to
   * be specified like: Integer.class
   *
   * @param name Ex: <code>static final String variableName = "variableName";</code>
   * @param type Ex: <code>Double.class</code>
   */
  public Persisted(String name, Class<T> type) {
    super(name, type.getSimpleName());
    this.defaultValue = initGet();
  }

  /**
   * getDefaultValue
   *
   * @return the internally stored default value
   */
  public T getDefaultValue() {
    return defaultValue;
  }

  public Double getDouble() {
    return Preferences.getDouble(getName(), (Double) defaultValue);
  }

  public Float getFloat() {
    return Preferences.getFloat(getName(), (Float) defaultValue);
  }

  public Long getLong() {
    return Preferences.getLong(getName(), (Long) defaultValue);
  }

  public Integer getInteger() {
    return Preferences.getInt(getName(), (Integer) defaultValue);
  }

  public Boolean getBoolean() {
    return Preferences.getBoolean(getName(), (Boolean) defaultValue);
  }

  public String getString() {
    return Preferences.getString(getName(), (String) defaultValue);
  }

  public void set(T value) {
    this.defaultValue = value;
    init();
  }

  public T get() {
    return initGet();
  }

  private void init() {
    switch (getType()) {
      case "Double":
        {
          Preferences.initDouble(getName(), (Double) defaultValue);
          break;
        }
      case "Float":
        {
          Preferences.initFloat(getName(), (Float) defaultValue);
          break;
        }
      case "Long":
        {
          Preferences.initLong(getName(), (Long) defaultValue);
          break;
        }
      case "Integer":
        {
          Preferences.initInt(getName(), (Integer) defaultValue);
          break;
        }
      case "String":
        {
          Preferences.initString(getName(), (String) defaultValue);
          break;
        }
      case "Boolean":
        {
          Preferences.initBoolean(getName(), (Boolean) defaultValue);
          break;
        }
      default:
        {
          Preferences.initString(getName(), defaultValue.toString());
          break;
        }
    }
  }

  @SuppressWarnings("unchecked")
  private T initGet() {
    switch (getType()) {
      case "Double":
        {
          return (T) doubleVal(name);
        }
      case "Float":
        {
          return (T) floatVal(name);
        }
      case "Long":
        {
          return (T) longVal(name);
        }
      case "Integer":
        {
          return (T) integerVal(name);
        }
      case "String":
        {
          return (T) stringVal(name);
        }
      case "Boolean":
        {
          return (T) booleanVal(name);
        }
      default:
        {
          return (T) stringVal(name);
        }
    }
  }

  /** Gets the value but doesn't display it in NetworkTables */
  public static Double doubleVal(String name) {
    return Preferences.getDouble(name, 0);
  }

  /** Gets the value but doesn't display it in NetworkTables */
  public static Float floatVal(String name) {
    return Preferences.getFloat(name, 0);
  }

  /** Gets the value but doesn't display it in NetworkTables */
  public static Long longVal(String name) {
    return Preferences.getLong(name, 0);
  }

  /** Gets the value but doesn't display it in NetworkTables */
  public static Integer integerVal(String name) {
    return Preferences.getInt(name, 0);
  }

  /** Gets the value but doesn't display it in NetworkTables */
  public static Boolean booleanVal(String name) {
    return Preferences.getBoolean(name, false);
  }

  /** Gets the value but doesn't display it in NetworkTables */
  public static String stringVal(String name) {
    return Preferences.getString(name, "");
  }
}
