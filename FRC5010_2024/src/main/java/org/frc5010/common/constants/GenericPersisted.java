// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.constants;

/** Add your docs here. */
public class GenericPersisted {
  protected String type;
  protected final String name;

  public GenericPersisted(String name) {
    this.name = name;
  }

  public GenericPersisted(String name, String type) {
    this.name = name;
    this.type = type;
  }

  public void setType(String type) {
    this.type = type;
  }

  public String getType() {
    return type;
  }

  public String getName() {
    return name;
  }
}
