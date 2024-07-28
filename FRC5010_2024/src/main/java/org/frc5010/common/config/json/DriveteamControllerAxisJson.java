// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.config.json;


/** A simple Java class used for storing configuration data for a single axis of a controller. 
 * @apiNote channel: The channel number of the axis on the controller.
 * @apiNote deadband: The mininal value of the axis to register motion.
 * @apiNote invert: Whether the axis should be inverted.
 * @apiNote scale: The scaling factor for the axis.
 * @apiNote curvePower: The mathematical power to apply to the axis to create a curved response.
 * @apiNote limit: The absolute maximum limit value for the axis.
 * @apiNote rate: Optional. The slew rate for the axis.
*/
public class DriveteamControllerAxisJson {
	public int channel;
	public double deadband = 0.0;
	public boolean invert = false;
	public double scale = 1.0;
	public double curvePower = 1.0;
	public double limit = 1.0;
	public double rate = 0.0;
}
