// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.util.Color8Bit;

/** Add your docs here. */

// ENUM for all colors
public enum Color {
    WHITE(255, 255, 255),
    OFF(0, 0, 0),
    RED(255, 0, 0),
    GREEN(0, 255, 0),
    BLUE(0, 0, 255),
    ORANGE(255, 165, 30),
    PURPLE(100, 0, 255),
    YELLOW(210, 255, 0);

    private Color8Bit color;

    private Color(int red, int green, int blue) {

        // using the integers in the enum we make a new Color8Bit
        this.color = new Color8Bit(red, green, blue);
    }

    public Color8Bit getColor8Bit() {
        // using the integers in each enum return the Color8bit
        return color;
    }

    public Color8Bit getColor8BitAlpha(double alphaPercent) {

        // change the intensity of the led color.
        // keeps it between 0 and 100 percent intensity
        alphaPercent = MathUtil.clamp(alphaPercent, 0, 85) / 100;
        int red = (int) (color.red * alphaPercent);
        int green = (int) (color.green * alphaPercent);
        int blue = (int) (color.blue * alphaPercent);

        return new Color8Bit(red, green, blue);
    }
}
