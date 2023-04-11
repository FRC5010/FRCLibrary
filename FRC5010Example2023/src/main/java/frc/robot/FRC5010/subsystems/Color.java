// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.util.Color8Bit;

/** Add your docs here. */
public enum Color {
    WHITE(255, 255, 255),
    BLACK(0, 0, 0),
    RED(255, 0, 0),
    GREEN(0, 255, 0),
    ORANGE(255, 50, 30),
    BLUE(0, 0, 255);

    private Color8Bit color;

    private Color(int red, int green, int blue) {
        this.color = new Color8Bit(red, green, blue);
    }

    public Color8Bit getColor8Bit() {
        return color;
    }

    public Color8Bit getColor8BitAlpha(double alphaPercent) {

        alphaPercent = MathUtil.clamp(alphaPercent, 0, 100) / 100;
        int red = (int) (color.red * alphaPercent);
        int green = (int) (color.green * alphaPercent);
        int blue = (int) (color.blue * alphaPercent);

        return new Color8Bit(red, green, blue);
    }
}
