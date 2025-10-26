package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.firstinspires.ftc.robotcore.external.JavaUtil;

public class sensors {

    private NormalizedColorSensor colorSensor;

    /**
     * Initializes the color sensor.
     */
    public void init(HardwareMap hardwareMap) {
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "i2c2");
    }

    /**
     * Returns hue and light level from the sensor.
     * @return double[] {hue (degrees), light (0–1)}
     */
    public double[] getHueAndLight() {
        if (colorSensor == null) return new double[] {0, 0};

        NormalizedRGBA color = colorSensor.getNormalizedColors();
        float hue = JavaUtil.colorToHue(color.toColor());
        double light = ((OpticalDistanceSensor) colorSensor).getLightDetected();

        return new double[] {hue, light};
    }

    /**
     * Returns RGB and light level from the sensor.
     * @return double[] {r, g, b, light (0–1)}
     */
    public double[] getRGBAndLight() {
        if (colorSensor == null) return new double[] {0, 0, 0, 0};

        NormalizedRGBA color = colorSensor.getNormalizedColors();
        double light = ((OpticalDistanceSensor) colorSensor).getLightDetected();

        return new double[] {color.red, color.green, color.blue, light};
    }

    /**
     * Gets only the light detected value (0–1).
     */
    public double getLightDetected() {
        if (colorSensor == null) return 0;
        return ((OpticalDistanceSensor) colorSensor).getLightDetected();
    }

    /**
     * Gets only the hue value (0–360).
     */
    public float getHue() {
        if (colorSensor == null) return 0;
        NormalizedRGBA color = colorSensor.getNormalizedColors();
        return JavaUtil.colorToHue(color.toColor());
    }
}
