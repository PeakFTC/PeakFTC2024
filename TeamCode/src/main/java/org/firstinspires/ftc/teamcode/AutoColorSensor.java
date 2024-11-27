package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class AutoColorSensor {
    private ColorSensor colorSensor;
    int gain;
    NormalizedRGBA normalizedColors;
    int color;
    float hue;
    float saturation;
    float value;
    Telemetry telemetry;
    HardwareMap hardwareMap;

    public enum DetectedColor{
        RED,
        BLUE,
        NONE
    }
    DetectedColor detectedColor = DetectedColor.NONE;

    public AutoColorSensor(HardwareMap hwMAP, Telemetry telemetry)
    {
        this.hardwareMap = hwMAP;
        this.telemetry = telemetry;
        gain = 2;
    }
    public DetectedColor getDetetedColor()
    {
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        ((NormalizedColorSensor) colorSensor).setGain(gain);

        normalizedColors = ((NormalizedColorSensor) colorSensor).getNormalizedColors();
        color = normalizedColors.toColor();
        hue = JavaUtil.colorToHue(color);
        saturation = JavaUtil.colorToSaturation(color);
        value = JavaUtil.colorToValue(color);

        // Show the color on the Robot Controller screen.
        JavaUtil.showColor(hardwareMap.appContext, color);
        // Use hue to determine if it's red, green, blue, etc..
        if (hue < 30) {
            telemetry.addData("Color", "Red");
            detectedColor=DetectedColor.RED;
        } else if (hue < 60) {
            telemetry.addData("Color", "Orange");
            detectedColor=DetectedColor.NONE;
        } else if (hue < 90) {
            telemetry.addData("Color", "Yellow");
            detectedColor=DetectedColor.NONE;
        } else if (hue < 150) {
            telemetry.addData("Color", "Green");
            detectedColor=DetectedColor.NONE;
        } else if (hue < 225) {
            telemetry.addData("Color", "Blue");
            detectedColor=DetectedColor.BLUE;
        } else if (hue < 350) {
            telemetry.addData("Color", "purple");
            detectedColor=DetectedColor.NONE;
        } else {
            telemetry.addData("Color", "Red");
            detectedColor=DetectedColor.NONE;
        }

        return detectedColor;
    }

}
