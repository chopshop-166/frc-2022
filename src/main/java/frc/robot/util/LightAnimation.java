package frc.robot.util;

import java.io.File;

import org.json.JSONObject;

import edu.wpi.first.wpilibj.util.Color;

public class LightAnimation {

    Color frames[];

    public LightAnimation(String filename) {
        JSONObject json = new JSONObject(new File(filename));
    }
}
