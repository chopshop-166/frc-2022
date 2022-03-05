package frc.robot.util;

import java.io.File;

import org.json.JSONArray;
import org.json.JSONObject;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.util.Color;

public class LightAnimation {

    private Color frames[][];

    public LightAnimation(String filename) {
        int numLeds, numFrames;
        String absolutePath = Filesystem.getDeployDirectory().getAbsolutePath();
        JSONObject json = new JSONObject(new File(absolutePath + "/" + filename));

        JSONArray frameArray = json.getJSONArray("leds");
        numFrames = frameArray.length();
        numLeds = frameArray.getJSONArray(0).length();
        frames = new Color[numFrames][numLeds];
        for (int j = 0; j < numFrames; j++) {
            JSONArray ledArray = frameArray.getJSONArray(j);
            for (int i = 0; i < numLeds; i++) {
                JSONObject colorObject = ledArray.getJSONObject(i);
                frames[j][i] = new Color(colorObject.getInt("r") / 255.0,
                        colorObject.getInt("g") / 255.0,
                        colorObject.getInt("b") / 255.0);
            }
        }
    }

    public Color getColor(int frame, int index) {
        return frames[frame][index];
    }

}
