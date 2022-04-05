package frc.robot.util;

import java.nio.file.Files;
import java.nio.file.Path;

import org.json.JSONObject;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

public class LightAnimation {

    private Color frames[][];
    private String name;

    /**
     * Loads an animation from a JSON file
     * 
     * @param filename Path to the JSON relative to the deploy folder
     * @param name     Name of the animation
     */
    public LightAnimation(String filename, String name) {
        final int numLeds = 10, numFrames = 10;
        this.name = name;

        try {
            String absolutePath = Filesystem.getDeployDirectory().getAbsolutePath();

            Path jsonPath = Path.of(absolutePath + "/" + filename);

            JSONObject json;
            String jsonString = Files.readString(jsonPath);
            json = new JSONObject(jsonString);

            frames = new Color[numFrames][numLeds];
            for (int j = 0; j < numFrames; j++) {
                for (int i = 0; i < numLeds; i++) {
                    JSONObject colorObject = json.getJSONArray("leds").getJSONArray(j).getJSONObject(i);
                    frames[j][i] = new Color(colorObject.getInt("r") / 255.0,
                            colorObject.getInt("g") / 255.0,
                            colorObject.getInt("b") / 255.0);
                }
            }
            // Too many possible exceptions to be able to individually catch them
        } catch (Exception e) {

            SmartDashboard.putString("JSON Error: " + name, e.toString());
            frames = new Color[numFrames][numLeds];

            // Blank animation
            for (int j = 0; j < numFrames; j++) {
                for (int i = 0; i < numLeds; i++) {
                    frames[j][i] = new Color(0, 0, 0);
                }
            }
        }

    }

    public String getName() {
        return name;
    }

    public Color getColor(int frame, int index) {
        double f = frame / 15.0;
        int wholeFrame = (int) f;

        Color a = frames[wholeFrame % frames.length][(index) % frames[0].length];

        return a;
    }

}
