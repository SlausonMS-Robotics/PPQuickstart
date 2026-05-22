package org.firstinspires.ftc.teamcode.simplifiedpathing.paths;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.simplifiedpathing.actions.Action;
import org.firstinspires.ftc.teamcode.simplifiedpathing.actions.SimpleActions.*;
import org.firstinspires.ftc.teamcode.simplifiedpathing.follower.SimpleBezier;
import org.firstinspires.ftc.teamcode.simplifiedpathing.follower.SimplifiedFollower;
import org.firstinspires.ftc.teamcode.simplifiedpathing.robot.SimplifiedRobot;
import org.json.JSONArray;
import org.json.JSONObject;

import java.io.InputStream;
import java.nio.charset.StandardCharsets;
import java.util.HashMap;
import java.util.Map;

/**
 * Loads paths and sequences from a .pp JSON file.
 */
public class SimplePathLoader {

    /**
     * Loads all paths defined in the .pp file into a Map, using their "name" as the key.
     * This allows you to decouple paths from the sequence and build your own auto manually.
     */
    public static Map<String, Action> loadPaths(String fileName, SimplifiedFollower follower, SimplifiedRobot robot) {
        Map<String, Action> pathMap = new HashMap<>();
        try {
            JSONObject root = loadJson(fileName);
            if (root == null) return pathMap;

            // Get Start Pose and set it
            JSONObject startObj = root.getJSONObject("startPoint");
            Pose2D startPose = new Pose2D(DistanceUnit.INCH, startObj.getDouble("x"), startObj.getDouble("y"), 
                                         AngleUnit.DEGREES, startObj.getDouble("startDeg"));
            robot.sensors.pinpoint.setPosition(startPose);

            JSONArray linesArray = root.getJSONArray("lines");
            double cursorX = startPose.getX(DistanceUnit.INCH);
            double cursorY = startPose.getY(DistanceUnit.INCH);
            double cursorDeg = startPose.getHeading(AngleUnit.DEGREES);

            for (int i = 0; i < linesArray.length(); i++) {
                JSONObject lineObj = linesArray.getJSONObject(i);
                String name = lineObj.optString("name", "Path " + i);
                JSONObject endPoint = lineObj.getJSONObject("endPoint");
                
                Pose2D pStart = new Pose2D(DistanceUnit.INCH, cursorX, cursorY, AngleUnit.DEGREES, cursorDeg);
                Pose2D pEnd = new Pose2D(DistanceUnit.INCH, endPoint.getDouble("x"), endPoint.getDouble("y"), 
                                        AngleUnit.DEGREES, endPoint.getDouble("endDeg"));

                JSONArray controlPoints = lineObj.getJSONArray("controlPoints");
                SimpleBezier curve;
                if (controlPoints.length() == 0) {
                    curve = new SimpleBezier(pStart, new Pose2D(DistanceUnit.INCH, (cursorX + pEnd.getX(DistanceUnit.INCH))/2, (cursorY + pEnd.getY(DistanceUnit.INCH))/2, AngleUnit.DEGREES, (cursorDeg + pEnd.getHeading(AngleUnit.DEGREES))/2), pEnd);
                } else if (controlPoints.length() == 1) {
                    JSONObject cp = controlPoints.getJSONObject(0);
                    curve = new SimpleBezier(pStart, new Pose2D(DistanceUnit.INCH, cp.getDouble("x"), cp.getDouble("y"), AngleUnit.DEGREES, (cursorDeg + pEnd.getHeading(AngleUnit.DEGREES))/2), pEnd);
                } else {
                    JSONObject cp1 = controlPoints.getJSONObject(0);
                    JSONObject cp2 = controlPoints.getJSONObject(1);
                    curve = new SimpleBezier(pStart, 
                        new Pose2D(DistanceUnit.INCH, cp1.getDouble("x"), cp1.getDouble("y"), AngleUnit.DEGREES, cursorDeg),
                        new Pose2D(DistanceUnit.INCH, cp2.getDouble("x"), cp2.getDouble("y"), AngleUnit.DEGREES, pEnd.getHeading(AngleUnit.DEGREES)),
                        pEnd);
                }
                
                pathMap.put(name, new PathAction(follower, robot, curve));
                
                cursorX = pEnd.getX(DistanceUnit.INCH);
                cursorY = pEnd.getY(DistanceUnit.INCH);
                cursorDeg = pEnd.getHeading(AngleUnit.DEGREES);
            }
        } catch (Exception e) {
            // Error handling
        }
        return pathMap;
    }

    private static JSONObject loadJson(String fileName) {
        try {
            String path = "/org/firstinspires/ftc/teamcode/simplifiedpathing/paths/" + fileName;
            InputStream is = SimplePathLoader.class.getResourceAsStream(path);
            if (is == null) return null;

            int size = is.available();
            byte[] buffer = new byte[size];
            is.read(buffer);
            is.close();
            return new JSONObject(new String(buffer, StandardCharsets.UTF_8));
        } catch (Exception e) {
            return null;
        }
    }
}
