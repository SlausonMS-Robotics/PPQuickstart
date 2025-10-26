package org.firstinspires.ftc.teamcode.robot;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.net.HttpURLConnection;
import java.net.InetSocketAddress;
import java.net.Socket;
import java.net.URL;
import java.nio.charset.StandardCharsets;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * Minimal FTC helper for Limelight 3A + MegaTag2 to compute range.
 * Works WITHOUT NetworkTables. Two data paths:
 *   A) HTTP JSON: http://<host>:5807/limelight-json
 *   B) llpython: use your existing Limelight client that exposes result.getPythonOutput()
 *
 * Coordinate frame (cameraPose_TargetSpace):
 *   X = forward (meters), Y = left (m), Z = up (m).
 * Ground range = hypot(X, Y). 3D range = sqrt(X^2 + Y^2 + Z^2).
 */

public class limelight_helpers {

    // --- HTTP JSON path ------------------------------------------------------

    /**
     * Quick reachability check so we fail fast & non-blocking in opmodes.
     */
    public static boolean isReachable(String host, int port, int timeoutMs) {
        try (Socket socket = new Socket()) {
            socket.connect(new InetSocketAddress(host, port), timeoutMs);
            return true;
        } catch (Exception e) {
            return false;
        }
    }

    /**
     * Reads Limelight JSON (best fiducial) and returns [x, y, z] in meters
     * from "cameraPose_TargetSpace". Returns null if unavailable.
     *
     * Example valid hosts:
     *   "10.xx.yy.11"   (static IP)
     *   "limelight.local"
     *   "limelight-nt.local" (if you renamed it)
     */
    public static double[] readBestTargetCameraPose(String host) {
        BufferedReader reader = null;
        HttpURLConnection conn = null;
        try {
            URL url = new URL("http://" + host + ":5807/results"); // <-- correct endpoint
            conn = (HttpURLConnection) url.openConnection();
            conn.setConnectTimeout(400);  // a bit more forgiving
            conn.setReadTimeout(400);
            conn.setRequestMethod("GET");

            if (conn.getResponseCode() != 200) return null;

            reader = new BufferedReader(new InputStreamReader(conn.getInputStream(), StandardCharsets.UTF_8));
            StringBuilder sb = new StringBuilder(2048);
            String line; while ((line = reader.readLine()) != null) sb.append(line);
            String json = sb.toString();

            // Fast bailout if no valid targets: look for "v":1 at top level
            if (!json.contains("\"v\":1")) return null;

            // Prefer target pose in CAMERA space: Fiducial[0].t6t_cs = [x,y,z,rx,ry,rz]
            Matcher m = Pattern.compile("\"Fiducial\"\\s*:\\s*\\[\\s*\\{[^}]*?\"t6t_cs\"\\s*:\\s*\\[([^\\]]+)\\]")
                    .matcher(json);
            if (!m.find()) {
                // Fallback: camera pose in TARGET space: Fiducial[0].t6c_ts
                m = Pattern.compile("\"Fiducial\"\\s*:\\s*\\[\\s*\\{[^}]*?\"t6c_ts\"\\s*:\\s*\\[([^\\]]+)\\]").matcher(json);
                if (!m.find()) return null;
            }

            String[] parts = m.group(1).split(",");
            if (parts.length < 3) return null;

            double x = Double.parseDouble(parts[0].trim());
            double y = Double.parseDouble(parts[1].trim());
            double z = Double.parseDouble(parts[2].trim());
            return new double[]{x, y, z};
        } catch (Exception e) {
            return null;
        } finally {
            try { if (reader != null) reader.close(); } catch (Exception ignore) {}
            if (conn != null) conn.disconnect();
        }
    }

    /** Ground range (meters) from cameraPose_TargetSpace via HTTP JSON. */
    public static double getGroundRangeM_FromHttp(String host) {
        double[] xyz = readBestTargetCameraPose(host);
        if (xyz == null) return Double.NaN;
        double x = xyz[0], y = xyz[1];
        return Math.hypot(x, y);
    }

    /** Full 3D straight-line distance (meters) via HTTP JSON. */
    public static double get3DRangeM_FromHttp(String host) {
        double[] xyz = readBestTargetCameraPose(host);
        if (xyz == null) return Double.NaN;
        double x = xyz[0], y = xyz[1], z = xyz[2];
        return Math.sqrt(x*x + y*y + z*z);
    }

    // --- llpython path -------------------------------------------------------

    /**
     * If your Limelight client provides llpython[] (e.g., result.getPythonOutput()),
     * and you ensure it contains [x,y,z,rx,ry,rz] of the BEST fiducial in CAMERA space,
     * use this to compute ground range.
     *
     * @param llpython array with at least 3 elements [x,y,z,...] in METERS.
     */
    public static double getGroundRangeM_FromLLPython(double[] llpython) {
        if (llpython == null || llpython.length < 3) return Double.NaN;
        return Math.hypot(llpython[0], llpython[1]);
    }

    /** 3D distance from llpython. */
    public static double get3DRangeM_FromLLPython(double[] llpython) {
        if (llpython == null || llpython.length < 3) return Double.NaN;
        double x = llpython[0], y = llpython[1], z = llpython[2];
        return Math.sqrt(x*x + y*y + z*z);
    }

    // --- Convenience ---------------------------------------------------------

    public static double metersToInches(double m) {
        return Double.isNaN(m) ? Double.NaN : (m * 39.37007874);
    }

    public static void telemetryRange(Telemetry tel, String label, double meters) {
        tel.addData(label + " (m)", "%.3f", meters);
        tel.addData(label + " (in)", "%.2f", metersToInches(meters));
    }
}
