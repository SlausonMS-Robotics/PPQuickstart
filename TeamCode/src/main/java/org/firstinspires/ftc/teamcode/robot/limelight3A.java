package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class limelight3A {

    // Constants
    public static final double LIMELIGHT_HEIGHT = 10.25; // inches
    public static final double LIMELIGHT_ANGLE = 66.0;   // degrees
    public static final double LIMELIGHT_Y_OFFSET = -3.0; // inches
    public static final int LIMELIGHT_OBJECT_COUNT = 5;

    public static final double LIMELIGHT_DRIVE_OFFSET_X = 17;
    public static final double LIMELIGHT_DRIVE_OFFSET_Y = -3;

    public boolean LLOn = false;


    private Pose sample = new Pose(), difference = new Pose(), target = new Pose(); // The best sample's position
    private Pose cachedTarget = new Pose(); // Cached best sample
    private PathChain toTarget;
    private Follower f;

    private Telemetry telemetry;

    // Hardware
    private Limelight3A limelight;
    private Servo headlight;

    // Results
    public LLResult result;
    public LLObject[] LLObjects = new LLObject[LIMELIGHT_OBJECT_COUNT];

    /**
     * Initializes the Limelight and headlight hardware.
     */
    public boolean init(HardwareMap hwMap, int pipeline, Follower f, Telemetry telemetry) {
        try {
            limelight = hwMap.get(Limelight3A.class, "Limelight");
            headlight = hwMap.get(Servo.class, "Servo0");
            this.f = f;
            this.telemetry = telemetry;

            limelight.setPollRateHz(100);
            limelight.pipelineSwitch(pipeline);


            return true;
        } catch (Exception e) {
            e.printStackTrace();
            return false;
        }
    }

    /**
     * Grabs the latest Limelight result and parses it into LLObjects.
     */

    public void startLL(int wait_ms){
        limelight.start();
        headlight.setPosition(1);  // turn on headlight
        long startTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - startTime < wait_ms) {
            // just waiting for the headlight to come on
        }
        LLOn = true;
    }

    public boolean getLLStatus(){
        return LLOn;
    }

    public void stopLL(){
        if (limelight != null) limelight.stop();
        if (headlight != null) headlight.setPosition(0);
        LLOn = false;

    }
    public boolean pollLimelight() {

        if (limelight == null) return false;


        result = limelight.getLatestResult();
        if (result == null) return false;
        //parseLLData();

        return true;
    }

    public double getLLAvgDist(){
        return result.getBotposeAvgDist();
    }

    public PathChain LLDriveTo(){

        if (result != null) {
            double x = getXDist(0) ;
            double y = getYDist(0) ;
            f.update();
            sample = new Pose(x,y,0);
            Pose curPose = f.getPose();
            telemetry.addData("sample", sample);
            telemetry.addData("current", curPose);
            telemetry.addData("x", x);
            telemetry.addData("y", y);
            x -= LIMELIGHT_DRIVE_OFFSET_X;
            y -= - LIMELIGHT_DRIVE_OFFSET_Y;
            double curHeading = curPose.getHeading();
            double c = Math.cos(curHeading);
            double s = Math.sin(curHeading);
            double x_field  = curPose.getX() + c*x + s*y;
            double y_field = curPose.getY() + s*x - c*y;


            target = new Pose(x_field , y_field, curPose.getHeading());
            telemetry.addData("target", target);
            telemetry.update();
            cachedTarget = target.copy();
            f.update();
            toTarget = new PathBuilder(f)
                    .addPath(new BezierLine(f.getPose(), target))
                    .setConstantHeadingInterpolation(f.getPose().getHeading())
                    //.setZeroPowerAccelerationMultiplier(5)
                    .build();
            return toTarget;
        }
        else {
            return toTarget = new PathBuilder(f)
                    .addPath(new BezierLine(f.getPose(), f.getPose())).setConstantHeadingInterpolation(f.getPose().getHeading()).build();
        }

    }

    /**
     * Parses Limelight Python output into LLObject[].
     */
    private void parseLLData() {
        if (result == null) return;

        double[] llpython = result.getPythonOutput();
        if (llpython == null || llpython.length < 6) return;

        int count = Math.min(llpython.length / 6, LIMELIGHT_OBJECT_COUNT);
        for (int i = 0; i < count; i++) {
            int base = i * 6;
            LLObjects[i] = new LLObject(
                    llpython[base],         // xDeg
                    llpython[base + 1],     // yDeg
                    llpython[base + 2],     // angle
                    llpython[base + 3],     // area
                    llpython[base + 4],     // ratio
                    (int) llpython[base + 5] // colorId
            );
        }
    }

    /**
     * Represents one detected object from Limelight.
     */
    public static class LLObject {
        public double xDeg, yDeg, angle, area, ratio;
        public int colorId;

        public LLObject(double xDeg, double yDeg, double angle, double area, double ratio, int colorId) {
            this.xDeg = xDeg;
            this.yDeg = yDeg;
            this.angle = angle;
            this.area = area;
            this.ratio = ratio;
            this.colorId = colorId;
        }

        public String getColorName() {
            if (colorId == 0) return "Red";
            else if (colorId == 1) return "Yellow";
            else if (colorId == 2) return "Blue";
            else return "Unknown";
        }
    }

    // ----- Accessors and Calculated Geometry -----

    public double getXDist(int index) {
        if (!isValid(index)) return 0;
        return LIMELIGHT_HEIGHT * 1.075 * Math.tan(Math.toRadians(LIMELIGHT_ANGLE + result.getTy()));
    }


    public double getYDist(int index) {
        if (!isValid(index)) return 0;
        return getXDist(0) / Math.tan(Math.toRadians(90 - result.getTx())) ;
    }

    public double getXDeg(int index) { return getSafe(index).xDeg; }
    public double getYDeg(int index) { return getSafe(index).yDeg; }
    public double getOrientation(int index) { return getSafe(index).angle; }
    public double getHeading(int index) { return getSafe(index).xDeg; }
    public double getArea(int index) { return getSafe(index).area; }
    public double getRatio(int index) { return getSafe(index).ratio; }
    public int getColorId(int index) { return getSafe(index).colorId; }

    /**
     * Stops the Limelight and turns off headlight.
     */

    // ----- Safety Helpers -----

    private boolean isValid(int index) {
        return LLObjects != null && index >= 0 && index < LLObjects.length && LLObjects[index] != null;
    }

    private LLObject getSafe(int index) {
        return isValid(index) ? LLObjects[index] : new LLObject(0, 0, 0, 0, 0, -1);
    }
}
