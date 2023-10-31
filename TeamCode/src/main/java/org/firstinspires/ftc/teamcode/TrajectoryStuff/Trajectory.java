package org.firstinspires.ftc.teamcode.TrajectoryStuff;

import com.acmerobotics.dashboard.canvas.GPose;

import org.firstinspires.ftc.teamcode.Utils.Pose;
import org.firstinspires.ftc.teamcode.Utils.Vector;

import java.util.ArrayList;

public class Trajectory {

    private ArrayList<TrajectorySegment> segments = new ArrayList<>();

    public Trajectory(TrajectorySegment segment) {
        segments.add(segment);
    }

    public Trajectory(ArrayList<TrajectorySegment> segments) {
        this.segments = segments;
        init();
    }

    private int numberOfSegments;
    private double length;

    private void init() {
        numberOfSegments = segments.size();
        for (TrajectorySegment segment : segments) length += segment.getLength();
    }

    public double getLength() {
        return length;
    }

    public double getLengthAt(double t) {
        if (t >= 1) return length;
        if (t <= 0) return 0;
        double ans = 0;
        t = t * (double) numberOfSegments;
        for (int i = 0; i < (int) t; i++) ans += segments.get(i).getLength();
        ans += segments.get((int) t).getLengthAt(t - (double) ((int) t));
        return ans;
    }

    public Pose getPose(double t) {
        if (t >= 1) return segments.get(numberOfSegments - 1).getPose(1);
        if (t <= 0) return segments.get(0).getPose(0);
        t = t * (double) numberOfSegments;
        return segments.get((int) t).getPose(t - (double) ((int) t));
    }

    public ArrayList<GPose> getAllGPoses() {
        double start = 0,end=1;
        final double eps = 0.001;
        ArrayList<GPose> ans = new ArrayList<>();
        while (start + eps < end) {
            ans.add(new GPose(
                    getPose(start).getX(), getPose(start).getY(),getPose(start).getHeading()
            ));
            start += eps;
        }
        return ans;
    }


    private double getDistanceFromPoint(Pose pose, double t) {
        if (t < 0 || t > 1) return Double.POSITIVE_INFINITY;
        return new Vector(pose.getX(), pose.getY()).plus(new Vector(-getPose(t).getX(), -getPose(t).getY())).getMagnitude();
    }

    public Vector getTangentVelocity(double t) {
        if (t < 0 || t >= 1) return new Vector();
        t *= (double) numberOfSegments;
        return segments.get((int) t).getTangentVelocity(t - (double) ((int) t));
    }

    public double getCurvature(double t) {
        if (t <= 0 || t >= 1) return 0;
        t *= (double) numberOfSegments;
        return segments.get((int) t).getCurvature(t - (double) ((int) t));
    }

    //the function in which we search for valleys can be derived and finding points where it's second derivative is positive
    //and the first derivative is equal to 0 would yield the same result, sadly the first derivative is a 5th grade polynomial
    //so I haven't found a good way to calculate or approximate roots
    public double getFollowedPoint(Pose currentPose, double currentFollowedPoint) {
        if (currentFollowedPoint == 1) return 1;

        ArrayList<Double> valleys = new ArrayList<>();

        double minDist = Double.POSITIVE_INFINITY, closestPoint = Double.POSITIVE_INFINITY;

        for (int i = 0; i < numberOfSegments; i++) {
            valleys = segments.get(i).getClosePoints(currentPose);
            for (int j = 0; j < valleys.size(); j++) {
//                System.out.println(valleys.get(j));
                if (Math.abs(((valleys.get(j) + i) * (1f / (float) numberOfSegments)) - currentFollowedPoint) < minDist) {
                    minDist = Math.abs(valleys.get(j) - currentFollowedPoint);
                    closestPoint = ((double) i + valleys.get(j)) / (double) numberOfSegments;
                }
            }
        }

        if (closestPoint == Double.POSITIVE_INFINITY) return currentFollowedPoint;
        return closestPoint;
    }

}
