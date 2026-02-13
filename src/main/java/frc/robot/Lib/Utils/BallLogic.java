// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Lib.Utils;

import java.util.ArrayList;

import org.photonvision.PhotonUtils;

import edu.wpi.first.math.geometry.Pose2d;

/** Add your docs here. */
public class BallLogic {
    public static ArrayList<Pose2d> rearangePoints(Pose2d startpoint, ArrayList<Pose2d> points, int indexLimit){
        int limit = (indexLimit > points.size()) ? indexLimit-1 : points.size()-1;
        ArrayList<Pose2d> optimizedPoseList = new ArrayList<>();
        optimizedPoseList.add(findClosestPoint(points, startpoint));
        points.remove(findClosestPoint(points, startpoint));
        for(int i = 0; i<limit; i++){
            optimizedPoseList.add(findClosestPoint(points, points.get(i)));
            points.remove(findClosestPoint(points, points.get(i)));
        }

        return optimizedPoseList;
    }

    public static Pose2d findClosestPoint(ArrayList<Pose2d> points, Pose2d currentPoint){
        double shortestDistance = Double.POSITIVE_INFINITY;
        Pose2d closestPose = null;
        for(Pose2d point : points){
            if(PhotonUtils.getDistanceToPose(currentPoint, point) < shortestDistance){
                closestPose = point;
            }
        }
        return closestPose;
    }
}
