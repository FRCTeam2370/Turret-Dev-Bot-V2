// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Lib.Utils;

import static edu.wpi.first.units.Units.Rotation;

import java.util.ArrayList;

import org.photonvision.PhotonUtils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class BallLogic {
    public static ArrayList<Pose2d> rearangePoints(Pose2d startpoint, ArrayList<Pose2d> points, int indexLimit){
        int limit = (points.size() < indexLimit) ? points.size() : indexLimit;
        ArrayList<Pose2d> optimizedPoseList = new ArrayList<>();
        optimizedPoseList.add(startpoint);
        optimizedPoseList.add(new Pose2d(findClosestPoint(points, startpoint).getTranslation(), Rotation2d.fromDegrees(getRotation2dToPose(startpoint, findClosestPoint(points, startpoint)).getDegrees())));
        points.remove(findClosestPoint(points, startpoint));
        if(limit >= 1){
            for(int i = 0; i<limit; i++){
                optimizedPoseList.add(new Pose2d(findClosestPoint(points, points.get(i)).getTranslation(), Rotation2d.fromDegrees(getRotation2dToPose(startpoint, findClosestPoint(points, points.get(i))).getDegrees())));//findClosestPoint(points, points.get(i)));
                points.remove(findClosestPoint(points, points.get(i)));
                limit = (points.size() < indexLimit) ? points.size() : indexLimit;//recalculate the limit if balls have dissapeared or moved since last check
            }
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

    public static Rotation2d getRotation2dToPose(Pose2d startPose, Pose2d targetPose){
        double xdis = targetPose.getX() - startPose.getX();
        double ydis = targetPose.getY() - startPose.getY();

        Rotation2d angleToPose = Rotation2d.fromRadians(Math.atan2(ydis, xdis));

        return angleToPose;
    }
}
