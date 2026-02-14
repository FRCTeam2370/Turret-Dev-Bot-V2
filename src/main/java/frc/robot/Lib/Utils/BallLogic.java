// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Lib.Utils;

import java.util.ArrayList;

import org.photonvision.PhotonUtils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class BallLogic {
    public static ArrayList<Pose2d> rearangePoints(Pose2d startpoint, ArrayList<Pose2d> points, int indexLimit){
        int limit = (points.size() < indexLimit) ? points.size() : indexLimit;//limit is whichever number is smaller
        ArrayList<Pose2d> optimizedPoseList = new ArrayList<>();//makes a new ArrayList to store our optimized list of poses

        optimizedPoseList.add(startpoint);//adds the initial position of the robot to the list

        //adds the closest point to the start point
        optimizedPoseList.add(new Pose2d(findClosestPoint(points, startpoint).getTranslation(), Rotation2d.fromDegrees(getRotation2dToPose(startpoint, findClosestPoint(points, startpoint)).getDegrees())));
        points.remove(findClosestPoint(points, startpoint));

        //if more than one ball is visible then add the rest to the optimized list
        if(limit >= 1){
            //iterates through the rest of the balls
            for(int i = 0; i<limit; i++){
                //adds the next closest ball to the list
                optimizedPoseList.add(new Pose2d(findClosestPoint(points, points.get(i)).getTranslation(), Rotation2d.fromDegrees(getRotation2dToPose(startpoint, findClosestPoint(points, points.get(i))).getDegrees())));//findClosestPoint(points, points.get(i)));
                points.remove(findClosestPoint(points, points.get(i)));//removes that ball from the initial list so we don't use it again
                limit = (points.size() < indexLimit) ? points.size() : indexLimit;//recalculate the limit if balls have dissapeared or moved since last check
            }
        }

        return optimizedPoseList;//returns our optimized list
    }

    //finds the closest point to the point specified
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

    //gets the rotation from the current post to the target pose (for some reason the Photon.Utils one wasn't working quite right so I added this)
    public static Rotation2d getRotation2dToPose(Pose2d startPose, Pose2d targetPose){
        double xdis = targetPose.getX() - startPose.getX();
        double ydis = targetPose.getY() - startPose.getY();

        Rotation2d angleToPose = Rotation2d.fromRadians(Math.atan2(ydis, xdis));

        return angleToPose;
    }
}
