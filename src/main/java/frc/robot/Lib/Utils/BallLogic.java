// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Lib.Utils;

import java.lang.reflect.Array;
import java.time.chrono.ThaiBuddhistChronology;
import java.util.ArrayList;

import org.photonvision.PhotonUtils;
import org.photonvision.proto.Photon;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class BallLogic {
    @SuppressWarnings("unlikely-arg-type")
    public static ArrayList<Pose2d> rearangePoints(Pose2d startpoint, ArrayList<Pose2d> points, int indexLimit){
        int limit = indexLimit;//limit is whichever number is smaller
        ArrayList<Pose2d> optimizedPoseList = new ArrayList<>();//makes a new ArrayList to store our optimized list of poses

        optimizedPoseList.add(startpoint);//adds the initial position of the robot to the list

        //if more than one ball is visible then add the rest to the optimized list
        for(int i=0; i <= limit; i++){
            optimizedPoseList.add(findClosestPoint(points, optimizedPoseList.get(i)).getFirst());
            points.remove(findClosestPoint(points, optimizedPoseList.get(i)).getSecond());
        }

        // for(int i = 1; i < optimizedPoseList.size(); i++){
        //     optimizedPoseList.set(i, new Pose2d(optimizedPoseList.get(i).getTranslation(), getRotation2dToPose(optimizedPoseList.get(i-1), optimizedPoseList.get(i))));
        // }

        return optimizedPoseList;//returns our optimized list
    }

    //finds the closest point to the point specified
    public static Pair<Pose2d, Integer> findClosestPoint(ArrayList<Pose2d> points, Pose2d currentPoint){
        double shortestDistance = Double.POSITIVE_INFINITY;
        Pose2d closestPose = null;
        int index = 0;
        // for(Pose2d point : points){
        //     if(PhotonUtils.getDistanceToPose(currentPoint, point) < shortestDistance){
        //         closestPose = point;
        //     }
        // }
        for(int i=0; i< points.size(); i ++){
            if(Math.abs(PhotonUtils.getDistanceToPose(currentPoint, points.get(i))) < shortestDistance){
                shortestDistance = Math.abs(PhotonUtils.getDistanceToPose(currentPoint, points.get(i)));
                closestPose = points.get(i);
                index = i;
            }
        }
        return Pair.of(closestPose, index);
    }

    public static Pose2d findLowestWeight(ArrayList<Pose2d> points, double[] weights){
        int lowestWeightIndex = 0;
        double lowestWeight = Double.NEGATIVE_INFINITY;
        for(int i = 0; i<weights.length; i++){
            if(weights[i] > lowestWeight){
                lowestWeight = weights[i];
                lowestWeightIndex = i;
            }
        }

        return points.get(lowestWeightIndex);
    }

    //gets the rotation from the current post to the target pose (for some reason the Photon.Utils one wasn't working quite right so I added this)
    public static Rotation2d getRotation2dToPose(Pose2d startPose, Pose2d targetPose){
        double xdis = targetPose.getX() - startPose.getX();
        double ydis = targetPose.getY() - startPose.getY();

        Rotation2d angleToPose = Rotation2d.fromRadians(Math.atan2(ydis, xdis));

        return angleToPose;
    }
}
