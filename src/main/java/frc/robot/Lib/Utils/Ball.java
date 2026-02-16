// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Lib.Utils;

/** Add your docs here. */
public class Ball {
    public double x, y, weight;
    public Ball(double x, double y, double weight){
        this.weight = weight;
        this.x = x;
        this.y = y;
    }

    public double getWeight(){
        return weight;
    }
}
