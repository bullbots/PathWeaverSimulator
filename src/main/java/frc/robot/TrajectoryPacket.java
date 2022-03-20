// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;

import java.io.*;
import java.util.ArrayList;

/** Data class for storing and generating Trajectory parameters */
public class TrajectoryPacket {
    public double firstX;
    public double firstY;
    public ArrayList<Translation2d> path_read;
    public double lastX;
    public double lastY;
    public double start_angle;
    public double end_angle;

    public static TrajectoryPacket generateTrajectoryPacket(File file) {
        var path_read = new ArrayList<Translation2d>();
        var angle_list = new ArrayList<Double>();

        BufferedReader br = null;
        try {
            System.out.printf("INFO: Reading file: %s%n", file.getAbsolutePath());
            br = new BufferedReader(new FileReader(file));
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }
        
        String line = "";

        try {
        while ((line = br.readLine()) != null) {
            try {
            String[] sections = line.split(",");
            
            double x = Double.parseDouble(sections[0]);
            double y = Double.parseDouble(sections[1]) + 8.23; // Need Field width offset ~8.23 meters

            path_read.add(new Translation2d(x, y));

            Double tangent_x = Double.parseDouble(sections[2]);
            Double tangent_y = Double.parseDouble(sections[3]);

            double angle = Math.atan2(tangent_y, tangent_x);
            
            angle_list.add(Math.toDegrees(angle));

            } catch (NumberFormatException error) {
            // System.out.println("Ignore this error:");
            // error.printStackTrace();
            }
        }
        } catch (IOException error) {
        // System.out.println("Ignore this error:");
        // error.printStackTrace();
        }

        var trajectoryPacket = new TrajectoryPacket();

        // Gets first and last elements and removes them from list.
        trajectoryPacket.firstX = path_read.get(0).getX();
        trajectoryPacket.firstY = path_read.get(0).getY();
        trajectoryPacket.lastX = path_read.get(path_read.size()-1).getX();
        trajectoryPacket.lastY = path_read.get(path_read.size()-1).getY();

        path_read.remove(0);
        path_read.remove(path_read.size()-1);

        trajectoryPacket.path_read = path_read;

        // Gets first and last angle and makes end angle relative to start angle instead of absolute.
        trajectoryPacket.start_angle = angle_list.get(0);
        trajectoryPacket.end_angle = angle_list.get(angle_list.size()-1);
        // trajectoryPacket.end_angle -= start_angle;  // <- This seems to be not correct...

        return trajectoryPacket;
    }
}
