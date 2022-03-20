// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.subsystems.Drivetrain;

import java.io.File;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.concurrent.locks.ReentrantLock;

/** Add your docs here. */
public class TrajectoryManager {

    private static HashMap<String, Trajectory> trajectories;

    // trajectories are now protected by ReentrantLock
    private static ReentrantLock trajectoriesLock = new ReentrantLock();

    public static void generateTrajectories() {

        Thread genTrajectoriesThread = new Thread(() -> {

            if (trajectories == null) {
                System.out.println("Info: Trajectories loading...");
                trajectoriesLock.lock();
    
                trajectories = new HashMap<>();
    
                List<String> pathNames = new ArrayList<>();
    
                Path deployDirectory = Paths.get(Filesystem.getDeployDirectory().toString(), "PathWeaver/Paths");
                File[] listOfFiles = deployDirectory.toFile().listFiles();
    
                for (var file : listOfFiles) {
                    System.out.printf("INFO: Loading Trajectory: %s%n", file.getName());
                    var trajPack = TrajectoryPacket.generateTrajectoryPacket(file);
    
                    var trajectory =
                    TrajectoryGenerator.generateTrajectory(
                        new Pose2d(trajPack.firstX, trajPack.firstY, Rotation2d.fromDegrees(trajPack.start_angle)),
                        trajPack.path_read,
                        new Pose2d(trajPack.lastX, trajPack.lastY, Rotation2d.fromDegrees(trajPack.end_angle)),
                        new TrajectoryConfig(Drivetrain.MAX_SPEED, Drivetrain.MAX_ANGULAR_SPEED));

                    trajectories.put(file.getName(), trajectory);
                }
                trajectoriesLock.unlock();
                System.out.println("INFO: All Trajectories loaded");
            }
        });

        genTrajectoriesThread.setDaemon(true);
        genTrajectoriesThread.start();
    }

    public static HashMap<String, Trajectory> getTrajectories() {
        HashMap<String, Trajectory> cur_trajectories = null;

        if (trajectories != null && trajectoriesLock.tryLock()) {
            cur_trajectories = trajectories;
            trajectoriesLock.unlock();
        }

        return cur_trajectories;
    }
}
