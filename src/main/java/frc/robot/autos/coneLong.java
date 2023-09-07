package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.Signaling;
import frc.robot.subsystems.Swerve;
import pabeles.concurrency.ConcurrencyOps.NewInstance;

import java.util.List;
import java.util.ArrayList;
import java.util.HashMap;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;




public class coneLong extends SequentialCommandGroup {
    public coneLong(Swerve s_Swerve){

 
//#######################################################################################################################


//loads path
PathPlannerTrajectory first = PathPlanner.loadPath("mobilityLong", new PathConstraints(.75, 0.75));

//creates swerve controller command
PPSwerveControllerCommand swerveControllerCommand =
new PPSwerveControllerCommand(
    first,
    s_Swerve::getPose,
    Constants.Swerve.swerveKinematics,
    new PIDController(2, 10, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.   
    //TODO try 0.5, 5, 0 at comp
    new PIDController(2, 10, 0), // Y controller (usually the same values as X controller)

    new PIDController(0, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
    s_Swerve::setModuleStates,
    false,
    s_Swerve);

//#######################################################################################################################
   
//auto commands


addCommands(
    new InstantCommand(() -> {Signaling.mode = 23;})
);

addCommands(
    new WaitCommand(1)
);

addCommands(
    new InstantCommand(() -> {Signaling.mode = 13;})
);

addCommands(
    new WaitCommand(1)
);

addCommands(
    new InstantCommand(() -> {Signaling.mode = 14;})
);

addCommands(
    new WaitCommand(1.5)
);

addCommands(
    new InstantCommand(() -> {Signaling.mode = 16;})
);

addCommands(
    new WaitCommand(1)
);

addCommands(
    new InstantCommand(() -> {Signaling.mode = 12;})
);

addCommands(
    new WaitCommand(0.75)
);

addCommands(
    new InstantCommand(() -> {Signaling.mode = 20;})
);

addCommands(
    new WaitCommand(0.75)
);

addCommands(
    new InstantCommand(() -> {Signaling.mode = 24;})
);

addCommands(
    new WaitCommand(1)
);

addCommands(//reset odometry, move to cube
new InstantCommand(
    () -> s_Swerve.resetOdometry(
            new Pose2d(
                first.getInitialPose().getX(),
                first.getInitialPose().getY(),
                first.getInitialHolonomicPose().getRotation()))),
                swerveControllerCommand
);

    }
}
