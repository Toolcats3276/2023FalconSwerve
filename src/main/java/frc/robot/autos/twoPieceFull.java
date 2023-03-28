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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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




public class twoPieceFull extends SequentialCommandGroup {
    public twoPieceFull(Swerve s_Swerve){
        
//loads path
PathPlannerTrajectory first = PathPlanner.loadPath("twoPieceFirst", new PathConstraints(2.5, 1.75));

//creates swerve controller command
PPSwerveControllerCommand swerveControllerCommand =
new PPSwerveControllerCommand(
    first,
    s_Swerve::getPose,
    Constants.Swerve.swerveKinematics,
    new PIDController(
        0.8, 0.005, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
        //0.8, 0.0005, 0.1
    new PIDController(
        0.8, 0.005, 0), // Y controller (usually the same values as X controller)
        //0.8, 0.0005, 0.1
    new PIDController(
        0.01, 0.05, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
        // 0.4, 0.0008, 0.0005 
    s_Swerve::setModuleStates,
    true,
    s_Swerve);




    //loads path
PathPlannerTrajectory second = PathPlanner.loadPath("twoPieceSecond", new PathConstraints(2, 0.55));

//creates swerve controller command
PPSwerveControllerCommand swerveControllerCommand2 =
new PPSwerveControllerCommand(
    second,
    s_Swerve::getPose,
    Constants.Swerve.swerveKinematics,
    new PIDController(
        0.8, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
        //0.8, 0.0005, 0.1
    new PIDController(
        0.8, 0, 0), // Y controller (usually the same values as X controller)
        //0.8, 0.0005, 0.1
    new PIDController(
        0.065, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
        // 0.4, 0.0008, 0.0005 
    s_Swerve::setModuleStates,
    true,
    s_Swerve);

//#######################################################################################################################
   
// auto commands

addCommands(

    // new InstantCommand(() -> {Signaling.mode = 13;}),

    // new WaitCommand(1),

    // new InstantCommand(() -> {Signaling.mode = 14;}),
    
    // new WaitCommand(2),

    // new InstantCommand(() -> {Signaling.mode = 8;}),

    // new WaitCommand(1),

    // new InstantCommand(() -> {Signaling.mode = 3;}),

    // new WaitCommand(1),

    new InstantCommand(
        () -> s_Swerve.resetOdometry(
                new Pose2d(
                    first.getInitialPose().getX(),
                    first.getInitialPose().getY(),
                    first.getInitialHolonomicPose().getRotation()))),
                    swerveControllerCommand,
                    swerveControllerCommand2);
    }
}










// package frc.robot.autos;

// import frc.robot.Constants;
// import frc.robot.Signaling;
// import frc.robot.subsystems.Swerve;
// import pabeles.concurrency.ConcurrencyOps.NewInstance;

// import java.util.List;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.trajectory.Trajectory;
// import edu.wpi.first.math.trajectory.TrajectoryConfig;
// import edu.wpi.first.math.trajectory.TrajectoryGenerator;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
// import edu.wpi.first.wpilibj2.command.WaitCommand;

// public class exampleAuto extends SequentialCommandGroup {
//     public exampleAuto(Swerve s_Swerve){
//         TrajectoryConfig config =
//             new TrajectoryConfig(
//                     1,
//                     1)
//                 .setKinematics(Constants.Swerve.swerveKinematics);

//         // An example trajectory to follow.  All units in meters.
//         Trajectory firstMove =
//             TrajectoryGenerator.generateTrajectory(
//                 // Start at the 5,5 facing Y+, for some reason, the "front" of the robot is the back so Y+ is straight back and X+ is left if you are driving the robot
//                 new Pose2d(8, 0, new Rotation2d(0)),
//                 // Pass through these two interior waypoints, making an Circle
//                 List.of(new Translation2d(6, 0)),// These positions are absolute and not relative
//                 // End where it started facing the same direction the whole time.
//                 new Pose2d(9, 0, new Rotation2d(0)),// 4.25 for left, 5 for right, 4 for mid
//                 config);


//         var thetaController =
//             new ProfiledPIDController(
//                 Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
//         thetaController.enableContinuousInput(-Math.PI, Math.PI);

//         SwerveControllerCommand swerveControllerCommand =
//             new SwerveControllerCommand(
//                 firstMove,
//                 s_Swerve::getPose,
//                 Constants.Swerve.swerveKinematics,
//                 new PIDController(Constants.AutoConstants.kPXController, 0, 0),
//                 new PIDController(Constants.AutoConstants.kPYController, 0, 0),
//                 thetaController,
//                 s_Swerve::setModuleStates,
//                 s_Swerve);

//         // addCommands(
//         //     new InstantCommand(() -> {Signaling.mode = 13;})
//         // );

//         // addCommands(
//         //     new WaitCommand(2)
//         // );

//         // addCommands(
//         //     new InstantCommand(() -> {Signaling.mode = 14;})
//         // );

//         // addCommands(
//         //     new WaitCommand(3)
//         // );

//         // addCommands(
//         //     new InstantCommand(() -> {Signaling.mode = 8;})
//         // );

//         // addCommands(
//         //     new WaitCommand(3)
//         // );
       
//         addCommands(
//             // new InstantCommand(() -> {Signaling.mode = 3;}),

//             new InstantCommand(() -> s_Swerve.resetOdometry(firstMove.getInitialPose())),
//             swerveControllerCommand
       
//         );

     



//         // addCommands(
//         //     new InstantCommand(() -> s_Swerve.resetOdometry(firstMove.getInitialPose())),
//         //     new InstantCommand(() -> {Signaling.mode = 8;}),
//         //     new WaitCommand(3),
//         //     new InstantCommand(() -> {Signaling.mode = 3;}),
//             // new WaitCommand(6),
            
//             // new InstantCommand(() -> {Signaling.mode = 8;}),
//             // new WaitCommand(5),
//             // new InstantCommand(() -> {Signaling.mode = 3;}),
//         //    new InstantCommand(() -> s_Swerve.resetOdometry(secondMove.getInitialPose())),


//         //     swerveControllerCommand
//         // );
//     }
// }