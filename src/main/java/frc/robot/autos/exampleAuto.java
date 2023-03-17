// package frc.robot.autos;

// import frc.robot.Constants;
// import frc.robot.Signaling;
// import frc.robot.subsystems.Swerve;
// import pabeles.concurrency.ConcurrencyOps.NewInstance;

// import java.util.List;
// import java.util.ArrayList;
// import java.util.HashMap;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.kinematics.SwerveModuleState;
// import edu.wpi.first.math.trajectory.Trajectory;
// import edu.wpi.first.math.trajectory.TrajectoryConfig;
// import edu.wpi.first.math.trajectory.TrajectoryGenerator;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.PrintCommand;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
// import edu.wpi.first.wpilibj2.command.WaitCommand;

// import com.pathplanner.lib.PathConstraints;
// import com.pathplanner.lib.PathPlanner;
// import com.pathplanner.lib.PathPlannerTrajectory;
// import com.pathplanner.lib.PathPoint;
// import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
// import com.pathplanner.lib.auto.PIDConstants;
// import com.pathplanner.lib.auto.SwerveAutoBuilder;
// import com.pathplanner.lib.commands.FollowPathWithEvents;
// import com.pathplanner.lib.commands.PPSwerveControllerCommand;




// public class exampleAuto extends SequentialCommandGroup {
//     public exampleAuto(Swerve s_Swerve){

// // This will load the file "Example Path.path" and generate it with a max velocity of 4 m/s and a max acceleration of 3 m/s^2
// PathPlannerTrajectory first = PathPlanner.loadPath("New Path", new PathConstraints(4, 3));

// var thetaController =
// new ProfiledPIDController(
//     Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
// thetaController.enableContinuousInput(-Math.PI, Math.PI);

// PPSwerveControllerCommand swerveControllerCommand =
// new PPSwerveControllerCommand(
//     first,
//     s_Swerve::getPose,
//     Constants.Swerve.swerveKinematics,
//     new PIDController(Constants.AutoConstants.kPXController, 0, 0),
//     new PIDController(Constants.AutoConstants.kPYController, 0, 0),
//     new PIDController(Constants.AutoConstants.kPThetaController,0,0),
//     s_Swerve::setModuleStates,
//     s_Swerve);

//     // This will load the file "Example Path.path" and generate it with a max velocity of 4 m/s and a max acceleration of 3 m/s^2
// PathPlannerTrajectory second = PathPlanner.loadPath("New Path", new PathConstraints(4, 3));

// var thetaController2 =
// new ProfiledPIDController(
//     Constants.AutoConstants.kPThetaController2, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
// thetaController2.enableContinuousInput(-Math.PI, Math.PI);

// PPSwerveControllerCommand swerveControllerCommand2 =
// new PPSwerveControllerCommand(
//     second,
//     s_Swerve::getPose,
//     Constants.Swerve.swerveKinematics,
//     new PIDController(Constants.AutoConstants.kPXController, 0, 0),
//     new PIDController(Constants.AutoConstants.kPYController, 0, 0),
//     new PIDController(Constants.AutoConstants.kPThetaController2,0,0),
//     s_Swerve::setModuleStates,
//     s_Swerve);

// addCommands(

//     new InstantCommand(
//             () -> s_Swerve.resetOdometry(
//                 new Pose2d(
//                     first.getInitialPose().getX(),
//                     first.getInitialPose().getY(), 
//                     Rotation2d.fromDegrees(0)
//                     )
//                 )
//             ),
//     swerveControllerCommand
// ); 

// addCommands(
//     new InstantCommand(),
//     swerveControllerCommand2
// );



//     }
// }










package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.Signaling;
import frc.robot.subsystems.Swerve;
import pabeles.concurrency.ConcurrencyOps.NewInstance;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class exampleAuto extends SequentialCommandGroup {
    public exampleAuto(Swerve s_Swerve){
        TrajectoryConfig config =
            new TrajectoryConfig(
                    1,
                    1)
                .setKinematics(Constants.Swerve.swerveKinematics);

        // An example trajectory to follow.  All units in meters.
        Trajectory firstMove =
            TrajectoryGenerator.generateTrajectory(
                // Start at the 5,5 facing Y+, for some reason, the "front" of the robot is the back so Y+ is straight back and X+ is left if you are driving the robot
                new Pose2d(8, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an Circle
                List.of(new Translation2d(6, 0)),// These positions are absolute and not relative
                // End where it started facing the same direction the whole time.
                new Pose2d(9, 0, new Rotation2d(0)),// 4.25 for left, 5 for right, 4 for mid
                config);


        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                firstMove,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);

        // addCommands(
        //     new InstantCommand(() -> {Signaling.mode = 13;})
        // );

        // addCommands(
        //     new WaitCommand(2)
        // );

        // addCommands(
        //     new InstantCommand(() -> {Signaling.mode = 14;})
        // );

        // addCommands(
        //     new WaitCommand(3)
        // );

        // addCommands(
        //     new InstantCommand(() -> {Signaling.mode = 8;})
        // );

        // addCommands(
        //     new WaitCommand(3)
        // );
       
        addCommands(
            // new InstantCommand(() -> {Signaling.mode = 3;}),

            new InstantCommand(() -> s_Swerve.resetOdometry(firstMove.getInitialPose())),
            swerveControllerCommand
       
        );

     



        // addCommands(
        //     new InstantCommand(() -> s_Swerve.resetOdometry(firstMove.getInitialPose())),
        //     new InstantCommand(() -> {Signaling.mode = 8;}),
        //     new WaitCommand(3),
        //     new InstantCommand(() -> {Signaling.mode = 3;}),
            // new WaitCommand(6),
            
            // new InstantCommand(() -> {Signaling.mode = 8;}),
            // new WaitCommand(5),
            // new InstantCommand(() -> {Signaling.mode = 3;}),
        //    new InstantCommand(() -> s_Swerve.resetOdometry(secondMove.getInitialPose())),


        //     swerveControllerCommand
        // );
    }
}