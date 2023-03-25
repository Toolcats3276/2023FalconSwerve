// // package frc.robot.autos;

// // import frc.robot.Constants;
// // import frc.robot.Signaling;
// // import frc.robot.subsystems.Swerve;

// // import java.util.ArrayList;
// // import java.util.List;

// // import edu.wpi.first.math.controller.PIDController;
// // import edu.wpi.first.math.controller.ProfiledPIDController;
// // import edu.wpi.first.math.geometry.Pose2d;
// // import edu.wpi.first.math.geometry.Rotation2d;
// // import edu.wpi.first.math.geometry.Translation2d;
// // import edu.wpi.first.math.trajectory.Trajectory;
// // import edu.wpi.first.math.trajectory.TrajectoryConfig;
// // import edu.wpi.first.math.trajectory.TrajectoryGenerator;
// // import edu.wpi.first.wpilibj2.command.InstantCommand;
// // import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// // import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
// // import edu.wpi.first.wpilibj2.command.WaitCommand;

// // public class idkAuto extends SequentialCommandGroup {
// //     public idkAuto(Swerve s_Swerve){
// //         TrajectoryConfig config =
// //             new TrajectoryConfig(
// //                     0.75,
// //                     0.7)
// //                 .setKinematics(Constants.Swerve.swerveKinematics);

// //         // An example trajectory to follow.  All units in meters.
// //         Trajectory firstMove =
// //             TrajectoryGenerator.generateTrajectory(
// //                 // Start at the 5,5 facing Y+, for some reason, the "front" of the robot is the back so Y+ is straight back and X+ is left if you are driving the robot
// //                 new Pose2d(10, 10, Rotation2d.fromDegrees(180)), //0,0,90
// //                 // Pass through these two interior waypoints, making a Circle
// //                 new ArrayList<>(), //List.of(new Translation2d(0, 0)), // These positions are absolute and not relative
// //                 // End where it started facing the same direction the whole time.
// //                 new Pose2d(11,10, Rotation2d.fromDegrees(180)),// 4.25 for left, 5 for right, 4 for mid  //4,0,0
// //                 config);


// //         Trajectory secondMove =
// //             TrajectoryGenerator.generateTrajectory(
// //                 // Start at the 5,5 facing Y+, for some reason, the "front" of the robot is the back so Y+ is straight back and X+ is left if you are driving the robot
// //                 new Pose2d(11,10, Rotation2d.fromDegrees(180)),
// //                 // Pass through these two interior waypoints, making an Circle
// //                 new ArrayList<>(), //List.of(new Translation2d(0, 0)), // These positions are absolute and not relative
// //                 // End where it started facing the same direction the whole time.
// //                 new Pose2d(0, 10, Rotation2d.fromDegrees(180)),
// //                 config);

// //         //         Trajectory thirdMove =
// //         //         TrajectoryGenerator.generateTrajectory(
// //         //             // Start at the 5,5 facing Y+, for some reason, the "front" of the robot is the back so Y+ is straight back and X+ is left if you are driving the robot
// //         //             new Pose2d(0, 0, new Rotation2d(0)),
// //         //             // Pass through these two interior waypoints, making an Circle
// //         //             List.of(new Translation2d(0, 0)), // These positions are absolute and not relative
// //         //             // End where it started facing the same direction the whole time.
// //         //             new Pose2d(0, 0, new Rotation2d(0)),
// //         //             config);

// //         var thetaController =
// //             new ProfiledPIDController(
// //                 Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
// //         thetaController.enableContinuousInput(-Math.PI, Math.PI);

// //         SwerveControllerCommand swerveControllerCommand =
// //             new SwerveControllerCommand(
// //                 firstMove,
// //                 s_Swerve::getPose,
// //                 Constants.Swerve.swerveKinematics,
// //                 new PIDController(Constants.AutoConstants.kPXController, 0, 0),
// //                 new PIDController(Constants.AutoConstants.kPYController, 0, 0),
// //                 thetaController,
// //                 () -> { return Rotation2d.fromDegrees(0); },
// //                 s_Swerve::setModuleStates,
// //                 s_Swerve);


// //          var thetaController2 =
// //             new ProfiledPIDController(
// //                 Constants.AutoConstants.kPThetaController2, 0, 0, Constants.AutoConstants.kThetaControllerConstraints2);
// //         thetaController2.enableContinuousInput(-Math.PI, Math.PI);

// //         SwerveControllerCommand swerveControllerCommand2 =
// //             new SwerveControllerCommand(
// //                 secondMove,
// //                 s_Swerve::getPose,
// //                 Constants.Swerve.swerveKinematics,
// //                 new PIDController(Constants.AutoConstants.kPXController, 0, 0),
// //                 new PIDController(Constants.AutoConstants.kPYController, 0, 0),
// //                 thetaController2,
// //                 () -> { return Rotation2d.fromDegrees(0); },
// //                 s_Swerve::setModuleStates,
// //                 s_Swerve);

// //         //  var thetaController3 =
// //         //         new ProfiledPIDController(
// //         //             Constants.AutoConstants.kPThetaController2, 0, 0, Constants.AutoConstants.kThetaControllerConstraints3);
// //         //     thetaController3.enableContinuousInput(-Math.PI, Math.PI);
    
// //         //     SwerveControllerCommand swerveControllerCommand3 =
// //         //         new SwerveControllerCommand(
// //         //             thirdMove,
// //         //             s_Swerve::getPose,
// //         //             Constants.Swerve.swerveKinematics,
// //         //             new PIDController(Constants.AutoConstants.kPXController, 0, 0),
// //         //             new PIDController(Constants.AutoConstants.kPYController, 0, 0),
// //         //             thetaController3,
// //         //             s_Swerve::setModuleStates,
// //         //             s_Swerve);

// //         // addCommands(
// //         //     new InstantCommand(() -> s_Swerve.resetOdometry(firstMove.getInitialPose())),
// //         //     swerveControllerCommand
// //         // );

// //         // addCommands(
// //         //     new WaitCommand(1)
// //         // );

// //         // addCommands(
// //         //     new InstantCommand(() -> {Signaling.mode = 5;})
// //         // );

// //         // addCommands(
// //         //     new WaitCommand(3)
// //         // );

// //         // addCommands(
// //         //     new InstantCommand(() -> s_Swerve.resetOdometry(secondMove.getInitialPose())),
// //         //     swerveControllerCommand2
// //         // );

// //         // addCommands(
// //         //     new InstantCommand(() -> {Signaling.mode = 8;})
// //         // );

// //         // addCommands(
// //         //     new WaitCommand(1)
// //         // );
// //         //     addCommands(
// //         //         new InstantCommand(() -> s_Swerve.resetOdometry(thirdMove.getInitialPose())),
// //         //         swerveControllerCommand3
// //         //     );
      
// //         // addCommands(
// //         //     new WaitCommand(1)
// //         // );
// //         // addCommands(
// //         //     new InstantCommand(() -> {Signaling.mode = 12;})
// //         // );
      
        


// //         addCommands(
// //             new InstantCommand(() -> {Signaling.mode = 8;

// //             }),
// //             new InstantCommand(
// //                 () -> s_Swerve.resetOdometry(
// //                     new Pose2d(
// //                         firstMove.getInitialPose().getX(),
// //                         firstMove.getInitialPose().getY(), 
// //                         Rotation2d.fromDegrees(0)
// //                     )
// //                 )
// //             ),
// //             swerveControllerCommand,// swerveControllerCommand2
// //             // new InstantCommand(() -> {Signaling.mode = 11;}),
// //              new WaitCommand(3),
// //              new InstantCommand(() -> {Signaling.mode = 3;})
// //             // new WaitCommand(6)
            
// //             // new InstantCommand(() -> {Signaling.mode = 8;}),
// //             // new WaitCommand(5),
// //             // new InstantCommand(() -> {Signaling.mode = 3;}),
// //         //    new InstantCommand(() -> s_Swerve.resetOdometry(secondMove.getInitialPose())),


            
// //         );
        
// //     }
// // }




// // // addCommands(
// // //     new InstantCommand(() -> s_Swerve.resetOdometry(firstMove.getInitialPose())),
// // //     swerveControllerCommand,
// // //     new InstantCommand(() -> {Signaling.mode = 11;}),
// // //     new WaitCommand(3),
// // //     new InstantCommand(() -> {Signaling.mode = 3;})
// // //     // new WaitCommand(6)
    
// // //     // new InstantCommand(() -> {Signaling.mode = 8;}),
// // //     // new WaitCommand(5),
// // //     // new InstantCommand(() -> {Signaling.mode = 3;}),
// // // //    new InstantCommand(() -> s_Swerve.resetOdometry(secondMove.getInitialPose())),









// // package frc.robot.autos;

// // import frc.robot.Constants;
// // import frc.robot.Signaling;
// // import frc.robot.subsystems.Swerve;
// // import pabeles.concurrency.ConcurrencyOps.NewInstance;

// // import java.util.ArrayList;
// // import java.util.List;

// // import edu.wpi.first.math.controller.PIDController;
// // import edu.wpi.first.math.controller.ProfiledPIDController;
// // import edu.wpi.first.math.geometry.Pose2d;
// // import edu.wpi.first.math.geometry.Rotation2d;
// // import edu.wpi.first.math.geometry.Translation2d;
// // import edu.wpi.first.math.trajectory.Trajectory;
// // import edu.wpi.first.math.trajectory.TrajectoryConfig;
// // import edu.wpi.first.math.trajectory.TrajectoryGenerator;
// // import edu.wpi.first.wpilibj2.command.InstantCommand;
// // import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// // import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
// // import edu.wpi.first.wpilibj2.command.WaitCommand;

// // public class exampleAuto extends SequentialCommandGroup {
// //     public exampleAuto(Swerve s_Swerve){
// //         TrajectoryConfig config =
// //             new TrajectoryConfig(
// //                     .75,
// //                     .4)
// //                 .setKinematics(Constants.Swerve.swerveKinematics);

// //         // An example trajectory to follow.  All units in meters.
// //         // Trajectory firstMove =
// //         //     TrajectoryGenerator.generateTrajectory(
// //         //         // Start at the 5,5 facing Y+, for some reason, the "front" of the robot is the back so Y+ is straight back and X+ is left if you are driving the robot
// //         //         new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
// //         //         // Pass through these two interior waypoints, making an Circle
// //         //         List.of(new Translation2d(3, 0), new Translation2d(4, 1), new Translation2d(6.72, 1), new Translation2d(6.72, 0)), // These positions are absolute and not relative
// //         //         new Pose2d(0, 0, Rotation2d.fromDegrees(0)),// 4.25 for left, 5 for right, 4 for mid
// //         //         config);
// //                 Trajectory firstMove =
// //                 TrajectoryGenerator.generateTrajectory(
// //                     // Start at the 5,5 facing Y+, for some reason, the "front" of the robot is the back so Y+ is straight back and X+ is left if you are driving the robot
// //                     new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
// //                     // Pass through these two interior waypoints, making an Circle
// //                     List.of(new Translation2d(3, 0), new Translation2d(5, 0)),// These positions are absolute and not relative
// //                     new Pose2d(6, 0, Rotation2d.fromDegrees(0)),// 4.25 for left, 5 for right, 4 for mid
// //                     config);

// //         var thetaController =
// //             new ProfiledPIDController(
// //                 Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
// //         thetaController.enableContinuousInput(-Math.PI, Math.PI);

// //         SwerveControllerCommand swerveControllerCommand =
// //             new SwerveControllerCommand(
// //                 firstMove,
// //                 s_Swerve::getPose,
// //                 Constants.Swerve.swerveKinematics,
// //                 new PIDController(Constants.AutoConstants.kPXController, 0, 0),
// //                 new PIDController(Constants.AutoConstants.kPYController, 0, 0),
// //                 thetaController,
// //                 s_Swerve::setModuleStates,
// //                 s_Swerve);

     
       
// //         addCommands(

// //             new InstantCommand(
// //               () -> s_Swerve.resetOdometry(
// //             new Pose2d(
// //                 firstMove.getInitialPose().getX(),
// //                 firstMove.getInitialPose().getY(), 
// //                 Rotation2d.fromDegrees(0)
// //             )
// //         )
// //     ),
// //             swerveControllerCommand
       
// //         );

// //     }
// // }







// package frc.robot.autos;

// import frc.robot.Constants;
// import frc.robot.Signaling;
// import frc.robot.subsystems.Swerve;
// import pabeles.concurrency.ConcurrencyOps.NewInstance;

// import java.util.ArrayList;
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
//                     .75,
//                     .4)
//                 .setKinematics(Constants.Swerve.swerveKinematics);

//         // An example trajectory to follow.  All units in meters.
//         // Trajectory firstMove =
//         //     TrajectoryGenerator.generateTrajectory(
//         //         // Start at the 5,5 facing Y+, for some reason, the "front" of the robot is the back so Y+ is straight back and X+ is left if you are driving the robot
//         //         new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
//         //         // Pass through these two interior waypoints, making an Circle
//         //         List.of(new Translation2d(3, 0), new Translation2d(4, 1), new Translation2d(6.72, 1), new Translation2d(6.72, 0)), // These positions are absolute and not relative
//         //         new Pose2d(0, 0, Rotation2d.fromDegrees(0)),// 4.25 for left, 5 for right, 4 for mid
//         //         config);
//         Trajectory firstMove =
//         TrajectoryGenerator.generateTrajectory(
//             // Start at the 5,5 facing Y+, for some reason, the "front" of the robot is the back so Y+ is straight back and X+ is left if you are driving the robot
//             new Pose2d(0, 0, new Rotation2d(0)),
//             // Pass through these two interior waypoints, making an Circle
//             List.of(new Translation2d(0, 0)), // These positions are absolute and not relative
//             // End where it started facing the same direction the whole time.
//             new Pose2d(4.7, 0, new Rotation2d(0)),// 4.25 for left, 5 for right, 4 for mid
//             config);

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

     
       
//         addCommands(

//             new InstantCommand(
//               () -> s_Swerve.resetOdometry(
//             new Pose2d(
//                 firstMove.getInitialPose().getX(),
//                 firstMove.getInitialPose().getY(), 
//                 Rotation2d.fromDegrees(0)
//             )
//         )
//     ),
//             swerveControllerCommand
       
//         );

//     }
// }