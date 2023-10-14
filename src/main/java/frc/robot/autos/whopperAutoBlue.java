package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.Signaling;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;




public class whopperAutoBlue extends SequentialCommandGroup {
    public whopperAutoBlue(Swerve s_Swerve){

 
//#######################################################################################################################


//loads path
PathPlannerTrajectory first = PathPlanner.loadPath("whopperOne", new PathConstraints(3, 1.5));

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



    //loads path
PathPlannerTrajectory second = PathPlanner.loadPath("whopperTwo", new PathConstraints(3, 1));

//creates swerve controller command
PPSwerveControllerCommand swerveControllerCommand2 =
new PPSwerveControllerCommand(
    second,
    s_Swerve::getPose,
    Constants.Swerve.swerveKinematics,
    new PIDController(2, 10, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.   
    //TODO try 0.5, 5, 0 at comp
    new PIDController(2, 10, 0), // Y controller (usually the same values as X controller)

    new PIDController(0, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
    s_Swerve::setModuleStates,
    false,
    s_Swerve);


    // PathPlannerTrajectory third = PathPlanner.loadPath("whopper2ndCube", new PathConstraints(1.5, 0.75));

    // PPSwerveControllerCommand swerveControllerCommand3 =
    // new PPSwerveControllerCommand(
    //     third,
    //     s_Swerve::getPose,
    //     Constants.Swerve.swerveKinematics,
    //     new PIDController(0, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.   
    //     //TODO try 0.5, 5, 0 at comp
    //     new PIDController(0, 0, 0), // Y controller (usually the same values as X controller)
    
    //     new PIDController(0, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
    //     s_Swerve::setModuleStates,
    //     false,
    //     s_Swerve);

//#######################################################################################################################
   
//auto commands


addCommands(
    new InstantCommand(() -> {s_Swerve.zeroGyro();})
);

addCommands(
    new InstantCommand(() -> {Signaling.mode = 13;})//arms up
);

addCommands(
    new WaitCommand(.5)
);
addCommands(
    new InstantCommand(() -> {Signaling.mode = 14;})//wrist out cone
    
);

addCommands(
    new WaitCommand(1.5)
);

addCommands(
    new InstantCommand(() -> {Signaling.mode = 7;})  //score cone
   
);

addCommands(
    new WaitCommand(.75)
);

addCommands(new SequentialCommandGroup(
    new InstantCommand(() -> {Signaling.mode = 3;}),//compliance
    new WaitCommand(1),
    new InstantCommand(() -> {Signaling.mode = 17;}),
    new InstantCommand(
    () -> s_Swerve.resetOdometry(
            new Pose2d(
                first.getInitialPose().getX(),
                first.getInitialPose().getY(),
                first.getInitialHolonomicPose().getRotation()))),
                swerveControllerCommand  
));


addCommands(//reset odometry, move to cube
  
);

addCommands(
//     // new InstantCommand(() -> {Signaling.mode = 17;}),

//     // new WaitCommand(2.5),
    swerveControllerCommand2
);

addCommands(
    new InstantCommand(() -> {Signaling.mode = 3;})
    // new InstantCommand(() -> {Signaling.mode = 15;})  
);

addCommands(
    new InstantCommand(() -> {Signaling.mode = 18;})
);
addCommands(
    new WaitCommand(.25)
);
// addCommands(
//     new InstantCommand(() -> {Signaling.mode = 15;})
// );
// addCommands(
//     new InstantCommand(() -> {Signaling.mode = 8;})

// );

addCommands(
    new InstantCommand(() -> {Signaling.mode = 3;})
);    

addCommands(
    new InstantCommand(() -> {s_Swerve.zeroGyroAuto();})
);

    }
}
