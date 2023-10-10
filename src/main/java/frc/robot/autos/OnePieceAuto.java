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




public class OnePieceAuto extends SequentialCommandGroup {
    public OnePieceAuto(Swerve s_Swerve){

 
//#######################################################################################################################


//loads path
PathPlannerTrajectory first = PathPlanner.loadPath("onePiece", new PathConstraints(1, 0.6));

//creates swerve controller command
PPSwerveControllerCommand swerveControllerCommand =
new PPSwerveControllerCommand(
    first,
    s_Swerve::getPose,
    Constants.Swerve.swerveKinematics,
    new PIDController(2, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.   
    //TODO try 0.5, 5, 0 at comp
    new PIDController(2, 0, 0), // Y controller (usually the same values as X controller)

    new PIDController(0, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
    s_Swerve::setModuleStates,
    false,
    s_Swerve);


addCommands(new InstantCommand(() -> s_Swerve.zeroGyro()));


addCommands(
    new InstantCommand(() -> {Signaling.mode = 13;})
);

addCommands(
    new WaitCommand(.75)
);
addCommands(
    new InstantCommand(() -> {Signaling.mode = 14;})
    
);

addCommands(
    new WaitCommand(.75)
);

addCommands(
    new InstantCommand(() -> {Signaling.mode = 7;})  //8
   
);


addCommands(
    new WaitCommand(1) //.25
);



addCommands(
    new InstantCommand(
    () -> s_Swerve.resetOdometry(
            new Pose2d(
                first.getInitialPose().getX(),
                first.getInitialPose().getY(),
                first.getInitialHolonomicPose().getRotation()
                ))),
                swerveControllerCommand
);

addCommands(new InstantCommand(() -> s_Swerve.zeroGyro()));

}
}