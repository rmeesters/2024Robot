package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.PneumaticsHandler;
import frc.robot.subsystems.Roller;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public class PrepareAmpCommand extends SequentialCommandGroup {

    private final Shooter s_Shooter = RobotContainer.s_Shooter;
    private final Swerve s_Swerve = RobotContainer.s_Swerve;
    private final Roller s_Roller = RobotContainer.s_Roller;
    private final Intake s_Intake = RobotContainer.s_Intake;
    private final PneumaticsHandler h_pneumatics = RobotContainer.h_pneumatics;
    
    public PrepareAmpCommand() {
        addCommands(
            new SequentialCommandGroup(
                //  * tilt
                new InstantCommand(() -> s_Shooter.setShaftRotation(Constants.Shooter.VERTICAL_POSITION)),
                new WaitCommand(2),
                new SequentialCommandGroup(
                    //  * pin off
                    //  * tilt on
                    //  * intake on
                    new InstantCommand(() -> {
                        h_pneumatics.setShooterSolenoid(false);
                        h_pneumatics.setTiltSolenoid(true);
                        s_Intake.setSpeed(0.3);
                    }),
                    //  * wait
                    new WaitCommand(1),
                    //  * pin on
                    new InstantCommand(() -> h_pneumatics.setShooterSolenoid(true)),
                    //  * wait
                    new WaitCommand(0.6),
                    //  * pin off
                    new InstantCommand(() -> h_pneumatics.setShooterSolenoid(false)),
                    //  * wait
                    new WaitCommand(0.6),
                    //  * shooter on
                    //  * roller on
                    //  * pin on
                    new InstantCommand(() -> {
                        s_Shooter.setSpeed(0.1);
                        s_Roller.setSpeed(-0.1);
                        h_pneumatics.setShooterSolenoid(true);
                    }),
                    //  * wait
                    new WaitCommand(0.6),
                    //  * intake off
                    //  * shooter off
                    //  * pin off
                    //  * roller off
                    new InstantCommand(() -> {
                        s_Shooter.setSpeed(0);
                        s_Intake.setSpeed(0);
                        s_Roller.setSpeed(0);
                        h_pneumatics.setShooterSolenoid(false);
                        h_pneumatics.setTiltSolenoid(false);
                    }))),
            //  * wait
            new WaitCommand(0.5),
            //  * tilt on
            new InstantCommand(() -> h_pneumatics.setTiltSolenoid(true)),
            //  * wait
            new WaitCommand(0.2),
            //  * reverse roller
            new InstantCommand(() -> s_Roller.setSpeed(0.2)),
            //  * wait
            new WaitCommand(1.5),
            //  * roller off
            //  * tilt off
            new InstantCommand(() -> {
                s_Roller.setSpeed(0);
                h_pneumatics.setTiltSolenoid(false);
            })
        );
    }

}
