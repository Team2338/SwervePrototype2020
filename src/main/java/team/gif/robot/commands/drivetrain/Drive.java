/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package team.gif.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import team.gif.lib.math.MathUtil;
import team.gif.lib.math.Vector2d;
import team.gif.robot.OI;
import team.gif.robot.subsystems.Drivetrain;
import team.gif.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An example command that uses an example subsystem.
 */
public class Drive extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    //private final Drivetrain m_subsystem;
    private final Drivetrain drive = Drivetrain.getInstance();
    //private final Notifier notifier;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public Drive(Drivetrain subsystem) {
        //m_subsystem = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(Drivetrain.getInstance());
    }

    double leftSpeed;
    double rightSpeed;
    private final OI oi = OI.getInstance();

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double x = MathUtil.deadband(OI.getInstance().driver.getY(GenericHID.Hand.kLeft), 0.05);
        double y = MathUtil.deadband(OI.getInstance().driver.getX(GenericHID.Hand.kLeft), 0.05);
        double rotVal = MathUtil.deadband(OI.getInstance().driver.getX(GenericHID.Hand.kRight), 0.05);
        Vector2d transVecR = new Vector2d(x, y);
        drive.set(0, transVecR.scale(0.1), 0.1 * rotVal);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
       // Drivetrain.getInstance().setSpeed(0, 0);
        //notifier.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
