package frc.robot.subsystems;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;

public class Autos {
    private final DriveSubsystem m_drivetrainSubsystem;
    private SendableChooser<String> autoChooser;
    private HashMap<String, PathPlannerAuto> m_commandMap;
    ShuffleboardTab autoTab = Shuffleboard.getTab("Autonomous");


    public Autos(DriveSubsystem drivetrainSubsystem) 
  {
    m_drivetrainSubsystem = drivetrainSubsystem;
    //AutoConstants.eventMap.put("name of command to run along path", "command itself");
    
    autoChooser = new SendableChooser<>(); //auto chooser is the widget on shuffleboard so that you can select which auto you want
    autoTab.add(autoChooser);
    m_commandMap = new HashMap<>(); //the command map is where the auto are stored on a table

    autoChooser.addOption("OpenSide", "OpenSide");
    autoChooser.addOption("AmpSide", "AmpSide");
    autoChooser.addOption("ShootAndSit", "ShootAndSit");
    autoChooser.addOption("FourFast", "FourFast");


    m_commandMap.put("OpenSide", new PathPlannerAuto("OpenSide"));
    m_commandMap.put("AmpSide", new PathPlannerAuto("AmpSide"));
    m_commandMap.put("ShootAndSit", new PathPlannerAuto("ShootAndSit"));
    m_commandMap.put("FourFast", new PathPlannerAuto("FourFast"));
  }



    public Command getAutonomousCommand() {
      //This command is taking input from the widget on shuffleboard, looking it up on the command map table, and return a fully completed path for the robot to run
        String auto = autoChooser.getSelected();
        return m_commandMap.get(auto);
    }
}