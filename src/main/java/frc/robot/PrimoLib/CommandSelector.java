package frc.robot.PrimoLib;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** Add your docs here. */
public class CommandSelector {
    SendableChooser<Command> chooser;
    Map<String,Command> commands;


    public CommandSelector(Map<String,Command> commands,String name) {
        List<Entry<String, Command>> commandList = new ArrayList<>();
        this.commands = commands;
        chooser = new SendableChooser<>();
        
        // fills commandList with commands and their names
        for(Entry<String,Command> e : commands.entrySet()){
            commandList.add(e);
        }

        chooser.setDefaultOption(commandList.get(0).getKey(), commandList.get(0).getValue()); // command name + command itself
        for(int i = 1; i < commandList.size();i++){
            chooser.addOption(commandList.get(i).getKey(), commandList.get(i).getValue());
        }
        SmartDashboard.putData(name, chooser);    
    }

    public void setCommands(Map<String,Command> commands) {
        
    }

    // starts the selected command
    public Command getCommand() {
        return chooser.getSelected();
    }
}