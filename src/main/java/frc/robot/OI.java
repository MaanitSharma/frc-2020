package frc.robot;
import javax.swing.plaf.basic.BasicOptionPaneUI.ButtonAreaLayout;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;

public class OI{
    //Creating Buttons
    public static Joystick ps4joy = new Joystick(Constants.ps4_port);

    public OI(){

    }

    public static double getPs4LeftYaxis() {
        return ps4joy.getRawAxis(Constants.ps4LeftYaxis);
    }

    public static double getPs4RightYaxis() {
        return ps4joy.getRawAxis(Constants.ps4RightYaxis);
    }

    public boolean getPs4ShooterButton(){
        return ps4joy.getRawButtonPressed(Constants.ps4ShooterButton);
    }

    public boolean getPs4AutoAdjustButton(){
        return ps4joy.getRawButtonPressed(Constants.ps4AutoAdjustButton);
    }

    public boolean getPs4ColorSensorButton()
    {
        return ps4joy.getRawButtonPressed(Constants.ps4ColorSensorButton);
    }

    public boolean getPs4IntakeOuttakeButton()
    {
        return ps4joy.getRawButtonPressed(Constants.ps4IntakeOuttakeButton);
    }

    public boolean getPs4IntakeIntakeButton()
    {
        return ps4joy.getRawButtonPressed(Constants.ps4IntakeIntakeButton);
    }

    public boolean getPs4IntakeDeployButton()
    {
        return ps4joy.getRawButtonPressed(Constants.ps4IntakeDeployButton);
    }

}
