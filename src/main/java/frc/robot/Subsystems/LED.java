// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.trobot5013lib.led.BlinkingPattern;
import frc.robot.trobot5013lib.led.ChasePattern;
import frc.robot.trobot5013lib.led.RainbowPattern;
import frc.robot.trobot5013lib.led.SolidColorPattern;
import frc.robot.trobot5013lib.led.TrobotAddressableLED;
import frc.robot.trobot5013lib.led.TrobotAddressableLEDPattern;
import frc.robot.trobot5013lib.led.IntensityPattern;
import frc.robot.trobot5013lib.led.ChaosPattern;
import frc.robot.trobot5013lib.led.ScannerPattern;
import frc.robot.trobot5013lib.led.AlternatingColorPattern;

public class LED extends SubsystemBase {

	private TrobotAddressableLED m_led = new TrobotAddressableLED(Constants.LEDConstants.LED_PWM_PORT, Constants.LEDConstants.LED_LENGTH);
	private boolean noteIndicator = false;
	private RobotContainer m_RobotContainer;
	private int intensityDegrees = 1;
    private Color[] redWhiteArray = {Color.kRed, Color.kBlack};
    private Color[] blueWhiteArray = {Color.kBlue, Color.kBlack};
	private Color[] pinkOrangeArray = {Constants.LEDConstants.pink, Constants.LEDConstants.orange};
	private Color[] blackOrangeArray = {Color.kBlack, Constants.LEDConstants.orange};
	private Color[] blueOrangeArray = {Color.kBlue, Constants.LEDConstants.orange};

	private Color[] redWhiteBlueArray = {Color.kRed, Color.kWhite, Color.kBlue};

	private TrobotAddressableLEDPattern m_bluePattern = new SolidColorPattern(Color.kBlue);
	private TrobotAddressableLEDPattern m_redPattern = new SolidColorPattern(Color.kRed);

	private TrobotAddressableLEDPattern m_alternatingColorPattern = new AlternatingColorPattern(redWhiteBlueArray);
	private TrobotAddressableLEDPattern m_blinkingRed = new BlinkingPattern(Color.kRed, 0.25);
	private TrobotAddressableLEDPattern m_blinkingGreen = new BlinkingPattern(Color.kGreen, 0.25);
	private TrobotAddressableLEDPattern m_chaosPattern = new ChaosPattern();
    private TrobotAddressableLEDPattern m_redChasePattern = new ChasePattern(redWhiteArray, 3);
    private TrobotAddressableLEDPattern m_blueChasePattern = new ChasePattern(blueWhiteArray, 3);
	private TrobotAddressableLEDPattern m_orangeChasePattern = new ChasePattern(blueOrangeArray, 6);
	private TrobotAddressableLEDPattern m_redWhiteBlueChasePattern = new ChasePattern(redWhiteBlueArray, 2);
	private IntensityPattern m_blueIntensityPattern = new IntensityPattern(Color.kBlue, intensityDegrees);
	private IntensityPattern m_redIntensityPattern = new IntensityPattern(Color.kRed, intensityDegrees);
	private IntensityPattern m_greenIntensityPattern = new IntensityPattern(Color.kGreen, intensityDegrees);
	private IntensityPattern m_offIntensityPattern = new IntensityPattern(Color.kBlack, 0);
	private TrobotAddressableLEDPattern m_rainbowPattern = new RainbowPattern();
	private TrobotAddressableLEDPattern m_scannerPattern = new ScannerPattern(Color.kChartreuse,Color.kDeepPink,8);
	private TrobotAddressableLEDPattern m_greenPattern = new SolidColorPattern(Color.kGreen);
	private TrobotAddressableLEDPattern m_yellowPattern = new SolidColorPattern(Color.kLightYellow);
	private TrobotAddressableLEDPattern m_purplePattern = new SolidColorPattern(Color.kPurple);
	
	private TrobotAddressableLEDPattern m_disabledPattern = m_offIntensityPattern;
	
	/** Creates a new StatusLED. */
	public LED(RobotContainer robotContainer) {
		super();
		m_RobotContainer = robotContainer;
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		// set patterns to run based on various statuses of the robot via the m_RobotContainer
		if (m_RobotContainer.isDisabled()) {
			m_led.setPattern(m_disabledPattern);
		} else if (m_RobotContainer.isRedAlliance()) {
			if(noteIndicator){
				m_led.setPattern(m_greenIntensityPattern);
			}else{
				m_led.setPattern(m_redIntensityPattern);
			}
		} else {
			if(noteIndicator){
				m_led.setPattern(m_greenIntensityPattern);
			}else{
				m_led.setPattern(m_blueIntensityPattern);
			}
		}

	}

	public void capturedNote(){
		noteIndicator = true;
	}
	
	public void noNote(){
		noteIndicator = false;
	}
}
