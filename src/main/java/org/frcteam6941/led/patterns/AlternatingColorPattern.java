// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frcteam6941.led.patterns;

import org.frcteam6941.led.AddressableLEDPattern;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

/** Add your docs here. */
public class AlternatingColorPattern implements AddressableLEDPattern{
	private Color[] m_colors;

	public AlternatingColorPattern(Color[] colors){
		super();
		this.m_colors = colors;
	}

	@Override
	public void setLEDs(AddressableLEDBuffer buffer) {	
		for (int index = 0; index < buffer.getLength(); index++){
			buffer.setLED(index, m_colors[index % m_colors.length]);
		}	
	}
}
