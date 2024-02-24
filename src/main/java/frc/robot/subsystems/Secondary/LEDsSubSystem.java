// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Secondary;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDsSubSystem extends SubsystemBase {
  /** Creates a new LEDs. */
  private static LEDsSubSystem m_controllerRight = null;
  private static LEDsSubSystem m_controllerLeft = null;
  //private static Spark m_RightLED;
  //private static Spark m_LeftLED;
  public static double LEDPattern; //See 5 LED PATTERN TABLE: https://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf
  public static double LEDBlink;

  static double lastTime = -1;
  private static Spark m_RightLED = new Spark(0);
  private static Spark m_LeftLED = new Spark(1);

  public LEDsSubSystem() {
    // Must be a PWM header, not MXP or DIO
    // Spark m_RightLED = new Spark(0);
    // Spark m_LeftLED = new Spark(1);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Set the LEDs
    //m_RightLED.set(LEDPattern);
    //m_LeftLED.set(LEDPattern);
  }
  
  public static LEDsSubSystem getInstanceRight() {
    if (m_controllerRight == null) m_controllerRight = new LEDsSubSystem();
    return m_controllerRight;
  }

  public static LEDsSubSystem getInstanceLeft() {
    if (m_controllerLeft == null) m_controllerLeft = new LEDsSubSystem();
    return m_controllerLeft;
  }

  public static Command setLED(double LEDPattern) {
    m_RightLED.set(LEDPattern);
    m_LeftLED.set(LEDPattern);
    return null;
  }

  public static Command setLEDwBlink(double LEDPattern, double LEDBlink) {
      if (lastTime != -1 && Timer.getFPGATimestamp() - lastTime <= LEDBlink) {
        m_RightLED.set(LEDPattern);
        m_LeftLED.set(LEDPattern);
      }
      else if (Timer.getFPGATimestamp() - lastTime <= LEDBlink * 2) {
        m_RightLED.set(.99);
        m_LeftLED.set(.99);
      }      
      else {
        lastTime = Timer.getFPGATimestamp();
      }
      return null;
    };
}

// 1	1005	-0.99	Fixed Palette Pattern Rainbow
// 2	1015	-0.97	Fixed Palette Pattern Rainbow
// 3	1025	-0.95	Fixed Palette Pattern Rainbow
// 4	1035	-0.93	Fixed Palette Pattern Rainbow
// 5	1045	-0.91	Fixed Palette Pattern Rainbow
// 6	1055	-0.89	Fixed Palette Pattern Rainbow with Glitter Pattern Density Speed Brightness
// 7	1065	-0.87	Fixed Palette Pattern Confetti Pattern Density Speed Brightness
// 8	1075	-0.85	Fixed Palette Pattern Shot
// 9	1085	-0.83	Fixed Palette Pattern Shot
// 10	1095	-0.81	Fixed Palette Pattern Shot
// 11	1105	-0.79	Fixed Palette Pattern Sinelon
// 12	1115	-0.77	Fixed Palette Pattern Sinelon
// 13	1125	-0.75	Fixed Palette Pattern Sinelon
// 14	1135	-0.73	Fixed Palette Pattern Sinelon
// 15	1145	-0.71	Fixed Palette Pattern Sinelon
// 16	1155	-0.69	Fixed Palette Pattern Beats per Minute
// 17	1165	-0.67	Fixed Palette Pattern Beats per Minute
// 18	1175	-0.65	Fixed Palette Pattern Beats per Minute
// 19	1185	-0.63	Fixed Palette Pattern Beats per Minute
// 20	1195	-0.61	Fixed Palette Pattern Beats per Minute
// 21	1205	-0.59	Fixed Palette Pattern Fire
// 22	1215	-0.57	Fixed Palette Pattern Fire
// 23	1225	-0.55	Fixed Palette Pattern Twinkles
// 24	1235	-0.53	Fixed Palette Pattern Twinkles
// 25	1245	-0.51	Fixed Palette Pattern Twinkles
// 26	1255	-0.49	Fixed Palette Pattern Twinkles
// 27	1265	-0.47	Fixed Palette Pattern Twinkles
// 28	1275	-0.45	Fixed Palette Pattern Color Waves
// 29	1285	-0.43	Fixed Palette Pattern Color Waves
// 30	1295	-0.41	Fixed Palette Pattern Color Waves
// 31	1305	-0.39	Fixed Palette Pattern Color Waves
// 32	1315	-0.37	Fixed Palette Pattern Color Waves
// 33	1325	-0.35	Fixed Palette Pattern Larson Scanner
// 34	1335	-0.33	Fixed Palette Pattern Larson Scanner
// 35	1345	-0.31	Fixed Palette Pattern Light Chase
// 36	1355	-0.29	Fixed Palette Pattern Light Chase
// 37	1365	-0.27	Fixed Palette Pattern Light Chase
// 38	1375	-0.25	Fixed Palette Pattern Heartbeat
// 39	1385	-0.23	Fixed Palette Pattern Heartbeat
// 40	1395	-0.21	Fixed Palette Pattern Heartbeat
// 41	1405	-0.19	Fixed Palette Pattern Heartbeat
// 42	1415	-0.17	Fixed Palette Pattern Breath
// 43	1425	-0.15	Fixed Palette Pattern Breath
// 44	1435	-0.13	Fixed Palette Pattern Breath
// 45	1445	-0.11	Fixed Palette Pattern Strobe
// 46	1455	-0.09	Fixed Palette Pattern Strobe
// 47	1465	-0.07	Fixed Palette Pattern Strobe
// 48	1475	-0.05	Fixed Palette Pattern Strobe
// 49	1485	-0.03	Color 1 Pattern End to End Blend to Black 
// 50	1495	-0.01	Color 1 Pattern Larson Scanner Pattern Width Speed Brightness
// 51	1505	0.01	Color 1 Pattern Light Chase Dimming Speed Brightness
// 52	1515	0.03	Color 1 Pattern Heartbeat Slow 
// 53	1525	0.05	Color 1 Pattern Heartbeat Medium 
// 54	1535	0.07	Color 1 Pattern Heartbeat Fast 
// 55	1545	0.09	Color 1 Pattern Breath Slow 
// 56	1555	0.11	Color 1 Pattern Breath Fast 
// 57	1565	0.13	Color 1 Pattern Shot 
// 58	1575	0.15	Color 1 Pattern Strobe 
// 59	1585	0.17	Color 2 Pattern End to End Blend to Black 
// 60	1595	0.19	Color 2 Pattern Larson Scanner Pattern Width Speed Brightness
// 61	1605	0.21	Color 2 Pattern Light Chase Dimming Speed Brightness
// 62	1615	0.23	Color 2 Pattern Heartbeat Slow 
// 63	1625	0.25	Color 2 Pattern Heartbeat Medium 
// 64	1635	0.27	Color 2 Pattern Heartbeat Fast 
// 65	1645	0.29	Color 2 Pattern Breath Slow 
// 66	1655	0.31	Color 2 Pattern Breath Fast 
// 67	1665	0.33	Color 2 Pattern Shot 
// 68	1675	0.35	Color 2 Pattern Strobe 
// 69	1685	0.37	Color 1 and 2 Pattern Sparkle
// 70	1695	0.39	Color 1 and 2 Pattern Sparkle
// 71	1705	0.41	Color 1 and 2 Pattern Color Gradient
// 72	1715	0.43	Color 1 and 2 Pattern Beats per Minute
// 73	1725	0.45	Color 1 and 2 Pattern End to End Blend
// 74	1735	0.47	Color 1 and 2 Pattern End to End Blend 
// 75	1745	0.49	Color 1 and 2 Pattern Color 1 and Color 2 no blending
// 76	1755	0.51	Color 1 and 2 Pattern Twinkles
// 77	1765	0.53	Color 1 and 2 Pattern Color Waves
// 78	1775	0.55	Color 1 and 2 Pattern Sinelon
// 79	1785	0.57	Solid Colors Hot Pink 
// 80	1795	0.59	Solid Colors Dark red 
// 81	1805	0.61	Solid Colors Red 
// 82	1815	0.63	Solid Colors Red Orange 
// 83	1825	0.65	Solid Colors Orange 
// 84	1835	0.67	Solid Colors Gold 
// 85	1845	0.69	Solid Colors Yellow 
// 86	1855	0.71	Solid Colors Lawn Green 
// 87	1865	0.73	Solid Colors Lime 
// 88	1875	0.75	Solid Colors Dark Green 
// 89	1885	0.77	Solid Colors Green 
// 90	1895	0.79	Solid Colors Blue Green 
// 91	1905	0.81	Solid Colors Aqua 
// 92	1915	0.83	Solid Colors Sky Blue 
// 93	1925	0.85	Solid Colors Dark Blue 
// 94	1935	0.87	Solid Colors Blue 
// 95	1945	0.89	Solid Colors Blue Violet 
// 96	1955	0.91	Solid Colors Violet 
// 97	1965	0.93	Solid Colors White 
// 98	1975	0.95	Solid Colors Gray 
// 99	1985	0.97	Solid Colors Dark Gray 
// 100	1995	0.99	Solid Colors Black 
