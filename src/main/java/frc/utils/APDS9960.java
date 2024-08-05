package frc.utils;

import edu.wpi.first.wpilibj.I2C;
import frc.constants.VisionConstants;

public class APDS9960 {
  I2C i2c = new I2C(I2C.Port.kOnboard, VisionConstants.APDS9960_I2C_ADDRESS);

  public APDS9960() {
    init();
  }

  public void init() {
    // enables power, sensor, interrupt
    setAndPreserve((byte) 0x80, (byte) 0b00100101);
  }

  public void disableSensor() {
    i2c.write(0x80, 0x00);
  }

  public void clearInterrupts() {
    i2c.write((byte) 0xE5, (byte) 0);
  }

  public void setProximityPersistence(byte value) {
    value <<= 4;
    setAndPreserve((byte) 0x8C, value);
  }

  public void setProximityLowThreshold(byte thresh) {
    i2c.write((byte) 0x89, thresh);
  }

  public void setProximityHighThreshold(byte thresh) {
    i2c.write((byte) 0x8B, thresh);
  }

  /**
   * @param value If you'd like to set bit 2, use 0b01000000
   */
  private void setAndPreserve(byte reg, byte value) {
    byte[] read = new byte[1];
    i2c.read(reg, 1, read);
    i2c.write(reg, (byte) (read[0] | value));
  }

  /**
   * @param value If you'd like to unset bit 2, use 0b10111111
   */
  private void unsetAndPreserve(byte reg, byte value) {
    byte[] read = new byte[1];
    i2c.read(reg, 1, read);
    i2c.write(reg, (byte) (read[0] & value));
  }
}
