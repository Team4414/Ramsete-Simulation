public class DriveTrainGearbox {

    // Stall Torque in N m
    final double kStallTorque = 1.41;
    // Stall Current in Amps
    final double kStallCurrent = 89;
    // Free Speed in RPM
    final double kFreeSpeed = 5840;
    // Free Current in Amps
    final double kFreeCurrent = 3;
    // Mass of the Elevator
    final double kMass = 33.3;

    // Number of motors
    final double kNumMotors = 3.0;
    // Resistance of the motor
    final double kResistance = 12.0 / kStallCurrent;
    // Motor velocity constant
    final double Kv = ((kFreeSpeed / 60.0 * 2.0 * Math.PI) / (12.0 - kResistance * kFreeCurrent));
    // Torque constant
    final double Kt = (kNumMotors * kStallTorque) / kStallCurrent;
    // Gear ratio
    final double kG = 7.0;
    // Radius of pulley
    final double kr = 2.0 * (0.0254);// / Math.PI / 2.0);

    double position_;
    double velocity_;

    double kDt;

    public DriveTrainGearbox(double position_, double velocity_, double kDt) {
        this.position_ = position_;
        this.velocity_ = velocity_;
        this.kDt = kDt;
    }

    // thanks 971
    public double getAcceleration(double voltage, double velocity) {
        return -Kt * kG * kG / (Kv * kResistance * kr * kr * kMass) * velocity
                + kG * Kt / (kResistance * kr * kMass) * voltage;
    }

    //simple integration methods
    public void calculate(double voltage) {

        this.position_ += this.velocity_ * kDt;
        this.velocity_ += getAcceleration(voltage, this.velocity_) * kDt;
    }
   

    public double getPosition_() {
        return position_;
    }

    public double getVelocity_() {
        return velocity_;
    }

}