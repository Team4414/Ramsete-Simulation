import java.util.Random;

public class DriveTrain {

    public DriveTrainGearbox left;
    public DriveTrainGearbox right;
    double kWheelBase;

    double kDt;
    double pos_;
    double posx_;
    double posy_;
    double vel_;
    double angle_ = 0;
    double anglev_;

    
    public DriveTrain(DriveTrainGearbox left, DriveTrainGearbox right,double kWheelBase, double kDt) {
        this.left = left;
        this.right = right;
        this.kWheelBase = kWheelBase;
        this.kDt = kDt;
    }


    public void calculate() {
        //Estimates heading... should be recorded for real robot
        anglev_ = (1/kWheelBase) * (right.velocity_ - left.velocity_);
        angle_ += (anglev_ *kDt) ;

        // the pos is the average of the 2 sides
        double d_pos_ =( left.velocity_ *kDt + right.velocity_ *kDt ) /2;
        pos_ += d_pos_;
        vel_ = (left.velocity_ + right.velocity_) /2 ;

        //use trig and linear approximation to find x and y
        posx_ += d_pos_ * Math.cos(angle_);
        posy_ += d_pos_ * Math.sin(angle_);


    }

    public double getPos_() {
        return pos_;
    }

    public double getAngle_() {
        return angle_;
    }

    public double getPosx_() {
        return posx_;
    }

    public double getPosy_() {
        return posy_;
    }





}