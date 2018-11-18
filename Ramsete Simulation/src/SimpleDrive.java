import java.util.Random;

import jaci.pathfinder.Pathfinder;

public class SimpleDrive {
	
	double x;
	double y;
	double heading;
	double wheelbase;
	double kDt;
	
	
	
	public SimpleDrive(double x, double y, double heading, double wheelbase, double kDt) {
        this.x = x;
        this.y = y;
        this.heading = heading;
        this.wheelbase = wheelbase;
        this.kDt = kDt;
    }
	
	public void calculate(double v, double w) {
		
		heading += w * kDt;
		x += v * Math.cos(heading) * kDt;
		y += v * Math.sin(heading) * kDt;
	}
}
