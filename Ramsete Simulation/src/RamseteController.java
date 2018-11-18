import jaci.pathfinder.Trajectory;

public class RamseteController {
	
	private Trajectory path;
	private boolean isFinished = false;
	private int seg = 0;
	
	
	public RamseteController (Trajectory path) {
		this.path = path;
		
	}
	
	
	public void calculate(DriveObject Drive) {
		
		if ( seg >= path.length() ) {
			isFinished = true;
		}
		double kZeta =.5;
		double kBeta = .7;

		
		// This is the constant function... read the paper for more info
		double k = 2.0 * kZeta * Math.sqrt(Math.pow(wd, 2.0) + kBeta * Math.pow(vd, 2.0));

		double ramv = vd * Math.cos(eAngle) + k * (Math.cos(angle) * (gx - rx) + Math.sin(angle) * (gy - ry));
		double ramw = wd + kBeta * vd * (Math.sin(eAngle) / (eAngle)) * (Math.cos(angle) * (gy - ry) - Math.sin(angle) * (gx - rx)) + k * (eAngle);
		double angleError = (gAngle - angle) ;
		
	}
	
	
}
