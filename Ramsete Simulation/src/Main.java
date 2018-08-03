import java.io.File;
import java.io.IOException;
import java.util.ArrayList;

import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;

public class Main {

	public static void main(String[] args) {

		double kDt = 0.05;
		// increment for simulation
		int seg = 0;

		// Drivetrain object used for simulation
		DriveTrain drive = new DriveTrain(new DriveTrainGearbox(0.0, 0.0, kDt), new DriveTrainGearbox(0.0, 0.0, kDt),
				.066, kDt);

		// Path for simulation
		Trajectory path = new Path().getTrajectory();

		// data to log
		ArrayList<Object> type = new ArrayList<>();
		ArrayList<Object> x = new ArrayList<>();
		ArrayList<Object> y = new ArrayList<>();
		ArrayList<Object> v = new ArrayList<>();
		ArrayList<Object> t = new ArrayList<>();
		ArrayList<Object> a = new ArrayList<>();
		ArrayList<Object> p = new ArrayList<>();

		/*
		 * So there is 2 ways to derive wd and vd...
		 * 
		 * The first is to use equations (as used in this example)
		 * 
		 * The second way is to simply use gVel for vd and (gAngle - last_gAngle)/2 for
		 * wd
		 * 
		 * Method 2 is so much easier... but for sake of example I did what the white
		 * paper said
		 */

		double last_gAngle = 0;
		double gxlast = 0;
		double gylast = 0;
		double gxdlast = 0;
		double gydlast = 0;

		while (seg <= path.length() - 1) {

			// This is the path vel and can be used for vd
			double gVel = path.get(seg).velocity;
			
			
			// This is the alternative way to calculate gAngleDot or wd
			// double gAngleDot = gAngle - last_gAngle;
			// last_gAngle = gAngle;
			
			//ez stuff
			double gx = path.get(seg).x;
			double gy = path.get(seg).y;
			double gXDot = (gx - gxlast) / kDt;
			double gYDot = (gy - gylast) / kDt;
			double gXDDot = (gXDot - gxdlast) / kDt;
			double gYDDot = (gYDot - gydlast) / kDt;
			gxlast = gx;
			gylast = gy;
			gydlast = gYDot;
			gxdlast = gXDot;
			
			// this is a STUPID way to calculate gAngle JUST CALL PATH.GET(SEG).HEADING... but for sake of... u get it
			double gAngle = Pathfinder.d2r(Pathfinder.boundHalfDegrees(Pathfinder.r2d(Math.atan2(gYDot, gXDot))));
			
			//robot localization is needed for this part and gyro can be used for angle
			double angle = Pathfinder.d2r(Pathfinder.boundHalfDegrees(Pathfinder.r2d(drive.angle_)));
			double rx = drive.posx_;
			double ry = drive.posy_;
			
			/*
			 *  These values tune your system
			 *   b is like P in a PID loop and creates a more responsive system--- b > 0
			 *   zeta is like D in a PID loop and will dampen your system--- Z = (0,1)
			 */
			double kzeta = 0.0;
			double kb = 20.0;

			/*
			 * This is the pointless code that is used to calculate your gVel and gAngleDot... just use jaci in the future 
			 */
			double vd = Math.sqrt(Math.pow(gXDot, 2) + Math.pow(gYDot, 2));
			double wd = (gYDDot * gXDot - gXDDot * gYDot) / (Math.pow(gXDot, 2) + Math.pow(gYDot, 2));
			
			
			// This is the constant function... read the paper for more info
			double k1 = 2.0 * kzeta * Math.sqrt(Math.pow(wd, 2.0) + kb * Math.pow(vd, 2.0));

			
			/*
			 * Here she is! Equation 5.12. if you notice while reading the white paper equation 5.12 is wrong... it improperly implements linear projection
			 * the correct implementation is (Math.cos(angle) * (gy - ry) - Math.sin(angle) * (gx - rx)) in the ramw		
			 */
			double ramv = vd * Math.cos(gAngle - angle)
					+ k1 * (Math.cos(angle) * (gx - rx) + Math.sin(angle) * (gy - ry));
			double ramw = wd + kb * vd * (Math.sin(gAngle - angle) / (gAngle - angle))
					* (Math.cos(angle) * (gy - ry) - Math.sin(angle) * (gx - rx)) + k1 * (gAngle - angle);

			// good ol differential kinematic  
			double velr = ramv + ramw * drive.kWheelBase / 2;
			double vell = ramv - ramw * drive.kWheelBase / 2;

			// Calculates new pos and vel
			// the  * 10.0 / 3.8 is a feed forward term and converts vel to voltage 
			drive.right.calculate(Math.min(10.0, Math.max(-10.0, velr * 10.0 / 3.8)));
			drive.left.calculate(Math.min(10.0, Math.max(-10.0, vell * 10.0 / 3.8)));

			// calculates pos and vel of whole model
			drive.calculate();

			// log all the things
			type.add("path");
			x.add(path.get(seg).x);
			y.add(path.get(seg).y);
			v.add(ramv);
			t.add(seg * kDt);
			a.add(Pathfinder.boundHalfDegrees(Pathfinder.r2d(path.get(seg).heading)));
			p.add(path.get(seg).position);

			type.add("robot");
			x.add(drive.getPosx_());
			y.add(drive.getPosy_());
			v.add(drive.vel_);
			t.add(seg * kDt);
			a.add(Pathfinder.boundHalfDegrees(Pathfinder.r2d(drive.angle_)));
			p.add(drive.getPos_());

			seg++;
		}

		// Logging stuffs
		ArrayList<CSVObject> list = new ArrayList<CSVObject>();

		list.add(new CSVObject("time", t));
		list.add(new CSVObject("type", type));
		list.add(new CSVObject("x", x));
		list.add(new CSVObject("y", y));
		list.add(new CSVObject("velocity", v));
		list.add(new CSVObject("angle", a));
		list.add(new CSVObject("pos", p));

		try {
			CSVHelper.writeCSV(list, new File("csv/output.csv"));
		} catch (IOException e) {
			System.out.println("!!! Main Could Not Write CSV File !!!");
		}

	}
}