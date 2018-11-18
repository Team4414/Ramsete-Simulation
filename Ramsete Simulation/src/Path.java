import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;

public class Path {

	
    Waypoint[] points = new Waypoint[] {

            new Waypoint(0, 0, Pathfinder.d2r(180)), // Waypoint @ x=0, y=0, exit angle=0 radians						// angle=-45 degrees
            new Waypoint(-2, -2, Pathfinder.d2r(180)), // Waypoint @ x=-2, y=-2, exit angle=0 radians				// angle=-45 degrees
            new Waypoint(-4, 0, Pathfinder.d2r(90)),
            new Waypoint(-2, 2, Pathfinder.d2r(0)),
            new Waypoint(2, 0, Pathfinder.d2r(-90)),
            new Waypoint(0, -4, Pathfinder.d2r(-180))
            
    };

    public Trajectory.Config config;
    public Trajectory trajectory;

    public Path() {
        this.config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH,
                Main.kDt, 4, 4, 10000);
        this.trajectory = Pathfinder.generate(points, config);
    }

    public Trajectory getTrajectory() {
        return trajectory;
    }
    
}