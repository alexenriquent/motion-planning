package solution;

import java.io.IOException;
import java.util.List;

import problem.ArmConfig;
import problem.ProblemSpec;

public class Program {

	public static void main(String[] args) throws IOException {
		
		ProblemSpec problem = new ProblemSpec();
		RRT RRT = new RRT();
		
		problem.loadProblem(args[0]);

		double start = System.nanoTime();
		
		double s1 = System.nanoTime();
		List<ArmConfig> path = RRT.search(problem);
		double f1 = System.nanoTime();
		double e1 = (f1 - s1) / 10e5;
		
		double s2 = System.nanoTime();
		List<ArmConfig> interpolatedPath = RRT.interpolate(problem, path);
		double f2 = System.nanoTime();
		double e2 = (f2 - s2) / 10e5;
		
		double finish = System.nanoTime();
		double elapsed = (finish - start) / 10e5;
		
		System.out.println("Total: " + elapsed);
		System.out.println("Search: " + e1);
		System.out.println("Interpolation: " + e2);
						
		problem.setPath(interpolatedPath);
		problem.saveSolution(args[1]);
	}
}
