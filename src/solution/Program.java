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
		List<ArmConfig> interpolatedPath = RRT.primitiveSteps(problem, path);
		
		double finish = System.nanoTime();
		
		System.out.println("Search: " + ((f1 - s1) / 10e5) + " ms");
		System.out.println("Total: " + ((finish - start) / 10e5) + " ms");
						
		problem.setPath(interpolatedPath);
		problem.saveSolution(args[1]);
	}
}
