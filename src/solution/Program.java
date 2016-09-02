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
		
		List<ArmConfig> path = RRT.search(problem);
		
		for (ArmConfig t : path) {
			System.out.println(t);
		}
		
		problem.setPath(path);
		problem.saveSolution(args[1]);
	}
}
