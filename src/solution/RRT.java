package solution;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import problem.ArmConfig;
import problem.Obstacle;
import problem.ProblemSpec;

public class RRT {

	public static final int MAX_GRIPPERS = 4;
	public static final int MAX_VERTICES = 50;
	public static final double MAX_ERROR = 1e-5;
	public static final double INTERPOLATION = 3000.0;
	public static final double TRIAL_INTERPOLATION = 180.0;
	public static final double MAX_JOINT_ANGLE = 150 * Math.PI / 180.0;
	public static final double MAX_JOINT_STEP = 0.1 * Math.PI / 180.0;
	public static final double MAX_BASE_STEP = 0.001;
	public static final double MAX_GRIPPER_STEP = 0.001;
	public static final double MIN_GRIPPER_LENGTH = 0.03;
	public static final double MAX_GRIPPER_LENGTH = 0.07;
	public static final Rectangle2D BOUNDS = new Rectangle2D.Double(0, 0, 1, 1);
	
	private Rectangle2D lenientBounds;
	
	public RRT() {
		lenientBounds = grow(BOUNDS, MAX_ERROR);
	}
	
	public List<ArmConfig> RRTconnect(ProblemSpec problem) {
		Tree<ArmConfig> tree1 = new Tree<ArmConfig>();	
		tree1.add(new Node<ArmConfig>(null, problem.getInitialState()));
		Tree<ArmConfig> tree2 = new Tree<ArmConfig>();	
		tree2.add(new Node<ArmConfig>(null, problem.getGoalState()));
						
		while (true) {
			Node<ArmConfig> parent1 = adjacent(tree1, tree2.get(tree2.size() - 1).getData());
			Node<ArmConfig> parent2 = adjacent(tree2, parent1.getData());
			if (!collision(problem, parent1.getData(), parent2.getData()) ||
				!collision(problem, parent2.getData(), parent1.getData())) {
				tree1.add(new Node<ArmConfig>(parent1, parent2.getData()));
				tree2.add(new Node<ArmConfig>(parent2, parent1.getData()));
				List<ArmConfig> path = path(tree1.get(tree1.size() - 1));
				path.addAll(reversedPath(tree2.get(tree2.size() - 1)));
				return path;
			}
			for (int i = 0; i < MAX_VERTICES; i++) {
				ArmConfig cfg1 = getValidSample(problem);
				ArmConfig cfg2 = getValidSample(problem);
				parent1 = adjacent(tree1, cfg1);
				parent2 = adjacent(tree2, cfg2);
				List<ArmConfig> path1 = trial(problem, parent1.getData(), cfg1);
				List<ArmConfig> path2 = trial(problem, parent2.getData(), cfg2);
				if (!lineCollision(problem, parent1.getData(), cfg1) && 
					!pathHasCollision(problem, path1)) {
					tree1.add(new Node<ArmConfig>(parent1, cfg1));
				}
				if (!lineCollision(problem, parent2.getData(), cfg2) && 
					!pathHasCollision(problem, path2)) {
					tree2.add(new Node<ArmConfig>(parent2, cfg2));
				}
			}
		}		
	}
	
	public List<ArmConfig> RRTbidirectional(ProblemSpec problem) {
		Tree<ArmConfig> tree1 = new Tree<ArmConfig>();	
		tree1.add(new Node<ArmConfig>(null, problem.getInitialState()));
		Tree<ArmConfig> tree2 = new Tree<ArmConfig>();	
		tree2.add(new Node<ArmConfig>(null, problem.getGoalState()));
						
		while (true) {
			Node<ArmConfig> parent1 = adjacent(tree1, problem.getGoalState());
			Node<ArmConfig> parent2 = adjacent(tree2, problem.getInitialState());
			if (!collision(problem, parent1.getData(), problem.getGoalState())) {
				tree1.add(new Node<ArmConfig>(parent1, problem.getGoalState()));
				return path(tree1.get(tree1.size() - 1));
			}
			if (!collision(problem, parent2.getData(), problem.getInitialState())) {
				tree2.add(new Node<ArmConfig>(parent2, problem.getInitialState()));
				return reversedPath(tree2.get(tree2.size() - 1));
			}
			for (int i = 0; i < MAX_VERTICES; i++) {
				ArmConfig cfg1 = getValidSample(problem);
				ArmConfig cfg2 = getValidSample(problem);
				parent1 = adjacent(tree1, cfg1);
				parent2 = adjacent(tree2, cfg2);
				List<ArmConfig> path1 = trial(problem, parent1.getData(), cfg1);
				List<ArmConfig> path2 = trial(problem, parent2.getData(), cfg2);
				if (!lineCollision(problem, parent1.getData(), cfg1) && 
					!pathHasCollision(problem, path1)) {
					tree1.add(new Node<ArmConfig>(parent1, cfg1));
				}
				if (!lineCollision(problem, parent2.getData(), cfg2) && 
					!pathHasCollision(problem, path2)) {
					tree2.add(new Node<ArmConfig>(parent2, cfg2));
				}
			}
		}		
	}
	
	public List<ArmConfig> RRTbasic(ProblemSpec problem) {
		Tree<ArmConfig> tree = new Tree<ArmConfig>();	
		tree.add(new Node<ArmConfig>(null, problem.getInitialState()));
						
		while (true) {
			Node<ArmConfig> parent = adjacent(tree, problem.getGoalState());
			if (!collision(problem, parent.getData(), problem.getGoalState())) {
				tree.add(new Node<ArmConfig>(parent, problem.getGoalState()));
				return path(tree.get(tree.size() - 1));
			}
			for (int i = 0; i < MAX_VERTICES; i++) {
				ArmConfig cfg = getValidSample(problem);
				parent = adjacent(tree, cfg);
				List<ArmConfig> path = trial(problem, parent.getData(), cfg);
				if (!lineCollision(problem, parent.getData(), cfg) && 
					!pathHasCollision(problem, path)) {
					tree.add(new Node<ArmConfig>(parent, cfg));
				}
			}
		}		
	}
	
	private Node<ArmConfig> adjacent(Tree<ArmConfig> tree, ArmConfig cfg) {
		Point2D base = cfg.getBaseCenter();
		Node<ArmConfig> node = null;
		Node<ArmConfig> adjacent = null;
		double adjacentDistance = 0.0;
		double distance = Double.POSITIVE_INFINITY;
		
		for (Node<ArmConfig> neighbour : tree.getTree()) {
			adjacent = neighbour;
			adjacentDistance = base.distanceSq(adjacent.getData().getBaseCenter());
			if (adjacentDistance < distance) {
				node = adjacent;
				distance = adjacentDistance;
			}
		}
		return node;
	}
	
	private boolean collision(ProblemSpec problem, ArmConfig cfg1, ArmConfig cfg2) {
		List<ArmConfig> path = trial(problem, cfg1, cfg2);
		Line2D line1 = new Line2D.Double(cfg1.getBaseCenter(), cfg2.getBaseCenter());
		
		if (!cfg1.getLinks().isEmpty()) {
			Line2D line2 = new Line2D.Double(
					   cfg1.getLinks().get(cfg1.getJointCount() - 1).getP1(), 
					   cfg2.getLinks().get(cfg1.getJointCount() - 1).getP1());
			
			for (Obstacle obstacle : problem.getObstacles()) {
				Rectangle2D lenientRect = grow(obstacle.getRect(), -MAX_ERROR);
				if (line1.intersects(lenientRect) ||
					line2.intersects(lenientRect) ||
					line1.intersectsLine(line2)) {
					return true;
				}
			}
		} else {
			for (Obstacle obstacle : problem.getObstacles()) {
				Rectangle2D lenientRect = grow(obstacle.getRect(), -MAX_ERROR);
				if (line1.intersects(lenientRect)) {
					return true;
				}
			}
		}
		
		if (pathHasCollision(problem, path)) {
			return true;
		}
		
		return false;
	}
	
	private boolean lineCollision(ProblemSpec problem, ArmConfig cfg1, ArmConfig cfg2) {
		Line2D line = new Line2D.Double(cfg1.getBaseCenter(), cfg2.getBaseCenter());
		
		for (Obstacle obstacle : problem.getObstacles()) {
			Rectangle2D lenientRect = grow(obstacle.getRect(), -MAX_ERROR);
			if (line.intersects(lenientRect)) {
				return true;
			}
		}
		
		return false;
	}
	
	private boolean hasCollision(ArmConfig cfg, List<Obstacle> obstacles) {
		for (Obstacle o : obstacles) {
			if (hasCollision(cfg, o)) {
				return true;
			}
		}
		return false;
	}
	
	private boolean hasCollision(ArmConfig cfg, Obstacle o) {
		Rectangle2D lenientRect = grow(o.getRect(), MAX_ERROR);
		List<Line2D> links = cfg.getLinks();
		for (Line2D link : links) {
			if (link.intersects(lenientRect)) {
				return true;
			}
		}
		List<Line2D> chair = cfg.getChair();
		for (Line2D border: chair) {
			if (border.intersects(lenientRect)) {
				return true;
			}
		}
		return false;
	}

	private Rectangle2D grow(Rectangle2D rect, double delta) {
		return new Rectangle2D.Double(rect.getX() - delta, rect.getY() - delta,
				rect.getWidth() + delta * 2, rect.getHeight() + delta * 2);
	}
	
	private boolean fitsBounds(ArmConfig cfg) {
		if (!lenientBounds.contains(cfg.getBaseCenter())) {
			return false;
		}
		List<Line2D> links = cfg.getLinks();
		for (Line2D link : links) {
			if (!lenientBounds.contains(link.getP2()) ||
				(!link.intersects(lenientBounds))) {
				return false;
			}
		}
		List<Line2D> chair = cfg.getChair();
		for (Line2D border : chair) {
			if (!lenientBounds.contains(border.getP2())) {
				return false;
			}
		}
		return true;
	}
	
	private boolean hasSelfCollision(ArmConfig cfg) {
		List<Line2D> links = cfg.getLinks();
		List<Line2D> chair = cfg.getChair();
		for (int i = 0; i < links.size(); i++) {
			if (cfg.hasGripper()) {
				if (links.size()-i <= 4) {
					for (int j = 0; j < links.size()-5; j++) {
						if (links.get(i).intersectsLine(links.get(j))) {
							return true;
						}
					}
				} else {
					for (int j = 0; j < i - 1; j++) {
						if (links.get(i).intersectsLine(links.get(j))) {
							return true;
						}
					}
				}
			} else {
				for (int j = 0; j < i - 1; j++) {
					if (links.get(i).intersectsLine(links.get(j))) {
						return true;
					}
				}
			}
			if(i > 0) {
				for(int j = 0; j < 4; j++) {
					if (links.get(i).intersectsLine(chair.get(j))) {
						return true;
					}
				}
			}
		}
		return false;
	}
	
	private boolean pathHasCollision(ProblemSpec problem, List<ArmConfig> path) {
		for (ArmConfig step : path) {
			if (!validSample(problem, step)) {
				return true;
			}
		}
		return false;
	}
	
	public boolean hasValidGripperLengths(ArmConfig cfg) {
		List<Double> gripperLengths = cfg.getGripperLengths();
		for (Double length : gripperLengths) {
			if (length <= MIN_GRIPPER_LENGTH - MAX_ERROR) {
				return false;
			} else if (length >= MAX_GRIPPER_LENGTH + MAX_ERROR) {
				return false;
			}
		}

		return true;
	}
	
	private Point2D generateSample() {
		return new Point2D.Double(Math.random(), Math.random());
	}
	
	private boolean validSample(ProblemSpec problem, ArmConfig cfg) {
		if (fitsBounds(cfg) && !hasSelfCollision(cfg) &&
			!hasCollision(cfg, problem.getObstacles())) {
			if (problem.hasGripper()) {
				if (hasValidGripperLengths(cfg)) {
					return true;
				}
			}
			return true;
		}
		return false;
	}
	
	private ArmConfig getSample(ProblemSpec problem) {
		List<Double> joints = new ArrayList<Double>();
		
		for (int i = 0; i < problem.getJointCount(); i++) {
			joints.add(Math.random() * 2 * MAX_JOINT_ANGLE - MAX_JOINT_ANGLE);
		}
		
		if (problem.hasGripper()) {
			List<Double> lengths = new ArrayList<Double>();
			
			for (int i = 0; i < MAX_GRIPPERS; i++) {
				lengths.add(Math.random() * (MAX_GRIPPER_LENGTH - MIN_GRIPPER_LENGTH) + MIN_GRIPPER_LENGTH);
			}
			return new ArmConfig(generateSample(), joints, lengths);
		}
		return new ArmConfig(generateSample(), joints);
	}
	
	private ArmConfig getValidSample(ProblemSpec problem) {
		while (true) {
			ArmConfig cfg = getSample(problem);
			if (validSample(problem, cfg)) {
				return cfg;
			}
		}
	}
	
	public boolean isValidStep(ArmConfig cfg0, ArmConfig cfg1) {
		if (cfg0.getJointCount() != cfg1.getJointCount()) {
			return false;
		} else if (cfg0.maxAngleDiff(cfg1) > MAX_JOINT_STEP + MAX_ERROR) {
			return false;
		} else if (cfg0.maxGripperDiff(cfg1) > MAX_GRIPPER_STEP + MAX_ERROR) {
			return false;
		} else if (cfg0.getBaseCenter().distance(cfg1.getBaseCenter()) > MAX_BASE_STEP + MAX_ERROR) {
			return false;
		}
		return true;
	}
	
	public List<ArmConfig> shortenPath(ProblemSpec problem, List<ArmConfig> path) {
		boolean valid = true;
		
		while (valid) {
			for (int i = 0; i < path.size() - 2; i++) {
				valid = false;
				if (isValidStep(path.get(i), path.get(i + 2))) {
					path.remove(i + 1);
					i--;
					valid = true;
				}
			}
		}
		return path;
	}
	
	private List<ArmConfig> path(Node<ArmConfig> goal) {
		List<ArmConfig> path = new ArrayList<ArmConfig>();
		Node<ArmConfig> node = goal;
		path.add(node.getData());
		
		while (node != null) {
			node = node.getParent();
			if (node != null) {
				path.add(node.getData());
			}
		}
		
		Collections.reverse(path);
		return path;
	}
	
	private List<ArmConfig> reversedPath(Node<ArmConfig> initial) {
		List<ArmConfig> path = new ArrayList<ArmConfig>();
		Node<ArmConfig> node = initial;
		path.add(node.getData());
		
		while (node != null) {
			node = node.getParent();
			if (node != null) {
				path.add(node.getData());
			}
		}
		
		return path;
	}
	
	private List<ArmConfig> trial(ProblemSpec problem, ArmConfig cfg1, ArmConfig cfg2) {
		List<ArmConfig> primitivePath = new ArrayList<ArmConfig>();
		
		if (problem.hasGripper()) {
			primitivePath.add(new ArmConfig(cfg1.getBaseCenter(), 
							cfg1.getJointAngles(),
							cfg1.getGripperLengths()));
		} else {
			primitivePath.add(new ArmConfig(cfg1));
		}
		
		double primitiveX = cfg1.getBaseCenter().getX();
		double primitiveY = cfg1.getBaseCenter().getY();
		List<Double> primitiveAngles = new ArrayList<Double>(cfg1.getJointAngles());
		List<Double> primitiveLengths = null;
		
		if (problem.hasGripper()) {
			primitiveLengths = new ArrayList<Double>(cfg1.getGripperLengths());
		}
		
		double x = (cfg2.getBaseCenter().getX() - cfg1.getBaseCenter().getX()) / TRIAL_INTERPOLATION;
		double y = (cfg2.getBaseCenter().getY() - cfg1.getBaseCenter().getY()) / TRIAL_INTERPOLATION;
		
		List<Double> jointAngles = new ArrayList<Double>();
		for (int i = 0; i < cfg1.getJointCount(); i++) {
			double angle = (cfg2.getJointAngles().get(i)
					     - cfg1.getJointAngles().get(i)) / TRIAL_INTERPOLATION;
			jointAngles.add(angle);
		}
		
		List<Double> gripperLengths = new ArrayList<Double>();	
		if (problem.hasGripper()) {
			for (int i = 0; i < cfg1.getGripperLengths().size(); i++) {
				double length = (cfg2.getGripperLengths().get(i)
							  - cfg1.getGripperLengths().get(i)) / TRIAL_INTERPOLATION;
				gripperLengths.add(length);
			}
		}
		
		for (int i = 0; i < TRIAL_INTERPOLATION; i++) {
			primitiveX += x;
			primitiveY += y;
			Point2D base = new Point2D.Double(primitiveX, primitiveY);
			
			for (int j = 0; j < primitiveAngles.size(); j++) {
				primitiveAngles.set(j, primitiveAngles.get(j) + jointAngles.get(j));
			}
			
			if (problem.hasGripper()) {
				for (int j = 0; j < cfg1.getGripperLengths().size(); j++) {
					primitiveLengths.set(j, primitiveLengths.get(j) + gripperLengths.get(j));
				}
				ArmConfig primitive = new ArmConfig(base, primitiveAngles, primitiveLengths);
				primitivePath.add(primitive);
			} else {
				ArmConfig primitive = new ArmConfig(base, primitiveAngles);
				primitivePath.add(primitive);
			}
		}
		
		if (problem.hasGripper()) {
			primitivePath.add(new ArmConfig(cfg2.getBaseCenter(), 
							cfg2.getJointAngles(),
							cfg2.getGripperLengths()));
		} else {
			primitivePath.add(new ArmConfig(cfg2));
		}
		
		return primitivePath;
	}
	
	public List<ArmConfig> primitiveSteps(ProblemSpec problem, List<ArmConfig> path) {
		List<ArmConfig> primitivePath = new ArrayList<ArmConfig>();
		
		for (int i = 0; i < path.size() - 1; i++) {
			if (problem.hasGripper()) {
				primitivePath.add(new ArmConfig(path.get(i).getBaseCenter(), 
								path.get(i).getJointAngles(),
								path.get(i).getGripperLengths()));
			} else {
				primitivePath.add(new ArmConfig(path.get(i)));
			}
			
			double primitiveX = path.get(i).getBaseCenter().getX();
			double primitiveY = path.get(i).getBaseCenter().getY();
			List<Double> primitiveAngles = new ArrayList<Double>(path.get(i).getJointAngles());
			List<Double> primitiveLengths = null;
			
			if (problem.hasGripper()) {
				primitiveLengths = new ArrayList<Double>(path.get(i).getGripperLengths());
			}
			
			Double x = (path.get(i + 1).getBaseCenter().getX()
					 - path.get(i).getBaseCenter().getX()) / INTERPOLATION;
			Double y = (path.get(i + 1).getBaseCenter().getY()
					 - path.get(i).getBaseCenter().getY()) / INTERPOLATION;
			
			List<Double> jointAngles = new ArrayList<Double>();
			for (int j = 0; j < path.get(i).getJointCount(); j++) {
				double angle = (path.get(i + 1).getJointAngles().get(j)
						     - path.get(i).getJointAngles().get(j)) / INTERPOLATION;
				jointAngles.add(angle);
			}
			
			List<Double> gripperLengths = new ArrayList<Double>();	
			if (problem.hasGripper()) {
				for (int j = 0; j < path.get(i).getGripperLengths().size(); j++) {
					double length = (path.get(i + 1).getGripperLengths().get(j)
								  - path.get(i).getGripperLengths().get(j)) / INTERPOLATION;
					gripperLengths.add(length);
				}
			}
			
			for (int j = 0; j < INTERPOLATION; j++) {
				primitiveX += x;
				primitiveY += y;
				Point2D base = new Point2D.Double(primitiveX, primitiveY);
				
				for (int k = 0; k < primitiveAngles.size(); k++) {
					primitiveAngles.set(k, primitiveAngles.get(k) + jointAngles.get(k));
				}
				
				if (problem.hasGripper()) {
					for (int k = 0; k < path.get(i).getGripperLengths().size(); k++) {
						primitiveLengths.set(k, primitiveLengths.get(k) + gripperLengths.get(k));
					}
					ArmConfig primitive = new ArmConfig(base, primitiveAngles, primitiveLengths);
						primitivePath.add(primitive);
				} else {
					ArmConfig primitive = new ArmConfig(base, primitiveAngles);
						primitivePath.add(primitive);
				}
			}
		}
		if (problem.hasGripper()) {
			primitivePath.add(new ArmConfig(path.get(path.size() - 1).getBaseCenter(), 
							path.get(path.size() - 1).getJointAngles(),
							path.get(path.size() - 1).getGripperLengths()));
		} else {
			primitivePath.add(new ArmConfig(path.get(path.size() - 1)));
		}
		return primitivePath;
	}
}
