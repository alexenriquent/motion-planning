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
	
	public static double MAX_ERROR = 1e-5;
	public static final int MAX_SAMPLE = 50;
	public static final double MAX_JOINT_ANGLE = 150 * Math.PI / 180;
	public static final Rectangle2D BOUNDS = new Rectangle2D.Double(0, 0, 1, 1);
	
	private Rectangle2D lenientBounds;
	
	public RRT() {
		lenientBounds = grow(BOUNDS, MAX_ERROR);
	}
	
	public List<ArmConfig> search(ProblemSpec problem) {
		List<Obstacle> obstacles = problem.getObstacles();
		Tree<ArmConfig> tree = new Tree<ArmConfig>();
		
		tree.add(new Node<ArmConfig>(null, problem.getInitialState()));
		
		while (true) {
			Node<ArmConfig> parent = adjacent(tree, problem.getGoalState());
			if (!collision(parent.getData(), problem.getGoalState(), obstacles)) {
				Node<ArmConfig> goal = new Node<ArmConfig>(parent, problem.getGoalState());
				tree.add(goal);
				return path(goal);
			}
			for (int i = 0; i < MAX_SAMPLE; i++) {
				ArmConfig cfg = getValidSample(problem);
				parent = adjacent(tree, cfg);
				if (!lineCollision(parent.getData(), cfg, obstacles)) {
					tree.add(new Node<ArmConfig>(parent, cfg));
				}
			}
		}		
	}
	
	private Node<ArmConfig> adjacent(Tree<ArmConfig> tree, ArmConfig cfg) {
		Point2D base = cfg.getBaseCenter();
		Node<ArmConfig> adjacent, node = null;
		double adjacentDistance = 0.0;
		double distance = Double.POSITIVE_INFINITY;
		
		for (int i = 0; i < tree.size(); i++) {
			adjacent = tree.get(i);
			adjacentDistance = base.distanceSq(adjacent.getData().getBaseCenter());
			if (adjacentDistance < distance) {
				node = adjacent;
				distance = adjacentDistance;
			}
		}
		return node;
	}
	
	public boolean collision(ArmConfig cfg1, ArmConfig cfg2, List<Obstacle> obstacles) {
		int jointCount = cfg1.getJointCount();
		Point2D base1 = cfg1.getLinks().get(jointCount - 1).getP1();
		Point2D base2 = cfg2.getLinks().get(jointCount - 1).getP1();
		Line2D line1 = new Line2D.Double(cfg1.getBaseCenter(), cfg2.getBaseCenter());
		Line2D line2 = new Line2D.Double(base1, base2);
		
		for (Obstacle obstacle : obstacles) {
			Rectangle2D lenientRect = grow(obstacle.getRect(), -MAX_ERROR);
			if (line1.intersects(lenientRect) ||
				line2.intersects(lenientRect) ||
				line1.intersectsLine(line2)) {
				return true;
			}
		}
		return false;
	}
	
	public boolean lineCollision(ArmConfig cfg1, ArmConfig cfg2, List<Obstacle> obstacles) {
		Line2D line = new Line2D.Double(cfg1.getBaseCenter(), cfg2.getBaseCenter());
		
		for (Obstacle obstacle : obstacles) {
			Rectangle2D lenientRect = grow(obstacle.getRect(), -MAX_ERROR);
			if (line.intersects(lenientRect)) {
				return true;
			}
		}
		return false;
	}
	
	public boolean hasCollision(ArmConfig cfg, List<Obstacle> obstacles) {
		for (Obstacle o : obstacles) {
			if (hasCollision(cfg, o)) {
				return true;
			}
		}
		return false;
	}
	
	public boolean hasCollision(ArmConfig cfg, Obstacle o) {
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

	public Rectangle2D grow(Rectangle2D rect, double delta) {
		return new Rectangle2D.Double(rect.getX() - delta, rect.getY() - delta,
				rect.getWidth() + delta * 2, rect.getHeight() + delta * 2);
	}
	
	public boolean fitsBounds(ArmConfig cfg) {
		if (!lenientBounds.contains(cfg.getBaseCenter())) {
			return false;
		}
		List<Line2D> links = cfg.getLinks();
		for (Line2D link : links) {
			if (!lenientBounds.contains(link.getP2())) {
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
	
	public boolean hasSelfCollision(ArmConfig cfg) {
		List<Line2D> links = cfg.getLinks();
		List<Line2D> chair = cfg.getChair();
		for (int i = 0; i < links.size(); i++) {
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
			if (i > 0) {
				for(int j = 0; j < 4; j++) {
					if (links.get(i).intersectsLine(chair.get(j))) {
						return true;
					}
				}
			}
		}
		return false;
	}
	
	public Point2D generateSample() {
		return new Point2D.Double(Math.random(), Math.random());
	}
	
	public boolean validSample(ProblemSpec problem, ArmConfig cfg) {
		if (fitsBounds(cfg) && !hasSelfCollision(cfg) &&
			!hasCollision(cfg, problem.getObstacles())) {
			return true;
		}
		return false;
	}
	
	public ArmConfig getSample(int jointCount) {
		List<Double> joints = new ArrayList<Double>();
		
		for (int i = 0; i < jointCount; i++) {
			joints.add(Math.random() * 2 * MAX_JOINT_ANGLE - MAX_JOINT_ANGLE);
		}
		return new ArmConfig(generateSample(), joints);
	}
	
	public ArmConfig getValidSample(ProblemSpec problem) {
		while (true) {
			ArmConfig cfg = getSample(problem.getJointCount());
			if (validSample(problem, cfg)) {
				return cfg;
			}
		}
	}
	
	public static List<ArmConfig> path(Node<ArmConfig> goal) {
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
}
