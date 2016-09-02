package solution;

import java.util.ArrayList;
import java.util.List;

public class Tree<T> {

	private List<Node<T>> tree;
	
	public Tree() {
		tree = new ArrayList<>();
	}
	
	public List<Node<T>> getTree() {
		return tree;
	}
	
	public void setTree(List<Node<T>> tree) {
		this.tree = tree;
	}
	
	public int size() {
		return tree.size();
	}
	
	public void add(Node<T> node) {
		tree.add(node);
	}
	
	public Node<T> get(int index) {
		return tree.get(index);
	}
	
	public boolean isEmpty() {
		return tree.isEmpty();
	}
}
