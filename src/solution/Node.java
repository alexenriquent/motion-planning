package solution;

public class Node<T> {

	private Node<T> parent;
	private T data;
	
	public Node() {
		this.parent = null;
		this.data = null;
	}

	public Node(Node<T> parent, T data) {
		this.parent = parent;
		this.data = data;
	}
	
	public Node<T> getParent() {
		return parent;
	}
	
	public T getData() {
		return data;
	}
}
