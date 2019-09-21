class Graph {
    Map<Integer, Set<Integer>> edgeTo;

    Graph() {
        this.edgeTo = new HashMap<Integer, Set<Integer>>();
    }

    public int size() {
        return edgeTo.size();
    }

    public void addEdge(int v1, int v2) {
        add(v1, v2);
        add(v2, v1);
    }

    private void add(int from, int to) {
        if (!edgeTo.containsKey(from)) {
            Set<Integer> s = new HashSet<Integer>();
            s.add(to);
            edgeTo.put(from, s);
        } else {
            edgeTo.get(from).add(to);
        }
    }

    public Set<Integer> adj(int v) {
        return edgeTo.get(v);
    }
}