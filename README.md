# temporary-name-placeholder
I am still deciding on what to do. The repository will be renamed when I do.



The Rule of Thumb

Large $\theta$ (e.g., > 1.0): The simulation is **Fast but Inaccurate**. It aggressively groups distant particles together, calculating forces based on large, low-resolution clusters.

Small THETA (e.g., < 0.2): The simulation is **Slow but Accurate**. It refuses to group particles unless they are very far away, forcing the algorithm to calculate forces against specific individual particles more often.

THETA = 0: This turns Barnes-Hut into a standard Brute Force ($O(N^2)$) algorithm, where every particle interacts with every other particle. Maximum accuracy, minimum speed.

---

The Math Behind ItThe Barnes-Hut algorithm makes a decision for every node in the QuadTree using this ratio:$$\frac{s}{d} < \theta$$$s$ = The width of the region (the node).4$d$ = The distance between the current particle and that region's center of mass.If the ratio of "size to distance" is less than your Theta, the algorithm says, "That cluster of particles is far enough away to be treated as a single massive dot."If you set Theta to be Large (e.g., 1.5), it is very easy to satisfy this condition. The algorithm "short-circuits" early, ignoring the details of the cluster.If you set Theta to be Small (e.g., 0.3), it is hard to satisfy this condition. The algorithm is forced to dive deeper into the tree (recurse) to find smaller nodes or individual leaves.




To run:

```
cargo run --target x86_64-pc-windows-gnu

cargo run --target x86_64-pc-windows-gnu --release
```