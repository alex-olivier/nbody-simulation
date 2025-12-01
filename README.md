# N-Body Simulation with Barnes-Hut Algorithm in Bevy

I am still deciding on what to do. The repository will be renamed when I do.


### Information on the _Barnes-Hut Algorithm_

Theta parameter :

#### The Rule of Thumb

- **Large $\theta$ (e.g., > 1.0):** The simulation is **Fast but Inaccurate**. It aggressively groups distant particles together, calculating forces based on large, low-resolution clusters.

- **Small $\theta$ (e.g., < 0.2):** The simulation is **Slow but Accurate**. It refuses to group particles unless they are very far away, forcing the algorithm to calculate forces against specific individual particles more often.

- **$\theta$ = 0:** This turns Barnes-Hut into a standard Brute Force ($O(N^2)$) algorithm, where every particle interacts with every other particle. Maximum accuracy, minimum speed.

#### The Math Behind It
The Barnes-Hut algorithm makes a decision for every node in the QuadTree using this ratio:

$$\frac{s}{d} < \theta$$

* $s$ = The width of the region (the node).
* $d$ = The distance between the current particle and that region's center of mass.

If the ratio of "size to distance" is **less than** $\theta$, the algorithm says, "That cluster of particles is far enough away to be treated as a single massive dot."

* If you set $\theta$ **to be large** (e.g., 1.5), it is very **easy** to satisfy this condition. The algorithm "short-circuits" early, ignoring the details of the cluster.

* If you set $\theta$ **to be small** (e.g., 0.3), it is **hard** to satisfy this condition. The algorithm is forced to dive deeper into the tree (recurse) to find smaller nodes or individual leaves.

### Comparison Table

| $\theta$     | Accuracy  | Speed (FPS)  | Behavior                                                |
| :----------: | :-------: | :----------: | :------------------------------------------------------ |
| 0.0          | Perfect   | Very Slow    | Checks every particle against every other.              |
| 0.5          | Good      | Fast         | Strikes a good balance between physics and performance. |
| 1.0          | Poor      | Very Fast    | Particles may behave strangely.                         |


---


To run:

```
cargo run --target x86_64-pc-windows-gnu

cargo run --target x86_64-pc-windows-gnu --release
```