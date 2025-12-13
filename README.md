# N-Body Simulation 

### **Information on the _Barnes-Hut Algorithm_**

#### Theta parameter

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

#### Comparison Table

| $\theta$     | Accuracy  | Speed (FPS)  | Behavior                                                |
| :----------: | :-------: | :----------: | :------------------------------------------------------ |
| 0.0          | Perfect   | Very Slow    | Checks every particle against every other.              |
| 0.5          | Good      | Fast         | Strikes a good balance between physics and performance. |
| 1.0          | Poor      | Very Fast    | Particles may behave strangely.                         |

---

### **Overview of the Code Structure**

- `resources.rs`: simulation constants and user-adjustable settings (gravity, theta, timestep, UI toggles).
- `components.rs`: ECS data for body state (position, velocity, acceleration, mass) and optional trails.
- `quadtree.rs`: Barnes-Hut quadtree, insertion, and force calculation with softening.
- `systems/core.rs`: simulation systems (spawn, quadtree rebuild, force calc, integration, trails, culling, camera follow/controls, reset handler).
- `systems/ui.rs`: egui panel wiring to mutate settings/config and trigger resets.

To run:

```
cargo run --release
```


**Note:** If using WSL, you may have to use the GNU toolchain to utilzie GPU acceleration. Ensure you have the necessary build tools installed:

```
rustup target add x86_64-pc-windows-gnu
```

When running use the additional flag:

```
--target x86_64-pc-windows-gnu
```

---

### **Use of Resources and AI**

- I referenced [The Nature of Code](https://natureofcode.com/) throughout the process to help with my understanding of the pieces required.
- Implementing the Quadtree proved to be very difficult and I had to utilize Generative AI to assist with the creation of it.
    - The AI was used to help generate the initial structure and logic for the Quadtree implementation, which I then refined and integrated into the overall simulation code.
- I also utiized AI to help with implementing a restart function, as I kept banging my head against a wall trying to figure it out.
    - Lifetimes were causing problems.

---

### **What was Learned**

- Figuring out what to use for the simulation was difficult at first. I researched different libraries and tried:
    - Nannou
    - Macroquad
    - Bevy (which I ultimately chose for it's performance)
- I started off with a brute-force N-body simulation and then implemented the Barnes-Hut algorithm to optimize performance.
    - It was difficult to jump right into using the Barnes-Hut algorithm. I had to first understand how it worked conceptually before I could implement it effectively.

