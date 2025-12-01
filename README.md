# paper ：A bi-level framework for multi-task path planning in mountainous orchards

# more code and information see in master branch.


## Abstract:In complex agricultural environments such as mountain orchards, small autonomous equipment is widely used for multi-objective agricultural operation tasks (e.g., inspection, picking, spraying) due to its high mobility and adaptability. However, unstructured terrain and limited energy significantly increase the complexity of path planning and task scheduling. For this reason, this paper proposes a bi-level planning framework for multi-task path optimization problems in mountain orchards. In the first level, the framework adopts the Improved A*-History (IA*-H) algorithm, to solve the problem of paths between fruit trees or from a warehouse to a fruit tree in a mountain orchard, where terrain ups and downs.In the second level, a new Multi-Strategy Discrete Grey Wolf Optimizer (MSD-GWO) algorithm is proposed, to solve the path problem for multi-task scheduling throughout the orchard. After two level execution the optimal sequence and path for multi-tasks is determined. The experiment utilized typical mountainous orchard terrain data (Chu orange base in Longling County, Baoshan City, Yunnan Province), and the experimental results showed that the planning time of our method was reduced by 94.5% compared to Dijkstra and 85.1% compared to Z*. And Compared to the greedy scheduling strategy, our approach reduces path length by 24.6% and energy consumption by 20.6%. which verified the effectiveness and feasibility of our proposed bi-level framework.


TSP_DATA:Our TSP test datasets TSPLIB.

PATH_DATA:Our path task place and map. 

tifread.m:read height map data, you can change it to your owner height map.

showmap.m:according to your height data map build grid map.

D_D.m:Distance as objective, use Dijkstra.

D_E.m:Energy as objective, use Dijkstra.

Astar_D: A*, Distance as objective, heuristic is distance.

Astar_E: A*, Energy as objective, heuristic is distance.

Zstar: A*, Energy as objective, heuristic is energy eq(11).

Improved_Astar.m: use our IA*，Energy as objective eq(10), heuristic is energy eq(11).

Test1_PRM: In test1 use heightmap.png and choose PRM, the result of Fig.9(g).

Test1_RRT: In test1 use heightmap.png and choose RRT*, the result of Fig.9(h).

Test1_DRL: In test1 use heightmap.png and choose DRL, the result of Figh.9(i).

Test1.m: In test1 use heightmap.png, choose D_D, D_E, Astar_D, Astar_E, Zstar and Improved_Astar, the result of Fig.9.

also see Fig.10 result in Test2

Test1_obs.m:consider dense row orchard as obstacle, path result in Fig.12.

TSP_Dijkstra: use Dijkstra for multi tasks path planning.

TSP_Dijkstra_Greedy: use Dijkstra for multi tasks path planning, and greedy for tasks schedule planning, the result of Fig.17.

TSP_Zstar_MSD_GWO.m: use Zstar for multi tasks path planning, and MSD_GWO for tasks schedule planning, the result of Fig.17.


