## Notes:
A T-Map, or terrain-map, is a mesh.

### tmap_arc: (NOT IN USE)
- Subclass of tmap_base to create the shape of an arc and evaluate it.
- MoonRanger's present means of arc evaluation stems from an arc/kinematic model defined within interfaces/moonranger_arc_model and is evaluated within rasmEval.cc.

### tmap_associated:
Subclass of tmap_base which adds centers and associations to triangles. This library is used to efficiently find triangle neighbors.

### tmap_astar:
Subclass of tmap_base which finds distances between points (TBD but, presumably, global-scale points). As far as I can tell, this sub-class is only partially being used by navigation.

### tmap_base:
The backbone of a tmap class; subsumes other tmap\_\* subclasses. Also utilizes some input-output.

### tmap_cleanable: TBD

### tmap_full:
A subclass of tmap_base (I think) which combines the functionalities of all other subclasses.

### tmap_group:
Subclass which finds clusters of points (presumably within the mesh itself (as opposed to the point-cloud)).

### tmap_icp:
Subclass of tmap_base used to apply ICP (iterative closest point) between two tmaps. Inherits from tmap_moveable and tmap_associated. Makes use of tmap_searchable.

tmap_icp is synonymous with "merge the two maps"; that is to say, when icp is enabled (via a config file), not only are the local meshes incorporated into the global mesh according to xy-coordinates, they are placed atop the global mesh and then their points are iteratively "matched" with the most similar points currently within the global mesh.

Note that within the tmap_icp files is the parameter 'world'. The use of this name is a bit confusing because the functions are called by a global mesh and a LOCAL mesh is passed as an argument to the parameter named 'world.' For instance, genericSensor.cc calls: accumulatedModel.icp(newData, ...); to perform icp on the newest local mesh it received.

### tmap_moveable:
Subclass of tmap_base used to apply translations and rotations to a tmap.

### tmap_pasteable:
Subclass of tmap_base used to combine two tmaps. Inherits from tmap_triangulate. Makes use of tmap_searchable.
tmap_pasteable is where the "cut a hole in the global mesh; fill the hole with local mesh" processing is defined.

### tmap_searchable:
Subclass of tmap_base used to find the closest points. Inherits from tmap_associated.

### tmap_shrinkable:
Reduces a point-cloud (with the option of decimating beforehand).

### tmap_triangulate:
Subclass of tmap_base used to turn a point-cloud into a triangle mesh.
