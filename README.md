### Signed Distance Field Mesh Generator
This project aims at creating a tool to generate meshes from user-defined signed distance fields (SDF).
The goal is to be able to combine various pre- and custom-defined SDFs in a tree-structure to allow building complex meshes in an intuitive way without requiring meticulous editing of individual polygons. This approach is expected to be faster for simple models, e.g. toon-ish characters, environmental objects or combinations of generic shapes in exchange for the ability to create ultra-high-precision meshes for e.g. realistic characters.
The algorithm used for mesh generation is [Dual Contouring](https://en.wikipedia.org/wiki/Isosurface#Dual_contouring).

#### Features
The project is still a heavy WIP and so far only the generation of a mesh from a hardcoded SDF and its visualization with controls in an openGL window works, meaning only the core component of the program exists at this point. I hope to be able to continue soon, but until then, the TODO list reads as follows.

#### TODO
MUST:
- GUI from the already included imgui:
	1. Replace the current main window with an imgui window and render the scene into the background of the imgui window
	2. Add a tree structure to add and edit signed distance fields and a variety interpolation methods
	3. Parse the tree to create the total signed distance field in runtime
- Export the meshes in stl format with normals and per-quad UVs

CAN:
- Improve the vertex finding of dual contouring with [this algorithm](https://www.inf.ufrgs.br/~comba/papers/thesis/diss-leonardo.pdf), which is expected to be more robust and possibly faster.
- Save and import the tree structure for intermediate saving and creation of templates
- Allow slicing open the mesh with custom planes to generate UVs over the entire mesh
	- if this is done, possibly allow loading textures for preview
