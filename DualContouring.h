#pragma once

#include <Eigen/Dense>
#include <Eigen/iterativeLinearSolvers>
#include <GL/glew.h>
#include <vector>
#include <unordered_map>
#include <map>

class DualContouring
{
private:
	float scale = 1.0f;
	const Eigen::Vector3i _dimensions;

	// A struct to store the intersection point and normal on an edge
	struct EdgeData {
		Eigen::Vector3d point{ 0 };
		Eigen::Vector3d normal{ 0 };
		bool valid = false;
	};

	// A struct to store the voxel data
	struct Voxel {
		// The corners of the voxel
		//Eigen::Vector3d corners[8];
		// The signs of the corners (-1 for inside, +1 for outside)
		//int signs[8];
		// The edge data for each edge (12 edges in total)
		EdgeData edges[12];
	};

	double sdf(Eigen::Vector3d);
	Eigen::Vector3d getNormalFromGradient(Eigen::Vector3d pos, Eigen::Vector3d& normal);

	// mapping voxel position to vertex indices in _vertices
	std::map<unsigned int, unsigned int> _vertexIndices;
	// all vertices
	std::vector<Eigen::Vector3d> _vertices;

	// buffers for opengl
	std::vector<GLfloat> _vertexBuffer;
	std::vector<GLfloat> _colorBuffer;
	std::vector<unsigned int> _triangles;

	// Vector3i voxel index to _vertiexIndices map index
	unsigned int toMapIndex(Eigen::Vector3i);
	// find position of zero along a sign change
	float findZero(float a, float b);
	// Find the zero crossing along an edge
	Eigen::Vector3d findZeroCrossing(Eigen::Vector3d, Eigen::Vector3d, double, double);
	// fill vertex and color buffer
	void fillBuffers();
	// fill _vertices
	void createVertices();
	// fill _triangles
	void createFaces();
	// make a single face given position, zero crossings and face direction
	void createFace(Eigen::Vector3i, std::string, bool);

	// generates a voxel object from a given voxel index to get all information needed to find optimal vertex position
	void CreateVoxelFromPosition(Eigen::Vector3i, Voxel& voxel);
	// get zero crossings to make faces
	std::tuple<bool, bool, bool> findZeroCrossings(Eigen::Vector3i);
	// Create QEF matrix to solve for optimal vertex position in voxel
	void calculateQEF(std::vector<EdgeData> & edges, Eigen::MatrixXd & A, Eigen::VectorXd & b);
	// Trinilearly interpolate forces of voxel corners
	void getForceOnPoint(std::vector<Eigen::Vector3d>& cornerForces, Eigen::Vector3i& voxelIndex, Eigen::Vector3d& position);
	// Perform Schmitz Particle Approximation to find optimal vertex position in voxel
	Eigen::Vector3d schmitzParticleApproximation(Eigen::Vector3d& position, std::vector<EdgeData> intesectingEdges);
	// Finds average position of surrouding voxels to fix a problematic position
	Eigen::Vector3d averagePositionSurrouding(Eigen::Vector3i voxelIndex);
	// get whether voxel contains vertex and if yes, get vertex position
	void findVertexInVoxel(Eigen::Vector3i&, volatile bool&, Eigen::Vector3d&);
	// Clip position into boundaries of a voxel
	void clipPosition(Eigen::Vector3i, Eigen::Vector3d&);
	// clip vertex position to stay within dimensions of one voxel
	Eigen::Vector3d clipSolutionPosition(Eigen::Vector3d);

public:
	DualContouring(Eigen::Vector3i dimensions) : _dimensions(dimensions) {
		_vertices.reserve(10000);
		_triangles.reserve(10000);
	}

	/**
	Performs dual contouring and fills the result vectors
	*/
	void execute();
	
	/**
	Returns the opengl buffers for vertices, color and triangles
	*/
	std::tuple<std::vector<GLfloat>, std::vector<GLfloat>, std::vector<unsigned int>> getData();
};

