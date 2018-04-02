#ifndef MESHPARTITION_H
#define MESHPARTITION_H

#include <iostream>
#include <string>
#include <vector>
#include <Eigen/Eigen>
#include <unordered_set>
#include <unordered_map>
#include "covariance.h"
#include "MxHeap.h"
#include "qemquadrics.h"

using namespace std;
using namespace Eigen;

///////////////////////////////////////////////////////////
/* Edge and its comparator (used in min-heap) */
class Edge : public MxHeapable
{
public:
	int v1, v2;
	Edge(int s, int t) { v1 = s; v2 = t; }
};

struct Vertex
{
	bool is_border, is_valid;
	Vector3d pt;
	Vector3f color, cluster_color;
	unordered_set<int> neighbors, belonging_faces;
	QEMQuadrics Q;
	Vertex() : is_border(false), is_valid(true){}
};

struct Face
{
	int indices[3];
	int cluster_id;
	bool is_visited, is_valid;
	unordered_set<int> neighbors;
	Face() : cluster_id(-1), is_visited(false), is_valid(true){}
};

struct SwapFace
{
	int face_id;
	int from;
	int to;
	SwapFace(int v, int f, int t){ face_id = v; from = f; to = t; }
};


struct Cluster
{
	bool is_new;
	unordered_set<int> elements;
	unordered_set<int> neighbors; // neighbor clusters
	vector<SwapFace> faces_to_swap;
	Vector3f color;
	double energy;
	CovObj cov, ini_cov;
	Cluster() : is_new(false), energy(0){}
	bool isValid() { return !elements.empty(); }
};

class MeshPartition
{
public:
	MeshPartition();
	~MeshPartition();

	bool readPLY(const string filename);
	bool writePLYWithFaceColors(const string filename);
	void runMeshPartition(int cluster_num);
	void saveClusterFile(const string filename);
	void runMeshSimplification(double ratio);
	bool writeSimplifiedPLY(const string filename);

private:
	void getFaceAndVertexNeighbors();

	/* Mesh Partition */
	void initMerging();
	void merging();
	void initSwap();
	void swapAll();
	void swapOnce();
	double totalEnergy();
	void postProcessClusters();
	bool runFaceEdgeContractionOnce();
	void collectClusterEnergies();
	void updateClusterCovs();
	void createClusterEdgesAndHeap();
	void eraseEdgeFromList(int cidx, Edge* edge);
	int findClusterNeighbors(int cidx);
	int findClusterNeighbors(int cidx, unordered_set<int>& cluster_elements, unordered_set<int>& neighbors);
	void computeEdgeEnergy(Edge* edge);
	void updateEdgeInHeap(Edge* edge);
	void applyFaceEdgeContraction(Edge* edge);
	void mergeClusters(int c1, int c2);
	void createClusterColors();
	void findSwapClusters();
	void applySwap();
	double computeSwapDeltaEnergy(int fidx, int from, int to);
	bool checkClusterConnectivity(int cidx, vector<unordered_set<int>>& connected_components);
	int traverseFaceBFS(int fidx, int cidx, unordered_set<int>& component);
	void splitCluster(int original_cidx, vector<unordered_set<int>>& connected_components);
	int getValidClusterList();
	void updateFaceClusterIDs();

	/* Clear data */
	void clearClusterEdgesAndHeap();

	/* Mesh Simplification */
	void initVtxEdgeContraction();
	void getBorderVertices();
	bool contractAllVtxEdges();
	void initVtxQuadrics();
	void createInitVtxEdges();
	bool runVtxEdgeContractionOnce();
	bool applyVtxEdgeContraction(Edge* edge);
	bool checkVtxEdgeContraction(Edge* edge);
	bool isContractedVtxValid(Edge* edge, int endpoint, const Vector3d& vtx);

	/* Small functions */
	bool checkFaceContainsVertices(int fidx, int v1, int v2){
		return checkFaceContainsVertices(fidx, v1) && checkFaceContainsVertices(fidx, v2);
	}

	bool checkFaceContainsVertices(int fidx, int v1){
		return faces_[fidx].indices[0] == v1 || faces_[fidx].indices[1] == v1 || faces_[fidx].indices[2] == v1;
	}


	inline long long getKey(long long a, long long b){ 
		return (a << 32) | b; // use one long long integer storing two 32-bit ints as key 
	};
	inline void getEdge(long long key, int& v1, int& v2){
		v2 = int(key & 0xffffffffLL);
		v1 = int(key >> 32);
	}

public:
	vector<Vertex> vertices_;
	vector<Face> faces_;
	vector<Cluster> clusters_;
	unordered_map<long long, vector<int>> edge2faces_;

	int vertex_num_, face_num_;
	int edge_num_, target_edge_num_;
	vector<vector<Edge*>> cluster_edges_;
	MxHeap heap_;
	int cluster_num_; // target/final cluster number
	int curr_cluster_num_; // current cluster number
	int count_swapping_faces_;
	unordered_set<int> clusters_in_swap_; // clusters related during swap
	unordered_map<int, int> clusters_new2old_; // clusters' new indices [0, cluster_num_) to old ones [0, face_num_)
	unordered_map<int, int> clusters_old2new_; // clusters' old indices [0, face_num_) to new ones [0, cluster_num_)
	const double kEdgeCoefficient = 100.0, kPointCoefficient = 1.0;
};


#endif