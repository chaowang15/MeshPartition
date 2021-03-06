#include "mesh_partition.h"

int main(int argc, char** argv)
{
	if (argc != 5)
	{
		cout << "Usage: MeshPartition ply_model target_cluster_number simp_inside_edge_ratio simp_border_edge_ratio" << endl;
		return -1;
	}
	string ply_fname(argv[1]);
	int cluster_num = atoi(argv[2]);
	double inside_edge_ratio = atof(argv[3]), border_edge_ratio = atof(argv[4]);
	MeshPartition mesh_partition;
	string fname = ply_fname.substr(0, ply_fname.length() - 4) + "-clusters" + std::to_string(cluster_num);

	// Read model
	cout << "Reading ply model " << ply_fname << " ..." << endl;
	if (!mesh_partition.readPLY(ply_fname))
		return -1;
	cout << "#Vertices: " << mesh_partition.vertex_num_ << ", #Faces: " << mesh_partition.face_num_ << endl;

	// Mesh partition
	cout << "Running partition (target #cluster: " << cluster_num << " ) ..." << endl;
	mesh_partition.runMeshPartition(cluster_num);
	string cluster_fname = fname + "-cluster.txt";
	cout << "Saving cluster file into '" << cluster_fname << "' ... " << endl;
	mesh_partition.saveClusterFile(cluster_fname);
	string output_ply_fname = fname + ".ply";
	cout << "Saving clustered PLY model '" << output_ply_fname << "' ... " << endl;
	mesh_partition.writePLYWithFaceColors(output_ply_fname);

	// Plane inner edge simplification
	cout << "Running inner edge contraction ..." << endl;
	mesh_partition.flag_preserve_topology_ = true;
	if (mesh_partition.flag_preserve_topology_)
		fname += "-topo";
	mesh_partition.runClusterInnerEdgeSimp(inside_edge_ratio);
	string inner_simp_fname = fname + "-inner" + string(argv[3]) + ".ply";
	cout << "Saving simplified PLY model '" << inner_simp_fname << "' ... " << endl;
	mesh_partition.writeSimplifiedPLY(inner_simp_fname);

	// Plane border edge simplification
	cout << "Running border edge contraction ..." << endl;
	mesh_partition.edge_simp_type_ = ALL_BORDER_EDGES;
	mesh_partition.runClusterBorderEdgeSimp(border_edge_ratio);
	string border_simp_name = fname + "-inner" + string(argv[3]) + "-border" + string(argv[4]);
	string border_simp_fname = border_simp_name + ".ply";
	cout << "Saving simplified PLY model '" << border_simp_fname << "' ... " << endl;
	mesh_partition.writeSimplifiedPLY(border_simp_fname);
	string simp_cluster_fname = border_simp_name + "-cluster.txt";
	cout << "Saving simplified mesh's cluster file into '" << simp_cluster_fname << "' ... " << endl;
	mesh_partition.saveSimplifiedClusterFile(simp_cluster_fname);

	cout << "ALL DONE." << endl;
	return 0;
}