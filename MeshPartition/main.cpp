#include "mesh_partition.h"

int main(int argc, char** argv)
{
	if (argc != 4)
	{
		cout << "Usage: MeshPartition ply_model target_cluster_number simplification_ratio" << endl;
		return -1;
	}
	string ply_fname(argv[1]);
	int cluster_num = atoi(argv[2]);
	double ratio = atof(argv[3]);
	MeshPartition mesh_partition;

	// Read model
	cout << "Reading ply model " << ply_fname << " ..." << endl;
	if (!mesh_partition.readPLY(ply_fname))
		return -1;

	// Mesh partition
	mesh_partition.runMeshPartition(cluster_num);

	// Mesh simplification
	mesh_partition.runMeshSimplification(ratio);

	// Save cluster file and mesh model
	string fname = ply_fname.substr(0, ply_fname.length() - 4) + "-clusters" + std::to_string(cluster_num);
	string cluster_fname = fname + ".txt";
	cout << "Saving cluster file into '" << cluster_fname << "' ... " << endl;
	mesh_partition.saveClusterFile(cluster_fname);
	
	string output_ply_fname = fname + ".ply";
	cout << "Saving clustered PLY model '" << output_ply_fname << "' ... " << endl;
	mesh_partition.writePLYWithFaceColors(output_ply_fname);

	string output_simp_ply_fname = fname + "-simp.ply";
	cout << "Saving simplified PLY model '" << output_simp_ply_fname << "' ... " << endl;
	mesh_partition.writeSimplifiedPLY(output_simp_ply_fname);


	cout << "ALL DONE." << endl;
	return 0;
}