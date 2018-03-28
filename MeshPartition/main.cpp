#include "mesh_partition.h"

int main(int argc, char** argv)
{
	if (argc != 3)
	{
		cout << "Usage: MeshPartition ply_model target_cluster_number" << endl;
		return -1;
	}
	string ply_fname(argv[1]);
	int cluster_num = atoi(argv[2]);
	MeshPartition mesh_partition;

	// Read model
	cout << "Reading ply model " << ply_fname << " ..." << endl;
	if (!mesh_partition.readPLY(ply_fname))
		return -1;

	// Partition and swap
	mesh_partition.cluster_num_ = cluster_num;
	cout << "Running partition with target cluster number " << cluster_num << " ..." << endl;
	mesh_partition.partition();
	double last_energy = mesh_partition.totalEnergy();
	cout << "Running swap ..." << endl;
	cout << "  Initial energy: " << last_energy << endl;
	double curr_energy = 0;
	const int kMaxIter = 300;
	int iter = 0;
	while (iter++ < kMaxIter)
	{
		mesh_partition.swapOnce();
		curr_energy = mesh_partition.totalEnergy();
		cout << "  Iteration " << iter << ", Energy: " << curr_energy << endl;
		if ((last_energy - curr_energy) / last_energy < 1e-7 || mesh_partition.count_swapping_faces_ == 0)
			break;
		last_energy = curr_energy;
	}
	cout << "Post-processing noisy clusters ... " << endl;
	mesh_partition.postProcessClusters();

	// Save cluster file and mesh model
	string fname = ply_fname.substr(0, ply_fname.length() - 4);
	size_t pos = fname.find_last_of("/\\");
	string cluster_fname = "clusters.txt";
	if (pos != string::npos)
		cluster_fname = fname.substr(0, pos + 1) + cluster_fname;
	cout << "Saving cluster file into '" << cluster_fname << "' ... " << endl;
	mesh_partition.saveClusterFile(cluster_fname);
	string output_ply_fname = fname + "-cluster.ply";
	cout << "Saving ply model '" << output_ply_fname << "' ... " << endl;
	mesh_partition.writePLYWithFaceColors(output_ply_fname);

	cout << "ALL DONE." << endl;
	return 0;
}