#include "mesh_partition.h"
#include <fstream>
#include <sstream>
#include <stdlib.h>
#include <random>
#include <queue>
#include <time.h>

MeshPartition::MeshPartition()
{
	vertex_num_ = face_num_ = 0;
	simp_ratio_ = 1.0;
}

MeshPartition::~MeshPartition()
{
	clearClusterEdges();
	cluster_edges_.clear();
}

/************************************************************************/
/* Data I/O
*/
/************************************************************************/
bool MeshPartition::readPLY(const string filename)
{
	FILE* fin;
	if (!(fin = fopen(filename.c_str(), "rb")))
	{
		cout << "ERROR: Unable to open file" << filename << endl;
		return false;
	}

	/************************************************************************/
	/* Read headers */

	// Mode for vertex and face type
	// 1 for vertex (no faces), 2 for vertex and faces,
	// 3 for vertex, vertex colors (no faces), 4 for vertex, vertex colors and faces
	int vertex_mode = 1;
	int ply_mode = 0;	  // 0 for ascii, 1 for little-endian
	int color_channel_num = 0;
	int vertex_quality_dim = 0; // vertex quality, such as vertex normal or other data per vertex defined by user
	int vertex_normal_dim = 0; // vertex normal dimension
	char seps[] = " ,\t\n\r "; // separators
	seps[5] = 10;
	int property_num = 0;
	char line[1024];
	while (true)
	{
		fgets(line, 1024, fin);
		char* token = strtok(line, seps);
		if (!strcmp(token, "end_header")) break;
		else if (!strcmp(token, "format"))
		{
			token = strtok(NULL, seps);
			if (!strcmp(token, "ascii"))
				ply_mode = 0;
			else if (!strcmp(token, "binary_little_endian"))
				ply_mode = 1;
			else
			{
				cout << "WARNING: can not read this type of PLY model: " << string(token) << endl;
				return false;
			}
		}
		else if (!strcmp(token, "element"))
		{
			token = strtok(NULL, seps);
			if (!strcmp(token, "vertex"))
			{
				// vertex count
				token = strtok(NULL, seps);
				sscanf(token, "%d", &vertex_num_);
				vertex_mode = 1;
			}
			else if (!strcmp(token, "face"))
			{
				// Face count
				token = strtok(NULL, seps);
				sscanf(token, "%d", &face_num_);
				vertex_mode++; // mode with faces is 1 larger than mode without faces
			}
		}
		else if (!strcmp(token, "property"))
		{
			if (vertex_mode % 2 == 1)
			{
				if (property_num >= 3) // skip property x,y,z
				{
					token = strtok(NULL, seps);
					if (!strcmp(token, "uchar")) // color
					{
						token = strtok(NULL, seps);
						color_channel_num++;
						if (color_channel_num >= 3) // color channel number must be 3 (RGB) or 4 (RGBA)
							vertex_mode = 3;
					}
					else if (!strcmp(token, "float")) // vertex quality data
					{
						// Currently just count it and skip
						token = strtok(NULL, seps);
						if (!strcmp(token, "nx") || !strcmp(token, "ny") || !strcmp(token, "nz"))
							vertex_normal_dim++;
						else
							vertex_quality_dim++;
					}
				}
				property_num++;
			}
			else if (vertex_mode % 2 == 0)
			{
				token = strtok(NULL, seps);
				bool face_flag = false;
				if (!strcmp(token, "list"))
				{
					token = strtok(NULL, seps);
					if (!strcmp(token, "uint8") || !strcmp(token, "uchar"))
					{
						token = strtok(NULL, seps);
						if (!strcmp(token, "int") || !strcmp(token, "int32"))
							face_flag = true;
					}
					if (!face_flag)
					{
						cout << "ERROR in Reading PLY model: the type of 'number of face indices' is not 'unsigned char', or the type of 'vertex_index' is not 'int'." << endl;
						return false;
					}
				}
			}
		}
	}
	if (color_channel_num != 0 && color_channel_num != 3 && color_channel_num != 4)
	{
		cout << "ERROR: Color channel number is " << color_channel_num << " but it has to be 0, 3, or 4." << endl;
		return false;
	}
	if (vertex_normal_dim != 0 && vertex_normal_dim != 3)
	{
		cout << "ERROR: Vertex normal dimension is " << vertex_normal_dim << " but it has to be 0 or 3." << endl;
		return false;
	}

	/************************************************************************/
	/* Read vertices and faces */
	vertices_.reserve(vertex_num_);
	faces_.reserve(face_num_);
	if (ply_mode == 1) // binary mode
	{
		for (int i = 0; i < vertex_num_; i++)
		{
			// The vertex data order is: coordinates -> normal -> color -> others (qualities, radius, curvatures, etc)
			size_t haveread = 0;
			Vertex vtx;
			float vert[3];
			if ((haveread = fread(vert, sizeof(float), 3, fin)) != 3)
			{
				cout << "ERROR in reading PLY vertices in position " << ftell(fin) << endl;
				return false;
			}
			if (vertex_normal_dim)
			{
				float nor[3];
				if ((haveread = fread(nor, sizeof(float), vertex_normal_dim, fin)) != vertex_normal_dim)
				{
					cout << "ERROR in reading PLY vertex normals in position " << ftell(fin) << endl;
					return false;
				}
				// Currently we just abandon the vertex normal
				//vtx.normal = Vector3d(nor[0], nor[1], nor[2]);
			}
			if (color_channel_num)
			{
				unsigned char color[4];
				if ((haveread = fread(color, sizeof(unsigned char), color_channel_num, fin)) != color_channel_num)
				{
					cout << "ERROR in reading PLY vertex colors in position " << ftell(fin) << endl;
					return false;
				}
				vtx.color = Vector3f(color[0], color[1], color[2]) / 255;
			}
			if (vertex_quality_dim)
			{
				float qual[3]; // Currently we just abandon the vertex quality data
				if ((haveread = fread(qual, sizeof(float), vertex_quality_dim, fin)) != vertex_quality_dim)
				{
					cout << "ERROR in reading PLY vertex qualities in position " << ftell(fin) << endl;
					return false;
				}
				// Currently we just abandon the vertex normal
			}
			vtx.pt = Vector3d(vert[0], vert[1], vert[2]);
			vertices_.push_back(vtx);
		}

		// Face data
		for (int i = 0; i < face_num_; ++i)
		{
			unsigned char channel_num;
			size_t haveread = fread(&channel_num, 1, 1, fin); // channel number for each face
			Face fa;
			if ((haveread = fread(fa.indices, sizeof(int), 3, fin)) != 3) // currently only support triangular face
			{
				cout << "ERROR in reading PLY face indices in position " << ftell(fin) << endl;
				return false;
			}
			faces_.push_back(fa);
		}
	}
	else // ASCII mode (face reader is still unfinished)
	{
		// Read vertices
		for (int i = 0; i < vertex_num_; i++)
		{
			// The vertex data order is: coordinates -> normal -> color -> others (qualities, radius, curvatures, etc)
			fgets(line, 1024, fin);
			char* token = strtok(line, seps);
			// Read 3D point
			Vertex vtx;
			float vert[3];
			for (int j = 0; j < 3; ++j)
			{
				token = strtok(NULL, seps);
				sscanf(token, "%f", &(vert[j]));
			}
			// Read vertex normal
			if (vertex_normal_dim)
			{
				float nor[3];
				for (int j = 0; j < vertex_normal_dim; ++j)
				{
					token = strtok(NULL, seps);
					sscanf(token, "%f", &(nor[j]));
				}
				// Currently we just abandon the vertex normal data
				//vtx.normal = Vector3d(nor[0], nor[1], nor[2]);
			}
			if (color_channel_num)
			{
				unsigned char color[4];
				for (int j = 0; j < vertex_quality_dim; ++j)
				{
					token = strtok(NULL, seps);
					sscanf(token, "%c", &(color[j]));
				}
				vtx.color = Vector3f(color[0], color[1], color[2]) / 255;
			}
			if (vertex_quality_dim)
			{
				float qual;
				for (int j = 0; j < vertex_quality_dim; ++j)
				{
					token = strtok(NULL, seps);
					sscanf(token, "%f", &qual);
				}
				// Currently we just abandon the vertex quality data
			}
			vtx.pt = Vector3d(vert[0], vert[1], vert[2]);
			vertices_.push_back(vtx);
		}
		// Read Faces
		for (int i = 0; i < face_num_; i++)
		{
			fgets(line, 1024, fin);
			char* token = strtok(line, seps);
			token = strtok(NULL, seps);
			for (int j = 0; j < 3; ++j)
			{
				token = strtok(NULL, seps);
				sscanf(token, "%d", &(faces_[i].indices[j]));
			}
		}
	}
	return true;
}

bool MeshPartition::writePLYWithFaceColors(const string filename)
{
	FILE *fout = NULL;
	fout = fopen(filename.c_str(), "wb"); // write in binary mode
	if (fout == NULL)
	{
		cout << "Unable to create file " << filename << endl;
		return false;
	}
	// Write headers
	fprintf(fout, "ply\n");
	fprintf(fout, "format binary_little_endian 1.0\n");
	fprintf(fout, "element vertex %d\n", vertex_num_);
	fprintf(fout, "property float x\n");
	fprintf(fout, "property float y\n");
	fprintf(fout, "property float z\n");
	fprintf(fout, "element face %d\n", face_num_);
	fprintf(fout, "property list uchar int vertex_indices\n");
	fprintf(fout, "property uchar red\n"); // face color
	fprintf(fout, "property uchar green\n");
	fprintf(fout, "property uchar blue\n");
	fprintf(fout, "property uchar alpha\n");
	fprintf(fout, "end_header\n");
	float pt3[3];
	unsigned char kFaceVtxNum = 3;
	unsigned char rgba[4] = { 255 };
	for (int i = 0; i != vertex_num_; ++i)
	{
		for (int j = 0; j < 3; ++j)
			pt3[j] = float(vertices_[i].pt[j]);
		fwrite(pt3, sizeof(float), 3, fout);
	}
	for (int i = 0; i != face_num_; ++i)
	{
		fwrite(&kFaceVtxNum, sizeof(unsigned char), 1, fout);
		fwrite(faces_[i].indices, sizeof(int), 3, fout);
		int cidx = faces_[i].cluster_id;
		if (cidx == -1)
		{
			cout << "WARNING: face " << i << " doesn't belong to any cluster!" << endl;
			continue;
		}
		for (int j = 0; j < 3; ++j)
			rgba[j] = (unsigned char)(clusters_[cidx].color[j] * 255);
		fwrite(rgba, sizeof(unsigned char), 4, fout);
	}
	fclose(fout);
	return true;
}

bool MeshPartition::writeSimplifiedPLY(const string filename)
{
	int new_face_num = 0, new_vertex_num = 0;
	unordered_map<int, int> vtx_old2new;
	for (int i = 0; i < vertex_num_; ++i)
	{
		if (vertices_[i].is_valid)
		{
			vtx_old2new[i] = new_vertex_num;
			new_vertex_num++;
		}
	}
	for (int i = 0; i < face_num_; ++i)
	{
		if (faces_[i].is_valid)
			new_face_num++;
	}
	FILE *fout = NULL;
	fout = fopen(filename.c_str(), "wb"); // write in binary mode
	if (fout == NULL)
	{
		cout << "Unable to create file " << filename << endl;
		return false;
	}
	// Write headers
	fprintf(fout, "ply\n");
	fprintf(fout, "format binary_little_endian 1.0\n");
	fprintf(fout, "element vertex %d\n", new_vertex_num);
	fprintf(fout, "property float x\n");
	fprintf(fout, "property float y\n");
	fprintf(fout, "property float z\n");
	fprintf(fout, "element face %d\n", new_face_num);
	fprintf(fout, "property list uchar int vertex_indices\n");
	fprintf(fout, "property uchar red\n"); // face color
	fprintf(fout, "property uchar green\n");
	fprintf(fout, "property uchar blue\n");
	fprintf(fout, "property uchar alpha\n");
	fprintf(fout, "end_header\n");

	float pt3[3];
	int f3[3];
	unsigned char kFaceVtxNum = 3;
	unsigned char rgba[4] = { 255 };
	for (int i = 0; i < vertex_num_; ++i)
	{
		if (vertices_[i].is_valid)
		{
			for (int j = 0; j < 3; ++j)
				pt3[j] = float(vertices_[i].pt[j]);
			fwrite(pt3, sizeof(float), 3, fout);
		}
	}
	for (int i = 0; i != face_num_; ++i)
	{
		if (faces_[i].is_valid)
		{
			for (int j = 0; j < 3; ++j)
				f3[j] = vtx_old2new[faces_[i].indices[j]];
			fwrite(&kFaceVtxNum, sizeof(unsigned char), 1, fout);
			fwrite(f3, sizeof(int), 3, fout);
			int cidx = faces_[i].cluster_id;
			if (cidx == -1)
			{
				cout << "WARNING: face " << i << " doesn't belong to any cluster!" << endl;
				continue;
			}
			for (int j = 0; j < 3; ++j)
				rgba[j] = (unsigned char)(clusters_[cidx].color[j] * 255);
			fwrite(rgba, sizeof(unsigned char), 4, fout);
		}
	}
	fclose(fout);
	return true;
}

void MeshPartition::saveClusterFile(const string filename)
{
	FILE *fout = fopen(filename.c_str(), "wb");
	fwrite(&cluster_num_, sizeof(int), 1, fout);
	for (int i = 0; i < face_num_; ++i)
	{
		if (clusters_[i].isValid())
		{
			int num = int(clusters_[i].elements.size());
			int cidx = clusters_old2new_[i];
			fwrite(&cidx, sizeof(int), 1, fout);
			fwrite(&num, sizeof(int), 1, fout);
			vector<int> indices = vector<int>(clusters_[i].elements.begin(), clusters_[i].elements.end());
			fwrite(&indices[0], sizeof(int), num, fout);
		}
	}
	fclose(fout);
}

void MeshPartition::saveSimplifiedClusterFile(const string filename)
{
	for (int i = 0; i < face_num_; ++i)
		clusters_[i].elements.clear();
	int count = 0;
	for (int i = 0; i < face_num_; ++i)
	{
		if (faces_[i].is_valid)
			clusters_[faces_[i].cluster_id].elements.insert(count++);
	}
	getCluterIdxMappingBtwnOldAndNew();

	FILE *fout = fopen(filename.c_str(), "wb");
	fwrite(&cluster_num_, sizeof(int), 1, fout);
	for (int i = 0; i < face_num_; ++i)
	{
		if (clusters_[i].isValid())
		{
			int num = int(clusters_[i].elements.size());
			int cidx = clusters_old2new_[i];
			fwrite(&cidx, sizeof(int), 1, fout);
			fwrite(&num, sizeof(int), 1, fout);
			vector<int> indices = vector<int>(clusters_[i].elements.begin(), clusters_[i].elements.end());
			fwrite(&indices[0], sizeof(int), num, fout);
		}
	}
	fclose(fout);
}


void MeshPartition::getFaceAndVertexNeighbors()
{
	vector<int> fa(3);
	for (int i = 0; i < face_num_; i++)
	{
		if (!faces_[i].is_valid) continue;
		for (int j = 0; j < 3; ++j)
			fa[j] = faces_[i].indices[j];
		std::sort(fa.begin(), fa.end());
		for (int j = 0; j < 3; ++j)
		{
			vertices_[fa[j]].neighbors.insert(fa[(j + 1) % 3]);
			vertices_[fa[j]].neighbors.insert(fa[(j + 2) % 3]);
			vertices_[fa[j]].belonging_faces.insert(i);
			int a = (j == 2) ? fa[0] : fa[j];
			int b = (j == 2) ? fa[j] : fa[j + 1];
			long long edge = getKey(a, b);
			for (int fidx : edge2faces_[edge])
			{
				faces_[fidx].neighbors.insert(i);
				faces_[i].neighbors.insert(fidx);
			}
			edge2faces_[edge].push_back(i);
		}
	}
}

// Release the object 'cluster_edges_' since it is special that each edge pointer is saved in two vectors.
void MeshPartition::clearClusterEdges()
{
	for (int i = 0; i < cluster_edges_.size(); ++i)
	{
		for (Edge* e : cluster_edges_[i])
		{
			int u = e->v1 == i ? e->v2 : e->v1;
			eraseEdgeFromList(u, e);
			heap_.remove(e);
			// Each edge's pointer is stored twice in the edge list, so only delete it once.
			delete e;
		}
		cluster_edges_[i].clear();
	}
}

void MeshPartition::clearHeap()
{
	while (heap_.size())
		heap_.extract();
}

/************************************************************************/
/* Partition
*/
/************************************************************************/
void MeshPartition::runMeshPartition(int cluster_num)
{
	cluster_num_ = cluster_num;

	cout << "Merging ..." << endl;
	merging();

	cout << "Swapping ..." << endl;
	swapAll();

	cout << "Post-processing noisy clusters ... " << endl;
	postProcessClusters();
}

void MeshPartition::merging()
{
	initMerging();
	while (curr_cluster_num_ > cluster_num_)
	{
		if (!runFaceEdgeContractionOnce())
			return;
	}
	assert(curr_cluster_num_ == cluster_num_);
	collectClusterEnergies();
}

void MeshPartition::swapAll()
{
	double last_energy = totalEnergy();
	cout << "   Energy(0): " << last_energy << endl;
	double curr_energy = 0;
	const int kMaxIter = 300;
	int iter = 0;
	while (iter++ < kMaxIter)
	{
		swapOnce();
		curr_energy = totalEnergy();
		cout << "   Energy(" << iter << "): " << curr_energy << endl;
		if ((last_energy - curr_energy) / last_energy < 1e-7 || count_swapping_faces_ == 0)
			break;
		last_energy = curr_energy;
	}
}

void MeshPartition::initMerging()
{
	getFaceAndVertexNeighbors();
	clusters_.resize(face_num_);
	for (int i = 0; i < face_num_; ++i)
	{
		Face& face = faces_[i];
		face.cluster_id = i;
		clusters_[i].elements.insert(i); // initially each face is a cluster
		CovObj Q(vertices_[face.indices[0]].pt, vertices_[face.indices[1]].pt, vertices_[face.indices[2]].pt);
		clusters_[i].ini_cov = Q;
	}
	updateClusterCovs();
	createClusterEdgesAndHeap();
	curr_cluster_num_ = face_num_;
}

void MeshPartition::updateClusterCovs()
{
	for (int i = 0; i < face_num_; ++i)
	{
		if (clusters_[i].isValid())
		{
			for (int fidx : clusters_[i].elements)
			{
				clusters_[i].cov += clusters_[fidx].ini_cov;
				faces_[fidx].cluster_id = i;
			}
		}
	}
}

void MeshPartition::createClusterEdgesAndHeap()
{
	if (cluster_edges_.empty())
		cluster_edges_.resize(face_num_);
	for (int i = 0; i < face_num_; ++i)
	{
		if (clusters_[i].isValid())
		{
			findClusterNeighbors(i);
			for (int ncidx : clusters_[i].neighbors)
			{
				if (i < ncidx) // only update each edge once
				{
					Edge* edge = new Edge(i, ncidx);
					computeEdgeEnergy(edge);
					updateEdgeInHeap(edge);
					cluster_edges_[i].push_back(edge);
					cluster_edges_[ncidx].push_back(edge);
				}
			}
		}
	}
}

// Find a cluster's neighbor clusters
int MeshPartition::findClusterNeighbors(int cidx)
{
	return findClusterNeighbors(cidx, clusters_[cidx].elements, clusters_[cidx].neighbors);
}

int MeshPartition::findClusterNeighbors(int cidx, unordered_set<int>& cluster_elements, unordered_set<int>& neighbors)
{
	neighbors.clear();
	for (int fidx : cluster_elements)
	{
		for (int nidx : faces_[fidx].neighbors)
		{
			int target_id = faces_[nidx].cluster_id;
			if (target_id != cidx)
				neighbors.insert(target_id);
		}
	}
	return int(neighbors.size());
}

void MeshPartition::computeEdgeEnergy(Edge* edge)
{
	CovObj cov1 = clusters_[edge->v1].cov;
	cov1 += clusters_[edge->v2].cov;
	double energy = cov1.energy() - clusters_[edge->v1].cov.energy() - clusters_[edge->v2].cov.energy();
	edge->heap_key(-energy); // it is a max-heap by default but we need a min-heap
}

void MeshPartition::updateEdgeInHeap(Edge* edge)
{
	if (edge->is_in_heap())
		heap_.update(edge);
	else
		heap_.insert(edge);
}

void MeshPartition::collectClusterEnergies()
{
	for (int i = 0; i < face_num_; i++)
		clusters_[i].energy = (clusters_[i].isValid()) ? clusters_[i].cov.energy() : 0;
}

bool MeshPartition::runFaceEdgeContractionOnce()
{
	Edge* edge = (Edge*)heap_.extract();
	if (!edge)
	{
		cout << "  ERROR: No edge exists in the heap. Quitting..." << endl;
		return false;
	}
	if (clusters_[edge->v1].isValid() && clusters_[edge->v2].isValid())
	{
		applyFaceEdgeContraction(edge);
		curr_cluster_num_--;
	}
	else
	{
		cout << "  ERROR: This edge does not exist in clusters. Something is wrong. Quiting..." << endl;
		return false;
	}
	return true;
}

void MeshPartition::applyFaceEdgeContraction(Edge* edge)
{
	int c1 = edge->v1, c2 = edge->v2;
	mergeClusters(c1, c2);
	clusters_[c1].cov += clusters_[c2].cov;

	// Get all neighbors of the new merged cluster v1
	findClusterNeighbors(c1);

	// Remove all old edges of both vertices from the heap and edge list
	for (Edge* e : cluster_edges_[c1])
	{
		int u = (e->v1 == c1) ? e->v2 : e->v1;
		heap_.remove(e);
		eraseEdgeFromList(u, e);
		//if (u != c2)
		delete e;
	}
	for (Edge* e : cluster_edges_[c2])
	{
		int u = (e->v1 == c2) ? e->v2 : e->v1;
		heap_.remove(e);
		eraseEdgeFromList(u, e);
		//if (u != c1)
		delete e;
	}
	cluster_edges_[c1].clear();
	cluster_edges_[c2].clear();

	// Add new edges between v1 and all its new neighbors into edge list
	for (int cidx : clusters_[c1].neighbors)
	{
		Edge *e = new Edge(c1, cidx);
		cluster_edges_[c1].push_back(e);
		cluster_edges_[cidx].push_back(e);
	}

	// Compute all new edges' energies and update/insert them in the max-heap
	for (Edge* e : cluster_edges_[c1])
	{
		computeEdgeEnergy(e);
		updateEdgeInHeap(e);
	}
}

void MeshPartition::eraseEdgeFromList(int cidx, Edge* edge)
{
	for (int i = 0; i < cluster_edges_[cidx].size(); i++)
	{
		if (cluster_edges_[cidx][i] == edge)
		{
			cluster_edges_[cidx].erase(cluster_edges_[cidx].begin() + i);
		}
	}
}

// Merge cluster c2 into cluster c1
void MeshPartition::mergeClusters(int c1, int c2)
{
	for (int fidx : clusters_[c2].elements)
	{
		clusters_[c1].elements.insert(fidx);
		faces_[fidx].cluster_id = c1;
	}
	clusters_[c2].elements.clear();
}

void MeshPartition::createClusterColors()
{
	//srand(time(NULL)); // randomize seed
	for (size_t i = 0; i < face_num_; ++i)
	{
		if (clusters_[i].isValid()) // create a random color
			clusters_[i].color = Vector3f(float(rand()) / RAND_MAX, float(rand()) / RAND_MAX, float(rand()) / RAND_MAX);
	}
}

/************************************************************************/
/* Swap
*/
/************************************************************************/

void MeshPartition::swapOnce()
{
	initSwap();
	findSwapClusters();
	applySwap();
	collectClusterEnergies();
}

void MeshPartition::initSwap()
{
	clusters_in_swap_.clear();
	for (int i = 0; i < face_num_; ++i)
		clusters_[i].faces_to_swap.clear();
	for (int i = 0; i < face_num_; ++i)
	{
		if (clusters_[i].isValid())
			clusters_in_swap_.insert(i);
	}
}

void MeshPartition::findSwapClusters()
{
	count_swapping_faces_ = 0;
	// Currently all valid clusters are involved in swapping process, but it can be
	// improved to involve only relevant clusters.
	for (int cidx : clusters_in_swap_)
	{
		for (int fidx : clusters_[cidx].elements)
		{
			unordered_set<int> visited_clusters;
			double max_delta_energy = 0;
			int max_cidx = -1;
			for (int nidx : faces_[fidx].neighbors)
			{
				int ncidx = faces_[nidx].cluster_id;
				// Skip the neighbor clusters visited before
				if (ncidx != cidx && visited_clusters.count(ncidx) == 0)
				{
					visited_clusters.insert(ncidx);
					double delta_energy = computeSwapDeltaEnergy(fidx, cidx, ncidx);
					if (delta_energy > max_delta_energy)
					{
						max_cidx = ncidx;
						max_delta_energy = delta_energy;
					}
				}
			}
			if (max_cidx != -1)
			{
				SwapFace sf(fidx, cidx, max_cidx);
				clusters_[cidx].faces_to_swap.push_back(sf);
				count_swapping_faces_++;
			}
		}
	}
	cout << "   #swappingfaces: " << count_swapping_faces_ << endl;
}

// Compute delta energy (energy changes) by swapping face 'fidx' from cluster 'from' to a neighbor cluster 'to'
double MeshPartition::computeSwapDeltaEnergy(int fidx, int from, int to)
{
	double energy0 = clusters_[from].energy + clusters_[to].energy;
	CovObj cov_from = clusters_[from].cov, cov_to = clusters_[to].cov;
	cov_from -= clusters_[fidx].ini_cov;
	cov_to += clusters_[fidx].ini_cov;
	double energy1 = cov_from.energy() + cov_to.energy();
	return energy0 - energy1;
}

void MeshPartition::applySwap()
{
	for (int i = 0; i < face_num_; ++i)
	{
		for (SwapFace& sf : clusters_[i].faces_to_swap)
		{
			int from = sf.from;
			int to = sf.to;
			int fidx = sf.face_id;
			faces_[fidx].cluster_id = to;
			clusters_[to].cov += clusters_[fidx].ini_cov;
			clusters_[from].cov -= clusters_[fidx].ini_cov;
			clusters_[from].elements.erase(fidx);
			clusters_[to].elements.insert(fidx);
		}
	}
}

double MeshPartition::totalEnergy()
{
	double energy = 0;
	for (int i = 0; i < face_num_; i++)
	{
		if (clusters_[i].isValid())
			energy += clusters_[i].energy;
	}
	return energy;
}

/************************************************************************/
/* Post-processing after swap
*/
/************************************************************************/

void MeshPartition::postProcessClusters()
{
	// Split clusters into connected components
	for (int i = 0; i < face_num_; ++i)
	{
		if (!clusters_[i].isValid() || clusters_[i].is_new) // skip newly created clusters
			continue;
		vector<unordered_set<int>> connected_components;
		if (!checkClusterConnectivity(i, connected_components))
		{
			// sorting here is to ensure the largest component is on top.
			auto cmp = [](unordered_set<int>& a, unordered_set<int>& b) { return a.size() > b.size(); };
			std::sort(connected_components.begin(), connected_components.end(), cmp);
			splitCluster(i, connected_components);
		}
	}

	createClusterColors();
	getCluterIdxMappingBtwnOldAndNew();
	updateFaceClusterIDs();
}

// Return true if the cluster only contains ONE connected components (don't need to split),
// and false otherwise.
bool MeshPartition::checkClusterConnectivity(int cidx, vector<unordered_set<int>>& connected_components)
{
	int component_idx = 0, count_left = int(clusters_[cidx].elements.size());
	connected_components.push_back(unordered_set<int>());
	for (int fidx : clusters_[cidx].elements)
	{
		assert(faces_[fidx].cluster_id == cidx);
		int count = traverseFaceBFS(fidx, cidx, connected_components[component_idx]);
		if (count == 0) continue;
		if (count == clusters_[cidx].elements.size())
			return true;
		count_left -= count;
		if (count_left == 0) break;
		connected_components.push_back(unordered_set<int>());
		component_idx++;
	}
	return false;
}

int MeshPartition::traverseFaceBFS(int fidx, int cidx, unordered_set<int>& component)
{
	if (faces_[fidx].is_visited) return 0;
	faces_[fidx].is_visited = true;
	queue<int> qe;
	qe.push(fidx);
	while (!qe.empty())
	{
		fidx = qe.front();
		qe.pop();
		component.insert(fidx);
		for (int nfidx : faces_[fidx].neighbors)
		{
			if (faces_[nfidx].is_visited || faces_[nfidx].cluster_id != cidx)
				continue;
			faces_[nfidx].is_visited = true;
			qe.push(nfidx);
		}
	}
	return int(component.size());
}

int MeshPartition::getCluterIdxMappingBtwnOldAndNew()
{
	cluster_num_ = 0;
	clusters_new2old_.clear();
	clusters_old2new_.clear();
	for (int i = 0; i < face_num_; ++i)
	{
		if (clusters_[i].isValid())
		{
			clusters_new2old_[cluster_num_] = i;
			clusters_old2new_[i] = cluster_num_;
			cluster_num_++;
		}
	}
	return cluster_num_;
}

void MeshPartition::updateFaceClusterIDs()
{
	for (int i = 0; i < cluster_num_; ++i)
	{
		int cidx = clusters_new2old_[i];
		for (int fidx : clusters_[cidx].elements)
		{
			faces_[fidx].cluster_id = cidx;
		}
	}
}

void MeshPartition::splitCluster(int original_cidx, vector<unordered_set<int>>& connected_components)
{
	assert(connected_components.size() > 1);
	for (int i = 1; i < connected_components.size(); ++i) // Leave the first/largest component
	{
		unordered_set<int> neighbors;
		int neighbor_num = findClusterNeighbors(original_cidx, connected_components[i], neighbors);
		assert(neighbor_num > 0);
		int target_cidx = *neighbors.begin();
		if (neighbor_num > 1)
		{
			// Find a neighbor with largest energy decrease
			CovObj cov_component;
			for (int fidx : connected_components[i])
				cov_component += clusters_[fidx].ini_cov;
			double delta_energy = -10000;
			for (int ncidx : neighbors)
			{
				CovObj cov_merged = cov_component;
				cov_merged += clusters_[ncidx].cov;
				double denergy = clusters_[ncidx].cov.energy() + cov_component.energy() - cov_merged.energy();
				if (denergy > delta_energy)
				{
					target_cidx = ncidx;
					delta_energy = denergy;
				}
			}
		}
		// Merge this component into its neighbor cluster
		for (int fidx : connected_components[i])
		{
			clusters_[target_cidx].elements.insert(fidx);
			clusters_[original_cidx].elements.erase(fidx);
			clusters_[original_cidx].cov -= clusters_[fidx].ini_cov;
			clusters_[target_cidx].cov += clusters_[fidx].ini_cov;
			faces_[fidx].cluster_id = target_cidx;
			faces_[fidx].is_visited = false; // do NOT forget this
		}
	}

}

/************************************************************************/
/* Mesh Simplification
*/
/************************************************************************/

void MeshPartition::runClusterInnerEdgeSimp(double ratio)
{
	edge_simp_type_ = INNER_EDGES;
	getBorderVertices();
	initInnerEdgeQuadrics();
	simp_ratio_ = ratio;
	contractInnerEdges();
}

void MeshPartition::runClusterBorderEdgeSimp(double ratio)
{
	simp_ratio_ = ratio;
	if (edge_simp_type_ == ALL_BORDER_EDGES)
	{
		initBorderEdgeContraction();
		contractAllBorderEdges();
	}
	else if (edge_simp_type_ == BORDER_EDGES_BY_CLUSTER)
	{
		initBorderEdgeContraction();
		contractBorderEdgesByCluster();
	}
}

void MeshPartition::contractInnerEdges()
{
	clearClusterEdges();
	for (int i = 0; i < cluster_num_; ++i)
	{
		//cout << "   Processing cluster " << i << "/" << cluster_num_ << endl;
		clearHeap();
		int cidx = clusters_new2old_[i];
		createInnerHeapEdgeForCluster(cidx);
		edge_num_ = heap_.size();
		target_edge_num_ = max(int(simp_ratio_ * edge_num_), kMinEdgeNum);
		while (edge_num_ > target_edge_num_)
		{
			Edge* edge = (Edge*)heap_.extract();
			if (!edge)
			{
				cout << "  ERROR: No edge exists in the heap. Quitting..." << endl;
				return;
			}
			applyVtxEdgeContraction(edge, cidx);
		}
		assert(edge_num_ <= target_edge_num_);
	}
}

void MeshPartition::contractAllBorderEdges()
{
	createClusterEdgesAndHeap();
	clearHeap();
	createAllBorderHeapEdges();
	edge_num_ = heap_.size();
	target_edge_num_ = max(int(simp_ratio_ * edge_num_), kMinEdgeNum);
	while (edge_num_ > target_edge_num_)
	{
		Edge* edge = (Edge*)heap_.extract();
		if (!edge)
		{
			cout << "  ERROR: No edge exists in the heap. Quitting..." << endl;
			return;
		}
		applyVtxEdgeContraction(edge);
	}
	assert(edge_num_ <= target_edge_num_);
}

void MeshPartition::contractBorderEdgesByCluster()
{
	for (int i = 0; i < cluster_num_; ++i)
	{
		//cout << "   Processing cluster " << i << "/" << cluster_num_ << endl;
		clearClusterEdges();
		clearHeap();
		int cidx = clusters_new2old_[i];
		createBorderHeapEdgeForCluster(cidx); // find border edges for each cluster before simp
		edge_num_ = heap_.size();
		target_edge_num_ = max(int(simp_ratio_ * edge_num_), kMinEdgeNum);
		while (edge_num_ > target_edge_num_)
		{
			Edge* edge = (Edge*)heap_.extract();
			if (!edge)
			{
				cout << "  ERROR: No edge exists in the heap. Quitting..." << endl;
				return;
			}
			applyVtxEdgeContraction(edge, cidx);
		}
		assert(edge_num_ <= target_edge_num_);
	}
}


void MeshPartition::initBorderEdgeContraction()
{
	// Get mapping edges -> faces
	edge2faces_.clear();
	vector<int> fa(3);
	for (int i = 0; i < cluster_num_; ++i)
	{
		int cidx = clusters_new2old_[i];
		clusters_[cidx].elements.clear();
	}
	for (int i = 0; i < face_num_; i++)
	{
		if (!faces_[i].is_valid) continue;
		for (int j = 0; j < 3; ++j)
			fa[j] = faces_[i].indices[j];
		std::sort(fa.begin(), fa.end());
		for (int j = 0; j < 3; ++j)
		{
			int a = (j == 2) ? fa[0] : fa[j];
			int b = (j == 2) ? fa[j] : fa[j + 1];
			long long edge = getKey(a, b);
			edge2faces_[edge].push_back(i);
		}
		int cidx = faces_[i].cluster_id;
		clusters_[cidx].elements.insert(i);
	}
	// Initialize quadrics
	for (int i = 0; i < vertex_num_; ++i)
	{
		vertices_[i].Q.reset();
		vertices_[i].cluster_id = -1;
	}
	// Get border edges
	for (auto it : edge2faces_)
	{
		long long key = it.first;
		int v1, v2;
		getEdge(key, v1, v2);
		size_t n = it.second.size();
		bool flag_border_edge = false;
		if (n == 0 || n > 2)
		{
			cout << "Edge (" << v1 << "," << v2 << ") ";
			if (n == 0) cout << "has empty face list. This shouldn't happen." << endl;
			else cout << "is a non-manifold edge." << endl;
		}
		else if (n == 1) 
			flag_border_edge = true; // mesh border
		else
		{
			int f1 = it.second[0], f2 = it.second[1];
			int c1 = faces_[f1].cluster_id, c2 = faces_[f2].cluster_id;
			if (c1 != -1 && c2 != -1 && c1 != c2)
				flag_border_edge = true; // cluster border
		}
		if (flag_border_edge)
		{
			QEMQuadrics Q(vertices_[v1].pt, vertices_[v2].pt);
			vertices_[v1].Q += Q;
			vertices_[v2].Q += Q;
			vertices_[v1].is_border = vertices_[v2].is_border = true;
			if (edge_simp_type_ == ALL_BORDER_EDGES)
				border_edges_.insert(key);
		}
	}
	// point quadrics
	for (int i = 0; i < vertex_num_; ++i)
	{
		if (vertices_[i].is_border)
		{
			QEMQuadrics Q(vertices_[i].pt);
			Q *= kPointCoefficient;
			vertices_[i].Q += Q;
		}
	}
}

void MeshPartition::getBorderVertices()
{
	for (int i = 0; i < face_num_; ++i)
	{
		if (clusters_[i].isValid())
		{
			for (int fidx : clusters_[i].elements)
				faces_[fidx].cluster_id = i;
			cluster2edges_[i].reserve(clusters_[i].elements.size());
		}
	}
	if (edge2faces_.empty())
		getFaceAndVertexNeighbors();
	for (auto it : edge2faces_)
	{
		long long key = it.first;
		int v1, v2;
		getEdge(key, v1, v2);
		size_t n = it.second.size();
		bool flag_border_edge = false;
		if (n == 0 || n > 2)
		{
			cout << "Edge (" << v1 << "," << v2 << ") ";
			if (n == 0) cout << "has empty face list. This shouldn't happen." << endl;
			else cout << "is a non-manifold edge." << endl;
		}
		else if (n == 1) flag_border_edge = true; // mesh border
		else
		{
			int f1 = it.second[0], f2 = it.second[1];
			int c1 = faces_[f1].cluster_id, c2 = faces_[f2].cluster_id;
			if (c1 == -1 || c2 == -1)
				cout << "Face " << f1 << " or " << f2 << " do not belong to any cluster. This shouldn't happen." << endl;
			else
			{
				if (c1 != c2)
					flag_border_edge = true; // cluster border
				else
					vertices_[v1].cluster_id = vertices_[v2].cluster_id = c1; // inner-cluster edge
			}
		}
		if (flag_border_edge)
			vertices_[v1].is_border = vertices_[v2].is_border = true;
	}
	// Must reset labels of border vertices after checking all edges above, otherwise
	// some border vertices may be falsely set as non-border (with positive cluster id).
	for (int i = 0; i < vertex_num_; ++i)
	{
		if (vertices_[i].is_border)
			vertices_[i].cluster_id = -1;
	}
	// Get all inner edges
	for (auto it : edge2faces_)
	{
		long long key = it.first;
		int v1, v2;
		getEdge(key, v1, v2);
		if (vertices_[v1].is_border || vertices_[v2].is_border || vertices_[v1].cluster_id == -1 || 
			vertices_[v2].cluster_id == -1 || vertices_[v1].cluster_id != vertices_[v2].cluster_id)
			continue;
		cluster2edges_[vertices_[v1].cluster_id].push_back(key);
	}
}

void MeshPartition::createInnerHeapEdgeForCluster(int cluster_idx)
{
	for (long long key : cluster2edges_[cluster_idx])
	// for (auto it : edge2faces_)
	{
		// long long key = it.first;
		int v1, v2;
		getEdge(key, v1, v2);
		// if (vertices_[v1].is_border || vertices_[v2].is_border || vertices_[v1].cluster_id != cluster_idx || vertices_[v2].cluster_id != cluster_idx)
		// 	continue;
		Edge *e = new Edge(v1, v2);
		if (checkEdgeContraction(e))
		{
			updateEdgeInHeap(e);
			cluster_edges_[v1].push_back(e);
			cluster_edges_[v2].push_back(e);
		}
	}
}

void MeshPartition::getAllEdgesForCluster(int cluster_idx)
{
	edge2faces_.clear();
	vector<int> fa(3);
	for (int fidx : clusters_[cluster_idx].elements)
	{
		if (!faces_[fidx].is_valid) continue;
		for (int j = 0; j < 3; ++j)
			fa[j] = faces_[fidx].indices[j];
		std::sort(fa.begin(), fa.end());
		for (int j = 0; j < 3; ++j)
		{
			int a = (j == 2) ? fa[0] : fa[j];
			int b = (j == 2) ? fa[j] : fa[j + 1];
			long long edge = getKey(a, b);
			edge2faces_[edge].push_back(fidx);
		}
	}
}

void MeshPartition::createAllBorderHeapEdges()
{
	for (long long key : border_edges_)
	{
		int v1, v2;
		getEdge(key, v1, v2);
		Edge *e = new Edge(v1, v2);
		if (checkEdgeContraction(e))
		{
			updateEdgeInHeap(e);
			cluster_edges_[v1].push_back(e);
			cluster_edges_[v2].push_back(e);
		}
	}
}

void MeshPartition::createBorderHeapEdgeForCluster(int cluster_idx)
{
	getAllEdgesForCluster(cluster_idx);

	// Get border edges for this cluster
	for (auto it : edge2faces_)
	{
		long long key = it.first;
		int v1, v2;
		getEdge(key, v1, v2);
		size_t n = it.second.size();
		bool flag_border_edge = false;
		if (n == 0 || n > 2) // for debug
		{
			cout << "Edge (" << v1 << "," << v2 << ") ";
			if (n == 0) cout << "has empty face list. This shouldn't happen." << endl;
			else cout << "is a non-manifold edge." << endl;
		}
		else if (n == 1) flag_border_edge = true; // mesh border
		else
		{
			int f1 = it.second[0], f2 = it.second[1];
			int c1 = faces_[f1].cluster_id, c2 = faces_[f2].cluster_id;
			if (c1 == -1 || c2 == -1) // for debug
				cout << "Face " << f1 << " or " << f2 << " do not belong to any cluster. This shouldn't happen." << endl;
			else if (c1 != c2)
				flag_border_edge = true; // cluster border
		}
		if (flag_border_edge)
		{
			Edge *e = new Edge(v1, v2);
			if (checkEdgeContraction(e))
			{
				updateEdgeInHeap(e);
				cluster_edges_[v1].push_back(e);
				cluster_edges_[v2].push_back(e);
				vertices_[v1].cluster_id = vertices_[v2].cluster_id = cluster_idx;
			}
		}
	}
}

void MeshPartition::initInnerEdgeQuadrics()
{
	// face quadrics
	for (int i = 0; i < face_num_; ++i)
	{
		QEMQuadrics Q(vertices_[faces_[i].indices[0]].pt, vertices_[faces_[i].indices[1]].pt, vertices_[faces_[i].indices[2]].pt);
		for (int j = 0; j < 3; ++j)
			vertices_[faces_[i].indices[j]].Q += Q;
	}
	const double kFaceFactor = 1.0 / 3; // normalizing factor from the paper
	for (int i = 0; i < vertex_num_; ++i)
		vertices_[i].Q *= kFaceFactor;
	// point quadrics
	for (int i = 0; i < vertex_num_; ++i)
	{
		QEMQuadrics Q(vertices_[i].pt);
		Q *= kPointCoefficient;
		vertices_[i].Q += Q;
	}
}

bool MeshPartition::checkEdgeContraction(Edge* edge)
{
	int v1 = edge->v1, v2 = edge->v2;
	QEMQuadrics Q = vertices_[v1].Q;
	Q += vertices_[v2].Q;
	double energy = 0;
	Vector3d vtx;
	if (Q.optimize(vtx, energy))
	{
		if (!isContractedVtxValid(edge, v1, vtx) || !isContractedVtxValid(edge, v2, vtx))
			return false;
	}
	else
	{	// A is singular, use one of two endpoints minimizing energy as contracted vertex
		double energy1 = Q(vertices_[v1].pt);
		double energy2 = Q(vertices_[v2].pt);
		energy = energy1 < energy2 ? energy1 : energy2;
	}
	edge->heap_key(-energy); // it is a max-heap by default but we need a min-heap
	return true;
}

// check if the edge contraction can cause simplex inversion
bool MeshPartition::isContractedVtxValid(Edge* edge, int endpoint, const Vector3d& vtx)
{
	assert(endpoint == edge->v1 || endpoint == edge->v2);
	int v1 = endpoint, v2 = (edge->v1 == endpoint) ? edge->v2 : edge->v1;
	int p1 = v1;
	for (int fidx : vertices_[v1].belonging_faces)
	{
		if (vertices_[v2].belonging_faces.find(fidx) != vertices_[v2].belonging_faces.end())
			continue; // skip faces shared by edge
		int idx = -1;
		for (int i = 0; i < 3; ++i)
		{
			if (faces_[fidx].indices[i] == v1)
			{
				idx = i;
				break;
			}
		}
		if (idx == -1)
		{
			cout << "Face " << fidx << " doesn't include vertex " << v1 << ". This shouldn't happen." << endl;
			return false;
		}
		int p2 = faces_[fidx].indices[(idx + 1) % 3], p3 = faces_[fidx].indices[(idx + 2) % 3];
		Vector3d e3 = vertices_[p3].pt - vertices_[p2].pt, e1 = vertices_[p1].pt - vertices_[p2].pt;
		e3.normalize();
		Vector3d n = e1 - (e1.dot(e3)) * e3; // plane normal through v1's opposite edge in the face
		n.normalize();
		if (n.dot(vtx - vertices_[p2].pt) <= 0) // distance(vtx, plane) <= 0
			return false;
	}
	return true;
}

/************************************************************************/
/* Apply edge contraction for inner edge, border edge by cluster, and 
*  global border edge.
*  Note that these cases are very similar except some details.
*/
/************************************************************************/

void MeshPartition::applyVtxEdgeContraction(Edge* edge, int cluster_idx)
{
	int v1 = edge->v1, v2 = edge->v2;
	vertices_[v1].Q += vertices_[v2].Q;
	vertices_[v2].Q.reset();
	vertices_[v2].is_valid = false;
	double energy = 0;
	if (!vertices_[v1].Q.optimize(vertices_[v1].pt, energy))
	{
		double energy1 = vertices_[v1].Q(vertices_[v1].pt);
		double energy2 = vertices_[v1].Q(vertices_[v2].pt);
		if (energy1 > energy2)
			vertices_[v1].pt = vertices_[v2].pt;
	}

	// Update neighbor list of v2's old neighbors
	for (int vidx : vertices_[v2].neighbors)
	{
		vertices_[vidx].neighbors.erase(v2);
		if (vidx != v1)
		{
			vertices_[v1].neighbors.insert(vidx);
			vertices_[vidx].neighbors.insert(v1);
		}
	}
	for (int fidx : vertices_[v2].belonging_faces)
	{
		if (checkFaceContainsVertices(fidx, v1, v2))
		{	// remove faces containing both v1 and v2
			for (int i = 0; i < 3; ++i)
			{
				faces_[fidx].is_valid = false;
				int vidx = faces_[fidx].indices[i];
				if (vidx != v2)
					vertices_[vidx].belonging_faces.erase(fidx);
			}
			if (edge_simp_type_ == BORDER_EDGES_BY_CLUSTER)
			{
				int cidx = faces_[fidx].cluster_id; // erase the face from cluster
				clusters_[cidx].elements.erase(fidx);
			}
		}
		else
		{	// replace v2 in the face by v1
			for (int i = 0; i < 3; ++i)
			{
				if (faces_[fidx].indices[i] == v2)
				{
					faces_[fidx].indices[i] = v1;
					break;
				}
			}
			vertices_[v1].belonging_faces.insert(fidx);
		}
	}

	// Remove all old edges from v1 and v2
	for (Edge* e : cluster_edges_[v1])
	{
		int u = (e->v1 == v1) ? e->v2 : e->v1;
		heap_.remove(e);
		eraseEdgeFromList(u, e);
		delete e;
		edge_num_--;
	}
	for (Edge* e : cluster_edges_[v2])
	{
		int u = (e->v1 == v2) ? e->v2 : e->v1;
		heap_.remove(e);
		eraseEdgeFromList(u, e);
		delete e;
		edge_num_--;
	}
	cluster_edges_[v1].clear();
	cluster_edges_[v2].clear();

	// Add new edges for v1
	for (int vidx : vertices_[v1].neighbors)
	{
		if (edge_simp_type_ == ALL_BORDER_EDGES)
		{
			if (!vertices_[vidx].is_border) 
				continue;
		}
		else
		{
			if (vertices_[vidx].cluster_id != cluster_idx) 
				continue;
		}
		Edge *e = (v1 < vidx) ? (new Edge(v1, vidx)) : (new Edge(vidx, v1));
		if (checkEdgeContraction(e))
		{
			updateEdgeInHeap(e);
			cluster_edges_[v1].push_back(e);
			cluster_edges_[vidx].push_back(e);
			edge_num_++;
		}
		else delete e;
	}
}

void MeshPartition::applyBorderEdgeContractionByCluster(Edge* edge, int cluster_idx)
{
	int v1 = edge->v1, v2 = edge->v2;
	vertices_[v1].Q += vertices_[v2].Q;
	vertices_[v2].Q.reset();
	vertices_[v2].is_valid = false;
	double energy = 0;
	if (!vertices_[v1].Q.optimize(vertices_[v1].pt, energy))
	{
		double energy1 = vertices_[v1].Q(vertices_[v1].pt);
		double energy2 = vertices_[v1].Q(vertices_[v2].pt);
		if (energy1 > energy2)
			vertices_[v1].pt = vertices_[v2].pt;
	}

	// Update neighbor list of v2's old neighbors
	for (int vidx : vertices_[v2].neighbors)
	{
		vertices_[vidx].neighbors.erase(v2);
		if (vidx != v1)
		{
			vertices_[v1].neighbors.insert(vidx);
			vertices_[vidx].neighbors.insert(v1);
		}
	}
	for (int fidx : vertices_[v2].belonging_faces)
	{
		if (checkFaceContainsVertices(fidx, v1, v2))
		{	// remove faces containing both v1 and v2
			for (int i = 0; i < 3; ++i)
			{
				faces_[fidx].is_valid = false;
				int vidx = faces_[fidx].indices[i];
				if (vidx != v2)
					vertices_[vidx].belonging_faces.erase(fidx);
			}
			int cidx = faces_[fidx].cluster_id; // erase the face from cluster
			clusters_[cidx].elements.erase(fidx);
		}
		else
		{	// replace v2 in the face by v1
			for (int i = 0; i < 3; ++i)
			{
				if (faces_[fidx].indices[i] == v2)
				{
					faces_[fidx].indices[i] = v1;
					break;
				}
			}
			vertices_[v1].belonging_faces.insert(fidx);
		}
	}

	// Remove all old edges from v1 and v2
	for (Edge* e : cluster_edges_[v1])
	{
		int u = (e->v1 == v1) ? e->v2 : e->v1;
		heap_.remove(e);
		eraseEdgeFromList(u, e);
		edge_num_--;
		delete e;
	}
	for (Edge* e : cluster_edges_[v2])
	{
		int u = (e->v1 == v2) ? e->v2 : e->v1;
		heap_.remove(e);
		eraseEdgeFromList(u, e);
		edge_num_--;
		delete e;
	}
	cluster_edges_[v1].clear();
	cluster_edges_[v2].clear();

	// Add new edges for v1
	for (int vidx : vertices_[v1].neighbors)
	{
		if (vertices_[vidx].cluster_id != cluster_idx) continue;
		Edge *e = (v1 < vidx) ? (new Edge(v1, vidx)) : (new Edge(vidx, v1));
		if (checkEdgeContraction(e))
		{
			updateEdgeInHeap(e);
			cluster_edges_[v1].push_back(e);
			cluster_edges_[vidx].push_back(e);
			edge_num_++;
		}
		else delete e;
	}
}

