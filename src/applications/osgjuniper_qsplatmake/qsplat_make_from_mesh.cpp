/*
Szymon Rusinkiewicz

qsplat_make_from_mesh.cpp

Code for reading in a triangle mesh from a ply file and initializing the
QSplat data structure.

This does not read and write general .ply files - only a very restricted
subset.  In particular, only binary big-endian ply files with just vertices
and faces or tstrips are supported.

Copyright (c) 1999-2000 The Board of Trustees of the
Leland Stanford Junior University.  All Rights Reserved.
*/

#include <stdio.h>
#include <string.h>
#include <float.h>
#include <vector>
#include "qsplat_make_from_mesh.h"

#define BIGNUM FLT_MAX


// Unpack tstrips into faces
static void unpack_tstrips(int tstripdatalen, const int *tstrips,
			   int &numfaces, face * &faces)
{
	int i;
	if (!tstrips || tstripdatalen < 4)
		return;

	printf("Unpacking triangle strips... "); fflush(stdout);

	// Count number of faces
	numfaces = 0;
	int this_tstrip_len = 0;
	for (i=0; i < tstripdatalen; i++) {
		if (tstrips[i] == -1) {
			this_tstrip_len = 0;
			continue;
		}
		this_tstrip_len++;
		if (this_tstrip_len >= 3)
			numfaces++;
	}
	printf("%d triangles... ", numfaces); fflush(stdout);

	faces = new face[numfaces];

	int whichface = 0;
	this_tstrip_len = 0;
	for (i=0; i < tstripdatalen; i++) {
		if (tstrips[i] == -1) {
			this_tstrip_len = 0;
			continue;
		}
		this_tstrip_len++;
		if (this_tstrip_len < 3)
			continue;
		if (this_tstrip_len % 2) {
			faces[whichface][0] = tstrips[i-2];
			faces[whichface][1] = tstrips[i-1];
		} else {
			faces[whichface][0] = tstrips[i-1];
			faces[whichface][1] = tstrips[i-2];
		}
		faces[whichface++][2] = tstrips[i];
	}

	printf("Done.\n");
}


// Try to read a plyfile, returning vertices and faces
bool read_ply(const char *plyfile,
	      int &numleaves, QTree_Node * &leaves,
	      int &numfaces, face * &faces,
	      bool &have_colors,
	      std::string &comments)
{
	bool have_faces=false, have_tstrips=false;
	int tstripdatalen=0, *tstripdata = NULL;
	int other_prop_len, color_offset;
	char buf[255];
	int i, result;

	have_colors = false;  numleaves = numfaces = 0;
	leaves = NULL;  faces = NULL;

	FILE *f = fopen(plyfile, "r");
	if (!f) {
		fprintf(stderr, "Can't open plyfile %s\n", plyfile);
		return false;
	}
	printf("Reading %s...\n", plyfile);


	// Read header
	if (!fgets(buf, 255, f) || strncmp(buf, "ply", 3)) {
		fprintf(stderr, "Not a ply file.\n");
		return false;
	}

#define GET_LINE() if (!fgets(buf, 255, f)) goto plyreaderror
#define LINE_IS(text) !strncasecmp(buf, text, strlen(text))

	GET_LINE();
	if (!LINE_IS("format binary_big_endian 1.0")) {
		fprintf(stderr, "Can only read binary big-endian ply files.\n");
		return false;
	}

	while (1) {
		GET_LINE();
		if (LINE_IS("obj_info")) {
			continue;
		} else if (LINE_IS("comment")) {
			comments += buf+8;
			continue;
		} else {
			break;
		}
	}

	result = sscanf(buf, "element vertex %d\n", &numleaves);
	if (result != 1) {
		fprintf(stderr, "Expected \"element vertex\"\n");
		goto plyreaderror;
	}

	GET_LINE();
	if (!LINE_IS("property float x")) {
		fprintf(stderr, "Expected \"property float x\"\n");
		goto plyreaderror;
	}

	GET_LINE();
	if (!LINE_IS("property float y")) {
		fprintf(stderr, "Expected \"property float y\"\n");
		goto plyreaderror;
	}

	GET_LINE();
	if (!LINE_IS("property float z")) {
		fprintf(stderr, "Expected \"property float z\"\n");
		goto plyreaderror;
	}

	other_prop_len = 0;
	GET_LINE();
	while (LINE_IS("property")) {
		if (LINE_IS("property char") ||
		    LINE_IS("property uchar")) {
			other_prop_len += 1;
		} else if (LINE_IS("property int") ||
			   LINE_IS("property uint") ||
			   LINE_IS("property float")) {
			other_prop_len += 4;
		} else {
			fprintf(stderr, "Unsupported vertex property: %s\n", buf);
			goto plyreaderror;
		}

		if (LINE_IS("property uchar diffuse_red")) {
			have_colors = true;
			color_offset = other_prop_len - 1;
		}

		GET_LINE();
	}


	result = sscanf(buf, "element face %d", &numfaces);
	if (result == 1) {
		have_faces = true;
		GET_LINE();
		if (!LINE_IS("property list uchar int vertex_indices"))
			goto plyreaderror;
		GET_LINE();
	} else if (LINE_IS("element tristrips 1")) {
		have_tstrips = true;
		GET_LINE();
		if (!LINE_IS("property list int int vertex_indices"))
			goto plyreaderror;
		GET_LINE();
	}

	if (!LINE_IS("end_header")) {
		fprintf(stderr, "Expected \"end_header\"\n");
		goto plyreaderror;
	}


	// OK, we think we've parsed the header. Slurp in the actual data...
	leaves = new QTree_Node[numleaves];

	printf(" Reading %d vertices... ", numleaves); fflush(stdout);
	for (i=0; i < numleaves; i++) {
		if (!fread((void *)&(leaves[i].pos[0]), 12, 1, f))
			goto plyreaderror;
		FIX_FLOAT(leaves[i].pos[0]);
		FIX_FLOAT(leaves[i].pos[1]);
		FIX_FLOAT(leaves[i].pos[2]);

		if (other_prop_len && !fread((void *)buf, other_prop_len, 1, f))
			goto plyreaderror;

		if (have_colors) {
			memcpy((void *)&(leaves[i].col[0]),
			       buf + color_offset,
			       sizeof(color));
		}
	}
	printf("Done.\n");

	if (have_tstrips) {
		printf(" Reading triangle strips... "); fflush(stdout);

		if (!fread((void *)&tstripdatalen, 4, 1, f))
			goto plyreaderror;
		FIX_LONG(tstripdatalen);

		tstripdata = new int[tstripdatalen];
		if (!fread((void *)tstripdata, 4*tstripdatalen, 1, f))
			goto plyreaderror;
		for (int t=0; t < tstripdatalen; t++)
			FIX_LONG(tstripdata[t]);
	} else if (have_faces) {
		printf(" Reading %d faces... ", numfaces); fflush(stdout);
		faces = new face[numfaces];
		for (i=0; i < numfaces; i++) {
			if (!fread((void *)buf, 1, 1, f))
				goto plyreaderror;
			if (buf[0] != 3) {
				fprintf(stderr, "Non-triangle found in mesh.\n");
			}
			if (!fread((void *)faces[i], 12, 1, f))
				goto plyreaderror;
			FIX_LONG(faces[i][0]);
			FIX_LONG(faces[i][1]);
			FIX_LONG(faces[i][2]);
		}
	}
	printf("Done.\n");
	if (tstripdatalen) {
		unpack_tstrips(tstripdatalen, tstripdata, numfaces, faces);
		delete [] tstripdata;
	}

	fgets(buf, 2, f);
	if (!feof(f)) {
		fprintf(stderr, "Warning: ignored excess garbage at end of ply file.\n");
	}

	fclose(f);
	return true;

plyreaderror:
	fclose(f);
	fprintf(stderr, "Error reading plyfile.\n");
	if (leaves) delete [] leaves;
	if (faces) delete [] faces;
	if (tstripdata) delete [] tstripdata;
	return false;
}


// Find normal of a triangle with the given vertices
static inline void FindNormal(const point &p1, const point &p2,
			      const point &p3, vec &n)
{
	vec u = { p2[0]-p1[0], p2[1]-p1[1], p2[2]-p1[2] };
	vec v = { p3[0]-p1[0], p3[1]-p1[1], p3[2]-p1[2] };
                                        
	CrossProd(u, v, n);
}


// Find per-vertex normals
void find_normals(int numleaves, QTree_Node *leaves,
		  int numfaces, const face *faces)
{
	int i;
	printf("Computing normals... "); fflush(stdout);

	for (i=0; i < numleaves; i++)
		leaves[i].norm[0] = leaves[i].norm[1] = leaves[i].norm[2] = 0.0f;

	// For each face...
	for (i=0; i < numfaces; i++) {
		// Find normal
		vec facenormal;
		FindNormal(leaves[faces[i][0]].pos,
			   leaves[faces[i][1]].pos,
			   leaves[faces[i][2]].pos,
			   facenormal);

		// Accumulate. Note that facenormal is not unit-length, so
		// it is really an *area-weighted* normal.
		vec &n0 = leaves[faces[i][0]].norm;
		n0[0] += facenormal[0];
		n0[1] += facenormal[1];
		n0[2] += facenormal[2];

		vec &n1 = leaves[faces[i][1]].norm;
		n1[0] += facenormal[0];
		n1[1] += facenormal[1];
		n1[2] += facenormal[2];

		vec &n2 = leaves[faces[i][2]].norm;
		n2[0] += facenormal[0];
		n2[1] += facenormal[1];
		n2[2] += facenormal[2];
	}

	printf("Done.\n");
}


// Find a sphere that encloses the given triangle
// Based on GGems III V.1, by Fernando Lopez-Lopez
// and GGems I "Triangles" by Ronald Goldman
static inline void TriBoundingSphere(const float *p1,
				     const float *p2,
				     const float *p3,
				     //float *cent,
				     float &r)
{
	vec a = { p2[0] - p3[0], p2[1] - p3[1], p2[2] - p3[2] };
	vec b = { p3[0] - p1[0], p3[1] - p1[1], p3[2] - p1[2] };
	vec c = { p1[0] - p2[0], p1[1] - p2[1], p1[2] - p2[2] };

	float d1 = -Dot(b,c);
	float d2 = -Dot(c,a);
	float d3 = -Dot(a,b);

	// If triangle is obtuse, just want midpt of longest side
	if (d1 <= 0.0f) {

		//cent[0] = 0.5f * (p2[0] + p3[0]);
		//cent[1] = 0.5f * (p2[1] + p3[1]);
		//cent[2] = 0.5f * (p2[2] + p3[2]);
		r = 0.5f * Len(a);
		return;

	} else if (d2 <= 0.0f) {

		//cent[0] = 0.5f * (p3[0] + p1[0]);
		//cent[1] = 0.5f * (p3[1] + p1[1]);
		//cent[2] = 0.5f * (p3[2] + p1[2]);
		r = 0.5f * Len(b);
		return;

	} else if (d3 <= 0.0f) {

		//cent[0] = 0.5f * (p1[0] + p2[0]);
		//cent[1] = 0.5f * (p1[1] + p2[1]);
		//cent[2] = 0.5f * (p1[2] + p2[2]);
		r = 0.5f * Len(c);
		return;

	}

	// Else compute circumcircle
	float e1 = d2*d3;
	float e2 = d3*d1;
	float e3 = d1*d2;
	float e = e1+e2+e3;
	if (e == 0.0f) { r = 0.0f; return; }
	//float tmp = 0.5f / e;
	//cent[0] = ((e2+e3)*p1[0] + (e3+e1)*p2[0] + (e1+e2)*p3[0]) * tmp;
	//cent[1] = ((e2+e3)*p1[1] + (e3+e1)*p2[1] + (e1+e2)*p3[1]) * tmp;
	//cent[2] = ((e2+e3)*p1[2] + (e3+e1)*p2[2] + (e1+e2)*p3[2]) * tmp;
	r = 0.5f * sqrtf((d1+d2)*(d2+d3)*(d3+d1)/e);
}


// Figure out how big the splat at each point has to be.
void find_splat_sizes(int numleaves, QTree_Node *leaves,
		      int numfaces, const face *faces)
{
	int i;
	printf("Computing splat sizes... "); fflush(stdout);

	for (i=0; i < numleaves; i++)
		leaves[i].r = 0.0f;

	for (i=0; i < numfaces; i++) {
		float r;
		int i1 = faces[i][0];
		int i2 = faces[i][1];
		int i3 = faces[i][2];
		TriBoundingSphere(leaves[i1].pos,
				  leaves[i2].pos,
				  leaves[i3].pos,
				  r);
		leaves[i1].r = max(leaves[i1].r, r);
		leaves[i2].r = max(leaves[i2].r, r);
		leaves[i3].r = max(leaves[i3].r, r);
	}
	printf("Done.\n");
}


// Find really short edges in the mesh and merge their endpoints
void merge_nodes(int &numleaves, QTree_Node *leaves,
		 int &numfaces, face *faces,
		 bool havecolor, float thresh)
{
	int i;
	for (i=0; i < numleaves; i++) {
		leaves[i].m.refcount = 0;
		leaves[i].m.remap = i;
	}

	for (i=0; i < numfaces; i++) {
		int v1 = faces[i][0], v2 = faces[i][1], v3 = faces[i][2];
		leaves[v1].m.refcount = leaves[v2].m.refcount =
			leaves[v3].m.refcount = 1;
		if (thresh <= 0.0f)
			continue;
		while (v1 != leaves[v1].m.remap) v1 = leaves[v1].m.remap;
		while (v2 != leaves[v2].m.remap) v2 = leaves[v1].m.remap;
		while (v3 != leaves[v3].m.remap) v3 = leaves[v1].m.remap;

		float d12 = Dist(leaves[v1].pos, leaves[v2].pos);
		float d23 = Dist(leaves[v2].pos, leaves[v3].pos);
		float d31 = Dist(leaves[v3].pos, leaves[v1].pos);

		if ((d12 < thresh) + (d23 < thresh) + (d31 < thresh) >= 2) {
			// This entire triangle goes away...
			leaves[v1].m.remap = leaves[v2].m.remap =
				leaves[v3].m.remap = min(min(v1, v2), v3);
		} else if (d12 < thresh) {
			leaves[v1].m.remap = leaves[v2].m.remap = min(v1,v2);
		} else if (d23 < thresh) {
			leaves[v2].m.remap = leaves[v3].m.remap = min(v2,v3);
		} else if (d31 < thresh) {
			leaves[v3].m.remap = leaves[v1].m.remap = min(v3,v1);
		}
	}

	if (thresh <= 0.0f)
		return;

	for (i=0; i < numleaves; i++) {
		if (leaves[i].m.refcount == 0)
			continue;
		if (leaves[i].m.remap == i) {
			if (havecolor) {
				leaves[i].m.col_tmp[0] = leaves[i].col[0];
				leaves[i].m.col_tmp[1] = leaves[i].col[1];
				leaves[i].m.col_tmp[2] = leaves[i].col[2];
			}
			continue;
		}
		int j = leaves[i].m.remap;
		while (leaves[j].m.remap != j)
			j = leaves[j].m.remap;
		leaves[j].m.refcount++;
		leaves[j].pos[0] += leaves[i].pos[0];
		leaves[j].pos[1] += leaves[i].pos[1];
		leaves[j].pos[2] += leaves[i].pos[2];
		leaves[j].norm[0] += leaves[i].norm[0];
		leaves[j].norm[1] += leaves[i].norm[1];
		leaves[j].norm[2] += leaves[i].norm[2];
		if (havecolor) {
			leaves[j].m.col_tmp[0] += leaves[i].col[0];
			leaves[j].m.col_tmp[1] += leaves[i].col[1];
			leaves[j].m.col_tmp[2] += leaves[i].col[2];
		}
		leaves[i].m.refcount = 0;
	}

	for (i=0; i < numleaves; i++) {
		if (leaves[i].m.refcount < 2)
			continue;
		float x = 1.0f / leaves[i].m.refcount;
		leaves[i].pos[0] *= x;
		leaves[i].pos[1] *= x;
		leaves[i].pos[2] *= x;
		// Normal will get fixed later
		if (havecolor) {
			leaves[i].col[0] = min(max((unsigned char)(leaves[i].m.col_tmp[0] * x + 0.5), (unsigned char)0), (unsigned char)255);
			leaves[i].col[1] = min(max((unsigned char)(leaves[i].m.col_tmp[1] * x + 0.5), (unsigned char)0), (unsigned char)255);
			leaves[i].col[2] = min(max((unsigned char)(leaves[i].m.col_tmp[2] * x + 0.5), (unsigned char)0), (unsigned char)255);
		}
	}

	for (i=0; i < numfaces; i++) {
		while (faces[i][0] != leaves[faces[i][0]].m.remap)
			faces[i][0] = leaves[faces[i][0]].m.remap;
		while (faces[i][1] != leaves[faces[i][1]].m.remap)
			faces[i][1] = leaves[faces[i][1]].m.remap;
		while (faces[i][2] != leaves[faces[i][2]].m.remap)
			faces[i][2] = leaves[faces[i][2]].m.remap;
	}
}

