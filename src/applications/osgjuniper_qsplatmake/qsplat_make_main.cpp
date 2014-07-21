/*
Szymon Rusinkiewicz

qsplat_make_main.cpp
The routine for building .qs files.

Copyright (c) 1999-2000 The Board of Trustees of the
Leland Stanford Junior University.  All Rights Reserved.
*/

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <string.h>
#include "qsplat_make_qtree_v11.h"
#include "qsplat_make_from_mesh.h"

#include <osg/Notify>
#include <osg/io_utils>
#include <osg/NodeVisitor>
#include <osg/Geometry>
#include <osg/TriangleFunctor>
#include <osgDB/ReadFile>


// Version stamp
const char *QSPLATMAKE_VERSION = "1.0";


static void usage(const char *myname)
{
	fprintf(stderr, "Usage: %s [-m threshold] in.ply out.qs\n", myname);
	exit(1);
}


struct Face
{
    Face(unsigned int v0, unsigned int v1, unsigned int v2):
_v0(v0),
_v1(v1),
_v2(v2)
    {
    }
    unsigned int _v0;
    unsigned int _v1;
    unsigned int _v2;
};


/* A list of faces */
typedef std::vector< Face > FaceList;


struct MyTriangleFunctor
{
    MyTriangleFunctor():
_flip(false)
    {
    }

    void operator() (const osg::Vec3& v1,const osg::Vec3& v2,const osg::Vec3& v3, bool) 
    {
        //Face f(_verts->size(), _verts->size()+1, _verts->size()+2);
        Face f(_verts->size(), _verts->size()+1, _verts->size()+2);
        _faces->push_back( f );
        //std::cout << "Added face " << f._v0 << ", " << f._v1 << ", " << f._v2 << std::endl;

        if (_flip)
        {
            _verts->push_back(v1);
            _verts->push_back(v3);
            _verts->push_back(v2);
        }
        else
        {
            _verts->push_back(v1);
            _verts->push_back(v2);
            _verts->push_back(v3);
        }
        //std::cout << "\t("<<v1<<") ("<<v2<<") ("<<v3<<") "<<")"<<std::endl;
    }

    osg::ref_ptr< osg::Vec3Array > _verts;
    FaceList *_faces;
    bool _flip;
};


class CollectVertsAndFacesVisitor : public osg::NodeVisitor
{
public:
    CollectVertsAndFacesVisitor():
      osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN),
      _flip(false)
      {
          _verts = new osg::Vec3Array();
      }

      void apply(osg::Geode& node) {
          osg::notify(osg::NOTICE) << "Applying geode " << std::endl;
          for (int i = 0; i < (int)node.getNumDrawables(); ++i) {
              osg::Geometry* geom = node.getDrawable(i)->asGeometry();
              if (geom)
              {
                  processGeometry( geom );
              }
          }
      }

      void processGeometry(osg::Geometry* geometry)
      {
          osg::TriangleFunctor<MyTriangleFunctor> f;
          f._verts = _verts;
          f._faces = &_faces;
          f._flip = _flip;
          geometry->accept( f );
      }

      osg::ref_ptr< osg::Vec3Array > _verts;
      FaceList _faces;
      bool _flip;
};




/**
*Try to read an OSG file, returning the vertices and faces
*/
bool read_osg(const char *filename,
	      int &numleaves, QTree_Node * &leaves,
	      int &numfaces, face * &faces,
	      bool &have_colors,
          bool flip,
	      std::string &comments)
{
    //Read the OSG file
    osg::ref_ptr< osg::Node > loadedModel = osgDB::readNodeFile( filename );
    if (!loadedModel.valid())
    {
        osg::notify(osg::NOTICE) << "Error reading " << filename << std::endl;
        return false;
    }

   CollectVertsAndFacesVisitor cvf;
   cvf._flip = flip;
   loadedModel->accept( cvf );

   //Convert our data to qSplat datastructures
   numleaves = cvf._verts->size();
   numfaces  = cvf._faces.size();

   leaves = new QTree_Node[numleaves];
   for (unsigned int i = 0; i < numleaves; ++i)
   {
       osg::Vec3 v = (*cvf._verts)[i];
       leaves[i].pos[0] = v.x();
       leaves[i].pos[1] = v.y();
       leaves[i].pos[2] = v.z();
   }

   faces = new face[numfaces];
   for (unsigned int i = 0; i < numfaces; ++i)
   {
       Face f = cvf._faces[i];
       faces[i][0] = f._v0;
       faces[i][1] = f._v1;
       faces[i][2] = f._v2;
   }

   have_colors = false;

    return true;
}





int main(int argc, char *argv[])
{
#ifdef WIN32
	_fmode = _O_BINARY;
#endif
	printf("This is QSplatMake version %s.\n", QSPLATMAKE_VERSION);

    osg::ArgumentParser arguments(&argc, argv);

	// Parse command-line params
	if ((argc < 3) ||
	    !strncasecmp(argv[1], "-h", 2) ||
	    !strncasecmp(argv[1], "--h", 3))
		usage(argv[0]);
	
    bool flip = false;
    while (arguments.read("--flip")) { flip = true; }

    float threshold = 0;
    while (arguments.read("-m", threshold));

    //Get the last two arguments, they are the in and out filenames
    const char *infilename = argv[argc-2];
	const char *outfilename = argv[argc-1];


	// Read the .ply file
	int numleaves, numfaces;
	face *faces;
	bool havecolor;
	QTree_Node *leaves;
	std::string comments;


    if (!read_osg(infilename, numleaves, leaves, numfaces, faces, havecolor, flip, comments)) {
		fprintf(stderr, "Couldn't read input file %s\n", infilename);
		exit(1);
	}

	if (numleaves < 4) {
		fprintf(stderr, "Ummm...  That's an awfully small mesh you've got there...\n");
		exit(1);
	}
	if (numfaces < 4) {
		fprintf(stderr, "Ummm... I need a *mesh* as input.  That means triangles 'n stuff...\n");
		exit(1);
	}

	// Compute per-vertex normals
	find_normals(numleaves, leaves, numfaces, faces);

	// Merge nodes
	merge_nodes(numleaves, leaves, numfaces, faces, havecolor, threshold);

	// Compute initial splat sizes
	find_splat_sizes(numleaves, leaves, numfaces, faces);

	// Don't need face data any more
	delete [] faces;

	// Initialize the tree
	QTree qt(numleaves, leaves, havecolor);

	// Build the tree...
	qt.BuildTree();

	// ... and write it out
	qt.Write(outfilename, comments);
	return 0;
}

