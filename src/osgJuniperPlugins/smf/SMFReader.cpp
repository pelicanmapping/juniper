/* -*-c++-*- */
/* osgJuniper - Large Dataset Visualization Toolkit for OpenSceneGraph
* Copyright 2010-2017 Pelican Mapping
* Pelican Mapping CONFIDENTIAL
* Copyright (c) 2010-2017 [Pelican Mapping], All Rights Reserved.
*
* NOTICE:  All information contained herein is, and remains the property of Pelican Mapping. The intellectual and technical concepts contained
* herein are proprietary to Pelican Mapping and may be covered by U.S. and Foreign Patents, patents in process, and are protected by trade secret or copyright law.
* Dissemination of this information or reproduction of this material is strictly forbidden unless prior written permission is obtained
* from Pelican Mapping.  Access to the source code contained herein is hereby forbidden to anyone except current Pelican Mapping employees, managers or contractors who have executed
* Confidentiality and Non-disclosure agreements explicitly covering such access.
*
* The copyright notice above does not evidence any actual or intended publication or disclosure  of  this source code, which includes
* information that is confidential and/or proprietary, and is a trade secret, of Pelican Mapping.   ANY REPRODUCTION, MODIFICATION, DISTRIBUTION, PUBLIC  PERFORMANCE,
* OR PUBLIC DISPLAY OF OR THROUGH USE  OF THIS  SOURCE CODE  WITHOUT  THE EXPRESS WRITTEN CONSENT OF PELICAN MAPPING IS STRICTLY PROHIBITED, AND IN VIOLATION OF APPLICABLE
* LAWS AND INTERNATIONAL TREATIES.  THE RECEIPT OR POSSESSION OF  THIS SOURCE CODE AND/OR RELATED INFORMATION DOES NOT CONVEY OR IMPLY ANY RIGHTS
* TO REPRODUCE, DISCLOSE OR DISTRIBUTE ITS CONTENTS, OR TO MANUFACTURE, USE, OR SELL ANYTHING THAT IT  MAY DESCRIBE, IN WHOLE OR IN PART.
*/
#include <osg/Geometry>
#include <osg/Geode>
#include <osg/Point>
#include <osg/LOD>
#include <osg/ShapeDrawable>
#include <osgDB/Registry>
#include <osgDB/FileNameUtils>
#include <osgDB/ReaderWriter>
#include <osgUtil/SmoothingVisitor>
#include <fstream>

/**
* Reader / Writer for the SMF format described at http://people.sc.fsu.edu/~jburkardt/data/smf/smf.html
* A few mesh simplification  programs like QSlim and progressive-messhes (http://code.google.com/p/progressive-meshes/) work with this format
*/


std::string trim(const std::string& s)
{
  if(s.length() == 0)
    return s;
  int b = s.find_first_not_of(" \t");
  int e = s.find_last_not_of(" \t");
  if(b == -1) // No non-spaces
    return "";
  return std::string(s, b, e - b + 1);
}


class SMFLoader
{
public:

    SMFLoader()
    {
    }

    void load(std::istream& in)
    {
        _vertices = new osg::Vec3Array();
        _normals = new osg::Vec3Array();

        const int BUF_SIZE = 1024;
        char buffer[BUF_SIZE];
        while (!in.eof())
        {
            in.getline(buffer, BUF_SIZE);            
            std::string line(buffer);
            line = trim(line);

            if (strncmp(line.c_str(),"v ", 2) == 0)
            {
                float x, y, z;
                unsigned int numRead = sscanf(line.c_str() + 2, "%f %f %f", &x, &y, &z);
                if (numRead == 3)
                {
                    _vertices->push_back( osg::Vec3(x, y, z) );
                }
            }
            if (strncmp(line.c_str(),"n ", 2) == 0)
            {
                float x, y, z;
                unsigned int numRead = sscanf(line.c_str() + 2, "%f %f %f", &x, &y, &z);
                if (numRead == 3)
                {
                    _normals->push_back( osg::Vec3(x, y, z) );
                }
            }
            else if (strncmp(line.c_str(),"f ", 2) == 0)
            {
                unsigned int v0, v1, v2;
                unsigned int numRead = sscanf(line.c_str() + 2, "%d %d %d", &v0, &v1, &v2);
                if (numRead == 3)
                {
                    _vertexIndices.push_back( v0 -1 );
                    _vertexIndices.push_back( v1 -1 );
                    _vertexIndices.push_back( v2 -1 );
                }
            }
        }
    }

    osg::ref_ptr< osg::Vec3Array > _vertices;
    std::vector< int > _vertexIndices;
    osg::ref_ptr< osg::Vec3Array > _normals;
};


/** writes all primitives of a primitive-set out to a stream, decomposes quads to triangles, line-strips to lines etc */
class SMFPrimitiveIndexWriter : public osg::PrimitiveIndexFunctor {
    
    public:
        SMFPrimitiveIndexWriter(std::ostream& fout,osg::Geometry* geo,const osg::Matrix& m = osg::Matrix::identity()) : 
            osg::PrimitiveIndexFunctor(), 
            _fout(fout),            
            _geo(geo),
            _m(m)
        {
            
        }
        
        virtual void setVertexArray(unsigned int,const osg::Vec2*) {}

        virtual void setVertexArray(unsigned int ,const osg::Vec3* ) {}

        virtual void setVertexArray(unsigned int,const osg::Vec4* ) {}
        
        virtual void setVertexArray(unsigned int,const osg::Vec2d*) {}

        virtual void setVertexArray(unsigned int ,const osg::Vec3d* ) {}

        virtual void setVertexArray(unsigned int,const osg::Vec4d* ) {}      
        
        // operator for facets - need to distinguish from triangles
        void writeFace(unsigned int i1, unsigned int i2, unsigned int i3)
         {                     
             _fout << "f " << i1 + 1 << " " << i2 + 1 << " " << i3 + 1 << std::endl;
        }

        // operator for triangles 
        void writeTriangle(unsigned int i1, unsigned int i2, unsigned int i3)
        {                    
            _fout << "f " << i1 + 1 << " " << i2 + 1 << " " << i3 + 1 << std::endl;
        }

        // operator for lines
        void writeLine(unsigned int i1, unsigned int i2) 
        {                     
        }
        
        // operator for points
        void writePoint(unsigned int i1) 
        {                       
        }

        virtual void begin(GLenum mode)
        {
            _modeCache = mode;
            _indexCache.clear();
        }

        virtual void vertex(unsigned int vert)
        {
            _indexCache.push_back(vert);
        }

        virtual void end()
        {
            if (!_indexCache.empty())
            {
                drawElements(_modeCache,_indexCache.size(),&_indexCache.front());
            }
        }

        virtual void drawArrays(GLenum mode,GLint first,GLsizei count);                    
        
        virtual void drawElements(GLenum mode,GLsizei count,const GLubyte* indices)
        {
            drawElementsImplementation<GLubyte>(mode, count, indices);
        }
        virtual void drawElements(GLenum mode,GLsizei count,const GLushort* indices)
        {
            drawElementsImplementation<GLushort>(mode, count, indices);
        }    

        virtual void drawElements(GLenum mode,GLsizei count,const GLuint* indices)
        {
            drawElementsImplementation<GLuint>(mode, count, indices);
        }    

    protected:
        
        template<typename T>void drawElementsImplementation(GLenum mode, GLsizei count, const T* indices) 
        {
            if (indices==0 || count==0) return;

            typedef const T* IndexPointer;                    

            switch(mode)
            {
                case(GL_TRIANGLES):
                {
                    IndexPointer ilast = &indices[count];
                    for(IndexPointer  iptr=indices;iptr<ilast;iptr+=3)
                        writeTriangle(*iptr,*(iptr+1),*(iptr+2));
    
                    break;
                }
                case(GL_TRIANGLE_STRIP):
                {
                    IndexPointer iptr = indices;
                    for(GLsizei i=2;i<count;++i,++iptr)
                    {
                        if ((i%2)) writeTriangle(*(iptr),*(iptr+2),*(iptr+1));
                        else       writeTriangle(*(iptr),*(iptr+1),*(iptr+2));
                    }
                    break;
                }
                case(GL_QUADS):
                {
                    IndexPointer iptr = indices;
                    for(GLsizei i=3;i<count;i+=4,iptr+=4)
                    {
                        writeTriangle(*(iptr),*(iptr+1),*(iptr+2));
                        writeTriangle(*(iptr),*(iptr+2),*(iptr+3));
                    }
                    break;
                }
                case(GL_QUAD_STRIP):
                {
                    IndexPointer iptr = indices;
                    for(GLsizei i=3;i<count;i+=2,iptr+=2)
                    {
                        writeTriangle(*(iptr),*(iptr+1),*(iptr+2));
                        writeTriangle(*(iptr+1),*(iptr+3),*(iptr+2));
                    }
                    break;
                }
                case(GL_POLYGON): // treat polygons as GL_TRIANGLE_FAN
                case(GL_TRIANGLE_FAN):
                {
                    IndexPointer iptr = indices;
                    unsigned int first = *iptr;
                    ++iptr;
                    for(GLsizei i=2;i<count;++i,++iptr)
                    {
                        writeTriangle(first,*(iptr),*(iptr+1));
                    }
                    break;
                }
                case(GL_POINTS):
                {
                    IndexPointer ilast = &indices[count];
                    for(IndexPointer  iptr=indices;iptr<ilast;++iptr)
                    
                    {
                        writePoint(*iptr);
                    }
                    break;
                }

                case(GL_LINES):
                {
                    IndexPointer ilast = &indices[count];
                    for(IndexPointer  iptr=indices;iptr<ilast;iptr+=2)
                    {
                        writeLine(*iptr, *(iptr+1));
                    }
                    break;
                }
                case(GL_LINE_STRIP):
                {
                    
                    IndexPointer ilast = &indices[count];
                    for(IndexPointer  iptr=indices+1;iptr<ilast;iptr+=2)

                    {
                        writeLine(*(iptr-1), *iptr);
                    }
                    break;
                }
                case(GL_LINE_LOOP):
                {
                    IndexPointer ilast = &indices[count];
                    for(IndexPointer  iptr=indices+1;iptr<ilast;iptr+=2)
                    {
                        writeLine(*(iptr-1), *iptr);
                    }
                    writeLine(*ilast, *indices);
                    break;
                }

                default:
                    // uhm should never come to this point :)
                    break;
            }
        }    
    
    private:

        SMFPrimitiveIndexWriter& operator = (const SMFPrimitiveIndexWriter&) { return *this; }

        std::ostream&         _fout;
        GLenum               _modeCache;
        std::vector<GLuint>  _indexCache;        
        osg::Geometry*         _geo;        
       
        osg::Matrix        _m;
};


void SMFPrimitiveIndexWriter::drawArrays(GLenum mode,GLint first,GLsizei count)
{
    switch(mode)
    {
        case(GL_TRIANGLES):
        {
            unsigned int pos=first;
            for(GLsizei i=2;i<count;i+=3,pos+=3)
            {
                writeTriangle(pos,pos+1,pos+2);
            }
            break;
        }
        case(GL_TRIANGLE_STRIP):
         {
            unsigned int pos=first;
            for(GLsizei i=2;i<count;++i,++pos)
            {
                if ((i%2)) writeTriangle(pos,pos+2,pos+1);
                else       writeTriangle(pos,pos+1,pos+2);
            }
            break;
        }
        case(GL_QUADS):
        {
            unsigned int pos=first;
            for(GLsizei i=3;i<count;i+=4,pos+=4)
            {
                writeTriangle(pos,pos+1,pos+2);
                writeTriangle(pos,pos+2,pos+3);
            }
            break;
        }
        case(GL_QUAD_STRIP):
        {
            unsigned int pos=first;
            for(GLsizei i=3;i<count;i+=2,pos+=2)
            {
                writeTriangle(pos,pos+1,pos+2);
                writeTriangle(pos+1,pos+3,pos+2);
            }
            break;
        }
        case(GL_POLYGON): // treat polygons as GL_TRIANGLE_FAN
        case(GL_TRIANGLE_FAN):
        {
            unsigned int pos=first+1;
            for(GLsizei i=2;i<count;++i,++pos)
            {
                writeTriangle(first,pos,pos+1);
            }
            break;
        }
        case(GL_POINTS):
        {
            
            for(GLsizei i=0;i<count;++i)
            {
                writePoint(i);
            }
            break;
        }

        case(GL_LINES):
        {
            for(GLsizei i=0;i<count;i+=2)
            {
                writeLine(i, i+1);
            }
            break;
        }
        case(GL_LINE_STRIP):
        {
            for(GLsizei i=1;i<count;++i)
            {
                writeLine(i-1, i);
            }
            break;
        }
        case(GL_LINE_LOOP):
        {
            for(GLsizei i=1;i<count;++i)
            {
                writeLine(i-1, i);
            }
            writeLine(count-1, 0);
            break;
        }
        default:
            osg::notify(osg::WARN) << "SMFPrimitiveIndexWriter :: can't handle mode " << mode << std::endl; 
            break;
    }
}


class WriteSMFVisitor : public osg::NodeVisitor
{
public:
    WriteSMFVisitor(std::ostream& fout):
      osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN),
      _fout(fout)
    {
    }

      void apply( osg::Geode &geode )
      {
          for (unsigned int i = 0; i < geode.getNumDrawables(); ++i)
          {
              osg::Geometry* geom = geode.getDrawable(i)->asGeometry();
              if (geom)
              {
                  processGeometry(geom);
              }
          }
      }

      void processGeometry(osg::Geometry* geometry)
      {          
          //Write out all the vertices
          osg::Vec3Array* vertices = dynamic_cast< osg::Vec3Array * > (geometry->getVertexArray());
          if (vertices)
          {
              for (osg::Vec3Array::iterator itr = vertices->begin(); itr != vertices->end(); ++itr)
              {
                  osg::Vec3 &v = *itr;
                  _fout << "v " << v.x() << " " << v.y() << " " << v.z() << std::endl;
              }

              //Write out all the primitive sets
              for (unsigned int i = 0; i < geometry->getNumPrimitiveSets(); ++i)
              {
                  osg::PrimitiveSet* ps = geometry->getPrimitiveSet( i );                

                  SMFPrimitiveIndexWriter pif(_fout, geometry);
                  ps->accept(pif);
              }
          }
      }

    std::ostream& _fout;

};


class SMFReader : public osgDB::ReaderWriter
{
public:
    SMFReader()
    {
        supportsExtension( "smf", className() );
    }

    virtual const char* className()
    {
        return "Juniper SMF Reader";
    }    

    virtual ReadResult readNode( const std::string& location, const osgDB::ReaderWriter::Options* options ) const
    {
        if ( !acceptsExtension( osgDB::getLowerCaseFileExtension( location ) ) )
            return ReadResult::FILE_NOT_HANDLED;

        std::ifstream in( location.c_str() );
        if ( !in.is_open() )
            return ReadResult::FILE_NOT_FOUND;

        SMFLoader loader;
        loader.load( in );

        if (loader._vertices->size() > 0)
        {
            osg::Geode *geode = new osg::Geode;
            osg::Geometry* geometry = new osg::Geometry;
            geometry->setVertexArray( loader._vertices.get() );
            
            geometry->setUseDisplayList( false );
            geometry->setUseVertexBufferObjects( true );

            bool smooth = true;
            if (loader._normals->size() > 0)
            {
                geometry->setNormalArray( loader._normals.get() );
                geometry->setNormalBinding( osg::Geometry::BIND_PER_VERTEX);
                smooth = false;
            }


            if (loader._vertexIndices.size() > 0)
            {
                osg::DrawElementsUInt* drawElements = new osg::DrawElementsUInt(GL_TRIANGLES);
                for (std::vector< int >::iterator itr = loader._vertexIndices.begin(); itr != loader._vertexIndices.end(); ++itr)
                {
                    drawElements->push_back( *itr );
                }
                geometry->addPrimitiveSet( drawElements );
            }
            else
            {
                //Just draw points
                geometry->addPrimitiveSet( new osg::DrawArrays( GL_POINTS, 0, loader._vertices->size()) );
                smooth = false;
            }

            geode->addDrawable( geometry );

            if (smooth)
            {
                osgUtil::SmoothingVisitor v;
                geode->accept( v);
            }

            return geode;
        }

        return ReadResult::ERROR_IN_READING_FILE;
    }

    virtual WriteResult writeNode(const osg::Node& node,const std::string& fileName,const Options* options =NULL) const 
    { 
        if (!acceptsExtension(osgDB::getFileExtension(fileName)))
            return WriteResult(WriteResult::FILE_NOT_HANDLED); 
        
        osgDB::ofstream f(fileName.c_str());

        WriteSMFVisitor nv(f);
        
        // we must cast away constness
        (const_cast<osg::Node*>(&node))->accept(nv);

         
        return WriteResult(WriteResult::FILE_SAVED); 
    }


};

REGISTER_OSGPLUGIN(smf, SMFReader)

