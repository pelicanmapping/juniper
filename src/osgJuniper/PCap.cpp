/* -*-c++-*- */
/* osgJuniper - Large Dataset Visualization Toolkit for OpenSceneGraph
 * Copyright 2010-2011 Pelican Ventures, Inc.
 * http://wush.net/trac/juniper
 *
 * osgEarth is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 */

#include <osgJuniper/PCap>
#include <osg/Notify>

using namespace osgJuniper;

PcapReader::PcapReader(const std::string& filename):
_filename(filename)
{
    _in.open( filename.c_str(), std::ios::in | std::ios::binary );

    //Go ahead and read the header info
    if (_in.read(reinterpret_cast<char*>(&_header), sizeof(_header)))
    {
        if (_header.magic_number == MAGIC_NUMBER_SWAP)
        {
            osg::notify(osg::DEBUG_INFO) << "MAGIC_NUMBER_SWAP" << std::endl;
        }
        else if (_header.magic_number == MAGIC_NUMBER_NOSWAP)
        {
            osg::notify(osg::DEBUG_INFO) << "MAGIC_NUMBER_NOSWAP" << std::endl;
        }
        else
        {
            osg::notify(osg::DEBUG_INFO) << "Magic number unknown " << std::endl;
        }

        osg::notify(osg::DEBUG_INFO) << "Got header" << std::endl
            << "magic_number=" <<  std::hex << _header.magic_number << std::dec << std::endl
            << "version_major=" << _header.version_major << std::endl
            << "version_minor=" << _header.version_minor << std::endl
            << "thiszone=" << _header.thiszone << std::endl
            << "sigfigs=" << _header.sigfigs << std::endl
            << "snaplen=" << _header.snaplen << std::endl
            << "network=" << _header.network << std::endl;

        osg::notify(osg::DEBUG_INFO) << "Position= " << _in.tellg() << std::endl;

        //Compute the filesize
        _in.seekg(0, std::ios_base::end);
        _filesize = _in.tellg();
        reset();

        osg::notify(osg::DEBUG_INFO) << "Read filesize " << getFileSize() << std::endl;
        osg::notify(osg::DEBUG_INFO) << "Position= " << _in.tellg() << std::endl;
    }
    else
    {
        osg::notify(osg::WARN) << "PcapReader:: Couldn't read header for " << _filename.c_str() <<  std::endl;
    }
}

unsigned int
PcapReader::getFileSize() const
{
    return _filesize;
}

void
PcapReader::reset()
{
    _in.clear();
    _in.seekg(sizeof(_header), std::ios_base::beg);
}

bool
PcapReader::hasMore() const
{
    return _in.is_open() && !_in.eof();
}

bool
PcapReader::nextPacket(PACKET& packet)
{
    if (hasMore())
    {
        //Read the header for the packet
        pcaprec_hdr_s packet_header;
        packet_header.ts_sec = 10;
        if (_in.read(reinterpret_cast<char*>(&packet_header), sizeof(packet_header)))
        {
            /*osg::notify(osg::NOTICE) << "Read packet header " << std::endl
                       << "ts_sec=" << packet_header.ts_sec << std::endl
                       << "ts_usec=" << packet_header.ts_usec << std::endl
                       << "incl_len=" << packet_header.incl_len << std::endl
                       << "orig_len=" << packet_header.orig_len << std::endl;
                       */

            //Now, read the actual data from the file
            packet.nBytes = packet_header.incl_len;
            if (_in.read(packet.Bytes, packet.nBytes))
            {
                return true;
            }
            else
            {
                osg::notify(osg::NOTICE) << "Error reading packet data" << std::endl;
            }
        }
        else
        {
            osg::notify(osg::NOTICE) << "Error reading packet header" << std::endl;
        }
    }
    return false;
}