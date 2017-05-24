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