// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * \defgroup 
 * @ingroup emorph_lib
 *
 * Interface for the event-based camera of the emorph project.
 *
 * Author: Francesco Rea
 * Copyright (C) 2010 RobotCub Consortium
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
*/


#ifndef __eventBottle__
#define __eventBottle__


#include <yarp/os/all.h>
#include <cstring>


/**
 * portable class for the bottle of events
 */
class eventBottle : public yarp::os::Portable {
public:
    eventBottle();
    eventBottle(char*, int);
    ~eventBottle();

    void operator =(eventBottle&);
    eventBottle(const eventBottle&);

    virtual bool write(yarp::os::ConnectionWriter&);
    virtual bool read (yarp::os::ConnectionReader&);

    void set_data(char*, int);

    yarp::os::Bottle* get_packet()  { return packet; };
    int get_sizeOfPacket(){ return size_of_the_packet; };

private:
    yarp::os::Bottle* packet;
    char* packetPointer;
    int size_of_the_packet;
};


#endif

//----- end-of-file --- ( next line intentionally left blank ) ------------------
