// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Francesco Rea
 * email:   francesco.rea@iit.it
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

/**
 * @file zynqGrabberModule.cpp
 * @brief Implementation of the zynqGrabberModule (see header file).
 */

#include <iCub/zynqGrabberModule.h>


bool zynqGrabberModule::configure(yarp::os::ResourceFinder &rf) {

    /* Process all parameters from both command-line and .ini file */
    std::cout << "Configuring the zynqGrabberModule" << std::endl;
    
    //printf("moduleName  %s \n", moduleName.c_str());
    std::string moduleName =
            rf.check("name", yarp::os::Value("zynqGrabber")).asString();
    setName(moduleName.c_str());


     //get the device name which will be used to read events
    std::string deviceName = rf.check("deviceName",
                          yarp::os::Value("/dev/spinn2neu")).asString();

    //get the maximum buffer size to use for device reading
    int maxBufferSize = rf.check("bufferSize", yarp::os::Value(65536)).asInt();

    //TODO: get all the bias settings




    // attach a port of the same name as the module (prefixed with a /)
    //to the module so that messages received from the port are redirected to
    //the respond method
    std::string handlerPortName =  "/" + moduleName;
    if (!handlerPort.open(handlerPortName.c_str())) {
        std::cout << "Unable to open RPC port @ " << handlerPortName << std::endl;
        return false;
    }
    attach(handlerPort);
            
    
    // class manageDevice
    devManager = new deviceManager(deviceName, maxBufferSize);
    if(!devManager->openDevice()) {
        std::cerr << "Could not open the device: " << deviceName << std::endl;
        return false;
    }

    
    //open rateThread device2yarp
    D2Y = new device2yarp();
    D2Y->attachDeviceManager(devManager);
    if(!D2Y->threadInit(moduleName)) {
        //could not start the thread
        return false;
    }
    D2Y->start();
    
    
    //open bufferedPort yarp2device
    Y2D.attachDeviceManager(devManager);
    if(!Y2D.open(moduleName))
    {
        std::cerr << " : Unable to open ports" << std::endl;
        return false;
    }
    
    return true;
}

bool zynqGrabberModule::interruptModule() {
    handlerPort.interrupt();
    Y2D.interrupt();
    // D2Y ???
    return true;
}

bool zynqGrabberModule::close() {
    
    closing = true;
    
    handlerPort.close();        // rpc of the RF module
    Y2D.close();
    D2Y->stop();                // bufferedport from yarp to device
    
    devManager->closeDevice();  // device

    return true;
}

/* Called periodically every getPeriod() seconds */
bool zynqGrabberModule::updateModule() {
    
    return !closing;
    
    return true;
}

double zynqGrabberModule::getPeriod() {
    /* module periodicity (seconds), called implicitly by myModule */
    return 1.0;
}

bool zynqGrabberModule::respond(const yarp::os::Bottle& command,
                                yarp::os::Bottle& reply) {
    bool ok = false;
    bool rec = false; // is the command recognized?
    std::string helpMessage =  std::string(getName().c_str()) +
    " commands are: \n" +
    "help \n" +
    "quit \n" +
    "set thr <n> ... set the threshold \n" +
    "(where <n> is an integer number) \n";

    reply.clear();

    if (command.get(0).asString()=="quit") {
        reply.addString("quitting");
        return false;
    }
    else if (command.get(0).asString()=="help") {
        std::cout << helpMessage;
        reply.addString("ok");
    }

    mutex.wait();
    switch (command.get(0).asVocab()) {
        case COMMAND_VOCAB_HELP:
            rec = true;
        {
            reply.addString("many");
            reply.addString("help");

            reply.addString("");

            ok = true;
        }
            break;
        case COMMAND_VOCAB_SUSPEND:
            rec = true;
        {
            D2Y->suspend();
            ok = true;
        }
            break;
        case COMMAND_VOCAB_RESUME:
            rec = true;
        {
            D2Y->resume();
            ok = true;
        }
            break;
        case COMMAND_VOCAB_SETBIAS:
            rec = true;
        {
            std::string biasName = command.get(1).asString();
            double biasValue = command.get(2).asDouble();
            int channel = command.get(3).asInt();

            // setBias function
            // biasManager.setBias(biasName, biasValue, channel);
            ok = true;
        }
            break;
        case COMMAND_VOCAB_PROG:
            rec= true;
        {
            int channel = command.get(1).asInt();

            // progBias function
            // biasManager.progBias(channel);
            ok = true;

        }
            break;
    }

    mutex.post();

    if (!rec)
        ok = RFModule::respond(command,reply);

    if (!ok) {
        reply.clear();
        reply.addVocab(COMMAND_VOCAB_FAILED);
    }
    else
        reply.addVocab(COMMAND_VOCAB_OK);

    return ok;

    return true;
}


