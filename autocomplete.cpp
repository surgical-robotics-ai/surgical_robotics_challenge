//==============================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2019-2021, AMBF
    (https://github.com/WPI-AIM/ambf)

    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.

    * Neither the name of authors nor the names of its contributors may
    be used to endorse or promote products derived from this software
    without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.

    \author    <amunawar@wpi.edu>
    \author    Adnan Munawar

    \author    <pkunjam1@jhu.edu>
    \author    Punit Kunjam
*/
//==============================================================================

#include "autocomplete.h"
#include <boost/program_options.hpp>
#include <ambf_server/RosComBase.h>
#include <std_msgs/Bool.h>

using namespace std;

//------------------------------------------------------------------------------
// DECLARED FUNCTIONS
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
afAutoCompletePlugin::afAutoCompletePlugin()
{
    cout << "Constructer!!" << endl;
}
int afAutoCompletePlugin::init(int argc, char **argv, const afWorldPtr a_afWorld)
{   
    cout << "Init Function!!" << endl;

    m_rosNode = afROSNode::getNode();
    m_commLossSub = m_rosNode->subscribe<std_msgs::Bool>("/communication_loss", 1, &afAutoCompletePlugin::communication_loss_cb, this);

    m_zeroColor = cColorb(0x00, 0x00, 0x00, 0x00);
    m_boneColor = cColorb(255, 249, 219, 255);
    m_storedColor = cColorb(0x00, 0x00, 0x00, 0x00);

    m_worldPtr = a_afWorld;

    // Get first camera
    m_mainCamera = m_worldPtr->getCamera("main_camera");
    if (m_mainCamera){
        cerr << "INFO! GOT CAMERA: " << m_mainCamera->getName() << endl;
    }
    else{
        cerr << "WARNING! COULD NOT FIND main_camera" << endl;
        m_mainCamera = m_worldPtr->getCameras()[0];
    }

    // Get camera
    m_stereoCameraL = m_worldPtr->getCamera("cameraL");
    m_stereoCameraR = m_worldPtr->getCamera("cameraR");


    // create a font
    cFontPtr font = NEW_CFONTCALIBRI40();

    m_drillControlModeText = new cLabel(font);
    m_drillControlModeText->setLocalPos(2 * m_mainCamera->m_width*0.4, 2 * m_mainCamera->m_height*0.85, 0);
    m_drillControlModeText->m_fontColor.setRed();
    m_drillControlModeText->setFontScale(0.8);
    m_drillControlModeText->setText("Communication Loss");
    m_mainCamera->getFrontLayer()->addChild(m_drillControlModeText);

    m_volumeSmoothingText = new cLabel(font);
    m_volumeSmoothingText->setLocalPos(2 * m_mainCamera->m_width*0.7, 2 * m_mainCamera->m_height*0.85, 0);
    m_volumeSmoothingText->m_fontColor.setBlue();
    m_volumeSmoothingText->setFontScale(0.8);
    m_volumeSmoothingText->setText("Blue: Prediction of the PSM");
    m_mainCamera->getFrontLayer()->addChild(m_volumeSmoothingText);

    cBackground *background = new cBackground();
    background->setCornerColors(cColorf(1.0f, 1.0f, 1.0f),
                                cColorf(1.0f, 1.0f, 1.0f),
                                cColorf(0.6f, 0.6f, 0.6f),
                                cColorf(0.6f, 0.6f, 0.6f));
    m_mainCamera->getBackLayer()->addChild(background);

    return 1;
}

void afAutoCompletePlugin::graphicsUpdate()
{
    m_drillControlModeText->setShowEnabled(m_comloss); 
}

void afAutoCompletePlugin::communication_loss_cb(const std_msgs::Bool::ConstPtr& comloss)
{
    m_comloss = comloss->data;
    cout << "subscribing ..." << endl;
}


void afAutoCompletePlugin::physicsUpdate(double dt)
{       
        
        // if(m_comloss == false)   
        // {
        //     m_drillControlModeText->setShowEnabled(m_comloss);  
        // }
        
}


// void afAutoCompletePlugin::keyboardUpdate(GLFWwindow *a_window, int a_key, int a_scancode, int a_action, int a_mods)
// {
    
// }

// void afAutoCompletePlugin::mouseBtnsUpdate(GLFWwindow *a_window, int a_button, int a_action, int a_modes)
// {
// }

// void afAutoCompletePlugin::mouseScrollUpdate(GLFWwindow *a_window, double x_pos, double y_pos)
// {
// }

// void afAutoCompletePlugin::reset()
// {
//     cerr << "INFO! PLUGIN RESET CALLED" << endl;
//     // resetDrill();
// }

// bool afAutoCompletePlugin::close()
// {
//     delete m_deviceHandler;

//     return true;
// }
