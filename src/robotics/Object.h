/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Ana Huaman <ahuaman3@gatech.edu>
 * Date: 2012-08-07
 *
 * Geoorgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 *
 *   @brief Class that describe Object, a subclass of Skeleton
 */
#ifndef __DART_ROBOTICS_OBJECT_H__
#define __DART_ROBOTICS_OBJECT_H__

#include <vector>
#include <string>
#include <Eigen/Geometry>
#include "dynamics/SkeletonDynamics.h"

extern "C" { struct aiScene; }

namespace robotics {
  #define MAX_Object_NAME 128

  /**
   * @class Object
   */
  class Object : public dynamics::SkeletonDynamics {
  public:
    Object();
    virtual ~Object();

    void addDefaultRootNode();

    inline void setMovable( bool _m ) { mMovable = _m; }

    void setPositionX( double _pos );
    void getPositionX( double &_pos );
    
    void setPositionY( double _pos );
    void getPositionY( double &_pos );
    
    void setPositionZ( double _pos );
    void getPositionZ( double &_pos );
    
    void setPositionXYZ( double _x, double _y, double _z );
    void getPositionXYZ( double &_x, double &_y, double &_z );       
    
    void setRotationRPY( double _roll, double _pitch, double _yaw );
    void getRotationRPY( double &_roll, double &_pitch, double &_yaw );

    void update();    

  private:
    bool mMovable;
  };

}

#endif /** __DART_ROBOTICS_OBJECT_H__ */

