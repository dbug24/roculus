/*
-----------------------------------------------------------------------------
Filename:    Roculus.h
-----------------------------------------------------------------------------

This source file is part of the
   ___                 __    __ _ _    _ 
  /___\__ _ _ __ ___  / / /\ \ (_) | _(_)
 //  // _` | '__/ _ \ \ \/  \/ / | |/ / |
/ \_// (_| | | |  __/  \  /\  /| |   <| |
\___/ \__, |_|  \___|   \/  \/ |_|_|\_\_|
      |___/                              
      Tutorial Framework
      http://www.ogre3d.org/tikiwiki/
-----------------------------------------------------------------------------
*/
#ifndef __Roculus_h_
#define __Roculus_h_

#include "BaseApplication.h"

/** \brief class to take care of the scene and object creation.
 * This class fills the scene with objects and in most cases creates the geometries manually. It has a method to parse room recordings in folders in order to include them
 * into the scene. Though it is the nominal main class, the majority of the implementation is found in the BaseApplication class.
 * */
class Roculus : public BaseApplication
{
public:
    Roculus(void);
    /**< Default constructor */
    virtual ~Roculus(void);
    /**< Default destructor */

protected:
    virtual void createScene(void);
    /**< Does the main work to compile the implemented manual objects into geometries (meshes) and to configure the materials and set up the SnapshotLibraries. NOTE: Includes hardcoded camera parameters for the standard geometry. */
    virtual void loadRecordedScene();
    /**< Hardcodes the './map/' directory to be searched for room recordings and load them into a SnapshotLibrary. Includes smoothing for the depth images. */
};

#endif // #ifndef __Roculus_h_
