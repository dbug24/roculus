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

class Roculus : public BaseApplication
{
public:
    Roculus(void);
    virtual ~Roculus(void);

protected:
    virtual void createScene(void);
    virtual void loadRecordedScene();
};

#endif // #ifndef __Roculus_h_
