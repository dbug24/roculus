/*
-----------------------------------------------------------------------------
Filename:    TutorialApplication.cpp
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
#include "TutorialApplication.h"
#include <math.h>

//-------------------------------------------------------------------------------------
TutorialApplication::TutorialApplication(void)
{
}
//-------------------------------------------------------------------------------------
TutorialApplication::~TutorialApplication(void)
{
}

//-------------------------------------------------------------------------------------
void TutorialApplication::createScene(void)
{
	using namespace Ogre;
	
	// Set camera resolution (X, Y, f) here
	Vector3 cam(640.0f, 480.0f, 574.0f);
	cam = cam/4; // Lower number of vertices (!), comment out for full resolution
	// For better results adapt the invResolution parameter in vertexColours.material (!)

	// Loading two dummy textures for the validity of the standard material
	// Actually the textures can be reused for the video stream
	TexturePtr pT_RGB = Ogre::TextureManager::getSingleton().createManual(
		"VideoRGBTexture", 				// name
		ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
		TEX_TYPE_2D,      // type
		640, 480,         		// width & height
		0,                		// number of mipmaps
		PF_BYTE_RGBA,     // pixel format
		TU_DYNAMIC_WRITE_ONLY_DISCARDABLE);  
		
	TexturePtr pT_Depth = Ogre::TextureManager::getSingleton().createManual(
		"VideoDepthTexture", 				// name
		ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
		TEX_TYPE_2D,      // type
		640, 480,         		// width & height
		0,                		// number of mipmaps
		PF_DEPTH,     // pixel format
		TU_DYNAMIC_WRITE_ONLY_DISCARDABLE); 
	
	TexturePtr pT_GlobalMap = Ogre::TextureManager::getSingleton().createManual(
		"GlobalMapTexture", 				// name
		ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
		TEX_TYPE_2D,      // type
		2048, 2048,         		// width & height
		0,                		// number of mipmaps
		PF_BYTE_RGBA,     // pixel format
		TU_STATIC);
			
	Ogre::Image imDefault;
	imDefault.load("KAMEN320x240.jpg","Popular");
	pT_RGB->loadImage(imDefault);
	pT_Depth->loadImage(imDefault);
	pT_GlobalMap->loadImage(imDefault);

	mPCRender= mSceneMgr->createManualObject();
	mPCRender->estimateVertexCount(cam.x * cam.y);
	mPCRender->estimateIndexCount(4 * cam.x * cam.y);
	mPCRender->begin("roculus3D/DynamicTextureMaterial", RenderOperation::OT_TRIANGLE_LIST);

	for (int w=0; w<cam.x; w++) {
		for (int h=0; h<cam.y; h++) {
			mPCRender->position(float(w - (cam.x-1.0f)/2.0f)/cam.z, float((cam.y-1.0f)/2.0f - h)/cam.z,0.0f);
			mPCRender->textureCoord(float(w)/(cam.x-1.0f),float(h)/(cam.y-1.0f));
			if (w>0 && h>0) {
				//mPCRender->quad(w*cam.y+h, (w-1)*cam.y+h, (w-1)*cam.y+h-1, w*cam.y+h-1);
				mPCRender->quad(w*cam.y+h, w*cam.y+h-1, (w-1)*cam.y+h-1, (w-1)*cam.y+h);
			}
		}
	}

	mPCRender->end();
	mPCRender->convertToMesh("DefGeometry");
	
	mPCRender= mSceneMgr->createManualObject();
	mPCRender->begin("roculus3D/BlankMaterial", RenderOperation::OT_LINE_LIST);

	mPCRender->position(0.0f,0.0f,0.0f);
	mPCRender->colour(Ogre::ColourValue(1.0f, 1.0f, 1.0f, 1.0f));
	mPCRender->position(1.0f,0.0f,0.0f);
	mPCRender->colour(Ogre::ColourValue(1.0f, 0.0f, 0.0f, 1.0f));
	mPCRender->position(0.0f,1.0f,0.0f);
	mPCRender->colour(Ogre::ColourValue(0.0f, 1.0f, 0.0f, 1.0f));
	mPCRender->position(0.0f,0.0f,1.0f);
	mPCRender->colour(Ogre::ColourValue(0.0f, 0.0f, 1.0f, 1.0f));

	mPCRender->index(0);
	mPCRender->index(1);
	mPCRender->index(0);
	mPCRender->index(2);
	mPCRender->index(0);
	mPCRender->index(3);
	
	mPCRender->end();
	
	mPCRender->convertToMesh("CoordSystem");
	
	vdVideo = new Video3D(mSceneMgr->createEntity("DefGeometry"), mSceneMgr->getRootSceneNode()->createChildSceneNode(), pT_Depth, pT_RGB);
	// PREallocate and manage memory
	snLib = new SnapshotLibrary(mSceneMgr, Ogre::String("DefGeometry"), Ogre::String("roculus3D/DynamicTextureMaterial"), 10);
	rsLib = new SnapshotLibrary(mSceneMgr, Ogre::String("DefGeometry"), Ogre::String("roculus3D/DynamicTextureMaterial"), 10);
}



#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
#define WIN32_LEAN_AND_MEAN
#include "windows.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
    INT WINAPI WinMain( HINSTANCE hInst, HINSTANCE, LPSTR strCmdLine, INT )
#else
    int main(int argc, char *argv[])
#endif
    {
        // Create application object
        TutorialApplication app;

        try {
	  app.go();
        } catch( Ogre::Exception& e ) {
#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
            MessageBox( NULL, e.getFullDescription().c_str(), "An exception has occured!", MB_OK | MB_ICONERROR | MB_TASKMODAL);
#else
            std::cerr << "An exception has occured: " <<
                e.getFullDescription().c_str() << std::endl;
#endif
        }

        return 0;
    }

#ifdef __cplusplus
}
#endif
