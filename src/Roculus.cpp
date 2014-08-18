/*
-----------------------------------------------------------------------------
Filename:    Roculus.cpp
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
#include <math.h>
#include <simpleXMLparser.h>
#include <simpleSummaryParser.h>
#include "Roculus.h"

typedef pcl::PointXYZRGB PointType;
typedef typename SimpleSummaryParser<PointType>::EntityStruct Entities;
//-------------------------------------------------------------------------------------
Roculus::Roculus(void)
{
}
//-------------------------------------------------------------------------------------
Roculus::~Roculus(void)
{
}

//-------------------------------------------------------------------------------------
void Roculus::createScene(void)
{	
	// Set camera resolution (X, Y, f) here
	Ogre::Vector3 cam(640.0f, 480.0f, 574.0f);
	cam = cam/2.0f; // Lower number of vertices (!), comment out for full resolution
	// For better results adapt the invResolution parameter in vertexColours.material (!)

	// Loading two dummy textures for the validity of the standard material
	// Actually the textures can be reused for the video stream
	Ogre::TexturePtr pT_RGB = Ogre::TextureManager::getSingleton().createManual(
		"VideoRGBTexture", 				// name
		Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
		Ogre::TEX_TYPE_2D,      // type
		640, 480,         		// width & height
		0,                		// number of mipmaps
		Ogre::PF_BYTE_RGB,     // pixel format
		Ogre::TU_DYNAMIC_WRITE_ONLY_DISCARDABLE);  //TU_DYNAMIC_WRITE_ONLY_DISCARDABLE
		
	Ogre::TexturePtr pT_Depth = Ogre::TextureManager::getSingleton().createManual(
		"VideoDepthTexture", 				// name
		Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
		Ogre::TEX_TYPE_2D,      // type
		640, 480,         		// width & height
		0,                		// number of mipmaps
		Ogre::PF_L16,     // pixel format
		Ogre::TU_DYNAMIC_WRITE_ONLY_DISCARDABLE); 
	
	Ogre::TexturePtr pT_GlobalMap = Ogre::TextureManager::getSingleton().createManual(
		"GlobalMapTexture", 				// name
		Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
		Ogre::TEX_TYPE_2D,      // type
		1024, 1024,         		// width & height
		0,                		// number of mipmaps
		Ogre::PF_BYTE_RGBA,     // pixel format
		Ogre::TU_STATIC);
			
	Ogre::Image imDefault;
	imDefault.load("KAMEN320x240.jpg","Popular");
	imDefault.resize(512,512);
	pT_RGB->loadImage(imDefault);
	pT_Depth->loadImage(imDefault);
	pT_GlobalMap->loadImage(imDefault);

	mPCRender= mSceneMgr->createManualObject();
	mPCRender->estimateVertexCount(cam.x * cam.y);
	mPCRender->estimateIndexCount(4 * cam.x * cam.y);
	mPCRender->begin("roculus3D/DynamicTextureMaterial", Ogre::RenderOperation::OT_TRIANGLE_LIST);

	float fake_z = 0.0f;
	for (int w=0; w<cam.x; w++) {
		for (int h=0; h<cam.y; h++) {
			if (fake_z > -4.0f) fake_z -= 0.05f;
			mPCRender->position(float(w - (cam.x-1.0f)/2.0f)/cam.z, float((cam.y-1.0f)/2.0f - h)/cam.z, fake_z);
			mPCRender->textureCoord(float(w)/(cam.x-1.0f),float(h)/(cam.y-1.0f));
			if (w>0 && h>0) {
				mPCRender->quad(w*cam.y+h, (w-1)*cam.y+h, (w-1)*cam.y+h-1, w*cam.y+h-1);
				mPCRender->quad(w*cam.y+h, w*cam.y+h-1, (w-1)*cam.y+h-1, (w-1)*cam.y+h);
			}
		}
	}

	mPCRender->end();
	mPCRender->convertToMesh("CamGeometry");
	
	mPCRender= mSceneMgr->createManualObject();
	mPCRender->begin("roculus3D/BlankMaterial", Ogre::RenderOperation::OT_LINE_LIST);

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
	
	mPCRender= mSceneMgr->createManualObject();
	mPCRender->begin("roculus3D/BlankMaterial", Ogre::RenderOperation::OT_LINE_STRIP);
	
	float const radius = 0.3;
    float const accuracy = 35; 
    unsigned point_index = 0;
    
    for(float theta = 0; theta <= 2 * Math::PI; theta += Math::PI / accuracy) {
        mPCRender->position(radius * cos(theta), 0.0f, radius * sin(theta));
        mPCRender->colour(Ogre::ColourValue(0.0f, 1.0f, 0.0f, 1.0f));
        mPCRender->index(point_index++);
    }
    mPCRender->index(0); // Rejoins the last point to the first.
 
	mPCRender->end();
	mPCRender->convertToMesh("CircleCursor");

	cursor = mSceneMgr->getRootSceneNode()->createChildSceneNode("CircleCursor");
	cursor->attachObject(mSceneMgr->createEntity("CircleCursor"));
	Ogre::Entity *wpMarker = mSceneMgr->createEntity("Cylinder.mesh");
	wpMarker->setMaterialName("roculus3D/WayPointMarkerTransparent");
	cursor->attachObject(wpMarker);
	
	
	
	vdVideo = new Video3D(mSceneMgr->createEntity("CamGeometry"), mSceneMgr->getRootSceneNode()->createChildSceneNode(), pT_Depth, pT_RGB);	
	
	/* Good for debugging: add some coordinate systems */
	//~ vdVideo->getTargetSceneNode()->attachObject(mSceneMgr->createEntity("CoordSystem"));
	//~ mSceneMgr->getRootSceneNode()->attachObject(mSceneMgr->createEntity("CoordSystem"));
	
	// PREallocate and manage memory to load/record snapshots
	snLib = new SnapshotLibrary(mSceneMgr, Ogre::String("CamGeometry"), Ogre::String("roculus3D/DynamicTextureMaterialSepia"), 10);
    rsLib = new SnapshotLibrary(mSceneMgr, Ogre::String("CamGeometry"), Ogre::String("roculus3D/DynamicTextureMaterialSepia"), 10);
    
    // Load the prerecorded environment    
	//~ loadRecordedScene();
    
}

void Roculus::loadRecordedScene() {
	
	SimpleSummaryParser<PointType> summary_parser("./map/index.xml");
    summary_parser.createSummaryXML("./map/");
    
	SimpleXMLParser<PointType> parser;
    SimpleXMLParser<PointType>::RoomData roomData;
    std::vector<Entities> allSweeps = summary_parser.getRooms();

	std::set<size_t> idc2process; // store all indecies that shall be processed and displayed {you can specify higher indices that don't actually exist!}
	idc2process.insert(0); 	idc2process.insert(2);	idc2process.insert(4);	idc2process.insert(6);	idc2process.insert(8);	idc2process.insert(10);
	idc2process.insert(12);	idc2process.insert(14);	idc2process.insert(16);	
	idc2process.insert(34); idc2process.insert(36);	idc2process.insert(38);	idc2process.insert(40);	idc2process.insert(42);	idc2process.insert(44);
	idc2process.insert(46); idc2process.insert(48);	idc2process.insert(50);	
	idc2process.insert(68);	idc2process.insert(70);	idc2process.insert(72);	idc2process.insert(74);	idc2process.insert(76);	idc2process.insert(78);
	idc2process.insert(80); idc2process.insert(82);	idc2process.insert(84);
	
	Ogre::Image oi_rgb, oi_depth;
	Ogre::Quaternion orientation;
	Ogre::Vector3 position;
	tfScalar yaw,pitch,roll;
	Matrix3 mRot;
	for (size_t i=0; i<allSweeps.size(); i++) {
		
		roomData = parser.loadRoomFromXML(allSweeps[i].roomXmlFile, &idc2process);
		for (size_t i=0; i<roomData.vIntermediateRoomClouds.size(); i++)
		{
			const cv::Mat& rgbImg = roomData.vIntermediateRGBImages[i];
			const cv::Mat& depthImg = roomData.vIntermediateDepthImages[i];
					
			// IMAGE FILTERING:
			cv::GaussianBlur(depthImg, depthImg, cv::Size(11,11), 0, 0);
					
			oi_rgb.loadDynamicImage(static_cast<uchar*>(rgbImg.data), rgbImg.cols, rgbImg.rows, 1, Ogre::PF_BYTE_RGB);
			oi_depth.loadDynamicImage(static_cast<uchar*>(depthImg.data), rgbImg.cols, rgbImg.rows, 1, Ogre::PF_L16);
			
			const tf::StampedTransform& tfCurrent = roomData.vIntermediateRoomCloudTransforms[i];
			position.x = -tfCurrent.getOrigin().y();
			position.y = tfCurrent.getOrigin().z();
			position.z = -tfCurrent.getOrigin().x();
			
			tfCurrent.getBasis().getEulerYPR(yaw,pitch,roll);
			mRot.FromEulerAnglesXYZ(-Radian(pitch),Radian(yaw),-Radian(roll));
			orientation.FromRotationMatrix(mRot);
			
			rsLib->placeInScene(oi_depth, oi_rgb, position, orientation);
		}
	}
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
        Roculus app;

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
