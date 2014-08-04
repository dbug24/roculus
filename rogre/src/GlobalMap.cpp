#include "GlobalMap.h"
#include <OgreStringConverter.h>

using namespace Ogre;

GlobalMap::GlobalMap(SceneManager* mgr) {
	this->mSceneMgr = mgr;
	this->pSceneNode = mSceneMgr->getRootSceneNode()->createChildSceneNode("GlobalMap");
}
GlobalMap::~GlobalMap() {
	// nothing to do
}

void GlobalMap::insertWHR(uint32 width, uint32 height, Real resolution) {
	this->width = Real(width);
	this->height = Real(height);
	this->resolution = resolution;
}

void GlobalMap::setOrigin(Vector3 origin) {
	this->origin = origin*resolution;
	Ogre::LogManager::getSingletonPtr()->logMessage("Map origin at: " + StringConverter::toString(this->origin));
}

void GlobalMap::includeMap(const Image& map) {
	TextureManager::getSingleton().getByName("GlobalMapTexture")->unload();
	TextureManager::getSingleton().getByName("GlobalMapTexture")->loadImage(map);
	mMapObj = mSceneMgr->createManualObject("GlobalMap");
	mMapObj->estimateVertexCount(4);
	mMapObj->estimateIndexCount(6);
	mMapObj->begin("roculus3D/GlobalMapMaterial", RenderOperation::OT_TRIANGLE_LIST);
	
	mMapObj->position(Vector3(-height/2.0f, -0.05f, width/2.0f)*resolution);
	mMapObj->textureCoord(0.0f,1.0f);
	mMapObj->position(Vector3(height/2.0f, -0.05f, width/2.0f)*resolution);
	mMapObj->textureCoord(0.0f,0.0f);
	mMapObj->position(Vector3(height/2.0f, -0.05f, -width/2.0f)*resolution);
	mMapObj->textureCoord(1.0f,0.0f);
	mMapObj->position(Vector3(-height/2.0f, -0.05f, -width/2.0f)*resolution);
	mMapObj->textureCoord(1.0f,1.0f);
	
	
	mMapObj->quad(0,1,2,3);
	mMapObj->end();
	
	pSceneNode->attachObject(mMapObj);
	pSceneNode->setPosition(2.0f*origin); // Why 2.0f times? - I have no idea...
}

Real  GlobalMap::getWidth() {
	return width;
}

Real GlobalMap::getHeight() {
	return height;
}

Real GlobalMap::getResolution() {
	return resolution;
}

Vector3 GlobalMap::getOrigin() {
	return origin;
}

void GlobalMap::flipVisibility() {
	pSceneNode->flipVisibility(true);
}
