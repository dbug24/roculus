#include "Snapshot.h"

Snapshot::Snapshot(Ogre::Entity *pSnapshot, Ogre::SceneNode *pSceneNode, const Ogre::TexturePtr &depthTexture, const Ogre::TexturePtr &rgbTexture) {
	this->snapshot = pSnapshot;
	this->targetSceneNode = pSceneNode;
	this->depthTexture = depthTexture;
	this->rgbTexture = rgbTexture;
	this->attached = false;
	//~ this->depthMask = depthMask;
}

Snapshot::~Snapshot() {
}

bool Snapshot::placeInScene(const Ogre::Image &depth, const Ogre::Image &rgb, const Ogre::Vector3 &pos, const Ogre::Quaternion &orientation) {
	depthTexture->unload();
	depthTexture->loadImage(depth);
	rgbTexture->unload();
	rgbTexture->loadImage(rgb);
	//~ depthMask->unload();
	//~ depthMask->loadImage(depthMask)
	targetSceneNode->setPosition(pos);
	targetSceneNode->setOrientation(orientation);
	targetSceneNode->yaw(Ogre::Degree(-90));
	targetSceneNode->roll(Ogre::Degree(180));
	if (!attached) {
		targetSceneNode->attachObject(snapshot);
		attached = true;
	}
	return true;
}

Ogre::SceneNode* Snapshot::getTargetSceneNode() {
	return this->targetSceneNode;
}

Ogre::TexturePtr Snapshot::getAssignedDepthTexture() {
	return this->depthTexture;
}

//~ Ogre::TexturePtr Snapshot::getAssignedDepthMask() {
	//~ return this->depthMask;
//~ }

Ogre::TexturePtr Snapshot::getAssignedRGBTexture() {
	return this->rgbTexture;
}

void Snapshot::setTargetSceneNode(Ogre::SceneNode* node) {
	this->targetSceneNode = node;
}

void Snapshot::assignDepthTexture(const Ogre::TexturePtr &tex) {
	this->depthTexture = tex;
}

//~ void Snapshot::assignDepthMask(const Ogre::TexturePtr &tex) {
	//~ this->depthMask;
//~ }

void Snapshot::assignRGBTexture(const Ogre::TexturePtr &tex) {
	this->rgbTexture = tex;
}

