#include "SnapshotLibrary.h"
#include <boost/lexical_cast.hpp>
#include <stdio.h>

SnapshotLibrary::SnapshotLibrary(Ogre::SceneManager *mSceneMgr, const Ogre::String &EntityPrototype, const Ogre::String &MaterialPrototype, int initSize, bool saveOnShutdown) : save(saveOnShutdown) {
	currentSnapshot = 0;
	maxSnapshots = 0;
	this->mSceneMgr = mSceneMgr;
	this->EntityPrototype = EntityPrototype;
	this->MaterialPrototype = MaterialPrototype;
	this->mMasterSceneNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
	this->allocate(initSize);
}

SnapshotLibrary::~SnapshotLibrary() {
	//~ // eventually save this map:
	//~ 
	//~ if (save) {
		//~ saveMap();
	//~ }
	
	//cleanup is done by OGRE, hopefully. TODO: check with valgrind!
	for (int i=0; i<library.size(); i++) {
		if (library[i]) {
			delete library[i];
			library[i] = NULL;
		}
	}
}

void SnapshotLibrary::setSaveOnShutdown(bool val) {
	save = val;
}

void SnapshotLibrary::allocate(int nr) {
	int initStart = maxSnapshots;
	maxSnapshots += nr;
	library.reserve(maxSnapshots);
	for (int cnt=initStart; cnt < maxSnapshots; cnt++) {
		Ogre::Entity *pEntity = mSceneMgr->createEntity(EntityPrototype);
		
		Ogre::String sCnt = boost::lexical_cast<std::string>(cnt);
		Ogre::String newMaterialName = MaterialPrototype + sCnt;
		Ogre::MaterialPtr pMat = Ogre::MaterialManager::getSingleton().getByName(MaterialPrototype)->clone(newMaterialName);
		
		Ogre::String tex1Name = "RGBSnapshot" + sCnt;
		Ogre::String tex2Name = "DepthSnapshot" + sCnt;
		//Ogre::String tex3Name = "DepthMaskSnapshot" + sCnt;
		
		Ogre::TexturePtr pT_RGB = Ogre::TextureManager::getSingleton().createManual(
		tex1Name, 				// name
		Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
		Ogre::TEX_TYPE_2D,      // type
		512, 512,         		// width & height
		0,                		// number of mipmaps
		Ogre::PF_BYTE_RGB,     // pixel format
		Ogre::TU_STATIC);  
		
		Ogre::TexturePtr pT_Depth = Ogre::TextureManager::getSingleton().createManual(
		tex2Name, 				// name
		Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
		Ogre::TEX_TYPE_2D,      // type
		512, 512,         		// width & height
		0,                		// number of mipmaps
		Ogre::PF_L16,			// pixel format
		Ogre::TU_STATIC); 
		
		//~ Ogre::TexturePtr pT_DepthMask = Ogre::TextureManager::getSingleton().createManual(
		//~ tex3Name, 				// name
		//~ Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
		//~ Ogre::TEX_TYPE_2D,      // type
		//~ 640, 480,         		// width & height
		//~ 0,                		// number of mipmaps
		//~ Ogre::PF_BYTE_A,			// pixel format
		//~ Ogre::TU_STATIC); 
		
		//~ pMat->getTechnique(0)->getPass(0)->getTextureUnitState(2)->setTexture(pT_DepthMask);
		pMat->getTechnique(0)->getPass(0)->getTextureUnitState(0)->setTexture(pT_RGB);
		pMat->getTechnique(0)->getPass(0)->getTextureUnitState(1)->setTexture(pT_Depth);
		
		pEntity->setMaterial(pMat); // override all submaterials to pMat
				
		Ogre::SceneNode* pSceneNode = mMasterSceneNode->createChildSceneNode();
		//pSceneNode->attachObject(mSceneMgr->createEntity("CoordSystem")); //good for debugging (!)
		
		Snapshot *pSnap = new Snapshot(pEntity, pSceneNode, pT_Depth, pT_RGB);
		library[cnt] = pSnap;
		pSnap = NULL;
	}
	
	std::cout << " <<< (PRE)ALLOCATION successful >>> " << std::endl;
}

bool SnapshotLibrary::placeInScene(const Ogre::Image &depth, const Ogre::Image &rgb, const Ogre::Vector3 &pos, const Ogre::Quaternion &ori) {

	if (currentSnapshot < maxSnapshots) {
		return library[currentSnapshot++]->placeInScene(depth, rgb, pos, ori);
	} else {
		SnapshotLibrary::allocate(10);
		return library[currentSnapshot++]->placeInScene(depth, rgb, pos, ori);
	}
}

void SnapshotLibrary::flipVisibility() {
	mMasterSceneNode->flipVisibility(); // do not cascade to subrenderables
}

//~ void SnapshotLibrary::saveMap() {
	//~ Ogre::Image depth;
    //~ Ogre::Image rgb;
    //~ for (int i=0;i<currentSnapshot;i++) {
			//~ std::cout<<"Saving depth and rgb textures"<<std::endl;
            //~ char filename_rgb[50];
            //~ std::sprintf(filename_rgb,"Znap_rgb%d.jpg",i);
            //~ char filename_depth[50];
            //~ std::sprintf(filename_depth,"Znap_depth%d.png",i);
            //~ std::ofstream myfile;
            //~ char filename_pos[50];
            //~ std::sprintf(filename_pos,"Znap_pos%d.txt",i);
            //~ 
            //~ library[i]->getAssignedDepthTexture()->convertToImage(depth);
            //~ library[i]->getAssignedRGBTexture()->convertToImage(rgb);
            //~ depth.save(filename_depth);
            //~ rgb.save(filename_rgb);
            //~ 
            //~ myfile.open (filename_pos);
            //~ myfile<<Ogre::StringConverter::toString(library[i]->getTargetSceneNode()->getPosition())<<"\n"<<Ogre::StringConverter::toString(library[i]->getTargetSceneNode()->getOrientation());
            //~ myfile.close();
	//~ }
//~ }
