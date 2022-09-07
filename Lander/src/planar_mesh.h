#include <vector>
#include <Eigen/Dense>
#include "TextureObject.h"
#include <string>

/// <summary>
/// Base class for mesh objects
/// </summary>
class mesh {
public:
	mesh(int LoDs = 1) : LevelsOfDetail(LoDs) {}; //default constructor
	virtual ~mesh() {};

	//Getters
	int getLevelOfDetail() { return LevelsOfDetail; };
	textureObject getTexture() { return texture; };
	std::vector<std::vector<int>> getIndices() { return indices; };
	std::vector<Eigen::Vector2d> getVertices() { return vertices; };
	std::vector<Eigen::Vector2d> getTextureCoordinates() { return textureCoordinates; }


protected:
	virtual void buildTexture() {};
	int LevelsOfDetail;
	std::vector<std::vector<int>> indices;
	std::vector<Eigen::Vector2d> vertices;
	std::vector<Eigen::Vector2d> textureCoordinates;

	textureObject texture;
};


/// <summary>
/// Class which handles square planar meshes
/// </summary>
class SquarePlaneMesh : public mesh {
public:
	SquarePlaneMesh(double size = 0, int meshResolution = 0, int textureRepeats=0) : size(size), meshResolution(meshResolution), textureRepeats(textureRepeats) {};
	~SquarePlaneMesh() {};

	//Getters
	double getSize();
	int getmeshResolution();
	int getTextureRepeats();

	void buildSquarePlane(int meshResolutionIn, int numberOfTextureRepeatsIn, double sizeIn);
	void drawMesh(double terrainAngle, double altitude, double transistionAltitude);


protected:
	double size;
	int meshResolution;
	int textureRepeats;
	std::string filenameOfTexture;
private:
	void loadVertices();
	void loadIndices();
	void loadTexture(std::string filename);

	friend class closeUpMeshes;

};

/// <summary>
/// Class which handles spherical meshes through the gluSphere function
/// </summary>
class sphericalMesh {
public:
	sphericalMesh() {};
	~sphericalMesh() {};

	void drawSphere(double radius, int slices, int stacks);
	void loadTexture(std::string filename);


protected:
	textureObject texture;
	std::string filenameOfTexture;

private:
	friend class closeUpMeshes;
	
};

/// <summary>
/// Class which handles all objects within the close up window 
/// </summary>
class closeUpMeshes {
public:
	closeUpMeshes() {};
	~closeUpMeshes() {};

	void buildcloseUpMeshes(int meshResolutionIn, int numberOfTextureRepeatsIn, 
		double sizeIn, std::string planetTexture, std::string surfaceTexture);

	sphericalMesh getPlanetObject() { return planet; };
	SquarePlaneMesh getPlaneMesh() { return surfaceMesh; };

protected:
	void loadTextures(std::string planetTextureName, std::string surfaceTextureName);
	sphericalMesh planet;
	SquarePlaneMesh surfaceMesh;
};