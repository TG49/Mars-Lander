#include <vector>
#include <Eigen/Dense>
#include "TextureObject.h"
#include <string>


class mesh {
public:
	mesh() {};
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

class SquarePlaneMesh : public mesh {
public:
	SquarePlaneMesh() {};
	~SquarePlaneMesh() {};

	//Getters
	int getSize();
	int getmeshResolution();
	int getTextureRepeats();

	void buildSquarePlane(int meshResolutionIn, int numberOfTextureRepeatsIn, double sizeIn, std::string filename);
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

};

class sphericalMesh {
public:
	sphericalMesh() {};
	~sphericalMesh() {};

	void drawSphere(double radius, int slices, int stacks);
	void buildSphereObject(std::string filename);


protected:
	textureObject texture;
	std::string filenameOfTexture;

private:
	void loadTexture(std::string filename);
	
};