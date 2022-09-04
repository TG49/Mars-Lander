#pragma once

#define NDEBUG
#include <GL/glut.h>
#include <vector>


class textureObject {
public:
	textureObject() { };
	~textureObject() { };


	void setTextureObject(GLuint indexIn, int heightIn, int widthIn, int channelsIn);


	GLuint getIndex();
	int getHeight();
	int getwidth();
	int getChannels();

protected:
	GLuint index;
	int height;
	int width;
	int nrChannels;

};



class heightMapTexture : private textureObject {
public:

	void setTextureObject(GLuint indexIn, int heightIn, int widthIn, int channelsIn);
	//Overload the constructor
	heightMapTexture();
	GLfloat getHeightValue(double u, double v);

private:
	void buildHeightVector();
	GLfloat HeightValue(double u, double v);
	void deletePixelData(); 
	void loadPixelData();
	//Members
	std::vector<GLfloat> heights;
	bool pixelDataLoaded;
	int elementsPerLine;
	GLubyte* pixels;
};