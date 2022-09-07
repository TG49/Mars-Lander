#pragma once

#define NDEBUG
#include <GL/glut.h>
#include <vector>

/// <summary>
/// Class to handle all necessary information for opengl textures
/// </summary>
class textureObject {
public:
	textureObject(GLuint index=0, int height=0, int width=0, int nrChannels=0) : index(index), height(height), width(width), nrChannels(nrChannels) { };
	~textureObject() { };


	void setTextureObject(GLuint indexIn, int heightIn, int widthIn, int channelsIn);


	GLuint getIndex();
	int getHeight();
	int getwidth();
	int getChannels();

protected:
	GLuint index; //Index within the texture buffer
	int height;	//height of 2D image
	int width;	//width of 2D image
	int nrChannels; //Number of channels associated with each pixel

};