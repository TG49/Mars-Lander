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