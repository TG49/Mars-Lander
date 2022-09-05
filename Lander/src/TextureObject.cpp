#include "TextureObject.h"
#define NDEBUG
#include <GL/glut.h>
#include <vector>

void textureObject::setTextureObject(GLuint indexIn, int heightIn, int widthIn, int channelsIn) {
	//If loading pixel data must do so when the texture object is bound
	index = indexIn;
	height = heightIn;
	width = widthIn;
	nrChannels = channelsIn;
}

GLuint textureObject::getIndex() { return index; };
int textureObject::getHeight() { return height; };
int textureObject::getwidth() { return width; };
int textureObject::getChannels() { return nrChannels; };