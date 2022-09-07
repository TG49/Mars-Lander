#include "TextureObject.h"
#define NDEBUG
#include <GL/glut.h>
#include <vector>

/// <summary>
/// Function constructs a texture object. Have done it as a function to enable easier implementation into other class' methods
/// </summary>
/// <param name="indexIn">index of texture in buffer</param>
/// <param name="heightIn">height of 2D texture</param>
/// <param name="widthIn">width of 2D texture</param>
/// <param name="channelsIn">channels associated with each pixel</param>
void textureObject::setTextureObject(GLuint indexIn, int heightIn, int widthIn, int channelsIn) {
	//If loading pixel data must do so when the texture object is bound
	index = indexIn;
	height = heightIn;
	width = widthIn;
	nrChannels = channelsIn;
}

/// <summary>
/// Returns index of texture in buffer
/// </summary>
/// <returns>GLuint</returns>
GLuint textureObject::getIndex() { return index; };

/// <summary>
/// Returns height of texture in buffer
/// </summary>
/// <returns>int</returns>
int textureObject::getHeight() { return height; };

/// <summary>
/// Returns width of texture in buffer
/// </summary>
/// <returns>int</returns>
int textureObject::getwidth() { return width; };

/// <summary>
/// Returns number of channels associated with each pixel
/// </summary>
/// <returns>int</returns>
int textureObject::getChannels() { return nrChannels; };