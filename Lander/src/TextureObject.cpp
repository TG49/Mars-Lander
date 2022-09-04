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

void heightMapTexture::setTextureObject(GLuint indexIn, int heightIn, int widthIn, int channelsIn) {
	//If loading pixel data must do so when the texture object is bound
	index = indexIn;
	height = heightIn;
	width = widthIn;
	nrChannels = channelsIn;

	buildHeightVector();

}

heightMapTexture::heightMapTexture() { pixels = NULL; pixelDataLoaded = false; };

GLfloat heightMapTexture::getHeightValue(double u, double v) {
	return HeightValue(u, v);
}

void heightMapTexture::buildHeightVector() {
	loadPixelData();
	for (int i = 0; i < height * width * nrChannels; i++)
	{
		heights.push_back((GLfloat)pixels[i]);
	}
	deletePixelData();
}

GLfloat heightMapTexture::HeightValue(double u, double v) {
	//cout << "U: " << u << " V: " << v << endl;
	int column = int(u * width);
	int row = int(v * height);

	while (row > height) {
		row -= height;
	}
	while (row < 0) {
		row += height;
	}
	while (column < 0) {
		column += width;
	}
	while (column > width) {
		column -= width;
	}

	//Required as if accessing north pole heightvertex is out of index
	if (row == height)
	{
		row -= 1;
	}

	if (row == 0) {
		row += 1;
	}

	int index = row * elementsPerLine + column * nrChannels;

	return heights.at(index);
}

void heightMapTexture::deletePixelData() {
	if (pixelDataLoaded)
	{
		delete[] pixels;
		pixelDataLoaded = false;
		pixels = NULL;
	}
}

void heightMapTexture::loadPixelData() {
	if (!pixelDataLoaded) {
		pixels = new GLubyte[height * width * nrChannels];
		elementsPerLine = width * nrChannels;
		glGetTexImage(GL_TEXTURE_2D, 0, GL_RED, GL_UNSIGNED_BYTE, pixels);
		pixelDataLoaded = true;
	}
}