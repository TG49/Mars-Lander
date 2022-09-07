#include "planar_mesh.h"
#define NDEBUG
#include <GL/glut.h>
#include <iostream>
#include <string>


#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"



void SquarePlaneMesh::buildSquarePlane(int meshResolutionIn, int numberOfTextureRepeatsIn, double sizeIn, std::string filename) {
   
    meshResolution = meshResolutionIn;
    size = sizeIn;
    textureRepeats = numberOfTextureRepeatsIn;

    loadVertices();
    loadIndices();
    loadTexture(filename);

}

void SquarePlaneMesh::loadVertices() {
    if (meshResolution >= 9) {
        meshResolution = 8;
    }
    else if (meshResolution < 3) { meshResolution = 3; }

    if (textureRepeats < 1) {
        textureRepeats = 1;
    }
    LevelsOfDetail = meshResolution - 2;
    meshResolution = pow(2, meshResolution);


    float m = meshResolution;
    float n = meshResolution;

    for (float k = 0; k < m; k++) {
        for (float i = 0; i < n; i++) {
            Eigen::Vector2d vertex(size * (-1.0 + 2.0 * i / (n - 1.0)), size * (1.0 - 2.0 * k / (m - 1.0)));
            vertices.push_back(vertex);

            Eigen::Vector2d textureCoords(textureRepeats * i / (n - 1.0), textureRepeats * (1.0 - k / (m - 1.0)));
            textureCoordinates.push_back(textureCoords);
        }
    }
}

void SquarePlaneMesh::loadIndices() {
    indices.resize(LevelsOfDetail);
    for (int j = 0; j < indices.size(); j++)
    {
        int multiple = pow(2, j);
        for (float k = 0; k < meshResolution - multiple; k += multiple) {
            for (float i = 0; i < meshResolution - multiple; i += multiple) {
                indices[j].push_back(k * meshResolution + i);
                indices[j].push_back(k * meshResolution + i + multiple);
                indices[j].push_back((k + multiple) * meshResolution + i + multiple);
                indices[j].push_back((k + multiple) * meshResolution + i);
            }
        }
    }
}

int SquarePlaneMesh::getSize() {
    return size;
}

int SquarePlaneMesh::getmeshResolution() {
    return meshResolution;
}

int SquarePlaneMesh::getTextureRepeats() {
    return textureRepeats;
}

void SquarePlaneMesh::loadTexture(std::string filename) {
    filenameOfTexture = filename;
    int height, width, nrChannels;
    unsigned char* data = stbi_load(filename.c_str(), &width, &height, &nrChannels, 0);
    GLuint index;
    glGenTextures(1, &index);
    glBindTexture(GL_TEXTURE_2D, index);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);

    if (data)
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data);
    else
        std::cout << "failed To Load Texture " << std::endl;

    stbi_image_free(data);
    texture.setTextureObject(index, height, width, nrChannels);
}

void sphericalMesh::drawSphere(double radius, int slices, int stacks) {
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, texture.getIndex());
    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);

    GLUquadric* quad = gluNewQuadric();
    gluQuadricTexture(quad, GL_TRUE);
    gluSphere(quad, radius, slices, stacks);
    glDisable(GL_TEXTURE_2D);
    gluDeleteQuadric(quad);
}

void sphericalMesh::loadTexture(std::string filename) {
    filenameOfTexture = filename;
    int height, width, nrChannels;
    unsigned char* data = stbi_load(filename.c_str(), &width, &height, &nrChannels, 0);
    GLuint index;
    glGenTextures(1, &index);
    glBindTexture(GL_TEXTURE_2D, index);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);

    if (data)
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data);
    else
        std::cout << "failed To Load Texture " << std::endl;

    stbi_image_free(data);
    texture.setTextureObject(index, height, width, nrChannels);
}

void sphericalMesh::buildSphereObject(std::string filename) {
    loadTexture(filename);
}

void SquarePlaneMesh::drawMesh(double terrainAngle, double altitude, double transistionAltitude) {
    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
    glBindTexture(GL_TEXTURE_2D, texture.getIndex());
    glEnable(GL_TEXTURE_2D);
    glColor3f(1.0, 1.0, 1.0);
    glNormal3d(0.0, 1.0, 0.0);
    glPushMatrix();
    glRotated(terrainAngle, 0.0, 1.0, 0.0);
    glBegin(GL_QUADS);

    int useLoD = 0;
    std::cout << indices.size() << std::endl;
    //Use LoDs. Select which lod to use
    while (useLoD < indices.size()-1)
    {
        if (altitude < transistionAltitude/(pow(1.2,useLoD)))
        {
            useLoD += 1;
        }
        else {
            break;
        }
    }
    std::cout << "Using LoD: " << useLoD << std::endl;
    for (int i = 0; i < indices[useLoD].size(); i++) {
        int toPlot = indices[useLoD][i];
        glTexCoord2d(textureCoordinates[toPlot][0], textureCoordinates[toPlot][1]);
        glVertex3d(vertices[toPlot][0], -altitude, vertices[toPlot][1]); //Draw Plane
    }

    glEnd();
    glPopMatrix();
    glDisable(GL_TEXTURE_2D);
    glDisable(GL_DEPTH_TEST);
}