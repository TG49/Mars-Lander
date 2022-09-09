#include "planar_mesh.h"
#define NDEBUG
#include <GL/glut.h>
#include <iostream>
#include <string>


#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"


/// <summary>
/// Function Builds and occupies a Square Plane Mesh Object
/// </summary>
/// <param name="meshResolutionIn">The resolution of the mesh. The number of vertices is pow(2,meshResolutionIn)</param>
/// <param name="numberOfTextureRepeatsIn">How many times the texture should be repeated over the mesh</param>
/// <param name="sizeIn">Half the side length of the mesh</param>
void SquarePlaneMesh::buildSquarePlane(int meshResolutionIn, int numberOfTextureRepeatsIn, double sizeIn) {
   
    meshResolution = meshResolutionIn;
    size = sizeIn;
    textureRepeats = numberOfTextureRepeatsIn;

    loadVertices();
    loadIndices();

    heightMap.buildHeightMap(0.001, 5, 0.5);


}

/// <summary>
/// Occupy the vertex and texture coordinate members
/// </summary>
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

/// <summary>
/// Occupy the indices vertex, including required levels of detail. Number of LoDs depends upon mesh resolution
/// </summary>
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

/// <summary>
/// Returns the size of a square plane mesh
/// </summary>
/// <returns>double</returns>
double SquarePlaneMesh::getSize() {
    return size;
}

/// <summary>
/// Returns mesh resolution of a square plane mesh
/// </summary>
/// <returns>int</returns>
int SquarePlaneMesh::getmeshResolution() {
    return meshResolution;
}

/// <summary>
/// Returns the number of times a texture is repeated
/// </summary>
/// <returns>int</returns>
int SquarePlaneMesh::getTextureRepeats() {
    return textureRepeats;
}

/// <summary>
/// Loads a texture associated with a square plane mesh. Can only be used if the square plane mesh is the only object in the glut window
/// </summary>
/// <param name="filename">The file name of the texture image</param>
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

/// <summary>
/// Draws the sphere object
/// </summary>
/// <param name="radius">Radius of sphere</param>
/// <param name="slices">Number of slices</param>
/// <param name="stacks">Number of stacks</param>
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

/// <summary>
/// Loads texture associated with spherical mesh. Can only be used if the spherical mesh is the only object in a glut window
/// </summary>
/// <param name="filename">Filename of the texture to be used for the spherical mesh</param>
void sphericalMesh::loadTexture(std::string filename) {
    filenameOfTexture = filename;
    int height, width, nrChannels;
    unsigned char* data = stbi_load(filename.c_str(), &width, &height, &nrChannels, 0);
    GLuint index;
    glGenTextures(1, &index);
    glBindTexture(GL_TEXTURE_2D, index);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

    if (data)
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data);
    else
        std::cout << "failed To Load Texture " << std::endl;

    stbi_image_free(data);
    texture.setTextureObject(index, height, width, nrChannels);
}

/// <summary>
/// Draws the square plane mesh
/// </summary>
/// <param name="terrainAngle">Angle of the terrain</param>
/// <param name="altitude">altitude of lander above the terrain</param>
/// <param name="transistionAltitude">Altitude which the mesh is first drawn. Used for determining which LoD to use</param>
void SquarePlaneMesh::drawMesh(double terrainAngle, double altitude, double transistionAltitude, Eigen::Vector3d position, double terrain_offset_x, double terrain_offset_y) {
    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
    glBindTexture(GL_TEXTURE_2D, texture.getIndex());
    glEnable(GL_TEXTURE_2D);
    glColor3f(1.0, 1.0, 1.0);
    glNormal3d(0.0, 1.0, 0.0);
    glPushMatrix();
    glRotated(terrainAngle, 0.0, 1.0, 0.0);
    glBegin(GL_QUADS);

    int useLoD = indices.size()-1;
    //std::cout << indices.size() << std::endl;
    //Use LoDs. Select which lod to use
    while (useLoD > 0)
    {
        if (altitude < transistionAltitude/(pow(1.5,(indices.size()-useLoD))))
        {
            useLoD -= 1;
        }
        else {
            break;
        }
    }
    std::cout << "LoD: " << useLoD << std::endl;
    //std::cout << "Using LoD: " << useLoD << std::endl;
    heightMap.setOffset(position);
    std::cout << "Height Offset: " << heightMap.getOffset() << std::endl;
    std::cout << "Texture Offsets: " << terrain_offset_x << ", " << terrain_offset_y << std::endl;
    for (int i = 0; i < indices[useLoD].size(); i++) {
        int toPlot = indices[useLoD][i];
        glTexCoord2d(textureCoordinates[toPlot][0] + terrain_offset_x, textureCoordinates[toPlot][1] + terrain_offset_y);
        double height = heightMap.getHeightValue(vertices[toPlot], position);
        glVertex3d(vertices[toPlot][0], -altitude + height, vertices[toPlot][1]); //Draw Plane
    }

    glEnd();
    glPopMatrix();
    glDisable(GL_TEXTURE_2D);
    glDisable(GL_DEPTH_TEST);
}

/// <summary>
/// Builds the necessary Libnoise modules
/// </summary>
/// <param name="frequency">Frequency of perlin noise module</param>
/// <param name="octaves">Octaves of Perlin Noise module</param>
/// <param name="persistance">Peristance of Perlin Noise module</param>
void terrainNoise::buildHeightMap(double frequency, double octaves, double persistance)
{
    positionOffset.SetFrequency(10);
    positionOffset.SetOctaveCount(4);

    heightModule.SetFrequency(frequency);
    heightModule.SetOctaveCount(octaves);
    heightModule.SetPersistence(persistance);
}

/// <summary>
/// Sets the offset from the position vector magnitude
/// </summary>
/// <param name="position"></param>
void terrainNoise::setOffset(Eigen::Vector3d position)
{
    offset = 1*positionOffset.GetValue(position.normalized()[0], position.normalized()[1], position.normalized()[2]);
}

/// <summary>
/// Gets the height of individual vertices
/// </summary>
/// <param name="vertex">Vertex</param>
/// <returns>The height of the vertex from the ground plane</returns>
double terrainNoise::getHeightValue(Eigen::Vector2d vertex, Eigen::Vector3d position)
{
    return 400 * heightModule.GetValue(position[0] + vertex[0], position[0], position[2]+vertex[1]);

}


/// <summary>
/// Builds all objects and meshes in the close up view window
/// </summary>
/// <param name="meshResolutionIn">Resolution of the surface mesh</param>
/// <param name="numberOfTextureRepeatsIn">Number of times the surface texture repeats</param>
/// <param name="sizeIn">Size of the surface plane</param>
/// <param name="planetTexture">Filename of the texture used for the planet</param>
/// <param name="surfaceTexture">Filename of the texture used for the surface</param>
void closeUpMeshes::buildcloseUpMeshes(int meshResolutionIn, int numberOfTextureRepeatsIn,
    double sizeIn, std::string planetTexture, std::string surfaceTexture) {
    
    surfaceMesh.buildSquarePlane(meshResolutionIn, numberOfTextureRepeatsIn, sizeIn);

    loadTextures(planetTexture, surfaceTexture);
}

/// <summary>
/// Loads textures associated with all objects in the clsoe up view window. Must be used as OpenGL cannot deal with multiple texture generation calls per window. 
/// </summary>
/// <param name="planetTextureName">Filename of the planet texture</param>
/// <param name="surfaceTextureName">Filename of the surface texture</param>
void closeUpMeshes::loadTextures(std::string planetTextureName, std::string surfaceTextureName)
{
    
    int height, width, nrChannels;
    GLuint* index = new GLuint[2];
    glGenTextures(2, index);

    planet.filenameOfTexture = planetTextureName;
    unsigned char* data = stbi_load(planet.filenameOfTexture.c_str(), &width, &height, &nrChannels, 0);
    glBindTexture(GL_TEXTURE_2D, index[0]);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

    if (data)
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data);
    else
        std::cout << "failed To Load Texture " << std::endl;
    std::cout << "Close Up Texture Planet: " << index[0] << std::endl;
    stbi_image_free(data);
    planet.texture.setTextureObject(index[0], height, width, nrChannels);

    surfaceMesh.filenameOfTexture = surfaceTextureName;
    unsigned char* data2 = stbi_load(surfaceMesh.filenameOfTexture.c_str(), &width, &height, &nrChannels, 0);
    glBindTexture(GL_TEXTURE_2D, index[1]);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

    if (data2)
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data2);
    else
        std::cout << "failed To Load Texture " << std::endl;

    std::cout << "Close Up Texture Surface: " << index[1] << std::endl;
    stbi_image_free(data2);
    surfaceMesh.texture.setTextureObject(index[1], height, width, nrChannels);
    delete[] index;
}