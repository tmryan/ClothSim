/*
	Cloth Simulation++ (Wind, Gravity, Collision) - CS116B
	Author: Tom Ryan
	Last Modified: 05/16/17
	
	CONTROLS:
	'W' - switch to downwards facing camera
	'A' - switch to right facing camera
	'S' - reset to default camera
	'D' - switch to left facing camera
	'Z' - Toggle wind (on by default)
	'X' - Toggle sphere movement (on by default)
	spacebar - drop cloth
	enter - pause simulation
*/

#include <stdlib.h>
#include <math.h>
#include <ctime>
#include <chrono>
#include <vector>
#include <queue>

#ifdef __APPLE__
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#include <GLUT/glut.h>
#elif __linux__
#include <GL/gl.h>
#include <GL/glut.h>
#elif _WIN32
#include <GLUT/glut.h>
#endif

#ifndef PI
#define PI 3.14159265358979323846
#endif

//////////////
// Constants
//////////

const GLint WIDTH = 1000;
const GLint HEIGHT = 800;
const GLfloat ASPECT = (GLfloat)WIDTH / (GLfloat)HEIGHT;
const GLfloat FOV = 70.0f;

const GLfloat PARTICLE_MASS_KG = 50.0f;
const int CONSTRAINT_ITERATIONS = 50;
const long MIN_TIME_STEP = 16;

//////////////////////////////
// Vector Maths Declarations
//////////////////////////

typedef struct vec3 {
	GLfloat x;
	GLfloat y;
	GLfloat z;
} vec3;

typedef struct vec4 {
	GLfloat x;
	GLfloat y;
	GLfloat z;
	GLfloat w;
} vec4;

GLfloat magnitude(const vec3 &vec);
vec3 normalize(const vec3 &vec);
vec3 cross(const vec3 &u, const vec3 &v);
GLfloat dot(const vec3 &u, const vec3 &v);
vec3 operator/(const vec3 &vec, const GLfloat &scalar);
vec3 operator*(const vec3 &vec, const GLfloat &scalar);
vec3 operator+(const vec3 &u, const vec3 &v);
vec3 operator-(const vec3 &u, const vec3 &v);
bool operator!=(const vec3 &u, const vec3 &v);
vec4 operator+(const vec4 &u, const vec4 &v);

///////////////////////////////
// Forces & Physics Constants
///////////////////////////

const vec3 gravity = vec3{ 0.0f, -0.02f, 0.0f };
const GLfloat springConstK = 0.00000000002f;
const GLfloat damperConstD = 1.0f - 0.00002f;

//////////////////////////////
// type Particle declaration
//////////////////////////

typedef struct Particle {
	vec3 position;
	vec3 prevPosition;
	vec3 acceleration;
	vec4 color;
	GLfloat mass;
	bool pinned;
} Particle;

typedef struct Spring {
	Particle *p0;
	Particle *p1;
	GLfloat restLength;
} Spring;

////////////////////////////////////////////////
// virtual class Actor & modifier declarations
////////////////////////////////////////////

class Actor {
	protected:
		vec3 position;
		vec4 color;
		std::vector<GLfloat> vertices;

	public:
		virtual void draw() = 0;
		virtual vec3 getPosition() = 0;
};

class Moveable {
	protected:
		bool isMoving;
	
	public:
		virtual void move(long deltaT) = 0;
};

class Collidable {
	public:
		virtual bool contains(vec3 point) = 0;
};

//////////////////////////////
// class Sphere declarations
//////////////////////////

class Sphere : public Actor, Collidable, Moveable {
private:
	vec3 scale;
	GLfloat radius;
	vec3 velocity;

public:
	Sphere(vec3 &position, vec4 &color, GLfloat radius, GLfloat scale, const std::vector<GLfloat> &vertices);
	void draw();
	void move(long deltaT);
	bool contains(vec3 point);
	void toggleMovement();
	vec3 getPosition();
	GLfloat getRadius();
};

////////////////////////////
// class Rope declarations
////////////////////////

class ClothSheet : public Actor, Moveable {
	private:
		std::vector< std::vector<Particle>> particles;
		std::vector< std::vector<Spring>> springs;
		std::vector<Sphere*> potentialColliders;
		std::queue<Particle*> pinnedParticles;
		vec3 vWindForce;

		void generateParticleSheet(GLfloat height, GLfloat width);
		void satisfyConstraints();
		void accumulateForces();

	public:
		ClothSheet(vec3 position, vec4 color, int width, int height);
		void draw();
		void move(long deltaT);
		void handleCollision();
		void applyWindForce(vec3 &windForce);
		void detach();
		void pushCollidable(Sphere *collidable);
		vec3 getPosition();
};

////////////////////////////
// class Wind declarations
////////////////////////

class Wind {
	private:
		bool enabled;
		long timeBlowing;
		vec3 windForce;
		
	public:
		Wind(vec3 &windForce);
		vec3 generateWindForce(long deltaT);
		void toggleWind();
};

///////////////////////////////////////////
// Cloth Simulation Function Declarations
///////////////////////////////////////

void generateCube(int smoothness, std::vector<GLfloat> &vertices);
void generateSpherifiedCube(int smoothness, std::vector<GLfloat> &vertices);
void pause();

////////////////////////
// OpenGL Declarations
////////////////////

typedef struct CamDirection {
	bool up;
	bool down;
	bool left;
	bool right;
	bool forward;
	bool backward;
} CamDirection;

typedef struct Camera {
	vec3 position;
	vec3 facing;
	vec3 up;
} Camera;

void initOpenGL();
void resetProjection();
void resizeViewport(GLint width, GLint height);
void draw();
void driver();
void switchCamera(Camera &camera);
void keyboardHandler(unsigned char key, int x, int y);

////////////
// Globals
////////

// Note: Using std::vector since actors are persistent
std::vector<Actor*> actors;
std::vector<Collidable*> collidables;

ClothSheet *cloth;
Sphere *sphere;
Wind *wind;

long lastUpdateT = 0;
bool paused = false;

// Lighting settings
GLfloat lightOneAmbient[] = { 0.0f, 0.0f, 0.0f, 1.0f };
GLfloat lightOneDiffuse[] = { 1.0f, 1.0f, 1.0f, 1.0f };
GLfloat lightOnePosition[] = { 1.0f, 2.0f, 5.0f, 1.0f };

// Note: Press 'S', 'W', 'A', 'D' to switch between these cameras
Camera camera = { vec3{ 0.0f, 0.0f, 1.0f }, vec3{ 0.0f, 0.0f, 0.0f }, vec3{ 0.0f, 1.0f, 0.0f } };
Camera cameraUp = { vec3{ 0.0f, 2.0f, 1.0f }, vec3{ 0.0f, 0.0f, -1.0f }, vec3{ 0.0f, 1.0f, 0.0f } };
Camera cameraLeft = { vec3{ -2.0f, 0.5f, -3.0f }, vec3{ 0.0f, 0.0f, -1.0f }, vec3{ 0.0f, 1.0f, 0.0f } };
Camera cameraRight = { vec3{ 2.0f, 0.5f, -3.0f }, vec3{ 0.0f, 0.0f, -1.0f }, vec3{ 0.0f, 1.0f, 0.0f } };

///////////
// main()
///////

int main(int argc, char *argv[]) {
	GLint window;

	srand(static_cast<unsigned int>(time(0)));

	// Initializing scene state
	std::vector<GLfloat> vertices(0);
	generateSpherifiedCube(16, vertices);
    vec3 spherePos = vec3{ -0.5f, -0.5f, -2.5f };
    vec4 sphereColor = vec4{ 0.212f, 0.969f, 0.627f, 1.0f };
	sphere = new Sphere(spherePos,
						sphereColor, 
						1.0f, 0.5f, vertices);
	actors.push_back(sphere);

	// Creating cloth
    vec3 clothPos = vec3{ -1.0f, 1.0f, -2.0f };
    vec4 clothColor = vec4{ 0.212f, 0.969f, 0.627f, 1.0f };
	cloth = new ClothSheet(clothPos, 
							clothColor, 
							50, 50);
	actors.push_back(cloth);

	// Pushing nearby Collidable actors to cloth
	cloth->pushCollidable(sphere);

	// Seeding wind force
    vec3 windForce = vec3{ 0.0f, -2.0f, -1.5f };
	wind = new Wind(windForce);

	// Initializing window
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_ALPHA | GLUT_DEPTH);
	glutInitWindowSize(WIDTH, HEIGHT);
	glutInitWindowPosition(0, 0);
	window = glutCreateWindow("Cloth Simulation");

	// Setting renderer callback functions
	glutDisplayFunc(&draw);
	glutIdleFunc(&driver);
	glutReshapeFunc(&resizeViewport);

	// Setting input callback functions
	glutKeyboardFunc(&keyboardHandler);

	// Initializing OpenGL
	initOpenGL();

	glutMainLoop();
}

/////////////////////////////
// OpenGL Related Functions
/////////////////////////

void initOpenGL() {
	glEnable(GL_DEPTH_TEST);

	// Setting up lights
	glLightfv(GL_LIGHT0, GL_AMBIENT, lightOneAmbient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, lightOneDiffuse);
	glLightfv(GL_LIGHT0, GL_POSITION, lightOnePosition);

	// Enabling lights
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glLightModelf(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);

	// Setting color materials
	glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT);
	glColorMaterial(GL_FRONT_AND_BACK, GL_DIFFUSE);
	glEnable(GL_COLOR_MATERIAL);

	// Setting shading technique to interpolate colors
	glShadeModel(GL_SMOOTH);

	// Preparing projection matrix
	resetProjection();

	// Setting default view
	switchCamera(camera);
}

// Resets projection matrix to initial values
void resetProjection() {
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	gluPerspective(FOV, ASPECT, 0.1, 100.0f);
}

// Adjust OpenGL state on window resize
void resizeViewport(GLint width, GLint height) {
	// Avoiding divide by 0 error
	if (height < 10) {
		height = 10;
	}

	// Setting viewport to new window size
	glViewport(0, 0, width, height);

	// Preparing projection matrix
	resetProjection();

	// Resetting view
	switchCamera(camera);
}

// Main "loop" since GLUT is event driven
void driver() {
	// This monster statement is getting the current time in milliseconds from steady_clock
	auto currT = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now());

	long deltaT;

	// Calculating the time difference since last tick
	if (lastUpdateT != 0) {
		deltaT = currT.time_since_epoch().count() - lastUpdateT;
	}
	else {
		// Eating first deltaT since it is very large
		deltaT = 0;
		lastUpdateT = currT.time_since_epoch().count();
	}

	// Update and draw if deltaT > minimum time step
	if (deltaT > MIN_TIME_STEP) {
		if (!paused) {
			// Updating state
			sphere->move(deltaT);
            vec3 windUpdate = wind->generateWindForce(deltaT);
			cloth->applyWindForce(windUpdate);
			cloth->move(deltaT);
		}

		// Drawing scene
		draw();

		// Saving last frame time
		lastUpdateT = currT.time_since_epoch().count();
	}
}

void draw() {
	// Clearing color and depth buffers
	glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	Actor *actor;

	// Delegating draw to each Actor object
	for (unsigned int i = 0; i < actors.size(); i++) {
		actor = actors.at(i);
		actor->draw();
	}

	glutSwapBuffers();
}

// Updates view to given camera
void switchCamera(Camera &camera) {
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	gluLookAt(camera.position.x, camera.position.y, camera.position.z,
		camera.facing.x, camera.facing.y, camera.facing.z,
		camera.up.x, camera.up.y, camera.up.z);
}

void keyboardHandler(unsigned char key, int x, int y) {
	switch (key) {
	case 27:
		// Exit program if escape key pressed
		exit(0);
		break;
	case 32:
		// Drop cloth if space bar pressed
		cloth->detach();
		break;
	case 13:
		// Press 'enter' to pause state updates
		pause();
		break;
	case 'a':
		// Swapping cameras
		switchCamera(cameraLeft);
		break;
	case 'd':
		switchCamera(cameraRight);
		break;
	case 'w':
		switchCamera(cameraUp);
		break;
	case 's':
		switchCamera(camera);
		break;
	case 'z':
		wind->toggleWind();
		break;
	case 'x':
		sphere->toggleMovement();
		break;
	default:
		break;
	}
}

///////////////////////////////
// CLoth Simulation functions
///////////////////////////

void generateCube(int smoothness, std::vector<GLfloat> &vertices) {
	GLfloat delta = 1.0f / smoothness;

	GLfloat z = 1.0f;
	for (GLfloat x = -1.0f; x <= 1.0f; x += delta) {
		for (GLfloat y = 1.0f; y >= -1.0f; y -= delta) {
			// Generating front of cube
			vertices.push_back(x);
			vertices.push_back(y - delta);
			vertices.push_back(z);

			vertices.push_back(x);
			vertices.push_back(y);
			vertices.push_back(z);

			vertices.push_back(x + delta);
			vertices.push_back(y);
			vertices.push_back(z);

			vertices.push_back(x);
			vertices.push_back(y - delta);
			vertices.push_back(z);

			vertices.push_back(x + delta);
			vertices.push_back(y);
			vertices.push_back(z);

			vertices.push_back(x + delta);
			vertices.push_back(y - delta);
			vertices.push_back(z);

			// Generating back of cube
			vertices.push_back(x);
			vertices.push_back(y - delta);
			vertices.push_back(-1 * z);

			vertices.push_back(x);
			vertices.push_back(y);
			vertices.push_back(-1 * z);

			vertices.push_back(x + delta);
			vertices.push_back(y);
			vertices.push_back(-1 * z);

			vertices.push_back(x);
			vertices.push_back(y - delta);
			vertices.push_back(-1 * z);

			vertices.push_back(x + delta);
			vertices.push_back(y);
			vertices.push_back(-1 * z);

			vertices.push_back(x + delta);
			vertices.push_back(y - delta);
			vertices.push_back(-1 * z);
		}
	}

	GLfloat x = 1.0f;
	for (GLfloat z = 1.0f; z >= -1.0f; z -= delta) {
		for (GLfloat y = 1.0f; y >= -1.0f; y -= delta) {
			// Generating right side of cube
			vertices.push_back(x);
			vertices.push_back(y - delta);
			vertices.push_back(z);

			vertices.push_back(x);
			vertices.push_back(y);
			vertices.push_back(z);

			vertices.push_back(x);
			vertices.push_back(y);
			vertices.push_back(z - delta);

			vertices.push_back(x);
			vertices.push_back(y - delta);
			vertices.push_back(z);

			vertices.push_back(x);
			vertices.push_back(y);
			vertices.push_back(z - delta);

			vertices.push_back(x);
			vertices.push_back(y - delta);
			vertices.push_back(z - delta);

			// Generating left side of cube
			vertices.push_back(-1 * x);
			vertices.push_back(y - delta);
			vertices.push_back(z);

			vertices.push_back(-1 * x);
			vertices.push_back(y);
			vertices.push_back(z);

			vertices.push_back(-1 * x);
			vertices.push_back(y);
			vertices.push_back(z - delta);

			vertices.push_back(-1 * x);
			vertices.push_back(y - delta);
			vertices.push_back(z);

			vertices.push_back(-1 * x);
			vertices.push_back(y);
			vertices.push_back(z - delta);

			vertices.push_back(-1 * x);
			vertices.push_back(y - delta);
			vertices.push_back(z - delta);
		}
	}

	GLfloat y = 1.0f;
	for (GLfloat x = -1.0f; x <= 1.0f; x += delta) {
		for (GLfloat z = 1.0f; z >= -1.0f; z -= delta) {
			// Generating top of cube
			vertices.push_back(x);
			vertices.push_back(y);
			vertices.push_back(z - delta);

			vertices.push_back(x);
			vertices.push_back(y);
			vertices.push_back(z);

			vertices.push_back(x + delta);
			vertices.push_back(y);
			vertices.push_back(z);

			vertices.push_back(x);
			vertices.push_back(y);
			vertices.push_back(z - delta);

			vertices.push_back(x + delta);
			vertices.push_back(y);
			vertices.push_back(z);

			vertices.push_back(x + delta);
			vertices.push_back(y);
			vertices.push_back(z - delta);

			// Generating bottom of cube
			vertices.push_back(x);
			vertices.push_back(-1 * y);
			vertices.push_back(z - delta);

			vertices.push_back(x);
			vertices.push_back(-1 * y);
			vertices.push_back(z);

			vertices.push_back(x + delta);
			vertices.push_back(-1 * y);
			vertices.push_back(z);

			vertices.push_back(x);
			vertices.push_back(-1 * y);
			vertices.push_back(z - delta);

			vertices.push_back(x + delta);
			vertices.push_back(-1 * y);
			vertices.push_back(z);

			vertices.push_back(x + delta);
			vertices.push_back(-1 * y);
			vertices.push_back(z - delta);
		}
	}
}

void generateSpherifiedCube(int smoothness, std::vector<GLfloat> &vertices) {
	generateCube(smoothness, vertices);

	vec3 point;
	vec3 normal;

	// Normalizing each vertex to create a unit sphere
	for (int i = 0; i < vertices.size(); i += 3) {
		point = vec3{ vertices.at(i), vertices.at(i + 1), vertices.at(i + 2) };
		normal = normalize(point);
		vertices.at(i) = normal.x;
		vertices.at(i + 1) = normal.y;
		vertices.at(i + 2) = normal.z;
	}
}

void pause() {
	paused = !paused;
}

//////////////////
// class: Sphere
//////////////

Sphere::Sphere(vec3 &position, vec4 &color, GLfloat radius, GLfloat scale, const std::vector<GLfloat> &vertices) {
	this->position = position;
	this->scale = vec3{ scale, scale, scale };
	this->color = color;
	this->radius = radius * scale;
	this->vertices = vertices;
	velocity = vec3{ 0.05f, 0.0f, 0.0f };
	isMoving = true;
}

void Sphere::draw() {
	vec3 normal;
	vec3 p1;
	vec3 p2;
	vec3 p3;

	// Vertex data attributes
	unsigned int vertexDataSize = 3;

	// Applying transformations
	glPushMatrix();
	glTranslatef(position.x, position.y, position.z);
	glScalef(scale.x, scale.y, scale.z);

	// Drawing current actor
	glBegin(GL_TRIANGLES);

	int vertCount = 0;

	// Drawing object
	for (int j = 0; j < vertices.size(); j += vertexDataSize) {
		glColor4f(color.x, color.y, color.z, color.w);

		// Finding normals for lighting
		if (vertCount % 6 == 0 || vertCount == 0) {
			p1 = { vertices.at(j), vertices.at(j + 1), vertices.at(j + 2) };
			p2 = { vertices.at(j + 3), vertices.at(j + 4), vertices.at(j + 5) };
			p3 = { vertices.at(j + 6), vertices.at(j + 7), vertices.at(j + 8) };

			normal = normalize(cross(p2 - p1, p3 - p1));
			glNormal3f(normal.x, normal.y, normal.z);
		}

		// Specifying triangle vertices
		glVertex3f(vertices.at(j), vertices.at(j + 1), vertices.at(j + 2));

		vertCount++;
	}

	// Done drawing current actor
	glEnd();
	glPopMatrix();
}

// Moves the Sphere back and forth along the x-axis between the hard coded bounds
void Sphere::move(long deltaT) {
	if (isMoving) {
		if (position.x < -1.5f) {
			velocity.x = 0.05f;
		} else if (position.x > 1.5f) {
			velocity.x = -0.05f;
		}

		position = position + velocity;
	}
}

void Sphere::toggleMovement() {
	isMoving = !isMoving;
}

// Checks whether a given point lies within this Sphere
bool Sphere::contains(vec3 point) {
	bool isContained = false;

	vec3 deltaDist = point - position;

	if (magnitude(deltaDist) < radius) {
		isContained = true;
	}

	return isContained;
}

vec3 Sphere::getPosition() {
	return position;
}

GLfloat Sphere::getRadius() {
	return radius;
}

//////////////////////
// class: ClothSheet
//////////////////

ClothSheet::ClothSheet(vec3 position,  vec4 color, int width, int height) {
	this->position = position;
	this->color = color;

	// Note: Not the best place to store a wind force, but can sort that out some other time
	vWindForce = vec3{ 0.0f, 0.0f, 0.0f };

	generateParticleSheet((GLfloat)width, (GLfloat)height);

	potentialColliders = std::vector<Sphere*>();

	pinnedParticles = std::queue<Particle*>();

	// Pinning top left three particles
	particles.at(0).at(0).pinned = true;
	pinnedParticles.push(&particles.at(0).at(0));
	particles.at(0).at(1).pinned = true;
	pinnedParticles.push(&particles.at(0).at(1));
	particles.at(0).at(2).pinned = true;
	pinnedParticles.push(&particles.at(0).at(2));

	// Pinning top right three particles
	particles.at(0).at(particles.size() - 1).pinned = true;
	pinnedParticles.push(&particles.at(0).at(particles.size() - 1));
	particles.at(0).at(particles.size() - 2).pinned = true;
	pinnedParticles.push(&particles.at(0).at(particles.size() - 2));
	particles.at(0).at(particles.size() - 3).pinned = true;
	pinnedParticles.push(&particles.at(0).at(particles.size() - 3));
}

// Draws cloth using particle positions for vertices
void ClothSheet::draw() {
	// Declaring normal calculation variables
	vec3 normal;
	vec3 p1;
	vec3 p2;
	vec3 p3;

	// Applying transformations
	glPushMatrix();

	// Drawing current actor
	glBegin(GL_TRIANGLES);

	// Drawing object
	for (int i = 0; i < particles.size() - 1; i++) {
		for (int j = 0; j < particles.at(i).size() - 1; j++) {
			//glColor4f(color.x, color.y, color.z, color.w);
			glColor4f(particles.at(i).at(j).color.x, particles.at(i).at(j).color.y, 
						particles.at(i).at(j).color.z, particles.at(i).at(j).color.w);

			// Finding upper tri normal for lighting
			p1 = particles.at(i + 1).at(j).position;
			p2 = particles.at(i).at(j).position;
			p3 = particles.at(i).at(j + 1).position;

			normal = normalize(cross(p2 - p1, p3 - p1));
			glNormal3f(normal.x, normal.y, normal.z);

			// Specifying upper triangle vertices
			glVertex3f(p1.x, p1.y, p1.z);
			glVertex3f(p2.x, p2.y, p2.z);
			glVertex3f(p3.x, p3.y, p3.z);

			// Finding lower tri normal for lighting
			p1 = particles.at(i + 1).at(j).position;
			p2 = particles.at(i).at(j + 1).position;
			p3 = particles.at(i + 1).at(j + 1).position;

			normal = normalize(cross(p2 - p1, p3 - p1));
			glNormal3f(normal.x, normal.y, normal.z);

			// Specifying lower triangle vertices
			glVertex3f(p1.x, p1.y, p1.z);
			glVertex3f(p2.x, p2.y, p2.z);
			glVertex3f(p3.x, p3.y, p3.z);
		}
	}

	// Done drawing current actor
	glEnd();
	glPopMatrix();
}

// Moves particles using Verlet integration
void ClothSheet::move(long deltaT) {
	// Note: Using a fixed timestep for this simulation
	GLfloat timeTSquared = 0.01f;
	vec3 vTempPos;

	Particle *particle;

	accumulateForces();
	satisfyConstraints();

	for (int i = 0; i < particles.size(); i++) {
		for (int j = 0; j < particles.at(i).size(); j++) {
			particle = &particles.at(i).at(j);

			if(!particle->pinned) {
				vTempPos = particle->position;
				
				// Calculating new position and storing previous position
				particle->position = ((particle->position * 2.0f) - particle->prevPosition * damperConstD) 
							+ (particle->acceleration * timeTSquared);
				particle->prevPosition = vTempPos;
			}
		}
	}

	handleCollision();
}

// Handles collisions with nearby Spheres
void ClothSheet::handleCollision() {
	Particle *particle;
	Sphere* collidable;
	vec3 vDistance;
	vec3 vNormalizedDist;
	vec3 vScaledDist;

	// Setting offset from surface when projecting
	GLfloat offsetScalar = 0.03f;

	for (int i = 0; i < potentialColliders.size(); i++) {
		collidable = potentialColliders.at(i);
		
		for (int i = 0; i < particles.size(); i++) {
			for (int j = 0; j < particles.at(i).size(); j++) {
				particle = &particles.at(i).at(j);

				if (collidable->contains(particle->position)) {
					vDistance = particle->position - collidable->getPosition();
					vNormalizedDist = normalize(vDistance);
					vScaledDist = (vNormalizedDist * sphere->getRadius());
					
					// Getting vector to position on surface of sphere from origin plus small offset
					particle->position = collidable->getPosition() 
										+ (vNormalizedDist * sphere->getRadius())
										+ (vScaledDist * offsetScalar);
				}
			}
		}
	}
}

// Applies a given wind force to the cloth
void ClothSheet::applyWindForce(vec3 &windForce) {
	vWindForce = windForce;
}

// Unpins pinned particles
void ClothSheet::detach() {
	while (!pinnedParticles.empty()) {
		pinnedParticles.front()->pinned = false;
		pinnedParticles.pop();
	}
}

// Adds an Actor to a list of possible collisions
void ClothSheet::pushCollidable(Sphere *collidable) {
	potentialColliders.push_back(collidable);
}

vec3 ClothSheet::getPosition() {
	return position;
}

// Generates a height*width matrix of particles and a matrix of springs
void ClothSheet::generateParticleSheet(GLfloat height, GLfloat width) {
	// Note: Spacings double as rest length of springs
	GLfloat xSpacing = 2.0f / (height - 1.0f);
	GLfloat ySpacing = 2.0f / (width - 1.0f);
	GLfloat xBendSpacing = xSpacing + xSpacing;
	GLfloat yBendSpacing = ySpacing + ySpacing;
	vec3 vSpacer = position;
	vec4 vColor;

	// Setting size of vector ahead of time to save cycles
	particles = std::vector< std::vector<Particle>>(height, std::vector<Particle>(width));

	// Generating particle matrix
	for (int i = 0; i < (int)height; i++) {
		vSpacer.x = position.x;

		for (int j = 0; j < (int)width; j++) {
			// Choosing color based on particle order
			if (i % 2 != 0 && j % 2 != 0) {
				vColor = vec4{ 0.941f, 0.427f, 0.102f, 1.0f };
			} else {
				vColor = vec4{ 0.996f, 1.0f, 0.906f, 1.0f };
			}
			
			particles.at(i).at(j) = Particle{ 
				vSpacer,
				vSpacer,
				vec3{ 0.0f, 0.0f, 0.0f },
				vColor,
				PARTICLE_MASS_KG,
				false };

			vSpacer.x += xSpacing;
		}

		vSpacer.y -= ySpacing;
	}

	springs = std::vector< std::vector<Spring>>((int)height - 1);

	int row = 0;
	int col = 0;
	int springGroupSize;

	// Generating spring matrix
	for (int k = 0; k < springs.size(); k++) {
		// Adding two bend springs per particle except for last two rows
		if (k < springs.size() - 3) {
			springs.at(k) = std::vector<Spring>(((int)width - 1) * 8 - 1);
			springGroupSize = 8;
		} else {
			springs.at(k) = std::vector<Spring>(((int)width - 1) * 6);
			springGroupSize = 6;
		}

		col = 0;

		for (int l = 0; l < springs.at(k).size(); l += springGroupSize) {
			// Generating four structural and two shear springs per particle
			springs.at(k).at(l) = Spring{ &particles.at(row).at(col), &particles.at(row + 1).at(col), ySpacing };
			springs.at(k).at(l + 1) = Spring{ &particles.at(row).at(col), &particles.at(row).at(col + 1), xSpacing };
			springs.at(k).at(l + 2) = Spring{ &particles.at(row).at(col + 1), &particles.at(row + 1).at(col + 1), ySpacing };
			springs.at(k).at(l + 3) = Spring{ &particles.at(row + 1).at(col), &particles.at(row + 1).at(col + 1), xSpacing };
			springs.at(k).at(l + 4) = Spring{ &particles.at(row + 1).at(col), &particles.at(row).at(col + 1),
												sqrt((xSpacing * xSpacing) + (ySpacing * ySpacing)) };
			springs.at(k).at(l + 5) = Spring{ &particles.at(row).at(col), &particles.at(row + 1).at(col + 1),
												sqrt((xSpacing * xSpacing) + (ySpacing * ySpacing)) };

			// Adding vertical bend spring
			if (k < springs.size() - 3) {
				springs.at(k).at(l + 6) = Spring{ &particles.at(row).at(col), &particles.at(row + 2).at(col), yBendSpacing };
			}

			// Adding horizontal bend spring
			if (l + 7 < springs.at(k).size() - 3) {
				particles.size();
				springs.at(k).at(l + 7) = Spring{ &particles.at(row).at(col), &particles.at(row).at(col + 2), xBendSpacing };
			}

			col++;
		}

		row++;
	}
}

// Moves particles closer to their spring rest length over some number of iterations per frame
void ClothSheet::satisfyConstraints() {
	GLfloat deltaDistance;
	vec3 vCurrentDistance;
	vec3 vConstraints;

	Particle *p0;
	Particle *p1;
	Spring *spring;

	// Satisfying constraints CONSTRAINT_ITERATIONS times per frame
	for (int iteration = 0; iteration < CONSTRAINT_ITERATIONS; iteration++) {
		for (int i = 0; i < springs.size(); i++) {
			for (int j = 0; j < springs.at(i).size(); j++) {
				spring = &springs.at(i).at(j);
				p0 = spring->p0;
				p1 = spring->p1;

				vCurrentDistance = p0->position - p1->position;
				deltaDistance = magnitude(vCurrentDistance);

				// Applying constraints to spring length under compression or tension
				vConstraints = vCurrentDistance * (1.0f - spring->restLength / deltaDistance);
				vConstraints = vConstraints * 0.5f;

				if (!p0->pinned) {
					p0->position = p0->position - vConstraints;
				}

				if (!p1->pinned) {
					p1->position = p1->position + vConstraints;
				}
			}
		}
	}
}

// Accumulates forces on each particle and stores acceleration
void ClothSheet::accumulateForces() {
	//Applying wind force
	vec3 vWindNormal = normalize(vWindForce);
	vec3 vWindAcceleration;

	Particle *v0;
	Particle *v1;
	Particle *v2;
	vec3 vFaceNormal;

	for (int k = 0; k < particles.size() - 1; k++) {
		for (int l = 0; l < particles.at(k).size() - 1; l++) {
			// Finding upper tri normal for wind force acceleration
			v0 = &particles.at(k + 1).at(l);
			v1 = &particles.at(k).at(l);
			v2 = &particles.at(k).at(l + 1);

			vFaceNormal = normalize(cross(v1->position - v0->position, v2->position - v0->position));

			vWindAcceleration = vFaceNormal * dot(vFaceNormal, vWindForce);
			vWindAcceleration = vWindAcceleration / (v0->mass + v1->mass + v2->mass);

			v0->acceleration = v0->acceleration + vWindAcceleration;
			v1->acceleration = v1->acceleration + vWindAcceleration;
			v2->acceleration = v2->acceleration + vWindAcceleration;

			// Finding lower tri normal for wind force acceleration
			v1 = v2;
			v2 = &particles.at(k + 1).at(l + 1);

			vFaceNormal = normalize(cross(v1->position - v0->position, v2->position - v0->position));

			vWindAcceleration = vFaceNormal * dot(vFaceNormal, vWindForce);
			vWindAcceleration = vWindAcceleration / (v0->mass + v1->mass + v2->mass);

			v0->acceleration = v0->acceleration + vWindAcceleration;
			v1->acceleration = v1->acceleration + vWindAcceleration;
			v2->acceleration = v2->acceleration + vWindAcceleration;
		}
	}

	// Applying gravity and spring forces
	GLfloat deltaDistance;
	GLfloat currentDistMagnitude;
	vec3 vCurrentDistance;
	vec3 vSpringAcceleration;
	vec3 vConstraints;

	Particle *p0;
	Particle *p1;
	Spring *spring;

	for (int i = 0; i < springs.size(); i++) {
		for (int j = 0; j < springs.at(i).size(); j++) {
			spring = &springs.at(i).at(j);
			p0 = spring->p0;
			p1 = spring->p1;

			vCurrentDistance = p0->position - p1->position;
			currentDistMagnitude = magnitude(vCurrentDistance);
			deltaDistance = currentDistMagnitude - spring->restLength;

			vSpringAcceleration = (vCurrentDistance / currentDistMagnitude) * (springConstK * deltaDistance);
			vSpringAcceleration = vSpringAcceleration / p0->mass;

			p0->acceleration = (gravity / p0->mass) - vSpringAcceleration + p0->acceleration;
			p1->acceleration = (gravity / p1->mass) + vSpringAcceleration + p1->acceleration;
		}
	}
}

////////////////
// class: Wind
/////////////

Wind::Wind(vec3 &windForce) {
	timeBlowing = 0;
	this->windForce = windForce;
	enabled = true;
}

vec3 Wind::generateWindForce(long deltaT) {
	timeBlowing += deltaT;

	// Switching wind direction every 1.2 seconds;
	if (timeBlowing > 1200) {
		timeBlowing = 0;
		windForce = windForce * -1.0f;
	}

    // Returning wind if enabled, 0 vector otherwise
	return (enabled) ? windForce : vec3{ 0.0f, 0.0f, 0.0f };
}

void Wind::toggleWind() {
	enabled = !enabled;
}

//////////////////////
// lib: Vector Maths
//////////////////

GLfloat magnitude(const vec3 &vec) {
	return sqrt((vec.x * vec.x) + (vec.y * vec.y) + (vec.z * vec.z));
}

vec3 normalize(const vec3 &vec) {
	return vec / magnitude(vec);
}

vec3 cross(const vec3 &u, const vec3 &v) {
	return vec3{ (u.y * v.z) - (u.z * v.y), (u.z * v.x) - (u.x * v.z), (u.x * v.y) - (u.y * v.x) };
}

GLfloat dot(const vec3 &u, const vec3 &v) {
	return (u.x * v.x) + (u.y * v.y) + (u.z * v.z);
}

vec3 operator/(const vec3 &vec, const GLfloat &scalar) {
	return vec3{ vec.x / scalar, vec.y / scalar, vec.z / scalar };
}

vec3 operator*(const vec3 &vec, const GLfloat &scalar) {
	return vec3{ vec.x * scalar, vec.y * scalar, vec.z * scalar };
}

vec3 operator+(const vec3 &u, const vec3 &v) {
	return vec3{ u.x + v.x, u.y + v.y, u.z + v.z };
}

vec3 operator-(const vec3 &u, const vec3 &v) {
	return vec3{ u.x - v.x, u.y - v.y, u.z - v.z };
}

bool operator!=(const vec3 &u, const vec3 &v) {
	return (u.x == v.x && u.y == v.y && u.z == v.z) ? true : false;
}

vec4 operator+(const vec4 &u, const vec4 &v) {
	return vec4{ u.x + v.x, u.y + v.y, u.z + v.z, u.w + v.w };
}

