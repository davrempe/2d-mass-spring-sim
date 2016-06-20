#include <gl/glew.h>
#include <gl/glut.h> 
#include <vector>
#include <math.h>
#include <iostream>

#include "LoadProgram.h"

#define BUFFER_OFFSET(offset) ((GLvoid*) offset)

//
// Forward declare helper functions
//

void subVec(float*, float*, float*);
void addVec(float*, float*, float*);
void scaleVec(float*, float*, float);
float lenVec(float*);
float dotVec(float*, float*);

//
// Data structures for simulation
//

// Mass particle data structure.
struct Particle {
	// mass
	float m;
	// position
	float x[3];
	// velocity
	float v[3];
	// force accumulator
	float f[3];
	// whether the particle is allowed to move or not
	bool fixed;

	Particle(float mass, float initPos[3], float initVel[3], bool isFixed) : m(mass), fixed(isFixed) {
		x[0] = initPos[0]; 
		x[1] = initPos[1];
		x[2] = initPos[2];
		if (!fixed) {
			v[0] = initVel[0]; 
			v[1] = initVel[1]; 
			v[2] = initVel[2];
		} else {
			v[0] = 0.0f;
			v[1] = 0.0f; 
			v[2] = 0.0f;
		}
		f[0] = 0.0f; 
		f[1] = 0.0f; 
		f[2] = 0.0f;
	}
};

// Mass particle system system data structure.
struct ParticleSystem {
	// number of particles
	int n;
	// array of particles in system
	std::vector<Particle*> p;
	// current system time
	float t;

	ParticleSystem(std::vector<Particle*> particles) : p(particles){
		n = particles.size();
		t = 0.0f;
	}

	~ParticleSystem() {
		for (int i = 0; i < n; i++) {
			delete p[i];
		}
	}

	// Adds a new particle to the system
	void addParticle(Particle *newParticle) {
		p.push_back(newParticle);
		n++;
	}

	// Removes particle most recently added to the system
	void removeEndParticle() {
		p.pop_back();
		n--;
	}
}; 

// Force object for a spring force between two mass particles
struct SpringForce {
	// pointer to particle system affected
	ParticleSystem *ps;
	// index in ps of first particle affected by force
	int p1Ind;
	// index in ps of second particle affected by force
	int p2Ind;
	// spring constant
	float ks;
	// spring damping constant
	float kd;
	// rest length of the spring
	float r;

	SpringForce(ParticleSystem *system, int p1Index, int p2Index, float k_s, float k_d, float restLen) :
		ps(system), p1Ind(p1Index), p2Ind(p2Index), ks(k_s), kd(k_d), r(restLen) {}

	// apply Hooke's Law for a pair of masses
	void apply() {
		std::vector<Particle*> particles = ps->p;
		Particle *p1 = particles[p1Ind];
		Particle *p2 = particles[p2Ind];

		float dx[3];
		float dv[3];
		float f1[3];
		float f2[3];

		// calculate derivative of position and velocity
		subVec(dx, p1->x, p2->x);
		subVec(dv, p1->v, p2->v);

		// calculate leading scalar coefficient
		float coeff = -(ks * (lenVec(dx) - r) + kd * (dotVec(dv, dx) / lenVec(dx)));
		// calculate actual force
		scaleVec(f1, dx, coeff / lenVec(dx));
		// f2 is just negated f1
		scaleVec(f2, f1, -1.0f);

		// update force accumulators
		addVec(p1->f, p1->f, f1);
		addVec(p2->f, p2->f, f2);
	}
};

//
// Data structures for rendering
//

struct ParticleVertexData {
	GLfloat vPos[3];
	GLfloat vCenter[3];
	GLfloat vColor[3];
	GLfloat vRadius;

	// must have default constructor for array declaration
	ParticleVertexData() {
		vPos[0] = 0.0;		vPos[1] = 0.0;		vPos[2] = 0.0;
		vCenter[0] = 0.0;		vCenter[1] = 0.0;		vCenter[2] = 0.0;
		vColor[0] = 0.0;		vColor[1] = 0.0;		vColor[2] = 0.0;
		vRadius = 0.0;
	}

	ParticleVertexData(GLfloat vertPos[3], GLfloat partCenter[3], GLfloat partColor[3], GLfloat radius) : vRadius(radius) {
		vPos[0] = vertPos[0];		vPos[1] = vertPos[1];		vPos[2] = vertPos[2];
		vCenter[0] = partCenter[0];		vCenter[1] = partCenter[1];		vCenter[2] = partCenter[2];
		vColor[0] = partColor[0];		vColor[1] = partColor[1];		vColor[2] = partColor[2];
	}
};

struct SpringVertexData {
	GLfloat vPos[3];
	GLfloat vColor[3];

	SpringVertexData() {
		vPos[0] = 0.0;		vPos[1] = 0.0;		vPos[2] = 0.0;
		vColor[0] = 0.0;		vColor[1] = 0.0;		vColor[2] = 0.0;
	}

	SpringVertexData(GLfloat vertPos[3], GLfloat vertColor[3]) {
		vPos[0] = vertPos[0];		vPos[1] = vertPos[1];		vPos[2] = vertPos[2];
		vColor[0] = vertColor[0];		vColor[1] = vertColor[1];		vColor[2] = vertColor[2];
	}
};

//
// Global variables
//

const bool DEBUG = true;
const bool INIT_WITH_STARTING_PARTICLES = false;
const int MAX_PARTICLES = 100;
const int MAX_FORCES = 1000;
const float DEFAULT_KS = 1.5f;
const float DEFAULT_KD = 0.5f;
const float DEFAULT_R = 0.3;

const int VERTEX_LENGTH = 3;
const int PARTICLES_PER_FORCE = 2;
const int VERTICES_PER_QUAD = 4;
const int INDICES_PER_QUAD = 6;
const int BYTES_PER_FLOAT = sizeof(GLfloat);

GLfloat DYNAMIC_PARTICLE_COLOR [] = { 1.0f, 0.3529f, 0.0f };
GLfloat STATIC_PARTICLE_COLOR[] = { 0.9f, 0.2667f, 0.372549f };
GLfloat SPRING_COLOR [] = { 0.956863f, 0.913725f, 0.376471f };
GLfloat BACKGROUND_COLOR[] = {0.223529f, 0.643137f, 0.917647f};

// Particle vertex data
ParticleVertexData particleVertData[MAX_PARTICLES * VERTICES_PER_QUAD];
// Particle quads index data
GLubyte particleIndData[MAX_PARTICLES * INDICES_PER_QUAD]; 
// Spring vertex data
SpringVertexData springVertData[MAX_PARTICLES];
// Spring line index data
GLubyte springIndData[MAX_FORCES * PARTICLES_PER_FORCE];
// current position of the mouse in windows coordinates
int curMousePos[2];
// Buffer for particle quad vertices
GLuint pvBuffer;
// Buffer for particle quad indices
GLuint piBuffer;
// Buffer for spring vertices
GLuint svBuffer;
// Buffer for spring index (line) relationships
GLuint siBuffer;
// Particle shader position attribute
GLuint pvPos;
// Particle shader radius attribute
GLuint pvRadius;
// Particle shader center attribute
GLuint pvCenter;
// Particle shader color attribute
GLuint pvColor;
// Spring shader position attribute
GLuint svPos;
// Spring shader color attribute
GLuint svColor;
// The particle shader program
GLuint pProgram;
// The spring shader program
GLuint sProgram;
// Particle data offsets
GLintptr pCenterOffset, pColorOffset, pRadiusOffset;
// Spring data offsets
GLintptr sColorOffset;

// True if the simulation is running
bool simRunning = false;
// True if user is adding new mass
bool addingMass = false;
// True if user is adding new spring
bool addingSpring = false;
// True if the user is interacting with mass during sim
bool draggingMass = false;
// Time step size
float dt = 0.03;
// The particle system;
ParticleSystem *ps;
// Array of forces to apply on each iteration
std::vector<SpringForce> forces;

// number of frames drawn so far
int frameCount;
// last the time there was a redraw call
int previousTime;
// the current fps
float fps;

//
// Vector helper functions
//

// Subtracts 3-vectors v1 - v2 and place the result in diff.
// Result param may be v1 or v2.
void subVec(float* diff, float* v1, float* v2) {
	diff[0] = v1[0] - v2[0];
	diff[1] = v1[1] - v2[1];
	diff[2] = v1[2] - v2[2];
}

// Adds 3-vectors v1 + v2 and place the result in sum.
// Result param may be v1 or v2.
void addVec(float* sum, float* v1, float* v2) {
	sum[0] = v1[0] + v2[0];
	sum[1] = v1[1] + v2[1];
	sum[2] = v1[2] + v2[2];
}

// Returns the magnitude of the given 3-vector
float lenVec(float* v) {
	return sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

// Returns the dot products of the given 3-vectors v1 and v2
float dotVec(float* v1, float* v2) {
	return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}

// Scale each entry in the 3-vector by the given scale an place in vScaled
void scaleVec(float* vScaled, float* v, float scale) {
	vScaled[0] = scale*v[0];
	vScaled[1] = scale*v[1];
	vScaled[2] = scale*v[2];
}

//
// Solver functions
//

// Returns the radius of a particle based on its mass
float getParticleRadius(Particle *p) {
	// mass of 5.0 will give maximum radius of 0.2
	float radius = p->m / 25.0f;
	if (radius > 0.2) {
		radius = 0.2f;
	}
	return radius;
}

// Clear force accumulators for all particles in the particle system.
void clearForces() {
	for (int i = 0; i < ps->n; i++) {
		scaleVec(ps->p[i]->f, ps->p[i]->f, 0.0f);
	}
}

// Calculate the forces on all particles and update their force accumulators.
void calcForces() {
	for (int i = 0; i < forces.size(); i++) {
		forces[i].apply();
	}
}

// Update the position of all particles based on current velocity
void updatePosition() {
	for (int i = 0; i < ps->n; i++) {
		// only update if not a fixed particle
		if (!ps->p[i]->fixed) {
			float temp[3];
			// dx/dt = v --- euler step
			scaleVec(temp, ps->p[i]->v, dt);
			// add change to position
			addVec(ps->p[i]->x, ps->p[i]->x, temp);
		}
	}
}

// Update the velocity of all particles based on current force and mass
void updateVelocity() {
	for (int i = 0; i < ps->n; i++) {
		// only update if not a fixed particle
		if (!ps->p[i]->fixed) {
			// first calc F/m
			float accel[3];
			scaleVec(accel, ps->p[i]->f, 1.0f / ps->p[i]->m);
			// dv/dt = F/m --- euler step
			float temp[3];
			scaleVec(temp, accel, dt);
			// add change to velocity
			addVec(ps->p[i]->v, ps->p[i]->v, temp);
		}
	}
}

// Update the velocities of two colliding, moving particles
void calculateMovingCollision(Particle* p1, Particle *p2) {
	// calculate new velocities based on equation at
	// https://en.wikipedia.org/wiki/Elastic_collision#Two-dimensional_collision_with_two_moving_objects
	// first calculate new velocity of p1
	float xDiff[3];
	subVec(xDiff, p1->x, p2->x);
	float dist = lenVec(xDiff);
	float vDiff[3];
	subVec(vDiff, p1->v, p2->v);
	float coeff = ((2 * p2->m) / (p1->m + p2->m)) * (dotVec(vDiff, xDiff) / (dist*dist));
	float toSub[3];
	scaleVec(toSub, xDiff, coeff);
	float v1New[3];
	subVec(v1New, p1->v, toSub);
	// now find new velocity for p2
	subVec(xDiff, p2->x, p1->x);
	subVec(vDiff, p2->v, p1->v);
	dist = lenVec(xDiff);
	coeff = ((2 * p1->m) / (p1->m + p2->m)) * (dotVec(vDiff, xDiff) / (dist*dist));
	scaleVec(toSub, xDiff, coeff);
	float v2New[3];
	subVec(v2New, p2->v, toSub);
	// update velocities if not a fixed particle
	p1->v[0] = v1New[0];	p1->v[1] = v1New[1];	p1->v[2] = v1New[2];
	p2->v[0] = v2New[0];	p2->v[1] = v2New[1];	p2->v[2] = v2New[2];
}

// Update the velocities of two colliding particles, the first fixed
// and the second moving
void calculateFixedCollision(Particle *pFixed, Particle *pMoving) {
	// find unit normal vector
	float normal[3];
	subVec(normal, pMoving->x, pFixed->x);
	scaleVec(normal, normal, 1.0f / lenVec(normal));
	// find unit tangential vector (the way it is calculated depends on it being 2D, NOT 3D)
	float tangent[3] = { -normal[1], normal[0], 0.0f };
	// now decompose moving velocity with respect to the normal and tangential and reverse
	// normal component
	float normalProjection = dotVec(pMoving->v, normal);
	float tangentProjection = dotVec(pMoving->v, tangent);
	float vNorm[3];
	float vTan[3];
	scaleVec(vNorm, normal, -normalProjection);
	scaleVec(vTan, tangent, tangentProjection);
	// set new velocity
	float newVel[3];
	addVec(newVel, vTan, vNorm);
	pMoving->v[0] = newVel[0];
	pMoving->v[1] = newVel[1];
	pMoving->v[2] = newVel[2];
}

// Detects any particle-particle collisions and updates
// both velocities respectively
void calculateCollisions() {
	// For each particle go through rest of list and check if colliding
	// If there is a dummy particle for user interaction, we don't want to calculate for that
	int n = ps->n;
	if (draggingMass) {
		n--;
	}
	for (int i = 0; i < n; i++) {
		Particle *p1 = ps->p[i];
		for (int j = i + 1; j < n; j++) {
			Particle *p2 = ps->p[j];

			// first check if colliding -
			// this is the case when the distance between their
			// centers is less than the sum of their radii
			float xDiff[3];
			subVec(xDiff, p1->x, p2->x);
			float dist = lenVec(xDiff);
			if (dist <= getParticleRadius(p1) + getParticleRadius(p2)) {
				// colliding
				// must handle moving and fixed collision differently
				if (p1->fixed) {
					calculateFixedCollision(p1, p2);
				} else if (p2->fixed) {
					calculateFixedCollision(p2, p1);
				} else { // both moving
					calculateMovingCollision(p1, p2);
				}
			}
		}
	}
}

// Updates the entire particle system for the current iteration via Euler's method.
void eulerStep() {
	clearForces();
	calcForces();
	updatePosition();
	updateVelocity();
	calculateCollisions();
	// update time
	ps->t += dt;
}

// Initialize the particle system.
void initParticleSystem() {
	// Create particle system
	std::vector<Particle*> particles;
	ps = new ParticleSystem(particles);

	// Create starting particles IF DESIRED
	if (INIT_WITH_STARTING_PARTICLES) {
		// Particle 1 
		float mass1 = 5.0f;
		float x1[3] = { -0.5f, 0.0f, 0.0f };
		float v1[3] = { -0.3f, 0.0f, 0.0f };
		Particle *p1 = new Particle(mass1, x1, v1, false);
		// Particle 2
		float mass2 = 1.0f;
		float x2[3] = { 0.5f, 0.0f, 0.0f };
		float v2[3] = { 0.0f, 0.0f, 0.0f };
		Particle *p2 = new Particle(mass2, x2, v2, true);
		// Particle 3
		float mass3 = 1.0f;
		float x3[3] = { 0.0f, 0.5f, 0.0f };
		float v3[3] = { 0.0f, 0.0f, 0.0f };
		Particle *p3 = new Particle(mass3, x3, v3, true);

		ps->addParticle(p1);
		ps->addParticle(p2);
		ps->addParticle(p3);

		// Create forces between particles and add to force array
		SpringForce f1(ps, 0, 1, 2.5f, 0.7f, 0.01f);
		SpringForce f2(ps, 0, 2, 2.5f, 1.0f, 0.1f);
		SpringForce f3(ps, 1, 2, 2.5f, 0.4f, 0.1f);
		forces.push_back(f1);
		forces.push_back(f2);
		forces.push_back(f3);
	}
}

// Updates the mass of the particle that is being added to the system
void addMass() {
	if (!simRunning) {
		Particle *addedParticle = ps->p[ps->n - 1];
		addedParticle->m += 0.025f;
	}
}

//
// Rendering-related functions
//

// Inserts the vector vect of length into the array data starting at index given
template <class T>
void insertVector(T* vect, int length, T* data, int index) {
	for (int i = 0; i < length; i++) {
		data[index + i] = vect[i];
	}
}

// transforms the window GLUT coordinate (x, y) to GL coordinates (between -1.0 and 1.0) and places in newcoord array [x y]
void windowToLocal(float newCoord[2], int x, int y) {
	int width = glutGet(GLUT_WINDOW_WIDTH);
	int height = glutGet(GLUT_WINDOW_HEIGHT);
	float xCoord = ((float)2 * x / (float)width) - 1.0;
	float yCoord = ((float)(2 * (height - y)) / (float)height) - 1.0;
	newCoord[0] = xCoord;
	newCoord[1] = yCoord;
}

// Updates the last particle in the particle system to the current position of the mouse
void updateInteraction() {
	// must update dummy mass particle to correct location
	// get mouse in GL coords
	float mousePos[2];
	windowToLocal(mousePos, curMousePos[0], curMousePos[1]);
	// update last "particle" with new coord
	Particle *mousePart = ps->p[ps->n - 1];
	mousePart->x[0] = mousePos[0];
	mousePart->x[1] = mousePos[1];
}

// Goes through particle position data and populates
// particle data and index arrays.
void updateParticleVertexData() {
	// Only go through all particles if no dummy particles in system
	int n;
	if (draggingMass) {
		n = ps->n - 1;
	}
	else {
		n = ps->n;
	}

	for (int i = 0; i < n; i++) {
		Particle *p = ps->p[i];
		GLfloat radius = getParticleRadius(p);
		
		// calculate quad data
		// first add the new vertices
		GLfloat topLeftTrans[3] = { -radius, radius, 0.0 };
		GLfloat bottomLeftTrans[3] = { -radius, -radius, 0.0 };
		GLfloat bottomRightTrans[3] = { radius, -radius, 0.0 };
		GLfloat topRightTrans[3] = { radius, radius, 0.0 };

		GLfloat topLeft[3], bottomLeft[3], bottomRight[3], topRight[3];
		addVec(topLeft, p->x, topLeftTrans);
		addVec(bottomLeft, p->x, bottomLeftTrans);
		addVec(bottomRight, p->x, bottomRightTrans);
		addVec(topRight, p->x, topRightTrans);

		// different color depending on static or dynamic particle
		GLfloat particleColor[3];
		if (p->fixed) {
			particleColor[0] = STATIC_PARTICLE_COLOR[0];
			particleColor[1] = STATIC_PARTICLE_COLOR[1];
			particleColor[2] = STATIC_PARTICLE_COLOR[2];
		}
		else {
			particleColor[0] = DYNAMIC_PARTICLE_COLOR[0];
			particleColor[1] = DYNAMIC_PARTICLE_COLOR[1];
			particleColor[2] = DYNAMIC_PARTICLE_COLOR[2];
		}

		// Create ParticleVertexData objects and add to vertex array
		ParticleVertexData pv0(topLeft, p->x, particleColor, radius);
		ParticleVertexData pv1(bottomLeft, p->x, particleColor, radius);
		ParticleVertexData pv2(bottomRight, p->x, particleColor, radius);
		ParticleVertexData pv3(topRight, p->x, particleColor, radius);

		GLubyte topLeftInd = i * VERTICES_PER_QUAD;
		GLubyte bottomLeftInd = i * VERTICES_PER_QUAD + 1;
		GLubyte bottomRightInd = i * VERTICES_PER_QUAD + 2;
		GLubyte topRightInd = i * VERTICES_PER_QUAD + 3;

		particleVertData[topLeftInd] = pv0;
		particleVertData[bottomLeftInd] = pv1;
		particleVertData[bottomRightInd] = pv2;
		particleVertData[topRightInd] = pv3;

		// Now add to index array
		GLubyte topLeftTri[3] = { topLeftInd, bottomLeftInd, topRightInd };
		GLubyte bottomRightTri[3] = { bottomLeftInd, bottomRightInd, topRightInd };
		insertVector<GLubyte>(topLeftTri, 3, particleIndData, i * INDICES_PER_QUAD);
		insertVector<GLubyte>(bottomRightTri, 3, particleIndData, i * INDICES_PER_QUAD + 3);
	}


}

// Goes through particle position data and populates spring
// vertex and index data arrays
void updateSpringVertexData() {
	// want to leave out any dummy particles and forces
	int leaveOut = 0;
	if (draggingMass) leaveOut++;
	for (int i = 0; i < ps->n - leaveOut; i++) {
		Particle *p = ps->p[i];
		// create SpringVertexData object and add to array
		SpringVertexData sv(p->x, SPRING_COLOR);
		springVertData[i] = sv;
	}

	// Go through forces and add to index array
	for (int i = 0; i < forces.size() - leaveOut; i++) {
		springIndData[i*PARTICLES_PER_FORCE] = forces[i].p1Ind;
		springIndData[i*PARTICLES_PER_FORCE + 1] = forces[i].p2Ind;
	}
}

// Updates the vertex and index buffers based on particles.
void updateBufferData() {
	// update data arrays
	updateParticleVertexData();
	updateSpringVertexData();

	// update buffers
	glBindBuffer(GL_ARRAY_BUFFER, pvBuffer);
	glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(particleVertData), particleVertData);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, piBuffer);
	glBufferSubData(GL_ELEMENT_ARRAY_BUFFER, 0, sizeof(particleIndData), particleIndData);

	glBindBuffer(GL_ARRAY_BUFFER, svBuffer);
	glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(springVertData), springVertData);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, siBuffer);
	glBufferSubData(GL_ELEMENT_ARRAY_BUFFER, 0, sizeof(springIndData), springIndData);
}

// Initializes the rendering system. 
void initGL() {

	// enable blending
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);


	// populate initial particle vertex data
	updateParticleVertexData();
	// populate initial spring vertex data
	updateSpringVertexData();

	// Calculate offsets
	pCenterOffset = BYTES_PER_FLOAT * 3;
	pColorOffset = pCenterOffset + BYTES_PER_FLOAT * 3;
	pRadiusOffset = pColorOffset + BYTES_PER_FLOAT * 3;
	sColorOffset = BYTES_PER_FLOAT * 3;

	// Create the buffers
	GLuint buffs[4];
	glGenBuffers(4, buffs);
	pvBuffer = buffs[0];
	piBuffer = buffs[1];
	svBuffer = buffs[2];
	siBuffer = buffs[3];

	// particle vertex buffer
	glBindBuffer(GL_ARRAY_BUFFER, pvBuffer);
	glBufferData(GL_ARRAY_BUFFER, sizeof(particleVertData), particleVertData, GL_DYNAMIC_DRAW);
	// particle index buffer
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, piBuffer);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(particleIndData), particleIndData, GL_DYNAMIC_DRAW);
	// spring vertex buffer
	glBindBuffer(GL_ARRAY_BUFFER, svBuffer);
	glBufferData(GL_ARRAY_BUFFER, sizeof(springVertData), springVertData, GL_DYNAMIC_DRAW);
	// spring index buffer
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, siBuffer);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(springIndData), springIndData, GL_DYNAMIC_DRAW);

	// create shaders
	const char* pvShadeFile = "particle_vert.glsl";
	const char* pfShadeFile = "particle_frag.glsl";
	const char* svShadeFile = "spring_vert.glsl";
	const char* sfShadeFile = "spring_frag.glsl";

	// compile and link shaders to programs
	pProgram = LoadProgram(pvShadeFile, pfShadeFile);
	pvPos = glGetAttribLocation(pProgram, "vPos");
	pvCenter = glGetAttribLocation(pProgram, "vCenter");
	pvRadius = glGetAttribLocation(pProgram, "vRadius");
	pvColor = glGetAttribLocation(pProgram, "vColor");

	sProgram = LoadProgram(svShadeFile, sfShadeFile);
	svPos = glGetAttribLocation(sProgram, "vPos");
	svColor = glGetAttribLocation(sProgram, "vColor");

	glClearColor(BACKGROUND_COLOR[0], BACKGROUND_COLOR[1], BACKGROUND_COLOR[2], 1.0);
}

// called at certain time increments to redraw and calculate fps
void timer(int id) {
	frameCount++;
	int currentTime = glutGet(GLUT_ELAPSED_TIME);
	int timeInterval = currentTime - previousTime;
	if (timeInterval > 1000)
	{
		fps = frameCount / (timeInterval / 1000.0f);
		previousTime = currentTime;
		frameCount = 0;
	}
	glutPostRedisplay();
}

// Display function called whenever glut determines a redrawing is needed
void display()
{
	// only take a time step if the simulation should be running
	if (simRunning) {
		// Update system to react to live interaction
		if (draggingMass) {
			updateInteraction();
		}
		// Update the spring-mass system
		eulerStep();
	}
	// need to grow mass if user is adding particle
	if (addingMass) {
		addMass();
	}

	// Updates the buffers based on new data
	updateBufferData();

	glClear(GL_COLOR_BUFFER_BIT);

	// draw spring connections
	glUseProgram(sProgram);
	glBindBuffer(GL_ARRAY_BUFFER, svBuffer);
	glVertexAttribPointer(svPos, 3, GL_FLOAT, GL_FALSE, sizeof(SpringVertexData), BUFFER_OFFSET(0));
	glVertexAttribPointer(svColor, 3, GL_FLOAT, GL_FALSE, sizeof(SpringVertexData), BUFFER_OFFSET(sColorOffset));
	glEnableVertexAttribArray(svPos);
	glEnableVertexAttribArray(svColor);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, siBuffer);
	glDrawElements(GL_LINES, forces.size() * PARTICLES_PER_FORCE, GL_UNSIGNED_BYTE, BUFFER_OFFSET(0));

	// draw the masses
	glUseProgram(pProgram);

	glBindBuffer(GL_ARRAY_BUFFER, pvBuffer);
	glVertexAttribPointer(pvPos, 3, GL_FLOAT, GL_FALSE, sizeof(ParticleVertexData), BUFFER_OFFSET(0));
	glVertexAttribPointer(pvCenter, 3, GL_FLOAT, GL_FALSE, sizeof(ParticleVertexData), BUFFER_OFFSET(pCenterOffset));
	glVertexAttribPointer(pvColor, 3, GL_FLOAT, GL_FALSE, sizeof(ParticleVertexData), BUFFER_OFFSET(pColorOffset));
	glVertexAttribPointer(pvRadius, 1, GL_FLOAT, GL_FALSE, sizeof(ParticleVertexData), BUFFER_OFFSET(pRadiusOffset));
	glEnableVertexAttribArray(pvPos);
	glEnableVertexAttribArray(pvCenter);
	glEnableVertexAttribArray(pvColor);
	glEnableVertexAttribArray(pvRadius);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, piBuffer);
	glDrawElements(GL_TRIANGLES, ps->n * INDICES_PER_QUAD, GL_UNSIGNED_BYTE, BUFFER_OFFSET(0));

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

	glutSwapBuffers();

	if (DEBUG) { 
		std::cout << "SIM TIME: " << ps->t << std::endl;
		std::cout << "FPS: " << fps << std::endl;
	}

	// set redisplay at 30 fps
	glutTimerFunc(33.3, timer, 0);
}

// handle keyboard input
void key(unsigned char key, int x, int y)
{
	switch (key) {
		case GLUT_KEY_RIGHT: // right arrow key
			simRunning = true;
			break;
		case GLUT_KEY_LEFT: // left arrow key
			simRunning = false;
			break;
		case 033: // escape key
			exit(EXIT_SUCCESS);
			break;
	}

	glutPostRedisplay();
}

// handle special keys input
void specialKey(int key, int x, int y) {
	switch (key) {
	case GLUT_KEY_RIGHT: // right arrow key
		simRunning = true;
		break;
	case GLUT_KEY_LEFT: // left arrow key
		simRunning = false;
		break;
	}
}

// handle mouse input
void mouse(int button, int state, int x, int y) {
	// translate x, y to GL coordinates
	float glCoord[2];
	windowToLocal(glCoord, x, y);
	float clickCoord[] = { glCoord[0], glCoord[1], 0.0f };
	// check if event is within a current particle
	bool inParticle = false;
	int clickedParticleInd = -1;
	for (int i = 0; i < ps->n; i++) {
		float radius = getParticleRadius(ps->p[i]);
		float diff[3];
		subVec(diff, clickCoord, ps->p[i]->x);
		if (abs(diff[0]) <= radius && abs(diff[1]) <= radius) {
			// if adding new spring, want to make sure the clicked particle is not the
			// dummy one anchored to the mouse
			if (addingSpring && (i == (ps->n - 1))) {
				break;
			}
			else {
				inParticle = true;
				clickedParticleInd = i;
				break; // just use first found particle
			}
		}
	}

	// only add new masses or springs if simulation NOT running
	if (!simRunning) {
		if (inParticle && !addingMass) {
			// dealing with new spring
			switch (button) {
			case GLUT_LEFT_BUTTON: // left mouse
				if (state == GLUT_DOWN) {
					addingSpring = true;
					// create temp particle at mouse position
					float newMass = 0.0f;
					float newX[3] = { clickCoord[0], clickCoord[1], 0.0f };
					float newV[3] = { 0.0f, 0.0f, 0.0f };
					Particle *newP = new Particle(newMass, newX, newV, false);
					ps->addParticle(newP);
					// now attach that particle to the clicked on particle with a force
					SpringForce newForce(ps, clickedParticleInd, ps->n - 1, DEFAULT_KS, DEFAULT_KD, DEFAULT_R);
					forces.push_back(newForce);
				}
				else {
					addingSpring = false;
					// update force to be between clicked particle
					forces[forces.size() - 1].p2Ind = clickedParticleInd;
					// delete dummy mouse particle from system
					ps->removeEndParticle();
				}
				break;
			} // end switch

		}
		else if (addingSpring) {
			// They tried to make a spring but missed a second particle
			addingSpring = false;
			// remove last force from system
			forces.pop_back();
			// delete dummy mouse particle from system
			ps->removeEndParticle();
		}
		else {
			// dealing with new particle
			switch (button) {
			case GLUT_LEFT_BUTTON: // left mouse
				if (state == GLUT_DOWN) {
					addingMass = true;
					float newMass = 0.0f;
					float newX[3] = { clickCoord[0], clickCoord[1], 0.0f };
					float newV[3] = { 0.0f, 0.0f, 0.0f };
					Particle *newP = new Particle(newMass, newX, newV, false);
					ps->addParticle(newP);
				}
				else {
					addingMass = false;
				}
				break;
			case GLUT_RIGHT_BUTTON: // right mouse
				if (state == GLUT_DOWN) {
					addingMass = true;
					float newMass = 0.0f;
					float newX[3] = { clickCoord[0], clickCoord[1], 0.0f };
					float newV[3] = { 0.0f, 0.0f, 0.0f };
					Particle *newP = new Particle(newMass, newX, newV, true);
					ps->addParticle(newP);
				}
				else {
					addingMass = false;
				}
				break;
			} // end switch
		} // end if in particle
	} // end if sim running
	else {
		// handle interaction with the live simulation
		switch (button) {
		case GLUT_LEFT_BUTTON: // left mouse
			if (state == GLUT_DOWN) {
				// check if they've clicked in a dynamic particle
				if (inParticle && !ps->p[clickedParticleInd]->fixed) {
					draggingMass = true;
					// create dummy particle at mouse point
					float newMass = 1.0f;
					float newX[3] = { clickCoord[0], clickCoord[1], 0.0f };
					float newV[3] = { 0.0f, 0.0f, 0.0f };
					Particle *newP = new Particle(newMass, newX, newV, false);
					ps->addParticle(newP);
					// create force between dummy particle and particle to drag, very small rest length to increase control
					SpringForce newForce(ps, clickedParticleInd, ps->n - 1, DEFAULT_KS, DEFAULT_KD, 0.01);
					forces.push_back(newForce);
				}
				
			}
			else {
				// only need to do anything if successfully interacting currently
				if (draggingMass) {
					draggingMass = false;
					// remove last force from system
					forces.pop_back();
					// delete dummy mouse particle from system
					ps->removeEndParticle();
				}
			}
			break;
		} // end switch
	} // end else
	
}

// track the current mouse position on all movement of mouse
void trackMousePos(int x, int y) {
	curMousePos[0] = x;
	curMousePos[1] = y;
}

// handle mouse drag motion
void drag(int x, int y) {
	// update current mouse position
	trackMousePos(x, y);
	if (addingSpring) {
		// must update dummy mass particle to correct location
		// get mouse in GL coords
		float mousePos[2];
		windowToLocal(mousePos, x, y);
		// update last "particle" with new coord
		Particle *mousePart = ps->p[ps->n - 1];
		mousePart->x[0] = mousePos[0];
		mousePart->x[1] = mousePos[1];
	}
}

// handle window resize
void reshape(int width, int height) {
	glViewport(0, 0, width, height);
}

// free all resources
void cleanup() {
	delete ps;
}

//
// MAIN METHOD
//
int main(int argc, char** argv)
{
	glutInit(&argc, argv);        
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
	glutInitWindowSize(720, 720);
	glutCreateWindow("2D Mass-Spring Simulation");

	glewInit();

	initParticleSystem();
	initGL();

	glutDisplayFunc(display);
	glutReshapeFunc(reshape);
	glutKeyboardFunc(key);
	glutSpecialFunc(specialKey);
	glutMouseFunc(mouse);
	glutMotionFunc(drag);
	glutPassiveMotionFunc(trackMousePos);

	glutMainLoop();

	cleanup();

	return 0;
}



