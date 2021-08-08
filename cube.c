#include "GL/freeglut.h" 
#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <signal.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include "I2CWrapper.h"
#include "mpu6050.h"
#include  <math.h>

float X_AXIS = 0.0, Y_AXIS = 0.0, Z_AXIS = 0.0; 
float DIRECTION = 1;

GForceStruct   Data;
const unsigned long long nano = 1000000000;


GLfloat vertices[][3] = {
    { -0.3,  0.3, -0.3 }, 
    { -0.3,  0.3,  0.3 }, 
    {  0.3,  0.3,  0.3 }, 
    {  0.3,  0.3, -0.3 }, 
    { -0.3, -0.3, -0.3 }, 
    { -0.3, -0.3,  0.3 }, 
    {  0.3, -0.3,  0.3 }, 
    {  0.3, -0.3, -0.3 }
};
 
GLfloat colors[][3] = {
    { 0.0, 0.0, 1.0 },      // blue  
    { 1.0, 1.0, 0.0 },      // yellow  
    { 0.0, 1.0, 0.0 },      // green  
    { 1.0, 0.0, 0.0 },      // red
    { 0.5, 0.5, 0.0 },      // gold
    { 0.0, 1.0, 1.0 }        // magenta  
};

//unsigned long t1 = 0, t2 = 0, delay = 0;
float dt = 0;

int i2c_handle;

const BUS = 1;
int I2C_Current_Slave_Adress=0x68;
float accel_dynamic_range = 2.0;
float gyro_dynamic_range = 250.0;
float SIGNIFICANCE_FACTOR = 0.95;
float PI = 3.141592;
float curr_gyrox, curr_gyroy, curr_gyroz; 
float gyrox_off, gyroy_off, gyroz_off;
float anglex, angley, anglez;
double start, end;
float before_anglex, before_angley, before_anglez;
float rot_x, rot_y, rot_z, gy_anglex, gy_angley, gy_anglez;
float velox=0.0, veloy=0.0, veloz=0.0; 
float dis_x, dis_y, dis_z;
float dis_accelx, dis_accely, dis_accelz;
float tran_x, tran_y, tran_z;
float before_dis_x, before_dis_y, before_dis_z;
const float  AccFactor=16.0 /32768.0;
const float  GyroFactor=500.0 / 32768.0;

int ExitFlag=0;

float* ziro()
{
	static float arr[3];
	i2c_handle = I2CWrapperOpen(BUS,I2C_Current_Slave_Adress);
        Setup_MPU6050(i2c_handle);
   	Get_Accel_Values(i2c_handle,&Data);
  	start = (double)clock() / CLOCKS_PER_SEC; 

	int16_t accelx_raw = Data.Gx;
	int16_t accely_raw = Data.Gy;
	int16_t accelz_raw = Data.Gz;

	int16_t temp_raw = Data.Temperature;
	int16_t gyrox_raw = Data.Gyrox;
	int16_t gyroy_raw = Data.Gyroy;
	int16_t gyroz_raw = Data.Gyroz;

	anglex = before_anglex;
	angley = before_angley;
	anglez = before_anglez;

	float accelx = (((float) ((float) accelx_raw)/((float) 16384.0)));
	float accely = (((float) ((float) accely_raw)/((float) 16384.0)));
	float accelz = (((float) ((float) accelz_raw)/((float) 16384.0)));
	float temp = ((float) temp_raw)/340.0 + 36.53;

	float gyrox = (((float) ((float) gyrox_raw)/((float) 131.0)));
	float gyroy = (((float) ((float) gyroy_raw)/((float) 131.0)));
	float gyroz = (((float) ((float) gyroz_raw)/((float) 131.0)));

	gy_anglex += (gyrox)*((float) dt/1000000.0);
	gy_angley += (gyroy)*((float) dt/1000000.0);
	gy_anglez += (gyroz)*((float) dt/1000000.0);

	float xAcc_ang = atan2(accelx, accelz)*180/PI;
	float yAcc_ang = atan2(accely, accelz)*180/PI;
	float zAcc_ang = atan2(accelx, accelz)*180/PI;

	rot_x = atanf(accely / sqrt(accelx*accelx + accelz*accelz))*180/PI;
	rot_y = -atanf(accelx/ sqrt(accely*accely + accelz*accelz))*180/PI;

	anglex = (anglex*SIGNIFICANCE_FACTOR) + (1-SIGNIFICANCE_FACTOR)*rot_x;
	angley = (angley*SIGNIFICANCE_FACTOR) + (1-SIGNIFICANCE_FACTOR)*rot_y;
	anglez = (anglez+gy_anglez)*SIGNIFICANCE_FACTOR + (1-SIGNIFICANCE_FACTOR)*rot_z;

	before_anglex = anglex;
	before_angley = angley;
	before_anglez = anglez;

	float angles360x = anglex;
	float angles360y = angley;
	float angles360z = anglez;

	while (angles360x > 360.0) {
		angles360x -= 360.0;
	}
	while (angles360x < 0) {
		angles360x += 360.0;
	}
	while (angles360y > 360.0) {
		angles360y -= 360.0;
	}
	while (angles360y < 0) {
		angles360y += 360.0;
	}
	while (angles360z > 360.0) {
		angles360z -= 360.0;
	}
	while (angles360z < 0) {
		angles360z += 360.0;
	}

	arr[0] = angles360x;
	arr[1] = angles360y;
	arr[2] = angles360z;
	
	end = (((double)clock()) / CLOCKS_PER_SEC);
	dt = (end - start);

	return arr;
}

void drawBitmapText(char *str, float x, float y, float z)
{
    glRasterPos3f(x, y, z); 
    while(*str)
    {
        glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_24, *str);
        str++;
    }
}

void draw_line()
{
    float legth = 0.5;

    glPushMatrix(); //X축 붉은색
    glColor3f(1.0, 0.0, 0.0);
    glBegin(GL_LINES);
    glVertex3f(legth, 0.0, 0.0);
    glVertex3f(-legth, 0.0, 0.0);
    glEnd();
    drawBitmapText("-Z", legth-0.05, 0.0, 0.0);
    drawBitmapText("+Z", -legth, 0.0, 0.0);
    glPopMatrix();
 
    glPushMatrix(); //+X축 원불
    glColor3f(1.0, 0.0, 0.0);
    glTranslatef(legth, 0.0, 0.0);
    glRotatef(90,0.0, legth, 0.0);
    glutSolidCone(0.01,0.03,100,100);
    glEnd();
    glPopMatrix();

    glPushMatrix(); //+X축 원불
    glColor3f(1.0, 0.0, 0.0);
    glTranslatef(-legth, 0.0, 0.0);
    glRotatef(-90,0.0, legth, 0.0);
    glutSolidCone(0.01,0.03,100,100);
    glEnd();
    glPopMatrix();

    glPushMatrix(); //Y축 녹색
    glColor3f(0.0, 1.0, 0.0);
    glBegin(GL_LINES);
    glVertex3f(0.0, legth, 0.0);
    glVertex3f(0.0, -legth, 0.0);
    glEnd();
    drawBitmapText("-X", 0.0, legth-0.05, 0.0);
    drawBitmapText("+X", 0.0, -legth, 0.0);
    glPopMatrix();
 
    glPushMatrix(); //+Y축 원불
    glColor3f(0.0, 1.0, 0.0);
    glTranslatef(0.0, legth, 0.0);
    glRotatef(-90,legth, 0.0, 0.0);
    glutSolidCone(0.01,0.03,100,100);
    glEnd();
    glPopMatrix();

    glPushMatrix(); //-Y축 원불
    glColor3f(0.0, 1.0, 0.0);
    glTranslatef(0.0, -legth, 0.0);
    glRotatef(270,-legth, 0.0, 0.0);
    glutSolidCone(0.01,0.03,100,100);
    glEnd();
    glPopMatrix();

    glPushMatrix(); //Z축 파란색
    glColor3f(0.0, 0.0, 1.0);
    glBegin(GL_LINES);
    glVertex3f(0.0, 0.0, legth);
    glVertex3f(0.0, 0.0, -legth);
    glEnd();
    drawBitmapText("-Y", 0.0, 0.0, legth-0.05);
    drawBitmapText("+Y", 0.0, 0.0, -legth);
    glPopMatrix();
 
    glPushMatrix(); //+Z축 원불
    glColor3f(0.0, 0.0, 1.0);
    glTranslatef(0.0, 0.0, legth);
    glRotatef(90, 0.0, 0.0, legth);
    glutSolidCone(0.01,0.03,100,100);
    glEnd();
    glPopMatrix();

    glPushMatrix(); //-Z축 원불
    glColor3f(0.0, 0.0, 1.0);
    glTranslatef(0.0, 0.0, -legth);
    glRotatef(180, legth, 0.0, 0.0);
    glutSolidCone(0.01,0.03,100,100);
    glEnd();
    glPopMatrix();
 
    glFlush();
}


void display(){

    float* ptr;
    ptr = ziro();

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();

    glTranslatef(0,0,0);

    glRotatef(X_AXIS, 1.0, 0.0, 0.0);
    glRotatef(Y_AXIS, 0.0, 1.0, 0.0);
    glRotatef(Z_AXIS, 0.0, 0.0, 1.0);
 
    glBegin(GL_QUADS);
 
    glColor3fv(colors[0]);
    glVertex3fv(vertices[0]);
    glVertex3fv(vertices[3]);
    glVertex3fv(vertices[2]);
    glVertex3fv(vertices[1]);
 
    glColor3fv(colors[1]);
    glVertex3fv(vertices[1]);
    glVertex3fv(vertices[2]);
    glVertex3fv(vertices[6]);
    glVertex3fv(vertices[5]);
 
    glColor3fv(colors[2]);
    glVertex3fv(vertices[3]);
    glVertex3fv(vertices[7]);
    glVertex3fv(vertices[6]);
    glVertex3fv(vertices[2]);
 
    glColor3fv(colors[3]);
    glVertex3fv(vertices[0]);
    glVertex3fv(vertices[3]);
    glVertex3fv(vertices[7]);
    glVertex3fv(vertices[4]);
 
    glColor3fv(colors[4]);
    glVertex3fv(vertices[7]);
    glVertex3fv(vertices[4]);
    glVertex3fv(vertices[5]);
    glVertex3fv(vertices[6]);
 
    glColor3fv(colors[5]);
    glVertex3fv(vertices[4]);
    glVertex3fv(vertices[0]);
    glVertex3fv(vertices[1]);
    glVertex3fv(vertices[5]);

    glEnd();

    X_AXIS = ptr[0];
    Y_AXIS = ptr[1];
    //Z_AXIS = ptr[2];

    draw_line();
    glutSwapBuffers();
}
 
 
void Init(){
 
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);
    glutInitWindowSize(500, 500);
    glutInitWindowPosition(0, 0);
 
    glutCreateWindow("Cube");

    glClearColor(0.0, 0.0, 0.0, 0.0);
    glClearDepth(1.0) ;
    glDepthFunc(GL_LESS);
    glEnable(GL_DEPTH_TEST);
    glShadeModel(GL_SMOOTH);  
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    glMatrixMode(GL_MODELVIEW);
   
}



int main(int argc, char **argv){


    glutInit(&argc, argv);
 
    Init();

    glutDisplayFunc(display);
    glutIdleFunc(display);
    glutMainLoop();
 
    return 0;
}

