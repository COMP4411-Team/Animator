#include "modelerdraw.h"
#include <FL/gl.h>
#include <GL/glu.h>
#include <cstdio>

#include "modelerglobals.h"
#include "ModelHelper.h"
#include "modelerview.h"

// ********************************************************
// Support functions from previous version of modeler
// ********************************************************
void _dump_current_modelview( void )
{
    ModelerDrawState *mds = ModelerDrawState::Instance();
    
    if (mds->m_rayFile == NULL)
    {
        fprintf(stderr, "No .ray file opened for writing, bailing out.\n");
        exit(-1);
    }
    
    GLdouble mv[16];
    glGetDoublev( GL_MODELVIEW_MATRIX, mv );
    fprintf( mds->m_rayFile, 
        "transform(\n    (%f,%f,%f,%f),\n    (%f,%f,%f,%f),\n     (%f,%f,%f,%f),\n    (%f,%f,%f,%f),\n",
        mv[0], mv[4], mv[8], mv[12],
        mv[1], mv[5], mv[9], mv[13],
        mv[2], mv[6], mv[10], mv[14],
        mv[3], mv[7], mv[11], mv[15] );
}

void _dump_current_material( void )
{
    ModelerDrawState *mds = ModelerDrawState::Instance();
    
    if (mds->m_rayFile == NULL)
    {
        fprintf(stderr, "No .ray file opened for writing, bailing out.\n");
        exit(-1);
    }
    
    fprintf( mds->m_rayFile, 
        "material={\n    diffuse=(%f,%f,%f);\n    ambient=(%f,%f,%f);\n}\n",
        mds->m_diffuseColor[0], mds->m_diffuseColor[1], mds->m_diffuseColor[2], 
        mds->m_diffuseColor[0], mds->m_diffuseColor[1], mds->m_diffuseColor[2]);
}

// ****************************************************************************

// Initially assign singleton instance to NULL
ModelerDrawState* ModelerDrawState::m_instance = NULL;

ModelerDrawState::ModelerDrawState() : m_drawMode(NORMAL), m_quality(MEDIUM)
{
    float grey[]  = {.5f, .5f, .5f, 1};
    float white[] = {1,1,1,1};
    float black[] = {0,0,0,1};
    
    memcpy(m_ambientColor, black, 4 * sizeof(float));
    memcpy(m_diffuseColor, grey, 4 * sizeof(float));
    memcpy(m_specularColor, white, 4 * sizeof(float));
    
    m_shininess = 0.5;
    
    m_rayFile = NULL;
}

// CLASS ModelerDrawState METHODS
ModelerDrawState* ModelerDrawState::Instance()
{
    // Return the singleton if it exists, otherwise, create it
    return (m_instance) ? (m_instance) : m_instance = new ModelerDrawState();
}

// ****************************************************************************
// Modeler functions for your use
// ****************************************************************************
// Set the current material properties

void setAmbientColor(float r, float g, float b)
{
    ModelerDrawState *mds = ModelerDrawState::Instance();
    
    mds->m_ambientColor[0] = (GLfloat)r;
    mds->m_ambientColor[1] = (GLfloat)g;
    mds->m_ambientColor[2] = (GLfloat)b;
    mds->m_ambientColor[3] = (GLfloat)1.0;
    
    if (mds->m_drawMode == NORMAL)
        glMaterialfv( GL_FRONT_AND_BACK, GL_AMBIENT, mds->m_ambientColor);
}

void setDiffuseColor(float r, float g, float b)
{
    ModelerDrawState *mds = ModelerDrawState::Instance();
    
    mds->m_diffuseColor[0] = (GLfloat)r;
    mds->m_diffuseColor[1] = (GLfloat)g;
    mds->m_diffuseColor[2] = (GLfloat)b;
    mds->m_diffuseColor[3] = (GLfloat)1.0;
    
    if (mds->m_drawMode == NORMAL)
        glMaterialfv( GL_FRONT_AND_BACK, GL_DIFFUSE, mds->m_diffuseColor);
    else
        glColor3f(r,g,b);
}

void setSpecularColor(float r, float g, float b)
{	
    ModelerDrawState *mds = ModelerDrawState::Instance();
    
    mds->m_specularColor[0] = (GLfloat)r;
    mds->m_specularColor[1] = (GLfloat)g;
    mds->m_specularColor[2] = (GLfloat)b;
    mds->m_specularColor[3] = (GLfloat)1.0;
    
    if (mds->m_drawMode == NORMAL)
        glMaterialfv( GL_FRONT_AND_BACK, GL_SPECULAR, mds->m_specularColor);
}

void setShininess(float s)
{
    ModelerDrawState *mds = ModelerDrawState::Instance();
    
    mds->m_shininess = (GLfloat)s;
    
    if (mds->m_drawMode == NORMAL)
        glMaterialf( GL_FRONT, GL_SHININESS, mds->m_shininess);
}

void setDrawMode(DrawModeSetting_t drawMode)
{
    ModelerDrawState::Instance()->m_drawMode = drawMode;
}

void setQuality(QualitySetting_t quality)
{
    ModelerDrawState::Instance()->m_quality = quality;
}

bool openRayFile(const char rayFileName[])
{
    ModelerDrawState *mds = ModelerDrawState::Instance();

	fprintf(stderr, "Ray file format output is buggy (ehsu)\n");
    
    if (!rayFileName)
        return false;
    
    if (mds->m_rayFile) 
        closeRayFile();
    
    mds->m_rayFile = fopen(rayFileName, "w");
    
    if (mds->m_rayFile != NULL) 
    {
        fprintf( mds->m_rayFile, "SBT-raytracer 1.0\n\n" );
        fprintf( mds->m_rayFile, "camera { fov=30; }\n\n" );
        fprintf( mds->m_rayFile, 
            "directional_light { direction=(-1,-1,-1); color=(0.7,0.7,0.7); }\n\n" );
        return true;
    }
    else
        return false;
}

void _setupOpenGl()
{
    ModelerDrawState *mds = ModelerDrawState::Instance();
	switch (mds->m_drawMode)
	{
	case NORMAL:
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		glShadeModel(GL_SMOOTH);
		break;
	case FLATSHADE:
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		glShadeModel(GL_FLAT);
		break;
	case WIREFRAME:
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		glShadeModel(GL_FLAT);
	default:
		break;
	}

}

void closeRayFile()
{
    ModelerDrawState *mds = ModelerDrawState::Instance();
    
    if (mds->m_rayFile) 
        fclose(mds->m_rayFile);
    
    mds->m_rayFile = NULL;
}

void drawSphere(double r)
{
    ModelerDrawState *mds = ModelerDrawState::Instance();

	_setupOpenGl();
    
    if (mds->m_rayFile)
    {
        _dump_current_modelview();
        fprintf(mds->m_rayFile, "scale(%f,%f,%f,sphere {\n", r, r, r );
        _dump_current_material();
        fprintf(mds->m_rayFile, "}))\n" );
    }
    else
    {
        int divisions; 
        GLUquadricObj* gluq;
        
        switch(mds->m_quality)
        {
        case HIGH: 
            divisions = 32; break;
        case MEDIUM: 
            divisions = 20; break;
        case LOW:
            divisions = 12; break;
        case POOR:
            divisions = 8; break;
        }
        
        gluq = gluNewQuadric();
        gluQuadricDrawStyle( gluq, GLU_FILL );
        gluQuadricTexture( gluq, GL_TRUE );
        gluSphere(gluq, r, divisions, divisions);
        gluDeleteQuadric( gluq );
    }
}


void drawBox( double x, double y, double z )
{
    ModelerDrawState *mds = ModelerDrawState::Instance();

	_setupOpenGl();
    
    if (mds->m_rayFile)
    {
        _dump_current_modelview();
        fprintf(mds->m_rayFile,  
            "scale(%f,%f,%f,translate(0.5,0.5,0.5,box {\n", x, y, z );
        _dump_current_material();
        fprintf(mds->m_rayFile,  "})))\n" );
    }
    else
    {
        /* remember which matrix mode OpenGL was in. */
        int savemode;
        glGetIntegerv( GL_MATRIX_MODE, &savemode );
        
        /* switch to the model matrix and scale by x,y,z. */
        glMatrixMode( GL_MODELVIEW );
        glPushMatrix();
        glScaled( x, y, z );
        
        glBegin( GL_QUADS );
        
        glNormal3d( 0.0, 0.0, -1.0 );
        glVertex3d( 0.0, 0.0, 0.0 ); glVertex3d( 0.0, 1.0, 0.0 );
        glVertex3d( 1.0, 1.0, 0.0 ); glVertex3d( 1.0, 0.0, 0.0 );
        
        glNormal3d( 0.0, -1.0, 0.0 );
        glVertex3d( 0.0, 0.0, 0.0 ); glVertex3d( 1.0, 0.0, 0.0 );
        glVertex3d( 1.0, 0.0, 1.0 ); glVertex3d( 0.0, 0.0, 1.0 );
        
        glNormal3d( -1.0, 0.0, 0.0 );
        glVertex3d( 0.0, 0.0, 0.0 ); glVertex3d( 0.0, 0.0, 1.0 );
        glVertex3d( 0.0, 1.0, 1.0 ); glVertex3d( 0.0, 1.0, 0.0 );
        
        glNormal3d( 0.0, 0.0, 1.0 );
        glVertex3d( 0.0, 0.0, 1.0 ); glVertex3d( 1.0, 0.0, 1.0 );
        glVertex3d( 1.0, 1.0, 1.0 ); glVertex3d( 0.0, 1.0, 1.0 );
        
        glNormal3d( 0.0, 1.0, 0.0 );
        glVertex3d( 0.0, 1.0, 0.0 ); glVertex3d( 0.0, 1.0, 1.0 );
        glVertex3d( 1.0, 1.0, 1.0 ); glVertex3d( 1.0, 1.0, 0.0 );
        
        glNormal3d( 1.0, 0.0, 0.0 );
        glVertex3d( 1.0, 0.0, 0.0 ); glVertex3d( 1.0, 1.0, 0.0 );
        glVertex3d( 1.0, 1.0, 1.0 ); glVertex3d( 1.0, 0.0, 1.0 );
        
        glEnd();
        
        /* restore the model matrix stack, and switch back to the matrix
        mode we were in. */
        glPopMatrix();
        glMatrixMode( savemode );
    }
}

void drawTextureBox( double x, double y, double z )
{
    // NOT IMPLEMENTED, SORRY (ehsu)
}

void drawCylinder( double h, double r1, double r2 )
{
    ModelerDrawState *mds = ModelerDrawState::Instance();
    int divisions;

	_setupOpenGl();
    
    switch(mds->m_quality)
    {
    case HIGH: 
        divisions = 32; break;
    case MEDIUM: 
        divisions = 20; break;
    case LOW:
        divisions = 12; break;
    case POOR:
        divisions = 8; break;
    }
    
    if (mds->m_rayFile)
    {
        _dump_current_modelview();
        fprintf(mds->m_rayFile, 
            "cone { height=%f; bottom_radius=%f; top_radius=%f;\n", h, r1, r2 );
        _dump_current_material();
        fprintf(mds->m_rayFile, "})\n" );
    }
    else
    {
        GLUquadricObj* gluq;
        
        /* GLU will again do the work.  draw the sides of the cylinder. */
        gluq = gluNewQuadric();
        gluQuadricDrawStyle( gluq, GLU_FILL );
        gluQuadricTexture( gluq, GL_TRUE );
        gluCylinder( gluq, r1, r2, h, divisions, divisions);
        gluDeleteQuadric( gluq );
        
        if ( r1 > 0.0 )
        {
        /* if the r1 end does not come to a point, draw a flat disk to
            cover it up. */
            
            gluq = gluNewQuadric();
            gluQuadricDrawStyle( gluq, GLU_FILL );
            gluQuadricTexture( gluq, GL_TRUE );
            gluQuadricOrientation( gluq, GLU_INSIDE );
            gluDisk( gluq, 0.0, r1, divisions, divisions);
            gluDeleteQuadric( gluq );
        }
        
        if ( r2 > 0.0 )
        {
        /* if the r2 end does not come to a point, draw a flat disk to
            cover it up. */
            
            /* save the current matrix mode. */	
            int savemode;
            glGetIntegerv( GL_MATRIX_MODE, &savemode );
            
            /* translate the origin to the other end of the cylinder. */
            glMatrixMode( GL_MODELVIEW );
            glPushMatrix();
            glTranslated( 0.0, 0.0, h );
            
            /* draw a disk centered at the new origin. */
            gluq = gluNewQuadric();
            gluQuadricDrawStyle( gluq, GLU_FILL );
            gluQuadricTexture( gluq, GL_TRUE );
            gluQuadricOrientation( gluq, GLU_OUTSIDE );
            gluDisk( gluq, 0.0, r2, divisions, divisions);
            gluDeleteQuadric( gluq );
            
            /* restore the matrix stack and mode. */
            glPopMatrix();
            glMatrixMode( savemode );
        }
    }
    
}
void drawTriangle( double x1, double y1, double z1,
                   double x2, double y2, double z2,
                   double x3, double y3, double z3 )
{
    ModelerDrawState *mds = ModelerDrawState::Instance();

	_setupOpenGl();

    if (mds->m_rayFile)
    {
        _dump_current_modelview();
        fprintf(mds->m_rayFile, 
            "polymesh { points=((%f,%f,%f),(%f,%f,%f),(%f,%f,%f)); faces=((0,1,2));\n", x1, y1, z1, x2, y2, z2, x3, y3, z3 );
        _dump_current_material();
        fprintf(mds->m_rayFile, "})\n" );
    }
    else
    {
        double a, b, c, d, e, f;
        
        /* the normal to the triangle is the cross product of two of its edges. */
        a = x2-x1;
        b = y2-y1;
        c = z2-z1;
        
        d = x3-x1;
        e = y3-y1;
        f = z3-z1;

        //if (mds->enableCelShading)
        //{
	       // glBegin(GL_TRIANGLES);
	       // glDisable(GL_LIGHTING);

	       // aiVector3D normal(b * f - c * e, c * d - a * f, a * e - b * d);
	       // aiVector3D color(1, 1, 0);

	       // auto shade = celShading(normal, aiVector3D(x1, y1, z1), color);
	       // glColor3f(shade.x, shade.y, shade.z);
	       // glVertex3d(x1, y1, z1);

	       // shade = celShading(normal, aiVector3D(x2, y2, z2), color);
	       // glColor3f(shade.x, shade.y, shade.z);
	       // glVertex3d(x2, y2, z2);

	       // shade = celShading(normal, aiVector3D(x3, y3, z3), color);
	       // glColor3f(shade.x, shade.y, shade.z);
	       // glVertex3d(x3, y3, z3);

	       // glEnd();
	       // glEnable(GL_LIGHTING);

	       // return;
        //}
        
        glBegin( GL_TRIANGLES );
        glNormal3d( b*f - c*e, c*d - a*f, a*e - b*d );
        glVertex3d( x1, y1, z1 );
        glVertex3d( x2, y2, z2 );
        glVertex3d( x3, y3, z3 );
        glEnd();
    }
}

void drawTriangle(Mesh& mesh, const aiFace& face)
{
	ModelerDrawState *mds = ModelerDrawState::Instance();

	_setupOpenGl();

    float x1, x2, x3, y1, y2, y3, z1, z2, z3;

	auto& v1 = mesh.vertices[face.mIndices[0]];
	auto& v2 = mesh.vertices[face.mIndices[1]];
	auto& v3 = mesh.vertices[face.mIndices[2]];

    x1 = v1.world_pos.x;
	x2 = v2.world_pos.x;
	x3 = v3.world_pos.x;

	y1 = v1.world_pos.y;
	y2 = v2.world_pos.y;
	y3 = v3.world_pos.y;

	z1 = v1.world_pos.z;
	z2 = v2.world_pos.z;
	z3 = v3.world_pos.z;

    if (mds->m_rayFile)
    {
        _dump_current_modelview();
        fprintf(mds->m_rayFile, 
            "polymesh { points=((%f,%f,%f),(%f,%f,%f),(%f,%f,%f)); faces=((0,1,2));\n", x1, y1, z1, x2, y2, z2, x3, y3, z3 );
        _dump_current_material();
        fprintf(mds->m_rayFile, "})\n" );
    }
    else
    {
        float a, b, c, d, e, f;
        
        /* the normal to the triangle is the cross product of two of its edges. */
        a = x2-x1;
        b = y2-y1;
        c = z2-z1;
        
        d = x3-x1;
        e = y3-y1;
        f = z3-z1;

    		if (mds->enableCelShading)
    		{
			glDisable(GL_LIGHTING);
			glBegin(GL_TRIANGLES);

    			aiVector3D normal{b*f - c*e, c*d - a*f, a*e - b*d};
    			aiVector3D color{0.87f, 0.72f, 0.53f};

    			auto shade = celShading(normal, {x1, y1, z1}, color);
    			glColor3f(shade.x, shade.y, shade.z);
			glVertex3f(x1, y1, z1);

			shade = celShading(normal, {x2, y2, z2}, color);
    			glColor3f(shade.x, shade.y, shade.z);
			glVertex3f(x2, y2, z2);

			shade = celShading(normal, {x3, y3, z3}, color);
    			glColor3f(shade.x, shade.y, shade.z);
			glVertex3f(x3, y3, z3);
    			
    			glEnd();
			glEnable(GL_LIGHTING);
    			
    			return;
    		}

        glBegin( GL_TRIANGLES );
        glNormal3f( b*f - c*e, c*d - a*f, a*e - b*d );

    		glTexCoord2f(v1.tex_coords.x, v1.tex_coords.y);
        glVertex3f( x1, y1, z1 );
    		glTexCoord2f(v2.tex_coords.x, v2.tex_coords.y);
        glVertex3f( x2, y2, z2 );
    		glTexCoord2f(v3.tex_coords.x, v3.tex_coords.y);
        glVertex3f( x3, y3, z3 );
        glEnd();
    }
}

void drawNurbs(float* control_points, int width, int height)
{
	GLUnurbs* nurbs_renderer = gluNewNurbsRenderer();

    int k = 4;
	float* s_knots = new float[width + k];
	float* t_knots = new float[height + k];

	for (int i = 0; i < width + k; ++i)
        s_knots[i] = i + 1;
	for (int i = 0; i < height + k; ++i)
        t_knots[i] = i + 1;

    gluNurbsProperty(nurbs_renderer, GLU_SAMPLING_TOLERANCE, 25);
	
    gluBeginSurface(nurbs_renderer);
	gluNurbsSurface(nurbs_renderer, width + k, s_knots, height + k, t_knots, 
        height * 3, 3, control_points, k, k, GL_MAP2_VERTEX_3);
    gluEndSurface(nurbs_renderer);
	
	delete [] s_knots;
	delete [] t_knots;
}

void drawTorus(double rl,double rs, double tl, double ts, double x, double y, double z, double rx, double ry, double rz, int flower, int numPetal) {
    ModelerDrawState* mds = ModelerDrawState::Instance();

    _setupOpenGl();

    if (mds->m_rayFile)
    {
        _dump_current_modelview();
        fprintf(mds->m_rayFile, "torus with ring long radius=%f, ring short radius=%f, rube long radius=%f, tube short radius=%f {\n", rl, rs, tl, ts);
        _dump_current_material();
        fprintf(mds->m_rayFile, "}))\n");
    }
    else
    {
        int divisionr, divisiont, divisionp;

        switch (mds->m_quality)
        {
        case HIGH:
            divisionr = 80; divisiont = 40; divisionp = 50; break;
        case MEDIUM:
            divisionr = 60; divisiont = 30; divisionp = 40; break;
        case LOW:
            divisionr = 40; divisiont = 20; divisionp = 30; break;
        case POOR:
            divisionr = 20; divisiont = 10; divisionp = 20; break;
        }

        rx = 2 * M_PI / 360 * rx;
        ry = 2 * M_PI / 360 * ry;
        rz = 2 * M_PI / 360 * rz;
        double ringStep = 2 * M_PI / divisionr;
        double tubeStep = 2 * M_PI / divisiont;
        double flowerStep = 2 * M_PI / numPetal;
        double petalStep = M_PI / divisionp;
        double ringA = 0;//ringAngle
        double tubeA = 0;//TubeAngle
        double petalA = 0;
        double tempx, tempy, tempz;
        double tempx1, tempy1, tempz1;

         /* remember which matrix mode OpenGL was in. */
        int savemode;
        glGetIntegerv(GL_MATRIX_MODE, &savemode);

        /* switch to the model matrix and scale by x,y,z. */
        glMatrixMode(GL_MODELVIEW);
        glPushMatrix();


        glBegin(GL_QUAD_STRIP);
        for (int i = 0; i < divisionr; i++) {
            ringA += ringStep;
            for (int j = 0; j <= divisiont; j++) {
                tubeA += tubeStep;
                double dy = cos(ringA) / (rl * sqrt(sin(ringA) * sin(ringA) / (rs * rs) + cos(ringA) * cos(ringA) / (rl * rl)));
                double dz = sin(ringA) / (rs * sqrt(sin(ringA) * sin(ringA) / (rs * rs) + cos(ringA) * cos(ringA) / (rl * rl)));
                glNormal3f(sin(tubeA)/ts, dy * cos(tubeA)/tl, dz * cos(tubeA)/tl);
                tempx = sin(tubeA) * ts; tempy = cos(ringA) * (rl + tl * cos(tubeA)); tempz = sin(ringA) * (rs + tl * cos(tubeA));
                tempx1 = tempx; tempy1 = tempy * cos(rx) - tempz * sin(rx); tempz1 = tempy * sin(rx) + tempz * cos(rx);//rotate X
                tempy = tempy1; tempx = tempx1 * cos(ry) + tempz1 * sin(ry); tempz = -tempx1 * sin(ry) + tempz1 * cos(ry);//rotate Y
                tempz1 = tempz; tempx1 = tempx * cos(rz) - tempy * sin(rz); tempy1 = tempx * sin(rz) + tempy * cos(rz);//rotate Z
                glVertex3f(tempx1+x, tempy1+y, tempz1+z);

                dy = cos(ringA + ringStep) / (rl * sqrt(sin(ringA + ringStep) * sin(ringA + ringStep) / (rs * rs) + cos(ringA + ringStep) * cos(ringA + ringStep) / (rl * rl)));
                dz = sin(ringA + ringStep) / (rs * sqrt(sin(ringA + ringStep) * sin(ringA + ringStep) / (rs * rs) + cos(ringA + ringStep) * cos(ringA + ringStep) / (rl * rl)));
                glNormal3f(sin(tubeA)/ts, dy* cos(tubeA)/tl, dz * cos(tubeA)/tl);
                tempx = sin(tubeA) * ts; tempy = cos(ringA + ringStep) * (rl + tl * cos(tubeA)); tempz = sin(ringA + ringStep) * (rs + tl * cos(tubeA));
                tempx1 = tempx; tempy1 = tempy * cos(rx) - tempz * sin(rx); tempz1 = tempy * sin(rx) + tempz * cos(rx);//rotate X
                tempy = tempy1; tempx = tempx1 * cos(ry) + tempz1 * sin(ry); tempz = -tempx1 * sin(ry) + tempz1 * cos(ry);//rotate Y
                tempz1 = tempz; tempx1 = tempx * cos(rz) - tempy * sin(rz); tempy1 = tempx * sin(rz) + tempy * cos(rz);//rotate Z
                glVertex3f(tempx1 + x, tempy1 + y, tempz1 + z);
            }
            tubeA = 0;
        }
        ringA = 0;
        if (flower) {
            for (int i = 0; i < numPetal; i++) {
                ringA = i * flowerStep;
                double petaly = rl * (cos(ringA) + cos(ringA + flowerStep)) / 2;
                double petalz = rs * (sin(ringA) + sin(ringA + flowerStep)) / 2;
                double dpy = rl * cos(ringA) - petaly;
                double dpz = rs * sin(ringA) - petalz;
                double petalR = sqrt(dpy * dpy + dpz * dpz);
                double petaliA = atan(dpz / dpy);
                if (dpy < 0) petaliA += M_PI;
                for (int j = 0; j < divisionp; j++) {
                    petalA = petaliA+j * petalStep;
                    for (int k = 0; k <= divisiont; k++) {
                        tubeA = k * tubeStep;
                        glNormal3f(sin(tubeA) / ts, cos(tubeA) / tl * cos(petalA), cos(tubeA) / tl * sin(petalA));
                        tempx = ts * sin(tubeA); tempy = (tl * cos(tubeA) + petalR) * cos(petalA) + petaly; tempz = (tl * cos(tubeA) + petalR) * sin(petalA) + petalz;
                        tempx1 = tempx; tempy1 = tempy * cos(rx) - tempz * sin(rx); tempz1 = tempy * sin(rx) + tempz * cos(rx);//rotate X
                        tempy = tempy1; tempx = tempx1 * cos(ry) + tempz1 * sin(ry); tempz = -tempx1 * sin(ry) + tempz1 * cos(ry);//rotate Y
                        tempz1 = tempz; tempx1 = tempx * cos(rz) - tempy * sin(rz); tempy1 = tempx * sin(rz) + tempy * cos(rz);//rotate Z
                        glVertex3f(tempx1 + x, tempy1 + y, tempz1 + z);

                        glNormal3f(sin(tubeA) / ts, cos(tubeA) / tl * cos(petalA + petalStep), cos(tubeA) / tl * sin(petalA + petalStep));
                        tempx = ts * sin(tubeA); tempy = (tl * cos(tubeA) + petalR) * cos(petalA + petalStep) + petaly; tempz = (tl * cos(tubeA) + petalR) * sin(petalA + petalStep) + petalz;
                        tempx1 = tempx; tempy1 = tempy * cos(rx) - tempz * sin(rx); tempz1 = tempy * sin(rx) + tempz * cos(rx);//rotate X
                        tempy = tempy1; tempx = tempx1 * cos(ry) + tempz1 * sin(ry); tempz = -tempx1 * sin(ry) + tempz1 * cos(ry);//rotate Y
                        tempz1 = tempz; tempx1 = tempx * cos(rz) - tempy * sin(rz); tempy1 = tempx * sin(rz) + tempy * cos(rz);//rotate Z
                        glVertex3f(tempx1 + x, tempy1 + y, tempz1 + z);
                    }
                }
                    
                    
            }
        }

        glEnd();
        glRotatef(30, 0, 1.0, 0);
        glPopMatrix();
        glMatrixMode(savemode);
    }
}

void drawRotation(double x1, double y1, double z1,
    double x2, double y2, double z2,
    double x3, double y3, double z3,
    double x4, double y4, double z4) {
    ModelerDrawState* mds = ModelerDrawState::Instance();

    _setupOpenGl();

    if (mds->m_rayFile)
    {
        _dump_current_modelview();
        fprintf(mds->m_rayFile,
            "BezierCurve { points=((%f,%f,%f),(%f,%f,%f),(%f,%f,%f),(%f,%f,%f); faces=((0,1,2));\n", x1, y1, z1, x2, y2, z2, x3, y3, z3, x4, y4, z4);
        _dump_current_material();
        fprintf(mds->m_rayFile, "})\n");
    }
    else {
        double t, dr;
        switch (mds->m_quality)
        {
        case HIGH:
            t = 80.0; dr = 50; break;
        case MEDIUM:
            t = 60.0; dr = 40; break;
        case LOW:
            t = 40.0; dr = 30; break;
        case POOR:
            t = 20.0; dr = 20; break;
        }

        double rstep = 2 * M_PI / dr;
        double r, tempr;
        double x, y, z;
        double tempx, tempy, tempz;
        glBegin(GL_QUAD_STRIP);

        for (int i = 0; i < t; i++) {
            x = pow(1 - i / t, 3) * x1 + 3 * i / t * pow(1 - i / t, 2) * x2 + 3 * i * i / t / t * (1 - i / t) * x3 + pow(i / t, 3) * x4;
            y = pow(1 - i / t, 3) * y1 + 3 * i / t * pow(1 - i / t, 2) * y2 + 3 * i * i / t / t * (1 - i / t) * y3 + pow(i / t, 3) * y4;
            z = pow(1 - i / t, 3) * z1 + 3 * i / t * pow(1 - i / t, 2) * z2 + 3 * i * i / t / t * (1 - i / t) * z3 + pow(i / t, 3) * z4;
            tempx = pow(1 - (i + 1) / t, 3) * x1 + 3 * (i + 1) / t * pow(1 - (i + 1) / t, 2) * x2 + 3 * (i + 1) * (i + 1) / t / t * (1 - (i + 1) / t) * x3 + pow((i + 1) / t, 3) * x4;
            tempy = pow(1 - (i + 1) / t, 3) * y1 + 3 * (i + 1) / t * pow(1 - (i + 1) / t, 2) * y2 + 3 * (i + 1) * (i + 1) / t / t * (1 - (i + 1) / t) * y3 + pow((i + 1) / t, 3) * y4;
            tempz = pow(1 - (i + 1) / t, 3) * z1 + 3 * (i + 1) / t * pow(1 - (i + 1) / t, 2) * z2 + 3 * (i + 1) * (i + 1) / t / t * (1 - (i + 1) / t) * z3 + pow((i + 1) / t, 3) * z4;
            r = sqrt(y * y + z * z); tempr = sqrt(tempy * tempy + tempz * tempz);
            for (int j = 0; j <= dr; j++) {
                double angle = j * rstep;
                double dx = tempx - x; double dy = tempr * cos(angle) - r * cos(angle); double dz = tempr * sin(angle) - r * sin(angle);
                glNormal3d(dz * sin(angle) + dy * cos(angle), -dx * cos(angle), -sin(angle) * dx);
                glVertex3d(x, r * cos(angle), r * sin(angle));
                glVertex3d(tempx, tempr * cos(angle), tempr * sin(angle));
                //double dx = tempx - x; double dy = tempr * cos(angle) - r * cos(angle); double dz = tempr * sin(angle) - r * sin(angle);
            }
        }
        glEnd();

    }
}

void drawCurve(double x1, double y1, double z1,
    double x2, double y2, double z2,
    double x3, double y3, double z3,
    double x4, double y4, double z4) {
    ModelerDrawState* mds = ModelerDrawState::Instance();

    _setupOpenGl();

    if (mds->m_rayFile)
    {
        _dump_current_modelview();
        fprintf(mds->m_rayFile,
            "BezierCurve { points=((%f,%f,%f),(%f,%f,%f),(%f,%f,%f),(%f,%f,%f); faces=((0,1,2));\n", x1, y1, z1, x2, y2, z2, x3, y3, z3, x4, y4, z4);
        _dump_current_material();
        fprintf(mds->m_rayFile, "})\n");
    }
    else {
        double t;
        switch (mds->m_quality)
        {
        case HIGH:
            t = 80.0; break;
        case MEDIUM:
            t = 60.0; break;
        case LOW:
            t = 40.0; break;
        case POOR:
            t = 20.0; break;
        }
        double x, y, z;
        glBegin(GL_LINE_STRIP);

        for (int i = 0; i <= t; i++) {
            x = pow(1 - i / t, 3) * x1 + 3 * i / t * pow(1 - i / t, 2) * x2 + 3 * i * i / t / t * (1 - i / t) * x3 + pow(i / t, 3) * x4;
            y = pow(1 - i / t, 3) * y1 + 3 * i / t * pow(1 - i / t, 2) * y2 + 3 * i * i / t / t * (1 - i / t) * y3 + pow(i / t, 3) * y4;
            z = pow(1 - i / t, 3) * z1 + 3 * i / t * pow(1 - i / t, 2) * z2 + 3 * i * i / t / t * (1 - i / t) * z3 + pow(i / t, 3) * z4;
            glVertex3d(x, y, z);
        }

        glEnd();
    }

}

void drawSlice(double x1, double y1, double z1,
    double x2, double y2, double z2,
    double x3, double y3, double z3,
    double x4, double y4, double z4) {
    drawTriangle(x1, y1, z1, x2, y2, z2, x3, y3, z3);
    drawTriangle(x2, y2, z2, x3, y3, z3, x4, y4, z4);
}

void drawSphere(float radius, const aiVector3D& position)
{
	glPushMatrix();
	glTranslatef(position.x, position.y, position.z);
	drawSphere(radius);
	glPopMatrix();
}

aiVector3D celShading(const aiVector3D& normal, const aiVector3D& pos, const aiVector3D& color)
{
    aiVector3D lightPosition0{5, 5, 5};
	aiVector3D lightPosition1{-5, -5, -5};

	float celShadingThreshold[4] = {0.02f, 0.1f, 0.6f, 0.9f};
    float celShadingVal[4] = {0.1f, 0.5f, 0.87f, 0.95f};

	auto lightDir = (lightPosition0 - pos).Normalize();
	float lDotN = lightDir * normal;

	aiVector3D color0 = color;

	if (lDotN < celShadingThreshold[0])
        color0 = color * celShadingVal[0];
	else if (lDotN < celShadingThreshold[1])
		color0 = color * celShadingVal[1];
	else if (lDotN < celShadingThreshold[2])
		color0 = color * celShadingVal[2];
	else if (lDotN < celShadingThreshold[3])
		color0 = color * celShadingVal[3];


	lightDir = (lightPosition1 - pos).Normalize();
	lDotN = lightDir * normal;
	aiVector3D color1 = color;

	if (lDotN < celShadingThreshold[0])
        color1 = color * celShadingVal[0];
	else if (lDotN < celShadingThreshold[1])
		color1 = color * celShadingVal[1];
	else if (lDotN < celShadingThreshold[2])
		color1 = color * celShadingVal[2];
	else if (lDotN < celShadingThreshold[3])
		color1 = color * celShadingVal[3];

	return (color0 + color1) * 0.5f;
}
