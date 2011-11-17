#include <GL/gl.h>

#include <stdlib.h>
#include <stdio.h>

#include "glcontext.h"

int main(int argc, char *argv[])
{
    glcontext_t *glc = glcontext_X11_create();
    int width=512, height=512;

    gl_fbo_t *fbo = gl_fbo_create(glc, width, height);
    gl_fbo_bind(fbo);

    glClearColor(1.0, 1.0, 1.0, 1.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(-10., 10., -10., 10., 1., 2000.);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    glTranslatef(0, 0, -20);

    glBegin(GL_QUADS);
    glColor3f(1., 0., 0.); glVertex3f(-.75, -.75, 0.);
    glColor3f(0., 1., 0.); glVertex3f( .75, -.75, 0.);
    glColor3f(0., 0., 1.); glVertex3f( .75,  .75, 0.);
    glColor3f(1., 1., 0.); glVertex3f(-.75,  .75, 0.);
    glEnd();

    uint8_t *buf = (uint8_t*) malloc(width*height*3);
    glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, buf);

    printf("glError %d\n", glGetError());

    FILE *f = fopen("out.pnm", "wb");
    fprintf(f, "P6\n%d %d\n255\n", width, height);
    fwrite(buf, 3, width*height, f);
    fclose(f);

    return 0;
}
