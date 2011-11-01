GCC = gcc -Wall -g -std=gnu99 -D_REENTRANT -fPIC -shared -D_FILE_OFFSET_BITS=64 -D_LARGEFILE_SOURCE -Wno-unused-parameter -Wno-format-zero-length

JNI_INCLUDES = -I/usr/lib/jvm/java-6-sun/include/ -I/usr/lib/jvm/java-6-sun/include/linux -I/usr/lib/jvm/java-6-openjdk/include/

all:    gltest libjgl.so

#############################################################
LIBJGL_OBJS = glcontext.o glcontext-x11.o gl_jni.o lphash.o parray.o
libjgl.so: april_vis_GL.h $(LIBJGL_OBJS)
	ld --shared $(LIBJGL_OBJS) -o libjgl.so -lX11 -lGL

#############################################################
GLTEST_OBJS = glcontext.o glcontext-x11.o gltest.o libjgl.so
gltest: $(GLTEST_OBJS)
	$(GCC) -o $@ $(GLTEST_OBJS) -lX11 -lGL

#############################################################
%.o: %.c
	$(GCC) -shared -O2 -c -fPIC -fno-omit-frame-pointer -fno-stack-protector -D_REENTRANT $< $(JNI_INCLUDES)

april_vis_GL.h: ../src/april/vis/GL.java
	echo "Rebuilding JNI headers. Ensure java file has been recently built."
	javah -classpath ../april.jar  april.vis.GL

clean:
	rm -rf *.o *~ gltest *.so
